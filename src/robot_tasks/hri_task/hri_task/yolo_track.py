#!/usr/bin/env python3
import sys, os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2

import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import time
import json
from erasers_g1_api.tts import TTS

# ============================================================
# 設定パラメータ
# ============================================================
# YOLO 追跡用
YOLO_SEARCH_SEC = 5.0  # 探索の蓄積時間 (秒)
YOLO_MAX_DIST = 1.3    # 最大追跡距離 (m)

# カメラとLiDARの位置ズレ補正（LiDARから見てカメラが前方に13cm）
CAMERA_OFFSET_X = 0.13
CAMERA_OFFSET_Y = 0.0

# 追跡
TRACKING_GATE = 0.5    # 追跡ゲート距離 (m) (他人に乗り移らないように厳しく)
MAX_LOST_FRAMES = 15   # 連続ロストフレーム上限

# LiDAR 障害物回避 (クラスタリングは行わない)
# ※G1はMid360が逆向き(丸い部分が下)についているため、Z軸は下向き正、Y軸は右向き正となる
LIDAR_TOPIC = '/utlidar/cloud_livox_mid360'
LIDAR_Z_MIN = -0.5     # 高さフィルタ下限 (センサーより上50cm〜)
LIDAR_Z_MAX = 1.05     # 高さフィルタ上限 (床1.2mとした場合、床面を除外)
LIDAR_X_MIN = 0.1      # 前方フィルタ下限 (m)
LIDAR_X_MAX = 1.5      # 前方フィルタ上限 (m)
LIDAR_CENTER_Y = 0.3   # 中央エリア半幅 (m)
LIDAR_SIDE_Y_MAX = 1.0 # 左右エリア外側 (m)
OBS_COUNT_THRESHOLD = 20  # 障害物判定の点群数閾値
AVOID_TURN = 0.6       # 回避旋回速度 (rad/s)

# 速度制御
FOLLOW_DISTANCE = 0.8  # 維持する距離 (m)
LINEAR_GAIN = 0.25
ANGULAR_GAIN = 1.2
LINEAR_MAX = 0.4
ANGULAR_MAX = 1.2
MIN_RANGE = 0.25

# ============================================================
# EKF（等速直線運動モデル）
# ============================================================
class EKF:
    def __init__(self):
        self.x = np.zeros(4)  # x, y, vx, vy
        self.P = np.eye(4)
        self.dt = 0.1
        self.Q = np.eye(4) * 0.02  # プロセスノイズ
        self.R = np.eye(2) * 0.3   # 観測ノイズ

    def predict(self):
        F = np.array([
            [1, 0, self.dt, 0],
            [0, 1, 0, self.dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q

    def update(self, z):
        H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
        y = z - H @ self.x
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ H) @ self.P

    def mahalanobis(self, z):
        H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
        y = z - H @ self.x
        S = H @ self.P @ H.T + self.R
        return y.T @ np.linalg.inv(S) @ y

# ============================================================
# 速度指令計算
# ============================================================
# ============================================================

# ============================================================
# ROS2 Node
# ============================================================
class YoloHumanTracker(Node):

    def __init__(self):
        super().__init__('yolo_human_tracker')

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # YOLO (メイン追跡用)
        self.yolo_sub = self.create_subscription(
            String, '/yolo_human/result', self.yolo_callback, 10)
        self.yolo_cmd_pub = self.create_publisher(
            String, '/yolo_human/command', 10)

        # LiDAR (障害物回避専用)
        self.sub_pointcloud = self.create_subscription(
            PointCloud2,
            '/yolo_human/pointcloud',
            self.lidar_callback,
            10
        )

        self.avoid_lateral = 0.0
        self.emergency_brake = False

        # TTS
        self.tts = TTS(self)
        self.last_cant_speak = 0.0

        # 状態
        self.ekf = EKF()
        self.state = 'SEARCHING'  # SEARCHING, TRACKING, LOST, FAILED
        self.search_start_time = time.time()
        self.lost_time_start = None
        self.reached_start_time = None
        self.latest_cmd = Twist()
        self.target_track_id = None
        
        # 空間の制限情報（LiDARから取得）
        self.min_left_y = 1.0
        self.max_right_y = -1.0
        self.min_front_x = 1.5
        self.last_step_time = time.time()

        self.get_logger().info("5秒間待機し、一番近い人を探索します (YOLOメイン)")

        # cmd_vel 定期送信 (20Hz)
        self.cmd_timer = self.create_timer(0.05, self.publish_cmd)

        # 初回だけ少し待ってYOLOを起動
        self.start_timer = self.create_timer(1.0, self.send_yolo_start_cmd)

    def compute_twist(self, pos):
        x, y = pos
        dist = np.linalg.norm(pos)
        angle = np.arctan2(y, x)

        error = dist - FOLLOW_DISTANCE

        # --- 前進速度（距離に応じてなめらかに減速） ---
        if abs(error) < 0.15:
            linear = 0.0
        elif error > 0:
            # 0.8m の誤差（距離1.6m）でMAXスピードになるようにキビキビ動かす
            speed_factor = min(error / 0.8, 1.0)
            linear = LINEAR_MAX * speed_factor
        else:
            # 近すぎる場合はゆっくり後退
            linear = -LINEAR_MAX * min(abs(error) * 0.5, 1.0)

        # --- 旋回速度 ---
        angular = ANGULAR_GAIN * angle

        linear = np.clip(linear, -LINEAR_MAX, LINEAR_MAX)
        angular = np.clip(angular, -ANGULAR_MAX, ANGULAR_MAX)

        if dist < MIN_RANGE + 0.2:  # 近すぎる場合
            linear = min(linear, 0.0)

        return linear, angular



    def publish_cmd(self):
        if self.state == 'TRACKING':
            self.pub.publish(self.latest_cmd)
        else:
            self.pub.publish(Twist())

    def send_yolo_start_cmd(self):
        msg = String()
        msg.data = json.dumps({"command": "start", "classes": ["person"]})
        self.yolo_cmd_pub.publish(msg)
        self.start_timer.cancel()
        self.get_logger().info("YOLO起動しました")

    # ------- LiDAR コールバック (純粋な障害物回避のみ) -------
    def lidar_callback(self, msg):
        min_left_y = 1.0
        max_right_y = -1.0
        min_front_x = 1.5
        
        px, py = self.ekf.x[:2]
        is_tracking = (self.state == 'TRACKING')

        for p in pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z")):
            x, y_raw, z_raw = p[0], p[1], p[2]

            # 逆向きマウント補正（X=前、Y=右、Z=下 となるため、Yを反転させてROSの左=正に合わせる）
            y = -y_raw
            z = z_raw

            # ターゲットの除外判定（追跡中の人を障害物として認識しないようにする）
            if is_tracking:
                # EKFの座標はカメラ基準なので、LiDAR基準の点群と比較するためにオフセットを足す
                px_lidar = px + CAMERA_OFFSET_X
                py_lidar = py + CAMERA_OFFSET_Y
                dist_to_person = np.hypot(x - px_lidar, y - py_lidar)
                # ターゲットの予測位置から半径60cm以内の点群は「ターゲット自身」とみなして障害物から除外
                if dist_to_person < 0.6:
                    continue

            # zは下向きが正。z=1.2mあたりが床面になるため、1.05m等でカットして床を障害物と誤認しないようにする
            if LIDAR_Z_MIN <= z <= LIDAR_Z_MAX:
                if 0.1 < x < 1.0: # 1.0m前方までの障害物をチェック
                    if 0.0 <= y <= 1.0:
                        min_left_y = min(min_left_y, y)
                    elif -1.0 <= y < 0.0:
                        max_right_y = max(max_right_y, y)
                    
                    if abs(y) <= 0.35:
                        min_front_x = min(min_front_x, x)

        # ----------------------------------------------------
        # 回避ベクトルの計算（左右の空きスペースを均等に通るロジック）
        # ロボット横幅53cmなので、隙間が0.6m以上あれば真ん中を通れる
        # ----------------------------------------------------
        gap_width = min_left_y - max_right_y
        gap_center = (min_left_y + max_right_y) / 2.0
        
        self.emergency_brake = False
        self.avoid_lateral = 0.0

        if min_front_x < 0.8:
            # 正面に障害物がある場合
            if gap_width < 0.6:
                # 隙間が60cm未満なら通れないので強制ストップ
                self.emergency_brake = True
                self.get_logger().warn(f"隙間が狭すぎます！(幅:{gap_width:.2f}m) 強制ストップ")
            else:
                # 通れる隙間があるなら、隙間の中心(gap_center)に向かってカニ歩き
                if gap_center > 0.05:
                    self.avoid_lateral = 0.25  # 左へ
                elif gap_center < -0.05:
                    self.avoid_lateral = -0.25 # 右へ
        else:
            # 正面にすぐぶつかる物はないが、横が近すぎる場合は少し真ん中に寄る
            if min_left_y < 0.35 or max_right_y > -0.35:
                if gap_center > 0.05:
                    self.avoid_lateral = 0.15
                elif gap_center < -0.05:
                    self.avoid_lateral = -0.15
                    
        # 空間の状態を保存（追跡ロジックの3軸制御で使用）
        self.min_left_y = min_left_y
        self.max_right_y = max_right_y
        self.min_front_x = min_front_x

    # ------- YOLO コールバック (メイン追跡) -------
    def yolo_callback(self, msg):
        now = time.time()
        self.ekf.dt = now - self.last_step_time
        self.last_step_time = now

        clusters = []
        try:
            detections = json.loads(msg.data)
            for det in detections:
                if det.get('label') != 'person':
                    continue
                
                dz = det.get('distance_z', 0.0)
                # YOLOの角度（右が正）をROS座標系（左が正）に合わせて反転する
                angle_ros = -det.get('angle_rad', 0.0)
                
                # 距離が0.0に落ちたノイズの場合、EKFの予測距離を使って位置を補完する
                if dz <= 0.01:
                    if self.state == 'TRACKING':
                        pred_dist = np.linalg.norm(self.ekf.x[:2])
                        if pred_dist <= YOLO_MAX_DIST:
                            dz = pred_dist
                        else:
                            continue # 遠すぎる場合は無視
                    else:
                        continue # TRACKING中でない場合は予測距離がないため無視
                
                # 最初の探索時は1.3m以内の人だけを対象にする
                if self.state == 'SEARCHING' and dz > YOLO_MAX_DIST:
                    continue
                # 一度ロックオンして追跡中の場合は、少し離れても(2.0mまで)許容する
                if self.state == 'TRACKING' and dz > YOLO_MAX_DIST + 0.7:
                    continue
                    
                # xy座標に変換
                x = dz * np.cos(angle_ros)
                y = dz * np.sin(angle_ros)
                tid = det.get('track_id', -1)
                clusters.append((np.array([x, y]), tid))
        except json.JSONDecodeError:
            pass

        # 状態遷移
        if self.state == 'SEARCHING':
            self._handle_searching(clusters)
        elif self.state == 'TRACKING':
            self._handle_tracking(clusters)
        elif self.state == 'LOST':
            self._handle_lost(clusters)

    def _handle_searching(self, clusters):
        elapsed = time.time() - self.search_start_time
        if elapsed < YOLO_SEARCH_SEC:
            return  # 蓄積待ち

        if len(clusters) == 0:
            self.get_logger().warn("YOLO人検出0件。探索を継続します。")
            self.search_start_time = time.time()
            return

        # 一番近い人を探す
        best_cluster = None
        best_tid = -1
        min_dist = float('inf')

        for pos, tid in clusters:
            dist = np.linalg.norm(pos)
            if dist < min_dist:
                min_dist = dist
                best_cluster = pos
                best_tid = tid

        if best_cluster is not None:
            # EKF初期化
            self.ekf.x[:2] = best_cluster
            self.ekf.x[2:] = 0.0
            self.ekf.P = np.diag([0.2, 0.2, 1.0, 1.0])
            self.state = 'TRACKING'
            self.target_track_id = best_tid if best_tid >= 0 else None
            self.get_logger().info(f"ターゲット決定! [{best_cluster[0]:.2f}, {best_cluster[1]:.2f}] track_id={best_tid}")

    def _handle_tracking(self, clusters):
        self.ekf.predict()

        if len(clusters) == 0:
            self.lost_frames += 1
            self.get_logger().warn(f"YOLO検出0件 lost={self.lost_frames} (予測軌道で補完中)")
            # 見失っていてもEKFの予測軌道に従って動かす
            lin, ang = self.compute_twist(self.ekf.x[:2])
            self._apply_cmd(lin, ang, matched=False)
            self._check_lost()
            return

        # --- track_id ベースのマッチング ---
        best_cluster = None
        matched_tid = None

        # 優先度 1: track_id が一致する人を探す
        if self.target_track_id is not None:
            for pos, tid in clusters:
                if tid == self.target_track_id:
                    best_cluster = pos
                    matched_tid = tid
                    break

        # 優先度 2: track_idが見つからない場合、EKF予測位置に最も近い人を使う
        if best_cluster is None:
            pred_pos = self.ekf.x[:2]
            dists = [np.linalg.norm(pos - pred_pos) for pos, _ in clusters]
            best_idx = int(np.argmin(dists))
            best_dist = dists[best_idx]

            if best_dist > TRACKING_GATE:
                self.lost_frames += 1
                self.get_logger().warn(
                    f"最近の人が遠い (d={best_dist:.2f}m > gate={TRACKING_GATE}m) lost={self.lost_frames}")
                lin, ang = self.compute_twist(self.ekf.x[:2])
                self._apply_cmd(lin, ang, matched=False)
                self._check_lost()
                return

            best_cluster = clusters[best_idx][0]
            matched_tid = clusters[best_idx][1]
            # track_id を更新（リアサインされた場合）
            if matched_tid >= 0:
                self.target_track_id = matched_tid

        # 更新
        self.ekf.update(best_cluster)
        self.lost_frames = 0

        # 速度指令
        lin, ang = self.compute_twist(self.ekf.x[:2])
        self._apply_cmd(lin, ang, matched=True)

    def _apply_cmd(self, lin, ang, matched=True):
        x, y = self.ekf.x[:2]
        dist = np.hypot(x, y)
        is_close = dist < (FOLLOW_DISTANCE + 0.2)  # 約1.0m
        is_sharp_turn = abs(ang) > 0.8  # 約45度以上

        # ====================================================
        # 独立3軸制御ロジック (Axis 1: 回転, Axis 2: 前進, Axis 3: 横移動)
        # ====================================================

        # --- Axis 1: 回転 (Rotation) ---
        # 空間の制限を無視し、いかなる時も人の方を向く（Look At）
        final_ang = float(np.clip(ang, -ANGULAR_MAX, ANGULAR_MAX))

        # --- Axis 2: 前進 (Forward) ---
        clearance_violated = (self.min_left_y < 0.3) or (self.max_right_y > -0.3)
        
        # 真正面に壁がある場合、または通路が狭すぎる場合は前進をゼロにする（後退は許可）
        # ※クリアランス違反(左右30cm未満)で前進を止めると遠くても動けなくなるため、前進は止めずに横移動で補正する
        if (self.min_front_x < 0.75 or self.emergency_brake) and lin > 0:
            final_lin = 0.0
            if matched:
                if self.emergency_brake:
                    self.get_logger().warn("通路幅が60cm未満のため前進停止")
                else:
                    self.get_logger().warn("前方に障害物！前進停止")
        else:
            if is_sharp_turn:
                final_lin = 0.0 # 急旋回時は前進停止
            else:
                final_lin = float(lin)

        # --- Axis 3: 横移動・カニ歩き (Lateral) ---
        if is_sharp_turn or is_close:
            track_lat = 0.0  # 近い時や急旋回時は人への横追従は殺す
        else:
            track_lat = np.clip(y * 0.6, -0.3, 0.3) # 人に合わせる横移動
            
        gap_width = self.min_left_y - self.max_right_y

        # 空間の制限によるカニ歩きの役割切替
        if self.min_front_x < 0.75 or gap_width < 1.2 or clearance_violated:
            # 狭所・行き止まり・クリアランス不足：人への横移動を捨て、障害物回避（空間の中心）の横移動を最優先
            # hri_task.zip の avoid_lateral は符号がそのまま使える（正=左、負=右）
            final_lat_y = float(self.avoid_lateral)
        else:
            # 広い場所：人に合わせる横移動を使用
            final_lat_y = float(track_lat)

        # 【絶対条件】横歩きの際の最終障害物確認（いかなる時もぶつからない）
        if final_lat_y > 0.0 and self.min_left_y < 0.25:
            final_lat_y = 0.0 # 左が壁なので左移動ブロック
        if final_lat_y < 0.0 and self.max_right_y > -0.25:
            final_lat_y = 0.0 # 右が壁なので右移動ブロック

        # ====================================================
        # 到着判定 (REACHED) ロジック
        # ====================================================
        dist = np.hypot(x, y)
        # 目標距離(0.8m)付近で、前進と横移動がほぼゼロ（安定）になっているか
        if abs(dist - FOLLOW_DISTANCE) < 0.2 and abs(final_lin) < 0.05 and abs(final_lat_y) < 0.05:
            if self.reached_start_time is None:
                self.reached_start_time = time.time()
            elif time.time() - self.reached_start_time > 2.0:
                # 2秒安定したら到着完了とする
                self.state = 'REACHED'
                self.get_logger().info("【追跡完了】目標地点に到達し安定しました。")
                self.latest_cmd = Twist()
                self.pub.publish(self.latest_cmd)
                return
        else:
            self.reached_start_time = None

        # コマンドのパブリッシュ
        self.latest_cmd = Twist()
        self.latest_cmd.linear.x = float(final_lin)
        self.latest_cmd.linear.y = float(final_lat_y)
        self.latest_cmd.angular.z = float(final_ang)

        if matched:
            self.get_logger().info(
                f"Track: lin={final_lin:.2f}, ang={final_ang:.2f}, lat_y={final_lat_y:.2f} | avoid={self.avoid_lateral:.2f}"
            )

    def _check_lost(self):
        if self.lost_frames > MAX_LOST_FRAMES:
            self.state = 'LOST'
            self.lost_time_start = time.time()
            self.last_known_pos = self.ekf.x[:2].copy()
            self.get_logger().warn("完全にロストしました。再探索フェーズに移行します。")
            self.latest_cmd = Twist()
            self.pub.publish(self.latest_cmd)

    def _handle_lost(self, clusters):
        lost_time = time.time() - self.lost_time_start
        if lost_time > 10.0:
            # 追跡終了ロジックの漏れ修正: 完全にロストした場合はFAILEDで固まらず、再度探索に戻る
            self.state = 'SEARCHING'
            self.search_start_time = time.time()
            self.get_logger().error("復帰タイムアウト。追従をリセットし、再探索を開始します。")
            self.latest_cmd = Twist()
            self.pub.publish(self.latest_cmd)
            return

        if len(clusters) == 0:
            return
            
        for pos, tid in clusters:
            if self.target_track_id is not None and tid == self.target_track_id:
                self.ekf.x[:2] = pos
                self.ekf.x[2:] = 0.0
                self.ekf.P = np.diag([0.2, 0.2, 1.0, 1.0])
                self.state = 'TRACKING'
                self.lost_frames = 0
                self.get_logger().info(f"ロストから復帰しました！ (ID一致)")
                return
                
        # 見つからなかった場合は予測位置に近い人を採用
        for pos, tid in clusters:
            if np.linalg.norm(pos - self.last_known_pos) < TRACKING_GATE:
                self.ekf.x[:2] = pos
                self.ekf.x[2:] = 0.0
                self.ekf.P = np.diag([0.2, 0.2, 1.0, 1.0])
                self.state = 'TRACKING'
                self.lost_frames = 0
                if tid >= 0:
                    self.target_track_id = tid
                self.get_logger().info(f"ロストから復帰しました！ (予測位置一致)")
                return

def main(args=None):
    rclpy.init(args=args)
    node = YoloHumanTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        msg = String()
        msg.data = json.dumps({"command": "stop"})
        node.yolo_cmd_pub.publish(msg)
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
