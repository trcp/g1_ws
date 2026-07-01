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
MAHA_GATE = 5.99       # マハラノビス距離閾値 (χ²分布 自由度2, 95%点)
TENTATIVE_ID_CONFIRM = 3  # 仮マッチIDの確定に必要な連続フレーム数

# 到着判定
REACHED_DISTANCE_TOLERANCE = 0.12  # 距離トレランス ±12cm (旧: ±20cm)
REACHED_STABLE_SEC = 3.5           # 安定維持時間 (旧: 2.0s)
REACHED_MIN_TRACKING_SEC = 5.0     # 追跡開始後この時間はREACHED判定を禁止

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

# Follow-me safety gates. Obstacles are considered only near the robot and in
# the tube between the robot and the followed person.
PATH_TUBE_HALF_WIDTH = 0.65
PATH_FRONT_HALF_WIDTH = 0.32
PATH_MAX_LOOKAHEAD = 1.5
NEAR_FRONT_HALF_WIDTH = 0.32
FRONT_SLOW_DISTANCE = 0.70
FRONT_STOP_DISTANCE = 0.45
DOORWAY_WIDTH_THRESHOLD = 0.90
WIDE_SPACE_WIDTH_THRESHOLD = 1.10
TARGET_POINT_EXCLUSION_RADIUS = 0.60
LOST_SHORT_SEC = 0.5
LOST_STOP_SEC = 1.5

# 速度制御
FOLLOW_DISTANCE = 1.2  # 維持する距離 (m)
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
            '/utlidar/cloud_livox_mid360',
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
        self.tracking_start_time = None  # 追跡開始時刻（REACHED判定の最低時間用）
        self.latest_cmd = Twist()
        self.target_track_id = None
        self.lost_frames = 0
        self.yolo_lost_start_time = None
        self.tracking_match_source = 'none'
        self.latest_lidar_person_pos = None
        self.latest_lidar_person_time = 0.0
        
        # 複数人対応: ID乗り移り防止
        self._tentative_tid = None
        self._tentative_tid_count = 0
        
        # 複数人対応: 外見一貫性チェック
        self.target_bbox_width = 0.0
        
        # 空間の制限情報（LiDARから取得）
        self.min_left_y = 1.0
        self.max_right_y = -1.0
        self.min_front_x = 1.5
        self.near_front_distance = float('inf')
        self.path_front_distance = float('inf')
        self.corridor_width = float('inf')
        self.corridor_center_error = 0.0
        self.has_left_wall = False
        self.has_right_wall = False
        self.path_obstacle_side = 0.0
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
        if abs(error) < 0.08:  # デッドバンド縮小 (旧: 0.15)
            linear = 0.0
        elif error > 0:
            # 0.8m の誤差（距離1.6m）でMAXスピードになるようにキビキビ動かす
            speed_factor = min(error / 0.8, 1.0)
            linear = LINEAR_MAX * speed_factor
        else:
            # 近すぎる場合はゆっくり後退
            linear = -LINEAR_MAX * min(abs(error) * 0.5, 1.0)

        # --- 旋回速度 ---
        # 距離が近いほど旋回ゲインを上げ、視野から人が外れる前に体を向ける。
        if dist < 0.9:
            dynamic_angular_gain = ANGULAR_GAIN * 1.8
        elif dist < 1.3:
            dynamic_angular_gain = ANGULAR_GAIN * 1.4
        else:
            dynamic_angular_gain = ANGULAR_GAIN
        angular = dynamic_angular_gain * angle

        # 人が横に大きくずれている時は、前進より旋回を優先する。
        if abs(angle) > 0.7:
            linear = 0.0
        elif abs(angle) > 0.4:
            linear *= 0.4

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
        near_front = []
        path_front = []
        left_sides = []
        right_sides = []
        obstacle_sides = []

        px, py = self.ekf.x[:2]
        is_tracking = (self.state == 'TRACKING')
        person_points = []
        target_dist = float(np.hypot(px, py))
        if target_dist > 0.05:
            target_dir = np.array([px, py]) / target_dist
        else:
            target_dir = np.array([1.0, 0.0])
        lookahead = min(max(target_dist, 0.5), PATH_MAX_LOOKAHEAD)

        for p in pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z")):
            x, y_raw, z_raw = p[0], p[1], p[2]

            # Mid360は丸い面が下向き。X=前、Y_raw=右、Z=下なので、YだけROSの左=正へ反転する。
            y = -y_raw
            z = z_raw

            if x == 0.0 and y == 0.0 and z == 0.0:
                continue
            if np.hypot(x, y) < 0.20:
                continue
            if not (LIDAR_Z_MIN <= z <= LIDAR_Z_MAX):
                continue

            # ターゲットの除外判定（追跡中の人を障害物として認識しないようにする）
            if is_tracking:
                target_lidar_x = px + CAMERA_OFFSET_X
                target_lidar_y = py + CAMERA_OFFSET_Y
                if np.hypot(x - target_lidar_x, y - target_lidar_y) < TARGET_POINT_EXCLUSION_RADIUS:
                    person_points.append((x, y))
                    continue

            if 0.20 < x < FRONT_SLOW_DISTANCE and abs(y) < NEAR_FRONT_HALF_WIDTH:
                near_front.append(x)

            point = np.array([x, y])
            forward = float(np.dot(point, target_dir))
            side = float(-x * target_dir[1] + y * target_dir[0])

            if 0.20 < forward < lookahead and abs(side) < PATH_TUBE_HALF_WIDTH:
                if abs(side) < PATH_FRONT_HALF_WIDTH:
                    path_front.append(forward)
                    obstacle_sides.append(side)
                elif side > 0.0:
                    left_sides.append(side)
                else:
                    right_sides.append(side)

        # ターゲット（人）のLiDAR点群重心を計算してキャッシュ
        if is_tracking and len(person_points) >= 10:
            xs = [pt[0] for pt in person_points]
            ys = [pt[1] for pt in person_points]
            mean_x_lidar = float(np.mean(xs))
            mean_y_lidar = float(np.mean(ys))
            mean_x_cam = mean_x_lidar - CAMERA_OFFSET_X
            mean_y_cam = mean_y_lidar - CAMERA_OFFSET_Y
            self.latest_lidar_person_pos = np.array([mean_x_cam, mean_y_cam])
            self.latest_lidar_person_time = time.time()
        else:
            self.latest_lidar_person_pos = None

        def percentile(values, pct, default):
            if not values:
                return default
            return float(np.percentile(np.asarray(values), pct))

        near_front_distance = percentile(near_front, 10, float('inf'))
        path_front_distance = percentile(path_front, 10, float('inf'))
        left_clearance = percentile(left_sides, 10, float('inf'))
        right_clearance = percentile(right_sides, 90, float('-inf'))

        has_left_wall = len(left_sides) >= 8
        has_right_wall = len(right_sides) >= 8
        if has_left_wall and has_right_wall:
            corridor_width = left_clearance - right_clearance
            corridor_center_error = (left_clearance + right_clearance) / 2.0
        else:
            corridor_width = float('inf')
            corridor_center_error = 0.0

        self.near_front_distance = near_front_distance
        self.path_front_distance = path_front_distance
        self.min_front_x = min(near_front_distance, path_front_distance)
        self.has_left_wall = has_left_wall
        self.has_right_wall = has_right_wall
        self.corridor_width = corridor_width
        self.corridor_center_error = corridor_center_error
        self.path_obstacle_side = float(np.median(obstacle_sides)) if obstacle_sides else 0.0
        self.min_left_y = left_clearance if has_left_wall else 1.0
        self.max_right_y = right_clearance if has_right_wall else -1.0

        self.emergency_brake = self.min_front_x < FRONT_STOP_DISTANCE
        self.avoid_lateral = 0.0

    # ------- YOLO コールバック (メイン追跡) -------
    def yolo_callback(self, msg):
        now = time.time()
        self.ekf.dt = now - self.last_step_time
        self.last_step_time = now

        clusters = []
        detections = []
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
                bwr = det.get('bbox_width_ratio', 0.0)
                clusters.append((np.array([x, y]), tid, bwr))
        except json.JSONDecodeError:
            pass

        # 状態遷移
        if self.state == 'SEARCHING':
            self._handle_searching(clusters, detections=detections)
        elif self.state == 'TRACKING':
            self._handle_tracking(clusters)
        elif self.state == 'LOST':
            self._handle_lost(clusters)

    def _handle_searching(self, clusters, detections=None):
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

        for pos, tid, bwr in clusters:
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
            self.tracking_start_time = time.time()  # 追跡開始時刻を記録
            self.target_track_id = best_tid if best_tid >= 0 else None
            self._tentative_tid = None
            self._tentative_tid_count = 0
            # bbox_width_ratio を記録（外見一貫性チェック用）
            # best_cluster に対応するdetectionから取得
            for det in detections:
                if det.get('label') != 'person':
                    continue
                tid = det.get('track_id', -1)
                if tid == best_tid or (best_tid < 0):
                    self.target_bbox_width = det.get('bbox_width_ratio', 0.0)
                    break
            self.get_logger().info(f"ターゲット決定! [{best_cluster[0]:.2f}, {best_cluster[1]:.2f}] track_id={best_tid} bwr={self.target_bbox_width:.3f}")

    def _handle_tracking(self, clusters):
        self.ekf.predict()

        # LiDARによる観測補完用データの取得
        lidar_pos = None
        if hasattr(self, 'latest_lidar_person_pos') and self.latest_lidar_person_pos is not None:
            if time.time() - self.latest_lidar_person_time < 0.5:
                lidar_pos = self.latest_lidar_person_pos

        if len(clusters) == 0:
            if self.yolo_lost_start_time is None:
                self.yolo_lost_start_time = time.time()
            lost_elapsed = time.time() - self.yolo_lost_start_time
            if lidar_pos is not None:
                self.ekf.update(lidar_pos)
                self.lost_frames = 0
                self.get_logger().info(f"YOLO検出0件ですが、LiDARで追従中: [{lidar_pos[0]:.2f}, {lidar_pos[1]:.2f}]")
                lin, ang = self.compute_twist(self.ekf.x[:2])
                self._apply_cmd(lin, ang, matched=True, source='lidar', lost_elapsed=lost_elapsed)
            else:
                self.lost_frames += 1
                self.get_logger().warn(f"YOLO検出0件 lost={self.lost_frames} (予測軌道で補完中)")
                # 見失っていてもEKFの予測軌道に従って動かす
                lin, ang = self.compute_twist(self.ekf.x[:2])
                self._apply_cmd(lin, ang, matched=False, source='prediction', lost_elapsed=lost_elapsed)
                self._check_lost()
            return
        else:
            self.yolo_lost_start_time = None

        # --- track_id ベースのマッチング ---
        best_cluster = None
        matched_tid = None

        # 優先度 1: track_id が一致する人を探す
        if self.target_track_id is not None:
            for pos, tid, bwr in clusters:
                if tid == self.target_track_id:
                    best_cluster = pos
                    matched_tid = tid
                    break

        # 優先度 2: track_idが見つからない場合、マハラノビス距離＋外見スコアで最適な人を選ぶ
        if best_cluster is None:
            pred_pos = self.ekf.x[:2]
            best_idx = None
            best_score = float('inf')

            for i, (pos, tid, bwr) in enumerate(clusters):
                # マハラノビス距離（EKFの不確実性を考慮した統計的距離）
                maha = self.ekf.mahalanobis(pos)
                score = maha
                
                # 外見一貫性: bbox_width_ratio が大きくずれている場合はペナルティ
                if self.target_bbox_width > 0.01:
                    bwr_diff = abs(bwr - self.target_bbox_width)
                    if bwr_diff > 0.15:
                        score *= 3.0  # 体格が大幅に異なる → 別人の可能性
                    elif bwr_diff > 0.08:
                        score *= 1.5  # やや異なる
                
                if score < best_score:
                    best_score = score
                    best_idx = i

            # マハラノビス距離の閾値チェック（χ²分布 自由度2, 95%点）
            if best_idx is None or best_score > MAHA_GATE:
                # ユークリッド距離でもフォールバック確認
                euclid_dists = [np.linalg.norm(pos - pred_pos) for pos, _, _ in clusters]
                euclid_best = int(np.argmin(euclid_dists))
                if euclid_dists[euclid_best] > TRACKING_GATE:
                    if lidar_pos is not None:
                        self.ekf.update(lidar_pos)
                        self.lost_frames = 0
                        self.get_logger().info(f"YOLO近傍検出なしですが、LiDARで追従中: [{lidar_pos[0]:.2f}, {lidar_pos[1]:.2f}]")
                        lin, ang = self.compute_twist(self.ekf.x[:2])
                        self._apply_cmd(lin, ang, matched=True, source='lidar')
                    else:
                        self.lost_frames += 1
                        self.get_logger().warn(
                            f"マハラノビス/ユークリッド両方で近傍なし (maha={best_score:.2f}, euclid={euclid_dists[euclid_best]:.2f}) lost={self.lost_frames}")
                        lin, ang = self.compute_twist(self.ekf.x[:2])
                        self._apply_cmd(lin, ang, matched=False, source='prediction')
                        self._check_lost()
                    return
                # ユークリッドでは通るがマハラノビスが高い → 注意付きで採用
                best_idx = euclid_best
                self.get_logger().info(f"マハラノビス距離が高い({best_score:.2f})がユークリッド({euclid_dists[euclid_best]:.2f}m)で採用")

            best_cluster = clusters[best_idx][0]
            matched_tid = clusters[best_idx][1]
            # track_id の「乗り移り防止」: 仮マッチを連続フレーム確認してから確定
            if matched_tid >= 0:
                if matched_tid == self._tentative_tid:
                    self._tentative_tid_count += 1
                else:
                    self._tentative_tid = matched_tid
                    self._tentative_tid_count = 1
                
                if self._tentative_tid_count >= TENTATIVE_ID_CONFIRM:
                    if self.target_track_id != matched_tid:
                        self.get_logger().info(f"Track ID更新: {self.target_track_id} → {matched_tid} (confirmed)")
                    self.target_track_id = matched_tid
            # bbox_width_ratio を移動平均的に更新
            bwr_new = clusters[best_idx][2]
            if bwr_new > 0.01:
                self.target_bbox_width = 0.8 * self.target_bbox_width + 0.2 * bwr_new

        # 更新
        self.ekf.update(best_cluster)
        self.lost_frames = 0

        # 速度指令
        lin, ang = self.compute_twist(self.ekf.x[:2])
        self._apply_cmd(lin, ang, matched=True, source='yolo')

    def _apply_cmd(self, lin, ang, matched=True, source='yolo', lost_elapsed=0.0):
        x, y = self.ekf.x[:2]
        dist = np.hypot(x, y)
        is_sharp_turn = abs(ang) > 0.8
        front_distance = min(self.near_front_distance, self.path_front_distance)
        is_near_stop = front_distance < FRONT_STOP_DISTANCE
        is_doorway = (
            self.has_left_wall
            and self.has_right_wall
            and self.corridor_width < DOORWAY_WIDTH_THRESHOLD
        )
        is_path_obstacle = (
            self.path_front_distance < FRONT_SLOW_DISTANCE
            and not is_doorway
        )

        # ====================================================
        # 独立3軸制御ロジック (Axis 1: 回転, Axis 2: 前進, Axis 3: 横移動)
        # ====================================================
        mode = 'FOLLOW_PERSON'
        final_ang = float(np.clip(ang, -ANGULAR_MAX, ANGULAR_MAX))
        final_lin = float(lin)
        final_lat_y = 0.0

        if source != 'yolo':
            mode = 'LOST_SEARCH'
            if lost_elapsed < LOST_SHORT_SEC and source == 'lidar':
                final_lin *= 0.35
            else:
                final_lin = 0.0
            if lost_elapsed > LOST_STOP_SEC:
                final_lin = 0.0
                final_ang = 0.0

        elif is_near_stop:
            mode = 'NEAR_STOP'
            final_lin = 0.0
            final_lat_y = 0.0

        elif is_doorway:
            mode = 'DOORWAY_CENTERING'
            # 狭所では横移動せず、低速前進と軽い旋回で中央を通る。
            center_gain = 0.8
            final_ang = float(np.clip((ang * 0.45) + (self.corridor_center_error * center_gain), -0.45, 0.45))
            if final_lin > 0.0:
                final_lin = min(final_lin, 0.15)
            final_lat_y = 0.0

        elif is_path_obstacle:
            mode = 'PATH_OBSTACLE_AVOID'
            if final_lin > 0.0:
                final_lin *= 0.45

            # 広い場所だけ弱く横移動を使う。障害物が左なら右へ、右なら左へ。
            can_lateral = (
                self.corridor_width > WIDE_SPACE_WIDTH_THRESHOLD
                and front_distance > FRONT_STOP_DISTANCE
                and abs(final_ang) < 0.6
                and matched
            )
            if can_lateral:
                if self.path_obstacle_side > 0.05:
                    final_lat_y = -0.15
                elif self.path_obstacle_side < -0.05:
                    final_lat_y = 0.15
                else:
                    left_open = self.min_left_y > abs(self.max_right_y)
                    final_lat_y = 0.15 if left_open else -0.15

            # 正面障害物に対しては少しだけ空いている方向へ向ける。
            if self.path_obstacle_side > 0.05:
                final_ang -= 0.25
            elif self.path_obstacle_side < -0.05:
                final_ang += 0.25
            final_ang = float(np.clip(final_ang, -ANGULAR_MAX, ANGULAR_MAX))

        else:
            mode = 'FOLLOW_PERSON'
            if front_distance < FRONT_SLOW_DISTANCE and final_lin > 0.0:
                final_lin *= 0.4

        if is_sharp_turn and final_lin > 0.0:
            final_lin *= 0.4
            final_lat_y = 0.0

        # ====================================================
        # 到着判定 (REACHED) ロジック
        # ====================================================
        dist = np.hypot(x, y)
        tracking_elapsed = (time.time() - self.tracking_start_time) if self.tracking_start_time else 0.0
        
        # 条件:
        # 1. matched=True（実際にターゲットを観測できている）
        # 2. 追跡開始から最低REACHED_MIN_TRACKING_SEC秒経過
        # 3. 距離が目標付近（±REACHED_DISTANCE_TOLERANCE）
        # 4. 前進・横移動・旋回がほぼゼロ（安定状態）
        if (matched
            and tracking_elapsed > REACHED_MIN_TRACKING_SEC
            and abs(dist - FOLLOW_DISTANCE) < REACHED_DISTANCE_TOLERANCE
            and abs(final_lin) < 0.03
            and abs(final_lat_y) < 0.03
            and abs(final_ang) < 0.15):
            if self.reached_start_time is None:
                self.reached_start_time = time.time()
                self.get_logger().info(f"到着判定カウント開始 (dist={dist:.2f}m, elapsed={tracking_elapsed:.1f}s)")
            elif time.time() - self.reached_start_time > REACHED_STABLE_SEC:
                # REACHED_STABLE_SEC秒安定したら到着完了とする
                self.state = 'REACHED'
                self.get_logger().info(f"【追跡完了】目標地点に到達し安定しました。(安定{REACHED_STABLE_SEC}s, 追跡{tracking_elapsed:.1f}s)")
                self.latest_cmd = Twist()
                self.pub.publish(self.latest_cmd)
                return
        else:
            if self.reached_start_time is not None:
                self.get_logger().info(f"到着判定カウントリセット (matched={matched}, dist={dist:.2f}, lin={final_lin:.2f}, ang={final_ang:.2f})")
            self.reached_start_time = None

        # コマンドのパブリッシュ
        self.latest_cmd = Twist()
        self.latest_cmd.linear.x = float(final_lin)
        self.latest_cmd.linear.y = float(final_lat_y)
        self.latest_cmd.angular.z = float(final_ang)

        if matched:
            self.get_logger().info(
                f"Track[{mode}/{source}]: lin={final_lin:.2f}, ang={final_ang:.2f}, "
                f"lat_y={final_lat_y:.2f}, front={front_distance:.2f}, "
                f"width={self.corridor_width:.2f}, center={self.corridor_center_error:.2f}"
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
            
        for pos, tid, bwr in clusters:
            if self.target_track_id is not None and tid == self.target_track_id:
                self.ekf.x[:2] = pos
                self.ekf.x[2:] = 0.0
                self.ekf.P = np.diag([0.2, 0.2, 1.0, 1.0])
                self.state = 'TRACKING'
                self.lost_frames = 0
                self._tentative_tid = None
                self._tentative_tid_count = 0
                self.get_logger().info(f"ロストから復帰しました！ (ID一致)")
                return
                
        # 見つからなかった場合は予測位置に近い人を採用（閾値を厳しく: 0.7倍）
        LOST_RECOVERY_GATE = TRACKING_GATE * 0.7
        for pos, tid, bwr in clusters:
            euclidean = np.linalg.norm(pos - self.last_known_pos)
            if euclidean < LOST_RECOVERY_GATE:
                # 外見一貫性もチェック
                if self.target_bbox_width > 0.01 and bwr > 0.01:
                    if abs(bwr - self.target_bbox_width) > 0.2:
                        self.get_logger().info(f"ロスト復帰候補(d={euclidean:.2f}m)だが体格不一致(bwr={bwr:.3f} vs {self.target_bbox_width:.3f})。スキップ")
                        continue
                self.ekf.x[:2] = pos
                self.ekf.x[2:] = 0.0
                self.ekf.P = np.diag([0.2, 0.2, 1.0, 1.0])
                self.state = 'TRACKING'
                self.lost_frames = 0
                self._tentative_tid = None
                self._tentative_tid_count = 0
                if tid >= 0:
                    self.target_track_id = tid
                self.get_logger().info(f"ロストから復帰しました！ (予測位置一致 d={euclidean:.2f}m)")
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
