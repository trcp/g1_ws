#!/usr/bin/env python3
"""
YOLO Docker ノードとのトピック通信を使った SMACH ステート群。
yolo_human_node が別コンテナで起動していることを前提とする。
"""
import sys, os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.node import Node
import smach
import json
import time
import math
import subprocess
import threading
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from direct_joint_control import ARM_POSE_EXTEND_LEFT, ARM_POSE_EXTEND_RIGHT, HOME_POSE
from bag_grasp_ik import calculate_bag_grasp_joints
from yolo_track import YoloHumanTracker

# ============================================================
# ベースクラス: YOLO コマンド送信 + 結果受信
# ============================================================
class BaseYoloState(smach.State):
    """YOLOノードへの start/stop コマンド送信と結果受信の共通処理。"""

    def __init__(self, node: Node, target_classes,
                 command_topic='/yolo_human/command',
                 result_topic='/yolo_human/result',
                 timeout=5.0,
                 input_keys=[],
                 output_keys=[]):
        smach.State.__init__(self, outcomes=['success', 'failure', 'timeout'],
                             input_keys=input_keys, output_keys=output_keys)
        self.node = node
        self.target_classes = target_classes
        self.timeout = timeout

        # Publisher（コマンド送信用）
        self.cmd_pub = self.node.create_publisher(String, command_topic, 10)

        # Subscriber（結果受信用）— 永続的に購読してキャッシュ
        self.latest_msg = None
        self._result_sub = self.node.create_subscription(
            String, result_topic, self._result_callback, 10)

    def _result_callback(self, msg):
        self.latest_msg = msg.data

    def start_yolo(self):
        """YOLOノードに推論開始コマンドを送信する。"""
        cmd = {"command": "start", "classes": self.target_classes}
        msg = String()
        msg.data = json.dumps(cmd)
        self.cmd_pub.publish(msg)
        self.node.get_logger().info(f"[YOLO] START classes={self.target_classes}")

    def stop_yolo(self):
        """YOLOノードに推論停止コマンドを送信する。"""
        cmd = {"command": "stop"}
        msg = String()
        msg.data = json.dumps(cmd)
        self.cmd_pub.publish(msg)
        self.node.get_logger().info("[YOLO] STOP")

    def wait_for_result(self, timeout=None, spin=True):
        """最新の結果が届くまで待機する。タイムアウトしたら None を返す。"""
        self.latest_msg = None
        t = timeout if timeout is not None else self.timeout
        start_time = time.time()
        while rclpy.ok() and self.latest_msg is None:
            if time.time() - start_time > t:
                return None
            if spin:
                rclpy.spin_once(self.node, timeout_sec=0.1)
            else:
                time.sleep(0.1)
        return self.latest_msg

    def get_closest_person(self, detections: list):
        """複数の人物の中から一番近い(distance_zが最小の)ものを探す"""
        people = [d for d in detections if d.get('label') == 'person']
        if not people:
            return None
        return min(people, key=lambda p: p.get('distance_z', 999.0))

    def parse_yolo_target(self, msg_str):
        """JSON文字列から一番近いターゲットを抽出し、基本情報を返す"""
        try:
            detections = json.loads(msg_str)
            target = self.get_closest_person(detections)
            if target:
                z = target.get('distance_z', 999.0)
                # YOLO出力は右が正。ジョイント座標系（左が正）に統一するため反転
                angle_rad = -target.get('angle_rad', 0.0)
                w_ratio = target.get('bbox_width_ratio', 0.0)
                return target, z, angle_rad, w_ratio
        except json.JSONDecodeError:
            pass
        return None, 999.0, 0.0, 0.0


# ============================================================
# 人物トラッキング: 腰の回転で一番近い人の方を向く
# ============================================================
class YoloTrackingState(BaseYoloState):
    """
    YOLO で一番近い person を検出し、腰（waist_yaw_joint）の回転で
    その人の方向を向くステート。

    - use_waist=True: 腰を回して追従（インタラクション時）
    - use_waist=False: cmd_vel 的なログ出力のみ（将来のフォロー用）
    - max_loops: 追従ループ回数（デフォルト3）
    """

    def __init__(self, node, target_classes=None, timeout=20.0,
                 direct_arm=None, use_waist=True, distance_threshold=1.3, consecutive_frames=3, max_loops=3):
        if target_classes is None:
            target_classes = ["person"]
        super().__init__(node, target_classes, timeout=timeout)
        self.direct_arm = direct_arm
        self.use_waist = use_waist
        self.distance_threshold = distance_threshold
        self.consecutive_frames = consecutive_frames
        self.max_loops = max_loops

    def execute(self, userdata):
        self.node.get_logger().info("[YOLO TRACKING] Starting...")
        self.start_yolo()
        
        start_time = time.time()
        close_frames = 0
        
        while rclpy.ok():
            if time.time() - start_time > self.timeout:
                self.node.get_logger().info("  -> Tracking timeout. Forcing success.")
                break

            msg = self.wait_for_result(timeout=1.0)
            if msg:
                target, z, angle_rad, w_ratio = self.parse_yolo_target(msg)
                if target:
                    # 人が1.3m以内に来た場合のみ処理を行う (z=0.0は深度取得エラーの可能性があるため弾く)
                    if 0.1 < z < self.distance_threshold:
                        # インタラクション開始条件：画面中央付近を向いているか（しきい値を緩く 0.35 rad = 約20度）
                        if abs(angle_rad) < 0.35:
                            close_frames += 1
                        else:
                            close_frames = 0

                        # 腰を回して対象をカメラの中央に捉える（追跡）
                        if self.use_waist and self.direct_arm and w_ratio < 0.6:
                            # 真ん中の判定を緩くし、中央から 0.2 rad 以上ずれたら腰を回す
                            if abs(angle_rad) > 0.2:
                                try:
                                    self.direct_arm.turn_waist_towards(angle_rad, hold_sec=0)
                                except Exception as e:
                                    self.node.get_logger().warn(f"  -> Waist move failed: {e}")

                        # 中央を向いた状態が一定フレーム続けばインタラクションへ
                        if close_frames >= self.consecutive_frames:
                            self.node.get_logger().info("  -> Target is close and centered. Ending tracking.")
                            break
                    else:
                        # 1.3mより遠い場合は無視し、カウントもリセット
                        close_frames = 0
                else:
                    close_frames = 0
            else:
                self.node.get_logger().info("  -> No YOLO result (timeout)")
            
            time.sleep(0.1)

        self.stop_yolo()

        if self.use_waist and self.direct_arm:
            self.node.get_logger().info("  -> Resetting waist to 0.0")
            try:
                self.direct_arm.send_joints({'waist_yaw_joint': 0.0}, hold_sec=1.0)
            except Exception as e:
                self.node.get_logger().warn(f"  -> Waist reset failed: {e}")

        return 'success'


# ============================================================
# 空席検出: person と chair の重なりから空いている椅子を見つける
# ============================================================
class YoloEmptyChairState(BaseYoloState):
    """
    YOLO で person と chair を同時に検出し、人と重なっていない椅子を
    「空席」として判定する。見つかった椅子の方向に腰を向ける。
    """

    def __init__(self, node, direct_arm=None, guest_index=1, timeout=5.0):
        super().__init__(node, target_classes=["person", "chair"], timeout=timeout, output_keys=['empty_seat_index'])
        self.direct_arm = direct_arm
        self.guest_index = guest_index

    def execute(self, userdata):
        self.node.get_logger().info("[YOLO CHAIR] Searching for empty chair...")
        self.start_yolo()

        for _ in range(5):
            msg = self.wait_for_result()
            if msg:
                try:
                    detections = json.loads(msg)
                    # 信頼度(score)がある場合は 0.5 以上のものだけを抽出する（YOLO側のデフォルトに依存）
                    people = [d for d in detections if d.get('label') == 'person' and d.get('score', 1.0) >= 0.7]
                    chairs = [d for d in detections if d.get('label') == 'chair' and d.get('score', 1.0) >= 0.7]

                    # bboxから正確な中心X座標を計算する関数
                    def get_x_center(item):
                        bbox = item.get('bbox', [0, 0, 0, 0])
                        if len(bbox) == 4:
                            return (bbox[0] + bbox[2]) / 2.0
                        return 0.0

                    # 椅子を左から右（x座標の昇順）にソート
                    chairs.sort(key=get_x_center)
                    
                    # --- 動的キャリブレーション（1回目の椅子の位置を保存） ---
                    if self.guest_index == 1 and self.direct_arm:
                        # 1回目は部屋がクリーンなので、すべての椅子のX座標を記憶しておく
                        self.direct_arm.saved_chair_x_centers = [get_x_center(c) for c in chairs]
                        self.node.get_logger().info(f"Saved initial chair centers: {self.direct_arm.saved_chair_x_centers}")

                    empty_chairs = []
                    occupied_chairs = []
                    occupied_info = []
                    seat_status_log = []

                    for i, c in enumerate(chairs):
                        c_box = c.get('bbox', [0, 0, 0, 0])
                        c_area = max(1.0, (c_box[2] - c_box[0]) * (c_box[3] - c_box[1]))
                        
                        is_empty = True
                        for p in people:
                            p_box = p.get('bbox', [0, 0, 0, 0])
                            
                            # 共通部分（Intersection）の計算
                            x_left = max(c_box[0], p_box[0])
                            y_top = max(c_box[1], p_box[1])
                            x_right = min(c_box[2], p_box[2])
                            y_bottom = min(c_box[3], p_box[3])
                            
                            if x_right > x_left and y_bottom > y_top:
                                overlap_area = (x_right - x_left) * (y_bottom - y_top)
                                overlap_ratio = overlap_area / c_area
                                
                                # 人の中心Xと椅子の中心Xの距離も考慮する
                                c_cx = (c_box[0] + c_box[2]) / 2.0
                                p_cx = (p_box[0] + p_box[2]) / 2.0
                                c_width = max(1.0, c_box[2] - c_box[0])
                                
                                # 重なりが10%以上あるか、X座標の中心同士が近い（椅子の幅の半分以下）場合は「座っている」と判定
                                if overlap_ratio > 0.1 or abs(c_cx - p_cx) < c_width * 0.6:
                                    is_empty = False
                                    occupied_chairs.append(c)
                                    occupied_info.append(f"左から{i+1}番目(重なり{int(overlap_ratio*100)}%)")
                                    seat_status_log.append(f"Chair {i+1}: Occupied")
                                    break
                                    
                        if is_empty:
                            seat_status_log.append(f"Chair {i+1}: Empty")
                            empty_chairs.append(c)

                    # 情報のログ出力
                    self.node.get_logger().info(
                        f"検出結果 -> 椅子合計: {len(chairs)}個, 人: {len(people)}人, 空き椅子: {len(empty_chairs)}個"
                    )
                    self.node.get_logger().info(f"Seat status (left to right): [{', '.join(seat_status_log)}]")
                    
                    if occupied_info:
                        self.node.get_logger().info(f"人が座っている椅子: {', '.join(occupied_info)}")

                    if empty_chairs:
                        # 常に一番左の本当に空いている椅子をターゲットにする
                        empty_chair = empty_chairs[0]
                        c_x = get_x_center(empty_chair)
                        
                        empty_idx = 1
                        # 記憶したX座標リストの中で、もっとも距離が近いもののインデックス（1-indexed）を採用
                        if self.direct_arm and hasattr(self.direct_arm, 'saved_chair_x_centers') and self.direct_arm.saved_chair_x_centers:
                            distances = [abs(c_x - saved_x) for saved_x in self.direct_arm.saved_chair_x_centers]
                            closest_idx = distances.index(min(distances))
                            empty_idx = closest_idx + 1
                            self.node.get_logger().info(f"-> Mapped empty chair to saved index: {empty_idx} (c_x={c_x:.1f})")
                        else:
                            # 万が一データがない場合は、画面の左半分か右半分かで判定
                            empty_idx = 1 if c_x < 320 else 2
                            self.node.get_logger().info(f"-> Mapped empty chair by screen half: {empty_idx} (c_x={c_x:.1f})")

                        userdata.empty_seat_index = empty_idx
                        if self.direct_arm:
                            self.direct_arm.empty_seat_index = empty_idx
                        self.node.get_logger().info(f"-> Selected empty chair index: {empty_idx} (from left)")
                        
                        if self.direct_arm:
                            # 万が一のために、人が座っている椅子の角度も計算して保存しておく
                            if occupied_chairs:
                                h_x = get_x_center(occupied_chairs[0])
                                # c_x < 320 (左) ならプラスの角度になるように計算（左旋回＝プラス）
                                h_angle_rad = (320 - h_x) * (87.0 / 640.0) * math.pi / 180.0
                                current_waist = self.direct_arm.current_joints.get('waist_yaw_joint', 0.0)
                                self.direct_arm.host_waist_yaw = current_waist + (h_angle_rad * 1.5 * 0.8)
                                self.node.get_logger().info(f"Host chair position saved as waist_yaw: {self.direct_arm.host_waist_yaw}")

                            # x_centerから回転角を再計算
                            c_x = get_x_center(empty_chair)
                            angle_rad = (320 - c_x) * (87.0 / 640.0) * math.pi / 180.0
                            target_turn = angle_rad * 1.5
                            
                            # 紹介フェーズのために、この「空き椅子の方向（＝ゲスト2）」も保存しておく
                            self.direct_arm.guest2_waist_yaw = current_waist + (target_turn * 0.8)
                            self.node.get_logger().info(f"Guest 2 chair position saved as waist_yaw: {self.direct_arm.guest2_waist_yaw}")
                            
                            self.direct_arm.turn_waist_towards(target_turn, hold_sec=0.0)
                        self.stop_yolo()
                        return 'success'

                except Exception as e:
                    self.node.get_logger().warn(f"  -> Parse error: {e}")
                    # 例外が出てもクラッシュさせずリトライする
            time.sleep(1.0)

        self.stop_yolo()
        if self.direct_arm:
            # フォールバック時もインデックスの更新を確実に行う
            fallback_idx = 1 if self.guest_index == 1 else 2
            userdata.empty_seat_index = fallback_idx
            self.direct_arm.empty_seat_index = fallback_idx
            
            # フォールバック時も hold_sec=0.0 で即座に次の腕の動作に移行させる
            fallback_turn = 0.5 if fallback_idx == 1 else -0.5
            self.direct_arm.turn_waist_towards(fallback_turn, hold_sec=0.0)
        return 'failure'


# ============================================================
# バッグ検出
# ============================================================
class YoloFindBagState(BaseYoloState):
    """YOLO で bag を検出するステート。"""

    def __init__(self, node, timeout=5.0):
        super().__init__(node, target_classes=["bag"], timeout=timeout)

    def execute(self, userdata):
        self.node.get_logger().info("[YOLO BAG] Searching for bag...")
        self.start_yolo()

        msg = self.wait_for_result()
        if msg:
            self.node.get_logger().info("  -> Bag detected!")
        else:
            self.node.get_logger().info("  -> No bag detected (timeout), continuing anyway")

        self.stop_yolo()
        return 'success'


# ============================================================
# ホスト追従: 止まったら終了
# ============================================================
class YoloFollowHostState(BaseYoloState):
    """
    YOLO で person を追従し、ホストが止まったことを検知して終了する。
    """

    def __init__(self, node, tts_say=None, direct_arm=None, control=None,
                 timeout=3.0, max_duration=60.0, stop_threshold=0.05,
                 stop_count_required=10, stop_distance=0.8):
        super().__init__(node, target_classes=["person"], timeout=timeout)
        self.tts_say = tts_say
        self.direct_arm = direct_arm
        self.control = control
        self.max_duration = max_duration
        self.stop_threshold = stop_threshold
        self.stop_count_required = stop_count_required
        self.stop_distance = stop_distance

    def execute(self, userdata):
        self.node.get_logger().info("[FOLLOW HOST] Starting host follow...")
        if self.tts_say:
            self.tts_say("Host, please guide me to your destination.")
            time.sleep(1.0)
            
        self.start_yolo()

        if self.control:
            try:
                self.control.pose_policy('running')
                self.control.move_head(tilt=0.0)
            except Exception as e:
                self.node.get_logger().warn(f"  -> Pose policy failed: {e}")

        prev_z = None
        prev_angle = None
        stop_count = 0
        start_time = time.time()
        
        # (以前は移動中の干渉を防ぐために direct_arm を pause していましたが、
        # 現在は10Hz化により両立できるため pause せず、腕の剛性を保ちます)
        if self.direct_arm:
            self.node.get_logger().info("  -> direct_arm remains active to keep joint stiffness.")

        # サブプロセスではなく、スレッド内でトラッカー(YoloHumanTracker)を起動
        self.tracker_node = None
        self.tracker_executor = None
        self.tracker_thread = None
        try:
            self.node.get_logger().info("  -> Starting YoloHumanTracker in background thread...")
            self.tracker_node = YoloHumanTracker()
            self.tracker_executor = SingleThreadedExecutor()
            self.tracker_executor.add_node(self.tracker_node)
            self.tracker_thread = threading.Thread(target=self.tracker_executor.spin, daemon=True)
            self.tracker_thread.start()
        except Exception as e:
            self.node.get_logger().error(f"  -> Failed to start tracker thread: {e}")

        try:
            if self.tts_say:
                self.tts_say("I am following you now.")
                time.sleep(1.0)
                
            while rclpy.ok():
                if time.time() - start_time > self.max_duration:
                    if self.tts_say:
                        self.tts_say("I have followed you for the maximum duration.")
                        time.sleep(1.0)
                    break

                msg = self.wait_for_result(timeout=2.0)
                if msg:
                    # _handle_tracking内で自己完結的に REACHED に移行したかをチェック
                    if self.tracker_node and self.tracker_node.state == 'REACHED':
                        self.node.get_logger().info("  -> Tracker node reports REACHED! Stop condition met.")
                        if self.tts_say:
                            self.tts_say("It seems we have arrived.")
                            time.sleep(1.0)
                        break
                        
                    target, z, angle, w_ratio = self.parse_yolo_target(msg)
                    if target:
                        self.node.get_logger().info(f"  -> Following: Z={z:.2f}m, rad={angle:.2f}")
                
        finally:
            if self.tracker_executor is not None:
                self.node.get_logger().info("  -> Shutting down YoloHumanTracker thread...")
                self.tracker_executor.shutdown()
            if self.tracker_node is not None:
                self.tracker_node.destroy_node()
            if self.tracker_thread is not None:
                self.tracker_thread.join(timeout=2.0)

            # 剛性保持のため pause していないので resume も不要
            if self.direct_arm:
                self.node.get_logger().info("  -> Finished tracking.")

        self.stop_yolo()
        if self.control:
            try:
                self.control.pose_policy('start')
                self.control.move_head(tilt=0.0)
            except Exception as e:
                self.node.get_logger().warn(f"  -> Pose reset failed: {e}")

        return 'success'

# ============================================================
# 右手のみのバッグ把持 (層的インタラクション)
# ============================================================
class YoloBagGraspInteractionState(BaseYoloState):
    """
    ゲストに特徴を伝えた直後に実行されるバッグ把持ステート。
    ルールに基づき、カメラを下に向け、足回りでかばんに近づき（画面中央・50cm）、右手のみで把持位置（上部少し右）へ腕を伸ばす。
    """

    def __init__(self, node, tts_say=None, direct_arm=None, control=None, timeout=8.0):
        super().__init__(node, target_classes=["bag"], timeout=timeout)
        self.tts_say = tts_say
        self.direct_arm = direct_arm
        self.control = control
        self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)

    def execute(self, userdata):
        self.node.get_logger().info("[YOLO BAG GRASP] Starting visual servoing for bag grasp...")

        try:
            # 0. 把持前ホームポジションへ（右肘を曲げて体にくっつけることでバッグへの接触を防ぐ）
            bag_home = HOME_POSE.copy()
            bag_home['right_elbow_joint'] = -1.0
            
            if self.direct_arm:
                self.node.get_logger().info("  -> Moving to bag grasp home posture...")
                self.direct_arm.send_joints(bag_home, hold_sec=1.5)

            # 1. 頭を下に向ける (かばんを探すため)
            head_tilt_val = -0.5
            if self.control:
                try:
                    self.node.get_logger().info("  -> Tilting head down...")
                    self.control.move_head(tilt=head_tilt_val)
                except Exception as e:
                    self.node.get_logger().warn(f"  -> Head tilt failed: {e}")
            
            time.sleep(1.0)
            self.start_yolo()

            # 2. バッグを1回だけ見つけて座標を取得（移動はしない）
            start_time = time.time()
            found_bag = False
            final_bag_cx = 320.0
            final_bag_cy = 240.0
            final_bag_z = 0.3
            
            while rclpy.ok() and (time.time() - start_time < self.timeout):
                msg = self.wait_for_result(timeout=1.0)
                if not msg:
                    continue

                try:
                    detections = json.loads(msg)
                    bags = [d for d in detections if d.get('label') == 'bag']
                    if not bags:
                        continue
                        
                    target_bag = min(bags, key=lambda b: b.get('distance_z', 999.0))
                    
                    bag_z = target_bag.get('distance_z', 0.3)
                    bbox = target_bag.get('bbox', [320, 240, 320, 240])
                    bbox_width = bbox[2] - bbox[0]
                    bbox_height = bbox[3] - bbox[1]
                    bag_cx = (bbox[0] + bbox[2]) / 2.0
                    
                    # 持ち手部分（上部）と、右手把持を考慮した右寄りオフセット
                    final_bag_cx = bag_cx + (bbox_width * 0.15)
                    final_bag_cy = bbox[1] + (bbox_height * 0.05)
                    final_bag_z = bag_z
                    
                    found_bag = True
                    break

                except Exception as e:
                    self.node.get_logger().error(f"  -> Error parsing bag: {e}")

            if not found_bag:
                self.node.get_logger().warn("  -> Could not find bag. Attempting grasp at default position.")

            # 3. 把持姿勢を取る
            self.node.get_logger().info(f"  -> Calculating IK for bag at cx={final_bag_cx}, cy={final_bag_cy}, z={final_bag_z}")
            
            target_joints = calculate_bag_grasp_joints(final_bag_cx, final_bag_cy, final_bag_z, head_tilt=head_tilt_val)
            
            if self.direct_arm:
                self.node.get_logger().info(f"  -> Sending Grasp Joints: {target_joints}")
                self.direct_arm.send_joints(target_joints, hold_sec=2.0)
                
                # 手を閉じる前の案内と待機
                if self.tts_say:
                    self.tts_say("I will close my hand in 3 seconds to grasp the bag.")
                
                self.node.get_logger().info("  -> Waiting 3 seconds before closing hand...")
                time.sleep(3.0)
                
                self.node.get_logger().info("[HAND COMMAND] hand_control('right', 'close') executed")
                time.sleep(1.0)
                
                # 腕を戻す（まずは把持ホームポジションを経由）
                self.node.get_logger().info("  -> Returning via bag grasp home posture...")
                self.direct_arm.send_joints(bag_home, hold_sec=1.5)

            return 'success'

        except Exception as e:
            self.node.get_logger().error(f"  -> Unexpected error in execute: {e}")
            return 'failure'

        finally:
            # 安全のためのクリーンアップ処理（例外発生時も必ず実行）
            try:
                self.stop_yolo()
            except Exception:
                pass
            
            try:
                self.cmd_vel_pub.publish(Twist()) # ベース停止
            except Exception:
                pass
            
            # 頭のピッチを戻す
            if self.control:
                try:
                    self.node.get_logger().info("  -> Resetting head tilt to 0.0...")
                    self.control.move_head(tilt=0.0)
                except Exception as e:
                    self.node.get_logger().warn(f"  -> Head reset failed: {e}")
