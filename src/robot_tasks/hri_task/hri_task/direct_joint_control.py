#!/usr/bin/env python3
"""
MoveItの代わりに /upper_joints_control を使って直接関節角を制御するモジュール。
バックグラウンドスレッドで50Hzでパブリッシュし続けることで、姿勢を保持します。
"""
import sys, os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import rclpy
import json
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import time
import threading
import math

# ============================================================
#  ポーズ定義
# ============================================================

# 初期姿勢（自然な腕下げ）
HOME_POSE = {
    "left_shoulder_pitch_joint": 0.29,
    "left_shoulder_roll_joint": 0.23,
    "left_shoulder_yaw_joint": -0.02,
    "left_elbow_joint": 0.97,
    "left_wrist_roll_joint": 0.08,
    "right_shoulder_pitch_joint": 0.29,
    "right_shoulder_roll_joint": -0.23,
    "right_shoulder_yaw_joint": 0.03,
    "right_elbow_joint": 0.97,
    "right_wrist_roll_joint": -0.13,
    "waist_yaw_joint": 0.0,
}

# 指差し/受け渡し用ポーズ
ARM_POSE_EXTEND_LEFT = {
    "left_shoulder_pitch_joint": -1.6,
    "left_shoulder_roll_joint": 1.1694,
    "left_shoulder_yaw_joint": 0.0698,
    "left_elbow_joint": 0.0,
    "left_wrist_roll_joint": 0.6981,
}

ARM_POSE_EXTEND_RIGHT = {
    "right_shoulder_pitch_joint": -1.6,
    "right_shoulder_roll_joint": -0.2,
    "right_shoulder_yaw_joint": -0.0698,
    "right_elbow_joint": 0.95,
    "right_wrist_roll_joint": -0.6981,
}


class DirectJointController:
    """
    /upper_joints_control トピック経由でアーム・腰を直接制御するクラス。
    MoveIt 不要。内部のスレッドが姿勢を保持し続けるため、力が抜けません。
    """

    def __init__(self, node: Node):
        self.node = node
        self.pub = node.create_publisher(
            JointState, '/upper_joints_control', 10)

        # 現在のジョイント状態を取得（BEST_EFFORT）
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.current_joints = {}
        self.sub = node.create_subscription(
            JointState, '/joint_states', self._joint_cb, qos)

        # 少し待って初期値を取得
        self.node.get_logger().info("Waiting for /joint_states...")
        for _ in range(30):
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.current_joints:
                break
        if self.current_joints:
            self.node.get_logger().info(f"Got joint states: {len(self.current_joints)} joints")
        else:
            self.node.get_logger().warn("Could not get joint states (continuing anyway)")

        # バックグラウンドでのパブリッシュ用
        # 初期の current_joints をターゲットに設定することで、起動直後にガクッと落ちるのを防ぐ
        self.target_joints = self.current_joints.copy()
        self.smoothed_joints = self.current_joints.copy() # 滑らかに補間するための内部状態
        self.lock = threading.Lock()
        
        # バックグラウンド追従用のフラグとサブスクライバ
        self.bg_tracking_active = False
        self.yolo_sub = self.node.create_subscription(
            String,
            '/yolo_human/result',
            self._yolo_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        
        self.active = True
        self.is_paused = False
        self.pub_thread = threading.Thread(target=self._publish_loop, daemon=True)
        self.pub_thread.start()

    def _joint_cb(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self.current_joints[name] = pos

    def _publish_loop(self):
        """常に target_joints を 10Hz でパブリッシュし続けるスレッド。
        急激な動きを防ぐため、smoothed_joints を使って最大速度を制限します。
        """
        while self.active and rclpy.ok():
            if self.is_paused:
                time.sleep(0.1)
                continue
                
            try:
                with self.lock:
                    target_copy = self.target_joints.copy()

                if target_copy:
                    # 速度制限（スムージング）の適用
                    for k, target_val in target_copy.items():
                        current_val = self.smoothed_joints.get(k, target_val)
                        diff = target_val - current_val
                        
                        # 腰は 0.5 rad/s (0.05/tick at 10Hz)、腕は 1.0 rad/s (0.1/tick at 10Hz) に制限
                        max_step = 0.05 if "waist" in k else 0.1
                        
                        step = max(-max_step, min(max_step, diff))
                        self.smoothed_joints[k] = current_val + step

                    msg = JointState()
                    msg.header.stamp = self.node.get_clock().now().to_msg()
                    msg.name = list(self.smoothed_joints.keys())
                    msg.position = list(self.smoothed_joints.values())
                    msg.velocity = [0.0] * len(self.smoothed_joints)
                    
                    self.pub.publish(msg)
            except Exception as e:
                # Node or publisher destroyed, or other error
                self.node.get_logger().error(f"[_publish_loop] Error: {e}")
                
            time.sleep(0.1)  # ~10Hz (50Hzだとコントローラーの負荷が高くなりナビゲーションをブロックするため)

    def send_joints(self, joint_dict: dict, hold_sec: float = 0.5):
        """
        指定したジョイントの目標角度を更新する。
        パブリッシュはバックグラウンドスレッドが継続して行うため、姿勢は維持される。

        Parameters
        ----------
        joint_dict : dict
            {ジョイント名: 角度(rad)} の辞書
        hold_sec : float
            呼び出し元でブロック（待機）する時間。
        """
        # ターゲットを更新
        with self.lock:
            for k, v in joint_dict.items():
                self.target_joints[k] = v

        # 呼び出し元がブロックしたい時間だけ待機
        if hold_sec > 0:
            start = time.time()
            while rclpy.ok() and time.time() - start < hold_sec:
                # 待機中もROSのコールバックを回しておく（STT等への影響を防ぐ）
                rclpy.spin_once(self.node, timeout_sec=0.05)
                time.sleep(0.05)

    def pause(self):
        """パブリッシュを一時停止する（Navi等との干渉回避用・現在は未使用）"""
        self.is_paused = True

    def resume(self):
        """パブリッシュを再開する"""
        self.is_paused = False

    def go_home(self, hold_sec: float = 3.0):
        """初期姿勢に戻す"""
        self.send_joints(HOME_POSE, hold_sec=hold_sec)

    def turn_waist_towards(self, angle_rad: float, gain: float = 0.8, hold_sec: float = 0.0):
        # 指定された角度（カメラ中心からのズレ）だけ腰を回す
        turn_rad = angle_rad * gain
        current_waist = self.current_joints.get('waist_yaw_joint', 0.0)
        target_waist = max(-1.2, min(1.2, current_waist + turn_rad))
        self.send_joints({'waist_yaw_joint': target_waist}, hold_sec=hold_sec)

    def point_right(self, hold_sec: float = 2.0):
        """右腕で指差す動作"""
        self.send_joints(ARM_POSE_EXTEND_RIGHT, hold_sec=hold_sec)

    def extend_both_arms(self, hold_sec: float = 3.0):
        """両腕を前に出す動作（バッグ受け取りなど）"""
        self.send_joints({**ARM_POSE_EXTEND_LEFT, **ARM_POSE_EXTEND_RIGHT}, hold_sec=hold_sec)

    def point_at_guest(self, current_waist_yaw: float, target_guest_waist_yaw: float, hold_sec: float = 2.0):
        """現在の腰の向きから、ターゲットの腰の向きへの差分を計算し、左手または右手で指差す"""
        diff = target_guest_waist_yaw - current_waist_yaw
        
        # 差分の絶対値を0〜90度(1.57rad)に制限
        theta = min(abs(diff), 1.57)
        
        # 正面(theta=0)なら pitch=-1.6, 真横(theta=1.57)なら pitch=0.0
        calculated_pitch = -1.6 * math.cos(theta)
        
        if diff > 0:
            # ターゲットが左側にいる場合、左手を使う
            # 正面ならroll=0.2, 真横ならroll=1.6
            calculated_roll = 0.2 + 1.4 * math.sin(theta)
            joints = {
                "waist_yaw_joint": current_waist_yaw,
                "left_shoulder_pitch_joint": calculated_pitch,
                "left_shoulder_roll_joint": calculated_roll,
                "left_shoulder_yaw_joint": 0.0698,
                "left_elbow_joint": 0.95,
                "left_wrist_roll_joint": 0.6981,
            }
        else:
            # ターゲットが右側にいる場合、右手を使う
            # 正面ならroll=-0.2, 真横ならroll=-1.6
            calculated_roll = -0.2 - 1.4 * math.sin(theta)
            joints = {
                "waist_yaw_joint": current_waist_yaw,
                "right_shoulder_pitch_joint": calculated_pitch,
                "right_shoulder_roll_joint": calculated_roll,
                "right_shoulder_yaw_joint": -0.0698,
                "right_elbow_joint": 0.95,
                "right_wrist_roll_joint": -0.6981,
            }
            
        self.send_joints(joints, hold_sec=hold_sec)

    # ================= バックグラウンド追従機能 =================
    
    def start_background_tracking(self):
        """YOLOの結果に基づく腰の自動追従を開始する"""
        self.node.get_logger().info("[DirectJointController] Background tracking STARTED")
        self.bg_tracking_active = True

    def stop_background_tracking(self):
        """YOLOの結果に基づく腰の自動追従を停止する"""
        self.node.get_logger().info("[DirectJointController] Background tracking STOPPED")
        self.bg_tracking_active = False

    def _yolo_callback(self, msg: String):
        """YOLOの検出結果を受け取り、フラグがTrueなら自動で腰を向ける"""
        if not self.bg_tracking_active:
            return
            
        try:
            detections = json.loads(msg.data)
            people = [d for d in detections if d.get('label') == 'person']
            if not people:
                return
                
            # 一番近い人を抽出
            target = min(people, key=lambda p: p.get('distance_z', 999.0))
            z = target.get('distance_z', 999.0)
            # YOLO出力は右が正。ジョイント座標系（左が正）に統一するため反転
            angle_rad = -target.get('angle_rad', 0.0)
            w_ratio = target.get('bbox_width_ratio', 0.0)
            
            # 1.3m以内であれば追従 (z=0.0は深度エラーとして除外)
            if w_ratio < 0.6 and 0.1 < z < 1.3:
                # 遊び（デッドバンド）: 0.2 rad
                if abs(angle_rad) > 0.2:
                    self.turn_waist_towards(angle_rad, hold_sec=0)
        except Exception:
            pass
