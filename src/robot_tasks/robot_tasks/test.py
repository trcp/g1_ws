#!/usr/bin/env python3
"""
MoveItなしでアーム・腰を直接制御するテストスクリプト。
/upper_joints_control トピック (sensor_msgs/JointState) を使用。

arm_joint_control ノードが bringup で起動していれば動作する。
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
import time

# ============================================================
#  ポーズ定義（ここを編集して好きなポーズを追加）
# ============================================================

# 初期姿勢（自然な腕下げ）— /joint_states から取得した実機のデフォルト値ベース
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

# 左手で前を指すリーチが長く人に当たる可能性ありポーズ椅子の指差しで使用
LEFT_ARM_EXTEND = {
    "left_shoulder_pitch_joint": -1.44,
    "left_shoulder_roll_joint": 0.2,
    "left_shoulder_yaw_joint": 0.0698,
    "left_elbow_joint": 0.95,
    "left_wrist_roll_joint": 0.6981,
}

# 左手の肘だけで椅子を指すポーズリーチ短し
LEFT_ARM_BEND = {
    "left_elbow_joint": -0.3,
}

# 腰を右に回す
WAIST_LEFT = {
    "waist_yaw_joint": -1.0,
}

# 腰を左に回す
WAIST_RIGHT = {
    "waist_yaw_joint": 0.5,
}

# 腰を正面に戻す
WAIST_CENTER = {
    "waist_yaw_joint": 0.0,
}


# ============================================================
#  ジョイント制御クラス
# ============================================================

class DirectJointController:
    """
    /upper_joints_control トピック経由でアーム・腰を直接制御するクラス。
    MoveIt 不要。bringup の arm_joint_control ノードに直接送信する。
    """

    def __init__(self, node: Node):
        self.node = node
        self.pub = node.create_publisher(
            JointState, '/upper_joints_control', 10)

        # 現在のジョイント状態を取得（BEST_EFFORT で受ける）
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.current_joints = {}
        self.sub = node.create_subscription(
            JointState, '/joint_states', self._joint_cb, qos)

        # 少し待って初期値を取得
        node.get_logger().info("Waiting for /joint_states...")
        for _ in range(30):
            rclpy.spin_once(node, timeout_sec=0.1)
            if self.current_joints:
                break
        if self.current_joints:
            node.get_logger().info(f"Got joint states: {len(self.current_joints)} joints")
        else:
            node.get_logger().warn("Could not get joint states (continuing anyway)")

    def _joint_cb(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self.current_joints[name] = pos

    def send_joints(self, joint_dict: dict, hold_sec: float = 0.5):
        """
                       指定したジョイント値を送信する。
                       指定されなかった関節は、自動的に HOME_POSE の値で補完される。
        """
        # 1. ベースとして HOME_POSE のコピーを作成
        full_pose = HOME_POSE.copy()

        # 2. 引数で渡された指定値（例: {"waist_yaw_joint": -1.0}）で上書き
        full_pose.update(joint_dict)

        # 3. あとは元通りの送信処理（full_pose を使うように変更）
        msg = JointState()
        msg.name = list(full_pose.keys())
        msg.position = list(full_pose.values())
        msg.velocity = [0.0] * len(full_pose)

        self.node.get_logger().info(
            f"Sending joints (merged with HOME): {list(joint_dict.keys())} -> hold {hold_sec}s")

        start = time.time()
        while time.time() - start < hold_sec:
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.05)
            time.sleep(0.02)  # ~50Hz

    def go_home(self, hold_sec: float = 3.0):
        """初期姿勢に戻す"""
        self.node.get_logger().info("Going to HOME pose...")
        self.send_joints(HOME_POSE, hold_sec=hold_sec)


# ============================================================
#  メイン
# ============================================================

def main():
    rclpy.init()
    node = Node("test")
    ctrl = DirectJointController(node)

    try:

        # 4. 腰を左に回す
        node.get_logger().info("=== Waist left ===")
        ctrl.send_joints(WAIST_LEFT, hold_sec=2.0)
        # 6. 全部ホームに戻す
        node.get_logger().info("=== Back to HOME ===")
        ctrl.go_home(hold_sec=3.0)

        # 1. 左腕を前に伸ばす
        node.get_logger().info("=== Left arm extend ===")
        ctrl.send_joints(LEFT_ARM_EXTEND, hold_sec=3.0)

        # 2. 左腕を曲げる
        node.get_logger().info("=== Left arm bend ===")
        ctrl.send_joints(LEFT_ARM_BEND, hold_sec=3.0)

        # 3. 左腕を再び伸ばす
        node.get_logger().info("=== Left arm extend again ===")
        ctrl.send_joints(LEFT_ARM_EXTEND, hold_sec=3.0)



        # 5. 腰を右に回す
        node.get_logger().info("=== Waist right ===")
        ctrl.send_joints(WAIST_RIGHT, hold_sec=2.0)


        node.get_logger().info("=== Test complete! ===")

    except KeyboardInterrupt:
        node.get_logger().info("Interrupted. Going home...")
        ctrl.go_home(hold_sec=3.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
