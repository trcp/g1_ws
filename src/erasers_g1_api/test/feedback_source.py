"""結合テスト用の独立した LowState・ArmActionState 送信プロセス。"""

import os
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String
from unitree_hg.msg import LowState


def main():
    config = Path(__file__).resolve().parents[2] / 'erasers_g1_common/test/cyclonedds.xml'
    if (os.environ.get('ROS_DOMAIN_ID') != '173' or
            os.environ.get('CYCLONEDDS_URI') != config.as_uri()):
        raise RuntimeError('テスト専用の loopback DDS 設定が必要です')
    rclpy.init()
    node = rclpy.create_node('integration_feedback')
    low = node.create_publisher(LowState, '/lowstate', 10)
    arm = node.create_publisher(String, '/arm/action/state', 10)
    state = LowState()
    enabled = True

    def switch(message):
        nonlocal enabled
        enabled = message.data

    node.create_subscription(
        Bool, '/integration/feedback_enabled', switch,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def tick():
        if enabled:
            state.tick += 1
            low.publish(state)
        arm.publish(String(data='{"id":0,"name":"normal","holding":false}'))

    node.create_timer(0.02, tick)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
