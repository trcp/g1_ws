#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class PointCloudTimestampFixer(Node):
    def __init__(self):
        super().__init__('pointcloud_timestamp_fixer')

        # UnitreeのBare DDS AppのQoSに合わせるため、Reliable設定を使用
        # ただし、ネットワーク状況に応じて Best Effort も許容できるよう柔軟に定義
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # オリジナルトピックの購読
        self.subscription = self.create_subscription(
            PointCloud2,
            '/utlidar/cloud_livox_mid360',
            self.listener_callback,
            qos_profile)

        # 修正後トピックの配信
        self.publisher = self.create_publisher(
            PointCloud2,
            '/utlidar/cloud_livox_mid360_fixed',
            qos_profile)

        self.get_logger().info('PointCloud Timestamp Fixer node has been started.')
        self.get_logger().info('Subscribing to: /utlidar/cloud_livox_mid360')
        self.get_logger().info('Publishing to: /utlidar/cloud_livox_mid360_fixed')

    def listener_callback(self, msg):
        # 元のタイムスタンプをバックアップ（デバッグ用）
        old_sec = msg.header.stamp.sec
        
        # 現在のシステム時刻で上書き
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # 配信
        self.publisher.publish(msg)
        
        # 6秒以上の乖離がある場合に警告を出す（デバッグ用）
        diff = msg.header.stamp.sec - old_sec
        if abs(diff) > 1:
            self.get_logger().debug(f'Adjusted timestamp diff: {diff} seconds')

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudTimestampFixer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
