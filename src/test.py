{"filename": "src/erasers_g1_common_cpp/scripts/timestamp_analyzer.py"}
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, Imu

class TimestampAnalyzer(Node):
    def __init__(self):
        super().__init__('timestamp_analyzer')

        # Cartographerに入力されるQoSと同一の設定
        qos_imu = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=2000
        )
        qos_lidar = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub_lidar = self.create_subscription(
            PointCloud2, '/utlidar/cloud_livox_mid360_fixed', self.lidar_cb, qos_lidar)
        self.sub_imu = self.create_subscription(
            Imu, '/utlidar/imu_livox_mid360_fixed', self.imu_cb, qos_imu)

        self.last_lidar_time = None
        self.last_imu_time = None

        self.lidar_count = 0
        self.imu_count = 0

        # 1秒ごとに同期状態をレポート
        self.timer = self.create_timer(1.0, self.report_status)

        self.get_logger().info("Timestamp Analyzer started. Monitoring _fixed topics...")

    def get_time_sec(self, stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def lidar_cb(self, msg):
        current_time = self.get_time_sec(msg.header.stamp)
        self.lidar_count += 1

        if self.last_lidar_time is not None:
            delta = current_time - self.last_lidar_time
            if delta == 0:
                self.get_logger().warn(f"[LiDAR] Duplicate timestamp detected! {current_time:.6f}")
            elif delta < 0:
                self.get_logger().error(f"[LiDAR] Time went backwards! Delta: {delta:.6f} s")

        self.last_lidar_time = current_time

    def imu_cb(self, msg):
        current_time = self.get_time_sec(msg.header.stamp)
        self.imu_count += 1

        if self.last_imu_time is not None:
            delta = current_time - self.last_imu_time
            if delta == 0:
                self.get_logger().warn(f"[IMU] Duplicate timestamp detected! {current_time:.6f}")
            elif delta < 0:
                self.get_logger().error(f"[IMU] Time went backwards! Delta: {delta:.6f} s")

        self.last_imu_time = current_time

    def report_status(self):
        if self.last_lidar_time is not None and self.last_imu_time is not None:
            diff = abs(self.last_lidar_time - self.last_imu_time)
            # Cartographerの仕様上、このdiffが数ミリ秒〜数十ミリ秒以内に収まっている必要がある
            if diff > 0.1:
                self.get_logger().error(f"SYNC LOST! LiDAR-IMU Diff: {diff:.6f} s")
            else:
                self.get_logger().info(
                    f"Sync OK | LiDAR-IMU Diff: {diff:.6f} s | "
                    f"LiDAR Msgs/s: {self.lidar_count} | IMU Msgs/s: {self.imu_count}"
                )
        else:
            self.get_logger().info("Waiting for both LiDAR and IMU data...")

        self.lidar_count = 0
        self.imu_count = 0

def main(args=None):
    rclpy.init(args=args)
    node = TimestampAnalyzer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
