#!/usr/bin/env python3
"""モック制御と同じ 120 Hz の固定刻みで ROS 時刻を配信する."""

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rosgraph_msgs.msg import Clock as ClockMessage


class MockClock(Node):
    def __init__(self):
        super().__init__('mock_clock')
        self._ticks = 0
        self._publisher = self.create_publisher(ClockMessage, '/clock', 10)
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._timer = self.create_timer(
            1.0 / 120.0, self._publish, clock=self._steady_clock)

    def _publish(self):
        # sleep_until の判定が壁時計の揺れで一周期余分に待たないよう固定刻みにする。
        # 負荷でタイマーが遅れた場合、シミュレーション時刻もその分ゆっくり進む。
        self._ticks += 1
        elapsed = self._ticks * 1_000_000_000 // 120
        msg = ClockMessage()
        msg.clock.sec, msg.clock.nanosec = divmod(elapsed, 1_000_000_000)
        self._publisher.publish(msg)


def main():
    rclpy.init()
    node = MockClock()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
