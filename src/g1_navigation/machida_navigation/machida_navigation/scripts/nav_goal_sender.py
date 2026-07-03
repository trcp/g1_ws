#!/usr/bin/env python3
"""
Send a NavigateToPose action goal and monitor feedback until completion.

Usage:
  ros2 run machida_navigation nav_goal_sender.py <x> <y> [yaw_deg]

Example:
  ros2 run machida_navigation nav_goal_sender.py 2.0 1.5 90
"""
import math
import sys

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class NavGoalSender(Node):
    def __init__(self):
        super().__init__('nav_goal_sender')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, x: float, y: float, yaw: float = 0.0) -> None:
        self.get_logger().info('Waiting for action server /navigate_to_pose ...')
        self._client.wait_for_server()

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(
            f'Sending goal  x={x:.3f}  y={y:.3f}  yaw={math.degrees(yaw):.1f} deg')

        future = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)
        future.add_done_callback(self._on_goal_accepted)

    # --- callbacks ---

    def _on_goal_accepted(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Goal was REJECTED by the server')
            rclpy.shutdown()
            return
        self.get_logger().info('Goal ACCEPTED — waiting for result ...')
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        pos = fb.current_pose.pose.position
        elapsed = fb.navigation_time.sec + fb.navigation_time.nanosec * 1e-9
        self.get_logger().info(
            f'  pos=({pos.x:.2f}, {pos.y:.2f})'
            f'  dist_remaining={fb.distance_remaining:.2f} m'
            f'  elapsed={elapsed:.1f} s'
        )

    def _on_result(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation SUCCEEDED')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('Navigation was CANCELLED')
        else:
            self.get_logger().error(f'Navigation FAILED  (status={status})')
        rclpy.shutdown()


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)

    x = float(sys.argv[1])
    y = float(sys.argv[2])
    yaw = math.radians(float(sys.argv[3])) if len(sys.argv) > 3 else 0.0

    rclpy.init()
    node = NavGoalSender()
    node.send_goal(x, y, yaw)
    rclpy.spin(node)


if __name__ == '__main__':
    main()
