#!/usr/bin/env python3
import os
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

"""
RealSense + yolo_human_node の結果だけで空席判定を確認する dry-run テスト。

前提:
  1. RealSense が /head_camera/d455/color/image_raw と
     /head_camera/d455/depth/image_rect_raw を publish している
  2. yolo_human_node が /yolo_human/command と /yolo_human/result で動いている

このテストは G1 の関節を動かさず、main.arm_action_cb の発話ロジックだけを通す。
"""

import argparse

import rclpy
from rclpy.node import Node
import smach

from yolo_states import YoloEmptyChairState
from main import arm_action_cb


class DryRunArm:
    """DirectJointController の最小 dry-run 代替。実機の関節は動かさない。"""

    def __init__(self, node):
        self.node = node
        self.current_joints = {'waist_yaw_joint': 0.0}
        self.empty_seat_index = 1
        self.selected_seat_description = None
        self.seat_waist_yaws = {}

    def turn_waist_towards(self, angle_rad: float, gain: float = 0.8, hold_sec: float = 0.0):
        target = self.current_joints.get('waist_yaw_joint', 0.0) + angle_rad * gain
        self.current_joints['waist_yaw_joint'] = max(-1.2, min(1.2, target))
        self.node.get_logger().info(
            f"[DRY RUN ARM] turn_waist_towards angle={angle_rad:.3f}, "
            f"gain={gain:.2f}, target={self.current_joints['waist_yaw_joint']:.3f}"
        )

    def point_right(self, hold_sec: float = 2.0):
        self.node.get_logger().info(f"[DRY RUN ARM] point_right hold_sec={hold_sec}")

    def go_home(self, hold_sec: float = 2.0):
        self.current_joints['waist_yaw_joint'] = 0.0
        self.node.get_logger().info(f"[DRY RUN ARM] go_home hold_sec={hold_sec}")


def dry_tts(text: str):
    print(f"[DRY RUN TTS] {text}", flush=True)


def main(args=None):
    parser = argparse.ArgumentParser(
        description="Dry-run empty-seat detection using RealSense-backed yolo_human_node."
    )
    parser.add_argument('--guest-index', type=int, default=1, choices=[1, 2])
    parser.add_argument('--timeout', type=float, default=12.0)
    parser.add_argument('--expected-seat-count', type=int, default=3)
    parser.add_argument(
        '--skip-point-message',
        action='store_true',
        help='Only run YoloEmptyChairState. Do not call main.arm_action_cb.'
    )
    parsed, _ = parser.parse_known_args(args)

    rclpy.init(args=args)
    node = Node('test_empty_chair_realsense')
    arm = DryRunArm(node)

    sm = smach.StateMachine(outcomes=['success', 'failure'])
    with sm:
        next_state = 'success' if parsed.skip_point_message else 'POINT_SEAT_DRY_RUN'
        smach.StateMachine.add(
            'FIND_EMPTY_SEAT',
            YoloEmptyChairState(
                node=node,
                direct_arm=arm,
                guest_index=parsed.guest_index,
                timeout=parsed.timeout,
                expected_seat_count=parsed.expected_seat_count,
            ),
            transitions={
                'success': next_state,
                'failure': next_state,
                'timeout': next_state,
            })

        if not parsed.skip_point_message:
            smach.StateMachine.add(
                'POINT_SEAT_DRY_RUN',
                smach.CBState(cb=arm_action_cb, cb_kwargs={
                    'node': node,
                    'tts_say': dry_tts,
                    'direct_arm': arm,
                    'action_type': 'point_seat',
                }),
                transitions={
                    'success': 'success',
                    'failure': 'failure',
                })

    node.get_logger().info(
        "Starting RealSense empty-seat dry-run. "
        "Make sure realsense_yolo_human_topics.launch.py and yolo_human_node are running."
    )
    outcome = sm.execute()
    node.get_logger().info(f"Dry-run finished with outcome: {outcome}")
    node.get_logger().info(
        f"Selected seat index={getattr(arm, 'empty_seat_index', None)}, "
        f"description={getattr(arm, 'selected_seat_description', None)}, "
        f"seat_waist_yaws={getattr(arm, 'seat_waist_yaws', {})}"
    )

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
