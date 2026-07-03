#!/usr/bin/env python3
import argparse
import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

"""
テスト: main.py と同じ空席検出 + 座席指差しフロー。
VLA/VLM は使わず、YoloEmptyChairState の bbox ベース処理だけを確認する。
"""
import rclpy
from rclpy.node import Node
import smach

from erasers_g1_api.tts import TTS
from direct_joint_control import DirectJointController

from yolo_states import YoloEmptyChairState
from main import arm_action_cb


def add_empty_chair_flow(sm, node, say, arm, guest_index, final_success='success'):
    with sm:
        smach.StateMachine.add(
            f'FIND_EMPTY_SEAT_{guest_index}',
            YoloEmptyChairState(
                node=node, direct_arm=arm,
                guest_index=guest_index, timeout=12.0),
            transitions={
                'success': f'POINT_SEAT_{guest_index}',
                'failure': f'POINT_SEAT_{guest_index}',
                'timeout': f'POINT_SEAT_{guest_index}'})

        smach.StateMachine.add(
            f'POINT_SEAT_{guest_index}',
            smach.CBState(cb=arm_action_cb, cb_kwargs={
                'node': node, 'tts_say': say, 'direct_arm': arm,
                'action_type': 'point_seat'}),
            transitions={
                'success': final_success,
                'failure': final_success})


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--guest-index',
        type=int,
        choices=[1, 2],
        default=1,
        help='Run the same empty-seat flow as main.py for this guest.')
    parser.add_argument(
        '--both-guests',
        action='store_true',
        help='Run guest 1 then guest 2 with the same arm object, like main.py.')
    parsed, ros_args = parser.parse_known_args(args)

    rclpy.init(args=ros_args)
    node = Node('test_empty_chair')

    tts = TTS(node)
    SAY = tts.say
    arm = DirectJointController(node)

    sm = smach.StateMachine(outcomes=['success', 'failure'])
    if parsed.both_guests:
        add_empty_chair_flow(sm, node, SAY, arm, guest_index=1, final_success='FIND_EMPTY_SEAT_2')
        add_empty_chair_flow(sm, node, SAY, arm, guest_index=2, final_success='success')
    else:
        add_empty_chair_flow(sm, node, SAY, arm, guest_index=parsed.guest_index)

    node.get_logger().info(
        "Starting Empty Chair Detection & Pointing Test "
        f"(guest_index={parsed.guest_index}, both_guests={parsed.both_guests})."
    )
    outcome = sm.execute()
    node.get_logger().info(f"Test finished with outcome: {outcome}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
