#!/usr/bin/env python3
import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

"""
テスト: 空席検出 + 椅子を指差す
"""
import rclpy
from rclpy.node import Node
import smach

from erasers_g1_api.tts import TTS
from direct_joint_control import DirectJointController

from yolo_states import YoloEmptyChairState
from main import arm_action_cb


def main(args=None):
    rclpy.init(args=args)
    node = Node('test_empty_chair')

    tts = TTS(node)
    SAY = tts.say
    arm = DirectJointController(node)

    GUEST_INDEX = 1

    sm = smach.StateMachine(outcomes=['success', 'failure'])
    with sm:
        smach.StateMachine.add(
            'FIND_EMPTY_SEAT',
            YoloEmptyChairState(
                node=node, direct_arm=arm,
                guest_index=GUEST_INDEX, timeout=12.0),
            transitions={
                'success': 'POINT_SEAT',
                'failure': 'POINT_SEAT',
                'timeout': 'POINT_SEAT'})

        smach.StateMachine.add(
            'POINT_SEAT',
            smach.CBState(cb=arm_action_cb, cb_kwargs={
                'node': node, 'tts_say': SAY, 'direct_arm': arm,
                'action_type': 'point_seat'}),
            transitions={
                'success': 'success',
                'failure': 'failure'})

    node.get_logger().info("Starting Empty Chair Detection & Pointing Test.")
    outcome = sm.execute()
    node.get_logger().info(f"Test finished with outcome: {outcome}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
