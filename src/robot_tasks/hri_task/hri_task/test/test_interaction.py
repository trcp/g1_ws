#!/usr/bin/env python3
import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

"""
テスト: STTインタラクション + 腰追従
main.py の YoloSpeechToTextState と parse_guest_info_cb を直接使用する。
"""
import rclpy
from rclpy.node import Node
import smach

from erasers_g1_api.tts import TTS
from direct_joint_control import DirectJointController

from main import YoloSpeechToTextState, parse_guest_info_cb, TARGET_DICT
from yolo_states import YoloTrackingState


def main(args=None):
    rclpy.init(args=args)
    node = Node('test_interaction')

    tts = TTS(node)
    arm = DirectJointController(node)

    sm = smach.StateMachine(outcomes=['success', 'failure'])
    sm.userdata.stt_text = ""
    sm.userdata.guest_name = ""
    sm.userdata.guest_drink = ""
    sm.userdata.num_challenge = 0
    sm.userdata.success_keywards = []

    with sm:
        smach.StateMachine.add(
            'TRACK_PERSON',
            YoloTrackingState(node=node, direct_arm=arm, timeout=60.0),
            transitions={
                'success': 'ASK_INFO',
                'timeout': 'ASK_INFO',
                'failure': 'failure'})

        smach.StateMachine.add(
            'ASK_INFO',
            YoloSpeechToTextState(
                node=node, tts=tts,
                start_msg="Hello! I am the host robot. "
                          "What is your name and favorite drink?",
                direct_arm=arm),
            transitions={
                'success': 'PARSE_INFO',
                'timeout': 'ASK_INFO',
                'failure': 'failure'})

        smach.StateMachine.add(
            'PARSE_INFO',
            smach.CBState(cb=parse_guest_info_cb, cb_kwargs={'node': node, 'direct_arm': arm}),
            transitions={
                'success': 'success',
                'retry': 'ASK_INFO',
                'failure': 'failure'})

    node.get_logger().info("Starting Interaction Test.")
    outcome = sm.execute()

    if outcome == 'success':
        node.get_logger().info(
            f"Final Result -> Name: {sm.userdata.guest_name}, "
            f"Drink: {sm.userdata.guest_drink}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
