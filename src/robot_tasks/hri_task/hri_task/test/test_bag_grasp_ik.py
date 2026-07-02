#!/usr/bin/env python3
import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

"""
テスト: バッグ把持の動作確認
main.py で動かしている YoloBagGraspInteractionState を実行し、
動作前に正しくしゃべるかなどをテストする。
"""
import rclpy
from rclpy.node import Node
import smach

from erasers_g1_api.tts import TTS
from erasers_g1_api.robot_control import G1Control
from direct_joint_control import DirectJointController
from yolo_states import YoloBagGraspInteractionState

def main(args=None):
    rclpy.init(args=args)
    node = Node('test_bag_grasp')

    tts = TTS(node)
    SAY = tts.say
    arm = DirectJointController(node)
    control = G1Control(node)

    # テスト開始前にホームポジションにする
    node.get_logger().info("Moving to home position before test...")
    arm.go_home(hold_sec=2.0)

    sm = smach.StateMachine(outcomes=['success', 'failure', 'timeout'])
    with sm:
        smach.StateMachine.add(
            'BAG_GRASP_INTERACTION',
            YoloBagGraspInteractionState(
                node=node, tts_say=SAY, direct_arm=arm, control=control, timeout=8.0),
            transitions={
                'success': 'success',
                'failure': 'failure',
                'timeout': 'timeout'})

    node.get_logger().info("Starting Bag Grasp Test. Watch and listen carefully.")
    outcome = sm.execute()
    node.get_logger().info(f"Test finished with outcome: {outcome}")

    node.get_logger().info("Moving back to home position...")
    arm.go_home(hold_sec=2.0)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
