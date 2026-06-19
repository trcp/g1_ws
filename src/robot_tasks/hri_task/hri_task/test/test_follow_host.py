#!/usr/bin/env python3
import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import rclpy
from rclpy.node import Node
import smach

from direct_joint_control import DirectJointController
from yolo_states import YoloFollowHostState
from erasers_g1_api.tts import TTS
from erasers_g1_api.robot_control import G1Control

def main(args=None):
    rclpy.init(args=args)
    node = Node('test_follow_host')
    
    node.get_logger().info("Initializing APIs...")
    tts = TTS(node)
    SAY = tts.say
    CONTROL = G1Control(node)
    ARM = DirectJointController(node)
    
    sm = smach.StateMachine(outcomes=['success', 'failure', 'timeout'])
    with sm:
        smach.StateMachine.add('FOLLOW_HOST', 
            YoloFollowHostState(
                node=node, tts_say=SAY, direct_arm=ARM, control=CONTROL,
                max_duration=60.0, stop_threshold=0.05,
                stop_count_required=10, stop_distance=0.8),
            transitions={'success': 'success', 'failure': 'failure', 'timeout': 'timeout'})
                               
    node.get_logger().info("Starting YOLO Follow Host Test...")
    
    outcome = sm.execute()
    node.get_logger().info(f"Test finished with outcome: {outcome}")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
