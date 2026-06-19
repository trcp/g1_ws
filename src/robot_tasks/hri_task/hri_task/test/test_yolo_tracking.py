#!/usr/bin/env python3
import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import rclpy
from rclpy.node import Node
import smach

from direct_joint_control import DirectJointController
from yolo_states import YoloTrackingState

def main(args=None):
    rclpy.init(args=args)
    node = Node('test_yolo_tracking')
    
    node.get_logger().info("Initializing DirectJointController...")
    arm = DirectJointController(node)
    
    # テスト用の簡単なステートマシン
    sm = smach.StateMachine(outcomes=['success', 'failure'])
    with sm:
        # 腰を使ったYOLO追従をテスト（20秒間）
        smach.StateMachine.add('TRACK_PERSON', 
                               YoloTrackingState(node=node, target_classes=["person"], direct_arm=arm, use_waist=True, timeout=20.0),
                               transitions={'success': 'success', 'failure': 'failure', 'timeout': 'success'})
                               
    node.get_logger().info("Starting YOLO Tracking Test. Please stand in front of the camera.")
    
    # 実行
    outcome = sm.execute()
    node.get_logger().info(f"Test finished with outcome: {outcome}")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
