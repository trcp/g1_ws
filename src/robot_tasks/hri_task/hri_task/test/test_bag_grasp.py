#!/usr/bin/env python3
import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

"""
テスト: バッグ把持のIK（運動学）計算を直接テストするスクリプト
YOLOの検出を待たずに、指定した仮想の座標に対してロボットが正しく把持姿勢を取るかを確認します。
"""
import rclpy
from rclpy.node import Node
from direct_joint_control import DirectJointController
from bag_grasp_ik import calculate_bag_grasp_joints
#from erasers_g1_api.robot_control import ArmControl

def main(args=None):
    rclpy.init(args=args)
    node = Node('test_bag_grasp')

    arm = DirectJointController(node)
    #hand = ArmControl(node)

    # テスト開始前にホームポジションにする
    node.get_logger().info("Moving to home position before test...")
    arm.go_home(hold_sec=3.0)

    # テスト用の仮想バッグ座標
    # cx=320 が正面。cx=500 はかなり右側（手より外側）にズレた位置を想定
    bag_cx = 500.0
    bag_cy = 240.0
    bag_z = 0.4  # 40cm先

    node.get_logger().info(f"Target Bag Position: cx={bag_cx}, cy={bag_cy}, distance={bag_z}m")
    
    # 修正した IK を使って関節角度を計算
    #hand_hand_control(command="open", hand="right")
    joints = calculate_bag_grasp_joints(bag_cx, bag_cy, bag_z)
    node.get_logger().info(f"Calculated Joints:\n{joints}")

    # 把持姿勢を取る
    node.get_logger().info("Taking grasp posture...")
    arm.send_joints(joints, hold_sec=5.0)
    #hand_hand_control(command="close", hand="right")
    

    # ホームに戻る
    node.get_logger().info("Moving back to home position...")
    arm.go_home(hold_sec=2.0)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
