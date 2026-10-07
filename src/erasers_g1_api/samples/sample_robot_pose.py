#!/usr/bin/env python3
'''
ロボットの現在の姿勢モードの取得，
ロボットの姿勢を変更するサンプルコード
'''
from rclpy.node import Node
import rclpy

from erasers_g1_api.robot_control import G1Control


def main():
    rclpy.init()
    node = Node('sample_robot_control')

    # G1Control を初期化
    robot = G1Control(node)

    # 現在のロボットの状態を取得
    fsm_id = robot.get_current_robot_pose()
    print(fsm_id)

    # ロボットの姿勢を変更する
    # src/erasers_g1_interfaces/srv/RobotPose.srv 参照
    robot.robot_pose(7)

    node.destroy_node()
    rclpy.shutdown()
