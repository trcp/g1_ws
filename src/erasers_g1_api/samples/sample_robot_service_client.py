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
    node = Node('sample_robot_led')

    # G1Control を初期化
    robot = G1Control(node)

    # サービス一覧を取得
    services = robot.get_robot_service_interfaces()
    print(services)

    # サービス停止，起動
    robot.robot_service_interface_setting('lidar_driver', False)
    robot.robot_service_interface_setting('lidar_driver', True)

    node.destroy_node()
    rclpy.shutdown()
