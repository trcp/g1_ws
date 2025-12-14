#!/usr/bin/env python3

'''
G1 頭部カメラのサーボモーターを制御するサンプルコード
'''

# ROS
from rclpy.node import Node
import rclpy

# API
from erasers_g1_api.robot_control import G1Control


def main():
    rclpy.init()
    node = Node('sample_head_control')

    # init
    control = G1Control(node)

    # init pose
    control.move_head()
