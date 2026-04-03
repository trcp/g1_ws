#!/usr/bin/env python3

'''
G1 アーム制御 サンプルコード
'''

# ROS
from rclpy.node import Node
import rclpy

# API
from erasers_g1_api.tts import TTS
from erasers_g1_api.robot_control import ArmControl


def main():
    rclpy.init()
    node = Node('sample_head_control')

    # init
    arm = ArmControl(node)

    # init arm pose
    arm.move_groupstate()

    arm.move_rel(pitch=0.1)