#!/usr/bin/env python3

'''
G1 アーム制御 サンプルコード
'''

# ROS
from rclpy.node import Node
import rclpy

# API
from erasers_g1_api.tts import TTS
from erasers_g1_api.robot_control import G1Control, ArmControl


def main():
    rclpy.init()
    node = Node('sample_head_control')

    # init
    tts = TTS(node)
    robot = G1Control(node)
    arm = ArmControl(node)

    arm.enable(True)
    arm.init_pose()
    robot.hand_control(command='open')
    arm.move_rel(x=0.3, arm='right')
    arm.init_pose()
    arm.move_rel(x=0.3, arm='left')
    robot.hand_control(command='close')
