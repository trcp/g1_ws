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

import math


def main():
    rclpy.init()
    node = Node('sample_head_control')

    # init
    arm = ArmControl(node)
    tts = TTS(node)
    say = tts.say

    # init arm pose
    say("Init pose")
    arm.move_groupstate()
    arm.move_rel(planning_group="arm_left", roll=1.57, x=0.2)
    arm.move_groupstate()
    arm.move_rel(planning_group="arm_right", roll=-1.57, x=0.2)
    arm.move_groupstate()

if __name__ == '__main__':
    main()
