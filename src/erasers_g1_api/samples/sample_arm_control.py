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
    say("Armed upper body control.")
    print(arm.get_current_joints_pose())
    arm.enable_upper_body_control(True)

    say("Init pose")
    arm.move_groupstate(group_state="walk")

    say("Disarmed upper body control.")
    arm.enable_upper_body_control(False)

if __name__ == '__main__':
    main()
