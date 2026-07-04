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
    arm.enable_upper_body_control(True)
    arm.joint_control(
        left_shoulder_pitch_joint=-0.735,
        left_wrist_roll_joint=-1.57,
        right_shoulder_pitch_joint=-0.735,
        right_wrist_roll_joint=1.57,
    )
    arm.hand_control(command="open", hand="both")

    say("Close hand, 5, 4, 3, 2, 1")

    arm.hand_control(command="close", hand="both")

    # arm.joint_control(
    #     left_shoulder_pitch_joint=0.273,
    #     left_shoulder_yaw_joint=-0.07,
    #     right_shoulder_pitch_joint=0.273,
    #     right_shoulder_yaw_joint=0.07,
    # )

    # arm.move_groupstate(group_name="home")
    arm.enable_upper_body_control(False)

if __name__ == '__main__':
    main()
