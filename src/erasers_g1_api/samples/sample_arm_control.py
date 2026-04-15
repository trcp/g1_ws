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
    arm.joint_control(
        left_shoulder_pitch_joint=1.57,
        right_shoulder_pitch_joint=1.57,
        left_shoulder_roll_joint=0.5,
        right_shoulder_roll_joint=-0.5,
        left_elbow_joint=-1.0,
        right_elbow_joint=-1.0
    )
    arm.move_groupstate()
    arm.move_rel(x=0.2, z=0.1, planning_group="arm_left")
    arm.move_groupstate()
    arm.move_rel(x=0.2, z=0.1, planning_group="arm_right")
    arm.move_groupstate()