#!/usr/bin/env python3
from rclpy.node import Node
import rclpy

from erasers_g1_api.tts import TTS
from erasers_g1_api.robot_control import ArmControl, G1Control, G1Navigation

import time


def main():
    rclpy.init()
    node = Node("test")

    tts = TTS(node)
    robot = G1Control(node)
    nav = G1Navigation(node)
    arm = ArmControl(node)


    # nav.move_rel(yaw=1.57)

    robot.pose_policy("start")
    arm.enable_upper_body_control(True)

    arm.move_groupstate()
    arm.hand_control(command="open", hand="left")
    arm.move_rel(planning_group="arm_left", roll=1.57, x=0.2)
    arm.hand_control(command="close", hand="left")
    arm.move_groupstate()
    arm.hand_control(command="open", hand="right")
    arm.move_rel(planning_group="arm_right", roll=-1.57, x=0.2)
    arm.hand_control(command="close", hand="right")
    arm.move_groupstate()

    arm.hand_control(command="open", hand="left")
    arm.joint_control(
        left_shoulder_pitch_joint=-1.44,
        left_shoulder_roll_joint=1.1694,
        left_shoulder_yaw_joint=0.0698,
        left_elbow_joint=0.0,
        left_wrist_roll_joint=0.6981
    )
    arm.joint_control(
        left_shoulder_pitch_joint=-1.1519,
        left_shoulder_roll_joint=1.2915,
        left_shoulder_yaw_joint=0.5411,
        left_elbow_joint=0.1571,
        left_wrist_roll_joint=0.5585
    )
    arm.joint_control(
        left_shoulder_pitch_joint=-1.44,
        left_shoulder_roll_joint=1.1694,
        left_shoulder_yaw_joint=0.0698,
        left_elbow_joint=0.0,
        left_wrist_roll_joint=0.6981
    )
    arm.joint_control(
        left_shoulder_pitch_joint=-1.1519,
        left_shoulder_roll_joint=1.2915,
        left_shoulder_yaw_joint=0.5411,
        left_elbow_joint=0.1571,
        left_wrist_roll_joint=0.5585
    )
    arm.joint_control(
        left_shoulder_pitch_joint=-1.44,
        left_shoulder_roll_joint=1.1694,
        left_shoulder_yaw_joint=0.0698,
        left_elbow_joint=0.0,
        left_wrist_roll_joint=0.6981
    )
    arm.hand_control(command="walk", hand="both")

    arm.enable_upper_body_control(False)

    time.sleep(5.0)
    robot.pose_policy("running")

    # nav.move_abs()
