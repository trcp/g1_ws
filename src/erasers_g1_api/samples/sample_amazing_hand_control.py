#!/usr/bin/env python3
from rclpy.node import Node
import rclpy

from erasers_g1_api.tts import TTS
from erasers_g1_api.robot_control import ArmControl


def main():
    rclpy.init()
    node = Node("sample_amazing_hand_control")
    tts = TTS(node)
    arm_control = ArmControl(node)

    # 両手を広げる
    tts.say('Open both hands')
    arm_control.hand_control(command="open", hand="both")

    # 両手を閉じる
    tts.say('Close both hands')
    arm_control.hand_control(command="close", hand="both")

    # 歩行用姿勢にする
    tts.say('Set to walking posture')
    arm_control.hand_control(command="walk", hand="both")
