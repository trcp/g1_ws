#!/usr/bin/env python3

'''
Amazing Hand を制御するサンプルコード
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
    tts = TTS(node)
    arm = ArmControl(node)

    tts.say('open the left hand', False)
    arm.hand_control(command='open', hand='left')

    tts.say('open the right hand', False)
    arm.hand_control(command='open', hand='right')

    tts.say('close the both hand', False)
    arm.hand_control(command='close', hand='both')

    tts.say('both hand for walking', False)
    arm.hand_control(command='walk', hand='both')
