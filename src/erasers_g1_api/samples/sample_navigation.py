#!/usr/bin/env python3

'''
G1 Navigation サンプルコード
'''

# ROS
from rclpy.node import Node
import rclpy

# API
from erasers_g1_api.tts import TTS
from erasers_g1_api.robot_control import G1Control, G1Navigation


def main():
    rclpy.init()
    node = Node('sample_head_control')

    # init
    tts = TTS(node)
    robot = G1Control(node)
    navigation = G1Navigation(node)

    tts.say('both hand for walking')
    robot.hand_control(command='walk', hand='both')

    navigation.move_rel(yaw=1.57)
