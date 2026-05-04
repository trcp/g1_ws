#!/usr/bin/env python3

'''
G1 頭部カメラのサーボモーターを制御するサンプルコード
'''

# ROS
from rclpy.node import Node
import rclpy

# API
from erasers_g1_api.tts import TTS
from erasers_g1_api.robot_control import G1Control


def main():
    rclpy.init()
    node = Node('sample_head_control')

    # init
    tts = TTS(node)
    control = G1Control(node)

    # init pose
    tts.say('Initial head pose')
    control.move_head()

    # adjust tilt
    tts.say('tilt up.')
    control.move_head(tilt=1.0)
    tts.say('tilt down.')
    control.move_head(tilt=-1.0)

    # adjust pan
    tts.say('pan left.')
    control.move_head(pan=1.0)
    tts.say('pan right.')
    control.move_head(pan=-1.0)

    # adjust pan & tilt
    tts.say('draw rectangle.')
    control.move_head(tilt=1.0, pan=1.0)
    control.move_head(tilt=1.0, pan=-1.0)
    control.move_head(tilt=-1.0, pan=-1.0)
    control.move_head(tilt=-1.0, pan=1.0)
    control.move_head()
