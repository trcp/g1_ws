#!/usr/bin/env python3

'''
G1 から任意のテキストを発話させるサンプルコード．
G1 のデフォルト TTS 機能を使うため，英語のみの発話をサポートします．

ros2 run erasers_g1_api sample_tts

'''

# ROS
from rclpy.node import Node
import rclpy

# TTS
from erasers_g1_api.tts import TTS


def main():
    # init rclpy
    rclpy.init()

    # create node
    node = Node('sample_tts')

    # init TTS
    tts = TTS(node)

    # create say func
    say = tts.say

    # let's speaking!
    say('Hello! I am erasers G1!')
    say('Do you like a potato?')

    # 日本語を書くと Japanese... Japanese... と意図しない発話をする．
    say('こんにちは')


# execute from python
if __name__ == '__main__':
    main()
