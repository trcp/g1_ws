#!/usr/bin/env python3
from rclpy.node import Node
import rclpy

from erasers_g1_api.tts import TTS

from ament_index_python.packages import get_package_share_directory
import os


def main():
    rclpy.init()
    node = Node("sample_play_audio")

    tts = TTS(node)
    say = tts.say
    play_audio = tts.audio

    say("Hello, I am Erasers G1.")
    play_audio(os.path.join(get_package_share_directory('erasers_g1_api'), 'config', 'declare_bringup.wav'), wait=True)
    play_audio(os.path.join(get_package_share_directory('erasers_g1_api'), 'config', 'req_sound.wav'), wait=False)
