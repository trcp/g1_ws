#!/usr/bin/env python3
"""G1 標準 TTS と VOICEVOX（話者 26）で短い発話を行う。"""

import rclpy
from rclpy.node import Node

from erasers_g1_api.tts import VuiTTS, VoicevoxTTS


def main():
    """再生完了を待ち、終了時に ROS ノードを片付ける。"""
    rclpy.init()
    node = Node("sample_tts")
    try:
        vui = VuiTTS(node)
        voicevox = VoicevoxTTS(node)  # 既定の speaker_id は 26。
        if not vui.say("Hello. I am G1."):
            node.get_logger().error("G1 標準 TTS の発話に失敗しました")
        if not voicevox.say("こんにちは。ジーワンです。"):
            node.get_logger().error("VOICEVOX の発話に失敗しました")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
