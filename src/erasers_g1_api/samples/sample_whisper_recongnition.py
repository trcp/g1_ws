#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from erasers_g1_api.tts import VoicevoxTTS
from erasers_g1_api.recongnision import WhisperSpeechToText


def main():
    rclpy.init()
    node = Node("sample_whisper_stt")

    tts = VoicevoxTTS(node)
    stt = WhisperSpeechToText(node)

    tts.say("なにか喋ってください．")
    result = stt.recongnition()
    print(f'Whisper result: {result}')
    tts.say(f"{result}")


if __name__ == "__main__":
    raise SystemExit(main())
