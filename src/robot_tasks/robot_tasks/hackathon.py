#!/usr/bin/env python3
from rclpy.node import Node
import rclpy

from erasers_g1_api.robot_control import G1Control
from erasers_g1_api.tts import TTS

import time


def main():
    rclpy.init()
    node = Node("hackathon")

    tts = TTS(node)
    robot = G1Control(node)
    
    robot.running()
    time.sleep(5)
    tts.say("Please ask request to me.")

    time.sleep(10)

    tts.say("OK. please follow me")

    robot.yaw(yaw=3.14, speed=1.0)
    robot.linear(distance=1.0, speed=0.5)

    robot.yaw(yaw=-1.57, speed=1.0)
    tts.say("Reach the trush area.")
    time.sleep(3)
    robot.shakehand()
    tts.say("Please trush here!")
    time.sleep(25)

    tts.say("I will go back to home point")
    robot.yaw(yaw=-1.57, speed=0.5)
    robot.linear(distance=1.0, speed=0.5)


    tts.say("Please ask request to me.")

    time.sleep(10)

    tts.say("OK. I will tidyup on the table.")
    robot.yaw(yaw=3.14, speed=1.0)
    robot.linear(distance=1.0, speed=0.5)
    robot.yaw(yaw=1.57, speed=0.5)
    

    robot.start()
    time.sleep(5)
    robot.shakehand()

    time.sleep(5)
    robot.linear(distance=0.2, speed=0.2)
    time.sleep(5)
    robot.lateral(distance=0.4, speed=0.2)

    tts.say("Finish the task!")