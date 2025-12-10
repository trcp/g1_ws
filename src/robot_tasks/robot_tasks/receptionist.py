#!/usr/bin/env python3
from rclpy.node import Node
import rclpy

# smach state
import smach

# G1 API
from erasers_g1_api.tts import TTS


def main():
    # init ROS
    rclpy.init()
    node = Node('receptionist_task')

    # init API
    tts = TTS(node)
    say = tts.say

    # declare start task
    node.get_logger().info('''
    =======================
    RECEPTIONIST TASK START
    =======================
    ''')
    say('Receptionist task start')

    # finish task
    node.get_logger().info('Finish the task.')
    say('Finish the task.')
    node.destroy_node()
