#!/usr/bin/env python3
from rclpy.node import Node
import rclpy

# smach state
import smach

# G1 API
from erasers_g1_api.tts import TTS
from erasers_g1_api.state_skills.gemini import GeminiVLMState

# preferences
import traceback


"""
main
"""
def main():
    # init ROS
    rclpy.init()
    node = Node('sample_gemini')

    # init API
    tts = TTS(node)
    SAY = tts.say

    # declare start task
    node.get_logger().info('''
    =====================
    sample_gemini START
    =====================
    ''')

    # init sm
    sm = smach.StateMachine(outcomes=['success', 'failure'])

    # userdata
    sm.userdata.prompt_message = "この画像に写っているものは何？"
    
    # sm
    with sm:
        smach.StateMachine.add('GEMINI', GeminiVLMState(node=node,
                                                         tts_say=SAY),
                                transitions={'success': 'success',
                                             'failure':'failure'})
    
    outcome = sm.execute()
    node.destroy_node()