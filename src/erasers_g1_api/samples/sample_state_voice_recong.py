#!/usr/bin/env python3
from rclpy.node import Node
import rclpy

# smach state
import smach

# G1 API
from erasers_g1_api.tts import TTS
from erasers_g1_api.state_skills.recongnition import SpeechToText


def main():
    # init ROS
    rclpy.init()
    node = Node('sample_voice_recongnition')

    # init API
    tts = TTS(node)
    SAY = tts.say

    # declare start task
    node.get_logger().info('''
    ===============================
    sample_voice_recongnition START
    ===============================
    ''')

    # init sm
    sm = smach.StateMachine(outcomes=['success', 'failure'])

    # userdata
    sm.userdata.num_challenge = 0

    # sm
    with sm:
        smach.StateMachine.add('VOICE_RECONG', SpeechToText(node=node,
                                                            tts_say=SAY,
                                                            model_size='large',
                                                            device='cuda',
                                                            lang='ja',
                                                            silence_duration=5.0),
                                transitions={'success': 'success',
                                             'timeout': 'VOICE_RECONG',
                                             'failure':'failure'})
    
    outcome = sm.execute()
    node.destroy_node()