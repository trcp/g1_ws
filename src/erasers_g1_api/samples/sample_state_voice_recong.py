#!/usr/bin/env python3
from rclpy.node import Node
import rclpy

# smach state
import smach

# G1 API
from erasers_g1_api.tts import TTS
from erasers_g1_api.state_skills.recongnition import SpeechToText

# preferences
import traceback


"""
聞こえたメッセージを確認する
"""
@smach.cb_interface(outcomes=['success', 'failure'],
                    input_keys=['stt_text'],
                    output_keys=[])
def cb_state_check_verify(userdata, node:Node, tts_say:TTS.say):
    try:
        stt_text = userdata.stt_text
        tts_say("I heard %s"%stt_text)
        return 'success'
    except Exception as e:
        node.get_logger().error(f"Error in grasp_bag: {e}\n{traceback.format_exc()}")
        return 'failure'


"""
main
"""
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
                                                            tts=tts,
                                                            device='cuda',
                                                            lang='en',
                                                            silence_duration=5.0),
                                transitions={'success': 'CHECK_VERIFY',
                                             'timeout': 'VOICE_RECONG',
                                             'failure':'failure'})
        smach.StateMachine.add('CHECK_VERIFY', smach.CBState(cb=cb_state_check_verify,
                                                             cb_kwargs={'node': node, 'tts_say': SAY}),
                                transitions={'success': 'success',
                                             'failure':'failure'})
    
    outcome = sm.execute()
    node.destroy_node()
