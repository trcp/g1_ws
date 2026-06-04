"""
Robot Inspection タスク
"""

#!/usr/bin/env python3

# ROS2
from rclpy.node import Node
import rclpy
import smach

from geometry_msgs.msg import PoseStamped

# eraasers g1 APIs
from erasers_g1_api.tts import TTS
from erasers_g1_api.robot_control import G1Navigation, G1Control
from erasers_g1_api.state_skills.wait_door_open import WaitDoorOpen
from erasers_g1_api.state_skills.recongnition import SpeechToText

# preferences
from ament_index_python.packages import get_package_share_directory
from typing import List
import traceback
import yaml
import os


"""
指定されたロケーションに移動する
"""
@smach.cb_interface(outcomes=['success', 'timeout', 'failure'],
                    input_keys=['abs_pose', 'initial_pose'],
                    output_keys=[])
def move_to_pose_cb(userdata, node:Node, tts_say:TTS.say, navigation:G1Navigation, init_pose:bool=False,  message:str='I will move.'):
    try:
        tts_say(message)
        if init_pose:
            navigation.set_initialpose(pose=[userdata.initial_pose[0], userdata.initial_pose[1], userdata.initial_pose[2]])
        navigation.move_abs(userdata.abs_pose[0],
                            userdata.abs_pose[1],
                            userdata.abs_pose[2])
        return 'success'

    except Exception as e:
        node.get_logger().error(f"Error in move_to_location_cb: {e}\n{traceback.format_exc()}")
        return 'failure'


"""
main
"""
def main():
    # init ROS
    rclpy.init()
    node = Node('robot_inspection_task')

    # init API
    tts = TTS(node)
    SAY = tts.say
    CONTROL = G1Control(node)
    NAVIGATION = G1Navigation(node)

    # init smach
    sm = smach.StateMachine(outcomes=['success', 'failure'])

    # userdatas
    sm.userdata.initial_pose = [-1.08, -0.3, -1.57]
    sm.userdata.inspection_point = [-3.50, -0.95, 0.0]
    sm.userdata.exit_point = [-2.5, -5.0, -1.57]
    sm.userdata.success_keywards = ['yes', 'YES', 'Yes']
    sm.userdata.num_challenge = 0

    SAY('robot inspection task start!')

    with sm:
        smach.StateMachine.add('WAIT_DOOR_OPEN', WaitDoorOpen(node=node,
                                                              tts_say=SAY,
                                                              timeout_sec=20,
                                                              threshold=1.5),
                               transitions={
                                   'success': 'MOVE_INSPECTION_POINT',
                                   'timeout': 'WAIT_DOOR_OPEN',
                                   'failure': 'failure'
                                })
        
        smach.StateMachine.add('MOVE_INSPECTION_POINT', smach.CBState(cb=move_to_pose_cb,
                                                                      cb_kwargs={'node': node, 'tts_say': SAY, 'navigation': NAVIGATION, 'init_pose': True, 'message': 'I will go to the inspection point.'}),
                                transitions={
                                    'success':'HUMAN_INTARACTION',
                                    'timeout':'failure',
                                    'failure':'failure',
                                },
                                remapping={'abs_pose':'inspection_point'})
        
        smach.StateMachine.add('HUMAN_INTARACTION', SpeechToText(node=node,
                                                                 tts=tts,
                                                                 start_msg='My name is erasers_g1, Did you finish my inspection?? If so, please say YES or NO after the chime sounds.',
                                                                 success_msg='Thank you! I will go to exit.',
                                                                 timeout_msg='OK. I will stay.',
                                                                 ),
                                transitions={'success': 'MOVE_EXIT_POINT',
                                             'timeout': 'HUMAN_INTARACTION',
                                             'failure': 'failure',})

        smach.StateMachine.add('MOVE_EXIT_POINT', smach.CBState(cb=move_to_pose_cb,
                                                                      cb_kwargs={'node': node, 'tts_say': SAY, 'navigation': NAVIGATION, 'message': 'Move to exit point.'}),
                                transitions={
                                    'success':'success',
                                    'timeout':'failure',
                                    'failure':'failure',
                                },
                                remapping={'abs_pose':'exit_point'})
    
    # execute smach states
    outcome = sm.execute()
    if outcome == 'success':
        SAY("I finished the task.")
    else:
        SAY("I failed to finish the task.")
    node.destroy_node()
