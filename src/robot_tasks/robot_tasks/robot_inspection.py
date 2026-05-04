"""
Robot Inspection タスク
"""

#!/usr/bin/env python3

# ROS2
from rclpy.node import Node
import rclpy
import smach

# eraasers g1 APIs
from erasers_g1_api.tts import TTS
from erasers_g1_api.robot_control import G1Navigation, G1Control
from erasers_g1_api.state_skills.wait_door_open import WaitDoorOpen

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
                    input_keys=['inspection_point'],
                    output_keys=[])
def move_to_inspection_point_cb(userdata, node:Node, tts_say:TTS.say, navigation:G1Navigation, control:G1Control):
    try:
        inspection_point = userdata.inspection_point
        control.pose_policy('running')
        tts_say("move to inspection point")
        navigation.move_abs(inspection_point[0], inspection_point[1], inspection_point[2])
        control.pose_policy('start')
        tts_say("I reach the inspection point.")
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
    sm.userdata.inspection_point = [1.0, 0.0, 0.0]

    SAY('robot inspection task start!')

    with sm:
        smach.StateMachine.add('WAIT_DOOR_OPEN', WaitDoorOpen(node=node,
                                                              tts_say=SAY,
                                                              threshold=2.0),
                               transitions={
                                   'success': 'MOVE_INSPECTION_POINT',
                                   'timeout': 'WAIT_DOOR_OPEN',
                                   'failure': 'failure'
                                })
        
        smach.StateMachine.add('MOVE_INSPECTION_POINT', smach.CBState(cb=move_to_inspection_point_cb,
                                                                      cb_kwargs={'node': node, 'tts_say': SAY, 'navigation': NAVIGATION, 'control': CONTROL}),
                                transitions={
                                    'success':'success',
                                    'timeout':'failure',
                                    'failure':'failure',
                                })
    
    # execute smach states
    outcome = sm.execute()
    if outcome == 'success':
        SAY("I finished the task.")
    else:
        SAY("I failed to finish the task.")
    node.destroy_node()