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
def move_to_inspection_point_cb(userdata, node:Node, tts_say:TTS.say, message:str='move to inspection point'):
    try:
        goal_pub = node.create_publisher(PoseStamped, '/goal_pose', 10)

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = userdata.inspection_point[0]
        goal.pose.position.y = userdata.inspection_point[1]
        goal.pose.position.z = 0.0
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        goal_pub.publish(goal)
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
    #NAVIGATION = G1Navigation(node)

    # init smach
    sm = smach.StateMachine(outcomes=['success', 'failure'])

    # userdatas
    sm.userdata.inspection_point = [-2.89, -1.9, 0.0]
    sm.userdata.exit_point = [6.4, 0.95, 0.0]

    SAY('robot inspection task start!')

    with sm:
        smach.StateMachine.add('WAIT_DOOR_OPEN', WaitDoorOpen(node=node,
                                                              tts_say=SAY,
                                                              timeout_sec=20,
                                                              threshold=1.0),
                               transitions={
                                   'success': 'MOVE_INSPECTION_POINT',
                                   'timeout': 'MOVE_INSPECTION_POINT',
                                   'failure': 'failure'
                                })
        
        smach.StateMachine.add('MOVE_INSPECTION_POINT', smach.CBState(cb=move_to_inspection_point_cb,
                                                                      cb_kwargs={'node': node, 'tts_say': SAY}),
                                transitions={
                                    'success':'MOVE_EXIT_POINT',
                                    'timeout':'failure',
                                    'failure':'failure',
                                })

        smach.StateMachine.add('MOVE_EXIT_POINT', smach.CBState(cb=move_to_inspection_point_cb,
                                                                      cb_kwargs={'node': node, 'tts_say': SAY, 'message': 'Move to exit point.'}),
                                transitions={
                                    'success':'success',
                                    'timeout':'failure',
                                    'failure':'failure',
                                },
                                remapping={'inspection_point':'exit_point'})
    
    # execute smach states
    outcome = sm.execute()
    if outcome == 'success':
        SAY("I finished the task.")
    else:
        SAY("I failed to finish the task.")
    node.destroy_node()
