#!/usr/bin/env python3
from rclpy.node import Node
import rclpy

# smach state
import smach

# msgs
from geometry_msgs.msg import Twist

# G1 API
from erasers_g1_api.tts import TTS
from erasers_g1_api.robot_control import G1Control
from erasers_g1_api.state_skills.wait_door_open import WaitDoorOpen

# general
import time


def main():
    # init ROS
    rclpy.init()
    node = Node('receptionist_task')

    # init API
    tts = TTS(node)
    control = G1Control(node)
    SAY = tts.say

    # init sys
    control.move_head(tilt=0.0)

    # declare start task
    node.get_logger().info('''
    =======================
    RECEPTIONIST TASK START
    =======================
    ''')
    SAY('Receptionist task start')

    control.move_head(tilt=-0.5)
    control.move_velosity(yaw=0.7, time_sec=1.0)
    SAY('Hi What your name?')
    time.sleep(10)
    SAY('I am thinking ...')
    time.sleep(10)
    SAY('Ok. Please back for me.')
    SAY('We will go to the drinnking area. Please follow me.')
    control.move_head(tilt=0.0)
    control.move_velosity(yaw=-0.7, time_sec=1.0)
    time.sleep(5)
    control.move_velosity(x=0.2, time_sec=1.0)

    # init pub
    SAY('Please follow me.')
    control.move_velosity(yaw=0.7, time_sec=1.0)

    SAY('What your favorite drink?')
    time.sleep(10)
    SAY('OK!')
    SAY('Please wait. I will serch')
    control.move_velosity(yaw=-0.35, time_sec=1.0)
    control.move_head(tilt=0.5)
    SAY('Serching now.')
    time.sleep(5)

    '''
    sm = smach.StateMachine(outcomes=['success', 'failure'])


    # states
    with sm:
        smach.StateMachine.add('WAIT_OPEN_DOOR', WaitDoorOpen(node,
                                                              SAY,
                                                              threshold=1.2),
                                transitions={
                                    'open': 'success',
                                    'timeout': 'WAIT_OPEN_DOOR',
                                    'failure': 'failure'}
                                )
    
    outcome = sm.execute()
    '''


    # finish task
    node.get_logger().info('Finish the task.')
    SAY('Finish the task.')
    node.destroy_node()
