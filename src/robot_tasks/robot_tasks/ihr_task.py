#!/usr/bin/env python3
from rclpy.node import Node
import rclpy

# TF
from tf2_ros import Buffer, TransformListener

# API
from erasers_g1_api.robot_control import G1Control, G1Navigation, ArmControl
from erasers_g1_api.tts import TTS

# smach skills
from erasers_g1_api.state_skills.recongnition import SimpleObjectDetector

import smach


def main():
    # init ROS2
    rclpy.init()
    node = Node('ihr_task')

    # init TF
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)

    # init API
    tts = TTS(node=node)
    SAY = tts.say
    ROBOT = G1Control(node=node)
    NAVIGATION = G1Navigation(node=node, tf_buffer=tf_buffer)
    ARM = ArmControl(node=node, tf_buffer=tf_buffer)

    # init pose
    ARM.enable(False)   # アームを脱力
    ROBOT.move_head()   # カメラを正面に向かせる

    # init smach
    sm = smach.StateMachine(outcomes=['success', 'failure'])
    ARM.enable(False)
    node.get_logger().info("""
    ================
    iHR TASK START !
    ================
                           """)
    SAY('I H R task start')


    with sm:
        smach.StateMachine.add('SERCHING', SimpleObjectDetector(node=node,
                                                                tts_say=SAY),
                                transitions={
                                    'success':'success',
                                    'timeout':'failure',
                                    'failure':'failure',
                                }
                               )
    
    outcome = sm.execute()
    node.destroy_node()
