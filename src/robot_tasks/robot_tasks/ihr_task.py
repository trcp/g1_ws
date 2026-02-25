#!/usr/bin/env python3
from rclpy.node import Node
import rclpy

# TF
from tf2_ros import Buffer, TransformListener

# API
from erasers_g1_api.robot_control import G1Control, G1Navigation, ArmControl
from erasers_g1_api.tts import TTS

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
    ARM.enable(False)

    node.get_logger().info("""
    ================
    iHR TASK START !
    ================
                           """)
    SAY('ihr task start')

    # 現在位置の取得
    pose = NAVIGATION.get_current_pose(simple=True)
    print(pose)

    ROBOT.move_head(tilt=-1.0)

    # 作業場所へ移動
    NAVIGATION.move_abs(0.75, -0.84, -1.13)
