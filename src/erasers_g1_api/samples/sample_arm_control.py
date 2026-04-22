#!/usr/bin/env python3

'''
G1 アーム制御 サンプルコード
'''

# ROS
from rclpy.node import Node
import rclpy

# API
from erasers_g1_api.tts import TTS
from erasers_g1_api.robot_control import ArmControl

import math


def main():
    rclpy.init()
    node = Node('sample_head_control')

    # init
    arm = ArmControl(node)
    tts = TTS(node)
    say = tts.say

    # init arm pose
    say("Init pose")
    arm.move_groupstate()

    # Add a small target object
    obj_name = "target_cube"
    arm.collision.add_box(obj_name, x=0.4, y=0.1, z=0.0, size=(0.04, 0.04, 0.04))
    say(f"Target object {obj_name} added. Starting grasp sequence")
    if arm.grasp_manager.grasp(obj_name):
        say("Grasp sequence completed successfully")
        # Lift and move
        arm.move_rel(z=0.1)
    else:
        say("Grasp sequence failed")

    say("Verification finished. Cleaning up")
    arm.collision.remove_collision("obstacle")
    arm.collision.remove_collision(obj_name)
    arm.move_groupstate()

if __name__ == '__main__':
    main()
