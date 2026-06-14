from rclpy.node import Node
import rclpy

from erasers_g1_api.robot_control import ArmControl

import math


rclpy.init()
node = Node("okawa_taisou")

arm = ArmControl(node)


arm.move_groupstate()


arm.joint_control(
    waist_yaw_joint=math.radians(0),
    left_shoulder_pitch_joint=math.radians(-90),
    left_shoulder_roll_joint=math.radians(30),
    left_shoulder_yaw_joint=math.radians(-90),
    left_elbow__joint=math.radians(90),
    left_wrist_roll_joint=math.radians(-113),
    right_shoulder_pitch_joint=math.radians(-91),
    right_shoulder_roll_joint=math.radians(-11),
    right_shoulder_yaw_joint=math.radians(100),
    right_elbow__joint=math.radians(90),
    right_wrist_roll_joint=math.radians(113),
)



