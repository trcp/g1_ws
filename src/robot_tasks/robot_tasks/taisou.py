#!/usr/bin/env python3
from rclpy.node import Node
import rclpy

from erasers_g1_api.robot_control import G1ArmControl


def main():
    rclpy.init()
    node = Node("taisou")

    arm = G1ArmControl(node)

    arm.move_dual_rel(rx=0.05, lx=0.05)