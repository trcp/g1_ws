#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import Command


def generate_launch_description():
    ld = LaunchDescription()


    # configurations
    xacro_file = os.path.join(
        get_package_share_directory('g1_description'),
        'urdf', 'erasers_g1.urdf.xacro'
    )
    robot_description_content = Command(['xacro ', xacro_file])
    params = {'robot_description': robot_description_content}


    # nodes
    # Pinoccio IK
    arm_endeffector_control = Node(
        package='erasers_g1_common_cpp',
        executable='arm_endeffector_control',
        parameters=[params],
        emulate_tty=True
    )

    # Cartesian trajectory planner
    cartesian_trajectory_planner = Node(
        package='erasers_g1_common_cpp',
        executable='cartesian_trajectory_planner',
        parameters=[params],
        emulate_tty=True
    )


    # launchers
    ld.add_action(arm_endeffector_control)
    ld.add_action(cartesian_trajectory_planner)


    return ld