#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ld = LaunchDescription()

    # configurations
    default_robot_model = os.path.join(
        get_package_share_directory('erasers_g1_description'),
        'urdf', 'erasers_g1.urdf.xacro'
    )

    robot_model = LaunchConfiguration('robot_model')

    declare_robot_model = DeclareLaunchArgument(
        'robot_model',
        default_value=default_robot_model,
        description='URDF/xacro model path for robot_description'
    )
    ld.add_action(declare_robot_model)

    robot_description_content = ParameterValue(
        Command(['xacro ', robot_model]),
        value_type=str
    )
    params = {'robot_description': robot_description_content}

    # nodes
    # Pinocchio IK
    arm_endeffector_control = Node(
        package='erasers_g1_common',
        executable='arm_endeffector_control',
        parameters=[params],
        output='screen',
        emulate_tty=True
    )

    # Cartesian trajectory planner
    cartesian_trajectory_planner = Node(
        package='erasers_g1_common',
        executable='cartesian_trajectory_planner',
        parameters=[params],
        output='screen',
        emulate_tty=True
    )

    ld.add_action(arm_endeffector_control)
    ld.add_action(cartesian_trajectory_planner)

    return ld