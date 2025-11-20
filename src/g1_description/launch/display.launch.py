#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()

    robot_description = LaunchConfiguration('robot_description')

    # default_values
    default_rviz_path = os.path.join(get_package_share_directory('g1_description'), 'rviz', 'g1.rviz')

    # args
    declare_robot_description = DeclareLaunchArgument(
        'robot_description',
        default_value = os.path.join(
            get_package_share_directory('g1_description'),
            'urdf', 'g1_comp.urdf'
        ),
        description='Full path for robot description.'
    )
    ld.add_action(declare_robot_description)

    # XACRO -> URDF
    robot_description_urdf =  ParameterValue(
        Command([
            'xacro ', robot_description, ' ',
        ]),
        value_type=str
    )

    # Nodes
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_urdf}]
    )
    joint_state_publisher = Node(
        package='erasers_g1_common_cpp',
        executable='joint_state_publisher',
        parameters=[{'have_wheel': have_wheel}]
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', default_rviz_path]
    )
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    ld.add_action(rviz)

    return ld
