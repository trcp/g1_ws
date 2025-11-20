#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()


    # nodes
    # G1 のモード切り替え（damp, stand_up など）を制御するノード
    loco_service_client = Node(
        package='erasers_g1_common_cpp',
        executable='loco_service_client',
        emulate_tty=True
    )
    # G1 に速度司令を与えるノード
    cmd_vel = Node(
        package='erasers_g1_common_cpp',
        executable='cmd_vel',
        emulate_tty=True
    )

    ld.add_action(loco_service_client)
    ld.add_action(cmd_vel)


    # launchers
    # LiDAR センサーを起動する
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('g1_bringup'),
                'launch', 'mid360.launch.py'
            )
        ])
    )
    # RViz と G1 の URDF, JointState を出力する launcher
    display = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('g1_description'),
                'launch', 'display.launch.py'
            )
        ])
    )

    ld.add_action(lidar)
    ld.add_action(display)


    # send launch description to ROS2
    return ld
