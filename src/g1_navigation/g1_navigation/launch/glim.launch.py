#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()


    default_config_path = os.path.join(get_package_share_directory('g1_navigation'), 'config')


    config_path = LaunchConfiguration('config_path')
    use_sim_time = LaunchConfiguration('use_sim_time')


    declare_config_path = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='path to GLIM config directory'
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='use sim time'
    )
    ld.add_action(declare_config_path)
    ld.add_action(declare_use_sim_time)


    glim_node = Node(
        package='glim_ros',
        executable='glim_rosnode',
        emulate_tty=True,
        parameters=[{'config_path': config_path}, {'use_sim_time': use_sim_time}]
    )
    ld.add_action(glim_node)


    return ld
