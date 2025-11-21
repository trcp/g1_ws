#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()


    # launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')


    # launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) or rosbag clock if true'
    )
    declare_config_path = DeclareLaunchArgument(
        'config_path', default_value=os.path.join(get_package_share_directory('g1_slam'), 'param', 'mid360.yaml'),
        description='Yaml config file path'
    )
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_config_path)
    

    # FAST-LIO を起動する
    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        emulate_tty=True,
        parameters=[
            config_path,
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )
    ld.add_action(fast_lio_node)


    return ld
