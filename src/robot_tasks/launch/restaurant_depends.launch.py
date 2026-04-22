#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()


    default_plefix_path = get_package_share_directory('robot_tasks') + '/launch/'


    lor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(default_plefix_path, 'lightweight_openpose.launch.py'))
    )
    sam3_roslaunch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(default_plefix_path, 'sam3_ros.launch.py'))
    )
    ld.add_action(lor_launch)
    ld.add_action(sam3_roslaunch)


    return ld