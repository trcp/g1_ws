#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params_file = os.path.join(
        get_package_share_directory('g1_bringup'),
        'params',
        'd455.yaml',
    )

    camera_params = LaunchConfiguration('camera_params')
    camera_namespace = LaunchConfiguration('camera_namespace')
    camera_name = LaunchConfiguration('camera_name')

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_params',
            default_value=default_params_file,
            description='RealSense parameter file. Defaults to g1_bringup d455.yaml.',
        ),
        DeclareLaunchArgument(
            'camera_namespace',
            default_value='head_camera',
            description='Namespace used by yolo_human_node topic names.',
        ),
        DeclareLaunchArgument(
            'camera_name',
            default_value='d455',
            description='Camera node name used by yolo_human_node topic names.',
        ),
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            namespace=camera_namespace,
            name=camera_name,
            output='screen',
            emulate_tty=True,
            parameters=[
                camera_params,
                {
                    'camera_name': camera_name,
                    'enable_color': True,
                    'enable_depth': True,
                    'align_depth.enable': True,
                },
            ],
        ),
    ])
