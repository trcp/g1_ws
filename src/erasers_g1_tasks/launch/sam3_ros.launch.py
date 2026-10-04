#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()
    
    default_params_file = os.path.join(get_package_share_directory('sam3_ros'), 'params', 'demo.yaml')
    default_model_path = os.path.join('/tmp/sam3.pt')

    params_file = LaunchConfiguration('params_file')
    model_path = LaunchConfiguration('model_path')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to the parameter file'
    )
    declare_model_path = DeclareLaunchArgument(
        'model_path',
        default_value=default_model_path,
        description='Path to the model file'
    )
    ld.add_action(declare_params_file)
    ld.add_action(declare_model_path)

    remappings = [
        ('color_image', '/head_camera/d455/color/image_raw'),
        ('depth_image', '/head_camera/d455/aligned_depth_to_color/image_raw'),
        ('color_info', '/head_camera/d455/color/camera_info'),
        ('depth_info', '/head_camera/d455/aligned_depth_to_color/camera_info'),
    ]

    sam3_ros = Node(
        package='sam3_ros',
        executable='sam3_ros',
        name='sam3_ros',
        emulate_tty=True,
        remappings=remappings,
        parameters=[
            params_file,
            {'model_path': model_path},
            {'auto_execute': False},
            #{'qos.reliability': 'BEST_EFFORT'},
        ],
        namespace='sam3',
        output='screen'
    )
    ld.add_action(sam3_ros)


    return ld
