#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()


    # configures
    device = LaunchConfiguration('device')
    

    # arguments
    declare_device = DeclareLaunchArgument(
        'device', default_value='cuda',
        description='use device cpu or cuda'
    )

    ld.add_action(declare_device)


    # remappings
    remappings = [
        ('image_color', '/head_camera/d455/color/image_raw'),
        ('image_depth', '/head_camera/d455/aligned_depth_to_color/image_raw'),
        ('color_camera_info', '/head_camera/d455/color/camera_info'),
        ('depth_camera_info', '/head_camera/d455/aligned_depth_to_color/camera_info'),
        #('execute', 'execute_person_detect')
    ]


    # nodes
    lightweight_openpose_ros2 = Node(
        package='lightweight_openpose_ros2',
        executable='lightweight_openpose_ros2',
        emulate_tty=True,
        parameters=[
            {'device': device}
        ],
        remappings=remappings
    )
    lor_transformer = Node(
        package='lor_transformer',
        executable='lor_transformer',
        emulate_tty=True,
        parameters=[
            {'target_frame': 'd455_link'}
        ],
        remappings=remappings
    )

    ld.add_action(lightweight_openpose_ros2)
    ld.add_action(lor_transformer)


    return ld