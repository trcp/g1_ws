#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    remappings = [
        ('/color_image', '/head_camera/d455/color/image_raw'),
        ('/depth_image', '/head_camera/d455/aligned_depth_to_color/image_raw'),
        ('/color_camera_info', '/head_camera/d455/color/camera_info'),
        ('/depth_camera_info', '/head_camera/d455/aligned_depth_to_color/camera_info'),
    ]

    person_pose = Node(
        package='nakalab_ultralytics_ros2',
        executable='person_pose',
        emulate_tty=True,
        remappings=remappings,
    )
    person_pose_3d = Node(
        package='nakalab_ultralytics_cpp',
        executable='person_pose_3d',
        emulate_tty=True,
        remappings=remappings,
    )

    ld.add_action(person_pose)
    ld.add_action(person_pose_3d)

    return ld
