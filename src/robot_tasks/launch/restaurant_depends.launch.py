#!/usr/bin/env python3
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from ament_index_python.packages import get_package_share_directory
# import os


# def generate_launch_description():
#     ld = LaunchDescription()


#     default_plefix_path = get_package_share_directory('robot_tasks') + '/launch/'


#     lor_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(os.path.join(default_plefix_path, 'lightweight_openpose.launch.py'))
#     )
#     sam3_roslaunch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(os.path.join(default_plefix_path, 'sam3_ros.launch.py'))
#     )
#     ld.add_action(lor_launch)
#     ld.add_action(sam3_roslaunch)


#     return ld


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
        parameters=[{'run_detect':True}]
    )
    person_pose_3d = Node(
        package='nakalab_ultralytics_cpp',
        executable='person_pose_3d',
        emulate_tty=True,
        remappings=remappings,
        parameters=[{'ref_frame': 'map'}],
    )

    ld.add_action(person_pose)
    ld.add_action(person_pose_3d)

    return ld
