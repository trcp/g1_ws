#!/usr/bin/env python3
'''
PCD 3D マップを RViz で視覚化する launch ファイル．
'''
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()


    pcd_path = LaunchConfiguration('pcd_path')
    rviz_path = LaunchConfiguration('rviz_path', default=os.path.join(get_package_share_directory('g1_slam'), 'rviz', 'pcd_vis.rviz'))


    declare_pcd_path = DeclareLaunchArgument(
        'pcd_path',
        description='PCD map file full path.'
    )

    ld.add_action(declare_pcd_path)


    pcd_to_pointcloud = Node(
        package='pcl_ros',
        executable='pcd_to_pointcloud',
        emulate_tty=True,
        parameters=[
            {'file_name': pcd_path},
            {'tf_frame': 'pcd_map'},
        ]
    )
    ld.add_action(pcd_to_pointcloud)

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        emulate_tty=True,
        arguments=['-d', rviz_path]
    )
    ld.add_action(rviz)


    return ld
