#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()


    use_sim_time = LaunchConfiguration('use_sim_time')
    dlio_config = LaunchConfiguration('dlio_config')
    dlio_params = LaunchConfiguration('dlio_params')
    rviz_path = LaunchConfiguration('rviz_path')


    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) or rosbag clock if true'
    )
    declare_dlio_config = DeclareLaunchArgument(
        'dlio_config', default_value=os.path.join(get_package_share_directory('g1_slam'), 'param', 'dlio_config_g1imu.yaml'),
        description='Yaml config file path'
    )
    declare_dlio_params = DeclareLaunchArgument(
        'dlio_params', default_value=os.path.join(get_package_share_directory('g1_slam'), 'param', 'dlio_params.yaml'),
        description='Yaml config file path'
    )
    declare_rviz_path = DeclareLaunchArgument(
        'rviz_path', default_value=os.path.join(get_package_share_directory('g1_slam'), 'rviz', 'dlio.rviz'),
        description='RViz config file path'
    )
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_dlio_config)
    ld.add_action(declare_dlio_params)
    ld.add_action(declare_rviz_path)
    

    # DLIO を起動する
    dlio_odom_node = Node(
        package='direct_lidar_inertial_odometry',
        executable='dlio_odom_node',
        emulate_tty=True,
        parameters=[
            dlio_config,
            dlio_params,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('pointcloud', '/livox/lidar'),
            ('imu', '/imu'),
            ('odom', 'dlio/odom_node/odom'),
            ('pose', 'dlio/odom_node/pose'),
            ('path', 'dlio/odom_node/path'),
            ('kf_pose', 'dlio/odom_node/keyframes'),
            ('kf_cloud', 'dlio/odom_node/pointcloud/keyframe'),
            ('deskewed', 'dlio/odom_node/pointcloud/deskewed'),
        ],
        output='screen'
    )
    ld.add_action(dlio_odom_node)
    dlio_map_node = Node(
        package='direct_lidar_inertial_odometry',
        executable='dlio_map_node',
        output='screen',
        emulate_tty=True,
        parameters=[dlio_config, dlio_params],
        remappings=[
            ('keyframes', 'dlio/odom_node/pointcloud/keyframe'),
        ],
    )
    ld.add_action(dlio_map_node)

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        emulate_tty=True,
        arguments=['-d', rviz_path]
    )
    ld.add_action(rviz)


    return ld
