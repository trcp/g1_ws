#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()


    # default value
    lua_dir = os.path.join(get_package_share_directory('g1_slam'), 'config')
    lua_name = 'cartographer.lua'
    rviz_path = os.path.join(get_package_share_directory('g1_slam'), 'rviz', 'cartographer.rviz')


    # configs
    localization_params = LaunchConfiguration('localization_params')
    resolution = LaunchConfiguration('resolution')
    publish_period_sec = LaunchConfiguration('publish_period_sec')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_name = LaunchConfiguration('map_name')
    map_path = LaunchConfiguration('map_path')
    map_save_late = LaunchConfiguration('map_save_late')


    # args
    declare_localization_params = DeclareLaunchArgument(
        'localization_params', default_value=os.path.join(get_package_share_directory('g1_slam'), 'param', 'localization.yaml'),
        description='Full path for 2d png map'
    )
    declare_resolution = DeclareLaunchArgument(
        'resolution', default_value='0.025',
        description='Map resolution [m]'
    )
    declare_publish_period_sec = DeclareLaunchArgument(
        'publish_period_sec', default_value='1.0',
        description='Map publish late [s]'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Enable show Localization Rviz'
    )
    declare_map_path = DeclareLaunchArgument(
        'map_path', default_value=os.path.join(os.environ['HOME'], 'colcon_ws', 'map'),
        description='Name of save map'
    )
    declare_map_name = DeclareLaunchArgument(
        'map_name', default_value='test',
        description='Name of save map'
    )
    declare_map_save_late = DeclareLaunchArgument(
        'map_save_late', default_value='5',
        description='Late of save map. [int]'
    )

    ld.add_action(declare_localization_params)
    ld.add_action(declare_resolution)
    ld.add_action(declare_publish_period_sec)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_map_path)
    ld.add_action(declare_map_name)
    ld.add_action(declare_map_save_late)


    map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        output="screen"
    )
    #ld.add_action(map_to_odom)
    
    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan_node',
        emulate_tty=True,
        parameters=[localization_params, {'use_sim_time': use_sim_time}],
        remappings=[
            ('cloud_in', '/livox/lidar'),
            ('scan', '/scan'),
        ],
    )
    ld.add_action(pointcloud_to_laserscan)

    # odometry 発行
    odom_publisher = Node(
        package='erasers_g1_common_cpp',
        executable='odom_publisher',
        emulate_tty=True
    )
    ld.add_action(odom_publisher)

    # cartographer
    node_cartographer = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        emulate_tty=True,
        arguments=[
            '-configuration_directory', lua_dir,
            '-configuration_basename', lua_name,
        ],
        remappings=[
            ('/points2', '/livox/lidar')
        ]
    )
    node_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        emulate_tty=True,
        arguments=[
            '-resolution', resolution,
            '-publish_period_sec', publish_period_sec,
        ],
    )
    ld.add_action(node_cartographer)
    ld.add_action(node_occupancy_grid_node)

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        emulate_tty=True,
        arguments=['-d', rviz_path]
    )
    ld.add_action(rviz)

    # map saver
    auto_map_saver = Node(
        package='g1_slam',
        executable='map_saver',
        emulate_tty=True,
        output='screen',
        parameters=[
            {'map_path': map_path},
            {'map_name': map_name},
            {'save_late': map_save_late}
        ],
    )
    ld.add_action(auto_map_saver)


    return ld
