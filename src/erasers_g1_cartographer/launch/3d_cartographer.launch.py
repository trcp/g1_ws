#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # default values
    default_g1_cartographer_prefix = get_package_share_directory('erasers_g1_cartographer')
    default_cartographer_config_dir = os.path.join(default_g1_cartographer_prefix, 'config')
    default_configuration_basename = 'g1_3d.lua'
    default_map_path = os.path.join(os.path.expanduser('~'), 'colcon_ws', 'map')
    default_map_name = 'map'
    default_save_late = 5000

    # configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    resolution = LaunchConfiguration('resolution')
    publish_period_sec = LaunchConfiguration('publish_period_sec')
    map_path = LaunchConfiguration('map_path')
    map_name = LaunchConfiguration('map_name')
    save_late = LaunchConfiguration('save_late')
    autostart = LaunchConfiguration('autostart')
    use_navigation = LaunchConfiguration('use_navigation')
    points2_topic = LaunchConfiguration('points2_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    use_auto_map_saver = LaunchConfiguration('use_auto_map_saver')

    # declare arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true',
        choices=['true', 'false']
    )
    declare_resolution = DeclareLaunchArgument(
        'resolution', default_value='0.05',
        description='Resolution of a grid cell in the mapped occupancy grid'
    )
    declare_publish_period_sec = DeclareLaunchArgument(
        'publish_period_sec', default_value='1.0',
        description='OccupancyGrid publishing period'
    )
    declare_map_path = DeclareLaunchArgument(
        'map_path', default_value=default_map_path,
        description='Path to save the map'
    )
    declare_map_name = DeclareLaunchArgument(
        'map_name', default_value=default_map_name,
        description='Name of the map'
    )
    declare_save_late = DeclareLaunchArgument(
        'save_late', default_value=str(default_save_late),
        description='Delay in milliseconds before saving the map'
    )
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Autostart',
        choices=['true', 'false']
    )
    declare_use_navigation = DeclareLaunchArgument(
        'use_navigation', default_value='false',
        description='Whether to start navigation',
        choices=['true', 'false']
    )
    declare_points2_topic = DeclareLaunchArgument(
        'points2_topic', default_value='/utlidar/cloud_livox_mid360',
        description='Input PointCloud2 topic for 3D cartographer'
    )
    declare_imu_topic = DeclareLaunchArgument(
        'imu_topic', default_value='/utlidar/imu_livox_mid360',
        description='Input IMU topic for 3D cartographer'
    )
    declare_use_auto_map_saver = DeclareLaunchArgument(
        'use_auto_map_saver', default_value='false',
        description='Whether to start auto_map_saver node',
        choices=['true', 'false']
    )

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_resolution)
    ld.add_action(declare_publish_period_sec)
    ld.add_action(declare_map_path)
    ld.add_action(declare_map_name)
    ld.add_action(declare_save_late)
    ld.add_action(declare_autostart)
    ld.add_action(declare_use_navigation)
    ld.add_action(declare_points2_topic)
    ld.add_action(declare_imu_topic)
    ld.add_action(declare_use_auto_map_saver)

    # Cartographer Nodes
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        arguments=[
            '-configuration_directory', default_cartographer_config_dir,
            '-configuration_basename', default_configuration_basename,
        ],
        remappings=[
            ('points2', points2_topic),
            ('imu', imu_topic),
            ('odom', '/odom'),
        ]
    )

    # 3DマップからNav2向けの2D Occupancy Gridを生成
    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': use_sim_time,
            'resolution': resolution,
            'publish_period_sec': publish_period_sec,
        }],
    )

    auto_map_saver = Node(
        package='erasers_g1_cartographer',
        executable='auto_map_saver',
        name='auto_map_saver',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'map_path': map_path,
            'map_name': map_name,
            'save_late': save_late,
        }],
        condition=IfCondition(use_auto_map_saver)
    )

    ld.add_action(cartographer_node)
    ld.add_action(cartographer_occupancy_grid_node)
    ld.add_action(auto_map_saver)

    # Navigation launcher
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('erasers_g1_navigation'),
                'launch', 'navigation.launch.py'
            )
        ]),
        launch_arguments={
            'use_map_server': 'false',
            'use_localization': 'false',
            'use_sim_time': use_sim_time,
            'autostart': autostart,
        }.items(),
        condition=IfCondition(use_navigation)
    )

    ld.add_action(navigation)

    return ld

