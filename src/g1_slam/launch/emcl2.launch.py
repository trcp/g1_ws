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


    # Launch configurations
    map = LaunchConfiguration('map')
    localization_params = LaunchConfiguration('localization_params')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')


    # declare arguments
    declare_map = DeclareLaunchArgument(
        'map', default_value=os.path.join(os.environ['PWD']),
        description='Full path for map YAML'
    )
    declare_localization_params = DeclareLaunchArgument(
        'localization_params', default_value=os.path.join(get_package_share_directory('g1_slam'), 'param', 'localization.yaml'),
        description='Full path for 2d png map'
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Enable show Localization Rviz'
    )
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Enable auto bringup Nav2 Stack.'
    )

    ld.add_action(declare_map)
    ld.add_action(declare_localization_params)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_autostart)

    
    lifecycle_nodes = [
        'map_server'
    ]
    remappings = [
        ('/localization/map', '/map'),
        ('/localization/scan', '/scan'),
        ('/localization/initialpose', '/initialpose'),
    ]


    # nodes
    # odometry 発行
    odom_publisher = Node(
        package='erasers_g1_common_cpp',
        executable='odom_publisher',
        emulate_tty=True
    )
    ld.add_action(odom_publisher)

    map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        output="screen"
    )
    ld.add_action(map_to_odom)

    # laserscan 発行
    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan_node',
        emulate_tty=True,
        parameters=[
            localization_params,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('cloud_in', '/livox/lidar'),
            ('scan', '/scan'),
        ],
    )
    ld.add_action(pointcloud_to_laserscan)
    
    # 自己位置推定
    emcl2 = Node(
        package='emcl2',
        executable='emcl2_node',
        name='emcl2',
        emulate_tty=True,
        parameters=[localization_params],
        remappings=[
            ('/scan', '/scan_reliable')
        ]
    )
    ld.add_action(emcl2)

    # マップ読み込み
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output='screen',
        emulate_tty=True,
        parameters=[
            localization_params,
            {'yaml_filename': map}
        ],
        remappings=remappings,
    )
    lifecycle_manager =  Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes}],
        emulate_tty=True,
    )
    ld.add_action(map_server)
    ld.add_action(lifecycle_manager)


    return ld
