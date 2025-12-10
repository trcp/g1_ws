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
        'map_server',
        'amcl'
    ]
    remappings = [
        ('/localization/map', '/map'),
        ('/localization/scan', '/scan'),
        ('/localization/initialpose', '/initialpose'),
    ]


    # nodes
    # 自己位置推定
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        emulate_tty=True,
        #respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[
            localization_params,
            {'use_sim_time': use_sim_time},
        ]
    )
    ld.add_action(amcl)

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
