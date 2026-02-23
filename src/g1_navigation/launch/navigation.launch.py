#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # =========================
    # Paths
    # =========================
    g1_cartographer_prefix = get_package_share_directory('g1_cartographer')
    pointcloud_to_laserscan_config = os.path.join(
        g1_cartographer_prefix,
        'config',
        'pointcloud_to_laserscan.yaml'
    )

    nav2_param = os.path.join(
        get_package_share_directory('g1_navigation'),
        'params',
        'nav2.yaml'
    )

    # =========================
    # Arguments
    # =========================
    map_yaml_file = LaunchConfiguration('map_yaml_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    default_map_yaml_file = os.path.join(
        os.environ['HOME'],
        'colcon_ws',
        'map',
        'map_02_23.yaml'
    )

    declare_map_yaml_file = DeclareLaunchArgument(
        'map_yaml_file',
        default_value=default_map_yaml_file,
        description='Path to map yaml file'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false'
    )

    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true'
    )

    ld.add_action(declare_map_yaml_file)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_autostart)

    # =========================
    # Lifecycle nodes（既存を維持）
    # =========================
    # lifecycle_nodes = ['map_server']
    lifecycle_nodes = [
        # 'amcl',
        'map_server',
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]

    # =========================
    # Environment
    # =========================
    ld.add_action(
        SetEnvironmentVariable(
            'RCUTILS_LOGGING_BUFFERED_STREAM',
            '1'
        )
    )

    # =========================
    # Map Server
    # =========================
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_yaml_file,
            'use_sim_time': use_sim_time
        }]
    )

    # =========================
    # Localization（既存）
    # =========================
    emcl2 = Node(
        package='emcl2',
        executable='emcl2_node',
        name='emcl2',
        parameters=[nav2_param],
        arguments=['--ros-args', '--log-level', 'error']
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        emulate_tty=True,
        parameters=[nav2_param]
    )

    # =========================
    # Nav2 Core（追加）
    # =========================
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[nav2_param],
        remappings=[('cmd_vel', 'cmd_vel_nav')]
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        output='screen',
        parameters=[nav2_param]
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        output='screen',
        parameters=[nav2_param]
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        output='screen',
        parameters=[nav2_param]
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        output='screen',
        parameters=[nav2_param]
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        output='screen',
        parameters=[nav2_param]
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        output='screen',
        parameters=[nav2_param],
        remappings=[
            ('cmd_vel', 'cmd_vel_nav'),
            ('cmd_vel_smoothed', 'cmd_vel')
        ]
    )

    # =========================
    # Lifecycle Manager（既存）
    # =========================
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': lifecycle_nodes
        }]
    )

    # =========================
    # Add nodes
    # =========================
    ld.add_action(map_server)
    ld.add_action(emcl2)
    # ld.add_action(amcl)

    ld.add_action(controller_server)
    ld.add_action(planner_server)
    ld.add_action(smoother_server)
    ld.add_action(behavior_server)
    ld.add_action(bt_navigator)
    ld.add_action(waypoint_follower)
    ld.add_action(velocity_smoother)

    ld.add_action(lifecycle_manager)

    return ld
