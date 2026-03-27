#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

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
    use_map_server = LaunchConfiguration('use_map_server')
    use_localization = LaunchConfiguration('use_localization')

    default_map_yaml_file = os.path.join(
        os.environ['HOME'],
        'colcon_ws',
        'map',
        'map.yaml'
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

    declare_use_map_server = DeclareLaunchArgument(
        'use_map_server',
        default_value='true',
        description='Whether to start map_server'
    )

    declare_use_localization = DeclareLaunchArgument(
        'use_localization',
        default_value='true',
        description='Whether to start localization nodes (emcl2)'
    )

    ld.add_action(declare_map_yaml_file)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_autostart)
    ld.add_action(declare_use_map_server)
    ld.add_action(declare_use_localization)

    # =========================
    # Environment
    # =========================
    ld.add_action(
        SetEnvironmentVariable(
            'RCUTILS_LOGGING_BUFFERED_STREAM',
            '1'
        )
    )

    def launch_setup(context, *args, **kwargs):
        _use_map_server = use_map_server.perform(context).lower() == 'true'
        _use_localization = use_localization.perform(context).lower() == 'true'

        # =========================
        # Lifecycle nodes
        # =========================
        lifecycle_nodes = [
            'controller_server',
            'smoother_server',
            'planner_server',
            'behavior_server',
            'bt_navigator',
            'waypoint_follower',
            'velocity_smoother',
        ]

        if _use_map_server:
            lifecycle_nodes.append('map_server')

        # =========================
        # Map Server
        # =========================
        map_server = Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'yaml_filename': map_yaml_file,
                'use_sim_time': use_sim_time
            }],
            condition=IfCondition(use_map_server)
        )

        # =========================
        # Localization
        # =========================
        emcl2 = Node(
            package='emcl2',
            executable='emcl2_node',
            name='emcl2',
            emulate_tty=True,
            parameters=[nav2_param],
            arguments=['--ros-args', '--log-level', 'error'],
            condition=IfCondition(use_localization)
        )

        # =========================
        # Nav2 Core
        # =========================
        controller_server = Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            emulate_tty=True,
            parameters=[nav2_param],
            remappings=[('/cmd_vel', '/cmd_vel_nav')]
        )

        planner_server = Node(
            package='nav2_planner',
            executable='planner_server',
            output='screen',
            emulate_tty=True,
            parameters=[nav2_param]
        )

        smoother_server = Node(
            package='nav2_smoother',
            executable='smoother_server',
            output='screen',
            emulate_tty=True,
            parameters=[nav2_param]
        )

        behavior_server = Node(
            package='nav2_behaviors',
            executable='behavior_server',
            output='screen',
            emulate_tty=True,
            parameters=[nav2_param],
            remappings=[('/cmd_vel', '/cmd_vel_nav')]
        )

        bt_navigator = Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            output='screen',
            emulate_tty=True,
            parameters=[nav2_param]
        )

        waypoint_follower = Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            output='screen',
            emulate_tty=True,
            parameters=[nav2_param]
        )

        velocity_smoother = Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            output='screen',
            emulate_tty=True,
            parameters=[nav2_param],
            remappings=[
                ('/cmd_vel', '/cmd_vel_nav'),
                ('/cmd_vel_smoothed', '/cmd_vel')
            ]
        )

        # =========================
        # Lifecycle Manager
        # =========================
        lifecycle_manager = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': lifecycle_nodes
            }]
        )

        return [
            map_server,
            emcl2,
            controller_server,
            planner_server,
            smoother_server,
            behavior_server,
            bt_navigator,
            waypoint_follower,
            velocity_smoother,
            lifecycle_manager
        ]

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
