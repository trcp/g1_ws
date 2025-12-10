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

    navigation_params = LaunchConfiguration('navigation_params')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    map = LaunchConfiguration('map')
    use_respawn = True

    declare_map = DeclareLaunchArgument(
        'map', default_value=os.path.join(os.environ['PWD']),
        description='Full path for map YAML'
    )
    declare_navigation_params = DeclareLaunchArgument(
        'navigation_params', default_value=os.path.join(get_package_share_directory('g1_slam'), 'param', 'navigation.yaml'),
        description='Full path for 2d png map'
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Enable show navigation Rviz'
    )
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Enable auto bringup Nav2 Stack.'
    )

    ld.add_action(declare_map)
    ld.add_action(declare_navigation_params)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_autostart)

    lifecycle_nodes = [
        'map_server',           
        'amcl',                 
        'controller_server',    
        'smoother_server',
        'planner_server',
        #'route_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
        #'collision_monitor',
        #'docking_server',
    ]
    
    remappings = [
        ('/cmd_vel_smoothed', '/cmd_vel')
    ]

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
            navigation_params,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('cloud_in', '/livox/lidar'),
            ('scan', '/scan'),
        ],
    )
    ld.add_action(pointcloud_to_laserscan)

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
            navigation_params,
            {'use_sim_time': use_sim_time},
        ]
    )
    # マップ読み込み
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output='screen',
        emulate_tty=True,
        parameters=[
            navigation_params,
            {'yaml_filename': map}
        ],
        remappings=remappings,
    )
    
    node_controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[navigation_params],
        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
        emulate_tty=True,
    )
    node_smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[navigation_params],
        remappings=remappings,
        emulate_tty=True,
    )
    node_planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[navigation_params],
        remappings=remappings,
        emulate_tty=True,
    )
    node_route_server = Node(
        package='nav2_route',
        executable='route_server',
        name='route_server',
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[navigation_params],
        remappings=remappings,
        emulate_tty=True,
    )
    node_behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[navigation_params],
        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
        emulate_tty=True,
    )
    node_bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[navigation_params],
        remappings=remappings,
        emulate_tty=True,
        
    )
    node_waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[navigation_params],
        remappings=remappings,
        emulate_tty=True,
    )
    node_velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[navigation_params],
        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
        emulate_tty=True,
    )
    node_collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[navigation_params],
        remappings=remappings,
        emulate_tty=True,
    )
    node_docking_server = Node(
        package='opennav_docking',
        executable='opennav_docking',
        name='docking_server',
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[navigation_params],
        remappings=remappings,
        emulate_tty=True,
    )
    lifecycle_manager =  Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes}],
        emulate_tty=True,
    )
    
    ld.add_action(map_server)
    ld.add_action(amcl)
    ld.add_action(node_controller_server)
    ld.add_action(node_smoother_server)
    ld.add_action(node_planner_server)
    # ld.add_action(node_route_server)
    ld.add_action(node_behavior_server)
    ld.add_action(node_bt_navigator)
    ld.add_action(node_waypoint_follower)
    ld.add_action(node_velocity_smoother)
    #ld.add_action(node_collision_monitor)
    # ld.add_action(node_docking_server)
    ld.add_action(lifecycle_manager)

    return ld
