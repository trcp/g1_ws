#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # Paths
    g1_cartographer_prefix = get_package_share_directory('g1_cartographer')
    pointcloud_to_laserscan_config = os.path.join(g1_cartographer_prefix, 'config', 'pointcloud_to_laserscan.yaml')
    nav2_param = os.path.join(get_package_share_directory('g1_navigation'), 'params', 'nav2.yaml')

    # Arguments
    map_yaml_file = LaunchConfiguration('map_yaml_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    
    # Defaults
    default_map_yaml_file = os.path.join(os.environ['HOME'], 'colcon_ws', 'map', 'map.yaml')

    declare_map_yaml_file = DeclareLaunchArgument(
        'map_yaml_file',
        default_value=default_map_yaml_file,
        description='Path to map yaml file'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )

    ld.add_action(declare_map_yaml_file)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_autostart)

    # Lifecycle nodes to manage
    lifecycle_nodes = ['map_server']
    #lifecycle_nodes = ['map_server', 'amcl']

    # Nodes
    
    # Static TF for Livox (if needed for transforms)
    # 2d_cartographer had this. navigation usually needs it if robot_state_publisher doesn't provide it.
    livox_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='mid360_to_livox_frame',
        arguments=['0', '0', '0', '0', '0', '0', 'mid360_link', 'livox_frame'],
        output='screen',
        emulate_tty=True
    )

    # Pointcloud to Laserscan
    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[pointcloud_to_laserscan_config, {'use_sim_time': use_sim_time}],
        remappings=[
            ('cloud_in', '/livox/lidar'),
            ('scan', '/scan')
        ]
    )

    # Map Server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'yaml_filename': map_yaml_file,
            'use_sim_time': use_sim_time
        }]
    )

    # 自己位置推定
    emcl2 = Node(
        package='emcl2',
        executable='emcl2_node',
        name='emcl2',
        emulate_tty=True,
        parameters=[nav2_param],
    )
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        emulate_tty=True,
        parameters=[nav2_param]
    )

    # Lifecycle Manager
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

    ld.add_action(livox_tf_publisher)
    ld.add_action(pointcloud_to_laserscan)
    ld.add_action(map_server)
    ld.add_action(emcl2)
    #ld.add_action(amcl)
    ld.add_action(lifecycle_manager)

    return ld