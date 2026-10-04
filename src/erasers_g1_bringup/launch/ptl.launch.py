#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # configurations
    pkg_g1_bringup = get_package_share_directory('erasers_g1_bringup')
    pkg_g1_description = get_package_share_directory('erasers_g1_description')

    default_ptl_params = os.path.join(pkg_g1_bringup, 'params', 'ptl.yaml')
    default_rviz_config = os.path.join(pkg_g1_description, 'rviz', 'g1.rviz')

    # launch configurations
    ptl_params = LaunchConfiguration('ptl_params')
    cloud_in = LaunchConfiguration('cloud_in')
    scan_out = LaunchConfiguration('scan_out')
    use_ptl = LaunchConfiguration('use_ptl')
    use_rviz = LaunchConfiguration('use_rviz')
    use_emc_joy = LaunchConfiguration('use_emc_joy')

    # launch arguments
    declare_ptl_params = DeclareLaunchArgument(
        'ptl_params',
        default_value=default_ptl_params,
        description='Path to pointcloud_to_laserscan configuration file'
    )
    declare_cloud_in = DeclareLaunchArgument(
        'cloud_in',
        default_value='/utlidar/cloud_livox_mid360',
        description='Input point cloud topic for laserscan conversion'
    )
    declare_scan_out = DeclareLaunchArgument(
        'scan_out',
        default_value='/scan',
        description='Output LaserScan topic'
    )
    declare_use_ptl = DeclareLaunchArgument(
        'use_ptl',
        default_value='true',
        description='Whether to start pointcloud_to_laserscan node',
        choices=['true', 'false']
    )
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Whether to start RViz2',
        choices=['true', 'false']
    )
    declare_use_emc_joy = DeclareLaunchArgument(
        'use_emc_joy',
        default_value='true',
        description='Whether to start emergency joy node',
        choices=['true', 'false']
    )

    ld.add_action(declare_ptl_params)
    ld.add_action(declare_cloud_in)
    ld.add_action(declare_scan_out)
    ld.add_action(declare_use_ptl)
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_use_emc_joy)

    # nodes
    ptl_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        emulate_tty=True,
        parameters=[ptl_params],
        remappings=[
            ('cloud_in', cloud_in),
            ('scan', scan_out),
        ],
        condition=IfCondition(use_ptl)
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', default_rviz_config],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(use_rviz)
    )

    emc_joy_node = Node(
        package='joy',
        executable='joy_node',
        namespace='emc',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(use_emc_joy)
    )

    ld.add_action(ptl_node)
    ld.add_action(rviz_node)
    ld.add_action(emc_joy_node)

    return ld

