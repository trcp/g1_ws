#!/usr/bin/env python3
import launch
import launch.actions
import launch.events

import launch_ros
import launch_ros.actions
import launch_ros.events

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node

import lifecycle_msgs.msg

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()
    

    use_sim_time = LaunchConfiguration('use_sim_time')
    pcd_path = LaunchConfiguration('pcd_path')
    localization_param = LaunchConfiguration('localization_param')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) or rosbag clock if true'
    )
    declare_pcd_path = DeclareLaunchArgument(
        'pcd_path',
        description='PCD map file full path.'
    )
    declare_localization_param = DeclareLaunchArgument(
        'localization_param', default_value=os.path.join(get_package_share_directory('g1_slam'), 'param', 'localization.yaml'),
        description='Yaml config file path'
    )

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_pcd_path)
    ld.add_action(declare_localization_param)


    # publish pcd map
    pcd_to_pointcloud = Node(
        package='pcl_ros',
        executable='pcd_to_pointcloud',
        emulate_tty=True,
        parameters=[
            {'file_name': pcd_path},
            {'tf_frame': 'map'},
        ]
    )
    pcd_map_qos_connv = Node(
        package='erasers_g1_common_cpp',
        executable='pcd_map_qos_connv',
        emulate_tty=True
    )
    # odometry 発行
    odom_publisher = Node(
        package='erasers_g1_common_cpp',
        executable='odom_publisher',
        emulate_tty=True
    )
    #ld.add_action(odom_publisher)
    ld.add_action(pcd_to_pointcloud)
    ld.add_action(pcd_map_qos_connv)

    map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        output="screen"
    )
    #ld.add_action(map_to_odom)
    
    # run lidar_localization
    lidar_localization = LifecycleNode(
        name='pcl_localization',
        namespace='',
        emulate_tty=True,
        package='pcl_localization_ros2',
        executable='pcl_localization_node',
        remappings=[
            ('/velodyne_points', '/livox/lidar'),
            #('/imu', '/imu'),
            ('/imu', '/livox/imu'),
            #('/map','/pcd_map_relay')
        ],
        parameters=[localization_param, {'map_path': pcd_path}, {'use_sim_time': use_sim_time}],
        output='screen'
    )
    to_inactive = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(lidar_localization),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )
    
    from_unconfigured_to_inactive = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=lidar_localization, 
            goal_state='unconfigured',
            entities=[
                launch.actions.LogInfo(msg="-- Unconfigured --"),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(lidar_localization),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                )),
            ],
        )
    )

    from_inactive_to_active = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=lidar_localization, 
            start_state = 'configuring',
            goal_state='inactive',
            entities=[
                launch.actions.LogInfo(msg="-- Inactive --"),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(lidar_localization),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    ld.add_action(from_unconfigured_to_inactive)
    ld.add_action(from_inactive_to_active)
    ld.add_action(lidar_localization)
    ld.add_action(to_inactive)
    

    return ld
