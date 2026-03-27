#!/usr/bin/env python3
from launch import LaunchDescription
from launch.events import matches_action
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
    LogInfo,
    IncludeLaunchDescription
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, AndSubstitution, NotSubstitution
from launch.conditions import IfCondition
from launch_ros.descriptions import ParameterFile
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()

    # configurations
    default_g1_cartographer_prefix = get_package_share_directory('g1_bringup')
    pointcloud_to_laserscan_config = os.path.join(default_g1_cartographer_prefix, 'params', 'ptl.yaml')
    default_map_path = os.path.join(os.environ['HOME'], 'colcon_ws', 'map')
    default_map_name = 'map'
    default_save_late = 5000

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_lifecycle = LaunchConfiguration('use_lifecycle')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    map_path = LaunchConfiguration('map_path')
    map_name = LaunchConfiguration('map_name')
    map_save_late = LaunchConfiguration('map_save_late')
    use_navigation = LaunchConfiguration('use_navigation')
    

    # declare argument
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    declare_use_lifecycle = DeclareLaunchArgument(
        'use_lifecycle',
        default_value='false',
        description='Use lifecycle'
    )
    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Autostart'
    )
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(get_package_share_directory('g1_cartographer'), 'params', 'slam_toolbox.yaml'),
        description='slam_toolbox parameters file'
    )
    declare_map_path = DeclareLaunchArgument(
        'map_path', default_value=default_map_path,
        description='Path to save the map'
    )
    declare_map_name = DeclareLaunchArgument(
        'map_name', default_value=default_map_name,
        description='Name of the map'
    )
    declare_map_save_late = DeclareLaunchArgument(
        'map_save_late', default_value=str(default_save_late),
        description='Delay in milliseconds before saving the map'
    )
    declare_use_navigation = DeclareLaunchArgument(
        'use_navigation',
        default_value='false',
        description='Whether to start navigation'
    )
    
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_use_lifecycle)
    ld.add_action(declare_autostart)
    ld.add_action(declare_params_file)
    ld.add_action(declare_map_path)
    ld.add_action(declare_map_name)
    ld.add_action(declare_map_save_late)
    ld.add_action(declare_use_navigation)
    

    # nodes
    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        emulate_tty=True,
        parameters=[
            pointcloud_to_laserscan_config,
            {'use_sim_time': False},
        ],
        remappings=[
            ('cloud_in', '/livox/lidar'),
            ('scan', '/scan'),
        ],
    )

    map_saver = Node(
        package='g1_cartographer',
        executable='auto_map_saver',
        name='auto_map_saver',
        emulate_tty=True,
        output='screen',
        parameters=[
            {'map_path': map_path},
            {'map_name': map_name},
            {'save_late': map_save_late}
        ],
    )

    slam_toolbox = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        emulate_tty=True,
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time},
            {'use_lifecycle': use_lifecycle}
        ],        
    )

    #ld.add_action(pointcloud_to_laserscan)
    ld.add_action(map_saver)
    ld.add_action(slam_toolbox)


    # launchers
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('g1_navigation'),
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


    # sequence / events
    configure_event = EmitEvent(
        event=ChangeState(
          lifecycle_node_matcher=matches_action(slam_toolbox),
          transition_id=Transition.TRANSITION_CONFIGURE
        ),
        condition=IfCondition(AndSubstitution(autostart, NotSubstitution(use_lifecycle)))
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[LifecycleLaunch] Slamtoolbox node is activating."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam_toolbox),
                    transition_id=Transition.TRANSITION_ACTIVATE
                ))
            ]
        ),
        condition=IfCondition(AndSubstitution(autostart, NotSubstitution(use_lifecycle)))
    )

    ld.add_action(configure_event)
    ld.add_action(activate_event)

    return ld
