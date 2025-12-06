#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()


    # configurations
    start_message = LaunchConfiguration('start_message')


    # declare argument
    declare_start_message = DeclareLaunchArgument(
        'start_message', default_value='Hello! erasers_g1 start',
        description='Bringup robot message.'
    )

    ld.add_action(declare_start_message)


    # nodes
    # G1 のモード切り替え（damp, stand_up など）を制御するノード
    loco_service_client = Node(
        package='erasers_g1_common_cpp',
        executable='loco_service_client',
        emulate_tty=True
    )
    # G1 に速度司令を与えるノード
    cmd_vel = Node(
        package='erasers_g1_common_cpp',
        executable='cmd_vel',
        emulate_tty=True
    )
    # G1 の IMU を出力するノード
    imu_publisher = Node(
        package='erasers_g1_common_cpp',
        executable='imu_publisher',
        emulate_tty=True
    )
    # TTS
    audio_client = Node(
        package='erasers_g1_common_cpp',
        executable='audio_client',
        emulate_tty=True
    )

    ld.add_action(loco_service_client)
    ld.add_action(audio_client)
    ld.add_action(imu_publisher)
    ld.add_action(cmd_vel)


    # launchers
    # LiDAR センサーを起動する
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('g1_bringup'),
                'launch', 'mid360.launch.py'
            )
        ])
    )
    # RViz と G1 の URDF, JointState を出力する launcher
    display = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('g1_description'),
                'launch', 'display.launch.py'
            )
        ])
    )

    ld.add_action(lidar)
    ld.add_action(display)


    # 起動時にロボットが発話する
    start_speech_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call',
            '/play_audio',
            'g1_srvs/srv/AudioClient',
            ['{type: 0, text: "', start_message, '", audio_path: ""}']
        ],
        output='screen'
    )
    speech_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=audio_client,
            on_start=[
                TimerAction(
                    period=5.0,
                    actions=[start_speech_cmd]
                )
            ]
        )
    )

    ld.add_action(speech_handler)


    # send launch description to ROS2
    return ld
