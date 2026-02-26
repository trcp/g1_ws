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
    ptl_params = LaunchConfiguration('ptl_params')
    camera_params = LaunchConfiguration('camera_params')
    start_message = LaunchConfiguration('start_message')
    use_rviz = LaunchConfiguration('use_rviz')


    # declare argument
    declare_start_message = DeclareLaunchArgument(
        'start_message', default_value='Hello! erasers_g1 start',
        description='Bringup robot message.'
    )
    declare_ptl_params = DeclareLaunchArgument(
        'ptl_params', default_value=os.path.join(get_package_share_directory('g1_bringup'), 'params', 'ptl.yaml'),
        description='Full path for 2d png map'
    )
    declare_camera_params = DeclareLaunchArgument(
        'camera_params', default_value=os.path.join(get_package_share_directory('g1_bringup'), 'params', 'd455.yaml'),
        description='Full path for 2d png map'
    )
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='false',
        description='Whether to start Rviz'
    )

    ld.add_action(declare_start_message)
    ld.add_action(declare_ptl_params)
    ld.add_action(declare_camera_params)
    ld.add_action(declare_use_rviz)


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
    # G1 の Odometry を出力するノード
    odom_publisher = Node(
        package='erasers_g1_common_cpp',
        executable='odom_publisher',
        emulate_tty=True
    )
    # 頭部ジョイントの起動
    head_joints = Node(
        package='head_servo_controller',
        executable='head_servo_controller',
        emulate_tty=True
    )
    # TTS
    audio_client = Node(
        package='erasers_g1_common_cpp',
        executable='audio_client',
        emulate_tty=True
    )
    # ジョイント制御
    arm_joint_control = Node(
        package='erasers_g1_common_cpp',
        executable='arm_joint_control',
        emulate_tty=True
    )
    # Pinoccio IK
    arm_endeffector_control = Node(
        package='erasers_g1_common_cpp',
        executable='arm_endeffector_control',
        emulate_tty=True
    )
    # Cartesian trajectory planner
    cartesian_trajectory_planner = Node(
        package='erasers_g1_common_cpp',
        executable='cartesian_trajectory_planner',
        emulate_tty=True
    )
    # Amazing Hand
    amazing_hand = Node(
        package='amazing_hand_nodes',
        executable='amazing_hand_node',
        emulate_tty=True
    )
    # 緊急停止
    emergency_stop = Node(
        package='erasers_g1_common_cpp',
        executable='emergency_stop',
        emulate_tty=True
    )

    ld.add_action(loco_service_client)
    ld.add_action(audio_client)
    ld.add_action(imu_publisher)
    ld.add_action(odom_publisher)
    ld.add_action(cmd_vel)
    ld.add_action(head_joints)
    ld.add_action(arm_joint_control)
    ld.add_action(arm_endeffector_control)
    ld.add_action(cartesian_trajectory_planner)
    ld.add_action(amazing_hand)
    ld.add_action(emergency_stop)


    # TF: base_link -> pelvis
    # base_link_to_pelvis =  Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     arguments=["0", "0", "0", "0", "0", "0", "base_link", "pelvis"],
    #     output="screen"
    # )

    # ld.add_action(base_link_to_pelvis)


    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        emulate_tty=True,
        parameters=[
            ptl_params,
            {'use_sim_time': False},
        ],
        remappings=[
            ('cloud_in', '/livox/lidar'),
            ('scan', '/scan'),
        ],
    )
    #ld.add_action(pointcloud_to_laserscan)



    # launchers
    # 頭部カメラの起動
    head_camera = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='d455',
        parameters=[
            camera_params,
            {'camera_name': 'd455'}
        ],
        output='screen',
        namespace='head_camera',
        emulate_tty=True,
    )
    head_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch', 'rs_launch.py'
            )
        ]),
        launch_arguments={
            'camera_name': 'd455',
            'camera_namespace': 'head_camera',
            'rgb_camera.profile': "640,480,15",
            'depth_module.profile': "640,480,15",
            'pointcloud.enable': 'true',
        }.items()
    )
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
        ]),
        launch_arguments={
            'use_rviz': use_rviz,
        }.items()
    )

    #ld.add_action(head_camera)
    #ld.add_action(head_camera_launch)
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

    # コマンド経由で d455 を起動
    launch_d455_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'launch',
            'realsense2_camera',
            'rs_launch.py',
            'camera_name:=d455',
            'camera_namespace:=head_camera',
            'rgb_camera.color_profile:=640x480x30',
            'depth_module.depth_profile:=640x480x30',
            'pointcloud.enable:=true',
        ],
        output='screen'
    )
    launch_d455_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=head_joints,
            on_start=[
                TimerAction(
                    period=1.0,
                    actions=[launch_d455_cmd]
                )
            ]
        )
    )

    #ld.add_action(launch_d455_handler)



    # 起動時に頭部カメラ初期位置に戻す
    start_init_head_cam_pose_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call',
            '/move_servo',
            'g1_srvs/srv/MoveServo',
            ['{pan: 0.0, tilt: 0.0}']
        ],
        output='screen'
    )
    init_head_cam_pose_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=audio_client,
            on_start=[
                TimerAction(
                    period=5.0,
                    actions=[start_init_head_cam_pose_cmd]
                )
            ]
        )
    )

    ld.add_action(init_head_cam_pose_handler)


    # send launch description to ROS2
    return ld
