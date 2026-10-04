#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()


    # default variables
    pkg_share_dir = get_package_share_directory('erasers_g1_bringup')
    erasers_g1_common_pkg_share_dir = get_package_share_directory('erasers_g1_common')
    erasers_g1_description_pkg_share_dir = get_package_share_directory('erasers_g1_description')
    erasers_g1_head_servo_controller_pkg_share_dir = get_package_share_directory(
        'erasers_g1_head_servo_controller')
    rosbridge_server_pkg_share_dir = get_package_share_directory('rosbridge_server')
    default_voicevox_root = os.path.expanduser('~/colcon_ws/voicevox')
    default_robot_model = os.path.join(
        erasers_g1_description_pkg_share_dir, 'urdf', 'erasers_g1.urdf')
    default_camera_params = os.path.join(pkg_share_dir, 'params', 'd455.yaml')
    default_head_servo_params = os.path.join(
        erasers_g1_head_servo_controller_pkg_share_dir, 'params', 'head_servo.yaml')
    default_ptl_params = os.path.join(pkg_share_dir, 'params', 'ptl.yaml')
    default_vui_client_params = os.path.join(
        erasers_g1_common_pkg_share_dir, 'config', 'vui_client.yaml')
    default_robot_controller_params = os.path.join(
        erasers_g1_common_pkg_share_dir, 'config', 'robot_controller.yaml')
    default_display_launch = os.path.join(
        erasers_g1_description_pkg_share_dir, 'launch', 'display.launch.py')
    default_rosbridge_launch = os.path.join(
        rosbridge_server_pkg_share_dir, 'launch', 'rosbridge_websocket_launch.xml')
    default_use_emc = os.environ.get('USE_EMC', 'false').lower().strip('\'"')
    default_use_head_camera = os.environ.get('USE_HEAD_CAMERA', 'false').lower().strip('\'"')
    default_use_amazing_hand = os.environ.get('USE_AMAZING_HAND', 'false').lower().strip('\'"')
    default_ah_path = os.environ.get('AH_PATH', '/dev/ttyACM0').strip('\'"')
    default_dx_path = os.environ.get('DX_PATH', '/dev/ttyUSB0').strip('\'"')
    default_voicevox_onnxruntime_path = os.path.join(
        default_voicevox_root, 'onnxruntime', 'lib', '')
    default_voicevox_model_path = os.path.join(default_voicevox_root, 'vvm', '8.vvm')
    default_open_jtalk_dict_dir = os.path.join(
        default_voicevox_root, 'dict', 'open_jtalk_dic_utf_8-1.11')


    # launch configurations
    use_head_camera = LaunchConfiguration('use_head_camera')
    use_amazing_hand = LaunchConfiguration('use_amazing_hand')
    use_emc = LaunchConfiguration('use_emc')
    robot_model = LaunchConfiguration('robot_model')
    camera_params = LaunchConfiguration('camera_params')
    ptl_params = LaunchConfiguration('ptl_params')
    ah_path = LaunchConfiguration('ah_path')
    dx_path = LaunchConfiguration('dx_path')
    device = LaunchConfiguration('device')
    mic_network_interface = LaunchConfiguration('mic_network_interface')


    # launch arguments
    declare_use_head_camera = DeclareLaunchArgument(
        'use_head_camera',
        default_value=default_use_head_camera,
        description='頭部カメラとサーボを有効にする',
        choices=['true', 'false'],
    )
    declare_use_amazing_hand = DeclareLaunchArgument(
        'use_amazing_hand',
        default_value=default_use_amazing_hand,
        description='amazing_hand を有効にする',
        choices=['true', 'false'],
    )
    declare_use_emc = DeclareLaunchArgument(
        'use_emc',
        default_value=default_use_emc,
        description='駆動系有効時に緊急停止用 Joy を起動する',
        choices=['true', 'false'],
    )
    declare_robot_model = DeclareLaunchArgument(
        'robot_model',
        default_value=default_robot_model,
        description='URDF のパス',
    )
    declare_camera_params = DeclareLaunchArgument(
        'camera_params',
        default_value=default_camera_params,
        description='カメラ設定',
    )
    declare_ptl_params = DeclareLaunchArgument(
        'ptl_params',
        default_value=default_ptl_params,
        description='点群変換設定',
    )
    declare_ah_path = DeclareLaunchArgument(
        'ah_path',
        default_value=default_ah_path,
        description='ハンドの接続先',
    )
    declare_dx_path = DeclareLaunchArgument(
        'dx_path',
        default_value=default_dx_path,
        description='頭部サーボの接続先',
    )
    declare_device = DeclareLaunchArgument(
        'device',
        default_value='cuda',
        description='Whisper の推論デバイス',
        choices=['cpu', 'cuda'],
    )
    declare_mic_network_interface = DeclareLaunchArgument(
        'mic_network_interface',
        default_value='',
        description='マイクの UDP 受信用 NIC。録音開始時に使用する',
    )
    ld.add_action(declare_use_head_camera)
    ld.add_action(declare_use_amazing_hand)
    ld.add_action(declare_use_emc)
    ld.add_action(declare_robot_model)
    ld.add_action(declare_camera_params)
    ld.add_action(declare_ptl_params)
    ld.add_action(declare_ah_path)
    ld.add_action(declare_dx_path)
    ld.add_action(declare_device)
    ld.add_action(declare_mic_network_interface)


    # include launch
    include_display = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(default_display_launch),
        launch_arguments={
            'use_rviz': 'false',
            'robot_description': robot_model,
        }.items(),
    )
    # rosbridge の既存 XML Launch を読み込むため、対応するローダーを維持する。
    include_rosbridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(default_rosbridge_launch),
    )
    ld.add_action(include_display)
    ld.add_action(include_rosbridge)


    # actions
    log_start_action = LogInfo(msg='Starting erasers_g1 bringup system...')
    ld.add_action(log_start_action)


    # nodes
    head_camera = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='d455',
        namespace='head_camera',
        parameters=[camera_params, {'camera_name': 'd455'}],
        condition=IfCondition(use_head_camera),
        output='screen',
        emulate_tty=True,
    )
    head_servo = Node(
        package='erasers_g1_head_servo_controller',
        executable='head_servo_node',
        parameters=[
            default_head_servo_params,
            {'dx_path': dx_path},
        ],
        condition=IfCondition(use_head_camera),
        output='screen',
        emulate_tty=True,
    )
    hand = Node(
        package='amazing_hand_nodes',
        executable='amazing_hand_node',
        parameters=[{'serial_port': ah_path}],
        condition=IfCondition(use_amazing_hand),
        output='screen',
        emulate_tty=True,
    )
    vui_client = Node(
        package='erasers_g1_common',
        executable='vui_client',
        parameters=[
            default_vui_client_params,
            {'mic_network_interface': mic_network_interface},
        ],
        output='screen',
        emulate_tty=True,
    )
    audio_client = Node(
        package='erasers_g1_common',
        executable='audio_client',
        output='screen',
        emulate_tty=True,
    )
    mic_server = Node(
        package='erasers_g1_common',
        executable='mic_server',
        output='screen',
        emulate_tty=True,
    )
    voicevox = Node(
        package='voicevox_ros2',
        executable='voicevox_ros2',
        name='voicevox_ros2',
        parameters=[{
            'voicevox_onnxruntime_path': default_voicevox_onnxruntime_path,
            'voicevox_model_path': default_voicevox_model_path,
            'open_jtalk_dict_dir': default_open_jtalk_dict_dir,
        }],
        output='screen',
        emulate_tty=True,
    )
    robot_controller = Node(
        package='erasers_g1_common',
        executable='robot_controller',
        parameters=[
            default_robot_controller_params,
            {'robot_description': ParameterValue(
                Command(['xacro ', robot_model]), value_type=str)},
        ],
        output='screen',
        emulate_tty=True,
    )
    loco_service_client = Node(
        package='erasers_g1_common',
        executable='loco_service_client',
        output='screen',
        emulate_tty=True,
    )
    emergency_stop = Node(
        package='erasers_g1_common',
        executable='emergency_stop',
        parameters=[{'emc_pose': 'safety'}],
        output='screen',
        emulate_tty=True,
    )
    emergency_stop_announcer = Node(
        package='erasers_g1_api',
        executable='emergency_stop_announcer',
        parameters=[{'repeat_interval_sec': 10.0}],
        output='screen',
        emulate_tty=True,
    )
    emc_joy = Node(
        package='joy',
        executable='joy_node',
        namespace='emc',
        condition=IfCondition(use_emc),
        output='screen',
        emulate_tty=True,
    )
    odom = Node(
        package='erasers_g1_common',
        executable='odom_publisher',
        output='screen',
        emulate_tty=True,
    )
    scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        parameters=[ptl_params, {'use_sim_time': False}],
        remappings=[
            ('cloud_in', '/utlidar/cloud_livox_mid360'),
            ('scan', '/scan'),
        ],
        output='screen',
        emulate_tty=True,
    )
    whisper_node = Node(
        package='erasers_g1_api',
        executable='whisper_node',
        name='whisper_node',
        parameters=[{'device': device}],
        output='screen',
        emulate_tty=True,
    )
    ld.add_action(head_camera)
    ld.add_action(head_servo)
    ld.add_action(hand)
    ld.add_action(vui_client)
    ld.add_action(audio_client)
    ld.add_action(mic_server)
    ld.add_action(voicevox)
    ld.add_action(robot_controller)
    ld.add_action(loco_service_client)
    ld.add_action(emergency_stop)
    ld.add_action(emergency_stop_announcer)
    ld.add_action(emc_joy)
    ld.add_action(odom)
    ld.add_action(scan)
    ld.add_action(whisper_node)


    return ld
