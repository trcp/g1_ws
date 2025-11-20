#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()


    # launch configurations
    xfer_format = LaunchConfiguration('xfer_format')
    multi_topic = LaunchConfiguration('multi_topic')
    data_src = LaunchConfiguration('data_src')
    publish_freq = LaunchConfiguration('publish_freq')
    output_type = LaunchConfiguration('output_type')
    frame_id = LaunchConfiguration('frame_id')
    lvx_file_path = LaunchConfiguration('lvx_file_path')
    cmdline_bd_code = LaunchConfiguration('cmdline_bd_code')
    user_config_path = LaunchConfiguration('user_config_path')


    # launch arguments
    declare_xfer_format = DeclareLaunchArgument(
        'xfer_format', default_value='0',
        description='0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format')

    declare_multi_topic = DeclareLaunchArgument(
        'multi_topic', default_value='0',
        description='0-All LiDARs share the same topic, 1-One LiDAR one topic')

    declare_data_src = DeclareLaunchArgument(
        'data_src', default_value='0',
        description='0-lidar, others-Invalid data src')

    declare_publish_freq = DeclareLaunchArgument(
        'publish_freq', default_value='10.0',
        description='frequency of publish, 5.0, 10.0, 20.0, 50.0, etc.')

    declare_output_type = DeclareLaunchArgument(
        'output_type', default_value='0',
        description='Output data type')

    declare_frame_id = DeclareLaunchArgument(
        'frame_id', default_value='mid360_link',
        description='Frame ID for the point cloud')

    declare_lvx_file_path = DeclareLaunchArgument(
        'lvx_file_path', default_value='%s/livox_test.lvx'%os.environ['HOME'],
        description='Path to the lvx file')

    declare_cmdline_bd_code = DeclareLaunchArgument(
        'cmdline_bd_code', default_value='livox0000000001',
        description='Command line input BD code')

    declare_user_config_path = DeclareLaunchArgument(
        'user_config_path', default_value=os.path.join(get_package_share_directory('g1_bringup'), 'config', 'MID360_config.json'),
        description='Path to the user config json file')

    ld.add_action(declare_xfer_format)
    ld.add_action(declare_multi_topic)
    ld.add_action(declare_data_src)
    ld.add_action(declare_publish_freq)
    ld.add_action(declare_output_type)
    ld.add_action(declare_frame_id)
    ld.add_action(declare_lvx_file_path)
    ld.add_action(declare_cmdline_bd_code)
    ld.add_action(declare_user_config_path)


    # MID-360 LiDAR を起動する
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='head_lidar',
        emulate_tty=True,
        parameters=[
            {"xfer_format": xfer_format},
            {"multi_topic": multi_topic},
            {"data_src": data_src},
            {"publish_freq": publish_freq},
            {"output_data_type": output_type},
            {"frame_id": frame_id},
            {"lvx_file_path": lvx_file_path},
            {"user_config_path": user_config_path},
            {"cmdline_input_bd_code": cmdline_bd_code}
        ]
    )

    ld.add_action(livox_driver)


    return ld
