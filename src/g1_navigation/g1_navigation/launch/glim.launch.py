#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration


def glim_wrapper_cmd(mode, config_path, use_sim_time, map_name, min_height, max_height, edited_dump_dir):
    return [
        'ros2',
        'run',
        'g1_navigation',
        'glim_wrapper',
        mode,
        config_path,
        use_sim_time,
        map_name,
        min_height,
        max_height,
        edited_dump_dir,
    ]


def generate_launch_description():
    ld = LaunchDescription()

    default_config_path = os.path.join(get_package_share_directory('g1_navigation'), 'config')

    config_path = LaunchConfiguration('config_path')
    use_sim_time = LaunchConfiguration('use_sim_time')
    edit_map = LaunchConfiguration('edit_map')
    display = LaunchConfiguration('display')
    edited_dump_dir = LaunchConfiguration('edited_dump_dir')
    map_name = LaunchConfiguration('map_name')
    min_height = LaunchConfiguration('min_height')
    max_height = LaunchConfiguration('max_height')

    ld.add_action(DeclareLaunchArgument(
        'config_path',
        default_value=default_config_path,
        description='path to GLIM config directory',
    ))
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='use sim time',
    ))
    ld.add_action(DeclareLaunchArgument(
        'edit_map',
        default_value='false',
        description='Run GLIM map_editor against /tmp/dump instead of glim_rosnode',
    ))
    ld.add_action(DeclareLaunchArgument(
        'display',
        default_value=EnvironmentVariable('DISPLAY', default_value=''),
        description='DISPLAY value used by GLIM GUI processes',
    ))
    ld.add_action(DeclareLaunchArgument(
        'edited_dump_dir',
        default_value='',
        description='Expected map_editor save directory. Empty uses offline_viewer_save recent path.',
    ))
    ld.add_action(DeclareLaunchArgument(
        'map_name',
        default_value='map',
        description='PCD map file name without extension',
    ))
    ld.add_action(DeclareLaunchArgument(
        'min_height',
        default_value='0.0',
        description='Minimum height of map from 3d PCD map to 2D png map.',
    ))
    ld.add_action(DeclareLaunchArgument(
        'max_height',
        default_value='4.0',
        description='Max height of map from 3d PCD map to 2D png map.',
    ))

    common_env = {'DISPLAY': display}

    ld.add_action(ExecuteProcess(
        cmd=glim_wrapper_cmd('run', config_path, use_sim_time, map_name, min_height, max_height, edited_dump_dir),
        name='glim_wrapper',
        output='screen',
        additional_env=common_env,
        sigterm_timeout='60',
        sigkill_timeout='30',
        condition=UnlessCondition(edit_map),
    ))
    ld.add_action(ExecuteProcess(
        cmd=glim_wrapper_cmd('edit', config_path, use_sim_time, map_name, min_height, max_height, edited_dump_dir),
        name='glim_map_editor_wrapper',
        output='screen',
        additional_env=common_env,
        sigterm_timeout='60',
        sigkill_timeout='30',
        condition=IfCondition(edit_map),
    ))

    return ld
