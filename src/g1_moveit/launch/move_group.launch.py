#!/usr/bin/env python3
"""
move_group.launch.py
G1 ヒューマノイドロボット上半身 MoveIt 2 の move_group ノード起動スクリプト

前提: g1_hw_controller の controller.launch.py が起動済みで
      /upper_body_controller/follow_joint_trajectory アクションサーバが有効であること。
"""
import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    ld = LaunchDescription()

    # --- パッケージパス ---
    pkg_g1_description = get_package_share_directory('g1_description')
    pkg_g1_moveit = get_package_share_directory('g1_moveit')

    default_robot_description_path = os.path.join(
        pkg_g1_description, 'urdf', 'erasers_g1.urdf'
    )
    default_use_sim_time = 'false'

    # --- Launch 引数 ---
    robot_description_path = LaunchConfiguration('robot_description_path')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')

    ld.add_action(DeclareLaunchArgument(
        'robot_description_path', default_value=default_robot_description_path,
        description='URDF/Xacro のパス'
    ))
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', default_value=default_use_sim_time,
        description='シミュレーション時間の使用'
    ))
    ld.add_action(DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='RViz を起動するか否か'
    ))

    # --- パラメータのロード ---
    robot_description = ParameterValue(
        Command(['xacro ', robot_description_path]),
        value_type=str
    )

    robot_description_semantic_content = load_file('g1_moveit', 'config/g1.srdf')
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_content}

    kinematics_yaml = load_yaml('g1_moveit', 'config/kinematics.yaml')
    robot_description_kinematics = {'robot_description_kinematics': kinematics_yaml}

    joint_limits_yaml = load_yaml('g1_moveit', 'config/joint_limits.yaml')
    robot_description_planning = {'robot_description_planning': joint_limits_yaml}

    # ROS2 Humble MoveIt2 形式: ompl 名前空間に包んで渡す
    ompl_planning_yaml = load_yaml('g1_moveit', 'config/ompl_planning.yaml')
    ompl_planning_pipeline = {'ompl': ompl_planning_yaml}

    moveit_controllers_yaml = load_yaml('g1_moveit', 'config/moveit_controllers.yaml')
    rviz_config_file = os.path.join(pkg_g1_moveit, 'rviz', 'moveit.rviz')

    # --- move_group ノード ---
    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'robot_description': robot_description},
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline,
            moveit_controllers_yaml,
            {'planning_pipelines': ['ompl']},
            {'default_planning_pipeline': 'ompl'},
            {'publish_robot_description': True},
            {'publish_robot_description_semantic': True},
            {'use_sim_time': use_sim_time},
            {'trajectory_execution.allowed_start_tolerance': 0.05},
        ],
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        emulate_tty=True,
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        parameters=[
            {'robot_description': robot_description},
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_yaml,
            moveit_controllers_yaml,
            {'use_sim_time': use_sim_time},
        ],
        condition=IfCondition(use_rviz)
    )
    ld.add_action(move_group)
    #ld.add_action(rviz_node)

    return ld
