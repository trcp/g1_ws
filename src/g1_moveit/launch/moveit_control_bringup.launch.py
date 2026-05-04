#!/usr/bin/env python3
"""
moveit_control_bringup.launch.py
G1 ヒューマノイドロボットのアーム制御と MoveIt (および RViz) を一括起動するスクリプト

起動されるノード/スクリプト:
1. controller.launch.py (g1_hw_controller - ros2_control)
2. arm_joint_control (erasers_g1_common_cpp)
3. move_group.launch.py (g1_moveit)
4. moveit_rviz.launch.py (g1_moveit)
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    # --- 引数 ---
    use_sim_time = LaunchConfiguration('use_sim_time')
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    ))

    # --- パッケージパス ---
    pkg_g1_moveit = get_package_share_directory('g1_moveit')

    pkg_g1_hw_controller = get_package_share_directory('g1_hw_controller')

    # --- ノード群 ---
    # 1. ros2_control (controller manager & hardware interface)
    ros2_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_g1_hw_controller, 'launch', 'controller.launch.py')
        )
    )
    ld.add_action(ros2_control_launch)

    # 2. Arm Joint Control (Existing SDK interface) - Already running on robot
    # arm_joint_control_node = Node(
    #     package='erasers_g1_common_cpp',
    #     executable='arm_joint_control',
    #     name='arm_joint_control',
    #     output='screen'
    # )
    # ld.add_action(arm_joint_control_node)

    # 3. MoveIt Move Group
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_g1_moveit, 'launch', 'move_group.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_rviz': 'true'
        }.items()
    )
    ld.add_action(move_group_launch)

    # 4. MoveIt RViz
    moveit_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_g1_moveit, 'launch', 'moveit_rviz.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    #ld.add_action(moveit_rviz_launch)

    return ld
