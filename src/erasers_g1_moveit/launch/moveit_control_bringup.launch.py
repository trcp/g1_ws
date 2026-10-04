#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()


    # default variables
    pkg_share_dir = get_package_share_directory('erasers_g1_moveit')
    erasers_g1_description_pkg_share_dir = get_package_share_directory('erasers_g1_description')
    default_robot_model = os.path.join(
        erasers_g1_description_pkg_share_dir, 'urdf', 'erasers_g1.urdf.xacro')
    default_move_group_launch = os.path.join(pkg_share_dir, 'launch', 'move_group.launch.py')
    default_moveit_rviz_launch = os.path.join(pkg_share_dir, 'launch', 'moveit_rviz.launch.py')


    # launch configurations
    robot_model = LaunchConfiguration('robot_model')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    auto_enable_upper_body = LaunchConfiguration('auto_enable_upper_body')


    # launch arguments
    declare_robot_model = DeclareLaunchArgument(
        'robot_model',
        default_value=default_robot_model,
        description='URDF モデルパス',
    )
    declare_use_mock_hardware = DeclareLaunchArgument(
        'use_mock_hardware',
        default_value='false',
        description='モックハードウェアを使用する',
        choices=['true', 'false'],
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='シミュレーション時刻を使用する',
        choices=['true', 'false'],
    )
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='MoveIt RViz を起動する',
        choices=['true', 'false'],
    )
    declare_auto_enable_upper_body = DeclareLaunchArgument(
        'auto_enable_upper_body',
        default_value='true',
        choices=['true', 'false'],
        description='実機コントローラの起動完了後、HW の上半身制御権を一度だけ要求する',
    )
    declare_start_robot_controller = DeclareLaunchArgument(
        'start_robot_controller',
        default_value='false',
        choices=['true', 'false'],
        description='非推奨: robot_controller は bringup.launch.py で一元管理されます',
    )
    ld.add_action(declare_robot_model)
    ld.add_action(declare_use_mock_hardware)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_auto_enable_upper_body)
    ld.add_action(declare_start_robot_controller)


    # include launch
    include_move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(default_move_group_launch),
        launch_arguments={
            'robot_description_path': robot_model,
            'use_mock_hardware': use_mock_hardware,
            'use_sim_time': use_sim_time,
        }.items(),
    )
    include_visualizer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(default_moveit_rviz_launch),
        launch_arguments={
            'robot_description_path': robot_model,
            'use_mock_hardware': use_mock_hardware,
            'use_sim_time': use_sim_time,
            'use_rviz': use_rviz,
        }.items(),
    )
    ld.add_action(include_move_group)
    ld.add_action(include_visualizer)


    # nodes
    # HW の中継サービスを使い、robot_controller の排他・停止判定を維持する。
    enable_upper_body_control = Node(
        package='erasers_g1_hw_controller',
        executable='enable_upper_body_control',
        parameters=[{'wait_timeout_sec': 30.0, 'response_timeout_sec': 10.0}],
        condition=IfCondition(PythonExpression([
            "'", auto_enable_upper_body, "' == 'true' and '",
            use_mock_hardware, "'.lower() == 'false' and '",
            use_sim_time, "'.lower() == 'false'",
        ])),
        output='screen',
        emulate_tty=True,
    )
    ld.add_action(enable_upper_body_control)


    return ld

