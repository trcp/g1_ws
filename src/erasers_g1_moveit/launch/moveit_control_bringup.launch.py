"""ros2_control、move_group、RViz の起動を統括する."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def _validate_mode(context):
    mock = LaunchConfiguration('use_mock_hardware').perform(context).lower() == 'true'
    real = LaunchConfiguration('start_robot_controller').perform(context).lower() == 'true'
    if mock and real:
        raise RuntimeError('モックと実機 robot_controller は同時に起動できません')
    return []


def generate_launch_description():
    ld = LaunchDescription()

    # 既定値
    moveit_share = get_package_share_directory('erasers_g1_moveit')
    hw_share = get_package_share_directory('erasers_g1_hw_controller')
    common_share = get_package_share_directory('erasers_g1_common')
    description_share = get_package_share_directory('erasers_g1_description')
    default_model = os.path.join(description_share, 'urdf', 'erasers_g1.urdf.xacro')

    # 起動時設定
    model = LaunchConfiguration('robot_model')
    mock = LaunchConfiguration('use_mock_hardware')
    sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('use_rviz')
    start_robot = LaunchConfiguration('start_robot_controller')
    auto_enable_upper_body = LaunchConfiguration('auto_enable_upper_body')

    # 起動引数
    ld.add_action(DeclareLaunchArgument('robot_model', default_value=default_model))
    ld.add_action(DeclareLaunchArgument('use_mock_hardware', default_value='false'))
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false'))
    ld.add_action(DeclareLaunchArgument('use_rviz', default_value='true'))
    ld.add_action(DeclareLaunchArgument('start_robot_controller', default_value='true'))
    declare_auto_enable_upper_body = DeclareLaunchArgument(
        'auto_enable_upper_body', default_value='true', choices=['true', 'false'],
        description='実機コントローラの起動完了後、HW の上半身制御権を一度だけ要求する')
    ld.add_action(declare_auto_enable_upper_body)

    # 子 Launch
    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(hw_share, 'launch', 'controller.launch.py')),
        launch_arguments={'robot_model': model, 'use_mock_hardware': mock,
                          'use_sim_time': sim_time}.items(),
    )
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_share, 'launch', 'move_group.launch.py')),
        launch_arguments={'robot_description_path': model, 'use_mock_hardware': mock,
                          'use_sim_time': sim_time}.items(),
    )
    visualizer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_share, 'launch', 'moveit_rviz.launch.py')),
        launch_arguments={'robot_description_path': model, 'use_mock_hardware': mock,
                          'use_sim_time': sim_time, 'use_rviz': rviz}.items(),
    )

    # 起動アクション
    ld.add_action(OpaqueFunction(function=_validate_mode))

    # ノード
    robot_controller = Node(
        package='erasers_g1_common', executable='robot_controller',
        parameters=[os.path.join(common_share, 'config', 'robot_controller.yaml'),
                    {'robot_description': ParameterValue(
                        Command(['xacro ', model]), value_type=str),
                     'use_sim_time': sim_time}],
        condition=IfCondition(start_robot), output='screen', emulate_tty=True,
    )
    # HW の中継サービスを使い、robot_controller の排他・停止判定を維持する。
    enable_upper_body_control = Node(
        package='erasers_g1_hw_controller',
        executable='enable_upper_body_control',
        parameters=[{'wait_timeout_sec': 30.0, 'response_timeout_sec': 10.0}],
        condition=IfCondition(PythonExpression([
            "'", auto_enable_upper_body, "' == 'true' and '",
            mock, "'.lower() == 'false' and '", sim_time, "'.lower() == 'false'",
        ])),
        output='screen',
        emulate_tty=True,
    )
    ld.add_action(control)
    ld.add_action(move_group)
    ld.add_action(visualizer)
    ld.add_action(robot_controller)
    ld.add_action(enable_upper_body_control)

    return ld
