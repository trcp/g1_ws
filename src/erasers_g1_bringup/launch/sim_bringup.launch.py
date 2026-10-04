"""実機通信を起動しない G1 上半身の仮想検証環境."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()

    # 既定値
    description_share = get_package_share_directory('erasers_g1_description')
    moveit_share = get_package_share_directory('erasers_g1_moveit')
    default_model = os.path.join(description_share, 'urdf', 'erasers_g1.urdf.xacro')

    # 起動時設定
    robot_model = LaunchConfiguration('robot_model')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    use_sim_time = LaunchConfiguration('use_sim_time')
    publish_mock_clock = LaunchConfiguration('publish_mock_clock')
    use_rviz = LaunchConfiguration('use_rviz')

    # 起動引数
    ld.add_action(DeclareLaunchArgument('robot_model', default_value=default_model))
    ld.add_action(DeclareLaunchArgument(
        'use_mock_hardware', default_value='true', choices=['true'],
        description='この入口では実機 HW を選択できません'))
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true'))
    ld.add_action(DeclareLaunchArgument('publish_mock_clock', default_value='true'))
    ld.add_action(DeclareLaunchArgument('use_rviz', default_value='true'))

    # 子 Launch
    display = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_share, 'launch', 'display.launch.py')),
        launch_arguments={
            'robot_description': robot_model, 'use_mock_hardware': use_mock_hardware,
            'use_sim_time': use_sim_time, 'use_rviz': 'false',
        }.items(),
    )
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_share, 'launch', 'moveit_control_bringup.launch.py')),
        launch_arguments={
            'robot_model': robot_model, 'use_mock_hardware': use_mock_hardware,
            'use_sim_time': use_sim_time, 'use_rviz': use_rviz,
            'start_robot_controller': 'false',
        }.items(),
    )

    # 起動アクション
    clock_condition = IfCondition(PythonExpression([
        "'", use_sim_time, "'.lower() == 'true' and '",
        publish_mock_clock, "'.lower() == 'true'",
    ]))

    # ノード
    clock = Node(
        package='erasers_g1_bringup', executable='mock_clock.py',
        name='mock_clock', condition=clock_condition,
        parameters=[{'use_sim_time': False}], output='screen', emulate_tty=True,
    )
    ld.add_action(clock)
    # 子 Launch の use_rviz などを親・兄弟へ漏らさない。
    ld.add_action(GroupAction(actions=[display], scoped=True))
    ld.add_action(GroupAction(actions=[moveit], scoped=True))

    return ld
