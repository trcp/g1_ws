from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()


    # default variables
    pkg_share_dir = get_package_share_directory('erasers_g1_description')
    erasers_g1_common_pkg_share_dir = get_package_share_directory('erasers_g1_common')
    default_rviz_path = os.path.join(pkg_share_dir, 'rviz', 'g1.rviz')
    default_robot_description = os.path.join(pkg_share_dir, 'urdf', 'erasers_g1.urdf.xacro')
    default_robot_manager_params = os.path.join(
        erasers_g1_common_pkg_share_dir, 'config', 'robot_manager.yaml')


    # launch configurations
    robot_description = LaunchConfiguration('robot_description')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')


    # launch arguments
    declare_robot_description = DeclareLaunchArgument(
        'robot_description', default_value=default_robot_description)
    declare_use_rviz = DeclareLaunchArgument('use_rviz', default_value='false')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false')
    declare_use_mock_hardware = DeclareLaunchArgument('use_mock_hardware', default_value='false')
    ld.add_action(declare_robot_description)
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_use_mock_hardware)


    # nodes
    description = ParameterValue(Command([
        'xacro ', robot_description, ' use_mock_hardware:=', use_mock_hardware,
    ]), value_type=str)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': description,
            'use_sim_time': use_sim_time,
            'publish_frequency': 30.0,
            # モックの 30 Hz 入力を ROS 時刻の量子化で間引かない。
            'ignore_timestamp': ParameterValue(use_mock_hardware, value_type=bool),
        }],
        output='screen',
        emulate_tty=True,
    )
    robot_manager = Node(
        package='erasers_g1_common',
        executable='robot_manager',
        parameters=[
            default_robot_manager_params,
            {'robot_description': description, 'use_sim_time': use_sim_time},
        ],
        condition=UnlessCondition(use_mock_hardware),
        output='screen',
        emulate_tty=True,
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', default_rviz_path],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
        output='screen',
        emulate_tty=True,
    )
    ld.add_action(robot_state_publisher)
    ld.add_action(robot_manager)
    ld.add_action(rviz)


    return ld
