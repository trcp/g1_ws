import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get paths
    g1_hw_controller_share = get_package_share_directory('g1_hw_controller')
    g1_description_share = get_package_share_directory('g1_description')

    # robot_description using the integrated xacro in g1_description
    robot_description_path = os.path.join(g1_description_share, 'urdf', 'erasers_g1.urdf.xacro')
    robot_description = {'robot_description': Command(['xacro ', robot_description_path])}

    # Controller configurations
    controllers_file = os.path.join(g1_hw_controller_share, 'config', 'g1_controllers.yaml')

    # Nodes
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers_file],
        output='both',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    upper_body_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['upper_body_controller', '--controller-manager', '/controller_manager'],
    )

    return LaunchDescription([
        control_node,
        joint_state_broadcaster_spawner,
        #forward_position_controller_spawner,
        upper_body_controller_spawner,
    ])
