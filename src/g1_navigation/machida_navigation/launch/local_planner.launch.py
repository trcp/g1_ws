from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock',
        ),
        DeclareLaunchArgument(
            'path_topic',
            default_value='/execute_path_plan',
            description='Path topic to follow',
        ),
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='/cmd_vel',
            description='Velocity command topic',
        ),
        DeclareLaunchArgument(
            'execute_topic',
            default_value='/execute_local_planner',
            description='Bool topic that enables/disables path execution',
        ),
        DeclareLaunchArgument(
            'map_frame',
            default_value='map',
            description='Fixed frame for TF lookup',
        ),
        DeclareLaunchArgument(
            'robot_base_frame',
            default_value='base_footprint',
            description='Robot base frame for TF lookup (map -> robot_base_frame)',
        ),
        DeclareLaunchArgument(
            'lookahead_distance',
            default_value='0.2',
            description='Pure Pursuit lookahead distance [m]',
        ),
        DeclareLaunchArgument(
            'linear_velocity',
            default_value='0.3',
            description='Nominal forward velocity [m/s]',
        ),
        DeclareLaunchArgument(
            'max_angular_velocity',
            default_value='1.0',
            description='Angular velocity limit [rad/s]',
        ),
        DeclareLaunchArgument(
            'goal_tolerance',
            default_value='0.15',
            description='Distance to goal for stopping [m]',
        ),
        DeclareLaunchArgument(
            'max_path_deviation',
            default_value='0.3',
            description='Stop if robot is farther than this from the global path [m]',
        ),
        DeclareLaunchArgument(
            'slowdown_distance',
            default_value='0.6',
            description='Distance from goal where linear velocity starts slowing down [m]',
        ),
        DeclareLaunchArgument(
            'min_linear_velocity',
            default_value='0.2',
            description='Minimum forward velocity sent to the robot [m/s] (hardware dead-band)',
        ),
        DeclareLaunchArgument(
            'max_linear_velocity',
            default_value='0.5',
            description='Maximum forward velocity sent to the robot [m/s]',
        ),
        DeclareLaunchArgument(
            'control_frequency',
            default_value='20.0',
            description='Control loop frequency [Hz]',
        ),
        DeclareLaunchArgument(
            'holonomic',
            default_value='true',
            description='Use holonomic (omni-wheel) mode instead of differential drive',
        ),

        Node(
            package='machida_navigation',
            executable='local_planner',
            name='pure_pursuit_local_planner',
            output='screen',
            parameters=[{
                'use_sim_time':         LaunchConfiguration('use_sim_time'),
                'path_topic':           LaunchConfiguration('path_topic'),
                'cmd_vel_topic':        LaunchConfiguration('cmd_vel_topic'),
                'execute_topic':        LaunchConfiguration('execute_topic'),
                'map_frame':            LaunchConfiguration('map_frame'),
                'robot_base_frame':     LaunchConfiguration('robot_base_frame'),
                'lookahead_distance':   LaunchConfiguration('lookahead_distance'),
                'linear_velocity':      LaunchConfiguration('linear_velocity'),
                'max_angular_velocity': LaunchConfiguration('max_angular_velocity'),
                'goal_tolerance':       LaunchConfiguration('goal_tolerance'),
                'max_path_deviation':   LaunchConfiguration('max_path_deviation'),
                'slowdown_distance':    LaunchConfiguration('slowdown_distance'),
                'min_linear_velocity':  LaunchConfiguration('min_linear_velocity'),
                'max_linear_velocity':  LaunchConfiguration('max_linear_velocity'),
                'control_frequency':    LaunchConfiguration('control_frequency'),
                'holonomic':            LaunchConfiguration('holonomic'),
            }],
        ),
    ])
