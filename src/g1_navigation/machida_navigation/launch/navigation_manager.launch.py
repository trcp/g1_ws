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
            'map_frame',
            default_value='map',
            description='Fixed frame for TF lookup (robot pose)',
        ),
        DeclareLaunchArgument(
            'robot_base_frame',
            default_value='base_footprint',
            description='Robot base frame for TF lookup (map -> robot_base_frame)',
        ),
        DeclareLaunchArgument(
            'local_costmap_topic',
            default_value='/local_costmap',
            description='Local costmap topic used for local A* planning',
        ),
        DeclareLaunchArgument(
            'path_obstacle_threshold',
            default_value='75',
            description='Costmap cost >= this value is stamped into obstacle memory (0-100)',
        ),
        DeclareLaunchArgument(
            'path_check_horizon',
            default_value='1.5',
            description='Distance ahead on the global path to check for obstacles [m]',
        ),
        DeclareLaunchArgument(
            'path_check_use_memory',
            default_value='true',
            description='true=obstacle memory grid, false=raw local costmap for path blocking check',
        ),
        DeclareLaunchArgument(
            'local_plan_frequency',
            default_value='5.0',
            description='Frequency of local A* replanning [Hz]',
        ),
        DeclareLaunchArgument(
            'goal_tolerance',
            default_value='0.15',
            description='Distance to final goal at which navigation is considered complete [m]',
        ),
        DeclareLaunchArgument(
            'goal_yaw_tolerance',
            default_value='0.05',
            description='Yaw error threshold at which goal orientation is considered reached [rad]',
        ),
        DeclareLaunchArgument(
            'replan_cooldown',
            default_value='2.0',
            description='Minimum time between consecutive global replan requests [s]',
        ),
        DeclareLaunchArgument(
            'augmented_costmap_topic',
            default_value='/augmented_local_costmap',
            description='Topic to publish the memory-augmented costmap for the global planner',
        ),
        DeclareLaunchArgument(
            'obstacle_decay_rate',
            default_value='5',
            description='Amount subtracted from memory grid cells per decay tick (0-100)',
        ),
        DeclareLaunchArgument(
            'decay_frequency',
            default_value='2.0',
            description='Frequency of memory decay and augmented costmap publish [Hz]',
        ),

        Node(
            package='machida_navigation',
            executable='navigation_manager',
            name='navigation_manager',
            output='screen',
            parameters=[{
                'use_sim_time':                 LaunchConfiguration('use_sim_time'),
                'map_frame':                    LaunchConfiguration('map_frame'),
                'robot_base_frame':             LaunchConfiguration('robot_base_frame'),
                'local_costmap_topic':          LaunchConfiguration('local_costmap_topic'),
                'path_obstacle_threshold':      LaunchConfiguration('path_obstacle_threshold'),
                'path_check_horizon':           LaunchConfiguration('path_check_horizon'),
                'path_check_use_memory':        LaunchConfiguration('path_check_use_memory'),
                'local_plan_frequency':         LaunchConfiguration('local_plan_frequency'),
                'goal_tolerance':               LaunchConfiguration('goal_tolerance'),
                'goal_yaw_tolerance':           LaunchConfiguration('goal_yaw_tolerance'),
                'replan_cooldown':              LaunchConfiguration('replan_cooldown'),
                'augmented_costmap_topic':      LaunchConfiguration('augmented_costmap_topic'),
                'obstacle_decay_rate':          LaunchConfiguration('obstacle_decay_rate'),
                'decay_frequency':              LaunchConfiguration('decay_frequency'),
            }],
        ),
    ])
