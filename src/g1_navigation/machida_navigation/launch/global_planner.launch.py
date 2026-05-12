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
            'obstacle_threshold',
            default_value='50',
            description='Occupancy value >= this is treated as obstacle',
        ),
        DeclareLaunchArgument(
            'use_smoothing',
            default_value='false',
            description='Apply gradient-descent path smoothing',
        ),
        DeclareLaunchArgument(
            'obstacle_cost_weight',
            default_value='5.0',
            description='Extra A* traversal cost weight for cells near obstacles',
        ),
        DeclareLaunchArgument(
            'planner_obstacle_threshold',
            default_value='99',
            description='Costmap value >= this is blocked for planning',
        ),
        DeclareLaunchArgument(
            'robot_base_frame',
            default_value='base_footprint',
            description='TF frame looked up as the planning start pose (map -> robot_base_frame)',
        ),
        DeclareLaunchArgument(
            'local_costmap_topic',
            default_value='/augmented_local_costmap',
            description='Costmap topic overlaid onto the global grid before A* planning '
                        '(use /augmented_local_costmap for obstacle memory, '
                        'or /local_costmap for raw sensor only)',
        ),

        Node(
            package='machida_navigation',
            executable='global_planner',
            name='global_planner',
            output='screen',
            parameters=[{
                'use_sim_time':               LaunchConfiguration('use_sim_time'),
                'obstacle_threshold':         LaunchConfiguration('obstacle_threshold'),
                'use_smoothing':              LaunchConfiguration('use_smoothing'),
                'obstacle_cost_weight':       LaunchConfiguration('obstacle_cost_weight'),
                'planner_obstacle_threshold': LaunchConfiguration('planner_obstacle_threshold'),
                'robot_base_frame':           LaunchConfiguration('robot_base_frame'),
                'local_costmap_topic':        LaunchConfiguration('local_costmap_topic'),
            }],
        ),
    ])
