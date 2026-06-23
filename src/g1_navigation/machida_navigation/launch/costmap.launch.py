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

        # --- GlobalCostmapNode / LocalCostmapNode ---
        DeclareLaunchArgument(
            'obstacle_threshold',
            default_value='50',
            description='Occupancy value >= this is treated as obstacle',
        ),
        DeclareLaunchArgument(
            'footprint',
            default_value="'0.3'",
            description='Robot footprint: radius [m], rectangle length,width [m], or vertices [x1,y1,...]',
        ),
        DeclareLaunchArgument(
            'global_clearance',
            default_value='0.3',
            description='Extra clearance beyond robot body for global costmap [m]',
        ),
        DeclareLaunchArgument(
            'local_clearance',
            default_value='0.3',
            description='Extra clearance beyond robot body for local costmap [m]',
        ),
        DeclareLaunchArgument(
            'free_space_weight',
            default_value='0.0',
            description='Weight for free-cell cost (k/d). 0 disables. Larger values prefer paths away from obstacles.',
        ),
        DeclareLaunchArgument(
            'unknown_cost',
            default_value='0',
            description='Cost assigned to unknown cells (-1 in OccupancyGrid). 0=free, 1-98=traversable with penalty, 99+=impassable.',
        ),

        # --- LocalCostmapNode ---
        DeclareLaunchArgument(
            'lidar_topic',
            default_value='/utlidar/cloud_livox_mid360',
            description='3D LiDAR PointCloud2 topic',
        ),
        DeclareLaunchArgument(
            'local_costmap_frame',
            default_value='odom',
            description='Fixed frame for the local costmap (rolling window)',
        ),
        DeclareLaunchArgument(
            'local_robot_base_frame',
            default_value='base_footprint',
            description='Robot base frame for local costmap TF lookup',
        ),
        DeclareLaunchArgument(
            'local_resolution',
            default_value='0.05',
            description='Local costmap grid resolution [m/cell]',
        ),
        DeclareLaunchArgument(
            'local_width',
            default_value='4.0',
            description='Local costmap width centred on robot [m]',
        ),
        DeclareLaunchArgument(
            'local_height',
            default_value='4.0',
            description='Local costmap height centred on robot [m]',
        ),
        DeclareLaunchArgument(
            'min_obstacle_height',
            default_value='0.1',
            description='Min point height above robot base treated as obstacle [m]',
        ),
        DeclareLaunchArgument(
            'max_obstacle_height',
            default_value='2.0',
            description='Max point height above robot base treated as obstacle [m]',
        ),
        DeclareLaunchArgument(
            'min_sensor_range',
            default_value='0.5',
            description='Min distance from sensor origin to accept a point [m] (LiDAR dead zone)',
        ),
        DeclareLaunchArgument(
            'realsense_topic',
            default_value='',
            description='RealSense PointCloud2 topic. Empty string disables RealSense input.',
        ),
        DeclareLaunchArgument(
            'realsense_min_obstacle_height',
            default_value='0.02',
            description='Min point height above robot base for RealSense [m]',
        ),
        DeclareLaunchArgument(
            'realsense_max_obstacle_height',
            default_value='1.0',
            description='Max point height above robot base for RealSense [m]',
        ),
        DeclareLaunchArgument(
            'realsense_min_sensor_range',
            default_value='0.2',
            description='Min distance from RealSense sensor origin to accept a point [m]',
        ),
        DeclareLaunchArgument(
            'min_robot_range',
            default_value='0.4',
            description='Min distance from robot center (odom frame) to accept a point as obstacle [m]. 0 disables.',
        ),

        # GlobalCostmapNode: /map2d -> /global_costmap
        Node(
            package='machida_navigation',
            executable='costmap_node',
            name='costmap_node',
            output='screen',
            parameters=[{
                'use_sim_time':       LaunchConfiguration('use_sim_time'),
                'obstacle_threshold': LaunchConfiguration('obstacle_threshold'),
                'footprint':          LaunchConfiguration('footprint'),
                'clearance':          LaunchConfiguration('global_clearance'),
                'free_space_weight':  LaunchConfiguration('free_space_weight'),
                'unknown_cost':       LaunchConfiguration('unknown_cost'),
            }],
        ),

        # LocalCostmapNode: PointCloud2 -> /local_costmap
        Node(
            package='machida_navigation',
            executable='local_costmap_node',
            name='local_costmap_node',
            output='screen',
            parameters=[{
                'use_sim_time':          LaunchConfiguration('use_sim_time'),
                'lidar_topic':           LaunchConfiguration('lidar_topic'),
                'local_costmap_frame':   LaunchConfiguration('local_costmap_frame'),
                'robot_base_frame':      LaunchConfiguration('local_robot_base_frame'),
                'resolution':            LaunchConfiguration('local_resolution'),
                'local_width':           LaunchConfiguration('local_width'),
                'local_height':          LaunchConfiguration('local_height'),
                'min_obstacle_height':   LaunchConfiguration('min_obstacle_height'),
                'max_obstacle_height':   LaunchConfiguration('max_obstacle_height'),
                'min_sensor_range':               LaunchConfiguration('min_sensor_range'),
                'footprint':                      LaunchConfiguration('footprint'),
                'clearance':                      LaunchConfiguration('local_clearance'),
                'free_space_weight':              LaunchConfiguration('free_space_weight'),
                'unknown_cost':                   LaunchConfiguration('unknown_cost'),
                'obstacle_threshold':             LaunchConfiguration('obstacle_threshold'),
                'realsense_topic':                LaunchConfiguration('realsense_topic'),
                'realsense_min_obstacle_height':  LaunchConfiguration('realsense_min_obstacle_height'),
                'realsense_max_obstacle_height':  LaunchConfiguration('realsense_max_obstacle_height'),
                'realsense_min_sensor_range':     LaunchConfiguration('realsense_min_sensor_range'),
                'min_robot_range':                LaunchConfiguration('min_robot_range'),
            }],
        ),
    ])
