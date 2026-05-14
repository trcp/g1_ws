#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    home = os.environ.get('HOME', '/home/roboworks')

    default_map_dir = os.path.join(home, 'colcon_ws', 'map')
    default_map_name = 'map'
    default_localization_param_dir = os.path.join(
        get_package_share_directory('g1_navigation'),
        'params',
        'nav2_ndt_g1.yaml',
    )

    localization_launch = os.path.join(
        get_package_share_directory('lidar_localization_ros2'),
        'launch',
        'nav2_navigation.launch.py',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    launch_map_server = LaunchConfiguration('launch_map_server')
    launch_localization = LaunchConfiguration('launch_localization')

    map_dir = LaunchConfiguration('map_dir')
    map_name = LaunchConfiguration('map_name')
    map_yaml_file = [map_dir, '/', map_name, '.yaml']
    localization_param_dir = LaunchConfiguration('localization_param_dir')
    pcd_map_path = [map_dir, '/', map_name, '.pcd']
    cloud_topic = LaunchConfiguration('cloud_topic')
    pointcloud_topic = LaunchConfiguration('pointcloud_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    twist_topic = LaunchConfiguration('twist_topic')
    odom_topic = LaunchConfiguration('odom_topic')

    map_frame = LaunchConfiguration('map_frame')
    odom_frame = LaunchConfiguration('odom_frame')
    localization_base_frame = LaunchConfiguration('localization_base_frame')
    robot_base_frame = LaunchConfiguration('robot_base_frame')
    lidar_frame_id = LaunchConfiguration('lidar_frame_id')
    imu_frame_id = LaunchConfiguration('imu_frame_id')

    declarations = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument(
            'launch_map_server',
            default_value='true',
            description='Start nav2_map_server and publish its map as /map2d.',
        ),
        DeclareLaunchArgument(
            'launch_localization',
            default_value='true',
            description='Start lidar_localization_ros2 for map->odom localization.',
        ),
        DeclareLaunchArgument(
            'map_dir',
            default_value=default_map_dir,
            description='Directory containing the map yaml and pcd files.',
        ),
        DeclareLaunchArgument(
            'map_name',
            default_value=default_map_name,
            description='Map file stem without extension. Uses <map_dir>/<map_name>.yaml and .pcd.',
        ),
        DeclareLaunchArgument(
            'localization_param_dir',
            default_value=default_localization_param_dir,
            description='Parameter file for lidar_localization_ros2.',
        ),
        DeclareLaunchArgument(
            'cloud_topic',
            default_value='/utlidar/cloud_livox_mid360',
            description='Input PointCloud2 topic for lidar localization.',
        ),
        DeclareLaunchArgument(
            'pointcloud_topic',
            default_value='/utlidar/cloud_livox_mid360',
            description='Input PointCloud2 topic for the local costmap.',
        ),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/utlidar/imu_livox_mid360',
            description='Input IMU topic for lidar localization.',
        ),
        DeclareLaunchArgument(
            'twist_topic',
            default_value='/twist',
            description='Input twist topic for lidar localization prediction.',
        ),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/odom',
            description='Odometry topic used by localization helper nodes.',
        ),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument(
            'localization_base_frame',
            default_value='base_link',
            description='Base frame used by lidar_localization_ros2.',
        ),
        DeclareLaunchArgument(
            'robot_base_frame',
            default_value='base_footprint',
            description='Planar robot frame used by machida_navigation.',
        ),
        DeclareLaunchArgument('lidar_frame_id', default_value='livox_frame'),
        DeclareLaunchArgument('imu_frame_id', default_value='livox_frame'),
        DeclareLaunchArgument('publish_lidar_tf', default_value='false'),
        DeclareLaunchArgument('publish_imu_tf', default_value='false'),
        DeclareLaunchArgument('set_initial_pose', default_value='true'),
        DeclareLaunchArgument('initial_pose_x', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_y', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_z', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_qx', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_qy', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_qz', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_qw', default_value='1.0'),
        DeclareLaunchArgument('enable_reinitialization_supervisor', default_value='true'),
        DeclareLaunchArgument('localizer_enable_map_odom_tf', default_value='true'),
        DeclareLaunchArgument('localizer_enable_timer_publishing', default_value='true'),
        DeclareLaunchArgument('localizer_pose_publish_frequency', default_value='10.0'),
        DeclareLaunchArgument('publish_localizer_pose_odom', default_value='false'),
        DeclareLaunchArgument(
            'publish_identity_odom',
            default_value='false',
            description='Publish replay-safe odom->base TF when rosbag TF is missing or stale.',
        ),
        DeclareLaunchArgument(
            'identity_odom_rate_hz',
            default_value='100.0',
            description='Publish rate for replay-safe identity odom TF.',
        ),
        DeclareLaunchArgument('publish_twist_odom', default_value='false'),
        DeclareLaunchArgument('publish_cmd_vel_odom', default_value='false'),
        DeclareLaunchArgument('obstacle_threshold', default_value='50'),
        DeclareLaunchArgument(
            'footprint',
            default_value='0.4,0.3',
            description='Robot footprint: radius, length,width, or x1,y1,... vertices.',
        ),
        DeclareLaunchArgument('clearance', default_value='0.1'),
        DeclareLaunchArgument('local_costmap_frame', default_value='odom'),
        DeclareLaunchArgument('local_resolution', default_value='0.05'),
        DeclareLaunchArgument('local_width', default_value='4.0'),
        DeclareLaunchArgument('local_height', default_value='4.0'),
        DeclareLaunchArgument('min_obstacle_height', default_value='0.1'),
        DeclareLaunchArgument('max_obstacle_height', default_value='2.0'),
        DeclareLaunchArgument('min_sensor_range', default_value='0.5'),
        DeclareLaunchArgument('use_smoothing', default_value='false'),
        DeclareLaunchArgument('obstacle_cost_weight', default_value='5.0'),
        DeclareLaunchArgument('planner_obstacle_threshold', default_value='99'),
        DeclareLaunchArgument('path_obstacle_threshold', default_value='75'),
        DeclareLaunchArgument('local_plan_frequency', default_value='5.0'),
        DeclareLaunchArgument('local_plan_horizon', default_value='0.5'),
        DeclareLaunchArgument('local_plan_obstacle_threshold', default_value='70'),
        DeclareLaunchArgument('use_local_smoothing', default_value='true'),
        DeclareLaunchArgument('goal_tolerance', default_value='0.15'),
        DeclareLaunchArgument('replan_cooldown', default_value='2.0'),
        DeclareLaunchArgument('augmented_costmap_topic', default_value='/augmented_local_costmap'),
        DeclareLaunchArgument('obstacle_decay_rate', default_value='5'),
        DeclareLaunchArgument('decay_frequency', default_value='2.0'),
        DeclareLaunchArgument('path_topic', default_value='/execute_path_plan'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('execute_topic', default_value='/execute_local_planner'),
        DeclareLaunchArgument('lookahead_distance', default_value='0.2'),
        DeclareLaunchArgument('linear_velocity', default_value='0.15'),
        DeclareLaunchArgument('max_angular_velocity', default_value='1.0'),
        DeclareLaunchArgument('max_path_deviation', default_value='0.3'),
        DeclareLaunchArgument('slowdown_distance', default_value='0.6'),
        DeclareLaunchArgument('min_linear_velocity', default_value='0.2'),
        DeclareLaunchArgument('control_frequency', default_value='20.0'),
        DeclareLaunchArgument('holonomic', default_value='true'),
        DeclareLaunchArgument('max_linear_acceleration', default_value='0.5'),
        DeclareLaunchArgument('max_angular_acceleration', default_value='2.0'),
    ]

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch),
        condition=IfCondition(launch_localization),
        launch_arguments={
            'launch_nav2': 'false',
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'localization_param_dir': localization_param_dir,
            'pcd_map_path': pcd_map_path,
            'cloud_topic': cloud_topic,
            'pointcloud_topic': pointcloud_topic,
            'imu_topic': imu_topic,
            'twist_topic': twist_topic,
            'odom_topic': odom_topic,
            'global_frame_id': map_frame,
            'odom_frame_id': odom_frame,
            'base_frame_id': localization_base_frame,
            'publish_lidar_tf': LaunchConfiguration('publish_lidar_tf'),
            'lidar_frame_id': lidar_frame_id,
            'publish_imu_tf': LaunchConfiguration('publish_imu_tf'),
            'imu_frame_id': imu_frame_id,
            'set_initial_pose': LaunchConfiguration('set_initial_pose'),
            'initial_pose_x': LaunchConfiguration('initial_pose_x'),
            'initial_pose_y': LaunchConfiguration('initial_pose_y'),
            'initial_pose_z': LaunchConfiguration('initial_pose_z'),
            'initial_pose_qx': LaunchConfiguration('initial_pose_qx'),
            'initial_pose_qy': LaunchConfiguration('initial_pose_qy'),
            'initial_pose_qz': LaunchConfiguration('initial_pose_qz'),
            'initial_pose_qw': LaunchConfiguration('initial_pose_qw'),
            'enable_reinitialization_supervisor': LaunchConfiguration(
                'enable_reinitialization_supervisor'
            ),
            'localizer_enable_map_odom_tf': LaunchConfiguration('localizer_enable_map_odom_tf'),
            'localizer_enable_timer_publishing': LaunchConfiguration(
                'localizer_enable_timer_publishing'
            ),
            'localizer_pose_publish_frequency': LaunchConfiguration(
                'localizer_pose_publish_frequency'
            ),
            'publish_localizer_pose_odom': LaunchConfiguration('publish_localizer_pose_odom'),
            'publish_identity_odom': LaunchConfiguration('publish_identity_odom'),
            'identity_odom_rate_hz': LaunchConfiguration('identity_odom_rate_hz'),
            'publish_twist_odom': LaunchConfiguration('publish_twist_odom'),
            'publish_cmd_vel_odom': LaunchConfiguration('publish_cmd_vel_odom'),
        }.items(),
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(launch_map_server),
        parameters=[{
            'yaml_filename': map_yaml_file,
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('/map', '/map2d'),
        ],
    )

    map_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_machida_map',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(launch_map_server),
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['map_server'],
        }],
    )

    global_costmap = Node(
        package='machida_navigation',
        executable='costmap_node',
        name='global_costmap_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'obstacle_threshold': LaunchConfiguration('obstacle_threshold'),
            'footprint': LaunchConfiguration('footprint'),
            'clearance': LaunchConfiguration('clearance'),
        }],
    )

    local_costmap = Node(
        package='machida_navigation',
        executable='local_costmap_node',
        name='local_costmap_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'lidar_topic': pointcloud_topic,
            'local_costmap_frame': LaunchConfiguration('local_costmap_frame'),
            'robot_base_frame': robot_base_frame,
            'resolution': LaunchConfiguration('local_resolution'),
            'local_width': LaunchConfiguration('local_width'),
            'local_height': LaunchConfiguration('local_height'),
            'min_obstacle_height': LaunchConfiguration('min_obstacle_height'),
            'max_obstacle_height': LaunchConfiguration('max_obstacle_height'),
            'min_sensor_range': LaunchConfiguration('min_sensor_range'),
            'footprint': LaunchConfiguration('footprint'),
            'clearance': LaunchConfiguration('clearance'),
            'obstacle_threshold': LaunchConfiguration('obstacle_threshold'),
        }],
    )

    global_planner = Node(
        package='machida_navigation',
        executable='global_planner',
        name='global_planner',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'obstacle_threshold': LaunchConfiguration('obstacle_threshold'),
            'use_smoothing': LaunchConfiguration('use_smoothing'),
            'obstacle_cost_weight': LaunchConfiguration('obstacle_cost_weight'),
            'planner_obstacle_threshold': LaunchConfiguration('planner_obstacle_threshold'),
            'robot_base_frame': robot_base_frame,
            'local_costmap_topic': LaunchConfiguration('augmented_costmap_topic'),
        }],
    )

    navigation_manager = Node(
        package='machida_navigation',
        executable='navigation_manager',
        name='navigation_manager',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'map_frame': map_frame,
            'robot_base_frame': robot_base_frame,
            'local_costmap_topic': '/local_costmap',
            'path_obstacle_threshold': LaunchConfiguration('path_obstacle_threshold'),
            'local_plan_frequency': LaunchConfiguration('local_plan_frequency'),
            'local_plan_horizon': LaunchConfiguration('local_plan_horizon'),
            'local_plan_obstacle_threshold': LaunchConfiguration('local_plan_obstacle_threshold'),
            'use_local_smoothing': LaunchConfiguration('use_local_smoothing'),
            'goal_tolerance': LaunchConfiguration('goal_tolerance'),
            'replan_cooldown': LaunchConfiguration('replan_cooldown'),
            'augmented_costmap_topic': LaunchConfiguration('augmented_costmap_topic'),
            'obstacle_decay_rate': LaunchConfiguration('obstacle_decay_rate'),
            'decay_frequency': LaunchConfiguration('decay_frequency'),
        }],
    )

    local_planner = Node(
        package='machida_navigation',
        executable='local_planner',
        name='pure_pursuit_local_planner',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'path_topic': LaunchConfiguration('path_topic'),
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'execute_topic': LaunchConfiguration('execute_topic'),
            'map_frame': map_frame,
            'robot_base_frame': robot_base_frame,
            'lookahead_distance': LaunchConfiguration('lookahead_distance'),
            'linear_velocity': LaunchConfiguration('linear_velocity'),
            'max_angular_velocity': LaunchConfiguration('max_angular_velocity'),
            'goal_tolerance': LaunchConfiguration('goal_tolerance'),
            'max_path_deviation': LaunchConfiguration('max_path_deviation'),
            'slowdown_distance': LaunchConfiguration('slowdown_distance'),
            'min_linear_velocity': LaunchConfiguration('min_linear_velocity'),
            'control_frequency': LaunchConfiguration('control_frequency'),
            'holonomic': LaunchConfiguration('holonomic'),
            'max_linear_acceleration': LaunchConfiguration('max_linear_acceleration'),
            'max_angular_acceleration': LaunchConfiguration('max_angular_acceleration'),
        }],
    )

    return LaunchDescription(
        declarations
        + [
            SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
            SetParameter(name='use_sim_time', value=use_sim_time),
            localization,
            map_server,
            map_lifecycle_manager,
            global_costmap,
            local_costmap,
            global_planner,
            navigation_manager,
            local_planner,
        ]
    )
