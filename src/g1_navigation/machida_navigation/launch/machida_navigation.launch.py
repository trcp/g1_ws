#!/usr/bin/env python3

import os
import tempfile
from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node, SetParameter


def _as_optional_bool(value: str):
    normalized = value.strip().lower()
    if not normalized:
        return None
    if normalized in ('1', 'true', 'yes', 'on'):
        return True
    if normalized in ('0', 'false', 'no', 'off'):
        return False
    raise RuntimeError(f'Invalid boolean launch argument value: {value}')


def _as_optional_float(value: str):
    normalized = value.strip()
    if not normalized:
        return None
    return float(normalized)


def _rewrite_localization_params(
    localization_param_file: str,
    pcd_map_path: str,
    set_initial_pose,
    initial_pose_values: dict,
    enable_timer_publishing,
    pose_publish_frequency,
):
    should_rewrite = (
        bool(pcd_map_path)
        or set_initial_pose is not None
        or enable_timer_publishing is not None
        or pose_publish_frequency is not None
    )
    if not should_rewrite:
        return localization_param_file

    with Path(localization_param_file).open('r', encoding='utf-8') as stream:
        params = yaml.safe_load(stream)

    ros_params = params['/**']['ros__parameters']
    if pcd_map_path:
        ros_params['map_path'] = pcd_map_path
    if set_initial_pose is not None:
        ros_params['set_initial_pose'] = bool(set_initial_pose)
        if set_initial_pose:
            ros_params.update(initial_pose_values)
    if enable_timer_publishing is not None:
        ros_params['enable_timer_publishing'] = bool(enable_timer_publishing)
    if pose_publish_frequency is not None:
        ros_params['pose_publish_frequency'] = float(pose_publish_frequency)

    fd, rewritten_path = tempfile.mkstemp(prefix='machida_lidar_localization_', suffix='.yaml')
    os.close(fd)
    with Path(rewritten_path).open('w', encoding='utf-8') as stream:
        yaml.safe_dump(params, stream, sort_keys=False)
    return rewritten_path


def _start_lidar_localization(context, *args, **kwargs):
    global_frame_id = LaunchConfiguration('map_frame').perform(context)
    odom_frame_id = LaunchConfiguration('odom_frame').perform(context)
    base_frame_id = LaunchConfiguration('localization_base_frame').perform(context)
    odom_topic = LaunchConfiguration('odom_topic').perform(context)

    initial_pose_values = {
        'initial_pose_x': float(LaunchConfiguration('initial_pose_x').perform(context)),
        'initial_pose_y': float(LaunchConfiguration('initial_pose_y').perform(context)),
        'initial_pose_z': float(LaunchConfiguration('initial_pose_z').perform(context)),
        'initial_pose_qx': float(LaunchConfiguration('initial_pose_qx').perform(context)),
        'initial_pose_qy': float(LaunchConfiguration('initial_pose_qy').perform(context)),
        'initial_pose_qz': float(LaunchConfiguration('initial_pose_qz').perform(context)),
        'initial_pose_qw': float(LaunchConfiguration('initial_pose_qw').perform(context)),
    }

    if _as_optional_bool(LaunchConfiguration('use_odom_localization_demo').perform(context)):
        return [
            LogInfo(
                msg=(
                    'Using odom localization demo backend with '
                    f'{global_frame_id}->{odom_frame_id} and {odom_topic}'
                )
            ),
            Node(
                package='lidar_localization_ros2',
                executable='publish_pose_from_odom.py',
                name='pose_from_odom_publisher',
                output='screen',
                parameters=[{
                    'odom_topic': odom_topic,
                    'pose_topic': '/localization/pose_with_covariance',
                    'global_frame_id': global_frame_id,
                    'odom_frame_id': odom_frame_id,
                    'base_frame_id': base_frame_id,
                    **initial_pose_values,
                }],
            ),
        ]

    package_share = get_package_share_directory('lidar_localization_ros2')
    localization_launch = os.path.join(
        package_share,
        'launch',
        'nav2_lidar_localization.launch.py',
    )

    localization_param_dir = LaunchConfiguration('localization_param_dir').perform(context)
    pcd_map_path = LaunchConfiguration('pcd_map_path').perform(context).strip()
    set_initial_pose = _as_optional_bool(LaunchConfiguration('set_initial_pose').perform(context))
    enable_timer_publishing = _as_optional_bool(
        LaunchConfiguration('localizer_enable_timer_publishing').perform(context)
    )
    pose_publish_frequency = _as_optional_float(
        LaunchConfiguration('localizer_pose_publish_frequency').perform(context)
    )

    effective_localization_params = _rewrite_localization_params(
        localization_param_file=localization_param_dir,
        pcd_map_path=pcd_map_path,
        set_initial_pose=set_initial_pose,
        initial_pose_values=initial_pose_values,
        enable_timer_publishing=enable_timer_publishing,
        pose_publish_frequency=pose_publish_frequency,
    )

    actions = []
    if pcd_map_path:
        actions.append(LogInfo(msg=f'Using pointcloud map override: {pcd_map_path}'))
    if set_initial_pose is True:
        actions.append(
            LogInfo(
                msg=(
                    'Using initial pose override: '
                    f"({initial_pose_values['initial_pose_x']}, "
                    f"{initial_pose_values['initial_pose_y']}, "
                    f"{initial_pose_values['initial_pose_z']})"
                )
            )
        )

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch),
            launch_arguments={
                'localization_param_dir': effective_localization_params,
                'use_sim_time': LaunchConfiguration('use_sim_time').perform(context),
                'enable_map_odom_tf': LaunchConfiguration(
                    'localizer_enable_map_odom_tf'
                ).perform(context),
                'global_frame_id': global_frame_id,
                'odom_frame_id': odom_frame_id,
                'base_frame_id': base_frame_id,
                'cloud_topic': LaunchConfiguration('cloud_topic').perform(context),
                'twist_topic': LaunchConfiguration('twist_topic').perform(context),
                'imu_topic': LaunchConfiguration('imu_topic').perform(context),
                'publish_lidar_tf': LaunchConfiguration('publish_lidar_tf').perform(context),
                'lidar_frame_id': LaunchConfiguration('lidar_frame_id').perform(context),
                'lidar_tf_x': LaunchConfiguration('lidar_tf_x').perform(context),
                'lidar_tf_y': LaunchConfiguration('lidar_tf_y').perform(context),
                'lidar_tf_z': LaunchConfiguration('lidar_tf_z').perform(context),
                'lidar_tf_roll': LaunchConfiguration('lidar_tf_roll').perform(context),
                'lidar_tf_pitch': LaunchConfiguration('lidar_tf_pitch').perform(context),
                'lidar_tf_yaw': LaunchConfiguration('lidar_tf_yaw').perform(context),
                'publish_imu_tf': LaunchConfiguration('publish_imu_tf').perform(context),
                'imu_frame_id': LaunchConfiguration('imu_frame_id').perform(context),
                'imu_tf_x': LaunchConfiguration('imu_tf_x').perform(context),
                'imu_tf_y': LaunchConfiguration('imu_tf_y').perform(context),
                'imu_tf_z': LaunchConfiguration('imu_tf_z').perform(context),
                'imu_tf_roll': LaunchConfiguration('imu_tf_roll').perform(context),
                'imu_tf_pitch': LaunchConfiguration('imu_tf_pitch').perform(context),
                'imu_tf_yaw': LaunchConfiguration('imu_tf_yaw').perform(context),
            }.items(),
        )
    )
    return actions


def generate_launch_description():
    home = os.environ.get('HOME', '/home/roboworks')

    default_map_dir = os.path.join(home, 'g1_ws', 'map')
    default_map_name = 'map'
    default_localization_param_dir = os.path.join(
        get_package_share_directory('g1_navigation'),
        'params',
        'nav2_ndt_g1.yaml',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    launch_map_server = LaunchConfiguration('launch_map_server')
    launch_localization = LaunchConfiguration('launch_localization')

    map_dir = LaunchConfiguration('map_dir')
    map_name = LaunchConfiguration('map_name')
    map_yaml_file = [map_dir, '/', map_name, '.yaml']
    pcd_map_path = [map_dir, '/', map_name, '.pcd']
    cloud_topic = LaunchConfiguration('cloud_topic')
    pointcloud_topic = LaunchConfiguration('pointcloud_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    twist_topic = LaunchConfiguration('twist_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')

    map_frame = LaunchConfiguration('map_frame')
    odom_frame = LaunchConfiguration('odom_frame')
    localization_base_frame = LaunchConfiguration('localization_base_frame')
    robot_base_frame = LaunchConfiguration('robot_base_frame')
    publish_localizer_pose_odom = LaunchConfiguration('publish_localizer_pose_odom')
    publish_identity_odom = LaunchConfiguration('publish_identity_odom')
    identity_odom_rate_hz = LaunchConfiguration('identity_odom_rate_hz')
    publish_twist_odom = LaunchConfiguration('publish_twist_odom')
    twist_odom_max_dt_sec = LaunchConfiguration('twist_odom_max_dt_sec')
    publish_cmd_vel_odom = LaunchConfiguration('publish_cmd_vel_odom')
    cmd_vel_odom_rate_hz = LaunchConfiguration('cmd_vel_odom_rate_hz')
    enable_reinitialization_supervisor = LaunchConfiguration('enable_reinitialization_supervisor')
    reinitialization_supervisor_use_latest_pose = LaunchConfiguration(
        'reinitialization_supervisor_use_latest_pose'
    )
    reinitialization_supervisor_publish_count = LaunchConfiguration(
        'reinitialization_supervisor_publish_count'
    )
    reinitialization_supervisor_publish_interval_sec = LaunchConfiguration(
        'reinitialization_supervisor_publish_interval_sec'
    )
    reinitialization_supervisor_cooldown_sec = LaunchConfiguration(
        'reinitialization_supervisor_cooldown_sec'
    )

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
            'pcd_map_path',
            default_value=pcd_map_path,
            description='PointCloud map for lidar_localization_ros2.',
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
        DeclareLaunchArgument('lidar_tf_x', default_value='0.0'),
        DeclareLaunchArgument('lidar_tf_y', default_value='0.0'),
        DeclareLaunchArgument('lidar_tf_z', default_value='0.0'),
        DeclareLaunchArgument('lidar_tf_roll', default_value='0.0'),
        DeclareLaunchArgument('lidar_tf_pitch', default_value='0.0'),
        DeclareLaunchArgument('lidar_tf_yaw', default_value='0.0'),
        DeclareLaunchArgument('publish_imu_tf', default_value='false'),
        DeclareLaunchArgument('imu_tf_x', default_value='0.0'),
        DeclareLaunchArgument('imu_tf_y', default_value='0.0'),
        DeclareLaunchArgument('imu_tf_z', default_value='0.0'),
        DeclareLaunchArgument('imu_tf_roll', default_value='0.0'),
        DeclareLaunchArgument('imu_tf_pitch', default_value='0.0'),
        DeclareLaunchArgument('imu_tf_yaw', default_value='0.0'),
        DeclareLaunchArgument('set_initial_pose', default_value='true'),
        DeclareLaunchArgument('initial_pose_x', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_y', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_z', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_qx', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_qy', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_qz', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_qw', default_value='1.0'),
        DeclareLaunchArgument('use_odom_localization_demo', default_value='false'),
        DeclareLaunchArgument('enable_reinitialization_supervisor', default_value='true'),
        DeclareLaunchArgument('reinitialization_supervisor_use_latest_pose', default_value='false'),
        DeclareLaunchArgument('reinitialization_supervisor_publish_count', default_value='1'),
        DeclareLaunchArgument(
            'reinitialization_supervisor_publish_interval_sec',
            default_value='0.2',
        ),
        DeclareLaunchArgument(
            'reinitialization_supervisor_cooldown_sec',
            default_value='15.0',
        ),
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
        DeclareLaunchArgument('twist_odom_max_dt_sec', default_value='0.5'),
        DeclareLaunchArgument('publish_cmd_vel_odom', default_value='false'),
        DeclareLaunchArgument('cmd_vel_odom_rate_hz', default_value='20.0'),
        DeclareLaunchArgument('obstacle_threshold', default_value='50'),
        DeclareLaunchArgument(
            'footprint',
            default_value="'0.1'",
            description='Robot footprint: radius, length,width, or x1,y1,... vertices.',
        ),
        DeclareLaunchArgument('global_clearance', default_value='0.1'),
        DeclareLaunchArgument('local_clearance', default_value='0.30'),
        DeclareLaunchArgument(
            'free_space_weight',
            default_value='150.0',
            description='Weight for free-cell cost (k/d). 0 disables. Larger values prefer paths away from obstacles.',
        ),
        DeclareLaunchArgument(
            'unknown_cost',
            default_value='0',
            description='Cost assigned to unknown cells (-1 in OccupancyGrid). 0=free, 1-98=traversable with penalty, 99+=impassable.',
        ),
        DeclareLaunchArgument('local_costmap_frame', default_value='odom'),
        DeclareLaunchArgument('local_resolution', default_value='0.05'),
        DeclareLaunchArgument('local_width', default_value='4.0'),
        DeclareLaunchArgument('local_height', default_value='4.0'),
        DeclareLaunchArgument('min_obstacle_height', default_value='0.1'),
        DeclareLaunchArgument('max_obstacle_height', default_value='2.0'),
        DeclareLaunchArgument('min_sensor_range', default_value='0.5'),
        DeclareLaunchArgument(
            'realsense_topic',
            default_value='/head_camera/d455/depth/color/points',
            description='RealSense PointCloud2 topic for local costmap. Empty to disable.',
        ),
        DeclareLaunchArgument('realsense_min_obstacle_height', default_value='0.02'),
        DeclareLaunchArgument('realsense_max_obstacle_height', default_value='1.0'),
        DeclareLaunchArgument('realsense_min_sensor_range',    default_value='0.2'),
        DeclareLaunchArgument('use_smoothing', default_value='true'),
        DeclareLaunchArgument('obstacle_cost_weight', default_value='5.0'),
        DeclareLaunchArgument('planner_obstacle_threshold', default_value='99'),
        DeclareLaunchArgument('path_obstacle_threshold', default_value='75'),
        DeclareLaunchArgument('local_plan_frequency', default_value='5.0'),
        DeclareLaunchArgument('local_plan_horizon', default_value='0.5'),
        DeclareLaunchArgument('local_plan_obstacle_threshold', default_value='70'),
        DeclareLaunchArgument('use_local_smoothing', default_value='true'),
        DeclareLaunchArgument('goal_tolerance', default_value='0.15'),
        DeclareLaunchArgument('goal_yaw_tolerance', default_value='0.05'),
        DeclareLaunchArgument('replan_cooldown', default_value='2.0'),
        DeclareLaunchArgument('augmented_costmap_topic', default_value='/augmented_local_costmap'),
        DeclareLaunchArgument('obstacle_decay_rate', default_value='5'),
        DeclareLaunchArgument('decay_frequency', default_value='2.0'),
        DeclareLaunchArgument('path_topic', default_value='/execute_path_plan'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('execute_topic', default_value='/execute_local_planner'),
        DeclareLaunchArgument('lookahead_distance', default_value='0.5'),
        DeclareLaunchArgument('linear_velocity', default_value='0.4'),
        DeclareLaunchArgument('max_angular_velocity', default_value='1.0'),
        DeclareLaunchArgument('max_path_deviation', default_value='0.3'),
        DeclareLaunchArgument('slowdown_distance', default_value='0.6'),
        DeclareLaunchArgument('min_linear_velocity', default_value='0.21'),
        DeclareLaunchArgument('max_linear_velocity', default_value='0.8'),
        DeclareLaunchArgument('control_frequency', default_value='20.0'),
        DeclareLaunchArgument('holonomic', default_value='true'),
        DeclareLaunchArgument('max_linear_acceleration', default_value='0.5'),
        DeclareLaunchArgument('max_angular_acceleration', default_value='2.0'),
    ]

    localization = GroupAction(
        condition=IfCondition(launch_localization),
        actions=[
            Node(
                package='lidar_localization_ros2',
                executable='publish_odom_from_localization.py',
                name='odom_from_localization_publisher',
                output='screen',
                condition=IfCondition(publish_localizer_pose_odom),
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'pose_topic': '/localization/pose_with_covariance',
                    'odom_topic': odom_topic,
                    'odom_frame_id': odom_frame,
                    'base_frame_id': localization_base_frame,
                    'publish_tf': True,
                }],
            ),
            Node(
                package='lidar_localization_ros2',
                executable='publish_identity_odom.py',
                name='identity_odom_publisher',
                output='screen',
                condition=IfCondition(publish_identity_odom),
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'odom_topic': odom_topic,
                    'odom_frame_id': odom_frame,
                    'base_frame_id': localization_base_frame,
                    'rate_hz': identity_odom_rate_hz,
                    'publish_tf': True,
                }],
            ),
            Node(
                package='lidar_localization_ros2',
                executable='publish_odom_from_twist.py',
                name='twist_odom_publisher',
                output='screen',
                condition=IfCondition(publish_twist_odom),
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'twist_topic': twist_topic,
                    'odom_topic': odom_topic,
                    'odom_frame_id': odom_frame,
                    'base_frame_id': localization_base_frame,
                    'max_dt_sec': twist_odom_max_dt_sec,
                    'publish_tf': True,
                }],
            ),
            Node(
                package='lidar_localization_ros2',
                executable='publish_cmd_vel_odom.py',
                name='cmd_vel_odom_publisher',
                output='screen',
                condition=IfCondition(publish_cmd_vel_odom),
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'cmd_vel_topic': cmd_vel_topic,
                    'odom_topic': odom_topic,
                    'odom_frame_id': odom_frame,
                    'base_frame_id': localization_base_frame,
                    'rate_hz': cmd_vel_odom_rate_hz,
                    'publish_tf': True,
                }],
            ),
            Node(
                package='lidar_localization_ros2',
                executable='republish_initialpose_on_reinit.py',
                name='reinitialization_supervisor',
                output='screen',
                condition=IfCondition(enable_reinitialization_supervisor),
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'request_topic': '/reinitialization_requested',
                    'initialpose_topic': '/initialpose',
                    'pose_topic': '/localization/pose_with_covariance',
                    'global_frame_id': map_frame,
                    'use_latest_pose': reinitialization_supervisor_use_latest_pose,
                    'publish_count': reinitialization_supervisor_publish_count,
                    'publish_interval_sec': reinitialization_supervisor_publish_interval_sec,
                    'republish_cooldown_sec': reinitialization_supervisor_cooldown_sec,
                    'initial_pose_x': LaunchConfiguration('initial_pose_x'),
                    'initial_pose_y': LaunchConfiguration('initial_pose_y'),
                    'initial_pose_z': LaunchConfiguration('initial_pose_z'),
                    'initial_pose_qx': LaunchConfiguration('initial_pose_qx'),
                    'initial_pose_qy': LaunchConfiguration('initial_pose_qy'),
                    'initial_pose_qz': LaunchConfiguration('initial_pose_qz'),
                    'initial_pose_qw': LaunchConfiguration('initial_pose_qw'),
                }],
            ),
            OpaqueFunction(function=_start_lidar_localization),
        ],
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
            'clearance': LaunchConfiguration('global_clearance'),
            'free_space_weight': LaunchConfiguration('free_space_weight'),
            'unknown_cost': LaunchConfiguration('unknown_cost'),
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
            'goal_yaw_tolerance': LaunchConfiguration('goal_yaw_tolerance'),
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
            'goal_yaw_tolerance': LaunchConfiguration('goal_yaw_tolerance'),
            'max_path_deviation': LaunchConfiguration('max_path_deviation'),
            'slowdown_distance': LaunchConfiguration('slowdown_distance'),
            'min_linear_velocity': LaunchConfiguration('min_linear_velocity'),
            'max_linear_velocity': LaunchConfiguration('max_linear_velocity'),
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
