#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory
import os


MAP_DIR = os.environ.get('HOME', '/home/roboworks') + '/g1_ws/map'


def generate_launch_description():
    ld = LaunchDescription()


    default_localization_param_dir = os.path.join(get_package_share_directory('g1_navigation'), 'params', 'nav2_ndt_g1.yaml')


    enable_reinitialization_supervisor = LaunchConfiguration('enable_reinitialization_supervisor')
    set_initial_pose = LaunchConfiguration('set_initial_pose')
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_z = LaunchConfiguration('initial_pose_z')
    initial_pose_qx = LaunchConfiguration('initial_pose_qx')
    initial_pose_qy = LaunchConfiguration('initial_pose_qy')
    initial_pose_qz = LaunchConfiguration('initial_pose_qz')
    initial_pose_qw = LaunchConfiguration('initial_pose_qw')
    pcd_map_path = LaunchConfiguration('pcd_map_path')
    localization_param_dir = LaunchConfiguration('localization_param_dir')
    cloud_topic = LaunchConfiguration('cloud_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    lidar_frame_id = LaunchConfiguration('lidar_frame_id')
    imu_frame_id = LaunchConfiguration('imu_frame_id')
    pointcloud_topic = LaunchConfiguration('pointcloud_topic')
    publish_lidar_tf = LaunchConfiguration('publish_lidar_tf')
    publish_imu_tf = LaunchConfiguration('publish_imu_tf')
    

    declare_enable_reinitialization_supervisor = DeclareLaunchArgument(
        'enable_reinitialization_supervisor', default_value='true',
        description='Enable reinitialization supervisor.'
    )
    declare_set_initial_pose = DeclareLaunchArgument(
        'set_initial_pose', default_value='true',
        description='Enable setting the initial pose of the robot.'
    )
    declare_initial_pose_x = DeclareLaunchArgument(
        'initial_pose_x', default_value='0.0',
        description='Initial pose X coordinate.'
    )
    declare_initial_pose_y = DeclareLaunchArgument(
        'initial_pose_y', default_value='0.0',
        description='Initial pose Y coordinate.'
    )
    declare_initial_pose_z = DeclareLaunchArgument(
        'initial_pose_z', default_value='0.0',
        description='Initial pose Z coordinate.'
    )
    declare_initial_pose_qx = DeclareLaunchArgument(
        'initial_pose_qx', default_value='0.0',
        description='Initial pose orientation quaternion X.'
    )
    declare_initial_pose_qy = DeclareLaunchArgument(
        'initial_pose_qy', default_value='0.0',
        description='Initial pose orientation quaternion Y.'
    )
    declare_initial_pose_qz = DeclareLaunchArgument(
        'initial_pose_qz', default_value='0.0',
        description='Initial pose orientation quaternion Z.'
    )
    declare_initial_pose_qw = DeclareLaunchArgument(
        'initial_pose_qw', default_value='1.0',
        description='Initial pose orientation quaternion W.'
    )
    declare_pcd_map_path = DeclareLaunchArgument(
        'pcd_map_path', default_value=MAP_DIR+'/map220.pcd',
        description='Full path to the PCD map file.'
    )
    declare_localization_param_dir = DeclareLaunchArgument(
        'localization_param_dir', default_value=default_localization_param_dir,
        description='Full path to the localization parameter file.'
    )
    declare_cloud_topic = DeclareLaunchArgument(
        'cloud_topic', default_value='/utlidar/cloud_livox_mid360',
        description='Topic name for input point cloud.'
    )
    declare_imu_topic = DeclareLaunchArgument(
        'imu_topic', default_value='/utlidar/imu_livox_mid360',
        description='Topic name for input IMU data.'
    )
    declare_lidar_frame_id = DeclareLaunchArgument(
        'lidar_frame_id', default_value='livox_frame',
        description='Frame ID for the LiDAR sensor.'
    )
    declare_imu_frame_id = DeclareLaunchArgument(
        'imu_frame_id', default_value='livox_frame',
        description='Frame ID for the IMU sensor.'
    )
    declare_pointcloud_topic = DeclareLaunchArgument(
        'pointcloud_topic', default_value='/utlidar/cloud_livox_mid360',
        description='Topic name for visualization/secondary point cloud data.'
    )
    declare_publish_lidar_tf = DeclareLaunchArgument(
        'publish_lidar_tf', default_value='false',
        description='Whether to publish the LiDAR TF from the localization node.'
    )
    declare_publish_imu_tf = DeclareLaunchArgument(
        'publish_imu_tf', default_value='false',
        description='Whether to publish the IMU TF from the localization node.'
    )

    ld.add_action(declare_enable_reinitialization_supervisor)
    ld.add_action(declare_set_initial_pose)
    ld.add_action(declare_initial_pose_x)
    ld.add_action(declare_initial_pose_y)
    ld.add_action(declare_initial_pose_z)
    ld.add_action(declare_initial_pose_qx)
    ld.add_action(declare_initial_pose_qy)
    ld.add_action(declare_initial_pose_qz)
    ld.add_action(declare_initial_pose_qw)
    ld.add_action(declare_pcd_map_path)
    ld.add_action(declare_localization_param_dir)
    ld.add_action(declare_cloud_topic)
    ld.add_action(declare_imu_topic)
    ld.add_action(declare_lidar_frame_id)
    ld.add_action(declare_imu_frame_id)
    ld.add_action(declare_pointcloud_topic)
    ld.add_action(declare_publish_lidar_tf)
    ld.add_action(declare_publish_imu_tf)


    lidar_localization_ros2_nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('lidar_localization_ros2'),
            'launch', 'nav2_navigation.launch.py')
        ]),
        launch_arguments={
            'enable_reinitialization_supervisor': enable_reinitialization_supervisor,
            'set_initial_pose': set_initial_pose,
            'initial_pose_x': initial_pose_x,
            'initial_pose_y': initial_pose_y,
            'initial_pose_z': initial_pose_z,
            'initial_pose_qx': initial_pose_qx,
            'initial_pose_qy': initial_pose_qy,
            'initial_pose_qz': initial_pose_qz,
            'initial_pose_qw': initial_pose_qw,
            'pcd_map_path': pcd_map_path,
            'localization_param_dir': localization_param_dir,
            'cloud_topic': cloud_topic,
            'imu_topic': imu_topic,
            'lidar_frame_id': lidar_frame_id,
            'imu_frame_id': imu_frame_id,
            'pointcloud_topic': pointcloud_topic,
            'publish_lidar_tf': publish_lidar_tf,
            'publish_imu_tf': publish_imu_tf
        }.items()
    )
    ld.add_action(lidar_localization_ros2_nav2_navigation)


    return ld
