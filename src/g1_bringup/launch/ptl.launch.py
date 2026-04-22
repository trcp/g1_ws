import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():

    # configurations
    pkg_g1_bringup = get_package_share_directory('g1_bringup')
    pkg_g1_description = get_package_share_directory('g1_description')

    ptl_params_file = os.path.join(pkg_g1_bringup, 'params', 'ptl.yaml')
    rviz_config_file = os.path.join(pkg_g1_description, 'rviz', 'g1.rviz')


    # nodes
    ptl_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[ptl_params_file],
        remappings=[
            ('cloud_in', '/utlidar/cloud_livox_mid360_fixed'),
            #('cloud_in', '/livox/lidar'),
        ]
    )

    '''
    camera_bridge_node = Node(
        package='erasers_g1_common_cpp',
        executable='camera_bridge',
        name='camera_bridge',
        output='screen',
        remappings=[
            ('input_image', '/head_camera/d455/aligned_depth_to_color/image_raw'),
            ('input_camera', '/head_camera/d455/aligned_depth_to_color/camera_info'),
            ('output0_image', '/lor/image_raw'),
            ('output0_camera', '/lor/camera_info'),
            ('output1_image', '/sam3/image_raw'),
            ('output1_camera', '/sam3/camera_info'),
        ]
    )
    '''

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )


    # launchers
    ld = LaunchDescription()

    #ld.add_action(ptl_node)
    #ld.add_action(camera_bridge_node)
    ld.add_action(rviz_node)

    return ld
