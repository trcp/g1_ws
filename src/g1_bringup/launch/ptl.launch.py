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
            ('cloud_in', '/utlidar/cloud_livox_mid360'),
            #('cloud_in', '/livox/lidar'),
        ]
    )

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
    ld.add_action(rviz_node)

    return ld
