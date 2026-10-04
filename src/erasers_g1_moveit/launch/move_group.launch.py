"""MoveIt の move_group ノードのみを起動する."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro
import yaml


def _node(context):
    mock = LaunchConfiguration('use_mock_hardware').perform(context).lower() == 'true'
    sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    model_path = LaunchConfiguration('robot_description_path').perform(context)
    description = xacro.process_file(
        model_path, mappings={'use_mock_hardware': str(mock).lower()}).toxml()
    share = get_package_share_directory('erasers_g1_moveit')

    def read_yaml(name):
        with open(os.path.join(share, 'config', name), encoding='utf-8') as stream:
            return yaml.safe_load(stream)

    with open(os.path.join(share, 'config', 'g1.srdf'), encoding='utf-8') as stream:
        semantic = stream.read()
    limits = read_yaml('joint_limits.yaml')
    if mock:
        for joint, values in read_yaml('mock_joint_limits.yaml')['joint_limits'].items():
            limits['joint_limits'].setdefault(joint, {}).update(values)
    ompl = read_yaml('ompl_planning.yaml')
    if mock:
        # 複数手先の経路制約も関節空間で表現する。
        for group in ('arm_both', 'arm_both_with_waist'):
            ompl[group]['enforce_joint_model_state_space'] = True
    parameters = [
        {'robot_description': description, 'robot_description_semantic': semantic},
        {'robot_description_kinematics': read_yaml('kinematics.yaml')},
        {'robot_description_planning': limits},
        {'ompl': ompl},
        read_yaml('mock_moveit_controllers.yaml' if mock else 'moveit_controllers.yaml'),
        {'planning_pipelines': ['ompl'], 'default_planning_pipeline': 'ompl',
         'use_sim_time': sim_time},
    ]
    parameters.append({
        'publish_robot_description': True, 'publish_robot_description_semantic': True,
        'trajectory_execution.allowed_start_tolerance': 0.05,
    })
    if mock:
        parameters.append({
            'constraint_samplers': 'erasers_g1_moveit/DualArmConstraintSampler',
            # RViz へ送る現在状態も 30 Hz を上限に更新する。
            'publish_planning_scene': True,
            'publish_state_updates': True,
            'publish_geometry_updates': True,
            'publish_transforms_updates': True,
            'publish_planning_scene_hz': 30.0,
        })
    return [Node(
        package='moveit_ros_move_group', executable='move_group', name='move_group',
        parameters=parameters, output='screen', emulate_tty=True,
    )]


def generate_launch_description():
    ld = LaunchDescription()

    # 既定値
    share = get_package_share_directory('erasers_g1_description')
    default_model = os.path.join(share, 'urdf', 'erasers_g1.urdf.xacro')

    # 起動時設定は _node で評価する。

    # 起動引数
    ld.add_action(DeclareLaunchArgument('robot_description_path', default_value=default_model))
    ld.add_action(DeclareLaunchArgument('use_mock_hardware', default_value='false'))
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false'))

    # 起動アクションとノード
    ld.add_action(OpaqueFunction(function=_node))

    return ld
