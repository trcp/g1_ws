"""MoveIt のモデル・設定を渡した RViz2 のみを起動する."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import tempfile
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
    parameters = [
        {'robot_description': description, 'robot_description_semantic': semantic},
        {'robot_description_kinematics': read_yaml('kinematics.yaml')},
        {'robot_description_planning': limits},
        {'ompl': read_yaml('ompl_planning.yaml')},
        read_yaml('mock_moveit_controllers.yaml' if mock else 'moveit_controllers.yaml'),
        {'planning_pipelines': ['ompl'], 'default_planning_pipeline': 'ompl',
         'use_sim_time': sim_time},
    ]
    rviz_config = os.path.join(share, 'rviz', 'moveit.rviz')
    cleanup = []
    if mock:
        with open(rviz_config, encoding='utf-8') as stream:
            config = yaml.safe_load(stream)
        for display in config['Visualization Manager']['Displays']:
            if display.get('Class') == 'moveit_rviz_plugin/MotionPlanning':
                display['Class'] = 'erasers_g1_moveit/CoordinatedMotionPlanning'
        with tempfile.NamedTemporaryFile(mode='w', suffix='.rviz', delete=False) as stream:
            yaml.safe_dump(config, stream, allow_unicode=True)
            rviz_config = stream.name

        def remove_config(_context):
            if os.path.exists(rviz_config):
                os.unlink(rviz_config)
            return []

        cleanup.append(RegisterEventHandler(OnShutdown(
            on_shutdown=[OpaqueFunction(function=remove_config)])))
    return cleanup + [Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_config],
        parameters=parameters, condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen', emulate_tty=True,
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
    ld.add_action(DeclareLaunchArgument('use_rviz', default_value='true'))

    # 起動アクションとノード
    ld.add_action(OpaqueFunction(function=_node))

    return ld
