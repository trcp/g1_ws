"""URDF と同じモードで ros2_control を起動する."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import tempfile
import xml.etree.ElementTree as ET
import xacro
import yaml


def _controllers(context):
    mock = LaunchConfiguration('use_mock_hardware').perform(context).lower() == 'true'
    sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    model_path = LaunchConfiguration('robot_model').perform(context)
    description = xacro.process_file(
        model_path, mappings={'use_mock_hardware': str(mock).lower()}).toxml()
    model = ET.fromstring(description)
    plugins = [node.text for node in model.findall('./ros2_control/hardware/plugin')]
    if mock and (not plugins or any(p != 'mock_components/GenericSystem' for p in plugins)):
        raise RuntimeError('モック用 URDF に実機 HW が含まれています')
    share = get_package_share_directory('erasers_g1_hw_controller')
    config = os.path.join(
        share, 'config', 'mock_controllers.yaml' if mock else 'g1_controllers.yaml')
    cleanup = []
    if mock:
        with open(config, encoding='utf-8') as stream:
            params = yaml.safe_load(stream)
        controlled = {j.get('name') for j in model.findall('./ros2_control/joint')}
        missing_states = {
            j.get('name') for j in model.findall('joint')
            if j.get('type') not in ('fixed', 'floating') and j.get('name') not in controlled
        }
        # 23 軸モデルにない 6 軸も本体の 29 フィールドとして保持する。
        reserved = {'waist_roll_joint', 'waist_pitch_joint',
                    'left_wrist_pitch_joint', 'left_wrist_yaw_joint',
                    'right_wrist_pitch_joint', 'right_wrist_yaw_joint'}
        params['joint_state_broadcaster']['ros__parameters'].update({
            'extra_joints': sorted(missing_states | (reserved - controlled)),
            'use_sim_time': sim_time})
        for name in ('controller_manager', 'upper_body_controller', 'head_controller'):
            params[name]['ros__parameters']['use_sim_time'] = sim_time
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as stream:
            yaml.safe_dump(params, stream)
            config = stream.name

        def remove_config(_context):
            if os.path.exists(config):
                os.unlink(config)
            return []

        cleanup.append(RegisterEventHandler(OnShutdown(
            on_shutdown=[OpaqueFunction(function=remove_config)])))

    manager = Node(
        package='controller_manager', executable='ros2_control_node',
        parameters=[config, {'robot_description': description, 'use_sim_time': sim_time}],
        output='screen', emulate_tty=True,
    )
    names = ['joint_state_broadcaster', 'upper_body_controller']
    if mock:
        names.append('head_controller')
    spawners = [Node(
        package='controller_manager', executable='spawner',
        arguments=[name, '--controller-manager', '/controller_manager',
                   '--param-file', config],
        parameters=[{'use_sim_time': sim_time}], output='screen', emulate_tty=True,
    ) for name in names]
    return cleanup + [manager] + spawners


def generate_launch_description():
    ld = LaunchDescription()

    # 既定値
    share = get_package_share_directory('erasers_g1_description')
    default_model = os.path.join(share, 'urdf', 'erasers_g1.urdf.xacro')

    # 起動時設定は _controllers で評価する。

    # 起動引数
    ld.add_action(DeclareLaunchArgument('robot_model', default_value=default_model))
    ld.add_action(DeclareLaunchArgument('use_mock_hardware', default_value='false'))
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false'))

    # 起動アクションとノード
    ld.add_action(OpaqueFunction(function=_controllers))

    return ld
