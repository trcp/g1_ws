"""モック構成の回帰試験。G1_MOCK_INTEGRATION=1 で実ノード検証を追加する."""

from contextlib import ExitStack
import importlib
import importlib.util
import math
import os
import re
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET

import pytest
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory

MOVEIT = Path(get_package_share_directory('erasers_g1_moveit'))
DESCRIPTION = Path(get_package_share_directory('erasers_g1_description'))
GROUPS = {
    'arm_left': 5, 'arm_left_with_waist': 6,
    'arm_right': 5, 'arm_right_with_waist': 6,
    'arm_both': 10, 'arm_both_with_waist': 11, 'head': 2,
}


def model(mock=True):
    return ET.fromstring(xacro.process_file(
        str(DESCRIPTION / 'urdf/erasers_g1.urdf.xacro'),
        mappings={'use_mock_hardware': str(mock).lower()}).toxml())


def states():
    srdf = ET.parse(MOVEIT / 'config/g1.srdf').getroot()
    return {e.get('group'): {j.get('name'): float(j.get('value')) for j in e.findall('joint')}
            for e in srdf.findall('group_state') if e.get('name') == 'home'}


def test_hardware_mode_isolation():
    fake = model()
    assert {p.text for p in fake.findall('./ros2_control/hardware/plugin')} == {
        'mock_components/GenericSystem'}
    assert len(fake.findall('./ros2_control/joint')) == 13
    real = model(False)
    assert [p.text for p in real.findall('./ros2_control/hardware/plugin')] == [
        'erasers_g1_hw_controller/G1UpperBodyHW']
    assert len(real.findall('./ros2_control/joint')) == 11


def test_groups_and_pick_ik():
    groups = states()
    assert {name: len(groups[name]) for name in GROUPS} == GROUPS
    assert 'waist_yaw_joint' not in groups['arm_both']
    assert set(groups['arm_both_with_waist']) == set(groups['arm_both']) | {'waist_yaw_joint'}
    assert set(groups['arm_left']).isdisjoint(groups['arm_right'])
    config = yaml.safe_load((MOVEIT / 'config/kinematics.yaml').read_text())
    assert set(config) == {'arm_left', 'arm_left_with_waist', 'arm_right', 'arm_right_with_waist'}
    for params in config.values():
        assert params['kinematics_solver'] == 'pick_ik/PickIkPlugin'
        assert params['rotation_scale'] == 0.05
        assert params['position_scale'] == 1.0
        assert params['position_threshold'] == 0.001
        assert params['orientation_threshold'] == 0.1
        assert params['mode'] == 'global'


def test_head_action_mapping():
    hw = Path(get_package_share_directory('erasers_g1_hw_controller'))
    config = yaml.safe_load((hw / 'config/mock_controllers.yaml').read_text())
    head = config['head_controller']['ros__parameters']
    assert head['joints'] == ['xl330_joint', 'd455_joint']
    assert head['command_interfaces'] == head['state_interfaces'] == ['position']
    assert not head['allow_partial_joints_goal']
    controllers = yaml.safe_load((MOVEIT / 'config/mock_moveit_controllers.yaml').read_text())
    action = controllers['moveit_simple_controller_manager']['head_controller']
    assert action['type'] == 'FollowJointTrajectory'
    assert action['action_ns'] == 'follow_joint_trajectory'
    assert action['joints'] == head['joints']


def test_sampler_registration_and_mock_scope(monkeypatch):
    from launch import LaunchContext

    spec = importlib.util.spec_from_file_location(
        'mock_move_group_launch', MOVEIT / 'launch/move_group.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    captured = []
    monkeypatch.setattr(module, 'Node', lambda **kwargs: captured.append(kwargs))
    for mock in (False, True):
        context = LaunchContext()
        context.launch_configurations.update({
            'use_mock_hardware': str(mock).lower(), 'use_sim_time': 'true',
            'robot_description_path': str(DESCRIPTION / 'urdf/erasers_g1.urdf.xacro')})
        module._node(context)
        parameters = {}
        for values in captured[-1]['parameters']:
            parameters.update(values)
        assert captured[-1]['executable'] == 'move_group'
        assert ('constraint_samplers' in parameters) == mock
        if mock:
            assert parameters['constraint_samplers'] == (
                'erasers_g1_moveit/DualArmConstraintSampler')
            assert parameters['publish_planning_scene_hz'] == 30.0
        for group in ('arm_both', 'arm_both_with_waist'):
            assert parameters['ompl'][group].get(
                'enforce_joint_model_state_space', False) == mock
    plugin = ET.parse(MOVEIT / 'constraint_sampler_plugins.xml').getroot()
    assert plugin.find('class').get('name') == (
        'erasers_g1_moveit/DualArmConstraintSampler')
    assert (MOVEIT.parents[1] / 'lib/libdual_arm_constraint_sampler.so').is_file()


def test_api_dual_goal_and_group_mapping():
    from rclpy.clock import Clock
    from types import SimpleNamespace
    module = importlib.import_module('erasers_g1_api.robot_control')
    cls = next(value for value in vars(module).values()
               if isinstance(value, type) and 'move_dual_abs' in value.__dict__)
    arm = object.__new__(cls)
    arm.node = SimpleNamespace(get_clock=lambda: Clock())
    setattr(arm, '_' + cls.__name__ + '__srdf_group_states', None)
    setattr(arm, '_' + cls.__name__ + '__joint_states', {})
    goals = []
    arm._send_move_group_goal = lambda goal, wait: goals.append(goal) or True

    def reject_independent_ik(*args, **kwargs):
        pytest.fail('双腕要求を独立した IK 解へ分解してはいけません')

    arm._solve_ik = reject_independent_ik
    assert arm.move_dual_abs(lx=0.2, ly=0.2, rx=0.2, ry=-0.2)
    request = goals[-1].request
    assert request.group_name == 'arm_both_with_waist'
    constraint = request.goal_constraints[0]
    assert not constraint.joint_constraints
    assert {p.link_name for p in constraint.position_constraints} == {
        'left_amazing_hand', 'right_amazing_hand'}
    assert cls._canonical_group('upper_body') == 'arm_both_with_waist'
    for name, count in GROUPS.items():
        assert arm.joint_control(planning_group=name)
        assert len(goals[-1].request.goal_constraints[0].joint_constraints) == count


def test_api_ik_filters_reserved_fields(monkeypatch):
    from concurrent.futures import Future
    from geometry_msgs.msg import PoseStamped
    from moveit_msgs.srv import GetPositionIK
    from types import SimpleNamespace

    module = importlib.import_module('erasers_g1_api.robot_control')
    arm = object.__new__(module.ArmControl)
    arm.node = object()
    arm._ArmControl__srdf_group_states = None
    # 23 軸 URDF にない予約フィールドを含む受信状態を模擬する。
    received = dict(states()['arm_both_with_waist'])
    received['left_wrist_pitch_joint'] = 0.7
    received['waist_roll_joint'] = 0.2
    arm._ArmControl__joint_states = received
    requests = []

    def call_ik(request):
        requests.append(request)
        response = GetPositionIK.Response()
        response.error_code.val = 1
        response.solution.joint_state.name = list(received)
        response.solution.joint_state.position = list(received.values())
        future = Future()
        future.set_result(response)
        return future

    arm._ArmControl__ik_cli = SimpleNamespace(call_async=call_ik)
    monkeypatch.setattr(module.rclpy, 'spin_until_future_complete', lambda *a, **kw: None)
    for group in ('arm_left', 'arm_left_with_waist', 'arm_right', 'arm_right_with_waist'):
        solved = arm._solve_ik(PoseStamped(), group)
        request = requests[-1].ik_request
        assert request.robot_state.is_diff
        assert request.avoid_collisions
        assert set(request.robot_state.joint_state.name) == set(states()[group])
        assert set(solved) == set(states()[group])
        assert 'waist_roll_joint' not in request.robot_state.joint_state.name


def wait_for(node, predicate, timeout=30.0):
    import rclpy
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return
    raise AssertionError('待機期限を超過しました')


def call(node, client, request, timeout=30.0):
    assert client.wait_for_service(timeout_sec=timeout), client.srv_name
    future = client.call_async(request)
    wait_for(node, future.done, timeout)
    return future.result()


def execute(node, client, goal):
    assert client.wait_for_server(timeout_sec=15)
    future = client.send_goal_async(goal)
    wait_for(node, future.done)
    handle = future.result()
    assert handle.accepted
    result = handle.get_result_async()
    wait_for(node, result.done, 45.0)
    wrapped = result.result()
    assert wrapped.status == 4, wrapped
    return wrapped.result


def stop_process(process, log_path):
    """起動途中の失敗でも Launch と子プロセスを終了する."""
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
    print('launch_exit', process.pid, process.returncode, flush=True)
    output = log_path.read_text()
    started = set(re.findall(r'process started with pid \[(\d+)\]', output))
    ended = set(re.findall(r'process has finished cleanly \[pid (\d+)\]', output))
    ended.update(re.findall(r'process has died \[pid (\d+),', output))
    crashes = [line for line in output.splitlines() if 'process has died' in line]
    print('process_cleanup', 'started', sorted(started), 'ended', sorted(ended),
          'abnormal_exits', len(crashes), flush=True)
    assert started <= ended, ('終了記録がないプロセス', started - ended)
    assert process.returncode == 0, ('Launch の異常終了', process.returncode)
    assert not crashes, ('子ノードの異常終了', crashes)


@pytest.mark.skipif(os.environ.get('G1_MOCK_INTEGRATION') != '1',
                    reason='実ノード検証は G1_MOCK_INTEGRATION=1 で明示実行する')
@pytest.mark.parametrize('sim_time', [False, True])
def test_virtual_execution(sim_time):
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from erasers_g1_api.robot_control import ArmControl
    from control_msgs.action import FollowJointTrajectory
    from geometry_msgs.msg import Pose
    from moveit_msgs.action import ExecuteTrajectory
    from moveit_msgs.msg import (
        Constraints, JointConstraint, OrientationConstraint, PlanningScene, PositionConstraint)
    from moveit_msgs.srv import GetMotionPlan, GetPositionFK, GetPositionIK, GetStateValidity
    from rosgraph_msgs.msg import Clock
    from rcl_interfaces.srv import GetParameters
    from sensor_msgs.msg import JointState
    from shape_msgs.msg import SolidPrimitive
    from tf2_msgs.msg import TFMessage
    from trajectory_msgs.msg import JointTrajectoryPoint

    with ExitStack() as resources:
        logs = Path(os.environ.get('G1_MOCK_LOG_DIR', 'build/mock_moveit_validation')).resolve()
        logs.mkdir(parents=True, exist_ok=True)
        rviz = os.environ.get('G1_MOCK_RVIZ') == '1' and sim_time
        log_path = logs / ('runtime_sim.log' if sim_time else 'runtime_wall.log')
        stream = resources.enter_context(log_path.open('w'))
        launch_target = ['erasers_g1_bringup', 'sim_bringup.launch.py']
        if rviz:
            # 標準 Plan/Execute の遠隔テスト入口は一時設定だけで有効にする。
            temporary = resources.enter_context(tempfile.TemporaryDirectory())
            wrapper = Path(temporary) / 'verify_rviz.launch.py'
            wrapper.write_text("""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import importlib.util


def rviz(context):
    path = Path(get_package_share_directory('erasers_g1_moveit')) / 'launch/moveit_rviz.launch.py'
    spec = importlib.util.spec_from_file_location('verification_rviz', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    original = module.yaml.safe_load

    def configure(stream):
        data = original(stream)
        if isinstance(data, dict) and 'Visualization Manager' in data:
            for display in data['Visualization Manager']['Displays']:
                if display.get('Class') == 'moveit_rviz_plugin/MotionPlanning':
                    display['MoveIt_Allow_External_Program'] = True
        return data

    module.yaml.safe_load = configure
    try:
        return module._node(context)
    finally:
        module.yaml.safe_load = original


def generate_launch_description():
    bringup = Path(get_package_share_directory('erasers_g1_bringup'))
    description = Path(get_package_share_directory('erasers_g1_description'))
    return LaunchDescription([
        DeclareLaunchArgument('use_mock_hardware', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('robot_description_path',
                              default_value=str(description / 'urdf/erasers_g1.urdf.xacro')),
        GroupAction(scoped=True, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup / 'launch/sim_bringup.launch.py')),
            launch_arguments={'use_rviz': 'false',
                              'use_sim_time': LaunchConfiguration('use_sim_time')}.items())]),
        OpaqueFunction(function=rviz),
    ])
""")
            launch_target = [str(wrapper)]
        command = ['ros2', 'launch'] + launch_target + [
            'use_mock_hardware:=true', 'use_sim_time:=' + str(sim_time).lower(),
            'use_rviz:=' + str(rviz).lower()]
        process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        print('launch_pid=', process.pid, 'stop_command=kill -INT ' + str(process.pid), flush=True)
        resources.callback(stop_process, process, log_path)
        rclpy.init()
        resources.callback(lambda: rclpy.shutdown() if rclpy.ok() else None)
        node = rclpy.create_node(
            'mock_moveit_verifier',
            parameter_overrides=[Parameter('use_sim_time', value=sim_time)])
        resources.callback(node.destroy_node)
        positions = {}
        clocks = []
        frames = set()
        arrivals = {topic: [] for topic in ('joint_states', 'tf', 'planning_scene')}

        def received(topic):
            arrivals[topic].append(time.monotonic())

        def joint_state(msg):
            received('joint_states')
            positions.update(zip(msg.name, msg.position))

        def transform(msg):
            received('tf')
            frames.update(t.child_frame_id for t in msg.transforms)

        subscriptions = [
            node.create_subscription(
                JointState, '/joint_states', joint_state, qos_profile_sensor_data),
            node.create_subscription(Clock, '/clock', lambda msg: clocks.append(msg.clock), 10),
            node.create_subscription(TFMessage, '/tf', transform, 10),
            node.create_subscription(PlanningScene, '/monitored_planning_scene',
                                     lambda msg: received('planning_scene'), 10),
        ]
        for subscription in subscriptions:
            resources.callback(node.destroy_subscription, subscription)
        planner = node.create_client(GetMotionPlan, '/plan_kinematic_path')
        fk = node.create_client(GetPositionFK, '/compute_fk')
        ik = node.create_client(GetPositionIK, '/compute_ik')
        validity = node.create_client(GetStateValidity, '/check_state_validity')
        api = object.__new__(ArmControl)
        api.node = node
        api._ArmControl__joint_states = positions
        api._ArmControl__srdf_group_states = None
        api._ArmControl__ik_cli = ik
        executor = ActionClient(node, ExecuteTrajectory, '/execute_trajectory')
        head = ActionClient(
            node, FollowJointTrajectory, '/head_controller/follow_joint_trajectory')

        for action in (executor, head):
            resources.callback(action.destroy)

        def fill_state(state, values):
            state.is_diff = True
            # URDF に存在する関節だけを MoveIt の要求へ渡す。
            known = {j.get('name') for j in model().findall('joint')}
            state.joint_state.name = [name for name in values if name in known]
            state.joint_state.position = [values[name] for name in state.joint_state.name]

        measured_scene = False

        def motion(group, constraint, path_constraint=None, expect_success=True):
            nonlocal measured_scene
            request = GetMotionPlan.Request()
            request.motion_plan_request.group_name = group
            request.motion_plan_request.pipeline_id = 'ompl'
            request.motion_plan_request.planner_id = 'RRTConnectkConfigDefault'
            request.motion_plan_request.allowed_planning_time = 10.0
            request.motion_plan_request.num_planning_attempts = 5
            request.motion_plan_request.max_velocity_scaling_factor = 0.1
            request.motion_plan_request.max_acceleration_scaling_factor = 0.1
            fill_state(request.motion_plan_request.start_state, positions)
            request.motion_plan_request.goal_constraints = [constraint]
            if path_constraint is not None:
                request.motion_plan_request.path_constraints = path_constraint
            if not expect_success:
                request.motion_plan_request.allowed_planning_time = 1.0
                request.motion_plan_request.num_planning_attempts = 1
            response = call(node, planner, request).motion_plan_response
            if not expect_success:
                assert response.error_code.val != 1
                assert not response.trajectory.joint_trajectory.points
                print('unreachable_rejected', group, response.error_code.val, flush=True)
                return
            for point in response.trajectory.joint_trajectory.points:
                check = GetStateValidity.Request(group_name=group)
                sample = dict(positions)
                sample.update(zip(response.trajectory.joint_trajectory.joint_names,
                                  point.positions))
                fill_state(check.robot_state, sample)
                if path_constraint is not None:
                    check.constraints = path_constraint
                assert call(node, validity, check).valid, ('軌道の妥当性', group)

            assert response.error_code.val == 1, (group, response.error_code.val, str(log_path))
            goal = ExecuteTrajectory.Goal()
            goal.trajectory = response.trajectory
            measure = not measured_scene
            if measure:
                # 連続して動く 5 秒間で、RViz が使う現在状態の配信頻度も測る。
                points = goal.trajectory.joint_trajectory.points
                last = points[-1].time_from_start
                duration = last.sec + last.nanosec / 1e9
                scale = max(1.0, 5.0 / duration)
                for point in points:
                    stamp = point.time_from_start
                    ns = round((stamp.sec * 1_000_000_000 + stamp.nanosec) * scale)
                    stamp.sec, stamp.nanosec = divmod(ns, 1_000_000_000)
                    point.velocities = [v / scale for v in point.velocities]
                    point.accelerations = [a / scale ** 2 for a in point.accelerations]
                arrivals['planning_scene'].clear()
            result = execute(node, executor, goal)
            if measure:
                timestamps = arrivals['planning_scene']
                assert len(timestamps) > 2
                # 起動・停止境界を除いた連続動作中の受信周期。
                steady = [t for t in timestamps if timestamps[0] + 0.5 < t < timestamps[-1] - 0.5]
                assert len(steady) > 2
                hz = (len(steady) - 1) / (steady[-1] - steady[0])
                print('state_rate_hz', 'planning_scene_moving', hz, flush=True)
                assert 28.0 <= hz <= 32.0, hz
                measured_scene = True
            assert result.error_code.val == 1, (group, result.error_code.val)
            end = response.trajectory.joint_trajectory
            wait_for(node, lambda: all(
                abs(positions.get(name, float('inf')) - value) < 0.001
                for name, value in zip(end.joint_names, end.points[-1].positions)))
            check = GetStateValidity.Request(group_name=group, constraints=constraint)
            fill_state(check.robot_state, positions)
            assert call(node, validity, check).valid, ('最終状態の制約・衝突', group)
            print('plan_execute', group, 'SUCCESS', flush=True)
        wait_for(node, lambda: len(positions) >= 31 and len(frames) > 15, 45)
        if sim_time:
            wait_for(node, lambda: len(clocks) >= 2)
            assert (clocks[-1].sec, clocks[-1].nanosec) > (clocks[0].sec, clocks[0].nanosec)
        assert process.poll() is None, log_path
        assert not node.get_publishers_info_by_topic('/arm_sdk')
        assert not node.get_subscriptions_info_by_topic('/lowstate')
        names = node.get_node_names()
        assert names.count('robot_state_publisher') == 1, names
        if rviz:
            wait_for(node, lambda: 'rviz2' in node.get_node_names())
        for name in ['robot_state_publisher', 'controller_manager',
                     'joint_state_broadcaster', 'upper_body_controller',
                     'head_controller', 'move_group'] + (['rviz2'] if rviz else []):
            client = node.create_client(GetParameters, '/' + name + '/get_parameters')
            result = call(node, client, GetParameters.Request(names=['use_sim_time']))
            assert result.values[0].bool_value == sim_time, name
            node.destroy_client(client)
        print('time_parameters', sim_time, 'SUCCESS', flush=True)
        print('topics', len(positions), 'joint_fields', len(frames), 'tf_frames',
              'sim_time', sim_time, 'rviz', rviz, flush=True)

        parameter_client = node.create_client(GetParameters, '/move_group/get_parameters')
        values = call(node, parameter_client, GetParameters.Request(
            names=['constraint_samplers'])).values
        assert values[0].string_value == 'erasers_g1_moveit/DualArmConstraintSampler'
        node.destroy_client(parameter_client)
        # 立ち上がりの蓄積分を除外し、実際の受信周期を測る。
        warmup = time.monotonic() + 1.0
        wait_for(node, lambda: time.monotonic() >= warmup)
        for timestamps in arrivals.values():
            timestamps.clear()
        deadline = time.monotonic() + 5.0
        wait_for(node, lambda: time.monotonic() >= deadline, 7.0)
        for topic, timestamps in arrivals.items():
            if topic == 'planning_scene':
                continue  # 状態変化時のみ配信されるため、動作中にも確認する。
            assert len(timestamps) > 2, topic
            hz = (len(timestamps) - 1) / (timestamps[-1] - timestamps[0])
            print('state_rate_hz', topic, hz, flush=True)
            assert 28.0 <= hz <= 32.0, (topic, hz)

        for index, group in enumerate(name for name in GROUPS if name != 'head'):
            targets = states()[group].copy()
            for name in targets:
                targets[name] = positions[name]
                if 'elbow' in name:
                    targets[name] += 0.08
                elif name == 'waist_yaw_joint':
                    targets[name] += 0.025
            constraints = Constraints()
            for name, value in targets.items():
                jc = JointConstraint(joint_name=name, position=value,
                                     tolerance_above=0.001, tolerance_below=0.001, weight=1.0)
                constraints.joint_constraints.append(jc)
            motion(group, constraints)

        if rviz:
            import copy
            from moveit_msgs.msg import DisplayTrajectory
            from std_msgs.msg import Empty, String
            from visualization_msgs.srv import GetInteractiveMarkers
            from visualization_msgs.msg import InteractiveMarkerFeedback

            marker_services = []

            def find_marker_service():
                marker_services[:] = [
                    name for name, types in node.get_service_names_and_types()
                    if 'visualization_msgs/srv/GetInteractiveMarkers' in types
                    and 'robot_interaction' in name]
                return len(marker_services) == 1

            wait_for(node, find_marker_service)
            marker_client = node.create_client(GetInteractiveMarkers, marker_services[0])
            resources.callback(node.destroy_client, marker_client)

            def goal_marker(dual=True):
                message = call(node, marker_client, GetInteractiveMarkers.Request())
                markers = [m for m in message.markers if 'goal' in m.name]
                assert len(markers) == 1, [m.name for m in message.markers]
                assert ('dual_arm' in markers[0].name) == dual, markers[0].name
                return markers[0]

            def delay(seconds):
                deadline = time.monotonic() + seconds
                wait_for(node, lambda: time.monotonic() >= deadline, seconds + 1.0)

            feedback_topics = [
                name for name, types in node.get_topic_names_and_types()
                if 'visualization_msgs/msg/InteractiveMarkerFeedback' in types
                and 'robot_interaction' in name]
            assert len(feedback_topics) == 1, feedback_topics
            feedback_pub = node.create_publisher(
                InteractiveMarkerFeedback, feedback_topics[0], 10)
            group_pub = node.create_publisher(String, '/rviz/moveit/select_planning_group', 10)
            plan_pub = node.create_publisher(Empty, '/rviz/moveit/plan', 10)
            execute_pub = node.create_publisher(Empty, '/rviz/moveit/execute', 10)
            reset_pub = node.create_publisher(Empty, '/rviz/moveit/update_goal_state', 10)
            publishers = [feedback_pub, group_pub, plan_pub, execute_pub, reset_pub]
            for publisher in publishers:
                resources.callback(node.destroy_publisher, publisher)
                wait_for(node, lambda: publisher.get_subscription_count() > 0)
            plans = []
            subscription = node.create_subscription(
                DisplayTrajectory, '/display_planned_path', plans.append, 10)
            resources.callback(node.destroy_subscription, subscription)
            for group in ('arm_both', 'arm_both_with_waist'):
                group_pub.publish(String(data=group))
                delay(1.0)
                reset_pub.publish(Empty())
                delay(1.0)
                marker = goal_marker()
                print('shared_marker_count', group, 1, marker.name, flush=True)
                reference = GetPositionFK.Request()
                reference.header.frame_id = 'base_link'
                reference.fk_link_names = ['left_amazing_hand', 'right_amazing_hand']
                fill_state(reference.robot_state, positions)
                original = call(node, fk, reference)
                assert original.error_code.val == 1
                original_x = marker.pose.position.x
                event = InteractiveMarkerFeedback(
                    header=marker.header, client_id='g1_mock_verifier',
                    marker_name=marker.name, control_name='move_0',
                    event_type=InteractiveMarkerFeedback.MOUSE_DOWN,
                    pose=copy.deepcopy(marker.pose))
                feedback_pub.publish(event)
                delay(0.3)
                event.event_type = InteractiveMarkerFeedback.POSE_UPDATE
                event.pose.position.x += 0.005
                feedback_pub.publish(event)
                delay(2.0)
                moved = goal_marker()
                displacement = moved.pose.position.x - original_x
                print('shared_marker_displacement_x', group, displacement, flush=True)
                assert 0.003 <= displacement <= 0.007, displacement
                event.event_type = InteractiveMarkerFeedback.MOUSE_UP
                feedback_pub.publish(event)
                delay(1.0)
                plans.clear()
                plan_pub.publish(Empty())
                wait_for(node, lambda: bool(plans), 30.0)
                trajectory = plans[-1].trajectory[-1].joint_trajectory
                assert set(trajectory.joint_names) == set(states()[group])
                assert len(trajectory.joint_names) == len(set(trajectory.joint_names))
                assert len(trajectory.points) > 1
                for point in trajectory.points:
                    check = GetStateValidity.Request(group_name=group)
                    values = dict(positions)
                    values.update(zip(trajectory.joint_names, point.positions))
                    fill_state(check.robot_state, values)
                    assert call(node, validity, check).valid
                execute_pub.publish(Empty())
                wait_for(node, lambda: all(
                    abs(positions[name] - target) < 0.001
                    for name, target in zip(
                        trajectory.joint_names, trajectory.points[-1].positions)), 30.0)
                fill_state(reference.robot_state, positions)
                reached = call(node, fk, reference)
                assert reached.error_code.val == 1
                for tip, before, after in zip(
                        reference.fk_link_names, original.pose_stamped, reached.pose_stamped):
                    distance = math.sqrt(sum(
                        (getattr(after.pose.position, axis) -
                         getattr(before.pose.position, axis) -
                         (0.005 if axis == 'x' else 0.0)) ** 2 for axis in 'xyz'))
                    dot = abs(sum(getattr(before.pose.orientation, axis) *
                                  getattr(after.pose.orientation, axis) for axis in 'xyzw'))
                    angle = 2 * math.acos(min(1.0, dot))
                    print('shared_marker_goal_error', group, tip, distance, angle, flush=True)
                    assert distance <= 0.0011
                    assert angle <= math.sqrt(3) * 0.1 + 0.001
                print('rviz_plan_execute', group, 'SUCCESS', flush=True)
                group_pub.publish(String(data='arm_left'))
                delay(1.0)
                single = goal_marker(dual=False)
                print('single_arm_marker_restored', single.name, flush=True)

        pose_failures = []
        # 到達可能な関節姿勢から手先目標を作り、単腕・双腕の姿勢目標を検証する。
        for group in (name for name in GROUPS if name != 'head'):
            target_state = dict(positions)
            for name in states()[group]:
                if 'elbow' in name:
                    target_state[name] += 0.04
            sides = ['left', 'right'] if 'both' in group else [
                'left' if 'left' in group else 'right']
            request = GetPositionFK.Request()
            request.header.frame_id = 'base_link'
            request.fk_link_names = [side + '_amazing_hand' for side in sides]
            fill_state(request.robot_state, target_state)
            response = call(node, fk, request)
            assert response.error_code.val == 1
            constraint = Constraints()
            for tip, pose in zip(request.fk_link_names, response.pose_stamped):
                pc = PositionConstraint()
                pc.header.frame_id = 'base_link'
                pc.link_name = tip
                pc.weight = 1.0
                pc.constraint_region.primitives = [SolidPrimitive(
                    type=SolidPrimitive.SPHERE, dimensions=[0.001])]
                region_pose = Pose()
                region_pose.position = pose.pose.position
                region_pose.orientation.w = 1.0
                pc.constraint_region.primitive_poses = [region_pose]
                oc = OrientationConstraint(
                    link_name=tip, orientation=pose.pose.orientation, weight=1.0,
                    absolute_x_axis_tolerance=0.1, absolute_y_axis_tolerance=0.1,
                    absolute_z_axis_tolerance=0.1)
                oc.header.frame_id = 'base_link'
                constraint.position_constraints.append(pc)
                constraint.orientation_constraints.append(oc)
                if 'both' not in group:
                    assert ik.wait_for_service(timeout_sec=15)
                    solved = api._solve_ik(pose, group, tip)
                    assert solved is not None, group
                    assert set(solved) == set(states()[group])
                    print('api_pick_ik', group, 'SUCCESS', flush=True)
            waist_before = positions['waist_yaw_joint']
            try:
                motion(group, constraint)
            except AssertionError as error:
                # 双腕の既知不具合も失敗として残し、独立した検証を最後まで実施する。
                if 'both' not in group:
                    raise
                pose_failures.append((group, str(error)))
                print('pose_plan_failed', group, str(error), flush=True)
                continue
            if group == 'arm_both':
                assert abs(positions['waist_yaw_joint'] - waist_before) < 0.001
            actual_request = GetPositionFK.Request()
            actual_request.header = request.header
            actual_request.fk_link_names = request.fk_link_names
            fill_state(actual_request.robot_state, positions)
            actual = call(node, fk, actual_request)
            assert actual.error_code.val == 1
            for tip, target, reached in zip(
                    request.fk_link_names, response.pose_stamped, actual.pose_stamped):
                distance = math.sqrt(sum(
                    (getattr(target.pose.position, axis) -
                     getattr(reached.pose.position, axis)) ** 2 for axis in 'xyz'))
                dot = abs(sum(getattr(target.pose.orientation, axis) *
                              getattr(reached.pose.orientation, axis) for axis in 'xyzw'))
                angle = 2 * math.acos(min(1.0, dot))
                print('goal_error', group, tip, 'position_m', distance,
                      'orientation_rad', angle, flush=True)
                assert distance <= 0.0011, (group, tip, distance)
                assert angle <= math.sqrt(3) * 0.1 + 0.001, (group, tip, angle)

        def current_pose_constraint(waist_offset=0.0):
            target = dict(positions)
            target['waist_yaw_joint'] += waist_offset
            req = GetPositionFK.Request()
            req.header.frame_id = 'base_link'
            req.fk_link_names = ['left_amazing_hand', 'right_amazing_hand']
            fill_state(req.robot_state, target)
            result = call(node, fk, req)
            assert result.error_code.val == 1
            constraint = Constraints()
            for tip, pose in zip(req.fk_link_names, result.pose_stamped):
                pc = PositionConstraint(link_name=tip, weight=1.0)
                pc.header.frame_id = 'base_link'
                pc.constraint_region.primitives = [
                    SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.001])]
                pc.constraint_region.primitive_poses = [
                    Pose(position=pose.pose.position)]
                pc.constraint_region.primitive_poses[0].orientation.w = 1.0
                oc = OrientationConstraint(
                    link_name=tip, orientation=pose.pose.orientation, weight=1.0,
                    absolute_x_axis_tolerance=0.1, absolute_y_axis_tolerance=0.1,
                    absolute_z_axis_tolerance=0.1)
                oc.header.frame_id = 'base_link'
                constraint.position_constraints.append(pc)
                constraint.orientation_constraints.append(oc)
            return constraint

        # 共通腰を明示した目標でも左右の手先制約が同時に成立することを確認する。
        waist_target = positions['waist_yaw_joint'] + 0.08
        constraint = current_pose_constraint(0.08)
        constraint.joint_constraints = [JointConstraint(
            joint_name='waist_yaw_joint', position=waist_target,
            tolerance_above=0.001, tolerance_below=0.001, weight=1.0)]
        motion('arm_both_with_waist', constraint)
        assert abs(positions['waist_yaw_joint'] - waist_target) < 0.0011
        print('shared_waist SUCCESS', flush=True)

        # 片手だけの制約では標準サンプラへ安全に委譲できることを確認する。
        constraint = current_pose_constraint()
        constraint.position_constraints = constraint.position_constraints[:1]
        constraint.orientation_constraints = constraint.orientation_constraints[:1]
        # 姿勢目標を与えない右腕は現在角度に固定し、独立した左手の制約を検証する。
        constraint.joint_constraints = [
            JointConstraint(joint_name=name, position=positions[name],
                            tolerance_above=0.001, tolerance_below=0.001, weight=1.0)
            for name in states()['arm_right']]
        motion('arm_both', constraint)
        print('single_tip_dual_group SUCCESS', flush=True)

        # 双腕の経路制約を含む要求と、その軌道上の妥当性を検証する。
        import copy
        constraint = current_pose_constraint()
        path = copy.deepcopy(constraint)
        for pc in path.position_constraints:
            pc.constraint_region.primitives[0].dimensions = [0.05]
        for oc in path.orientation_constraints:
            oc.absolute_x_axis_tolerance = 0.3
            oc.absolute_y_axis_tolerance = 0.3
            oc.absolute_z_axis_tolerance = 0.3
        check = GetStateValidity.Request(group_name='arm_both', constraints=path)
        fill_state(check.robot_state, positions)
        path_start = call(node, validity, check)
        print('path_start_validity', path_start, flush=True)
        for group in ('arm_both', 'arm_both_with_waist'):
            try:
                motion(group, constraint, path_constraint=path)
                print('dual_path_constraints', group, 'SUCCESS', flush=True)
            except AssertionError as error:
                pose_failures.append(('dual_path_constraints', group, str(error)))
                print('path_plan_failed', group, str(error), flush=True)

        # 明らかな不達目標を成功や空軌道の実行として扱わない。
        unreachable = current_pose_constraint()
        unreachable.position_constraints[0].constraint_region.primitive_poses[0].position.x = 10.0
        motion('arm_both', unreachable, expect_success=False)
        runtime = log_path.read_text()
        assert '更新済み FK の双腕サンプラ: arm_both' in runtime
        assert '更新済み FK の双腕サンプラ: arm_both_with_waist' in runtime
        assert 'dirty robot state' not in runtime.lower()

        velocities = {j.get('name'): float(j.find('limit').get('velocity'))
                      for j in model().findall('joint')
                      if j.get('name') in ('xl330_joint', 'd455_joint')}
        if min(velocities.values()) <= 0:
            print('HEAD_EXECUTION_PENDING: モータ型番・電圧の確認待ち', flush=True)
        else:
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = ['xl330_joint', 'd455_joint']
            point = JointTrajectoryPoint(positions=[0.05, -0.05])
            point.time_from_start.sec = 2
            goal.trajectory.points = [point]
            result = execute(node, head, goal)
            assert result.error_code == 0
            wait_for(node, lambda: abs(positions['xl330_joint'] - 0.05) < 0.001
                     and abs(positions['d455_joint'] + 0.05) < 0.001)
            print('head_follow_joint_trajectory SUCCESS', flush=True)
        assert not pose_failures, pose_failures
