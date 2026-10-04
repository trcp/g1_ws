"""loopback DDS 上で実ノードと模擬 Unitree 応答を接続する回帰試験。"""

from contextlib import contextmanager
import importlib.util
import io
import json
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading
import time
import uuid
import wave

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray
from erasers_g1_interfaces.action import VuiAudio, VuiTTS
from erasers_g1_interfaces.srv import ArmAction, AudioClient, PosePolicy, RobotPose
from geometry_msgs.msg import Twist
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Int16MultiArray, String
from std_srvs.srv import SetBool
from unitree_api.msg import Request, Response
from unitree_hg.msg import LowCmd


def wait_for(predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    assert predicate(), '期限内に期待する状態へ遷移しませんでした'


def resolve(future, timeout=5.0):
    wait_for(future.done, timeout)
    return future.result()


class Runtime:
    def __init__(self):
        self.node = rclpy.create_node('integration_' + uuid.uuid4().hex[:8])
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.node)
        self.spin_errors = []
        self.thread = threading.Thread(target=self.spin)
        self.thread.start()
        self.requests = []
        self.fsm = 500
        self.voice_hook = None
        self.arm_hook = None
        self.refs = []
        self.clients = {}
        for channel in ('voice', 'sport', 'arm'):
            pub = self.node.create_publisher(Response, f'/api/{channel}/response', 10)
            self.refs.append(pub)
            self.refs.append(self.node.create_subscription(
                Request, f'/api/{channel}/request',
                lambda msg, channel=channel, pub=pub: self.respond(channel, pub, msg), 10))
        self.audio_state = self.node.create_publisher(String, '/audio_msg', 10)

    def spin(self):
        try:
            self.executor.spin()
        except Exception as error:
            self.spin_errors.append(error)

    def respond(self, channel, publisher, request):
        api = request.header.identity.api_id
        self.requests.append((channel, api, request, time.monotonic()))
        response = Response()
        response.header.identity = request.header.identity
        response.header.status.code = 0
        if channel == 'sport':
            if api == 7101:
                self.fsm = json.loads(request.parameter)['data']
            response.data = json.dumps({'data': self.fsm if api == 7001 else 0})
        else:
            response.data = '{}'
        publisher.publish(response)
        if channel == 'voice' and self.voice_hook:
            self.voice_hook(api)
        if channel == 'arm' and self.arm_hook:
            self.arm_hook(api, request)

    def service(self, srv_type, name, request, timeout=5.0):
        client = self.client(srv_type, name)
        assert client.wait_for_service(timeout_sec=5.0), name
        return resolve(client.call_async(request), timeout)

    def client(self, srv_type, name):
        # DDS エンティティの生成は制御開始前に済ませる。
        key = (srv_type, name)
        if key not in self.clients:
            self.clients[key] = self.node.create_client(srv_type, name)
        return self.clients[key]

    def play_state(self, state):
        self.audio_state.publish(String(data=json.dumps({'play_state': state})))

    def later(self, delay, callback):
        def run():
            timer.cancel()
            callback()
        timer = self.node.create_timer(delay, run)
        self.refs.append(timer)

    @contextmanager
    def process(self, executable, parameters=None, package='erasers_g1_common'):
        if executable == 'feedback_source.py':
            command = [sys.executable, str(Path(__file__).with_name(executable)), '--ros-args']
        else:
            command = [str(Path(get_package_prefix(package)) /
                           'lib' / package / executable), '--ros-args']
        for name, value in (parameters or {}).items():
            command += ['-p', f'{name}:={value}']
        if executable == 'robot_controller':
            command += ['-r', '/arm_sdk:=/integration/arm_sdk']
        with tempfile.TemporaryFile(mode='w+') as log:
            process = subprocess.Popen(command, stdout=log, stderr=log)
            print(f'起動 PID={process.pid} 終了コマンド=kill -INT {process.pid}', flush=True)
            try:
                yield process
            finally:
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=3)
                log.seek(0)
                output = log.read()
                print(output[-12000:])
                print(f'終了確認 PID={process.pid} code={process.returncode}', flush=True)
                assert process.returncode == 0, output

    def close(self):
        self.executor.shutdown(timeout_sec=5.0)
        self.thread.join(timeout=5)
        assert not self.thread.is_alive()
        self.node.destroy_node()
        assert not self.spin_errors, self.spin_errors


@pytest.fixture
def runtime(monkeypatch):
    config = Path(__file__).resolve().parents[2] / 'erasers_g1_common/test/cyclonedds.xml'
    monkeypatch.setenv('ROS_DOMAIN_ID', '173')
    monkeypatch.setenv('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    monkeypatch.setenv('CYCLONEDDS_URI', config.as_uri())
    rclpy.init()
    rt = Runtime()
    try:
        yield rt
    finally:
        rt.close()
        rclpy.shutdown()


@pytest.fixture
def vui(runtime):
    parameters = {
        'timeout_sec': 0.5, 'playback_start_timeout_sec': 0.8,
        'tts_playback_finish_timeout_sec': 2.0,
        'audio_playback_finish_grace_sec': 1.0,
        'playback_completion_quiet_sec': 0.15,
        'playback_feedback_period_sec': 0.02,
        'audio_chunk_interval_sec': 0.01,
        'audio_stop_timeout_sec': 0.5, 'audio_stop_state_timeout_sec': 0.5,
    }
    with runtime.process('vui_client', parameters):
        client = ActionClient(runtime.node, VuiTTS, '/vui_tts')
        assert client.wait_for_server(timeout_sec=5.0)
        wait_for(lambda: runtime.audio_state.get_subscription_count() > 0)
        runtime.play_state(0)
        time.sleep(0.1)
        yield runtime
        client.destroy()


def test_tts_requires_playing_and_stable_stop(vui):
    client = ActionClient(vui.node, VuiTTS, '/vui_tts')
    feedback = []
    goal = resolve(client.send_goal_async(
        VuiTTS.Goal(text='test'),
        feedback_callback=lambda msg: feedback.append(msg.feedback)))
    assert goal.accepted
    result = goal.get_result_async()
    wait_for(lambda: any(api == 1001 for _, api, _, _ in vui.requests))
    time.sleep(0.1)
    assert not result.done()
    vui.play_state(1)
    time.sleep(0.05)
    vui.play_state(0)
    time.sleep(0.05)
    assert not result.done()
    vui.play_state(1)
    time.sleep(0.2)
    assert not result.done()
    vui.play_state(0)
    completed = resolve(result)
    assert completed.status == GoalStatus.STATUS_SUCCEEDED
    assert completed.result.success
    wait_for(lambda: any(item.done for item in feedback))
    client.destroy()


def test_audio_stopped_only_does_not_report_completion(vui):
    client = ActionClient(vui.node, VuiAudio, '/vui_audio')
    assert client.wait_for_server(timeout_sec=3.0)
    goal = VuiAudio.Goal(source_type=VuiAudio.Goal.SOURCE_PCM16_MONO_16K,
                         audio_data=[0] * 320, stop_after_play=False)
    handle = resolve(client.send_goal_async(goal))
    assert handle.accepted
    result = resolve(handle.get_result_async())
    assert result.status == GoalStatus.STATUS_ABORTED
    assert not result.result.success
    assert result.result.bytes_sent == 320
    client.destroy()


def test_tts_cancel_and_simultaneous_goal_rejection(vui):
    client = ActionClient(vui.node, VuiTTS, '/vui_tts')
    first = resolve(client.send_goal_async(VuiTTS.Goal(text='first')))
    assert first.accepted
    wait_for(lambda: any(api == 1001 for _, api, _, _ in vui.requests))
    second = resolve(client.send_goal_async(VuiTTS.Goal(text='second')))
    assert not second.accepted
    assert resolve(first.cancel_goal_async()).goals_canceling
    assert resolve(first.get_result_async()).status == GoalStatus.STATUS_CANCELED
    client.destroy()


def test_shutdown_with_active_tts_joins_worker(vui):
    client = ActionClient(vui.node, VuiTTS, '/vui_tts')
    handle = resolve(client.send_goal_async(VuiTTS.Goal(text='shutdown')))
    assert handle.accepted
    wait_for(lambda: any(api == 1001 for _, api, _, _ in vui.requests))
    # fixture が実行中ノードを停止し、正常終了と子プロセス回収を検証する。
    client.destroy()


def test_python_audio_api_and_legacy_service(vui):
    from erasers_g1_api.tts import TTS
    from erasers_g1_api.vui_audio import PlayAudio

    def play(api):
        if api in (1001, 1003):
            vui.play_state(1)
            vui.later(0.15, lambda: vui.play_state(0))
    vui.voice_hook = play
    tts = TTS(vui.node, timeout_sec=3)
    assert tts.say('completed')
    assert vui.node.executor is vui.executor
    # 通知音の同一性検証は行わず、人工 WAV で API の転送と完了を確認する。
    data = io.BytesIO()
    with wave.open(data, 'wb') as output:
        output.setnchannels(1)
        output.setsampwidth(2)
        output.setframerate(16000)
        output.writeframes(b'\0\0' * 1600)
    audio = PlayAudio(vui.node, timeout_sec=3)
    assert audio.play_wav_bytes(data.getvalue())
    with vui.process('audio_client'):
        result = vui.service(AudioClient, '/play_audio', AudioClient.Request(text='legacy'))
        assert result.success
        result = vui.service(AudioClient, '/play_audio', AudioClient.Request(type=99))
        assert not result.success


def test_legacy_mic_relay(runtime):
    def enable(request, response):
        response.success = request.data
        response.message = 'mock'
        return response
    service = runtime.node.create_service(SetBool, '/enable_mic', enable)
    samples = []
    runtime.refs.append(runtime.node.create_subscription(
        Int16MultiArray, '/audio/raw', lambda msg: samples.append(list(msg.data)), 10))
    pub = runtime.node.create_publisher(Int16MultiArray, '/mic_data', 10)
    with runtime.process('mic_server'):
        assert runtime.service(SetBool, '/mic_rec', SetBool.Request(data=True)).success
        assert not runtime.service(SetBool, '/mic_rec', SetBool.Request(data=False)).success
        wait_for(lambda: pub.get_subscription_count() > 0)
        pub.publish(Int16MultiArray(data=[-32768, 0, 32767]))
        wait_for(lambda: bool(samples))
        assert samples[-1] == [-32768, 0, 32767]
    runtime.node.destroy_service(service)


@pytest.fixture
def robot(runtime):
    runtime.lowstate_enabled = True
    runtime.target = None
    runtime.sdk = []
    runtime.diagnostics = {}
    runtime.tick_times = []
    runtime.upper = runtime.node.create_publisher(JointState, '/upper_joints_control', 10)
    runtime.velocity = runtime.node.create_publisher(Twist, '/cmd_vel', 10)
    runtime.stop = runtime.node.create_publisher(
        Bool, '/emergency_stop/active',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    state_enabled = runtime.node.create_publisher(
        Bool, '/integration/feedback_enabled',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    runtime.refs.append(runtime.node.create_subscription(
        LowCmd, '/integration/arm_sdk', lambda msg: runtime.sdk.append(msg), 100))

    def diagnostics(message):
        for status in message.status:
            runtime.diagnostics.update({entry.key: entry.value for entry in status.values})
    runtime.refs.append(runtime.node.create_subscription(
        DiagnosticArray, '/robot_controller/diagnostics', diagnostics, 10))

    def tick():
        runtime.tick_times.append(time.monotonic())
        state_enabled.publish(Bool(data=runtime.lowstate_enabled))
        target = runtime.target
        if target is not None:
            message = JointState(name=['left_shoulder_pitch_joint'], position=[target])
            message.header.stamp = runtime.node.get_clock().now().to_msg()
            runtime.upper.publish(message)
    feedback_stop = threading.Event()
    feedback_errors = []

    def feedback_loop():
        try:
            while not feedback_stop.is_set():
                tick()
                feedback_stop.wait(0.02)
        except Exception as error:
            feedback_errors.append(error)
    feedback_thread = threading.Thread(target=feedback_loop)
    feedback_thread.start()
    urdf = Path(get_package_share_directory('erasers_g1_description')) / 'urdf/erasers_g1.urdf'
    try:
        with runtime.process('feedback_source.py'), runtime.process(
            'robot_controller', {'urdf': str(urdf)}
        ):
            wait_for(lambda: runtime.diagnostics.get('upper_body.owner') == 'NONE')
            for srv_type, name in (
                (ArmAction, '/arm_action'), (RobotPose, '/robot_pose'),
                (SetBool, '/robot_controller/upper_body/enable'),
            ):
                assert runtime.client(srv_type, name).wait_for_service(timeout_sec=3.0)
            yield runtime
    finally:
        feedback_stop.set()
        feedback_thread.join(timeout=3)
        assert not feedback_thread.is_alive()
        assert not feedback_errors, feedback_errors
        gaps = [b - a for a, b in zip(runtime.tick_times, runtime.tick_times[1:])]
        print('模擬指令診断:', max(gaps, default=0), runtime.diagnostics)


def enable_upper(robot, enable=True):
    response = robot.service(
        SetBool, '/robot_controller/upper_body/enable', SetBool.Request(data=enable))
    assert response.success, response.message


def test_upper_body_gating_mask_ownership_and_watchdog(robot):
    robot.target = 0.05
    time.sleep(0.2)
    assert not robot.sdk
    enable_upper(robot)
    wait_for(lambda: any(msg.motor_cmd[29].q == 1.0 for msg in robot.sdk))
    result = robot.service(ArmAction, '/arm_action', ArmAction.Request(mode=99))
    assert not result.success
    assert not any(channel == 'arm' and api == 7106 for channel, api, _, _ in robot.requests)
    robot.target = None
    wait_for(lambda: robot.sdk and robot.sdk[-1].motor_cmd[29].q == 0.0, timeout=3)
    count = len(robot.sdk)
    time.sleep(0.15)
    assert len(robot.sdk) == count
    for command in robot.sdk:
        assert len(command.motor_cmd) == 35
        assert command.mode_pr == 0
        for index in (*range(12), 13, 14, 20, 21, 27, 28):
            assert command.motor_cmd[index].kp == 0.0


def test_upper_body_stale_feedback_faults_and_closes_output(robot):
    enable_upper(robot)
    robot.target = 0.01
    wait_for(lambda: len(robot.sdk) > 5)
    robot.lowstate_enabled = False
    wait_for(lambda: robot.diagnostics.get('upper_body.phase') == 'FAULT')
    count = len(robot.sdk)
    time.sleep(0.15)
    assert len(robot.sdk) == count
    result = robot.service(
        SetBool, '/robot_controller/upper_body/enable', SetBool.Request(data=True))
    assert not result.success


def test_hardware_plugin_requires_permission_and_relays_disable(robot):
    import xacro

    # 実際の HW プラグインを唯一の JointState 送信元とする。
    robot.node.destroy_publisher(robot.upper)
    model = (Path(get_package_share_directory('erasers_g1_description')) /
             'urdf/erasers_g1.urdf.xacro')
    description = xacro.process_file(str(model)).toxml()
    with robot.process('ros2_control_node', {
        'robot_description': description, 'update_rate': 100,
    }, package='controller_manager'):
        client = robot.node.create_client(SetBool, '/enable_upper_body_control')
        assert client.wait_for_service(timeout_sec=5.0)
        time.sleep(0.3)
        assert not robot.sdk
        result = resolve(client.call_async(SetBool.Request(data=True)))
        assert result.success, result.message
        wait_for(lambda: any(msg.motor_cmd[29].q == 1.0 for msg in robot.sdk))
        result = resolve(client.call_async(SetBool.Request(data=False)))
        assert result.success, result.message
        wait_for(lambda: robot.sdk[-1].motor_cmd[29].q == 0.0)
        count = len(robot.sdk)
        time.sleep(0.15)
        assert len(robot.sdk) == count
        robot.node.destroy_client(client)


def test_walk_watchdog_pose_compatibility_and_emergency_latch(robot):
    command = Twist()
    command.linear.x = 10.0
    command.angular.z = 10.0
    end = time.monotonic() + 0.5
    while time.monotonic() < end:
        robot.velocity.publish(command)
        time.sleep(0.03)

    def velocities():
        return [(json.loads(request.parameter)['velocity'], stamp)
                for channel, api, request, stamp in robot.requests
                if channel == 'sport' and api == 7105]

    wait_for(lambda: any(values[0] > 0 for values, _ in velocities()))
    wait_for(lambda: velocities()[-1][0] == [0, 0, 0])
    assert all(abs(v[0]) <= 1.0 and abs(v[2]) <= 1.57 for v, _ in velocities())
    with robot.process('loco_service_client'):
        assert robot.service(PosePolicy, '/pose_policy', PosePolicy.Request(pose='damp')).success
        assert robot.fsm == 1
        assert robot.service(
            RobotPose, '/robot_pose', RobotPose.Request(mode=RobotPose.Request.START)).success
        assert robot.fsm == 500
    robot.stop.publish(Bool(data=True))
    time.sleep(0.15)
    robot.stop.publish(Bool(data=False))
    baseline = len(velocities())
    for _ in range(10):
        robot.velocity.publish(command)
        time.sleep(0.03)
    assert all(v == [0, 0, 0] for v, _ in velocities()[baseline:])
    result = robot.service(
        SetBool, '/robot_controller/upper_body/enable', SetBool.Request(data=True))
    assert not result.success


@pytest.mark.parametrize('package,filename', [
    ('erasers_g1_bringup', 'bringup.launch.py'),
    ('erasers_g1_hw_controller', 'controller.launch.py'),
    ('erasers_g1_moveit', 'moveit_control_bringup.launch.py'),
    ('erasers_g1_description', 'display.launch.py'),
])
def test_launch_resources_resolve_without_starting_nodes(package, filename):
    path = Path(get_package_share_directory(package)) / 'launch' / filename
    spec = importlib.util.spec_from_file_location('launch_under_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert module.generate_launch_description().entities


SOURCE_ROOT = Path(__file__).resolve().parents[2]
CALLERS = sorted((SOURCE_ROOT / 'erasers_g1_api/samples').glob('*.py')) + sorted(
    (SOURCE_ROOT / 'erasers_g1_tasks/erasers_g1_tasks').glob('*.py'))


@pytest.mark.parametrize('path', CALLERS, ids=lambda path: path.name)
def test_samples_and_tasks_import(path):
    spec = importlib.util.spec_from_file_location('caller_' + path.stem, path)
    module = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(module)
    except ModuleNotFoundError as error:
        if error.name.split('.')[0] in {'lor_interfaces', 'sam3_ros_interfaces'}:
            pytest.skip('廃止インターフェース参照はユーザー指定の許容対象')
        raise


def test_voicevox_tts_default_speaker_and_completion(vui):
    from erasers_g1_api.tts import VoicevoxTTS
    from voicevox_ros2_msgs.srv import Speaking

    wav = io.BytesIO()
    with wave.open(wav, 'wb') as output:
        output.setnchannels(1)
        output.setsampwidth(2)
        output.setframerate(16000)
        output.writeframes(b'\0\0' * 1600)
    requests = []

    def synthesize(request, response):
        requests.append((request.text, request.speaker_id))
        response.success = True
        response.wav_data = list(wav.getvalue())
        return response

    service = vui.node.create_service(Speaking, '/speak', synthesize)

    def play(api):
        if api == 1003:
            vui.play_state(1)
            vui.later(0.15, lambda: vui.play_state(0))

    vui.voice_hook = play
    try:
        tts = VoicevoxTTS(vui.node, timeout_sec=3)
        assert tts.say('既定話者の検証')
        assert requests == [('既定話者の検証', 26)]
        assert any(api == 1003 for _, api, _, _ in vui.requests)
        assert vui.node.executor is vui.executor
    finally:
        vui.node.destroy_service(service)


@pytest.mark.parametrize('success', [False, True])
def test_voicevox_tts_invalid_synthesis_never_plays(vui, success):
    from erasers_g1_api.tts import VoicevoxTTS
    from voicevox_ros2_msgs.srv import Speaking

    def synthesize(request, response):
        # 失敗応答と、不正な空 WAV の成功応答をともに拒否する。
        response.success = success
        response.wav_data = []
        return response

    service = vui.node.create_service(Speaking, '/speak', synthesize)
    try:
        before = len(vui.requests)
        tts = VoicevoxTTS(vui.node, timeout_sec=3)
        assert not tts.say('再生してはいけない合成結果')
        assert len(vui.requests) == before
    finally:
        vui.node.destroy_service(service)


def test_tts_standalone_node_after_spin_once(vui):
    from erasers_g1_api.tts import VuiTTS as NativeTTS, VoicevoxTTS
    from voicevox_ros2_msgs.srv import Speaking

    wav = io.BytesIO()
    with wave.open(wav, 'wb') as output:
        output.setnchannels(1)
        output.setsampwidth(2)
        output.setframerate(16000)
        output.writeframes(b'\0\0' * 1600)

    def synthesize(request, response):
        response.success = True
        response.wav_data = list(wav.getvalue())
        return response

    def play(api):
        if api in (1001, 1003):
            vui.play_state(1)
            vui.later(0.15, lambda: vui.play_state(0))

    service = vui.node.create_service(Speaking, '/speak', synthesize)
    vui.voice_hook = play
    standalone = rclpy.create_node('standalone_tts_verifier')
    try:
        # Humble は remove_node 後も node.executor の参照が残る。
        rclpy.spin_once(standalone, timeout_sec=0)
        assert standalone not in standalone.executor.get_nodes()
        native = NativeTTS(standalone, timeout_sec=3)
        japanese = VoicevoxTTS(standalone, timeout_sec=3)
        assert native.say('単独ノードの待機確認')
        assert japanese.say('連続する発話の待機確認')
    finally:
        standalone.destroy_node()
        vui.node.destroy_service(service)

@pytest.mark.parametrize('action_type,api_id', [(VuiTTS, 1001), (VuiAudio, 1003)])
def test_voice_waits_for_both_ros_topic_endpoints(runtime, action_type, api_id):
    """要求先だけでなく応答元の接続も待ち、初回要求を一度だけ送る。"""
    request_topic = '/late_voice/request'
    response_topic = '/late_voice/response'
    requests = []
    publishers = []

    def respond(request):
        requests.append(request)
        if publishers:
            runtime.respond('voice', publishers[0], request)

    def play(api):
        if api == api_id:
            runtime.play_state(1)
            runtime.later(0.15, lambda: runtime.play_state(0))

    runtime.voice_hook = play
    with runtime.process('vui_client', {
        'request_topic': request_topic, 'response_topic': response_topic,
        'timeout_sec': 2.0, 'playback_completion_quiet_sec': 0.1,
        'playback_feedback_period_sec': 0.02,
    }):
        topic = '/vui_tts' if action_type is VuiTTS else '/vui_audio'
        client = ActionClient(runtime.node, action_type, topic)
        subscription = None
        try:
            assert client.wait_for_server(timeout_sec=5.0)
            wait_for(lambda: runtime.audio_state.get_subscription_count() > 0)
            if action_type is VuiTTS:
                goal = VuiTTS.Goal(text='接続完了後に一度だけ発話')
            else:
                goal = VuiAudio.Goal(
                    source_type=VuiAudio.Goal.SOURCE_PCM16_MONO_16K,
                    audio_data=[0] * 320, stop_after_play=False)
            handle = resolve(client.send_goal_async(goal))
            assert handle.accepted
            result = handle.get_result_async()
            subscription = runtime.node.create_subscription(
                Request, request_topic, respond, 10)
            wait_for(lambda: runtime.node.get_publishers_info_by_topic(request_topic))
            time.sleep(0.15)
            assert not requests
            assert not result.done()
            publishers.append(runtime.node.create_publisher(
                Response, response_topic, 10))
            completed = resolve(result)
            assert completed.status == GoalStatus.STATUS_SUCCEEDED
            assert completed.result.success
            assert [r.header.identity.api_id for r in requests] == [api_id]
        finally:
            client.destroy()
            if subscription is not None:
                runtime.node.destroy_subscription(subscription)
            for publisher in publishers:
                runtime.node.destroy_publisher(publisher)


def test_native_tts_language_and_legacy_import(vui):
    from erasers_g1_api.tts import TTS, VuiTTS as NativeTTS
    from erasers_g1_api.vui_audio import TTS as LegacyTTS

    assert TTS is NativeTTS

    def play(api):
        if api == 1001:
            vui.play_state(1)
            vui.later(0.15, lambda: vui.play_state(0))

    vui.voice_hook = play
    assert NativeTTS(vui.node, timeout_sec=3, speaker_id=0).say('你好')
    assert LegacyTTS(vui.node, timeout_sec=3).say('Hello', True)
    speakers = [
        json.loads(request.parameter)['speaker_id']
        for _, api, request, _ in vui.requests if api == 1001]
    assert speakers == [0, 1]
