"""G1 の VUI Action と通知音を扱う API。"""

from pathlib import Path
from threading import Event
import math

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from erasers_g1_interfaces.action import VuiAudio
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup


def wait_future(node, future, timeout_sec):
    """既存 Executor を外さず、期限付きで応答を待つ。"""
    if future.done():
        return True
    executor = node.executor
    # Humble の remove_node は node.executor の弱参照を消さない。
    # 実際に Executor に登録されている場合だけ別スレッドの処理を待つ。
    if executor is not None and node in executor.get_nodes():
        completed = Event()
        future.add_done_callback(lambda _: completed.set())
        completed.wait(timeout_sec)
    else:
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    return future.done()


class _ActionCalls:
    def __init__(self, node, action_type, topic, timeout_sec=120.0):
        if not math.isfinite(timeout_sec) or timeout_sec <= 0:
            raise ValueError("timeout_sec は正の有限値で指定してください")
        self.node = node
        self.timeout_sec = timeout_sec
        self.client = ActionClient(
            node, action_type, topic, callback_group=ReentrantCallbackGroup()
        )
        self._goal = None

    def send(self, goal, wait=True):
        if not self.client.wait_for_server(timeout_sec=2.0):
            return False
        future = self.client.send_goal_async(goal)
        if not wait_future(self.node, future, 3.0):
            # 遅れて受理された要求も放置しない。
            future.add_done_callback(self._cancel_late_goal)
            return False
        try:
            handle = future.result()
            if handle is None or not handle.accepted:
                return False
            self._goal = handle
            result_future = handle.get_result_async()
            result_future.add_done_callback(
                lambda _: self._clear_goal(handle)
            )
            if not wait:
                return True
            if not wait_future(self.node, result_future, self.timeout_sec):
                handle.cancel_goal_async()
                return False
            result = result_future.result()
            return (
                result.status == GoalStatus.STATUS_SUCCEEDED
                and bool(result.result.success)
            )
        except Exception as error:
            self.node.get_logger().error(f"音声 Action が失敗しました: {error}")
            return False

    def _clear_goal(self, handle):
        if self._goal is handle:
            self._goal = None

    @staticmethod
    def _cancel_late_goal(future):
        try:
            handle = future.result()
            if handle is not None and handle.accepted:
                handle.cancel_goal_async()
        except Exception:
            pass

    def cancel(self):
        """実行中の音声要求をキャンセルする。"""
        handle = self._goal
        if handle is None:
            return False
        future = handle.cancel_goal_async()
        if not wait_future(self.node, future, 3.0):
            return False
        return bool(future.result().goals_canceling)


class PlayAudio:
    """通知音をクライアント側で読み込み、WAV データとして送信する。"""

    def __init__(self, node, timeout_sec=120.0):
        self._calls = _ActionCalls(node, VuiAudio, "/vui_audio", timeout_sec)

    def play_wav_bytes(self, wav_bytes, wait=True, stop_after_play=True):
        """WAV を送信し、wait=True では再生完了を待つ。"""
        data = bytes(wav_bytes)
        if not data or len(data) > 10485760:
            raise ValueError("WAV のサイズは 1～10485760 byte にしてください")
        goal = VuiAudio.Goal()
        goal.source_type = VuiAudio.Goal.SOURCE_WAV_BYTES
        goal.audio_data = list(data)
        goal.stop_after_play = stop_after_play
        return self._calls.send(goal, wait)

    def play_file(self, file_path, wait=True, stop_after_play=True):
        """このクライアントから参照できるファイルを再生する。"""
        path = Path(file_path).expanduser()
        if path.stat().st_size > 10485760:
            raise ValueError("WAV が容量上限を超えています")
        return self.play_wav_bytes(path.read_bytes(), wait, stop_after_play)

    def _beep(self, filename, wait, stop_after_play):
        path = Path(get_package_share_directory("erasers_g1_common")) / "audio" / filename
        return self.play_file(path, wait, stop_after_play)

    def agree(self, wait=True, stop_after_play=True):
        """肯定の通知音を再生する。"""
        return self._beep("agree.wav", wait, stop_after_play)

    def disagree(self, wait=True, stop_after_play=True):
        """否定の通知音を再生する。"""
        return self._beep("disagree.wav", wait, stop_after_play)

    def voice_recong_pin(self, wait=True, stop_after_play=True):
        """音声認識の開始音を再生する。"""
        return self._beep("voice_recong_pin.wav", wait, stop_after_play)

    def cancel(self):
        """実行中の再生をキャンセルする。"""
        return self._calls.cancel()


class TTS:
    """旧インポート経路を保持する VuiTTS の互換ラッパー。"""

    def __init__(self, node, debug=False, timeout_sec=120.0):
        # 共通の音声転送処理を tts から使うため、循環インポートを避ける。
        from erasers_g1_api.tts import VuiTTS

        self._implementation = VuiTTS(node, debug, timeout_sec)

    def say(self, text, logger="info", wait=True):
        """VuiTTS の発話処理へ委譲する。"""
        return self._implementation.say(text, logger, wait)

    def audio(self, audio_path, logger="info", wait=True):
        """VuiTTS の WAV 再生処理へ委譲する。"""
        return self._implementation.audio(audio_path, logger, wait)

    def cancel(self):
        """VuiTTS のキャンセル処理へ委譲する。"""
        return self._implementation.cancel()
