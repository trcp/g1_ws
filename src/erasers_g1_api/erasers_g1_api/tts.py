"""G1 標準 TTS と VOICEVOX を共通の say API で扱う。"""

import math

from rclpy.callback_groups import ReentrantCallbackGroup

from erasers_g1_interfaces.action import VuiTTS as VuiTTSAction

from erasers_g1_api.vui_audio import _ActionCalls, PlayAudio, wait_future


class VuiTTS:
    """G1 標準 TTS を ROS 2 の /vui_tts Action 経由で使用する。

    speaker_id は英語が 1、中国語が 0。日本語には VoicevoxTTS を使用する。
    wait=True は再生完了、wait=False は要求の受理を成功条件とする。
    Executor に登録済みの node は、別スレッドで spin しておく。
    """

    def __init__(self, node, debug=False, timeout_sec=120.0,
                 speaker_id=VuiTTSAction.Goal.ENGLISH):
        if (isinstance(speaker_id, bool) or not isinstance(speaker_id, int)
                or speaker_id not in (
                    VuiTTSAction.Goal.ENGLISH, VuiTTSAction.Goal.CHINESE)):
            raise ValueError("speaker_id は英語の 1 または中国語の 0 を指定してください")
        self._node = node
        self._debug = debug
        self.speaker_id = speaker_id
        self._tts = _ActionCalls(node, VuiTTSAction, "/vui_tts", timeout_sec)
        self._audio = PlayAudio(node, timeout_sec)

    def say(self, text, logger="info", wait=True):
        """テキストを発話し、指定時はロボットの再生完了まで待つ。"""
        if isinstance(logger, bool):
            wait, logger = logger, "info"
        if not isinstance(text, str) or not text.strip():
            raise ValueError("読み上げテキストを指定してください")
        if logger in ("info", "warn", "error", "debug"):
            getattr(self._node.get_logger(), logger)(text)
        goal = VuiTTSAction.Goal()
        goal.text = text
        goal.speaker_id = self.speaker_id
        return self._tts.send(goal, wait=wait and not self._debug)

    def audio(self, audio_path, logger="info", wait=True):
        """WAV ファイルを /vui_audio Action 経由で再生する。"""
        if isinstance(logger, bool):
            wait = logger
        return self._audio.play_file(audio_path, wait=wait and not self._debug)

    def cancel(self):
        """このインスタンスの発話・音声再生 Action をキャンセルする。

        標準 TTS は Action のキャンセル後も実機の発話停止を保証しない。
        """
        tts_canceled = self._tts.cancel()
        audio_canceled = self._audio.cancel()
        return tts_canceled or audio_canceled


class VoicevoxTTS:
    """VOICEVOX の合成結果を G1 の /vui_audio で再生する。

    wait=False でも合成と再生要求の受付までは待つ。
    wait=True の成功は、合成成功だけでなく実機の再生完了を表す。
    """

    def __init__(self, node, debug=False, timeout_sec=120.0, speaker_id=26,
                 synthesis_timeout_sec=30.0):
        from voicevox_ros2_msgs.srv import Speaking

        if isinstance(speaker_id, bool) or not isinstance(speaker_id, int):
            raise ValueError("speaker_id は整数で指定してください")
        if not 0 <= speaker_id <= 127:
            raise ValueError("speaker_id は 0～127 で指定してください")
        if not math.isfinite(synthesis_timeout_sec) or synthesis_timeout_sec <= 0:
            raise ValueError("synthesis_timeout_sec は正の有限値で指定してください")
        self._node = node
        self._debug = debug
        self.speaker_id = speaker_id
        self._synthesis_timeout_sec = synthesis_timeout_sec
        self._request_type = Speaking.Request
        self._audio = PlayAudio(node, timeout_sec)
        self._synthesis = node.create_client(
            Speaking, "/speak", callback_group=ReentrantCallbackGroup())

    def say(self, text, logger="info", wait=True):
        """テキストを合成して発話し、指定時は再生完了まで待つ。"""
        if isinstance(logger, bool):
            wait, logger = logger, "info"
        if not isinstance(text, str) or not text.strip():
            raise ValueError("読み上げテキストを指定してください")
        if logger in ("info", "warn", "error", "debug"):
            getattr(self._node.get_logger(), logger)(text)
        if not self._synthesis.wait_for_service(timeout_sec=2.0):
            self._node.get_logger().error("VOICEVOX の /speak が見つかりません")
            return False
        request = self._request_type()
        request.text = text
        request.speaker_id = self.speaker_id
        request.enable_interrogative_upspeak = True
        try:
            future = self._synthesis.call_async(request)
            if not wait_future(self._node, future, self._synthesis_timeout_sec):
                future.cancel()
                self._node.get_logger().error("VOICEVOX の音声合成がタイムアウトしました")
                return False
            response = future.result()
            if response is None or not response.success or not response.wav_data:
                self._node.get_logger().error("VOICEVOX の音声合成に失敗しました")
                return False
            return self._audio.play_wav_bytes(
                response.wav_data, wait=wait and not self._debug)
        except Exception as error:
            self._node.get_logger().error(f"VOICEVOX の発話に失敗しました: {error}")
            return False

    def audio(self, audio_path, logger="info", wait=True):
        """既存 API と同じ形式で WAV ファイルを実機再生する。"""
        if isinstance(logger, bool):
            wait = logger
        return self._audio.play_file(audio_path, wait=wait and not self._debug)

    def cancel(self):
        """再生中の音声をキャンセルする。合成サービス自体は停止しない。"""
        return self._audio.cancel()


# 既存の from erasers_g1_api.tts import TTS を維持する。
TTS = VuiTTS

__all__ = ["VuiTTS", "VoicevoxTTS", "TTS"]
