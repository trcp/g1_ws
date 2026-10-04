#!/usr/bin/env python3

"""ROS 2 のマイク音声を Faster-Whisper で認識する Action サーバー。"""

import math
import threading
import time

import ctranslate2
import numpy as np
import rclpy
from faster_whisper import WhisperModel
from erasers_g1_interfaces.action import WhisperRecongnition
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
from std_srvs.srv import SetBool

from erasers_g1_api.audio_processing import condition_mic_audio
from erasers_g1_api.vui_audio import PlayAudio


class WhisperNode(Node, PlayAudio):
    """マイクの PCM を録音し、Action 経由で認識結果を返す。"""

    def __init__(self):
        super().__init__('whisper_node')
        PlayAudio.__init__(self, self)

        self.declare_parameter('device', 'cuda')
        self.declare_parameter('model_path', '/tmp/whisper')
        self.declare_parameter('mic_topic', '/mic_data')
        self.declare_parameter('mic_service', '/enable_mic')
        self.declare_parameter('mic_service_timeout_sec', 10.0)
        self.declare_parameter('silence_threshold_rms', 1000.0)
        self.declare_parameter('beam_size', 5)
        self.declare_parameter('feedback_period_sec', 0.25)
        self.declare_parameter('start_notification_delay_sec', 1.5)

        device = self.get_parameter('device').value
        model_path = self.get_parameter('model_path').value
        supported = ctranslate2.get_supported_compute_types(device)
        if device == 'cuda':
            compute_type = 'float16' if 'float16' in supported else 'float32'
        else:
            compute_type = 'int8' if 'int8' in supported else 'float32'

        self.get_logger().info('''
        LOAD MODEL
        --------------------------
        Model Path     : %s
        Device         : %s
        Computing Type : %s
        ''' % (model_path, device, compute_type))
        self.whisper = WhisperModel(
            model_path,
            device=device,
            compute_type=compute_type,
        )

        # セグメントを消費するまで推論されないため、起動時に最後まで評価する。
        warmup_segments, _ = self.whisper.transcribe(
            np.zeros(16000, dtype=np.float32),
            language='en',
            beam_size=1,
            vad_filter=False,
            condition_on_previous_text=False,
        )
        list(warmup_segments)
        self.get_logger().info('''
        WHISPER START !
        --------------------------
        Model Path     : %s
        Device         : %s
        Computing Type : %s
        ''' % (model_path, device, compute_type))

        self._callback_group = ReentrantCallbackGroup()
        self._mic_client = self.create_client(
            SetBool,
            self.get_parameter('mic_service').value,
            callback_group=self._callback_group,
        )
        mic_service_timeout = float(
            self.get_parameter('mic_service_timeout_sec').value
        )
        if not math.isfinite(mic_service_timeout) or mic_service_timeout <= 0.0:
            raise ValueError('mic_service_timeout_sec must be finite and positive')
        while not self._mic_client.wait_for_service(
            timeout_sec=mic_service_timeout
        ):
            self.get_logger().error('Microphone service is unavailable.')
            raise RuntimeError('Microphone service is unavailable.')
        self._mic_subscription = self.create_subscription(
            Int16MultiArray,
            self.get_parameter('mic_topic').value,
            self._on_mic_data,
            10,
            callback_group=self._callback_group,
        )
        self._action_server = ActionServer(
            self,
            WhisperRecongnition,
            '/whisper_recongnition_action',
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )

        self._state_lock = threading.Lock()
        self._busy = False
        self._capturing = False
        self._chunks = []
        self._speech_detected = False
        self._silence_started_at = None

    def _goal_callback(self, goal_request):
        language = goal_request.language
        times = (
            goal_request.minimum_recording_time,
            goal_request.max_silence_time,
            goal_request.maximum_recording_time,
        )
        if language not in ('', 'en', 'ja'):
            self.get_logger().warning(f'Rejected unsupported language: {language}')
            return GoalResponse.REJECT
        if not all(math.isfinite(value) for value in times):
            self.get_logger().warning('Rejected non-finite recording duration')
            return GoalResponse.REJECT
        if (
            goal_request.minimum_recording_time < 0.0
            or goal_request.max_silence_time <= 0.0
            or goal_request.maximum_recording_time <= 0.0
            or goal_request.minimum_recording_time > goal_request.maximum_recording_time
        ):
            self.get_logger().warning('Rejected invalid recording duration')
            return GoalResponse.REJECT

        with self._state_lock:
            if self._busy:
                self.get_logger().warning('Rejected recognition request while busy')
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    @staticmethod
    def _cancel_callback(unused_goal_handle):
        return CancelResponse.ACCEPT

    def _on_mic_data(self, message):
        samples = np.asarray(message.data, dtype=np.int16).copy()
        if samples.size == 0:
            return
        now = time.monotonic()
        threshold = float(self.get_parameter('silence_threshold_rms').value)
        rms = float(np.sqrt(np.mean(samples.astype(np.float64) ** 2)))

        with self._state_lock:
            if not self._capturing:
                return
            self._chunks.append(samples)
            if rms >= threshold:
                self._speech_detected = True
                self._silence_started_at = None
            elif self._speech_detected and self._silence_started_at is None:
                self._silence_started_at = now

    def _publish_feedback(self, goal_handle, phase, started_at, partial_result=''):
        feedback = WhisperRecongnition.Feedback()
        feedback.phase = phase
        feedback.elapsed_time = float(time.monotonic() - started_at)
        feedback.partial_result = partial_result
        goal_handle.publish_feedback(feedback)

    def _set_mic(self, enabled, goal_handle):
        timeout = float(self.get_parameter('mic_service_timeout_sec').value)
        if not math.isfinite(timeout) or timeout <= 0.0:
            return False, 'invalid mic_service_timeout_sec'
        request = SetBool.Request()
        request.data = bool(enabled)
        try:
            future = self._mic_client.call_async(request)
        except Exception as error:
            return False, f'failed to call microphone service: {error}'

        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            if enabled and goal_handle.is_cancel_requested:
                return False, 'canceled while enabling microphone'
            time.sleep(0.05)

        if not future.done():
            return False, 'microphone service timed out'
        try:
            response = future.result()
        except Exception as error:
            return False, f'microphone service failed: {error}'
        if response is None or not response.success:
            detail = '' if response is None else response.message
            return False, f'microphone service rejected request: {detail}'
        return True, response.message

    def _stop_capture(self):
        with self._state_lock:
            self._capturing = False
            samples = (
                np.concatenate(self._chunks).astype(np.int16, copy=False)
                if self._chunks else np.empty(0, dtype=np.int16)
            )
            speech_detected = self._speech_detected
        return samples, speech_detected

    @staticmethod
    def _result(text='', language='', success=False, message=''):
        result = WhisperRecongnition.Result()
        result.result = text
        result.language = language
        result.success = success
        result.message = message
        return result

    def _execute(self, goal_handle):
        goal = goal_handle.request
        started_at = time.monotonic()
        mic_enabled = False
        try:
            self._publish_feedback(
                goal_handle, WhisperRecongnition.Feedback.WAITING_FOR_MIC, started_at)
            if not self.voice_recong_pin(wait=False):
                self.get_logger().warning(
                    'Failed to play the recognition-start notification sound.'
                )
            start_notification_delay = float(
                self.get_parameter('start_notification_delay_sec').value
            )
            if math.isfinite(start_notification_delay) and start_notification_delay > 0.0:
                time.sleep(start_notification_delay)
            ok, message = self._set_mic(True, goal_handle)
            if not ok:
                result = self._result(
                    message='canceled'
                    if goal_handle.is_cancel_requested else message
                )
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                else:
                    goal_handle.abort()
                return result
            mic_enabled = True

            with self._state_lock:
                self._chunks = []
                self._speech_detected = False
                self._silence_started_at = None
                self._capturing = True

            record_started_at = time.monotonic()
            next_feedback_at = record_started_at
            record_reason = 'maximum recording time reached'
            while rclpy.ok(context=self.context):
                now = time.monotonic()
                elapsed = now - record_started_at
                if goal_handle.is_cancel_requested:
                    result = self._result(message='canceled')
                    goal_handle.canceled()
                    return result

                with self._state_lock:
                    silence_started_at = self._silence_started_at
                    speech_detected = self._speech_detected
                if (
                    speech_detected
                    and silence_started_at is not None
                    and elapsed >= goal.minimum_recording_time
                    and now - silence_started_at >= goal.max_silence_time
                ):
                    record_reason = 'silence detected'
                    break
                if elapsed >= goal.maximum_recording_time:
                    break
                if now >= next_feedback_at:
                    self._publish_feedback(
                        goal_handle,
                        WhisperRecongnition.Feedback.RECORDING,
                        record_started_at,
                    )
                    next_feedback_at = now + float(
                        self.get_parameter('feedback_period_sec').value)
                time.sleep(0.02)

            raw_samples, speech_detected = self._stop_capture()
            if raw_samples.size == 0:
                result = self._result(message='no microphone audio received')
                goal_handle.abort()
                return result
            if not speech_detected:
                result = self._result(message='no speech detected before maximum recording time')
                goal_handle.abort()
                return result

            conditioned = condition_mic_audio(raw_samples, noise_samples=None)
            if conditioned.processed_samples.size == 0 or not conditioned.speech_detected:
                result = self._result(message='no speech detected after audio conditioning')
                goal_handle.abort()
                return result

            self._publish_feedback(
                goal_handle, WhisperRecongnition.Feedback.TRANSCRIBING, started_at)
            audio = conditioned.processed_samples.astype(np.float32) / 32768.0
            segments, info = self.whisper.transcribe(
                audio,
                language=goal.language or None,
                beam_size=int(self.get_parameter('beam_size').value),
                vad_filter=False,
                condition_on_previous_text=False,
            )
            parts = []
            for segment in segments:
                if goal_handle.is_cancel_requested:
                    result = self._result(message='canceled')
                    goal_handle.canceled()
                    return result
                parts.append(segment.text)
                self._publish_feedback(
                    goal_handle,
                    WhisperRecongnition.Feedback.TRANSCRIBING,
                    started_at,
                    ''.join(parts).strip(),
                )

            recognized_text = ''.join(parts).strip()
            detected_language = getattr(info, 'language', '')
            if detected_language not in ('en', 'ja'):
                result = self._result(
                    language=detected_language,
                    message='unsupported automatically detected language',
                )
                goal_handle.abort()
                return result
            if not recognized_text:
                result = self._result(
                    language=detected_language,
                    message=f'empty transcription after {record_reason}',
                )
                goal_handle.abort()
                return result

            result = self._result(
                text=recognized_text,
                language=detected_language,
                success=True,
                message=record_reason,
            )
            goal_handle.succeed()
            return result
        except Exception as error:
            self.get_logger().error(f'Whisper recognition failed: {error}')
            result = self._result(message=f'whisper recognition failed: {error}')
            goal_handle.abort()
            return result
        finally:
            self._stop_capture()
            if mic_enabled:
                ok, message = self._set_mic(False, goal_handle)
                if not ok:
                    self.get_logger().error(f'Failed to disable microphone: {message}')
                # 終了音が再生枠を解放してから認識結果を返す。
                elif not self.disagree(wait=True):
                    self.get_logger().warning(
                        'Failed to play the recognition-end notification sound.'
                    )
            with self._state_lock:
                self._busy = False


def main():
    rclpy.init()
    node = WhisperNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
