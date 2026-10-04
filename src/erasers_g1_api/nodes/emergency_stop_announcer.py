"""停止処理を待たせず、日本語と英語で緊急停止の状態を通知する。"""

import math
from threading import Condition, Thread

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from erasers_g1_api.tts import VoicevoxTTS, VuiTTS


class EmergencyStopAnnouncer(Node):
    """制御経路を持たない、停止状態の音声通知ノード。"""

    def __init__(self):
        super().__init__("emergency_stop_announcer")
        interval = self.declare_parameter("repeat_interval_sec", 10.0).value
        if not math.isfinite(interval) or interval <= 0:
            raise ValueError("repeat_interval_sec は正の有限値にしてください")
        self._interval = interval
        self._condition = Condition()
        self._active = None
        self._latched = False
        self._phase = None
        self._generation = 0
        self._stopping = False
        self._japanese = VoicevoxTTS(self, speaker_id=26, timeout_sec=30.0)
        self._english = VuiTTS(self, speaker_id=1, timeout_sec=30.0)
        qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._active_sub = self.create_subscription(
            Bool, "/emergency_stop/active", self._active_callback, qos,
        )
        self._latched_sub = self.create_subscription(
            Bool, "/emergency_stop/latched", self._latched_callback, qos,
        )
        self._worker = Thread(target=self._run, name="emergency_stop_voice")
        self._worker.start()

    def _active_callback(self, message):
        with self._condition:
            self._active = message.data
            self._latched = self._latched or message.data
            self._update_phase()

    def _latched_callback(self, message):
        with self._condition:
            self._latched = self._latched or message.data
            self._update_phase()

    def _update_phase(self):
        phase = "stop" if self._active else (
            "released" if self._active is False and self._latched else None
        )
        if phase != self._phase:
            self._phase = phase
            self._generation += 1
            self._condition.notify_all()

    def _run(self):
        while True:
            with self._condition:
                self._condition.wait_for(lambda: self._stopping or self._phase is not None)
                if self._stopping:
                    return
                phase = self._phase
                generation = self._generation
            if phase == "stop":
                japanese = "緊急停止"
                english = "Emergency Stop"
            else:
                japanese = "緊急停止ボタンが解除されました．コンテナを再起動してください"
                english = (
                    "The emergency stop button has been released. "
                    "Please restart the container."
                )
            # 同じワーカーで完了を待ち、二言語を同時に再生しない。
            for speaker, text in ((self._japanese, japanese), (self._english, english)):
                with self._condition:
                    if self._stopping:
                        return
                    if generation != self._generation:
                        break
                try:
                    if not speaker.say(text, wait=True):
                        self.get_logger().warning("停止状態の音声通知が完了しませんでした")
                except Exception as error:
                    self.get_logger().error(f"音声通知が失敗しました: {error}")
            with self._condition:
                self._condition.wait_for(
                    lambda: self._stopping or generation != self._generation,
                    timeout=self._interval,
                )

    def close(self):
        """新しい発話を止め、期限付きの発話待機が終了してからノードを破棄する。"""
        with self._condition:
            self._stopping = True
            self._condition.notify_all()
        self._worker.join()


def main(args=None):
    rclpy.init(args=args)
    node = EmergencyStopAnnouncer()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
