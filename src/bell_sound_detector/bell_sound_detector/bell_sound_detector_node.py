from __future__ import annotations

import json
from dataclasses import asdict

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Int16MultiArray, String
from std_srvs.srv import Trigger

from bell_sound_detector.bell_detector import BellDetectionConfig, BellDetector
from bell_sound_detector.waiter import BellWaiter


class BellSoundDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("bell_sound_detector")
        self._callback_group = ReentrantCallbackGroup()
        self._detector = BellDetector(self._make_config())
        self._waiter = BellWaiter()
        self._wait_timeout_s = self._declare_float("wait_timeout_s", 30.0)

        audio_topic = self.declare_parameter("audio_topic", "/audio/raw").value
        self._detected_pub = self.create_publisher(Bool, "~/detected", 10)
        self._event_pub = self.create_publisher(String, "~/event", 10)
        self._audio_sub = self.create_subscription(
            Int16MultiArray,
            audio_topic,
            self._on_audio,
            10,
            callback_group=self._callback_group,
        )
        self._wait_service = self.create_service(
            Trigger,
            "wait_bell_sound",
            self._on_wait_bell_sound,
            callback_group=self._callback_group,
        )
        self.get_logger().info(f"bell_sound_detector subscribed to {audio_topic}")

    def _make_config(self) -> BellDetectionConfig:
        return BellDetectionConfig(
            sample_rate=self._declare_int("sample_rate", 16000),
            frame_size=self._declare_int("frame_size", 512),
            hop_size=self._declare_int("hop_size", 160),
            bell_band_min_hz=self._declare_float("bell_band_min_hz", 800.0),
            bell_band_max_hz=self._declare_float("bell_band_max_hz", 4000.0),
            total_band_min_hz=self._declare_float("total_band_min_hz", 80.0),
            total_band_max_hz=self._declare_float("total_band_max_hz", 7000.0),
            min_snr_db=self._declare_float("min_snr_db", 10.0),
            min_high_energy_ratio=self._declare_float("min_high_energy_ratio", 0.30),
            min_peak_to_median_db=self._declare_float("min_peak_to_median_db", 10.0),
            max_spectral_flatness=self._declare_float("max_spectral_flatness", 0.35),
            min_note1_duration_s=self._declare_float("min_note1_duration_s", 0.08),
            max_note1_duration_s=self._declare_float("max_note1_duration_s", 0.70),
            min_note2_duration_s=self._declare_float("min_note2_duration_s", 0.12),
            max_note2_duration_s=self._declare_float("max_note2_duration_s", 1.20),
            min_gap_s=self._declare_float("min_gap_s", 0.03),
            max_gap_s=self._declare_float("max_gap_s", 0.70),
            max_freq_std_hz=self._declare_float("max_freq_std_hz", 180.0),
            max_freq_std_ratio=self._declare_float("max_freq_std_ratio", 0.15),
            freq_relation=str(self.declare_parameter("freq_relation", "loose").value),
            max_missing_tonal_frames=self._declare_int("max_missing_tonal_frames", 3),
            cooldown_s=self._declare_float("cooldown_s", 3.0),
            noise_history_size=self._declare_int("noise_history_size", 300),
            warmup_s=self._declare_float("warmup_s", 0.3),
            min_event_score=self._declare_float("min_event_score", 0.65),
        )

    def _declare_int(self, name: str, default: int) -> int:
        return int(self.declare_parameter(name, default).value)

    def _declare_float(self, name: str, default: float) -> float:
        return float(self.declare_parameter(name, default).value)

    def _on_audio(self, msg: Int16MultiArray) -> None:
        audio = np.asarray(msg.data, dtype=np.int16)
        for event in self._detector.predict(audio):
            event_dict = asdict(event)
            self._detected_pub.publish(Bool(data=True))
            self._event_pub.publish(String(data=json.dumps(event_dict)))
            self._waiter.notify(event_dict)
            self.get_logger().info(
                "bell sound detected: "
                f"score={event.score:.2f}, "
                f"note1={event.note1_freq_hz:.0f}Hz, "
                f"note2={event.note2_freq_hz:.0f}Hz"
            )

    def _on_wait_bell_sound(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request
        event = self._waiter.wait(self._wait_timeout_s)
        if event is None:
            response.success = False
            response.message = f"Timed out after {self._wait_timeout_s:.1f}s"
        else:
            response.success = True
            response.message = json.dumps(event)
        return response


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = BellSoundDetectorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
