"""RGB画像と整列済み深度画像を安全に扱うための共通処理。

ROSメッセージの具体的な型には依存しない。YOLOコンテナと把持側の両方から
同じ検証規則を使い、時刻・画像サイズ・光学座標系・カメラ校正の不一致を
座標計算へ持ち込まないことを目的とする。
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Any, Mapping, Optional, TypedDict


MAX_PAIR_SKEW_SEC = 0.05
MAX_FRAME_AGE_SEC = 2.0
MAX_FUTURE_STAMP_SEC = 0.1


class CameraModel(TypedDict):
    """JSONでYOLOから把持側へ渡すカメラ内部パラメータ。"""

    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    d: list[float]
    distortion_model: str
    frame_id: str
    source: str


def stamp_ns(message: Any) -> int:
    """ROSメッセージのheader stampをナノ秒へ変換する。"""

    stamp = message.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


@dataclass(frozen=True)
class ReceivedFrame:
    """ROSメッセージと、このノードが受信したROS時刻。"""

    message: Any
    received_ns: int


class FramePairs:
    """時刻が近い未使用のRGB・深度画像を1組ずつ返す。

    ``take``で返した画像は再利用しない。同じ画素を参照できる整列済み画像だけを
    通すため、画像サイズと光学座標系もここで確認する。RGB・深度間の同期には
    カメラstamp、鮮度にはノード受信時刻を使い、カメラ時計の固定ずれを許容する。
    """

    def __init__(self, cache_size: int = 30) -> None:
        self.rgb = deque(maxlen=cache_size)
        self.depth = deque(maxlen=cache_size)
        self.last_rgb_stamp_ns = 0
        self.last_depth_stamp_ns = 0
        self.last_pair_received_ns = 0

    def add_rgb(self, message: Any, received_ns: int) -> None:
        """RGB画像をノード受信時刻と一緒に保存する。"""

        self.rgb.append(ReceivedFrame(message, int(received_ns)))

    def add_depth(self, message: Any, received_ns: int) -> None:
        """整列済み深度画像をノード受信時刻と一緒に保存する。"""

        self.depth.append(ReceivedFrame(message, int(received_ns)))

    @staticmethod
    def _is_fresh(
        frame: ReceivedFrame,
        previous_stamp_ns: int,
        now_ns: int,
    ) -> bool:
        current_stamp_ns = stamp_ns(frame.message)
        age_sec = (now_ns - frame.received_ns) / 1e9
        return (
            current_stamp_ns > previous_stamp_ns
            and -MAX_FUTURE_STAMP_SEC <= age_sec <= MAX_FRAME_AGE_SEC
        )

    def take(self, now_ns: int) -> Optional[tuple[Any, Any]]:
        """最新側から探索し、時刻差が許容内の未使用ペアを返す。"""

        unused_depth = [
            frame
            for frame in self.depth
            if self._is_fresh(frame, self.last_depth_stamp_ns, now_ns)
        ]
        if not unused_depth:
            return None

        for rgb_frame in reversed(self.rgb):
            if not self._is_fresh(rgb_frame, self.last_rgb_stamp_ns, now_ns):
                continue

            rgb_message = rgb_frame.message
            rgb_stamp_ns = stamp_ns(rgb_message)
            depth_frame = min(
                unused_depth,
                key=lambda frame: abs(stamp_ns(frame.message) - rgb_stamp_ns),
            )
            depth_message = depth_frame.message
            depth_stamp_ns = stamp_ns(depth_message)
            if abs(depth_stamp_ns - rgb_stamp_ns) / 1e9 > MAX_PAIR_SKEW_SEC:
                continue

            self._validate_aligned_pair(rgb_message, depth_message)
            self.last_rgb_stamp_ns = rgb_stamp_ns
            self.last_depth_stamp_ns = depth_stamp_ns
            self.last_pair_received_ns = max(
                rgb_frame.received_ns,
                depth_frame.received_ns,
            )
            return rgb_message, depth_message

        return None

    @staticmethod
    def _validate_aligned_pair(rgb_message: Any, depth_message: Any) -> None:
        if (rgb_message.width, rgb_message.height) != (
            depth_message.width,
            depth_message.height,
        ):
            raise ValueError("RGB/aligned depth image sizes differ")

        rgb_frame = str(rgb_message.header.frame_id)
        depth_frame = str(depth_message.header.frame_id)
        if not rgb_frame or rgb_frame != depth_frame:
            raise ValueError("RGB/aligned depth optical frames differ")


def validate_camera(
    model: Mapping[str, Any],
    width: int,
    height: int,
    frame_id: str,
) -> CameraModel:
    """外部から受け取ったCameraInfo相当の辞書を検証・正規化する。"""

    if not isinstance(model, Mapping):
        raise TypeError("Camera model must be a mapping")

    try:
        normalized: CameraModel = {
            "width": int(model["width"]),
            "height": int(model["height"]),
            "fx": float(model["fx"]),
            "fy": float(model["fy"]),
            "cx": float(model["cx"]),
            "cy": float(model["cy"]),
            "d": [float(value) for value in model.get("d", [])],
            "distortion_model": str(model.get("distortion_model", "")),
            "frame_id": str(model["frame_id"]),
            "source": str(model.get("source", "unknown")),
        }
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"Malformed camera model: {error}") from error

    if normalized["width"] <= 0 or normalized["height"] <= 0:
        raise ValueError("CameraInfo dimensions must be positive")
    if (normalized["width"], normalized["height"]) != (int(width), int(height)):
        raise ValueError("CameraInfo/image size mismatch")
    if not frame_id or normalized["frame_id"] != frame_id:
        raise ValueError("CameraInfo/image frame mismatch")

    intrinsics = [normalized[key] for key in ("fx", "fy", "cx", "cy")]
    if not all(math.isfinite(value) for value in intrinsics):
        raise ValueError("Invalid camera intrinsics")
    if normalized["fx"] <= 0.0 or normalized["fy"] <= 0.0:
        raise ValueError("Camera focal lengths must be positive")
    if not all(math.isfinite(value) for value in normalized["d"]):
        raise ValueError("Invalid camera distortion coefficients")

    distortion_model = normalized["distortion_model"]
    coefficient_count = len(normalized["d"])
    supported_models = ("", "plumb_bob", "rational_polynomial", "equidistant")
    if distortion_model not in supported_models:
        raise ValueError(f"Unsupported distortion model: {distortion_model}")
    if distortion_model == "" and any(normalized["d"]):
        raise ValueError("Distortion coefficients without a model")
    if distortion_model == "equidistant" and coefficient_count != 4:
        raise ValueError("Equidistant model requires four coefficients")
    if (
        distortion_model in ("plumb_bob", "rational_polynomial")
        and coefficient_count not in (0, 4, 5, 8, 12, 14)
    ):
        raise ValueError("Invalid distortion coefficient count")

    return normalized


def camera_from_info(info: Any, image: Any) -> Optional[CameraModel]:
    """ROS CameraInfoを辞書へ変換する。未受信・未校正ならNoneを返す。"""

    if info is None:
        return None
    try:
        if len(info.k) < 9 or float(info.k[0]) == 0.0:
            return None
        model = {
            "width": info.width,
            "height": info.height,
            "fx": info.k[0],
            "fy": info.k[4],
            "cx": info.k[2],
            "cy": info.k[5],
            "d": list(info.d),
            "distortion_model": info.distortion_model,
            "frame_id": info.header.frame_id,
            "source": "camera_info",
        }
    except (AttributeError, TypeError) as error:
        raise ValueError(f"Malformed CameraInfo: {error}") from error

    return validate_camera(model, image.width, image.height, image.header.frame_id)


def deproject(
    u: float,
    v: float,
    depth_m: float,
    model: Mapping[str, Any],
) -> tuple[float, float, float]:
    """RGB画素と光学軸方向の深度から、カメラ座標X/Y/Zを求める。"""

    if not math.isfinite(depth_m) or depth_m <= 0.0:
        raise ValueError("Depth must be a positive finite value")

    x = (float(u) - float(model["cx"])) / float(model["fx"])
    y = (float(v) - float(model["cy"])) / float(model["fy"])
    distortion = [float(value) for value in model.get("d", [])]
    if any(distortion):
        import cv2
        import numpy as np

        camera_matrix = np.array(
            [
                [model["fx"], 0.0, model["cx"]],
                [0.0, model["fy"], model["cy"]],
                [0.0, 0.0, 1.0],
            ],
            dtype=float,
        )
        pixels = np.array([[[u, v]]], dtype=float)
        coefficients = np.array(distortion, dtype=float)
        if model["distortion_model"] == "equidistant":
            ray = cv2.fisheye.undistortPoints(
                pixels, camera_matrix, coefficients
            )
        else:
            ray = cv2.undistortPoints(pixels, camera_matrix, coefficients)
        x, y = map(float, ray[0, 0])

    point = (x * depth_m, y * depth_m, depth_m)
    if not all(math.isfinite(value) for value in point):
        raise ValueError("Non-finite camera point")
    return point


def depth_in_meters(values: Any, encoding: str):
    """ROS Image encodingに従って深度配列をメートル単位へ変換する。"""

    import numpy as np

    depth = np.asarray(values, dtype=np.float32)
    normalized_encoding = str(encoding).upper()
    if normalized_encoding == "16UC1":
        return depth * 0.001
    if normalized_encoding == "32FC1":
        return depth
    raise ValueError(f"Unsupported depth encoding: {encoding}")
