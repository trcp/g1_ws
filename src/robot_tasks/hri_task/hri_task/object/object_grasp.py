#!/usr/bin/env python3
"""YOLO の認識結果から物体把持用の簡易 IK を計算する。

このモジュールは腕を動かさない。認識結果、3 次元座標、IK 結果を
``GraspPlan`` に保存し、実行側が任意の腕制御ツールへ渡せるようにする。

IK と把持点の決め方は、既存の bag_grasp のロジックを基本的に踏襲する。
"""

from __future__ import annotations

import json
import math
import os
import sys
import time
from dataclasses import asdict, dataclass
from typing import Any, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# test ディレクトリから親ディレクトリの設定を読み込む。
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from g1_config import CameraParams, RobotGeometry  # noqa: E402


@dataclass
class ObjectDetection:
    """YOLO から受け取った、把持対象1個分の情報。"""

    label: str
    confidence: float
    bbox: list[float]
    depth_m: float
    valid_depth: bool


@dataclass
class GraspPlan:
    """認識結果から計算した把持計画。

    ``joints`` はまだロボットへ送信しない。新しい腕制御ツールへ接続するときに
    この辞書を渡す。
    """

    object_name: str
    grasp_strategy: str
    confidence: float
    bbox: list[float]
    grasp_pixel: tuple[float, float]
    depth_m: float
    camera_xyz: tuple[float, float, float]
    robot_xyz: tuple[float, float, float]
    arm_distance_m: float
    reachable: bool
    reason: str
    joints: dict[str, float]

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def safe_acos(value: float) -> float:
    """数値誤差を [-1, 1] に収めて acos を計算する。"""

    return math.acos(max(-1.0, min(1.0, value)))


def solve_2link_ik(
    forward: float,
    downward: float,
    upper_arm_length: float,
    lower_arm_length: float,
) -> tuple[float, float]:
    """既存の bag_grasp と同じ2リンクIKを計算する。

    到達範囲外では、既存ロジックと同じく計算用の距離だけを境界へ丸める。
    到達できるかどうかは ``GraspPlan.reachable`` に別途保存する。
    """

    distance = math.hypot(forward, downward)
    max_reach = upper_arm_length + lower_arm_length
    min_reach = abs(upper_arm_length - lower_arm_length)

    if distance > max_reach:
        ratio = (max_reach - 0.001) / distance if distance > 0.0 else 0.0
        forward *= ratio
        downward *= ratio
        distance = max_reach - 0.001
    elif distance < min_reach:
        ratio = (min_reach + 0.001) / distance if distance > 0.0 else 0.0
        forward *= ratio
        downward *= ratio
        distance = min_reach + 0.001

    cos_elbow = (
        upper_arm_length**2 + lower_arm_length**2 - distance**2
    ) / (2.0 * upper_arm_length * lower_arm_length)
    elbow_inner = safe_acos(cos_elbow)
    elbow_angle = math.pi - elbow_inner

    angle_to_target = math.atan2(forward, downward + 1e-9)
    cos_beta = (
        upper_arm_length**2 + distance**2 - lower_arm_length**2
    ) / (2.0 * upper_arm_length * distance)
    beta = safe_acos(cos_beta)
    shoulder_angle = angle_to_target - beta

    return shoulder_angle, elbow_angle


def calculate_object_grasp_plan(
    detection: ObjectDetection,
    head_tilt: float = -0.5,
    grasp_strategy: str = "center",
) -> GraspPlan:
    """物体検出結果から右腕用の把持計画を計算する。

    ``center`` は一般物体用に bbox 中心を使う。
    ``bag`` は既存のバッグ把持と同じく bbox 上部の少し右側を使う。
    """

    geometry = RobotGeometry()
    camera = CameraParams()

    x1, y1, x2, y2 = detection.bbox
    bbox_width = x2 - x1
    bbox_height = y2 - y1

    bbox_center_x = (x1 + x2) / 2.0
    bbox_center_y = (y1 + y2) / 2.0
    if grasp_strategy == "center":
        grasp_x = bbox_center_x
        grasp_y = bbox_center_y
    elif grasp_strategy == "bag":
        grasp_x = bbox_center_x + bbox_width * 0.15
        grasp_y = y1 + bbox_height * 0.05
    else:
        raise ValueError(f"Unknown grasp strategy: {grasp_strategy}")
    depth_m = detection.depth_m

    # カメラ座標系: X=右、Y=下、Z=奥。
    camera_x = depth_m * (grasp_x - camera.CX) / camera.FX
    camera_y = depth_m * (grasp_y - camera.CY) / camera.FY
    camera_z = depth_m

    # ロボット座標系: X=前、Y=左、Z=上。
    cos_tilt = math.cos(head_tilt)
    sin_tilt = math.sin(head_tilt)
    object_x = depth_m * cos_tilt + camera_y * sin_tilt
    object_y = -camera_x
    object_z = depth_m * sin_tilt - camera_y * cos_tilt

    camera_offset_x = geometry.NECK_BASE_X + camera.CAM_FROM_NECK_X
    camera_offset_y = geometry.NECK_BASE_Y + camera.CAM_FROM_NECK_Y
    camera_offset_z = geometry.NECK_BASE_Z + camera.CAM_FROM_NECK_Z

    target_x = object_x + camera_offset_x
    target_y = object_y + camera_offset_y
    target_z = object_z + camera_offset_z

    # 右肩の正面へ対象物を合わせる腰角度。
    right_shoulder_y = -geometry.SHOULDER_Y
    horizontal_distance = math.hypot(target_x, target_y)
    asin_value = (
        right_shoulder_y / horizontal_distance
        if horizontal_distance > abs(right_shoulder_y)
        else math.copysign(1.0, right_shoulder_y)
    )
    waist_yaw = math.atan2(target_y, target_x) - math.asin(asin_value)
    waist_yaw = max(-1.2, min(1.2, waist_yaw))

    cos_waist = math.cos(waist_yaw)
    sin_waist = math.sin(waist_yaw)
    shoulder_x = (
        geometry.SHOULDER_X * cos_waist - right_shoulder_y * sin_waist
    )
    shoulder_y = (
        geometry.SHOULDER_X * sin_waist + right_shoulder_y * cos_waist
    )

    forward = (
        (target_x - shoulder_x) * cos_waist
        + (target_y - shoulder_y) * sin_waist
    )
    height_from_shoulder = target_z - geometry.SHOULDER_Z
    downward = -height_from_shoulder
    arm_distance = math.hypot(forward, downward)

    min_reach = abs(geometry.L_UPPER - geometry.L_LOWER)
    max_reach = geometry.L_UPPER + geometry.L_LOWER
    reachable = min_reach <= arm_distance <= max_reach

    if arm_distance > max_reach:
        reason = (
            f"target is too far: {arm_distance:.3f} m > {max_reach:.3f} m"
        )
    elif arm_distance < min_reach:
        reason = (
            f"target is too close: {arm_distance:.3f} m < {min_reach:.3f} m"
        )
    else:
        reason = ""

    shoulder_angle, elbow_angle = solve_2link_ik(
        forward,
        downward,
        geometry.L_UPPER,
        geometry.L_LOWER,
    )

    joints = {
        "waist_yaw_joint": waist_yaw,
        "right_shoulder_pitch_joint": -shoulder_angle,
        "right_elbow_joint": 1.5708 - elbow_angle,
        "right_shoulder_roll_joint": -0.2,
        "right_shoulder_yaw_joint": 0.0,
        "right_wrist_roll_joint": 0.0,
    }

    return GraspPlan(
        object_name=detection.label,
        grasp_strategy=grasp_strategy,
        confidence=detection.confidence,
        bbox=detection.bbox,
        grasp_pixel=(grasp_x, grasp_y),
        depth_m=depth_m,
        camera_xyz=(camera_x, camera_y, camera_z),
        robot_xyz=(target_x, target_y, target_z),
        arm_distance_m=arm_distance,
        reachable=reachable,
        reason=reason,
        joints=joints,
    )


class YoloObjectDetector:
    """既存YOLOノードへ対象名を送り、最も近い対象を1つ取得する。"""

    def __init__(
        self,
        node: Node,
        command_topic: str = "/yolo_human/command",
        result_topic: str = "/yolo_human/result",
    ) -> None:
        self.node = node
        self.command_topic = command_topic
        self.result_topic = result_topic
        self.latest_message: Optional[str] = None
        self.received_result_count = 0
        self.received_empty_count = 0
        self.last_failure_reason = ""
        self.command_publisher = node.create_publisher(String, command_topic, 10)
        self.result_subscription = node.create_subscription(
            String,
            result_topic,
            self._result_callback,
            10,
        )

    def _result_callback(self, message: String) -> None:
        self.latest_message = message.data
        self.received_result_count += 1

    def wait_until_ready(self, timeout: float = 5.0) -> bool:
        """YOLOノードのcommand購読とresult配信が見えるまで待つ。"""

        start_time = time.time()
        while rclpy.ok() and time.time() - start_time < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            command_subscribers = self.node.count_subscribers(self.command_topic)
            result_publishers = self.node.count_publishers(self.result_topic)
            if command_subscribers > 0 and result_publishers > 0:
                return True

        command_subscribers = self.node.count_subscribers(self.command_topic)
        result_publishers = self.node.count_publishers(self.result_topic)
        self.last_failure_reason = (
            "YOLO node is not connected: "
            f"command_subscribers={command_subscribers}, "
            f"result_publishers={result_publishers}"
        )
        return False

    def start(self, target_name: str) -> None:
        self.latest_message = None
        message = String()
        message.data = json.dumps(
            {"command": "start", "classes": [target_name]}
        )
        self.command_publisher.publish(message)
        self.node.get_logger().info(
            f"[OBJECT GRASP] YOLO start: target={target_name}"
        )

    def stop(self) -> None:
        message = String()
        message.data = json.dumps({"command": "stop"})
        self.command_publisher.publish(message)
        self.node.get_logger().info("[OBJECT GRASP] YOLO stop")

    def detect(self, target_name: str, timeout: float = 8.0) -> Optional[ObjectDetection]:
        """指定ラベルのうち、既存ロジックと同じく最も近いものを返す。"""

        target_name = target_name.strip().lower()
        self.received_result_count = 0
        self.received_empty_count = 0
        self.last_failure_reason = ""

        if not self.wait_until_ready(timeout=min(5.0, timeout)):
            return None

        # 接続確認中に届いた古い結果を除外し、start後の結果だけを数える。
        self.received_result_count = 0
        self.received_empty_count = 0
        self.start(target_name)
        start_time = time.time()

        while rclpy.ok() and time.time() - start_time < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.latest_message is None:
                continue

            raw_message = self.latest_message
            self.latest_message = None

            try:
                detections = json.loads(raw_message)
            except json.JSONDecodeError:
                self.node.get_logger().warn("Invalid YOLO JSON was ignored.")
                continue

            if not isinstance(detections, list):
                continue

            if not detections:
                self.received_empty_count += 1
                continue

            candidates = [
                item
                for item in detections
                if (
                    isinstance(item, dict)
                    and str(item.get("label", "")).strip().lower() == target_name
                )
            ]
            if not candidates:
                continue

            try:
                target = min(
                    candidates,
                    key=lambda item: float(item.get("distance_z", 999.0)),
                )
                bbox = target.get("bbox")
                if (
                    not isinstance(bbox, list)
                    or len(bbox) != 4
                    or float(bbox[2]) <= float(bbox[0])
                    or float(bbox[3]) <= float(bbox[1])
                ):
                    self.last_failure_reason = f"Object bbox is invalid: {bbox}"
                    self.node.get_logger().warn(f"Invalid bbox was ignored: {bbox}")
                    continue

                depth_m = float(target.get("distance_z", 999.0))
                valid_depth = bool(target.get("valid_depth", True))
                if not valid_depth or not math.isfinite(depth_m) or not 0.1 < depth_m < 10.0:
                    self.last_failure_reason = (
                        f"Object depth is invalid: depth={depth_m}, "
                        f"valid_depth={valid_depth}"
                    )
                    self.node.get_logger().warn(
                        f"Invalid depth was ignored: depth={depth_m}, "
                        f"valid_depth={valid_depth}"
                    )
                    continue

                return ObjectDetection(
                    label=str(target.get("label", target_name)),
                    confidence=float(target.get("confidence", 0.0)),
                    bbox=[float(value) for value in bbox],
                    depth_m=depth_m,
                    valid_depth=valid_depth,
                )
            except (TypeError, ValueError) as error:
                self.last_failure_reason = f"Detection data is invalid: {error}"
                self.node.get_logger().warn(
                    f"Invalid detection was ignored: {error}"
                )

        if self.last_failure_reason:
            return None
        if self.received_result_count == 0:
            self.last_failure_reason = (
                "YOLO node is connected, but no result message was received. "
                "Check RGB/depth camera topics and YOLO container logs."
            )
        else:
            self.last_failure_reason = (
                f"YOLO results were received, but '{target_name}' was not detected "
                f"within {timeout:.1f} seconds."
            )
        return None
