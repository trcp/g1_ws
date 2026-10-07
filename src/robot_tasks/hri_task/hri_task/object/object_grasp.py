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
from collections import deque
from dataclasses import asdict, dataclass, replace
from statistics import median
from typing import Any, Mapping, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# このファイルの1階層上にある共通設定を読み込む。
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from g1_config import CameraParams, RobotGeometry  # noqa: E402
from rgbd_geometry import (
    MAX_FRAME_AGE_SEC,
    MAX_FUTURE_STAMP_SEC,
    MAX_PAIR_SKEW_SEC,
    CameraModel,
    deproject,
    validate_camera,
)

STABLE_MIN_FRAMES = 5
STABLE_MIN_DURATION_SEC = 0.5
STABLE_MAX_GAP_SEC = 0.5
STABLE_CENTER_RANGE_PX = 6.0
STABLE_POSITION_RANGE_M = 0.02
MIN_VALID_DEPTH_M = 0.1
MAX_VALID_DEPTH_M = 10.0
MIN_TRACKING_IOU = 0.5
JOINT_LIMIT_MARGIN_RAD = 0.10
JOINT_LIMITS = {
    "waist_yaw_joint": (-2.618, 2.618),
    "left_shoulder_pitch_joint": (-3.0892, 2.6704),
    "left_shoulder_roll_joint": (-1.5882, 2.2515),
    "left_shoulder_yaw_joint": (-2.618, 2.618),
    "left_elbow_joint": (-1.0472, 2.0944),
    "left_wrist_roll_joint": (-1.972222054, 1.972222054),
    "right_shoulder_pitch_joint": (-3.0892, 2.6704),
    "right_shoulder_roll_joint": (-2.2515, 1.5882),
    "right_shoulder_yaw_joint": (-2.618, 2.618),
    "right_elbow_joint": (-1.0472, 2.0944),
    "right_wrist_roll_joint": (-1.972222054, 1.972222054),
}


def joint_limit_violations(
    joints: Mapping[str, float],
    margin_rad: float = 0.0,
) -> list[str]:
    """URDF関節範囲と指定余裕を外れる目標を列挙する。"""

    if not math.isfinite(margin_rad) or margin_rad < 0.0:
        raise ValueError("Joint limit margin must be finite and non-negative")

    violations = []
    for joint_name, value in joints.items():
        limits = JOINT_LIMITS.get(joint_name)
        if limits is None:
            continue
        lower, upper = limits
        safe_lower = lower + margin_rad
        safe_upper = upper - margin_rad
        if not safe_lower <= value <= safe_upper:
            violations.append(
                f"{joint_name}={value:.3f} outside safe range "
                f"[{safe_lower:.3f}, {safe_upper:.3f}]"
            )
    return violations


def joint_margin_violations(joints: Mapping[str, float]) -> list[str]:
    """URDF関節範囲から0.10 rad以上離れていない目標を列挙する。"""

    return joint_limit_violations(joints, JOINT_LIMIT_MARGIN_RAD)


@dataclass
class ObjectDetection:
    """YOLO から受け取った、把持対象1個分の情報。"""

    label: str
    confidence: float
    bbox: list[float]
    depth_m: float
    valid_depth: bool
    camera_model: Optional[CameraModel] = None
    image_width: int = 640
    image_height: int = 480
    camera_frame: str = ""
    rgb_stamp_ns: int = 0
    depth_stamp_ns: int = 0
    frame_received_ns: int = 0
    result_stamp_ns: int = 0
    track_id: int = -1


def detection_camera(detection: ObjectDetection) -> CameraModel:
    """CameraInfoを優先し、未取得時は既存の640x480固定値を残す。"""

    model = detection.camera_model
    if model is None:
        camera = CameraParams()
        model = {
            "width": camera.IMG_WIDTH,
            "height": camera.IMG_HEIGHT,
            "fx": camera.FX,
            "fy": camera.FY,
            "cx": camera.CX,
            "cy": camera.CY,
            "d": [],
            "distortion_model": "",
            "frame_id": detection.camera_frame,
            "source": "fixed",
        }
    return validate_camera(
        model,
        detection.image_width,
        detection.image_height,
        detection.camera_frame,
    )


def bbox_iou(first: list[float], second: list[float]) -> float:
    """2つのbboxのIntersection over Unionを返す。"""

    intersection_width = max(
        0.0, min(first[2], second[2]) - max(first[0], second[0])
    )
    intersection_height = max(
        0.0, min(first[3], second[3]) - max(first[1], second[1])
    )
    intersection = intersection_width * intersection_height
    first_area = (first[2] - first[0]) * (first[3] - first[1])
    second_area = (second[2] - second[0]) * (second[3] - second[1])
    union = first_area + second_area - intersection
    return intersection / union if union > 0.0 else 0.0


@dataclass(frozen=True)
class StableSample:
    """安定判定に使う1撮影分の検出と投影結果。"""

    detection: ObjectDetection
    center_u: float
    center_v: float
    camera_x: float
    camera_y: float
    camera_z: float

    def values(self) -> tuple[float, float, float, float, float]:
        return (
            self.center_u,
            self.center_v,
            self.camera_x,
            self.camera_y,
            self.camera_z,
        )


class StableDetections:
    """異なる撮影フレームで同じ対象の位置が落ち着くまで待つ。"""

    def __init__(self):
        self.samples = deque(maxlen=30)
        self.last_rgb_stamp = 0
        self.last_depth_stamp = 0

    def clear(self):
        """現在の安定区間だけを捨てる。処理済み時刻は巻き戻さない。"""

        self.samples.clear()

    @staticmethod
    def same_target(previous: ObjectDetection, current: ObjectDetection) -> bool:
        if previous.track_id >= 0:
            return current.track_id == previous.track_id
        return bbox_iou(previous.bbox, current.bbox) >= MIN_TRACKING_IOU

    def _starts_new_window(
        self,
        detection: ObjectDetection,
        model: CameraModel,
    ) -> bool:
        if not self.samples:
            return False
        previous = self.samples[-1].detection
        frame_gap_sec = (detection.rgb_stamp_ns - previous.rgb_stamp_ns) / 1e9
        return (
            frame_gap_sec > STABLE_MAX_GAP_SEC
            or detection_camera(previous) != model
            or not self.same_target(previous, detection)
        )

    def _trim_unstable_prefix(self) -> None:
        limits = (
            STABLE_CENTER_RANGE_PX,
            STABLE_CENTER_RANGE_PX,
            STABLE_POSITION_RANGE_M,
            STABLE_POSITION_RANGE_M,
            STABLE_POSITION_RANGE_M,
        )
        while len(self.samples) > 1:
            columns = list(zip(*(sample.values() for sample in self.samples)))
            if all(
                max(column) - min(column) <= limit
                for column, limit in zip(columns, limits)
            ):
                return
            self.samples.popleft()

    def add(self, detection: ObjectDetection) -> Optional[ObjectDetection]:
        """新しい検出を加え、安定条件を満たした場合だけ中央値を返す。"""

        if (
            detection.rgb_stamp_ns <= self.last_rgb_stamp
            or detection.depth_stamp_ns <= self.last_depth_stamp
        ):
            return None  # 同一画像の再配信では安定フレーム数を増やさない。
        self.last_rgb_stamp = detection.rgb_stamp_ns
        self.last_depth_stamp = detection.depth_stamp_ns

        model = detection_camera(detection)
        if self._starts_new_window(detection, model):
            self.clear()

        u = (detection.bbox[0] + detection.bbox[2]) / 2
        v = (detection.bbox[1] + detection.bbox[3]) / 2
        camera_x, camera_y, camera_z = deproject(u, v, detection.depth_m, model)
        self.samples.append(
            StableSample(detection, u, v, camera_x, camera_y, camera_z)
        )
        self._trim_unstable_prefix()

        if len(self.samples) < STABLE_MIN_FRAMES:
            return None
        first_stamp_ns = self.samples[0].detection.rgb_stamp_ns
        if (detection.rgb_stamp_ns - first_stamp_ns) / 1e9 < STABLE_MIN_DURATION_SEC:
            return None

        return replace(
            detection,
            bbox=[
                median(sample.detection.bbox[index] for sample in self.samples)
                for index in range(4)
            ],
            depth_m=median(
                sample.detection.depth_m for sample in self.samples
            ),
        )


@dataclass
class GraspPlan:
    """認識結果から計算した把持計画。

    ``joints`` はまだロボットへ送信しない。新しい腕制御ツールへ接続するときに
    この辞書を渡す。
    """

    object_name: str
    arm: str
    grasp_strategy: str
    confidence: float
    bbox: list[float]
    grasp_pixel: tuple[float, float]
    depth_m: float
    camera_xyz: tuple[float, float, float]
    robot_xyz: tuple[float, float, float]
    coordinate_source: str
    kinematics_source: str
    shoulder_distance_at_zero_m: float
    arm_distance_m: float
    reachable: bool
    reason: str
    joints: dict[str, float]

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class CameraTransform:
    """カメラ光学座標系からロボット基準座標系への剛体変換。"""

    target_frame: str
    source_frame: str
    translation: tuple[float, float, float]
    rotation_xyzw: tuple[float, float, float, float]


def transform_camera_point(
    camera_xyz: tuple[float, float, float],
    transform: CameraTransform,
) -> tuple[float, float, float]:
    """TFの平行移動とクォータニオンを3次元点へ適用する。"""

    values = (*camera_xyz, *transform.translation, *transform.rotation_xyzw)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("Camera transform contains a non-finite value")

    qx, qy, qz, qw = transform.rotation_xyzw
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1e-9:
        raise ValueError("Camera transform quaternion has zero length")
    qx, qy, qz, qw = (value / norm for value in (qx, qy, qz, qw))

    # q * p * conjugate(q) と同値の回転行列。
    px, py, pz = camera_xyz
    rotated_x = (
        (1.0 - 2.0 * (qy * qy + qz * qz)) * px
        + 2.0 * (qx * qy - qz * qw) * py
        + 2.0 * (qx * qz + qy * qw) * pz
    )
    rotated_y = (
        2.0 * (qx * qy + qz * qw) * px
        + (1.0 - 2.0 * (qx * qx + qz * qz)) * py
        + 2.0 * (qy * qz - qx * qw) * pz
    )
    rotated_z = (
        2.0 * (qx * qz - qy * qw) * px
        + 2.0 * (qy * qz + qx * qw) * py
        + (1.0 - 2.0 * (qx * qx + qy * qy)) * pz
    )
    tx, ty, tz = transform.translation
    return rotated_x + tx, rotated_y + ty, rotated_z + tz


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
    arm: str = "right",
    head_tilt: float = -0.5,
    grasp_strategy: str = "center",
    offset_x_m: float = 0.0,
    offset_y_m: float = 0.0,
    offset_z_m: float = 0.0,
    wrist_roll: float = 0.0,
    camera_transform: Optional[CameraTransform] = None,
) -> GraspPlan:
    """物体検出結果から指定した左右いずれかの把持計画を計算する。

    ``center`` は一般物体用に bbox 中心を使う。
    ``bag`` は既存のバッグ把持と同じく bbox 上部の少し右側を使う。
    """

    if arm not in ("left", "right"):
        raise ValueError(f"Unknown arm: {arm}")

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
    camera_x, camera_y, camera_z = deproject(
        grasp_x, grasp_y, depth_m, detection_camera(detection)
    )

    if camera_transform is not None:
        object_x, object_y, object_z = transform_camera_point(
            (camera_x, camera_y, camera_z), camera_transform
        )
        coordinate_source = (
            f"tf:{camera_transform.target_frame}"
            f"<-{camera_transform.source_frame}"
        )
    else:
        # 後方互換と単体計算用。実機実行側はTFを必須にする。
        # ロボット座標系: X=前、Y=左、Z=上。
        cos_tilt = math.cos(head_tilt)
        sin_tilt = math.sin(head_tilt)
        object_x = depth_m * cos_tilt + camera_y * sin_tilt
        object_y = -camera_x
        object_z = depth_m * sin_tilt - camera_y * cos_tilt

        object_x += geometry.NECK_BASE_X + camera.CAM_FROM_NECK_X
        object_y += geometry.NECK_BASE_Y + camera.CAM_FROM_NECK_Y
        object_z += geometry.NECK_BASE_Z + camera.CAM_FROM_NECK_Z
        coordinate_source = "manual_camera_model"

    # IK の解法は変えず、ロボット座標系で目標位置だけを補正する。
    target_x = object_x + offset_x_m
    target_y = object_y + offset_y_m
    target_z = object_z + offset_z_m

    # 選択した肩の正面へ対象物を合わせる腰角度。
    shoulder_y_at_zero = (
        geometry.SHOULDER_Y if arm == "left" else -geometry.SHOULDER_Y
    )
    shoulder_distance_at_zero = math.sqrt(
        (target_x - geometry.SHOULDER_X) ** 2
        + (target_y - shoulder_y_at_zero) ** 2
        + (target_z - geometry.SHOULDER_Z) ** 2
    )
    horizontal_distance = math.hypot(target_x, target_y)
    asin_value = (
        shoulder_y_at_zero / horizontal_distance
        if horizontal_distance > abs(shoulder_y_at_zero)
        else math.copysign(1.0, shoulder_y_at_zero)
    )
    waist_yaw = math.atan2(target_y, target_x) - math.asin(asin_value)
    waist_yaw = max(-1.2, min(1.2, waist_yaw))

    cos_waist = math.cos(waist_yaw)
    sin_waist = math.sin(waist_yaw)
    shoulder_x = (
        geometry.SHOULDER_X * cos_waist - shoulder_y_at_zero * sin_waist
    )
    shoulder_y = (
        geometry.SHOULDER_X * sin_waist + shoulder_y_at_zero * cos_waist
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

    arm_sign = 1.0 if arm == "left" else -1.0
    joints = {
        "waist_yaw_joint": waist_yaw,
        f"{arm}_shoulder_pitch_joint": -shoulder_angle,
        f"{arm}_elbow_joint": 1.5708 - elbow_angle,
        f"{arm}_shoulder_roll_joint": arm_sign * 0.2,
        f"{arm}_shoulder_yaw_joint": 0.0,
        f"{arm}_wrist_roll_joint": wrist_roll,
    }

    joint_violations = joint_limit_violations(joints)
    if joint_violations:
        reachable = False
        joint_reason = (
            "joint limit exceeded: " + "; ".join(joint_violations)
        )
        reason = f"{reason}; {joint_reason}" if reason else joint_reason

    return GraspPlan(
        object_name=detection.label,
        arm=arm,
        grasp_strategy=grasp_strategy,
        confidence=detection.confidence,
        bbox=detection.bbox,
        grasp_pixel=(grasp_x, grasp_y),
        depth_m=depth_m,
        camera_xyz=(camera_x, camera_y, camera_z),
        robot_xyz=(target_x, target_y, target_z),
        coordinate_source=coordinate_source,
        kinematics_source="simplified_2link",
        shoulder_distance_at_zero_m=shoulder_distance_at_zero,
        arm_distance_m=arm_distance,
        reachable=reachable,
        reason=reason,
        joints=joints,
    )


def parse_yolo_detection(
    payload: Mapping[str, Any],
    request_start_ns: int,
    now_ns: int,
) -> ObjectDetection:
    """YOLO結果JSONの1要素を検証し、ObjectDetectionへ変換する。"""

    try:
        bbox = [float(value) for value in payload["bbox"]]
        image_width = int(payload["image_width"])
        image_height = int(payload["image_height"])
        depth_m = float(payload["distance_z"])
        confidence = float(payload.get("confidence", 0.0))
        rgb_stamp_ns = int(payload["rgb_stamp_ns"])
        depth_stamp_ns = int(payload["depth_stamp_ns"])
        frame_received_ns = int(payload["frame_received_ns"])
        result_stamp_ns = int(payload["result_stamp_ns"])
        camera_frame = str(payload["camera_frame"])
        track_id = int(payload.get("track_id", -1))
        label = str(payload["label"])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"Malformed object detection: {error}") from error

    if (
        len(bbox) != 4
        or not all(math.isfinite(value) for value in bbox)
        or not 0.0 <= bbox[0] < bbox[2] <= image_width
        or not 0.0 <= bbox[1] < bbox[3] <= image_height
    ):
        raise ValueError("Invalid object bbox/image size")
    if (
        payload.get("valid_depth") is not True
        or not math.isfinite(depth_m)
        or not MIN_VALID_DEPTH_M < depth_m < MAX_VALID_DEPTH_M
    ):
        raise ValueError("Invalid object depth")
    if not math.isfinite(confidence):
        raise ValueError("Invalid object confidence")
    if payload.get("depth_aligned") is not True:
        raise ValueError("Aligned depth metadata required; update the YOLO node")
    if not camera_frame:
        raise ValueError("Missing camera optical frame")
    if not label.strip():
        raise ValueError("Missing object label")
    if abs(rgb_stamp_ns - depth_stamp_ns) / 1e9 > MAX_PAIR_SKEW_SEC:
        raise ValueError("RGB/depth timestamps differ")
    if frame_received_ns < request_start_ns:
        raise ValueError("Image reception predates this detection request")
    if result_stamp_ns < frame_received_ns:
        raise ValueError("YOLO result predates image reception")

    for stamp in (frame_received_ns, result_stamp_ns):
        age_sec = (now_ns - stamp) / 1e9
        if not -MAX_FUTURE_STAMP_SEC <= age_sec <= MAX_FRAME_AGE_SEC:
            raise ValueError("Stale YOLO result or image reception")

    detection = ObjectDetection(
        label=label,
        confidence=confidence,
        bbox=bbox,
        depth_m=depth_m,
        valid_depth=True,
        camera_model=payload.get("camera_model"),
        image_width=image_width,
        image_height=image_height,
        camera_frame=camera_frame,
        rgb_stamp_ns=rgb_stamp_ns,
        depth_stamp_ns=depth_stamp_ns,
        frame_received_ns=frame_received_ns,
        result_stamp_ns=result_stamp_ns,
        track_id=track_id,
    )
    # camera_modelがNoneなら固定値を検証し、辞書なら内容を正規化する。
    model = detection_camera(detection)
    if detection.camera_model is not None:
        detection.camera_model = model
    return detection


def choose_tracked_candidate(
    candidates: list[ObjectDetection],
    previous: Optional[ObjectDetection],
) -> ObjectDetection:
    """同じ対象を優先し、追跡できない場合は最も近い対象を選ぶ。"""

    if not candidates:
        raise ValueError("No detection candidates")
    if previous is not None:
        tracked = [
            candidate
            for candidate in candidates
            if StableDetections.same_target(previous, candidate)
        ]
        if tracked:
            return min(tracked, key=lambda candidate: candidate.depth_m)
    return min(candidates, key=lambda candidate: candidate.depth_m)


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
        self.started = False
        self.connection_ready = False
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
                self.connection_ready = True
                return True

        command_subscribers = self.node.count_subscribers(self.command_topic)
        result_publishers = self.node.count_publishers(self.result_topic)
        self.last_failure_reason = (
            "YOLO node is not connected: "
            f"command_subscribers={command_subscribers}, "
            f"result_publishers={result_publishers}"
        )
        self.connection_ready = False
        return False

    def start(self, target_name: str) -> None:
        self.latest_message = None
        message = String()
        message.data = json.dumps(
            {"command": "start", "classes": [target_name]}
        )
        self.command_publisher.publish(message)
        self.started = True
        self.node.get_logger().info(
            f"[OBJECT GRASP] YOLO start: target={target_name}"
        )

    def stop(self) -> None:
        if not self.started:
            return
        message = String()
        message.data = json.dumps({"command": "stop"})
        self.command_publisher.publish(message)
        self.started = False
        self.node.get_logger().info("[OBJECT GRASP] YOLO stop")

    def detect(self, target_name: str, timeout: float = 8.0) -> Optional[ObjectDetection]:
        """近い対象を選び、新しい複数フレームで安定を確認して返す。"""

        target_name = target_name.strip().lower()
        self.received_result_count = 0
        self.received_empty_count = 0
        self.last_failure_reason = ""

        if (
            not self.connection_ready
            and not self.wait_until_ready(timeout=min(5.0, timeout))
        ):
            return None

        # 接続確認中に届いた古い結果を除外し、start後の結果だけを数える。
        self.received_result_count = 0
        self.received_empty_count = 0
        self.start(target_name)
        start_time = time.monotonic()
        start_stamp_ns = self.node.get_clock().now().nanoseconds
        stable = StableDetections()
        had_candidate = False

        while rclpy.ok() and time.monotonic() - start_time < timeout:
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
                self.last_failure_reason = "YOLO result must be a JSON list."
                self.node.get_logger().warn(self.last_failure_reason)
                continue

            if not detections:
                self.received_empty_count += 1
                stable.clear()
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
                stable.clear()
                continue

            parsed: list[ObjectDetection] = []
            now_ns = self.node.get_clock().now().nanoseconds
            for target in candidates:
                try:
                    parsed.append(
                        parse_yolo_detection(target, start_stamp_ns, now_ns)
                    )
                except ValueError as error:
                    self.last_failure_reason = f"Object RGBD data is invalid: {error}"
                    self.node.get_logger().warn(
                        self.last_failure_reason,
                        throttle_duration_sec=2.0,
                    )
            if not parsed:
                stable.clear()
                continue
            had_candidate = True
            self.last_failure_reason = ""
            # 同名物体が複数ある場合も、可能な限り同じ対象を追う。
            previous = (
                stable.samples[-1].detection if stable.samples else None
            )
            candidate = choose_tracked_candidate(parsed, previous)
            try:
                result = stable.add(candidate)
            except (TypeError, ValueError) as error:
                stable.clear()
                self.last_failure_reason = f"Object projection failed: {error}"
                continue
            if result is not None:
                model = detection_camera(result)
                self.node.get_logger().info(
                    f"Stable object: {result.label}, frames={len(stable.samples)}, "
                    f"depth={result.depth_m:.3f}m, intrinsics={model['source']}"
                )
                if model["source"] == "fixed":
                    self.node.get_logger().warn(
                        "CameraInfo unavailable: using fixed CameraParams intrinsics (640x480)."
                    )
                return result

        if had_candidate and not self.last_failure_reason:
            self.last_failure_reason = (
                "Object was detected but did not remain stable before timeout."
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
