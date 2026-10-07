#!/usr/bin/env python3
"""物体をYOLOで認識し、簡易IKの関節目標をMoveItで実行する。"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import time


# hri_task の bringup / YOLO コンテナと同じ ROS 設定を使う。
# 既にコンテナ内で適切な CYCLONEDDS_URI が設定されている場合は上書きしない。
os.environ.setdefault("ROS_DOMAIN_ID", "0")
os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
_HOST_CYCLONEDDS_CONFIG = (
    "/home/roboworks/g1_ws/cyclonedds/cyclonedds.katana.xml"
)
if "CYCLONEDDS_URI" not in os.environ and os.path.isfile(_HOST_CYCLONEDDS_CONFIG):
    os.environ["CYCLONEDDS_URI"] = f"file://{_HOST_CYCLONEDDS_CONFIG}"

import rclpy
from rclpy.node import Node

from erasers_g1_api.robot_control import ArmControl, G1Control
from object_grasp import (
    CameraTransform,
    YoloObjectDetector,
    calculate_object_grasp_plan,
    joint_limit_violations,
    joint_margin_violations,
)


# ============================================================
# 把持動作の調整値
# ============================================================

DETECTION_TIMEOUT_SEC = 8.0
GRASP_STRATEGY = "center"

# d455_joint は正方向が下向き。保存した初期角から絶対値で移動する。
HEAD_TILT_RETRY_COUNT = 2
HEAD_TILT_STEP_DOWN_DEG = 2 #(7)
HEAD_TILT_SETTLE_TOLERANCE_RAD = 0.02
HEAD_TILT_SETTLE_TIMEOUT_SEC = 2.0

# ロボット座標系での把持点補正。X=前、Y=左、Z=上。
GRASP_OFFSET_X_M = 0.0
GRASP_OFFSET_Y_M = 0.0
GRASP_OFFSET_Z_M = 0.03

# 正面の物体へ横から接近するため、把持点の10cm手前から開始する。
# 把持後は机から離すため、従来どおり上方向へリフトする。
SIDE_APPROACH_CLEARANCE_M = 0.10
POST_GRASP_LIFT_M = 0.10

# 直接関節制御で使用していた自然な腕下げ姿勢。MoveItでも同じ関節基準を使う。
WALK_POSE = {
    "left_shoulder_pitch_joint": 0.29,
    "left_shoulder_roll_joint": 0.23,
    "left_shoulder_yaw_joint": -0.02,
    "left_elbow_joint": 0.97,
    "left_wrist_roll_joint": 0.08,
    "right_shoulder_pitch_joint": 0.29,
    "right_shoulder_roll_joint": -0.23,
    "right_shoulder_yaw_joint": 0.03,
    "right_elbow_joint": 0.97,
    "right_wrist_roll_joint": -0.13,
    "waist_yaw_joint": 0.0,
}

# 実機で親指が上になることを確認したWALK姿勢の手首角を使う。
SIDE_GRASP_WRIST_ROLL_RAD = {
    "left": WALK_POSE["left_wrist_roll_joint"],
    "right": WALK_POSE["right_wrist_roll_joint"],
}
TOP_GRASP_ELBOW_RAD = -0.90
ARM_NAMES = ("right", "left")

READY_TIMEOUT_SEC = 8.0
# MoveItモデルの速度・加速度上限に掛ける倍率。0より大きく1以下。
ARM_VELOCITY_SCALE = 0.5
ARM_ACCELERATION_SCALE = 0.5
MOTION_PAUSE_SEC = 1.0
ROBOT_BASE_FRAME = "base_link"
CAMERA_TF_TIMEOUT_SEC = 3.0

MOVEIT_JOINTS = frozenset(WALK_POSE)
OBJECT_LIST = ("apple", "bottle", "orange", "banana")
VOICE_TARGET_ATTEMPTS = 3


class GraspArmControl(ArmControl):
    """この把持スクリプトのMoveIt要求だけに減速設定を適用する。"""

    def _send_move_group_goal(self, goal_msg, wait):
        for scale in (ARM_VELOCITY_SCALE, ARM_ACCELERATION_SCALE):
            if not math.isfinite(scale) or not 0.0 < scale <= 1.0:
                raise ValueError("Arm speed scales must be finite and in (0, 1].")
        goal_msg.request.max_velocity_scaling_factor = ARM_VELOCITY_SCALE
        goal_msg.request.max_acceleration_scaling_factor = ARM_ACCELERATION_SCALE
        return super()._send_move_group_goal(goal_msg, wait)

def pause_after_motion(node) -> bool:
    """動作間に待機し、その間も関節フィードバックを受信する。"""

    node.get_logger().info(f"Pausing for {MOTION_PAUSE_SEC:.1f} seconds after motion.")
    deadline = time.monotonic() + MOTION_PAUSE_SEC
    while rclpy.ok():
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            return True
        rclpy.spin_once(node, timeout_sec=min(0.1, remaining))
    return False


class OptionalHandController:
    """Amazing Handの有無を吸収する。既定は実機を開閉するreal。"""

    def __init__(self, node, arm: ArmControl, mode: str = "real") -> None:
        self.node = node
        self.arm = arm
        self.mode = mode
        self.node.get_logger().info(f"Amazing Hand mode: {mode}")

    def command(self, command: str, hand: str = "right") -> bool:
        if self.mode == "disabled":
            return True
        if self.mode == "dummy":
            print(f"[DUMMY HAND] {command} {hand} hand")
            return pause_after_motion(self.node)

        try:
            success = bool(self.arm.hand_control(command=command, hand=hand))
        except Exception as error:
            self.node.get_logger().warn(
                f"Hand command failed: command={command}, hand={hand}: {error}"
            )
            return False
        return success and pause_after_motion(self.node)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Recognize an object and print its grasp IK plan."
    )
    parser.add_argument(
        "--target",
        help="YOLOへ渡す物体名。省略すると音声認識で尋ねる。例: bottle",
    )
    parser.add_argument(
        "--hand-mode",
        choices=("dummy", "real", "disabled"),
        default="real",
        help="Amazing Hand: dummy=ログのみ、real=実機（既定）、disabled=処理なし",
    )
    parser.add_argument(
        "--plan-only",
        action="store_true",
        help="YOLO検出とIK計算だけを行い、腕・頭・ハンドを動かさない",
    )
    return parser.parse_args()


def recognize_target_name(node) -> str | None:
    """HRI Taskと同じTTS/SpeechToTextで、候補から1つを聞き取る。"""

    # --target指定時はWhisper/SMACHなどの音声依存を読み込まない。
    import smach
    from erasers_g1_api.tts import TTS
    from erasers_g1_api.state_skills.recongnition import SpeechToText
    from g1_srvs.srv import AudioClient
    from std_srvs.srv import SetBool

    for service_type, service_name in (
        (AudioClient, "/play_audio"), (SetBool, "/mic_rec")
    ):
        client = node.create_client(service_type, service_name)
        try:
            if not client.wait_for_service(timeout_sec=READY_TIMEOUT_SEC):
                node.get_logger().error(f"Voice service unavailable: {service_name}")
                return None
        finally:
            node.destroy_client(client)

    tts = TTS(node)
    speech = SpeechToText(
        node=node,
        tts=tts,
        start_msg=(
            "Please say the name of the object you would like me to pick up "
            "after the beep. You can choose "
            + ", ".join(OBJECT_LIST) + "."
        ),
        success_msg="Please wait.",
        timeout_msg="Please say apple, bottle, orange, or banana after the beep.",
        lang="en",
        max_challenge=VOICE_TARGET_ATTEMPTS,
    )
    userdata = smach.UserData()
    userdata.num_challenge = 0
    # API側の綴りに合わせる。
    userdata.success_keywards = list(OBJECT_LIST)
    for _ in range(VOICE_TARGET_ATTEMPTS):
        if not rclpy.ok():
            return None
        userdata.stt_text = ""
        outcome = speech.execute(userdata)
        if outcome == "failure":
            return None
        if outcome != "success":
            continue
        matches = [
            name for name in OBJECT_LIST
            if re.search(r"\b" + re.escape(name) + r"s?\b", userdata.stt_text.lower())
        ]
        if len(matches) == 1:
            target = matches[0]
            node.get_logger().info(f"Voice target: {target}")
            tts.say(f"I will grasp the {target}.")
            return target
        node.get_logger().warn(f"Expected one object, heard: {userdata.stt_text}")
        # 次の試行で候補一覧を含む質問を再度読み上げる。
        userdata.num_challenge = 0

    return None


def print_grasp_plan(plan) -> None:
    """確認しやすい形式で認識結果とIKを表示する。"""

    print("\n========== OBJECT GRASP PLAN ==========")
    print(f"object       : {plan.object_name}")
    print(f"arm          : {plan.arm}")
    print(f"strategy     : {plan.grasp_strategy}")
    print(f"confidence   : {plan.confidence:.3f}")
    print(f"bbox         : {plan.bbox}")
    print(f"grasp pixel  : {plan.grasp_pixel}")
    print(f"depth        : {plan.depth_m:.3f} m")
    print(f"camera xyz   : {plan.camera_xyz}")
    print(f"robot xyz    : {plan.robot_xyz}")
    print(f"coordinates  : {plan.coordinate_source}")
    print(f"IK source    : {plan.kinematics_source}")
    print(f"shoulder @0  : {plan.shoulder_distance_at_zero_m:.3f} m")
    print(f"arm distance : {plan.arm_distance_m:.3f} m")
    print(f"reachable    : {plan.reachable}")
    print(f"reason       : {plan.reason or '-'}")
    print("IK joints:")
    for joint_name, angle in plan.joints.items():
        print(f"  {joint_name}: {angle:.6f}")

    print("\nJSON:")
    print(json.dumps(plan.to_dict(), indent=2, ensure_ascii=False))
    print("=======================================\n")


def wait_for_arm_ready(node, arm, timeout: float = READY_TIMEOUT_SEC):
    """MoveIt対象関節のフィードバックを待ち、初期姿勢を返す。"""

    required_joints = set(MOVEIT_JOINTS)
    start_time = time.time()

    while rclpy.ok() and time.time() - start_time < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
        current_joints = arm.get_current_joints_pose()
        if required_joints.issubset(current_joints):
            return {
                joint_name: float(current_joints[joint_name])
                for joint_name in MOVEIT_JOINTS
            }

    current_joints = arm.get_current_joints_pose()
    missing_joints = sorted(required_joints - set(current_joints))
    node.get_logger().error(
        f"Arm is not ready: missing_joints={missing_joints}"
    )
    return None


def wait_for_head_tilt_feedback(node, arm, timeout: float = 2.0):
    """d455_joint の実測現在値が届くまで待つ。"""

    start_time = time.time()
    while rclpy.ok() and time.time() - start_time < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
        current_joints = arm.get_current_joints_pose()
        if "d455_joint" in current_joints:
            return float(current_joints["d455_joint"])
    return None


def move_head_tilt_and_wait(node, robot, arm, target_tilt: float) -> bool:
    """d455_joint を動かし、フィードバックが目標付近へ来るまで待つ。"""

    # G1Controlのtiltは負方向が下、d455_jointの実測値は正方向が下。
    if not robot.move_head(tilt=-target_tilt, pan=0.0):
        node.get_logger().warn(
            f"Head tilt command failed: target={target_tilt:.3f}"
        )
        return False

    start_time = time.time()
    actual = None
    while rclpy.ok() and time.time() - start_time < HEAD_TILT_SETTLE_TIMEOUT_SEC:
        rclpy.spin_once(node, timeout_sec=0.1)
        actual = arm.get_current_joints_pose().get("d455_joint")
        settled = (
            actual is not None
            and abs(float(actual) - target_tilt)
            <= HEAD_TILT_SETTLE_TOLERANCE_RAD
        )
        if settled:
            return pause_after_motion(node)

    node.get_logger().warn(
        "Head tilt did not settle: "
        f"target={target_tilt:.3f}, "
        f"actual={actual}"
    )
    return False


def create_tf_buffer(node):
    """plan-onlyでもTFを受信できるバッファとリスナーを作る。"""

    from tf2_ros import Buffer, TransformListener

    buffer = Buffer()
    listener = TransformListener(buffer, node)
    return buffer, listener


def wait_for_camera_transform(node, tf_buffer, camera_frame: str):
    """最新のカメラ光学座標系をbase_linkへ変換するTFを待つ。

    このTFにはd455_jointの実測角、URDFの固定取付角0.1 rad、RealSenseの
    光学座標軸変換が含まれる。別のチルト補正は加えない。
    """

    if not camera_frame:
        node.get_logger().error("Detection did not provide a camera frame.")
        return None

    from rclpy.time import Time

    deadline = time.monotonic() + CAMERA_TF_TIMEOUT_SEC
    last_error = None
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            message = tf_buffer.lookup_transform(
                ROBOT_BASE_FRAME,
                camera_frame,
                Time(),
            )
            translation = message.transform.translation
            rotation = message.transform.rotation
            transform = CameraTransform(
                target_frame=ROBOT_BASE_FRAME,
                source_frame=camera_frame,
                translation=(
                    float(translation.x),
                    float(translation.y),
                    float(translation.z),
                ),
                rotation_xyzw=(
                    float(rotation.x),
                    float(rotation.y),
                    float(rotation.z),
                    float(rotation.w),
                ),
            )
            node.get_logger().info(
                "Camera TF: "
                f"{ROBOT_BASE_FRAME}<-{camera_frame}, "
                f"translation={transform.translation}, "
                f"rotation_xyzw={transform.rotation_xyzw}"
            )
            return transform
        except Exception as error:
            last_error = error

    node.get_logger().error(
        "Camera TF is unavailable; object-directed motion is disabled. "
        f"Required transform: {ROBOT_BASE_FRAME}<-{camera_frame}. "
        f"Last error: {last_error}"
    )
    return None


def build_grasp_plan(
    detection,
    camera_transform,
    arm_name: str,
    extra_x_m: float = 0.0,
    extra_z_m: float = 0.0,
):
    """共通の補正値へX/Zの段階別補正を加え、把持計画を作る。"""

    return calculate_object_grasp_plan(
        detection,
        arm=arm_name,
        grasp_strategy=GRASP_STRATEGY,
        offset_x_m=GRASP_OFFSET_X_M + extra_x_m,
        offset_y_m=GRASP_OFFSET_Y_M,
        offset_z_m=GRASP_OFFSET_Z_M + extra_z_m,
        wrist_roll=SIDE_GRASP_WRIST_ROLL_RAD[arm_name],
        camera_transform=camera_transform,
    )


def build_grasp_sequence(detection, camera_transform, arm_name: str) -> dict:
    """手前待機、横把持、把持後リフトの3つの目標を作る。"""

    return {
        "pre-grasp": build_grasp_plan(
            detection,
            camera_transform,
            arm_name,
            extra_x_m=-SIDE_APPROACH_CLEARANCE_M,
        ),
        "grasp": build_grasp_plan(
            detection,
            camera_transform,
            arm_name,
        ),
        "lift": build_grasp_plan(
            detection,
            camera_transform,
            arm_name,
            extra_z_m=POST_GRASP_LIFT_M,
        ),
    }


def build_arm_candidates(detection, camera_transform) -> dict:
    """左右それぞれの把持シーケンスを同じ検出結果から作る。"""

    return {
        arm_name: build_grasp_sequence(
            detection,
            camera_transform,
            arm_name,
        )
        for arm_name in ARM_NAMES
    }


def print_grasp_sequence(plans: dict, arm_name: str | None = None) -> None:
    """各段階の目標座標と関節角を、実行順に表示する。"""

    if arm_name:
        print(f"\n######## {arm_name.upper()} ARM CANDIDATE ########")
    for stage, plan in plans.items():
        print(f"\n--- {stage.upper()} ---")
        print_grasp_plan(plan)


def sequence_distance_score(plans: dict) -> tuple[float, float, float]:
    """候補の距離をログ表示するための診断値を返す。"""

    return (
        max(plan.shoulder_distance_at_zero_m for plan in plans.values()),
        max(plan.arm_distance_m for plan in plans.values()),
        abs(plans["grasp"].joints["waist_yaw_joint"]),
    )


def select_grasp_arm_candidate(
    node,
    candidates: dict,
) -> tuple[str | None, dict | None]:
    """簡易IKを左右とも検査し、右腕優先で実行候補を返す。"""

    for arm_name, plans in candidates.items():
        print_grasp_sequence(plans, arm_name)

    executable = {}
    for arm_name in ARM_NAMES:  # ARM_NAMES = ("right", "left")
        plans = candidates.get(arm_name)
        if plans is None:
            continue
        required_joints = selected_arm_joint_names(arm_name)
        errors = []
        for stage, plan in plans.items():
            missing = sorted(required_joints - set(plan.joints))
            if missing:
                errors.append(f"{stage}: missing joints {missing}")
                continue
            joints = {
                name: float(plan.joints[name]) for name in required_joints
            }
            if not all(math.isfinite(value) for value in joints.values()):
                errors.append(f"{stage}: non-finite joint value")
                continue
            hard_limit_errors = joint_limit_violations(joints)
            if hard_limit_errors:
                errors.append(
                    f"{stage}: " + "; ".join(hard_limit_errors)
                )

        if errors:
            node.get_logger().warn(
                f"{arm_name} arm simplified joint targets cannot be sent: "
                + " | ".join(errors)
            )
            continue

        executable[arm_name] = plans

    # 基本は右腕。右の3姿勢が簡易モデルで到達可能なら左の距離に関係なく右を使う。
    # 右が到達不可で左が到達可能な場合だけ左へ切り替える。
    selection_order = [
        arm_name
        for arm_name in ARM_NAMES
        if arm_name in executable
        and all(plan.reachable for plan in executable[arm_name].values())
    ]
    if not selection_order:
        # 旧git版と同様、両腕とも距離近似だけが範囲外なら、境界へ丸めた右腕の
        # 関節値を優先する。URDFのハードリミット外は上の検査で除外済み。
        selection_order = [
            arm_name for arm_name in ARM_NAMES if arm_name in executable
        ]

    if selection_order:
        arm_name = selection_order[0]
        plans = executable[arm_name]
        if arm_name == "left":
            node.get_logger().warn(
                "The right-arm simplified sequence is not executable; "
                "using the left arm as fallback."
            )

        shoulder_distance, max_distance, waist_rotation = sequence_distance_score(plans)
        node.get_logger().info(
            f"Selected {arm_name} arm from simplified joint targets: "
            f"shoulder_distance={shoulder_distance:.3f} m, "
            f"planar_distance={max_distance:.3f} m, "
            f"waist={waist_rotation:.3f} rad."
        )
        unreachable = [
            stage for stage, plan in plans.items() if not plan.reachable
        ]
        if unreachable:
            node.get_logger().warn(
                "Using legacy clamped simplified IK for stages: "
                + ", ".join(unreachable)
            )
        return arm_name, plans

    node.get_logger().error(
        "Neither arm has a finite simplified joint sequence inside URDF limits."
    )
    return None, None


def selected_arm_joint_names(arm_name: str) -> frozenset[str]:
    """左右別MoveItチェーンで必須となる6関節を返す。"""

    if arm_name not in ARM_NAMES:
        raise ValueError(f"Unknown arm: {arm_name}")
    return frozenset({
        "waist_yaw_joint",
        f"{arm_name}_shoulder_pitch_joint",
        f"{arm_name}_shoulder_roll_joint",
        f"{arm_name}_shoulder_yaw_joint",
        f"{arm_name}_elbow_joint",
        f"{arm_name}_wrist_roll_joint",
    })


def move_arm_joints(node, arm, joints: dict[str, float]) -> bool:
    """計算済み関節角を1つのMoveItゴールとして送る。"""

    unsupported = sorted(set(joints) - MOVEIT_JOINTS)
    if unsupported:
        node.get_logger().warn(
            f"Ignoring joints outside MoveIt upper_body: {unsupported}"
        )

    targets = {
        name: float(value)
        for name, value in joints.items()
        if name in MOVEIT_JOINTS
    }
    if not targets:
        node.get_logger().error("No MoveIt joint target was provided.")
        return False

    success = bool(
        arm.joint_control(
            planning_group="upper_body",
            wait=True,
            **targets,
        )
    )
    return success and pause_after_motion(node)


def top_grasp_home_pose(initial_joints: dict, arm_name: str) -> dict:
    """非選択腕を初期角のまま保ち、選択腕だけを把持HOMEへ畳む。"""

    if arm_name not in ARM_NAMES:
        raise ValueError(f"Unknown arm: {arm_name}")
    pose = {
        joint_name: float(initial_joints[joint_name])
        for joint_name in MOVEIT_JOINTS
    }
    pose["waist_yaw_joint"] = 0.0
    pose[f"{arm_name}_elbow_joint"] = TOP_GRASP_ELBOW_RAD
    pose[f"{arm_name}_wrist_roll_joint"] = SIDE_GRASP_WRIST_ROLL_RAD[arm_name]
    return pose


def return_to_top_home(node, arm, home_pose: dict, arm_name: str) -> bool:
    """選択腕の肩と肘を1つのMoveItゴールで把持HOMEへ戻す。"""

    current_joints = arm.get_current_joints_pose()
    if not MOVEIT_JOINTS.issubset(current_joints):
        node.get_logger().error("Cannot return home: joint feedback is incomplete.")
        return False

    print(
        f"Returning {arm_name} shoulder and elbow together "
        "to TOP GRASP HOME..."
    )
    return move_arm_joints(node, arm, home_pose)


def run_plan_only(
    node,
    detector,
    tf_buffer,
    target_name: str,
) -> bool:
    """ロボット制御を初期化せず、認識と座標・IK計算だけを行う。"""

    print("PLAN-ONLY mode: no arm, head, or hand command will be sent.")
    print("Checking YOLO connection...")
    if not detector.wait_until_ready(timeout=READY_TIMEOUT_SEC):
        print(detector.last_failure_reason)
        return False

    print(f"Searching for: {target_name} (coordinates use TF)")
    detection = detector.detect(target_name, timeout=DETECTION_TIMEOUT_SEC)
    if detection is None:
        print(
            detector.last_failure_reason
            or f"Object was not detected: {target_name}"
        )
        return False

    camera_transform = wait_for_camera_transform(
        node, tf_buffer, detection.camera_frame
    )
    if camera_transform is None:
        return False

    candidates = build_arm_candidates(
        detection,
        camera_transform,
    )
    selected_arm, _ = select_grasp_arm_candidate(node, candidates)
    if selected_arm is not None:
        node.get_logger().info(
            f"PLAN-ONLY selected {selected_arm} arm. Normal mode sends these "
            "simplified joint targets through MoveIt joint planning."
        )
    return selected_arm is not None


def stop_detector_safely(node, detector) -> None:
    """この処理から開始したYOLOだけを停止し、終了処理を継続する。"""

    try:
        detector.stop()
    except Exception as error:
        node.get_logger().warn(f"Failed to stop YOLO: {error}")


def main() -> None:
    args = parse_args()
    target_name = args.target

    rclpy.init()
    node = Node("run_object_grasp")
    if not target_name:
        try:
            target_name = recognize_target_name(node)
        except Exception as error:
            node.get_logger().error(f"Voice target recognition failed: {error}")
        finally:
            if not target_name:
                node.get_logger().error(
                    "No voice target selected. No motion command was sent. "
                    "Check HRI speech services or use --target orange."
                )
                node.destroy_node()
                rclpy.shutdown()
        if not target_name:
            return

    detector = YoloObjectDetector(node)
    if args.plan_only:
        tf_buffer, tf_listener = create_tf_buffer(node)
        try:
            run_plan_only(
                node,
                detector,
                tf_buffer,
                target_name,
            )
        finally:
            # リスナーを関数終了まで保持する。
            del tf_listener
            stop_detector_safely(node, detector)
            node.destroy_node()
            rclpy.shutdown()
        return

    arm = GraspArmControl(node)
    robot = G1Control(node)
    hand = OptionalHandController(node, arm, mode=args.hand_mode)
    initial_joints = None
    initial_head_tilt = None
    grasp_plans = None
    selected_arm = None
    selected_home_pose = None
    motion_started = False
    grasp_attempted = False
    lift_completed = False
    returned_to_top_home = False
    home_return_attempted = False
    upper_body_control_enabled = False
    hand_closed = False

    try:
        print(
            "ROS connection: "
            f"domain={os.environ.get('ROS_DOMAIN_ID')}, "
            f"rmw={os.environ.get('RMW_IMPLEMENTATION')}"
        )

        print("Checking YOLO connection...")
        if not detector.wait_until_ready(timeout=READY_TIMEOUT_SEC):
            print(detector.last_failure_reason)
            return

        print("Checking joint feedback and arm controller...")
        initial_joints = wait_for_arm_ready(node, arm)
        if initial_joints is None:
            print("Arm startup check failed. No motion command was sent.")
            return

        print("Enabling MoveIt upper-body control...")
        if not arm.enable_upper_body_control(True):
            node.get_logger().error("Failed to enable upper-body control.")
            return
        upper_body_control_enabled = True

        initial_head_tilt = wait_for_head_tilt_feedback(node, arm)
        if initial_head_tilt is None:
            node.get_logger().warn(
                "d455_joint feedback is unavailable. "
                "Head tilt retry will be skipped."
            )

        # カメラ下降・認識中は腕を動かさず、指の干渉を避けるため両手を閉じる。
        print("Closing both hands before camera search...")
        hand_closed = hand.command("close", "both")
        if not hand_closed:
            node.get_logger().error(
                "Both hands must be closed before camera search."
            )
            return

        detection = None
        actual_detection_tilt = initial_head_tilt
        retry_count = HEAD_TILT_RETRY_COUNT if initial_head_tilt is not None else 0

        for attempt in range(retry_count + 1):
            if attempt > 0:
                target_head_tilt = initial_head_tilt + math.radians(
                    HEAD_TILT_STEP_DOWN_DEG * attempt
                )
                print(
                    f"Object not found. Tilting the camera down "
                    f"(retry {attempt}/{retry_count}, "
                    f"target d455={target_head_tilt:.3f} rad)..."
                )
                if not move_head_tilt_and_wait(
                    node, robot, arm, target_head_tilt
                ):
                    break
                actual_detection_tilt = float(
                    arm.get_current_joints_pose().get(
                        "d455_joint", target_head_tilt
                    )
                )

            print(
                f"Searching for: {target_name} "
                f"(d455={actual_detection_tilt}; coordinates use TF)"
            )
            detection = detector.detect(
                target_name,
                timeout=DETECTION_TIMEOUT_SEC,
            )
            if detection is not None:
                break

            # 接続・画像・深度異常では頭を動かさず、純粋な未検出だけ再探索する。
            if not detector.last_failure_reason.startswith(
                "YOLO results were received, but"
            ):
                break

        if detection is None:
            print(
                detector.last_failure_reason
                or f"Object was not detected: {target_name}"
            )
            return

        camera_transform = wait_for_camera_transform(
            node,
            arm.tf_buffer,
            detection.camera_frame,
        )
        if camera_transform is None:
            return

        candidates = build_arm_candidates(
            detection,
            camera_transform,
        )
        selected_arm, grasp_plans = select_grasp_arm_candidate(node, candidates)
        if selected_arm is None:
            return

        selected_home_pose = top_grasp_home_pose(initial_joints, selected_arm)
        home_violations = joint_margin_violations(selected_home_pose)
        if home_violations:
            node.get_logger().error(
                f"{selected_arm}-arm HOME violates joint safety margin: "
                + "; ".join(home_violations)
            )
            return
        print(
            f"Moving the selected {selected_arm} arm "
            "from the initial posture to TOP GRASP HOME..."
        )
        motion_started = True
        if not move_arm_joints(node, arm, selected_home_pose):
            node.get_logger().error(
                f"Failed to move the {selected_arm} arm to TOP GRASP HOME."
            )
            return

        # 左右の簡易関節値計算が終わるまでは閉じたままにし、接近直前だけ開く。
        hand_closed = False
        if not hand.command("open", selected_arm):
            node.get_logger().error(
                f"Failed to open the {selected_arm} hand before approach."
            )
            return

        print(f"Moving the {selected_arm} hand in front of the object...")
        if not move_arm_joints(node, arm, grasp_plans["pre-grasp"].joints):
            node.get_logger().error("Failed to execute the pre-grasp posture.")
            return

        print(f"Advancing the {selected_arm} hand horizontally to the grasp posture...")
        # Y/Zを保ち、物体手前からX正方向へ横移動する。
        if not move_arm_joints(node, arm, grasp_plans["grasp"].joints):
            node.get_logger().error("Failed to execute the grasp posture.")
            return

        # 応答失敗時も物体を保持している可能性があるため、以降はリフトを必須とする。
        grasp_attempted = True
        hand_closed = hand.command("close", selected_arm)
        if not hand_closed:
            node.get_logger().warn(
                f"Failed to confirm that the {selected_arm} hand closed."
            )

        print("Lifting the hand above the object before returning HOME...")
        if not move_arm_joints(node, arm, grasp_plans["lift"].joints):
            node.get_logger().error(
                "Lift failed after the grasp attempt. Automatic HOME return "
                "is disabled to avoid dragging the hand across the table."
            )
            return
        lift_completed = True

        home_return_attempted = True
        returned_to_top_home = return_to_top_home(
            node,
            arm,
            selected_home_pose,
            selected_arm,
        )
        if not returned_to_top_home:
            node.get_logger().error("Failed to return to TOP GRASP HOME.")
            return
        print("Grasp posture check finished.")
    finally:
        stop_detector_safely(node, detector)

        safe_to_return_home = not grasp_attempted or lift_completed
        if (
            motion_started
            and not home_return_attempted
            and safe_to_return_home
            and selected_home_pose is not None
            and selected_arm is not None
        ):
            try:
                home_return_attempted = True
                returned_to_top_home = return_to_top_home(
                    node,
                    arm,
                    selected_home_pose,
                    selected_arm,
                )
            except Exception as error:
                node.get_logger().error(
                    f"Failed to return to TOP GRASP HOME: {error}"
                )
        elif motion_started and not home_return_attempted:
            node.get_logger().error(
                "Automatic HOME return skipped because the grasp was attempted "
                "but the lift was not confirmed. Operator recovery is required."
            )

        safe_to_restore = not motion_started or returned_to_top_home

        # 腕を初期姿勢へ下げる前に、選択した手の閉状態を必須にする。
        if (
            selected_arm is not None
            and motion_started
            and returned_to_top_home
            and not hand_closed
        ):
            print(f"Closing the {selected_arm} hand before lowering the arm...")
            try:
                hand_closed = hand.command("close", selected_arm)
            except Exception as error:
                node.get_logger().error(
                    f"Failed to close the {selected_arm} hand: {error}"
                )
                hand_closed = False
            if not hand_closed:
                node.get_logger().error(
                    "Initial-posture return is disabled because hand closure "
                    "was not confirmed. Operator recovery is required."
                )

        if initial_head_tilt is not None and safe_to_restore:
            print("Restoring the initial head tilt...")
            try:
                move_head_tilt_and_wait(
                    node, robot, arm, initial_head_tilt
                )
            except Exception as error:
                node.get_logger().error(
                    f"Failed to restore the initial head tilt: {error}"
                )

        if (
            motion_started
            and returned_to_top_home
            and hand_closed
            and initial_joints is not None
        ):
            print("Moving from TOP GRASP HOME back to the initial posture...")
            try:
                move_arm_joints(node, arm, initial_joints)
            except Exception as error:
                node.get_logger().error(
                    f"Failed to return to the initial posture: {error}"
                )

        safe_to_release_control = safe_to_restore and (
            not motion_started or hand_closed
        )
        if upper_body_control_enabled and safe_to_release_control:
            try:
                arm.enable_upper_body_control(False)
            except Exception as error:
                node.get_logger().error(
                    f"Failed to disable upper-body control: {error}"
                )
        elif upper_body_control_enabled:
            node.get_logger().error(
                "Upper-body control remains enabled because a safe recovery "
                "posture was not confirmed. Operator recovery is required."
            )

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
