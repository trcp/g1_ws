#!/usr/bin/env python3
"""物体をYOLOで認識し、把持用IKを計算して表示する実行スクリプト。"""

from __future__ import annotations

import argparse
import json
import math
import os
import time


# hri_task の bringup / YOLO コンテナと同じ ROS 設定を使う。
# 既にコンテナ内で適切な CYCLONEDDS_URI が設定されている場合は上書きしない。
os.environ["ROS_DOMAIN_ID"] = "0"
os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
_HOST_CYCLONEDDS_CONFIG = (
    "/home/roboworks/g1_ws/cyclonedds/cyclonedds.katana.xml"
)
if "CYCLONEDDS_URI" not in os.environ and os.path.isfile(_HOST_CYCLONEDDS_CONFIG):
    os.environ["CYCLONEDDS_URI"] = f"file://{_HOST_CYCLONEDDS_CONFIG}"

import rclpy
from rclpy.node import Node

from object_grasp import YoloObjectDetector, calculate_object_grasp_plan
from direct_joint_control import DirectJointController, HOME_POSE


# ============================================================
# 実験機能の有効・無効
# ============================================================

# Amazing Hand は未装着のため、現在は必ずダミー動作にする。
ENABLE_REAL_HAND_CONTROL = False

# 今回使用する実験機能。
ENABLE_HEAD_TILT_RETRY = True
ENABLE_CAMERA_ZERO_DOWN_BIAS = True
ENABLE_GRASP_POSITION_OFFSETS = True
# 接近時は肩と肘を同時に動かす。
ENABLE_STAGED_ARM_MOTION = False
# 上方退避はカメラと干渉するため、現在は使用しない。
ENABLE_SAFE_LIFT_BEFORE_HOME = False


# ============================================================
# 実験用パラメータ
# ============================================================

DETECTION_TIMEOUT_SEC = 8.0
GRASP_STRATEGY = "center"

# d455_joint は正方向が下向き。保存した初期角から絶対値で移動する。
HEAD_TILT_RETRY_COUNT = 2
HEAD_TILT_STEP_DOWN_DEG = 7.0
HEAD_TILT_SETTLE_TOLERANCE_RAD = 0.02
HEAD_TILT_SETTLE_TIMEOUT_SEC = 2.0

# サーボ角 0 rad のときも、カメラが実際には少し下を向いている補正。
# 把持計算の head_tilt は負方向が下向きなので、計算時に減算する。
CAMERA_ZERO_DOWN_BIAS_DEG = 7.0
FALLBACK_HEAD_TILT_RAD = 0.0

# ロボット座標系での把持点補正。X=前、Y=左、Z=上。
GRASP_OFFSET_X_M = 0.0
GRASP_OFFSET_Y_M = 0.0
GRASP_OFFSET_Z_M = -0.02
SAFE_LIFT_OFFSET_Z_M = 0.08

# 基本姿勢の右手首 -0.13 rad から負方向へ 90 度回した固定値。
# 正方向では手の甲が内側を向いたため、実機確認に合わせて反転する。
TOP_GRASP_WRIST_ROLL_RAD = -0.13 - math.pi / 2.0

# 右肘を畳み、手の甲を外側へ向けた上方把持用HOME。
TOP_GRASP_HOME_POSE = HOME_POSE.copy()
TOP_GRASP_HOME_POSE["right_elbow_joint"] = -1.0
TOP_GRASP_HOME_POSE["right_wrist_roll_joint"] = TOP_GRASP_WRIST_ROLL_RAD

# 動作確認用の待機時間。DirectJointController は待機中も目標姿勢を保持する。
TOP_HOME_HOLD_SEC = 3.0
SHOULDER_STAGE_HOLD_SEC = 2.0
ELBOW_STAGE_HOLD_SEC = 2.0
GRASP_HOLD_SEC = 3.0
SAFE_LIFT_HOLD_SEC = 2.0
ELBOW_FOLD_HOLD_SEC = 1.5
INITIAL_HOLD_SEC = 3.0
READY_TIMEOUT_SEC = 8.0


class OptionalHandController:
    """実ハンド未装着時は、同じ動作順序をログだけで再現する。"""

    def __init__(self, node) -> None:
        self.node = node
        self.client = None
        self.hand_command_type = None

        if not ENABLE_REAL_HAND_CONTROL:
            self.node.get_logger().info(
                "Real hand control is disabled; using dummy hand commands."
            )
            return

        try:
            from amazing_hand_interfaces.srv import HandCommand

            client = node.create_client(HandCommand, "/hand_command")
            if client.wait_for_service(timeout_sec=1.0):
                self.client = client
                self.hand_command_type = HandCommand
            else:
                self.node.get_logger().warn(
                    "/hand_command is unavailable; falling back to dummy mode."
                )
        except Exception as error:
            self.node.get_logger().warn(
                f"Hand control is unavailable; falling back to dummy mode: {error}"
            )

    def command(self, command: str, hand: str = "right") -> bool:
        if self.client is None or self.hand_command_type is None:
            print(f"[DUMMY HAND] {command} {hand} hand")
            return True

        request = self.hand_command_type.Request()
        request.command = command
        request.hand = hand
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=3.0)
        if not future.done() or future.result() is None:
            self.node.get_logger().warn(
                f"Hand command timed out: command={command}, hand={hand}"
            )
            return False
        return bool(future.result().success)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Recognize an object and print its grasp IK plan."
    )
    parser.add_argument(
        "--target",
        help="YOLOへ渡す物体名。省略すると実行時に入力する。例: bottle",
    )
    return parser.parse_args()


def print_grasp_plan(plan) -> None:
    """確認しやすい形式で認識結果とIKを表示する。"""

    print("\n========== OBJECT GRASP PLAN ==========")
    print(f"object       : {plan.object_name}")
    print(f"strategy     : {plan.grasp_strategy}")
    print(f"confidence   : {plan.confidence:.3f}")
    print(f"bbox         : {plan.bbox}")
    print(f"grasp pixel  : {plan.grasp_pixel}")
    print(f"depth        : {plan.depth_m:.3f} m")
    print(f"camera xyz   : {plan.camera_xyz}")
    print(f"robot xyz    : {plan.robot_xyz}")
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
    """関節フィードバックと腕制御subscriberを確認して初期姿勢を返す。"""

    required_joints = set(HOME_POSE)
    start_time = time.time()

    while rclpy.ok() and time.time() - start_time < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
        available_joints = set(arm.current_joints)
        command_subscribers = node.count_subscribers("/upper_joints_control")
        if required_joints.issubset(available_joints) and command_subscribers > 0:
            return {
                joint_name: arm.current_joints[joint_name]
                for joint_name in HOME_POSE
            }

    missing_joints = sorted(required_joints - set(arm.current_joints))
    command_subscribers = node.count_subscribers("/upper_joints_control")
    node.get_logger().error(
        "Arm is not ready: "
        f"missing_joints={missing_joints}, "
        f"upper_joints_control_subscribers={command_subscribers}"
    )
    return None


def wait_for_head_tilt_feedback(node, arm, timeout: float = 2.0):
    """d455_joint の実測現在値が届くまで待つ。"""

    start_time = time.time()
    while rclpy.ok() and time.time() - start_time < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
        if "d455_joint" in arm.current_joints:
            return float(arm.current_joints["d455_joint"])
    return None


def move_head_tilt_and_wait(node, arm, target_tilt: float) -> bool:
    """d455_joint を動かし、フィードバックが目標付近へ来るまで待つ。"""

    arm.send_joints({"d455_joint": target_tilt}, hold_sec=0.0)
    start_time = time.time()
    while rclpy.ok() and time.time() - start_time < HEAD_TILT_SETTLE_TIMEOUT_SEC:
        rclpy.spin_once(node, timeout_sec=0.1)
        actual = arm.current_joints.get("d455_joint")
        if actual is not None and abs(float(actual) - target_tilt) <= HEAD_TILT_SETTLE_TOLERANCE_RAD:
            return True

    node.get_logger().warn(
        "Head tilt did not settle: "
        f"target={target_tilt:.3f}, "
        f"actual={arm.current_joints.get('d455_joint')}"
    )
    return False


def calculation_head_tilt(actual_d455_tilt) -> float:
    """実機ジョイント角を、既存把持計算の負=下向き規則へ変換する。"""

    if actual_d455_tilt is None:
        head_tilt = FALLBACK_HEAD_TILT_RAD
    else:
        head_tilt = -float(actual_d455_tilt)

    if ENABLE_CAMERA_ZERO_DOWN_BIAS:
        head_tilt -= math.radians(CAMERA_ZERO_DOWN_BIAS_DEG)
    return head_tilt


def grasp_offsets(extra_z_m: float = 0.0) -> tuple[float, float, float]:
    """機能無効時は補正なし、機能有効時は設定したXYZ補正を返す。"""

    if not ENABLE_GRASP_POSITION_OFFSETS:
        return 0.0, 0.0, extra_z_m
    return (
        GRASP_OFFSET_X_M,
        GRASP_OFFSET_Y_M,
        GRASP_OFFSET_Z_M + extra_z_m,
    )


def send_grasp_posture(arm, joints: dict[str, float]) -> None:
    """設定に応じて、同時または肩→肘の順で把持姿勢へ移動する。"""

    if not ENABLE_STAGED_ARM_MOTION:
        arm.send_joints(joints, hold_sec=GRASP_HOLD_SEC)
        return

    shoulder_stage = {
        name: value
        for name, value in joints.items()
        if name != "right_elbow_joint"
    }
    arm.send_joints(shoulder_stage, hold_sec=SHOULDER_STAGE_HOLD_SEC)
    arm.send_joints(
        {"right_elbow_joint": joints["right_elbow_joint"]},
        hold_sec=ELBOW_STAGE_HOLD_SEC,
    )


def main() -> None:
    args = parse_args()
    target_name = args.target
    if not target_name:
        target_name = input("把持する物体名を英語で入力してください: ").strip()

    if not target_name:
        print("物体名が空のため終了します。")
        return

    rclpy.init()
    node = Node("run_object_grasp")
    detector = YoloObjectDetector(node)
    arm = DirectJointController(node)
    hand = OptionalHandController(node)
    initial_joints = None
    initial_head_tilt = None
    grasp_plan = None
    lift_plan = None
    motion_started = False
    returned_to_top_home = False

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

        initial_head_tilt = wait_for_head_tilt_feedback(node, arm)
        if initial_head_tilt is None:
            node.get_logger().warn(
                "d455_joint feedback is unavailable. "
                "Head tilt retry will be skipped and fallback tilt will be used."
            )

        print("Moving from the initial posture to TOP GRASP HOME...")
        motion_started = True
        arm.send_joints(TOP_GRASP_HOME_POSE, hold_sec=TOP_HOME_HOLD_SEC)

        # 物体上で開くと指が干渉するため、HOME到達直後に開く。
        if not hand.command("open", "right"):
            node.get_logger().warn("Failed to open the right hand.")

        detection = None
        actual_detection_tilt = initial_head_tilt
        retry_count = (
            HEAD_TILT_RETRY_COUNT
            if ENABLE_HEAD_TILT_RETRY and initial_head_tilt is not None
            else 0
        )

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
                if not move_head_tilt_and_wait(node, arm, target_head_tilt):
                    break
                actual_detection_tilt = float(
                    arm.current_joints.get("d455_joint", target_head_tilt)
                )

            effective_head_tilt = calculation_head_tilt(actual_detection_tilt)
            print(
                f"Searching for: {target_name} "
                f"(d455={actual_detection_tilt}, "
                f"calculation tilt={effective_head_tilt:.3f} rad)"
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
            print(detector.last_failure_reason or f"Object was not detected: {target_name}")
            return

        offset_x, offset_y, offset_z = grasp_offsets()
        grasp_plan = calculate_object_grasp_plan(
            detection,
            head_tilt=calculation_head_tilt(actual_detection_tilt),
            grasp_strategy=GRASP_STRATEGY,
            offset_x_m=offset_x,
            offset_y_m=offset_y,
            offset_z_m=offset_z,
            right_wrist_roll=TOP_GRASP_WRIST_ROLL_RAD,
        )
        print_grasp_plan(grasp_plan)

        if not grasp_plan.reachable:
            print(
                "警告: 対象は計算上の到達範囲外です。"
                "既存のbag_graspと同様に、到達限界へ丸めたIKを送信します。"
            )

        print("Moving shoulder and elbow together to the grasp posture...")
        send_grasp_posture(arm, grasp_plan.joints)

        if not hand.command("close", "right"):
            node.get_logger().warn("Failed to close the right hand.")

        if ENABLE_SAFE_LIFT_BEFORE_HOME:
            lift_x, lift_y, lift_z = grasp_offsets(SAFE_LIFT_OFFSET_Z_M)
            lift_plan = calculate_object_grasp_plan(
                detection,
                head_tilt=calculation_head_tilt(actual_detection_tilt),
                grasp_strategy=GRASP_STRATEGY,
                offset_x_m=lift_x,
                offset_y_m=lift_y,
                offset_z_m=lift_z,
                right_wrist_roll=TOP_GRASP_WRIST_ROLL_RAD,
            )
            print("Lifting the hand before folding the arm...")
            arm.send_joints(lift_plan.joints, hold_sec=SAFE_LIFT_HOLD_SEC)

        # カメラ側へ腕を上げず、まず肘を畳んで手を体側へ戻す。
        print("Folding the elbow before returning the shoulder to HOME...")
        arm.send_joints(
            {"right_elbow_joint": TOP_GRASP_HOME_POSE["right_elbow_joint"]},
            hold_sec=ELBOW_FOLD_HOLD_SEC,
        )
        arm.send_joints(TOP_GRASP_HOME_POSE, hold_sec=TOP_HOME_HOLD_SEC)
        returned_to_top_home = True
        print("Grasp posture check finished.")
    finally:
        try:
            detector.stop()
        except Exception as error:
            node.get_logger().warn(f"Failed to stop YOLO: {error}")

        if motion_started and not returned_to_top_home:
            print("Folding the elbow, then moving the shoulder back to TOP GRASP HOME...")
            try:
                arm.send_joints(
                    {
                        "right_elbow_joint":
                            TOP_GRASP_HOME_POSE["right_elbow_joint"]
                    },
                    hold_sec=ELBOW_FOLD_HOLD_SEC,
                )
                arm.send_joints(
                    TOP_GRASP_HOME_POSE,
                    hold_sec=TOP_HOME_HOLD_SEC,
                )
            except Exception as error:
                node.get_logger().error(
                    f"Failed to return to TOP GRASP HOME: {error}"
                )

        if initial_head_tilt is not None:
            print("Restoring the initial head tilt...")
            try:
                move_head_tilt_and_wait(node, arm, initial_head_tilt)
            except Exception as error:
                node.get_logger().error(
                    f"Failed to restore the initial head tilt: {error}"
                )

        if motion_started and initial_joints is not None:
            print("Moving from TOP GRASP HOME back to the initial posture...")
            try:
                arm.send_joints(initial_joints, hold_sec=INITIAL_HOLD_SEC)
            except Exception as error:
                node.get_logger().error(
                    f"Failed to return to the initial posture: {error}"
                )

        arm.active = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
