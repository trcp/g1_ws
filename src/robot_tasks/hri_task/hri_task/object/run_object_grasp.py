#!/usr/bin/env python3
"""物体をYOLOで認識し、把持用IKを計算して表示する実行スクリプト。"""

from __future__ import annotations

import argparse
import json
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


# 既存のバッグ把持で使っている、右腕を畳んだ退避姿勢。
BAG_GRASP_HOME_POSE = HOME_POSE.copy()
BAG_GRASP_HOME_POSE["right_elbow_joint"] = -1.0

# 動作確認用の待機時間。DirectJointController は待機中も目標姿勢を保持する。
BAG_HOME_HOLD_SEC = 3.0
GRASP_HOLD_SEC = 5.0
INITIAL_HOLD_SEC = 3.0
READY_TIMEOUT_SEC = 8.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Recognize an object and print its grasp IK plan."
    )
    parser.add_argument(
        "--target",
        help="YOLOへ渡す物体名。省略すると実行時に入力する。例: bottle",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=8.0,
        help="物体認識のタイムアウト秒数（default: 8.0）",
    )
    parser.add_argument(
        "--head-tilt",
        type=float,
        default=0.0,
        help=(
            "現在の実際の頭部チルト角 [rad]。このスクリプトは頭を動かさない"
            "（default: 0.0）"
        ),
    )
    parser.add_argument(
        "--grasp-strategy",
        choices=("center", "bag"),
        default="center",
        help=(
            "把持点。centerは一般物体のbbox中心、bagは従来のバッグ上部"
            "（default: center）"
        ),
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
    initial_joints = None
    motion_started = False

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

        print("Moving from the initial posture to BAG GRASP HOME...")
        motion_started = True
        arm.send_joints(BAG_GRASP_HOME_POSE, hold_sec=BAG_HOME_HOLD_SEC)

        print(
            f"Searching for: {target_name} "
            f"(assumed head tilt={args.head_tilt:.3f} rad)"
        )
        detection = detector.detect(target_name, timeout=args.timeout)
        if detection is None:
            print(detector.last_failure_reason or f"Object was not detected: {target_name}")
            return

        plan = calculate_object_grasp_plan(
            detection,
            head_tilt=args.head_tilt,
            grasp_strategy=args.grasp_strategy,
        )
        print_grasp_plan(plan)

        if not plan.reachable:
            print(
                "警告: 対象は計算上の到達範囲外です。"
                "既存のbag_graspと同様に、到達限界へ丸めたIKを送信します。"
            )

        print("Moving from BAG GRASP HOME to the calculated grasp posture...")
        arm.send_joints(plan.joints, hold_sec=GRASP_HOLD_SEC)
        print("Grasp posture check finished.")
    finally:
        try:
            detector.stop()
        except Exception as error:
            node.get_logger().warn(f"Failed to stop YOLO: {error}")

        if motion_started:
            print("Moving back to BAG GRASP HOME...")
            try:
                arm.send_joints(
                    BAG_GRASP_HOME_POSE,
                    hold_sec=BAG_HOME_HOLD_SEC,
                )
            except Exception as error:
                node.get_logger().error(
                    f"Failed to return to BAG GRASP HOME: {error}"
                )

        if motion_started and initial_joints is not None:
            print("Moving from BAG GRASP HOME back to the initial posture...")
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
