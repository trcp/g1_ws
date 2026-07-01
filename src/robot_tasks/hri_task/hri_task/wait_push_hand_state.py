#!/usr/bin/env python3
"""Start-signal state: wait until the operator pushes the robot hand."""
import argparse
import os
import sys
import time
import traceback

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import smach
from sensor_msgs.msg import JointState

from direct_joint_control import DirectJointController
from erasers_g1_api.tts import TTS


WAIT_PUSH_POSE_RIGHT = {
    "right_shoulder_pitch_joint": -0.735,
    "right_elbow_joint": 0.97,
    "right_wrist_roll_joint": 1.57,
}

WAIT_PUSH_POSE_LEFT = {
    "left_shoulder_pitch_joint": -0.735,
    "left_elbow_joint": 0.97,
    "left_wrist_roll_joint": -1.57,
}


class WaitPushHandState(smach.State):
    """Wait for a push on one arm using direct joint control and effort delta."""

    def __init__(
        self,
        node: Node,
        tts_say,
        direct_arm: DirectJointController,
        start_msg: str = "I will up right arm. Please push my arm after up the arm.",
        success_msg: str = "Thank you.",
        timeout_msg: str = "No push detected. I will start anyway.",
        failure_msg: str = "Push hand detection failed. I will start anyway.",
        hand: str = "right",
        timeout_sec: float = 30.0,
        threshold: float = 1.5,
        restore_home: bool = True,
    ):
        smach.State.__init__(self, outcomes=["success", "timeout", "failure"])
        self.node = node
        self.say = tts_say
        self.direct_arm = direct_arm
        self.start_msg = start_msg
        self.success_msg = success_msg
        self.timeout_msg = timeout_msg
        self.failure_msg = failure_msg
        self.hand = hand
        self.timeout_sec = timeout_sec
        self.threshold = threshold
        self.restore_home = restore_home

        if self.hand not in ("left", "right"):
            raise ValueError("hand must be 'left' or 'right'")

        self.push_detected = False
        self.initial_effort = None
        self.last_delta = 0.0
        self.effort_joint = f"{self.hand}_elbow_joint"

    def _wait_pose(self):
        return WAIT_PUSH_POSE_LEFT if self.hand == "left" else WAIT_PUSH_POSE_RIGHT

    def _joint_state_cb(self, msg: JointState):
        try:
            if self.effort_joint not in msg.name:
                return
            idx = msg.name.index(self.effort_joint)
            if idx >= len(msg.effort):
                return

            effort = float(msg.effort[idx])
            if effort != effort:
                return

            if self.initial_effort is None:
                self.initial_effort = effort
                self.node.get_logger().info(
                    f"[WAIT PUSH HAND] Initial {self.effort_joint} effort={effort:.3f}"
                )
                return

            self.last_delta = self.initial_effort - effort
            if self.last_delta >= self.threshold:
                self.node.get_logger().info(
                    f"[WAIT PUSH HAND] Push detected: delta={self.last_delta:.3f}"
                )
                self.push_detected = True
        except Exception as exc:
            self.node.get_logger().warn(f"[WAIT PUSH HAND] JointState callback error: {exc}")

    def _safely_say(self, text):
        if not self.say or not text:
            return
        try:
            self.say(text)
        except Exception as exc:
            self.node.get_logger().warn(f"[WAIT PUSH HAND] TTS failed: {exc}")

    def execute(self, userdata):
        self.node.get_logger().info("[WAIT PUSH HAND] Waiting for start push...")
        self.push_detected = False
        self.initial_effort = None
        self.last_delta = 0.0
        sub = None

        try:
            self._safely_say(self.start_msg)
            if self.direct_arm is None:
                self.node.get_logger().warn("[WAIT PUSH HAND] direct_arm is not set.")
                self._safely_say(self.failure_msg)
                return "failure"

            self.direct_arm.send_joints(self._wait_pose(), hold_sec=1.0)

            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            )
            sub = self.node.create_subscription(
                JointState, "/joint_states", self._joint_state_cb, qos
            )

            
            start_time = time.time()
            while rclpy.ok() and time.time() - start_time < self.timeout_sec:
                if self.push_detected:
                    break
                rclpy.spin_once(self.node, timeout_sec=0.1)

            if self.restore_home:
                self.direct_arm.go_home(hold_sec=0.5)

            if self.push_detected:
                self._safely_say(self.success_msg)
                return "success"

            self.node.get_logger().warn(
                f"[WAIT PUSH HAND] Timeout. last_delta={self.last_delta:.3f}"
            )
            self._safely_say(self.timeout_msg)
            return "timeout"

        except Exception as exc:
            self.node.get_logger().warn(f"[WAIT PUSH HAND] Error: {exc}")
            self.node.get_logger().warn(traceback.format_exc())
            self._safely_say(self.failure_msg)
            return "failure"

        finally:
            if sub is not None:
                try:
                    self.node.destroy_subscription(sub)
                except Exception:
                    pass


def main():
    parser = argparse.ArgumentParser(description="Standalone wait-push-hand test.")
    parser.add_argument("--hand", choices=["left", "right"], default="right")
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--threshold", type=float, default=1.5)
    args = parser.parse_args()

    rclpy.init()
    node = Node("wait_push_hand_test")
    try:
        tts = TTS(node)
        arm = DirectJointController(node)
        state = WaitPushHandState(
            node=node,
            tts_say=tts.say,
            direct_arm=arm,
            hand=args.hand,
            timeout_sec=args.timeout,
            threshold=args.threshold,
        )
        outcome = state.execute(None)
        print(outcome)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
