#!/usr/bin/env python3
"""
Door-opening posture tuning tool.

Starts from all-zero upper-body joints, keeps publishing the current target
posture, and lets you increment joints or send/save named poses.
"""
import ast
import json
import os
import sys
import threading
import time

import rclpy
from rclpy.node import Node

HRI_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(HRI_DIR)

from direct_joint_control import DirectJointController  # noqa: E402


JOINTS = [
    "waist_yaw_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
]

ALIASES = {
    "waist": "waist_yaw_joint",
    "lsp": "left_shoulder_pitch_joint",
    "lsr": "left_shoulder_roll_joint",
    "lsy": "left_shoulder_yaw_joint",
    "le": "left_elbow_joint",
    "lwr": "left_wrist_roll_joint",
    "rsp": "right_shoulder_pitch_joint",
    "rsr": "right_shoulder_roll_joint",
    "rsy": "right_shoulder_yaw_joint",
    "re": "right_elbow_joint",
    "rwr": "right_wrist_roll_joint",
}

DEFAULT_SAVE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "door_joint_poses.json")


class DoorJointTuningTool(Node):
    def __init__(self):
        super().__init__("door_joint_tuning_tool")
        self.direct_arm = DirectJointController(self)
        self.pose = {joint: 0.0 for joint in JOINTS}
        self.last_joint = None
        self.step = 0.05
        self.save_path = DEFAULT_SAVE_PATH

    def send_pose(self, hold_sec=0.0):
        self.direct_arm.send_joints(self.pose.copy(), hold_sec=hold_sec)

    def set_joint(self, joint, value):
        self.pose[joint] = value
        self.last_joint = joint
        self.send_pose()

    def add_joint(self, joint, delta):
        self.set_joint(joint, self.pose.get(joint, 0.0) + delta)

    def apply_pose(self, pose):
        valid = {}
        for key, value in pose.items():
            joint = resolve_joint(key)
            if joint not in JOINTS:
                raise ValueError(f"Unknown joint: {key}")
            valid[joint] = float(value)
        self.pose.update(valid)
        self.send_pose()

    def load_saved_poses(self):
        if not os.path.exists(self.save_path):
            return {}
        with open(self.save_path, "r", encoding="utf-8") as f:
            return json.load(f)

    def save_named_pose(self, name):
        data = self.load_saved_poses()
        data[name] = {
            "saved_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            "pose": self.pose.copy(),
        }
        with open(self.save_path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        return self.save_path

    def load_named_pose(self, name):
        data = self.load_saved_poses()
        if name not in data:
            raise KeyError(f"No saved pose named {name!r}")
        pose = data[name].get("pose", data[name])
        self.apply_pose(pose)


def resolve_joint(name):
    key = name.strip()
    return ALIASES.get(key, key)


def parse_pose_text(text):
    value = ast.literal_eval(text)
    if not isinstance(value, dict):
        raise ValueError("Pose must be a dict")
    return value


def print_help():
    print("""
Commands:
  show                         print current target pose
  joints                       print aliases
  step 0.05                    set default increment
  zero                         send all target joints to 0.0 and hold
  <joint> <value>              set one joint, e.g. lsp -0.8
  <joint> +                    add current step to one joint
  <joint> -                    subtract current step from one joint
  + / -                        add/subtract step from the last selected joint
  add <joint> <delta>          add explicit delta, e.g. add waist 0.03
  pose {"lsp": -0.8, "le": 1.0} send multiple values at once
  save <name>                  save current pose to door_joint_poses.json
  load <name>                  load saved pose and hold it
  saved                        list saved pose names
  exit                         quit
""".strip())


def print_pose(pose):
    for joint in JOINTS:
        print(f"  {joint:<28} {pose.get(joint, 0.0): .4f}")


def main():
    rclpy.init()
    tool = DoorJointTuningTool()
    spin_thread = threading.Thread(target=rclpy.spin, args=(tool,), daemon=True)
    spin_thread.start()

    print("\n=== Door Joint Tuning Tool ===")
    print("Sending all-zero upper-body pose and holding it.")
    tool.send_pose(hold_sec=1.0)
    print_help()

    try:
        while rclpy.ok():
            line = input("\ndoor> ").strip()
            if not line:
                continue
            parts = line.split(maxsplit=2)
            cmd = parts[0].lower()

            try:
                if cmd in ("exit", "quit"):
                    break
                if cmd == "help":
                    print_help()
                elif cmd == "joints":
                    for alias, joint in ALIASES.items():
                        print(f"  {alias:<5} -> {joint}")
                elif cmd == "show":
                    print_pose(tool.pose)
                elif cmd == "step":
                    if len(parts) < 2:
                        print(f"step = {tool.step}")
                    else:
                        tool.step = float(parts[1])
                        print(f"step = {tool.step}")
                elif cmd == "zero":
                    tool.pose = {joint: 0.0 for joint in JOINTS}
                    tool.send_pose()
                    print("Sent zero pose.")
                elif cmd == "pose":
                    if len(parts) < 2:
                        print("Usage: pose {'lsp': -0.8, 'le': 1.0}")
                        continue
                    pose_text = line[len("pose"):].strip()
                    tool.apply_pose(parse_pose_text(pose_text))
                    print("Sent pose.")
                elif cmd == "save":
                    if len(parts) < 2:
                        print("Usage: save <name>")
                        continue
                    path = tool.save_named_pose(parts[1])
                    print(f"Saved {parts[1]!r} to {path}")
                elif cmd == "saved":
                    data = tool.load_saved_poses()
                    print("Saved poses:")
                    for name in data:
                        print(f"  {name}")
                elif cmd == "load":
                    if len(parts) < 2:
                        print("Usage: load <name>")
                        continue
                    tool.load_named_pose(parts[1])
                    print(f"Loaded {parts[1]!r}.")
                elif cmd in ("+", "-"):
                    if tool.last_joint is None:
                        print("No last joint. Set a joint first, e.g. lsp -0.5")
                        continue
                    delta = tool.step if cmd == "+" else -tool.step
                    tool.add_joint(tool.last_joint, delta)
                    print(f"{tool.last_joint} = {tool.pose[tool.last_joint]:.4f}")
                elif cmd == "add":
                    if len(parts) < 3:
                        print("Usage: add <joint> <delta>")
                        continue
                    joint = resolve_joint(parts[1])
                    tool.add_joint(joint, float(parts[2]))
                    print(f"{joint} = {tool.pose[joint]:.4f}")
                else:
                    if len(parts) < 2:
                        print("Usage: <joint> <value|+|->")
                        continue
                    joint = resolve_joint(parts[0])
                    if joint not in JOINTS:
                        print(f"Unknown joint: {parts[0]}")
                        continue
                    op = parts[1]
                    if op == "+":
                        tool.add_joint(joint, tool.step)
                    elif op == "-":
                        tool.add_joint(joint, -tool.step)
                    else:
                        tool.set_joint(joint, float(op))
                    print(f"{joint} = {tool.pose[joint]:.4f}")
            except Exception as e:
                print(f"Error: {e}")
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        tool.direct_arm.active = False
        rclpy.shutdown()


if __name__ == "__main__":
    main()
