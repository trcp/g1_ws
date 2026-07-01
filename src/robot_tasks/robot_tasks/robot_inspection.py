"""
Robot Inspection タスク
"""

#!/usr/bin/env python3

# ROS2
from rclpy.node import Node
import rclpy
import smach

from geometry_msgs.msg import PoseStamped

# eraasers g1 APIs
from erasers_g1_api.tts import TTS
from erasers_g1_api.robot_control import G1Navigation, G1Control
from erasers_g1_api.state_skills.wait_door_open import WaitDoorOpen
from erasers_g1_api.state_skills.recongnition import SpeechToText

# preferences
from ament_index_python.packages import get_package_share_directory
from typing import List
import traceback
import yaml
import os
import time


"""
指定されたロケーションに移動する
"""


@smach.cb_interface(
    outcomes=["success", "timeout", "failure"],
    input_keys=["initial_pose"],
    output_keys=[],
)
def initial_pose_cb(
    userdata,
    node: Node,
    tts_say: TTS.say,
    navigation: G1Navigation,
    message: str = "I will move.",
):
    try:
        navigation.move_rel(0.8, 0.0, 0.0, use_odom_only=True, tolerance=0.1)
        navigation.set_initialpose(
            pose=[
                userdata.initial_pose[0],
                userdata.initial_pose[1],
                userdata.initial_pose[2],
            ],
            # max_attempts=100,
            max_attempts=10,
            tolerance=0.65,
            settle_time=3.0,
        )
        tts_say(message)
        return "success"
    except Exception as e:
        node.get_logger().error(
            f"Error in move_to_location_cb: {e}\n{traceback.format_exc()}"
        )
        return "failure"


"""
指定されたロケーションに移動する
"""


@smach.cb_interface(
    outcomes=["success", "timeout", "failure"],
    input_keys=["abs_pose"],
    output_keys=[],
)
def move_to_pose_cb(
    userdata,
    node: Node,
    tts_say: TTS.say,
    navigation: G1Navigation,
    init_pose: bool = False,
    message: str = "I will move.",
):
    try:
        navigation.move_abs(
            userdata.abs_pose[0], userdata.abs_pose[1], userdata.abs_pose[2]
        )
        return "success"

    except Exception as e:
        node.get_logger().error(
            f"Error in move_to_location_cb: {e}\n{traceback.format_exc()}"
        )
        return "failure"


"""
main
"""


def main():
    # init ROS
    rclpy.init()
    node = Node("robot_inspection_task")

    # init API
    tts = TTS(node)
    SAY = tts.say
    CONTROL = G1Control(node)
    NAVIGATION = G1Navigation(node)
    NAVIGATION.TIMEOUT_SEC = 120.0

    # init smach
    sm = smach.StateMachine(outcomes=["success", "failure"])

    # userdatas
    sm.userdata.initial_pose = [-0.73, -0.59, -2.81]
    sm.userdata.inspection_point = [-5.67, -1.85, -2.81]
    sm.userdata.exit_point = [0.936, -4.187, 0.435]
    sm.userdata.success_keywards = [
        "byebye",
        "ByeBye",
        "BYEBYE",
        "bye bye",
        "Bye Bye",
        "BYE BYE",
        "Bye-Bye",
        "bye-bye",
    ]
    sm.userdata.num_challenge = 0

    SAY("robot inspection task start!")

    with sm:
        smach.StateMachine.add(
            "WAIT_DOOR_OPEN",
            WaitDoorOpen(node=node, tts_say=SAY, timeout_sec=20, threshold=1.0),
            transitions={
                "success": "INIT_POSE",
                "timeout": "WAIT_DOOR_OPEN",
                "failure": "failure",
            },
        )

        smach.StateMachine.add(
            "INIT_POSE",
            smach.CBState(
                cb=initial_pose_cb,
                cb_kwargs={
                    "node": node,
                    "tts_say": SAY,
                    "navigation": NAVIGATION,
                    "message": "I will go to the inspection point.",
                },
            ),
            transitions={
                "success": "MOVE_INSPECTION_POINT",
                "timeout": "failure",
                "failure": "failure",
            },
            # remapping={"initial_pose": "inspection_point"},
        )

        smach.StateMachine.add(
            "MOVE_INSPECTION_POINT",
            smach.CBState(
                cb=move_to_pose_cb,
                cb_kwargs={
                    "node": node,
                    "tts_say": SAY,
                    "navigation": NAVIGATION,
                    "init_pose": True,
                },
            ),
            transitions={
                "success": "HUMAN_INTARACTION",
                "timeout": "failure",
                "failure": "failure",
            },
            remapping={"abs_pose": "inspection_point"},
        )

        smach.StateMachine.add(
            "HUMAN_INTARACTION",
            SpeechToText(
                node=node,
                tts=tts,
                start_msg="My name is erasers_g1, Did you finish my inspection?? If so, please say ByeBye or NO after the chime sounds.",
                success_msg="Thank you! I will go to exit.",
                timeout_msg="OK. I will stay.",
                max_challenge=100,
            ),
            transitions={
                "success": "MOVE_EXIT_POINT",
                # "success": "success",
                "timeout": "HUMAN_INTARACTION",
                "failure": "failure",
            },
        )

        smach.StateMachine.add(
            "MOVE_EXIT_POINT",
            smach.CBState(
                cb=move_to_pose_cb,
                cb_kwargs={
                    "node": node,
                    "tts_say": SAY,
                    "navigation": NAVIGATION,
                    "message": "Move to exit point.",
                },
            ),
            transitions={
                "success": "success",
                "timeout": "failure",
                "failure": "failure",
            },
            remapping={"abs_pose": "exit_point"},
        )

    # execute smach states
    outcome = sm.execute()
    if outcome == "success":
        SAY("I finished the task.")
    else:
        SAY("I failed to finish the task.")
    node.destroy_node()
