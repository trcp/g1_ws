#!/usr/bin/env python3
from rclpy.node import Node
import rclpy

# smach state
import smach

# G1 API
from erasers_g1_api.robot_control import ArmControl, Collision, Grasp
from erasers_g1_api.tts import TTS
from erasers_g1_api.state_skills.wait_push_hand import WaitPushHand

# preferences
import traceback


"""
聞こえたメッセージを確認する
"""


@smach.cb_interface(
    outcomes=["success", "failure"], input_keys=["stt_text"], output_keys=[]
)
def cb_state_check_verify(userdata, node: Node, tts_say: TTS.say):
    try:
        stt_text = userdata.stt_text
        tts_say("I heard %s" % stt_text)
        return "success"
    except Exception as e:
        node.get_logger().error(f"Error in grasp_bag: {e}\n{traceback.format_exc()}")
        return "failure"


"""
main
"""


def main():
    # init ROS
    rclpy.init()
    node = Node("sample_wait_push_hand")

    # init API
    tts = TTS(node)
    SAY = tts.say
    # ARM = ArmControl(node)

    # ARM.enable_upper_body_control(True)
    # ARM.move_groupstate(group_state="walk")
    # ARM.enable_upper_body_control(False)

    # declare start task
    node.get_logger().info("""
    ===============================
    sample_wait_push_hand START
    ===============================
    """)

    # init sm
    sm = smach.StateMachine(outcomes=["success", "failure"])

    # sm
    with sm:
        smach.StateMachine.add(
            "WAIT_PUSH_HAND",
            WaitPushHand(
                node=node,
                tts_say=SAY,
            ),
            # arm_control=ARM),
            transitions={
                "success": "success",
                "timeout": "WAIT_PUSH_HAND",
                "failure": "failure",
            },
        )

    outcome = sm.execute()
    node.destroy_node()
