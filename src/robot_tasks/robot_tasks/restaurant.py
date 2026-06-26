"""
レストランタスク
"""

#!/usr/bin/env python3

# ROS2
from rclpy.node import Node
import rclpy
import smach

# interfaces
from geometry_msgs.msg import PoseStamped
from lor_interfaces.msg import Person3D, Persons3D  # Light Weight Open Pose

# eraasers g1 APIs
from erasers_g1_api.tts import TTS
from erasers_g1_api.robot_control import (
    G1Navigation,
    G1Control,
    ArmControl,
    Collision,
    Grasp,
)
from erasers_g1_api.state_skills.recongnition import (
    SpeechToText,
    LOR,
    Sam3ObjectDetector,
)
from erasers_g1_api.state_skills.grasping import object_grasping

# preferences
from ament_index_python.packages import get_package_share_directory
from typing import List
import traceback
import yaml
import os
import random  # ランダム選択用に追加


# --- デバッグ・シミュレーション用定数 ---
SKIP_VOICE_INTARACT = True  # True のとき音声対話ステートをスキップして success 扱いにする
SKIP_HAND_CONTROL = True # True のときハンド操作をスキップする


"""
パラメータを読み込む
"""


def load_params(node: Node, params_file: str):
    node.get_logger().info("Get tasks parameter from: %s" % params_file)
    try:
        with open(params_file, "r") as f:
            params = yaml.safe_load(f)
            objects = params["restaurant_task"]["ros_parameters"]["objects"]
            objects_str = "\n".join(objects.keys())
            node.get_logger().info(f"Objects list:\n{objects_str}")
            # キーワードリストを生成
            keywords_list = [kw for obj in objects.values() for kw in obj[0]]
            return keywords_list, objects
    except Exception as e:
        node.get_logger().error(f"Failed to load params: {e}")
        node.get_logger().error(traceback.format_exc())


"""
音声対話スキップ用のダミーステート（SKIP_VOICE_INTARACT = True の時に使用）
"""


@smach.cb_interface(outcomes=["success"], input_keys=["object_keywards"], output_keys=["stt_text", "order_list"])
def cb_state_skip_request_order(userdata, node: Node):
    try:
        kws = userdata.object_keywards
        # キーワードからランダムに2つ選択（要素数が2未満の場合は全て選択）
        chosen = random.sample(kws, min(2, len(kws)))
        
        # 後続のチェックステート(cb_state_check_order)が壊れないように文字列でも結合して代入
        userdata.stt_text = " ".join(chosen)
        userdata.order_list = chosen
        
        node.get_logger().info(f"[SKIP_VOICE] Randomly selected orders: {chosen}")
        return "success"
    except Exception as e:
        node.get_logger().error(f"Error in cb_state_skip_request_order: {e}")
        return "success"


@smach.cb_interface(outcomes=["success"], input_keys=[], output_keys=["stt_text"])
def cb_state_skip_request_apply(userdata, node: Node):
    # 注文確認で肯定（yes）をシミュレート
    userdata.stt_text = "yes"
    node.get_logger().info("[SKIP_VOICE] Skipped request apply -> simulated 'yes'")
    return "success"


@smach.cb_interface(outcomes=["success"], input_keys=[], output_keys=[])
def cb_state_skip_grasp_apply(userdata, node: Node):
    node.get_logger().info("[SKIP_VOICE] Skipped grasp apply")
    return "success"


@smach.cb_interface(outcomes=["success"], input_keys=[], output_keys=["stt_text"])
def cb_state_skip_handover_confirm(userdata, node: Node):
    # 受け取り確認で肯定（yes）をシミュレート
    userdata.stt_text = "yes"
    node.get_logger().info("[SKIP_VOICE] Skipped handover confirm -> simulated 'yes'")
    return "success"


"""
周囲のマップを作成
"""


@smach.cb_interface(outcomes=["success", "failure"], input_keys=[], output_keys=[])
def cb_state_create_around_map(
    userdata, node: Node, tts_say: TTS.say, navigation: G1Navigation, control: G1Control
):
    try:
        control.pose_policy("running")
        tts_say("I will create aound map. Please wait a moment.")

        navigation.move_rel(yaw=1.57)
        navigation.move_rel(yaw=1.57)
        navigation.move_rel(yaw=1.57)
        navigation.move_rel(yaw=1.57)
        navigation.move_abs()

        tts_say("I created aound map!")
        control.pose_policy("start")
        return "success"
    except Exception as e:
        node.get_logger().error(f"Error in grasp_bag: {e}\n{traceback.format_exc()}")
        return "failure"


"""
お客さんまで移動
"""


@smach.cb_interface(
    outcomes=["success", "failure"], input_keys=["person_poses", "person_poses_header"], output_keys=[]
)
def cb_state_move_to_customer(
    userdata,
    node: Node,
    tts_say: TTS.say,
    navigation: G1Navigation,
    control: G1Control,
    tolerance: float = 1.5,
):
    try:
        control.pose_policy("running")
        tts_say("I will move to the customer. Please wait a moment.")
        person_pose: Person3D = userdata.person_poses[0]
        person_x = person_pose.pose.position.x
        person_y = person_pose.pose.position.y
        person_header = userdata.person_poses_header

        node.get_logger().info("frame_id: %s" % person_header.frame_id)
        node.get_logger().info("pose x: %f" % person_x)
        node.get_logger().info("pose y: %f" % person_y)

        customer_pose = PoseStamped()
        customer_pose.header = person_header
        customer_pose.pose = person_pose.pose
        if not navigation.move_to_pose(customer_pose, tolerance=tolerance):
            node.get_logger().error("Failed to move to customer")
            control.pose_policy("start")
            return "failure"

        tts_say("I reached the customer!")
        control.pose_policy("start")
        return "success"

    except Exception as e:
        node.get_logger().error(f"Error in grasp_bag: {e}\n{traceback.format_exc()}")
        return "failure"


"""
お客さんの注文を確認する
"""


@smach.cb_interface(
    outcomes=["success", "timeout", "failure"],
    input_keys=["object_keywards", "stt_text"],
    output_keys=["order_list"],
)
def cb_state_check_order(userdata, node: Node, tts_say: TTS.say):
    try:
        voice_message = userdata.stt_text
        order_list = [
            kw for kw in userdata.object_keywards if kw in voice_message.lower()
        ]
        userdata.order_list = order_list
        if order_list == []:
            tts_say("Sorry, I failed to understand your order. Please try again.")
            return "timeout"
        tts_say("You ordered %s." % "，".join(order_list))
        return "success"
    except:
        node.get_logger().error(
            "Error is occured in check_order\n%s" % traceback.format_exc()
        )
        return "failure"


"""
お客さんのからの確認結果
"""


@smach.cb_interface(
    outcomes=["apply", "reject", "failure"],
    input_keys=["stt_text", "order_list", "objects_dict"],
    output_keys=["stt_text", "request_objects_dict"],
)
def cb_state_check_order_confirmation(userdata, node: Node, tts_say: TTS.say):
    try:
        voice_message = userdata.stt_text.lower()
        if "yes" in voice_message and "no" not in voice_message:
            tts_say("OK. I will go to the bar counter.")

            # 注文された品物に絞り込む
            request_objects_dict = {}
            for name, data in userdata.objects_dict.items():
                kws, conf = data
                if any(kw in userdata.order_list for kw in kws):
                    request_objects_dict[name] = conf

            userdata.request_objects_dict = request_objects_dict
            node.get_logger().info(f"Request objects: {request_objects_dict}")

            return "apply"
        else:
            tts_say("OK. I will ask your order again.")
            userdata.order_list = []
            return "reject"

    except:
        node.get_logger().error(
            "Error is occured in check_order\n%s" % traceback.format_exc()
        )
        return "failure"


"""
バーカウンターへ戻る
"""


@smach.cb_interface(outcomes=["success", "failure"], input_keys=[], output_keys=[])
def cb_state_move_to_bar_counter(
    userdata, node: Node, navigation: G1Navigation, control: G1Control
):
    try:
        control.pose_policy("running")

        navigation.move_abs(0.0, 0.0, 0.0)

        control.pose_policy("start")
        return "success"
    except Exception as e:
        node.get_logger().error(f"Error in grasp_bag: {e}\n{traceback.format_exc()}")
        return "failure"


"""
お客さんの注文を報告する
"""


@smach.cb_interface(
    outcomes=["success", "failure"],
    input_keys=["order_list"],
    output_keys=["order_list"],
)
def cb_state_order_report(userdata, node: Node, tts_say: TTS.say, arm:ArmControl):
    try:
        tts_say("Barman, the customer ordered %s." % "，".join(userdata.order_list))
        tts_say(
            "I will grasp there. Please put ther both my hand"
            if len(userdata.order_list) > 1
            else "I will grasp it. Please put ther which my hand"
        )
        arm.enable_upper_body_control(True)
        arm.joint_control(left_shoulder_pitch_joint=-0.735,
                      left_wrist_roll_joint=-1.57,
                      right_shoulder_pitch_joint=-0.735,
                      right_wrist_roll_joint=1.57,)
        userdata.order_list = []
        return "success"
    except:
        node.get_logger().error(
            "Error is occured in cb_state_order_report\n%s" % traceback.format_exc()
        )
        return "failure"


"""
アイテムを掴む
"""


@smach.cb_interface(
    outcomes=["success", "failure"],
    input_keys=[],
    output_keys=[],
)
def cb_state_grasp_item(userdata, node: Node, arm:ArmControl):
    try:
        if not SKIP_HAND_CONTROL: arm.hand_control(command="close")
        arm.enable_upper_body_control(False)
        return "success"
    except:
        node.get_logger().error(
            "Error is occured in cb_state_grasp_item\n%s" % traceback.format_exc()
        )
        return "failure"


"""
注文客へアイテムを渡す
"""


@smach.cb_interface(
    outcomes=["success", "failure"],
    input_keys=[],
    output_keys=[],
)
def cb_state_handover_item(userdata, node: Node, tts_say: TTS.say, arm: ArmControl):
    try:
        tts_say("I will hand over your order. Please take it from my hand.")
        arm.enable_upper_body_control(True)
        arm.joint_control(
            left_shoulder_pitch_joint=-0.735,
            left_wrist_roll_joint=-1.57,
            right_shoulder_pitch_joint=-0.735,
            right_wrist_roll_joint=1.57,
        )
        if not SKIP_HAND_CONTROL: arm.hand_control(command="open")
        return "success"
    except:
        node.get_logger().error(
            "Error is occured in cb_state_handover_item\n%s"
            % traceback.format_exc()
        )
        return "failure"


"""
注文客がアイテムを受け取ったか確認する
"""


@smach.cb_interface(
    outcomes=["received", "not_received", "failure"],
    input_keys=["stt_text"],
    output_keys=[],
)
def cb_state_check_handover_confirmation(userdata, node: Node, tts_say: TTS.say):
    try:
        voice_message = userdata.stt_text.lower()
        if "yes" in voice_message and "no" not in voice_message:
            tts_say("Thank you. I will go back to the barcounter.")
            return "received"

        tts_say("OK. Please take the item from my hand.")
        return "not_received"
    except:
        node.get_logger().error(
            "Error is occured in cb_state_check_handover_confirmation\n%s"
            % traceback.format_exc()
        )
        return "failure"


"""
初期姿勢・初期位置へ戻る
"""


@smach.cb_interface(
    outcomes=["next_order", "done", "failure"],
    input_keys=["completed_orders", "max_orders"],
    output_keys=["completed_orders", "stt_text", "order_list", "request_objects_dict"],
)
def cb_state_reset_and_return(
    userdata,
    node: Node,
    navigation: G1Navigation,
    control: G1Control,
    arm: ArmControl,
):
    try:
        arm.enable_upper_body_control(False)
        control.pose_policy("running")
        navigation.move_abs()
        control.pose_policy("start")

        userdata.completed_orders += 1
        node.get_logger().info(
            "Completed restaurant orders: %d/%d"
            % (userdata.completed_orders, userdata.max_orders)
        )

        userdata.stt_text = ""
        userdata.order_list = []
        userdata.request_objects_dict = {}

        if userdata.completed_orders < userdata.max_orders:
            return "next_order"
        return "done"
    except Exception as e:
        node.get_logger().error(
            f"Error in cb_state_reset_and_return: {e}\n{traceback.format_exc()}"
        )
        return "failure"



"""
main
"""


def main():
    # Init ROS2
    rclpy.init()
    node = Node("restaurant_task")

    # 注文品リストを取得する
    default_params_file_path = os.path.join(
        get_package_share_directory("robot_tasks"), "params", "restaurant_task.yaml"
    )
    node.declare_parameter("task_params", default_params_file_path)
    object_keywards, objects_dict = load_params(
        node, node.get_parameter("task_params").value
    )

    # init APIs
    tts = TTS(node)
    arm = ArmControl(node)
    collision = Collision(node)
    CONROL = G1Control(node)
    ARM = Grasp(arm, collision)
    SAY = tts.say
    NAVIGATION = G1Navigation(node)

    # init pose
    # ARM.arm.enable_upper_body_control(False)
    ARM.arm.move_groupstate(group_state="walk")
    ARM.arm.enable_upper_body_control(False)

    # init smach
    sm = smach.StateMachine(outcomes=["success", "failure"])

    # userdatas
    sm.userdata.num_challenge = 0
    sm.userdata.stt_text = ""  # 音声認識の結果
    sm.userdata.order_list = []  # 注文品リスト
    sm.userdata.object_keywards = object_keywards  # 注文品のキーワードリスト
    sm.userdata.apply_keywards = ["yes", "no"]  # 注文確認のキーワードリスト
    sm.userdata.objects_dict = objects_dict
    sm.userdata.request_objects_dict = {}
    sm.userdata.completed_orders = 0
    sm.userdata.max_orders = 2

    SAY("restaurant task start!")

    with sm:
        # smach.StateMachine.add('CREATE_AROUND_MAP', smach.CBState(cb=cb_state_create_around_map,
        #                                                           cb_kwargs={'node': node, 'tts_say': SAY, 'navigation': NAVIGATION, 'control': CONROL},),
        #                          transitions={'success': 'SEARCH_CUSTOMER',
        #                                       'failure': 'failure'})

        smach.StateMachine.add(
            "SEARCH_CUSTOMER",
            LOR(
                node=node,
                tts_say=SAY,
                robot_control=CONROL,
                searching_area=[[0.0, -1.0], [0.0, 1.0]],
                start_msg="Hi customers! Please raise your hand if you want to order.",
                timeout_msg="Sorry, I couldn't find any customers.",
                success_msg="I found a customer!",
                detect_condition="hand_up",
            ),
            transitions={
                "success": "MOVE_TO_CUSTOMER",  #'success',
                "timeout": "SEARCH_CUSTOMER",
                "failure": "failure",
            },
        )

        smach.StateMachine.add(
            "MOVE_TO_CUSTOMER",
            smach.CBState(
                cb=cb_state_move_to_customer,
                cb_kwargs={
                    "node": node,
                    "tts_say": SAY,
                    "navigation": NAVIGATION,
                    "control": CONROL,
                },
            ),
            transitions={"success": "REQUEST_ORDER", "failure": "REQUEST_ORDER"},
        )

        # --- REQUEST_ORDER ステートの分岐 ---
        if SKIP_VOICE_INTARACT:
            smach.StateMachine.add(
                "REQUEST_ORDER",
                smach.CBState(cb=cb_state_skip_request_order, cb_kwargs={"node": node}),
                transitions={"success": "REQUEST_CHECK"}
            )
        else:
            smach.StateMachine.add(
                "REQUEST_ORDER",
                SpeechToText(
                    node=node,
                    tts=tts,
                    start_msg="hi customer. What is your order ? Please speak after the beep sound.",
                    success_msg="OK",
                    timeout_msg="Sorry, I didn't hear your order.",
                    device="cpu",
                    lang="en",
                ),
                transitions={
                    "success": "REQUEST_CHECK",
                    "timeout": "REQUEST_ORDER",
                    "failure": "failure",
                },
                remapping={"success_keywards": "object_keywards"},
            )

        smach.StateMachine.add(
            "REQUEST_CHECK",
            smach.CBState(
                cb=cb_state_check_order,
                cb_kwargs={"node": node, "tts_say": SAY},
            ),
            transitions={
                "success": "REQUEST_APPLY",
                "timeout": "REQUEST_ORDER",
                "failure": "failure",
            },
        )

        # --- REQUEST_APPLY ステートの分岐 ---
        if SKIP_VOICE_INTARACT:
            smach.StateMachine.add(
                "REQUEST_APPLY",
                smach.CBState(cb=cb_state_skip_request_apply, cb_kwargs={"node": node}),
                transitions={"success": "ORDER_CONFIRMATION"}
            )
        else:
            smach.StateMachine.add(
                "REQUEST_APPLY",
                SpeechToText(
                    node=node,
                    tts=tts,
                    start_msg="Is this correct ? Please say yes or no after the beep sound.",
                    success_msg="OK",
                    timeout_msg="OK. I will go back to the bar counter.",
                    device="cpu",
                    lang="en",
                ),
                transitions={
                    "success": "ORDER_CONFIRMATION",
                    "timeout": "MOVE_TO_BARCOUNTER",
                    "failure": "failure",
                },
                remapping={"success_keywards": "apply_keywards"},
            )

        smach.StateMachine.add(
            "ORDER_CONFIRMATION",
            smach.CBState(
                cb=cb_state_check_order_confirmation,
                cb_kwargs={"node": node, "tts_say": SAY},
            ),
            transitions={
                "apply": "MOVE_TO_BARCOUNTER",
                "reject": "REQUEST_ORDER",
                "failure": "failure",
            },
        )

        smach.StateMachine.add(
            "MOVE_TO_BARCOUNTER",
            smach.CBState(
                cb=cb_state_move_to_bar_counter,
                cb_kwargs={"node": node, "navigation": NAVIGATION, "control": CONROL},
            ),
            transitions={"success": "ORDER_REPORT", "failure": "failure"},
        )

        smach.StateMachine.add(
            "ORDER_REPORT",
            smach.CBState(
                cb=cb_state_order_report,
                cb_kwargs={"node": node, "tts_say": SAY, "arm": ARM.arm},
            ),
            transitions={"success": "GRASP_APPLY", "failure": "failure"},
        )

        # --- GRASP_APPLY ステートの分岐 ---
        if SKIP_VOICE_INTARACT:
            smach.StateMachine.add(
                "GRASP_APPLY",
                smach.CBState(cb=cb_state_skip_grasp_apply, cb_kwargs={"node": node}),
                transitions={"success": "GRASP"}
            )
        else:
            smach.StateMachine.add(
                "GRASP_APPLY",
                SpeechToText(
                    node=node,
                    tts=tts,
                    start_msg="Did you put the item in my hand? say yes or no after the beep sound.",
                    success_msg="OK",
                    timeout_msg="OK. I will grasp it.",
                    device="cpu",
                    lang="en",
                ),
                transitions={
                    "success": "GRASP",
                    "timeout": "GRASP_APPLY",
                    "failure": "failure",
                },
                remapping={"success_keywards": "apply_keywards"},
            )

        smach.StateMachine.add(
            "GRASP",
            smach.CBState(
                cb=cb_state_grasp_item,
                cb_kwargs={
                    "node": node,
                    "arm": ARM.arm,
                },
            ),
            transitions={"success": "BACK_TO_CUSTOMER", "failure": "failure"},
        )

        smach.StateMachine.add(
            "BACK_TO_CUSTOMER",
            smach.CBState(
                cb=cb_state_move_to_customer,
                cb_kwargs={
                    "node": node,
                    "tts_say": SAY,
                    "navigation": NAVIGATION,
                    "control": CONROL,
                },
            ),
            transitions={"success": "HANDOVER_ITEM", "failure": "failure"},
        )

        smach.StateMachine.add(
            "HANDOVER_ITEM",
            smach.CBState(
                cb=cb_state_handover_item,
                cb_kwargs={"node": node, "tts_say": SAY, "arm": ARM.arm},
            ),
            transitions={"success": "HANDOVER_CONFIRM", "failure": "failure"},
        )

        # --- HANDOVER_CONFIRM ステートの分岐 ---
        if SKIP_VOICE_INTARACT:
            smach.StateMachine.add(
                "HANDOVER_CONFIRM",
                smach.CBState(cb=cb_state_skip_handover_confirm, cb_kwargs={"node": node}),
                transitions={"success": "CHECK_HANDOVER_CONFIRMATION"}
            )
        else:
            smach.StateMachine.add(
                "HANDOVER_CONFIRM",
                SpeechToText(
                    node=node,
                    tts=tts,
                    start_msg="Did you receive your order? Please say yes or no after the beep sound.",
                    success_msg="OK",
                    timeout_msg="Sorry, I didn't hear you. Please answer yes or no.",
                    device="cpu",
                    lang="en",
                ),
                transitions={
                    "success": "CHECK_HANDOVER_CONFIRMATION",
                    "timeout": "HANDOVER_CONFIRM",
                    "failure": "failure",
                },
                remapping={"success_keywards": "apply_keywards"},
            )

        smach.StateMachine.add(
            "CHECK_HANDOVER_CONFIRMATION",
            smach.CBState(
                cb=cb_state_check_handover_confirmation,
                cb_kwargs={"node": node, "tts_say": SAY},
            ),
            transitions={
                "received": "RESET_AND_RETURN",
                "not_received": "HANDOVER_CONFIRM",
                "failure": "failure",
            },
        )

        smach.StateMachine.add(
            "RESET_AND_RETURN",
            smach.CBState(
                cb=cb_state_reset_and_return,
                cb_kwargs={
                    "node": node,
                    "navigation": NAVIGATION,
                    "control": CONROL,
                    "arm": ARM.arm,
                },
            ),
            transitions={
                "next_order": "SEARCH_CUSTOMER",
                "done": "success",
                "failure": "failure",
            },
        )

    # execute smach states
    outcome = sm.execute()
    if outcome == "success":
        SAY("I finished the task.")
    else:
        SAY("I failed to finish the task.")
    node.destroy_node()
