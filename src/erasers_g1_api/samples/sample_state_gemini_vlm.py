import rclpy
from rclpy.node import Node
import smach

# API
from erasers_g1_api.robot_control import HeadControl
from erasers_g1_api.tts import VoicevoxTTS
from erasers_g1_api.state_skills.recongnition import GeminiVLM


def main():
    # ROS 2 初期化
    rclpy.init()
    node = Node("sample_gemini_vlm")

    # API 初期化
    head_control = HeadControl(node)
    tts = VoicevoxTTS(node)

    # SMACH ステートマシンの初期化
    sm = smach.StateMachine(outcomes=["success", "failure"])

    # userdata の設定（prompt と schema は必須）
    sm.userdata.prompt = "画像に写っている人物の特徴（服装や持っている物）を説明してください。"
    sm.userdata.schema = "{'description': 'str', 'holding_items': 'list[str]'}"

    with sm:
        # GeminiVLM ステート
        smach.StateMachine.add("GEMINI_VLM",GeminiVLM(node=node,
                                                      tts=tts,
                                                      head_control=head_control),
                                transitions={
                                    "success": "success",
                                    "failure": "failure",
                                },
        )

    # ステートマシンの実行
    outcome = sm.execute()
    node.get_logger().info(f"Result: {sm.userdata.output}")

    node.destroy_node()
    rclpy.shutdown()
