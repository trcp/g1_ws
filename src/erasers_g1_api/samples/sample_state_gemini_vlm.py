import rclpy
from rclpy.node import Node
import smach

# API
from erasers_g1_api.robot_control import HeadControl
from erasers_g1_api.tts import VoicevoxTTS, VuiTTS
from erasers_g1_api.state_skills.recongnition import GeminiVLM, SpeechToText


def main():
    # ROS 2 初期化
    rclpy.init()
    node = Node("sample_gemini_vlm")

    # API 初期化
    head_control = HeadControl(node)
    head_control.move_to_pose()
    tts = VoicevoxTTS(node) # 日本語にするにはこれをコメントアウトしてすべての message 引数に日本語を代入すること．
    #tts = VuiTTS(node)

    # SMACH ステートマシンの初期化
    sm = smach.StateMachine(outcomes=["success", "failure"])

    # userdata
    sm.userdata.prompt = "床にエビフライはいくつありますか？"
    sm.userdata.success_keywards = []
    sm.userdata.schema = "{'description': 'str'}"

    with sm:
        # 音声認識
        '''
        smach.StateMachine.add('VOICE_RECONG', SpeechToText(node=node,
                                                            tts=tts,
                                                            req_message="Please say anything after beep sound.",
                                                            success_message="",
                                                            device='cuda',
                                                            lang='en',
                                                            silence_duration=5.0),
                                transitions={'success': 'GEMINI_VLM',
                                             'timeout': 'VOICE_RECONG',
                                             'failure':'failure'},
                                remapping={'stt_text': 'prompt'})
        '''
        # GeminiVLM ステート
        smach.StateMachine.add("GEMINI_VLM",GeminiVLM(node=node,
                                                      tts=tts,
                                                      head_control=head_control,
                                                      req_message="わかりました．",
                                                      wait_message="考え中です．",
                                                      failure_message="エラー．",
                                                      success_message=""),
                                transitions={
                                    "success": "success",
                                    "failure": "failure",
                                },
        )

    # ステートマシンの実行
    outcome = sm.execute()
    tts.say(sm.userdata.output['description'])

    node.destroy_node()
    rclpy.shutdown()
