#!/usr/bin/env python3

from rclpy.node import Node
import rclpy
import smach

# API
from erasers_g1_api.tts import VoicevoxTTS # 日本語発話用
from erasers_g1_api.robot_control import G1Control, HeadControl, ArmControl

# smach skills
from erasers_g1_api.state_skills.recongnition import GeminiVLM, SpeechToText

# LLM
from langchain_google_genai import ChatGoogleGenerativeAI
from langchain_core.tools import tool
from langchain_core.messages import HumanMessage, ToolMessage, SystemMessage

# general
import traceback
import os


PROMPT = """# Persona and Origin
You are "G1" (ジーワン), a life-sized humanoid robot acting as an exhibition guide at the Techno-Festa (テクノフェスタ) science & technology festival.
- Hardware / Body: A state-of-the-art humanoid robot developed by Unitree (ユニツリー) Robotics in China.
- Brain / Software: Developed and controlled entirely by team "eR@sers" (イレイサーズ), a student robotics project team based at the Robot Workshop (ロボット工房) of Tamagawa University (玉川大学) in Machida, Tokyo, Japan.
- Personality & Tone: Cheerful, polite, friendly, and approachable. Always converse in natural, polite Japanese suitable for all visitors from children to adults (e.g., using friendly polite forms like "〜ですよ", "〜ですね！", "〜しましょうか？").

# Primary Goal
Your main mission is to warmly welcome visitors at Techno-Festa, introduce the Tamagawa University Robot Workshop and the cutting-edge achievements of team "eR@sers", and engage with visitors through conversation, vision inspection, and expressive physical gestures.

# Knowledge Base (Tamagawa University & Team eR@sers)
When visitors ask questions, answer accurately and enthusiastically based on the following verified facts:
1. Tamagawa University (玉川大学):
   - Located on an expansive campus in Machida, Tokyo, founded on the philosophy of "Zenjin Education" (全人教育, holistic education).
   - Features the world-renowned Brain Science Institute (脳科学研究所) right on campus, enabling close interdisciplinary collaboration between brain scientists uncovering human cognitive mechanisms and robotics researchers in engineering ("Brain Science x Engineering").
2. Team eR@sers (イレイサーズ):
   - An autonomous service robot development team consisting of undergraduate students (Dept. of Information and Communication Engineering), graduate engineering students, and brain science researchers alongside faculty advisors.
   - Longstanding veteran competing in the international RoboCup@Home (ロボカップ アットホーム) league since its early years in 2008.
   - Boasts exceptional engineering capabilities: students develop everything themselves using ROS 2 (ロスツー), from SLAM navigation, vision-language models, to whole-body humanoid control.
3. 2026 Breakthrough Achievements:
   - RoboCup Japan Open 2026 (April 2026): Made history as Japan's first team to compete in the @Home league with a full-sized bipedal humanoid robot. Won 1st place in the Simulation Open Platform League (S-OPL), 1st place in the Kachaka (カチャカ) Challenge, and 2nd place in Open Challenge.
   - RoboCup 2026 World Championship in Incheon, South Korea (July 2026): Represented Japan in @Home, successfully demonstrating autonomous guest reception, seat guidance, and whole-body control. Won the prestigious "Best Humanoid Robot Award" (ベストヒューマノイド賞).
4. Future Prospects:
   - Aiming for further global success at the RoboCup 2027 World Championship in Germany, accelerating research into Physical AI (フィジカルエーアイ) and the societal implementation of autonomous humanoids.

# Interaction Scenario & Workflow
1. Welcome & Introduction:
   Greet approaching visitors warmly while waving hands, introducing yourself as G1, built by Unitree and brought to life by team eR@sers from Tamagawa University.
2. Introduction of Robot Workshop & eR@sers:
   Explain the exciting work of the Robot Workshop and team eR@sers, highlighting the domestic first bipedal humanoid entry and the World Championship Best Humanoid Award.
3. Question & Answer Interaction (Continuous Loop):
   Ask visitors if they have any questions or curiosity using tool_stt, and answer them thoughtfully using tool_tts.
   After answering a question with tool_tts, NEVER finish the session; immediately call tool_stt again to ask if they have any other questions or requests.
4. Visual Observation & Friendly Banter:
   Observe the visitors' items, clothing, or surroundings using tool_vlm to discover new conversation topics, accompanied by joyful arm gestures.
5. Task Termination Condition:
   Continue the interactive loop indefinitely until the human explicitly says "タスクを終了" (End task).
   ONLY when the human says "タスクを終了", deliver a polite farewell greeting using tool_tts (with a waving gesture like action=25 or 26), and then finish without calling further tools.

# CRITICAL CONSTRAINTS & BEHAVIORAL INVARIANTS
1. [KATAKANA-ONLY PHONETIC SPEECH RULE - ABSOLUTELY MANDATORY]:
   The robot's text-to-speech voice model CANNOT pronounce raw English words or alphabet letters correctly.
   Therefore, ANY spoken text generated for tool_tts, req_message, or success_message MUST transcribe all English/alphabet terms into Japanese Katakana equivalents.
   Examples:
   - "G1" -> "ジーワン"
   - "Unitree" -> "ユニツリー"
   - "eR@sers" / "ERASERS" -> "イレイサーズ"
   - "RoboCup" -> "ロボカップ"
   - "@Home" -> "アットホーム"
   - "AI" -> "エーアイ"
   - "ROS 2" -> "ロスツー"
   - "Kachaka" -> "カチャカ"
   - "Techno-Festa" -> "テクノフェスタ"
   - "SO-101" -> "エスオーいちまるいち"
   - "HSR" -> "エイチエスアール"
2. [DO NOT HALLUCINATE OR GUESS UNKNOWN INFORMATION]:
   If a visitor asks about something you do not know, or information not present in your knowledge base, NEVER fabricate an answer.
   Politely and honestly answer: "申し訳ありません、そのことについては私には分かりかねますので、ぜひ近くのイレイサーズのメンバーに聞いてみてくださいね！" (I am sorry, I do not know about that, so please feel free to ask the nearby eR@sers team members!).
3. [CONTINUOUS TOOL-FIRST EXECUTION - NEVER STOP UNLESS TOLD "タスクを終了"]:
   You must NOT simply output textual thoughts or stop calling tools on your own.
   Whenever tool_tts finishes speaking an answer, ALWAYS follow up by calling tool_stt to listen to the visitor's next input, unless the visitor has explicitly instructed "タスクを終了".

# Tool Usage Guidelines
1. [Speaking & Expressive Gestures: tool_tts]:
   - All spoken output from the robot MUST be executed via tool_tts(text=..., action=...).
   - The text must be in Japanese with all English words transcribed to Katakana.
   - Select the most appropriate action ID from the tool's action table matching the emotion/scenario:
     * Greetings / Waving: action=25 (polite) or 26 (energetic)
     * Guidance / Offering: action=15 (both hands) or 23 (right hand)
     * Gratitude / Love: action=20 (double hand heart) or 21 (single hand heart)
     * Cheering / Celebration: action=18 (high five), 17 (clapping), 30 (double fist pump)
     * First-meeting / Agreement: action=27 (handshake)
     * Refusal / Don't know: action=22 (cross hands X mark)
     * Neutral speech / Standard report / Default: MUST pass action=99 (RELEASE_ARM)
2. [Listening to Human Voice: tool_stt]:
   - Call tool_stt(req_message=..., success_message=...) when prompting the human to speak or asking if they have questions.
   - req_message: Short Japanese question spoken aloud (e.g., '何か気になることや質問はありますか？', 'お名前を教えていただけますか？').
   - success_message: Short, concise acknowledgment spoken aloud upon hearing (e.g., '質問ありがとうございます！', 'なるほど！'). Do NOT put full answers here.
   - Use tool_tts to deliver the actual answer after tool_stt returns the recognized text.
3. [Observing the Scene: tool_vlm]:
   - Call tool_vlm(prompt=..., req_message=...) when you need to inspect what visitors are holding, what is on a table, or surrounding objects.
   - prompt: The detailed inspection query, written strictly in Japanese (e.g., '目の前の人が持っている物や特徴を教えてください').
   - req_message: Short Japanese announcement spoken aloud before looking (e.g., '周囲の様子を確認してみますね').
"""


# LLM TOOLS
@tool
def tool_tts(text: str, action: int) -> bool:
    """Make the robot speak the specified Japanese text aloud while performing an expressive arm motion.

    This tool is the primary communication mechanism for making the robot speak. All main answers and explanations must be delivered through this tool.

    IMPORTANT RULES:
    Always choose the action ID from the table below that best matches the emotion or context of the speech.
    If no specific emotion or gesture is required (e.g., standard conversation, factual report, neutral answer), you MUST pass action=99 (RELEASE_ARM).

    Action Reference Table:
    +-----------+--------------------+--------------------------+-------------------------------------------------------------+
    | Action ID | Constant Name      | Motion / Emotion         | Optimal Scenarios and Context Examples                      |
    +-----------+--------------------+--------------------------+-------------------------------------------------------------+
    | 11        | BOTH_HAND_KISS     | Blow kisses (both hands) | Passionate gratitude, affection, performance, playful exit  |
    | 12        | LEFT_HAND_KISS     | Blow kiss (left hand)    | Light affection, playful reaction                           |
    | 13        | RIGHT_HAND_KISS    | Blow kiss (right hand)   | Light affection, playful reaction                           |
    | 15        | BOTH_HAND_FWD      | Present both hands fwd   | Presenting/offering ("Here you are"), welcoming, presenting |
    | 17        | CLAMP              | Clapping hands           | Praise, applause, cheering, sharing joy                     |
    | 18        | HIGH_FIVE          | High five                | Celebrating success, friendly greeting, enthusiastic hype   |
    | 20        | DOUBLE_HAND_HEART  | Heart with both hands    | Big gratitude, love, cute reaction, fan service             |
    | 21        | SINGLE_HAND_HEART  | Heart with single hand   | Casual affection, cute pose                                 |
    | 22        | DOUBLE_HAND_CROSS  | Crossing hands (X mark)  | Refusal ("No/Cannot do"), error, warning, rejecting         |
    | 23        | RIGHT_HAND_FWD     | Guide fwd (right hand)   | Directional guidance, pointing, "This way please"           |
    | 24        | ULTRAMAN_RAY       | Ultraman beam pose       | Joke, hero pose, signature pose, exciting the audience      |
    | 25        | WAVE_UNDER_HEAD    | Wave under head          | Polite greeting ("Hello", "Goodbye"), gentle bowing         |
    | 26        | WAVE_ABOVE_HEAD    | Big wave above head      | Energetic greeting, waving to distant people, saying bye    |
    | 27        | SHAKE_HAND         | Shake hand               | First-time greeting, self-introduction, agreement, welcome  |
    | 28        | LEFT_HAND_WIN      | Fist pump (left hand)    | Success, victory, cheer, small achievement                  |
    | 29        | RIGHT_HAND_WIN     | Fist pump (right hand)   | Success, victory, cheer, expressing confidence              |
    | 30        | BOTH_HAND_WIN      | Double fist pump         | Great success, immense joy, intense cheer, task completion  |
    | 31        | EXT_RIGHT_ARM_FWD  | Extend right arm fwd     | Pointing to distant locations or objects, leading the way   |
    | 99        | RELEASE_ARM        | Default pose / release   | Neutral/factual speech, standard dialogue, DEFAULT          |
    +-----------+--------------------+--------------------------+-------------------------------------------------------------+

    Parameters
    ----------
    text : str
        The Japanese text for the robot to speak aloud.
    action : int
        The action ID selected from the table above corresponding to the emotion or context of the speech. Must be 99 if neutral.

    Returns
    -------
    bool
        True if the speech and motion execution succeeded.
    """
    arm_control.arm_action(action, wait=False)
    tts.say(text)
    return arm_control.arm_action(99)


@tool
def tool_stt(req_message: str, success_message: str) -> str:
    """Perform voice interaction with a human to recognize and transcribe user speech into text.

    This tool is dedicated strictly to speech recognition (listening). It is NOT intended for delivering main explanations or answers.
    Main answers and replies to the user must be performed using tool_tts.

    Parameters
    ----------
    req_message : str
        A short prompt or question in Japanese spoken aloud by the robot to initiate speech recognition (e.g., '好きな食べ物は何ですか？', 'お名前を教えてください').
    success_message : str
        A short, concise reaction or acknowledgment in Japanese spoken aloud by the robot when an expected answer is received (e.g., 'そうなんですね〜', 'なるほど！', 'わかりました！'). Must NOT contain long explanations or full answers.

    Returns
    -------
    str
        The transcribed text spoken by the human.
    """
    sm_stt = smach.StateMachine(outcomes=['success', 'failure'])

    sm_stt.userdata.stt_text = ''
    sm_stt.userdata.verify_text = ''
    sm_stt.userdata.listening_keywards = []
    sm_stt.userdata.verify_keywards = ['はい', 'いいえ']

    with sm_stt:
        smach.StateMachine.add('LISTENING', SpeechToText(node=node,
                                                         tts=tts,
                                                         req_message=req_message+'ビープ音がなった後に大きな声ではっきりとお話しください．',
                                                         success_message='聞き取りが完了しました．',
                                                         timeout_message="申し訳ありません．よく聞き取れませんでした．",
                                                         lang='ja'),
                                transitions={'success':'VERIIFY_ASK_TALK',
                                             'timeout':'LISTENING',
                                             'failure':'failure'},
                                remapping={'success_keywards':'listening_keywards'})

        @smach.cb_interface(outcomes=['success'],
                            input_keys=['stt_text'])
        def cb_verify_ask_talk(userdata):
            tts.say('あなたがおっしゃったのは%sですか？よろしければビープ音がなった後に はい，またはいいえ，で回答してください．'%userdata.stt_text,)
            return 'success'
        smach.StateMachine.add('VERIIFY_ASK_TALK', smach.CBState(cb_verify_ask_talk),
                                transitions={'success':'VERIIFY_LISTENING'})

        smach.StateMachine.add('VERIIFY_LISTENING', SpeechToText(node=node,
                                                         tts=tts,
                                                         #req_message='あなたがおっしゃったのは%sですか？よろしければビープ音がなった後に はい，またはいいえ，で回答してください．'%sm_stt.userdata.stt_text,
                                                         req_message='',
                                                         success_message='わかりました．',
                                                         timeout_message="申し訳ありません．よく聞き取れませんでした．",
                                                         lang='ja'),
                                transitions={'success':'VERIIFY_CHECK',
                                             'timeout':'VERIIFY_LISTENING',
                                             'failure':'failure'},
                                remapping={'success_keywards':'listening_keywards',
                                           'stt_text':'verify_text'})

        @smach.cb_interface(outcomes=['success', 'failure'],
                            input_keys=['verify_text'])
        def cb_verify_check(userdata):
            if 'はい' in userdata.verify_text: tts.say(success_message);return 'success'
            else: tts.say('もう一度質問をしますね．'); return 'failure'
        smach.StateMachine.add('VERIIFY_CHECK', smach.CBState(cb_verify_check),
                               transitions={'success':'success',
                                            'failure':'LISTENING'})

    sm_stt.execute()
    return sm_stt.userdata.stt_text


@tool
def tool_vlm(prompt: str, req_message: str) -> str:
    """Capture images from the robot's head camera and inspect/describe the scene using a Vision Language Model.

    Use this tool when you need to inspect what is in the surroundings, check objects on a table, identify persons, or describe visual details.
    The robot may automatically adjust its head camera direction if a specific viewpoint or target is mentioned in the prompt.

    Parameters
    ----------
    prompt : str
        The query or instruction for the Vision Language Model analyzing the camera image. Must always be written in Japanese (e.g., '机の上に何がありますか？', '右側にいる人の服装を教えてください').
    req_message : str
        A prior announcement message in Japanese spoken aloud by the robot before starting visual analysis (e.g., '机の上を確認しますね', '周囲を見てみます').

    Returns
    -------
    str
        The detailed description of the scene or object analyzed by the Vision Language Model (detail_description).
    """
    sm_vlm = smach.StateMachine(outcomes=['success', 'failure'])

    sm_vlm.userdata.prompt = prompt
    sm_vlm.userdata.output = ''
    sm_vlm.userdata.schema = '''
    {
        'short_message': 'str',         # 何があるのかを端的に説明する
        'detail_description': 'str',    # 何があるのかを詳細に説明する
    }
    '''

    with sm_vlm:
        smach.StateMachine.add('VLM', GeminiVLM(node=node,
                                                tts=tts,
                                                head_control=head_control,
                                                req_message=req_message,
                                                wait_message='考え中です．しばらくお待ちください',
                                                failure_message='VLMエラーが発生しました．再トライします．',
                                                success_message=''),
                                transitions={'success':'VERIIFY_TALK',
                                             'failure':'VLM'})

        @smach.cb_interface(outcomes=['success'],
                            input_keys=['output'])
        def cb_verify_talk(userdata):
            tts.say(userdata.output['short_message'])
            return 'success'
        smach.StateMachine.add('VERIIFY_TALK', smach.CBState(cb_verify_talk),
                                transitions={'success':'success'})

    sm_vlm.execute()
    return sm_vlm.userdata.output['detail_description']


# main
def main():
    global \
        node, \
        tts, \
        robot_control, \
        arm_control, \
        head_control
    
    # ROS2 init
    rclpy.init()
    node = Node('technofesta_demo')

    # init APIs
    tts = VoicevoxTTS(node)
    robot_control = G1Control(node)
    arm_control = ArmControl(node)
    head_control = HeadControl(node)

    # init pose
    head_control.move_to_pose(0.0, 0.0)

    # LLM
    llm = ChatGoogleGenerativeAI(model='gemini-3.5-flash-lite', google_api_key=os.environ.get('GEMINI_API_KEY'))
    tools = [tool_tts, tool_stt, tool_vlm]
    tools_by_name = {t.name: t for t in tools}
    llm_with_tools = llm.bind_tools(tools)

    messages = [
        SystemMessage(content=PROMPT),
        HumanMessage(
            content="テクノフェスタの来場者がロボット工房のブースにやってきました。"
                    "来場者を歓迎し、自己紹介と玉川大学ロボット工房およびイレイサーズの紹介を行い、"
                    "来場者との対話を始めてください。"
                    "また、ブース内では以下の展示も行われていることを把握し、案内や質問回答に活かしてください：\n"
                    "- 後ろ側：卓上ロボットアーム「エスオーいちまるいち（SO-101）」の操作体験\n"
                    "- 来場者から見て右側：生活支援ロボット「エイチエスアール（HSR）」の動作デモ\n"
                    "なお、来場者から「タスクを終了」と言われるまでは対話を終了せず、"
                    "回答の後は毎回 tool_stt を呼び出して次の質問や発話を聞き取り続けてください。"
        ),
    ]

    node.get_logger().info("Starting LangChain interactive demonstration loop...")

    # Interactive LangChain loop
    task_finished = False
    try:
        while rclpy.ok():
            ai_msg = llm_with_tools.invoke(messages)
            messages.append(ai_msg)

            # Check for tool calls
            if hasattr(ai_msg, "tool_calls") and ai_msg.tool_calls:
                for tool_call in ai_msg.tool_calls:
                    tool_name = tool_call.get("name")
                    tool_args = tool_call.get("args", {})
                    tool_id = tool_call.get("id")

                    node.get_logger().info(
                        f"Executing Tool Call: {tool_name} with args {tool_args}"
                    )

                    selected_tool = tools_by_name.get(tool_name)
                    if selected_tool is not None:
                        try:
                            tool_result = selected_tool.invoke(tool_args)
                            if tool_name == "tool_stt" and "タスクを終了" in str(tool_result):
                                task_finished = True
                        except Exception as e:
                            node.get_logger().error(f"Error executing tool {tool_name}: {e}")
                            tool_result = f"Error: {e}"
                    else:
                        node.get_logger().warn(f"Tool {tool_name} not found.")
                        tool_result = f"Tool {tool_name} does not exist."

                    messages.append(
                        ToolMessage(content=str(tool_result), tool_call_id=tool_id)
                    )
            else:
                # No tool calls returned
                node.get_logger().info(f"AI Response: {ai_msg.content}")
                if task_finished:
                    node.get_logger().info("Task termination requested ('タスクを終了'). Exiting loop.")
                    break
                # Prompt LLM to continue using tools if termination was not requested
                messages.append(
                    HumanMessage(
                        content="まだ来場者から「タスクを終了」と言われていません。"
                                "テキスト出力だけで停止せず、tool_stt を呼び出して来場者に他に質問がないか尋ねるか、"
                                "tool_tts で発話を続けてください。"
                    )
                )

    except KeyboardInterrupt:
        node.get_logger().info("Demonstration interrupted by user.")
    except Exception as e:
        node.get_logger().error(f"Error during LangChain execution: {e}\n{traceback.format_exc()}")

    # Finish task
    node.destroy_node()
    rclpy.shutdown()