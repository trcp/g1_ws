#!/usr/bin/env python3

# ROS2
import rclpy
from rclpy import qos
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy_util.util import TemporarySubscriber, TemporaryApproximateTimeSynchronizer
import smach

# interfaces
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Int16MultiArray
from std_srvs.srv import SetBool

# Vision
from cv_bridge import CvBridge
import cv2

# AI
from langchain_google_genai import ChatGoogleGenerativeAI
from langchain_core.tools import tool
from langchain_core.messages import HumanMessage, ToolMessage
from google.genai import types
from google import genai

# API
from ament_index_python.packages import get_package_share_directory
from faster_whisper import WhisperModel
from erasers_g1_api.tts import VoicevoxTTS, VuiTTS
from erasers_g1_api.robot_control import HeadControl

import os
import re
import json
import time
import traceback
import numpy as np
from typing import Any, List, Optional, Union


JA_PATTERN = re.compile(r'[\u3040-\u30ff\u4e00-\u9faf\uff00-\uffef]')


class GeminiVLM(smach.State):
    def __init__(
        self,
        node: Node,
        tts: Union[VoicevoxTTS, VuiTTS],
        head_control: HeadControl,
        req_message: str = "I will check it.",
        wait_message: str = "I am thinking. Please wait",
        success_message: str = "I found it.",
        failure_message: str = "Sorry. some error is occured.",
        model: str = "gemini-3.5-flash-lite",
        rgb_image_topic: str = "/head_camera/d455/color/image_raw",
        rgb_camera_info_topic: str = "/head_camera/d455/color/camera_info",
        depth_image_topic: str = "/head_camera/d455/aligned_depth_to_color/image_raw",
        depth_camera_info_topic: str = "/head_camera/d455/aligned_depth_to_color/camera_info",
        allow_head_control: bool = True,
    ):
        """Gemini VLM を用いた画像と言語によるマルチモーダル認識状態。

        Parameters
        ----------
        node : Node
            ログ出力や ROS 2 通信に使用するノードインスタンス。
        head_control : HeadControl
            ロボットの頭部制御を行うインスタンス。
        req_message : str, optional
            認識処理開始時に発話または通知するメッセージ、デフォルトは '画像を確認します．'。
        wait_message : str, optional
            推論待機中に発話または通知するメッセージ、デフォルトは '考え中です．しばらくお待ちください．'。
        success_message : str, optional
            認識処理成功時に発話または通知するメッセージ、デフォルトは '完了しました．'。
        failure_message : str, optional
            認識処理失敗時に発話または通知するメッセージ、デフォルトは '申し訳ありません．問題が発生しました．'。
        model : str, optional
            使用する Gemini のモデル名、デフォルトは 'gemini-2.5-flash'。
        rgb_image_topic : str, optional
            RGB 画像を取得するトピック名、デフォルトは '/head_camera/d455/color/image_raw'。
        rgb_camera_info_topic : str, optional
            RGB カメラ情報を取得するトピック名、デフォルトは '/head_camera/d455/color/camera_info'。
        depth_image_topic : str, optional
            深度画像を取得するトピック名、デフォルトは '/head_camera/d455/aligned_depth_to_color/image_raw'。
        depth_camera_info_topic : str, optional
            深度カメラ情報を取得するトピック名、デフォルトは '/head_camera/d455/aligned_depth_to_color/camera_info'。
        allow_head_control : bool, optional
            頭部制御を許可するかどうかのフラグ、デフォルトは True。

        Outcomes
        --------
        success :
            Gemini による認識およびレスポンス生成が成功した場合。
        failure :
            画像の取得失敗や API 呼び出しエラーなどが発生した場合。

        Userdata
        --------
        prompt : str
            Gemini に送信する指示プロンプト文字列。
        schema : str
            レスポンスの構造化スキーマ指定文字列。
        output : str
            Gemini からの応答テキストまたは構造化出力結果。
        """
        # init smach
        smach.State.__init__(
            self,
            outcomes=["success", "failure"],
            input_keys=["prompt", "schema"],
            output_keys=["output"],
        )

        # init values
        self.__node: Node = node
        self.__tts = tts
        self.__head_control: HeadControl = head_control
        self.__req_message: str = req_message
        self.__wait_message: str = wait_message
        self.__success_message: str = success_message
        self.__failure_message: str = failure_message
        self.__model: str = model
        self.__rgb_image_topic: str = rgb_image_topic
        self.__rgb_camera_info_topic: str = rgb_camera_info_topic
        self.__depth_image_topic: str = depth_image_topic
        self.__depth_camera_info_topic: str = depth_camera_info_topic
        self.__allow_head_control: bool = allow_head_control
        self.__bridge = CvBridge()

        # LLM
        self.__gemini_client = genai.Client(api_key=os.environ.get('GEMINI_API_KEY'))
        self.__gemini_llm = ChatGoogleGenerativeAI(model=model, google_api_key=os.environ.get('GEMINI_API_KEY'))

        @tool
        def tool_get_head_pose() -> List[float]:
            """Get the current head camera servo angles [pan, tilt] in radians.

            Returns
            -------
            List[float]
                Current servo angles [pan, tilt] in radians.
            """
            return self.__head_control.get_current_pose()

        @tool
        def tool_head_control(pan: float, tilt: float) -> bool:
            """Rotate the robot head camera to the specified pan and tilt angles in radians.

            Parameters
            ----------
            pan : float
                Horizontal pan angle in radians.
                Positive values turn right, negative values turn left, and 0.0 faces center.
                Valid range is approximately -1.57 to 1.57.
            tilt : float
                Vertical tilt angle in radians.
                Positive values tilt up, negative values tilt down (e.g., toward tables or floor), and 0.0 is horizontal.
                Valid range is approximately -1.57 to 1.57.

            Returns
            -------
            bool
                True if the movement request succeeded, False otherwise.
            """
            return self.__head_control.move_to_pose(pan=pan, tilt=tilt, wait=True)

        self.__tool_get_head_pose = tool_get_head_pose
        self.__tool_head_control = tool_head_control
        self.__llm_tools = self.__gemini_llm.bind_tools([self.__tool_get_head_pose, self.__tool_head_control])

        # images
        self.__rgb_image = None
        self.__depth_image = None
        self.__rgb_camera_info: Optional[CameraInfo] = None
        self.__depth_camera_info: Optional[CameraInfo] = None

    def __execute_langchain_task(self, prompt: str) -> None:
        """Analyze the user's task instruction via LangChain and adjust the head camera direction.

        Parameters
        ----------
        prompt : str
            The user task instruction.
        """
        head_prompt = """
You are an intelligent assistant controlling the head camera of a humanoid robot.
Analyze the user's task instruction and decide whether to adjust the head camera direction:
1. If the instruction specifies a direction, object, or location to look at (e.g., "table", "floor", "left", "right", "up", "down", "look at the person on the left"), call tool_head_control with appropriate pan and tilt angles in radians.
2. If the instruction explicitly requests to look forward/front/straight (e.g., "look straight", "正面を向いて", "前を見て", "face forward"), call tool_head_control(pan=0.0, tilt=0.0).
3. If no specific viewing direction or location is requested in the user instruction, do nothing (do not call any tool).

User task instruction: %s
        """ % prompt

        ai_msg = self.__llm_tools.invoke([HumanMessage(content=head_prompt)])
        if hasattr(ai_msg, "tool_calls") and ai_msg.tool_calls:
            for tool_call in ai_msg.tool_calls:
                name = tool_call.get("name")
                args = tool_call.get("args", {})
                self.__node.get_logger().info(
                    "HeadControl Tool Call: %s with args %s" % (name, args)
                )
                if name in ("tool_head_control", "__tool_head_control"):
                    pan = float(args.get("pan", 0.0))
                    tilt = float(args.get("tilt", 0.0))
                    self.__head_control.move_to_pose(pan=pan, tilt=tilt, wait=True)
                elif name in ("tool_get_head_pose", "__tool_get_head_pose"):
                    self.__head_control.get_current_pose()

    def __cb_caminfo(self, rgb_info_msg: CameraInfo, depth_info_msg: CameraInfo):
        self.__rgb_camera_info = rgb_info_msg
        self.__depth_camera_info = depth_info_msg

    def __cb_image(self, rgb_msg: Image, depth_msg: Image):
        self.__rgb_image = self.__bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        self.__depth_image = self.__bridge.imgmsg_to_cv2(
            depth_msg, desired_encoding="passthrough"
        )

    def execute(self, userdata):
        try:
            # init values
            self.__rgb_image = None
            self.__depth_image = None
            self.__rgb_camera_info = None
            self.__depth_camera_info = None
            
            # Type Check
            if not isinstance(userdata.prompt, str) or not userdata.prompt: raise ValueError('userdata prompt type is invalid. not str or empty.')
            if not isinstance(userdata.schema, str) or not userdata.schema: raise ValueError('userdata schema type is invalid. not str or empty.')

            # get userdata
            main_prompt:str = userdata.prompt
            schema:str = userdata.schema
            schema:str = json.dumps(schema)

            # langchain tasks
            self.__tts.say(self.__req_message)
            if self.__allow_head_control:
                self.__execute_langchain_task(main_prompt)

            # get image datas
            with (TemporaryApproximateTimeSynchronizer(self.__node,
                                                      [(CameraInfo, self.__rgb_camera_info_topic),
                                                      (CameraInfo, self.__depth_camera_info_topic)],
                                                      10, 0.1, self.__cb_caminfo,),
                TemporaryApproximateTimeSynchronizer(self.__node,
                                                     [(Image, self.__rgb_image_topic),
                                                     (Image, self.__depth_image_topic)],
                                                     10, 0.1, self.__cb_image)):
                it = time.time()
                while rclpy.ok() and (self.__rgb_image is None or self.__rgb_camera_info is None):
                    if time.time() - it > 10.0:
                        raise TimeoutError("Failed to receive camera images and camera info within timeout.")
                    rclpy.spin_once(self.__node, timeout_sec=0.1)

            s, e = cv2.imencode('.jpg', self.__rgb_image, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
            image_content = types.Part.from_bytes(data=e.tobytes(), mime_type='image/jpeg')

            # make prompt
            prompt = """
Task instruction: %s
Output format requirement: Output must be a valid JSON adhering to the specified JSON schema.
Expected JSON Schema: %s
            """ % (main_prompt, schema)

            # show description
            self.__node.get_logger().info("""
VLM PROMPT
===================
PROMPT:\n%s\n
SCHEMA:\n%s\n
            """%(main_prompt, json.dumps(schema, indent=2, ensure_ascii=False)))

            # Thinking
            self.__tts.say(self.__wait_message)
            response = self.__gemini_client.models.generate_content(
                model=self.__model,
                contents=[image_content, prompt],
                config=types.GenerateContentConfig(response_mime_type='application/json')
            )
            self.__node.get_logger().info("RECONG RESULT... \n%s"%response.text)

            userdata.output = json.loads(response.text)
            self.__tts.say(self.__success_message)
            return 'success'
        except Exception:
            self.__tts.say(self.__failure_message)
            self.__node.get_logger().error("Gemini VLM Failure.\n%s"%traceback.format_exc())
            return 'failure'


class SpeechToText(smach.State):
    def __init__(
        self,
        node: Node,
        tts: Union[VoicevoxTTS, VuiTTS],
        timeout_sec: float = 10.0,
        req_message: str = "Please task for me.",
        success_message: str = "I can hear! Please wait.",
        timeout_message: str = "Sorry. I can not hear.",
        device: str = "cuda",
        model_size: str = os.path.join(
            get_package_share_directory("erasers_g1_api"),
            "config",
            "faster-whisper-small",
        ),
        lang: str = "en",
        beep_sound_path: str = os.path.join(
            get_package_share_directory("erasers_g1_api"), "config", "req_sound.wav"
        ),
        speech_threshold: float = 1000.0,
        silence_duration: float = 1.5,
        max_record_duration: float = 10.0,
        max_challenge: int = 3,
    ):
        """Whisper と ROS マイク音声を使った音声認識状態。

        Parameters
        ----------
        node : Node
            サービス呼び出しとログ出力に使用する ROS ノードインスタンス。
        tts_say : TTS.say
            音声応答に使うテキスト読み上げ関数。
        timeout_sec : float, optional
            マイクサービスの待機や状態遷移のタイムアウト時間（秒）、デフォルトは 10.0。
        req_message : str, optional
            認識開始時に読み上げるメッセージ、デフォルトは 'Please task for me.'。
        success_message : str, optional
            認識成功時に読み上げるメッセージ、デフォルトは 'I can hear! Please wait.'。
        timeout_message : str, optional
            認識タイムアウト時に読み上げるメッセージ、デフォルトは 'Sorry. I can not hear.'。
        device : str, optional
            Whisper の実行デバイス、デフォルトは 'cpu'。
        model_size : str, optional
            Whisper モデルサイズ、デフォルトは 'small'。
        lang : str, optional
            音声認識に使用する言語コード、デフォルトは 'en'。
        speech_threshold : float, optional
            音声を検出する VAD の RMS 閾値、デフォルトは 1000.0。
        silence_duration : float, optional
            無音とみなすまでの継続時間（秒）、デフォルトは 1.5。
        max_record_duration : float, optional
            録音の最大継続時間（秒）、デフォルトは 10.0。
        max_challenge : int, optional
            失敗後にリトライする最大回数、デフォルトは 3。

        userdata
        --------
        Input Keys:
            success_keywards : list
                認識結果に含まれている必要があるキーワードのリスト。

        Output Keys:
            stt_text : str
                認識結果のテキスト。成功時に出力される。

        Outcomes:
        ----------
        success:
            音声認識が成功し、必要なキーワードが含まれている場合。
        timeout:
            音声が検出されなかった、または認識結果に必要なキーワードが含まれていなかった場合。リトライ可能。
        failure:
            認識処理中にエラーが発生した場合、またはリトライ回数が max_challenge を超えた場合。
        """

        # init smach
        smach.State.__init__(
            self,
            outcomes=["success", "timeout", "failure"],
            input_keys=["success_keywards"],
            output_keys=["stt_text"],
        )

        # init values
        self.__node: Node = node
        self.__tts: TTS = tts
        self.__timeout_sec = timeout_sec
        self.__beep_sound_path = beep_sound_path
        self.__req_message = req_message
        self.__success_message = success_message
        self.__timeout_message = timeout_message
        self.__lang: str = lang
        self.__whisper_model = WhisperModel(model_size, device=device)  # init whisper
        # VAD parameters
        self.__speech_threshold = speech_threshold  # Adjust based on mic sensitivity
        self.__silence_duration = silence_duration  # seconds
        self.__max_record_duration = max_record_duration  # seconds
        self.__max_challenge = max_challenge
        self.__num_challenge = 0

        # mic service
        self.__mic_cli = self.__node.create_client(SetBool, "mic_rec")
        while not self.__mic_cli.wait_for_service(timeout_sec=1.0):
            self.__node.get_logger().error("mic_service not available")
            raise RuntimeError("mic_service not available")

    def __send_mic_req(self, req: SetBool.Request):
        future = self.__mic_cli.call_async(req)
        rclpy.spin_until_future_complete(self.__node, future)
        response: SetBool.Response = future.result()
        self.__node.get_logger().info("mic_service response: %s" % response.message)
        return response.success

    def __audio_cb(self, msg: Int16MultiArray):
        if self.__recording_finished:
            return

        # append data
        self.__audio_buffer.extend(msg.data)

        # VAD check
        chunk_data = np.array(msg.data, dtype=np.float32)
        rms = np.sqrt(np.mean(chunk_data**2))

        if rms > self.__speech_threshold:
            if not self.__speech_started:
                self.__speech_started = True
                self.__node.get_logger().info(
                    "Speech started (RMS: {:.2f})".format(rms)
                )
            self.__silence_start_time = None
        elif self.__speech_started:
            if self.__silence_start_time is None:
                self.__silence_start_time = time.time()
            elif time.time() - self.__silence_start_time > self.__silence_duration:
                self.__recording_finished = True
                self.__node.get_logger().info("Silence detected, finishing recording")

    def __handle_retryable_failure(self, message: str) -> str:
        self.__num_challenge += 1
        if self.__num_challenge >= self.__max_challenge:
            self.__node.get_logger().error(
                "Voice recong challenge is %d times. challenge is over."
                % self.__num_challenge
            )
            self.__tts.say(self.__failure_msg)
            self.__num_challenge = 0
            return "failure"

        self.__node.get_logger().warn(message)
        self.__tts.say(self.__timeout_message)
        return "timeout"

    def execute(self, userdata):
        """SMACH state を実行し、音声認識結果を userdata に格納する。

        Parameters
        ----------
        userdata : smach.UserData
            stt_text に認識結果を格納する。

        Returns
        -------
        str
            SMACH outcome。'success'、'timeout'、'failure' のいずれか。
        """
        try:
            if self.__num_challenge > 0:
                self.__node.get_logger().warn(
                    "Voice recong challenge is %d times. Remaining %d times."
                    % (
                        self.__num_challenge,
                        self.__max_challenge - self.__num_challenge,
                    )
                )

            # bringup mic
            self.__tts.say(text=self.__req_message)
            request = SetBool.Request()
            request.data = True
            if not self.__send_mic_req(request):
                self.__node.get_logger().error("mic_service request failed")
                self.__tts.say(self.__failure_msg)
                return "failure"
            time.sleep(10)
            self.__tts.audio(self.__beep_sound_path, wait=False)
            self.__node.get_logger().info("""
            =================================
                VOICE RECOGNITION START
            =================================
            """)
            self.__audio_buffer = []
            self.__speech_started = False
            self.__silence_start_time = None
            self.__recording_finished = False

            # Subscribe to audio
            qos_profile = qos.QoSProfile(depth=10)
            with TemporarySubscriber(
                self.__node, Int16MultiArray, "/audio/raw", qos_profile, self.__audio_cb
            ):
                start_time = time.time()
                while not self.__recording_finished:
                    if time.time() - start_time > self.__max_record_duration:
                        self.__node.get_logger().warn("Max recording duration reached")
                        break
                    rclpy.spin_once(self.__node, timeout_sec=0.1)

            # Stop mic
            self.__node.get_logger().info("""
            =================================
                VOICE RECOGNITION STOP...
            =================================
            """)
            request.data = False
            self.__send_mic_req(request)

            # detect voice
            if not self.__audio_buffer:
                return self.__handle_retryable_failure("No audio data recorded")
            audio_np = np.array(self.__audio_buffer, dtype=np.float32)
            audio_np = audio_np / 32768.0
            # Input is already 16kHz from G1Mic, so no downsampling needed
            segments, info = self.__whisper_model.transcribe(
                audio_np, beam_size=5, language=self.__lang
            )
            text_result = ""
            for segment in segments:
                text_result += segment.text
            self.__node.get_logger().info(f"Detected text: {text_result}")

            if not text_result:
                return self.__handle_retryable_failure("Recong text is empty.")

            # check keywords if provided
            if userdata.success_keywards:
                if not any(
                    keyword in text_result.lower()
                    for keyword in userdata.success_keywards
                ):
                    return self.__handle_retryable_failure(
                        f"Keywords not detected in: {text_result}"
                    )

            userdata.stt_text = text_result
            self.__num_challenge = 0
            self.__tts.say(self.__success_message)
            return "success"

        except:
            # Ensure mic is stopped on error
            try:
                request = SetBool.Request()
                request.data = False
                self.__send_mic_req(request)
            except:
                pass

            self.__node.get_logger().error(
                "Error is occured in SpeechToText\n%s" % traceback.format_exc()
            )
            return "failure"
