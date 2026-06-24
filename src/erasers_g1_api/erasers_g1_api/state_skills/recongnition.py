#!/usr/bin/env python3

# ROS2
from rclpy_util.util import TemporarySubscriber, TemporaryApproximateTimeSynchronizer
from rclpy.node import Node
from rclpy import qos
import rclpy

# TF
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs

# interfaces
from sensor_msgs.msg import Image, CameraInfo
from std_srvs.srv import SetBool
from std_msgs.msg import Header, Int16MultiArray
from geometry_msgs.msg import PoseArray, PoseStamped
from lor_interfaces.msg import Person3D, Persons3D  # Light Weight Open Pose
from sam3_ros_interfaces.msg import PredictArray, Predict
from sam3_ros_interfaces.srv import ExecPredict

# erasers API
from erasers_g1_api.tts import TTS
from erasers_g1_api.robot_control import G1Navigation, G1Control, Grasp

# state machine
import smach

# vision
from cv_bridge import CvBridge
import cv2

# whisper
from faster_whisper import WhisperModel

# preferences
from ament_index_python.packages import get_package_share_directory
from typing import List
import numpy as np
import traceback
import time
import copy
import os


"""
人物認識
"""


class LOR(smach.State):
    def __init__(
        self,
        node: Node,
        tts_say: TTS.say,
        robot_control: G1Control,
        start_msg: str = "I will search the person. Please wait a moment.",
        timeout_msg: str = "The person was not found.",
        failure_msg: str = "An error occurred while detecting the person.",
        success_msg: str = "The person was detected.",
        scan_sec: int = 3,
        timeout_sec: int = 5,
        detect_condition: str = "normal",
        hand_up_margin: float = 0.05,
        searching_area: List[float] = [[0.0, 0.0], [0.0, 0.0]],
        number_of_searching: int = 3,
        person_pose_array_topic: str = "detected_person_poses",
    ):
        """人物検出。

        Parameters
        ----------
        node : Node
            サービス呼び出しとログ出力に使用する ROS ノードインスタンス。
        tts_say : TTS.say
            音声応答に使うテキスト読み上げ関数。
        start_msg : str, optional
            人物検出開始時に読み上げるメッセージ。
        timeout_msg : str, optional
            人物が見つからなかった場合に読み上げるメッセージ。
        failure_msg : str, optional
            検出処理中にエラーが発生した場合に読み上げるメッセージ。
        success_msg : str, optional
            人物検出成功時に読み上げるメッセージ。
        scan_sec : int, optional
            検出処理を実行するスキャン時間（秒）。
        timeout_sec : int, optional
            検出処理の最大待機時間（秒）。
        detect_condition : str, optional
            検出条件。'normal' または 'hand_up' を指定可能。
        hand_up_margin : float, optional
            手を挙げたと判定する手首と肩の高さ差の最小値(m)。
        searching_area : List[List[float, float], List[float, float]], optional
            頭部カメラの探索範囲。[[tilt_min, pan_min], [tilt_max, pan_max]] の形式で指定。
        number_of_searching : int, optional
            探索ポイントの数。探索範囲内でこの数だけ頭部カメラを移動させて検出を試みる。
        person_pose_array_topic : str, optional
            detect_condition に合致した人物の位置姿勢を PoseArray で publish するトピック名。

        userdata
        --------
        Output Keys:
            person_poses : List[Person3D]
                見つかった人物の3Dポーズ情報。成功時に出力される。
            person_poses_header : Header
                person_poses の基準座標系とタイムスタンプ。

        Outcomes
        ----------
        success:
            人物が検出された場合。
        timeout:
            人物が見つからなかった場合。
        failure:
            検出処理中にエラーが発生した場合。
        """

        # init smach
        smach.State.__init__(
            self,
            outcomes=["success", "timeout", "failure"],
            input_keys=[],
            output_keys=["person_poses", "person_poses_header"],
        )

        # init values
        self.__node: Node = node
        self.__say: TTS.say = tts_say
        self.__robot_control: G1Control = robot_control
        self.__start_msg: str = start_msg
        self.__timeout_msg: str = timeout_msg
        self.__failure_msg: str = failure_msg
        self.__success_msg: str = success_msg
        self.__scan_sec: int = scan_sec
        self.__timeout_sec: int = timeout_sec
        self.__detect_condition: str = detect_condition  # normal, hand_up
        self.__hand_up_margin: float = hand_up_margin
        self.__searching_area: List[List[float, float], List[float, float]] = (
            searching_area  # [[tilt_min, pan_min], [tilt_max, pan_max]]
        )
        self.__number_of_searching: int = number_of_searching
        self.__person_poses: List[Person3D] = []
        self.__person_poses_header: Header = Header()
        self.__person_pose_array_pub = self.__node.create_publisher(
            PoseArray, person_pose_array_topic, 10
        )

        # TF2 Setup
        self.__tf_buffer = Buffer()
        self.__tf_listener = TransformListener(self.__tf_buffer, self.__node)

        # service
        self.__lor_cli = self.__node.create_client(SetBool, "execute_person_detect")
        while not self.__lor_cli.wait_for_service(timeout_sec=5.0):
            self.__node.get_logger().error("lightweight_openpose_ros2 not available")
            raise RuntimeError("lightweight_openpose_ros2 not available")

    def __send_lor_req(self, req: SetBool.Request):
        future = self.__lor_cli.call_async(req)
        rclpy.spin_until_future_complete(self.__node, future)
        response: SetBool.Response = future.result()
        self.__node.get_logger().info(
            "lightweight_openpose_ros2 response: %s" % response.message
        )
        return response.success

    @staticmethod
    def __is_valid_keypoint(point) -> bool:
        values = [point.x, point.y, point.z]
        return bool(np.all(np.isfinite(values))) and any(
            abs(v) > 1e-6 for v in values
        )

    def __is_hand_up(
        self, person: Person3D, wrist_index: int, shoulder_index: int
    ) -> bool:
        try:
            wrist = person.keypoints[wrist_index]
            shoulder = person.keypoints[shoulder_index]
        except IndexError:
            return False

        if not (
            self.__is_valid_keypoint(wrist)
            and self.__is_valid_keypoint(shoulder)
        ):
            return False

        return wrist.z > shoulder.z + self.__hand_up_margin

    def __transform_person_poses_to_map(
        self, persons: List[Person3D], header: Header
    ) -> bool:
        if not persons:
            self.__person_poses = []
            self.__person_poses_header = Header()
            return True

        if not header.frame_id:
            self.__node.get_logger().warn(
                "Person pose header frame_id is empty. Dropping detected persons."
            )
            self.__person_poses = []
            self.__person_poses_header = Header()
            return False

        map_header = Header()
        map_header.stamp = header.stamp
        map_header.frame_id = "map"

        if header.frame_id == "map":
            self.__person_poses = [copy.deepcopy(person) for person in persons]
            self.__person_poses_header = map_header
            return True

        try:
            transform = self.__tf_buffer.lookup_transform(
                "map",
                header.frame_id,
                rclpy.time.Time.from_msg(header.stamp),
                rclpy.duration.Duration(seconds=1.0),
            )
        except Exception as e:
            self.__node.get_logger().warn(
                "Failed to transform person poses from %s to map: %s"
                % (header.frame_id, str(e))
            )
            self.__person_poses = []
            self.__person_poses_header = Header()
            return False

        transformed_persons: List[Person3D] = []
        for person in persons:
            pose_stamped = PoseStamped()
            pose_stamped.header = header
            pose_stamped.pose = person.pose

            try:
                pose_transformed = tf2_geometry_msgs.do_transform_pose_stamped(
                    pose_stamped, transform
                )
            except Exception as e:
                self.__node.get_logger().warn(
                    "Failed to transform a person pose from %s to map: %s"
                    % (header.frame_id, str(e))
                )
                self.__person_poses = []
                self.__person_poses_header = Header()
                return False

            person_transformed = copy.deepcopy(person)
            person_transformed.pose = pose_transformed.pose
            transformed_persons.append(person_transformed)

        self.__person_poses = transformed_persons
        self.__person_poses_header = map_header
        return True

    def __person_cb(self, msg: Persons3D):
        print(msg)
        filtered_person_poses: List[Person3D] = []

        if self.__detect_condition == "normal":
            filtered_person_poses = list(msg.persons)
        elif self.__detect_condition == "hand_up":
            for person in msg.persons:
                # 0:nose, 1:neck,
                # 2:r_sho, 3:r_elb, 4:r_wri,
                # 5:l_sho, 6:l_elb, 7:l_wri

                is_r_hand_up = self.__is_hand_up(
                    person, wrist_index=4, shoulder_index=2
                )
                is_l_hand_up = self.__is_hand_up(
                    person, wrist_index=7, shoulder_index=5
                )

                if is_r_hand_up or is_l_hand_up:
                    filtered_person_poses.append(person)
        else:
            self.__node.get_logger().warn(
                "Unsupported detect_condition: %s" % self.__detect_condition
            )

        self.__transform_person_poses_to_map(filtered_person_poses, msg.header)

        pose_array = PoseArray()
        pose_array.header = self.__person_poses_header
        pose_array.poses = [person.pose for person in self.__person_poses]
        self.__person_pose_array_pub.publish(pose_array)

    def execute(self, userdata):
        try:
            # declare msg
            self.__node.get_logger().info("""
                WAIT PERSON RECOGNITION ....
            """)
            self.__person_poses: List[Person3D] = []
            self.__person_poses_header = Header()

            # 探索時にロボットの頭部カメラを旋回させるポイントを作成
            searching_points = np.linspace(
                self.__searching_area[0],
                self.__searching_area[1],
                self.__number_of_searching,
            ).tolist()

            # request mic start
            request = SetBool.Request()
            request.data = True
            if not self.__send_lor_req(request):
                self.__node.get_logger().error(
                    "lightweight_openpose_ros2 request failed"
                )
                self.__say(text=self.__failure_msg)
                return "failure"

            self.__node.get_logger().info("""
            =================================
                PERSON RECOGNITION START ...
            =================================
            """)

            for searching_point in searching_points:
                # detect person pose
                self.__robot_control.move_head(
                    tilt=searching_point[0], pan=searching_point[1]
                )
                with TemporarySubscriber(
                    self.__node,
                    Persons3D,
                    f"/human_3d_poses",
                    10,
                    self.__person_cb,
                ):
                    # declare msg
                    self.__node.get_logger().info("""
                        SEARCHING ...
                    """)
                    self.__say(self.__start_msg)

                    init_time = time.time()
                    while (time.time() - init_time < self.__scan_sec) or (
                        time.time() - init_time < self.__timeout_sec
                        and not self.__person_poses
                    ):
                        rclpy.spin_once(self.__node, timeout_sec=0.1)
                    if self.__person_poses:
                        break

            # request mic stop
            self.__robot_control.move_head(tilt=0.0, pan=0.0)  # move head to front
            request = SetBool.Request()
            request.data = False
            if not self.__send_lor_req(request):
                self.__node.get_logger().error(
                    "lightweight_openpose_ros2 request failed"
                )
                self.__say(self.__failure_msg)
                return "failure"
            else:
                print(self.__person_poses)
                userdata.person_poses = self.__person_poses
                userdata.person_poses_header = self.__person_poses_header
                if not self.__person_poses:
                    self.__node.get_logger().warn("Person is not detected")
                    self.__say(text=self.__timeout_msg)
                    return "timeout"
                else:
                    self.__node.get_logger().info("Person is detected")
                    self.__say(text=self.__success_msg)
                    return "success"
        except:
            # Ensure lor is stopped on error
            try:
                request = SetBool.Request()
                request.data = False
                self.__send_lor_req(request)
            except:
                pass

            self.__say(text="Error is occured in PersonRecongnition")
            self.__node.get_logger().error(
                "Error is occured in PersonRecongnition\n%s" % traceback.format_exc()
            )
            return "failure"


"""
音声認識
"""


class SpeechToText(smach.State):
    def __init__(
        self,
        node: Node,
        tts: TTS,
        timeout_sec: float = 10.0,
        start_msg: str = "Please task for me.",
        success_msg: str = "I can hear! Please wait.",
        timeout_msg: str = "Sorry. I can not hear.",
        device: str = "cpu",
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
        start_msg : str, optional
            認識開始時に読み上げるメッセージ、デフォルトは 'Please task for me.'。
        success_msg : str, optional
            認識成功時に読み上げるメッセージ、デフォルトは 'I can hear! Please wait.'。
        timeout_msg : str, optional
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
            num_challenge : int
                既に試行した認識リトライ回数。状態はこの値を読み書きする。
            success_keywards : list
                認識結果に含まれている必要があるキーワードのリスト。

        Output Keys:
            num_challenge : int
                更新された認識リトライ回数。失敗やタイムアウト時にインクリメントされ、成功時にリセットされる。
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
            input_keys=["num_challenge", "success_keywards"],
            output_keys=["num_challenge", "stt_text"],
        )

        # init values
        self.__node: Node = node
        self.__tts: TTS = tts
        self.__timeout_sec = timeout_sec
        self.__beep_sound_path = beep_sound_path
        self.__start_msg = start_msg
        self.__success_msg = success_msg
        self.__timeout_msg = timeout_msg
        self.__lang: str = lang
        self.__whisper_model = WhisperModel(model_size, device=device)  # init whisper
        # VAD parameters
        self.__speech_threshold = speech_threshold  # Adjust based on mic sensitivity
        self.__silence_duration = silence_duration  # seconds
        self.__max_record_duration = max_record_duration  # seconds
        self.__max_challenge = max_challenge
        self.__max_challenge = max_challenge

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

    def execute(self, userdata):
        try:
            num_challenge = userdata.num_challenge
            if num_challenge > 0:
                self.__node.get_logger().warn(
                    "Voice recong challenge is %d times. Remaining %d times."
                    % (num_challenge, self.__max_challenge - num_challenge)
                )

            # bringup mic
            self.__tts.say(text=self.__start_msg)
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
                userdata.num_challenge = num_challenge + 1
                if userdata.num_challenge >= self.__max_challenge:
                    self.__node.get_logger().error(
                        "Voice recong challenge is %d times. challenge is over."
                        % (num_challenge)
                    )
                    self.__tts.say(self.__failure_msg)
                    userdata.num_challenge = 0  # init challenge count
                    return "failure"
                else:
                    self.__node.get_logger().warn("No audio data recorded")
                    self.__tts.say(self.__timeout_msg)
                    return "timeout"
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
                userdata.num_challenge = num_challenge + 1
                if userdata.num_challenge >= self.__max_challenge:
                    self.__node.get_logger().error(
                        "Voice recong challenge is %d times. challenge is over."
                        % (num_challenge)
                    )
                    self.__tts.say(self.__failure_msg)
                    userdata.num_challenge = 0  # init challenge count
                    return "failure"
                else:
                    self.__node.get_logger().warn("Recong text is empty.")
                    self.__tts.say(self.__timeout_msg)
                    return "timeout"

            # check keywords if provided
            if userdata.success_keywards:
                if not any(
                    keyword in text_result.lower()
                    for keyword in userdata.success_keywards
                ):
                    userdata.num_challenge = num_challenge + 1
                    if userdata.num_challenge >= self.__max_challenge:
                        self.__node.get_logger().error(
                            "Voice recong challenge is %d times. challenge is over."
                            % (num_challenge)
                        )
                        self.__tts.say(self.__failure_msg)
                        userdata.num_challenge = 0  # init challenge count
                        return "failure"
                    else:
                        self.__node.get_logger().warn(
                            f"Keywords not detected in: {text_result}"
                        )
                        self.__tts.say(self.__timeout_msg)
                        return "timeout"

            userdata.stt_text = text_result
            userdata.num_challenge = 0  # init challenge count
            self.__tts.say(self.__success_msg)
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


"""
SAM3 を用いた物体検出
"""


class Sam3ObjectDetector(smach.State):
    def __init__(
        self,
        node: Node,
        tts_say: TTS.say,
        robot_control: G1Control,
        arm_control: Grasp,
        timeout_sec: float = 10.0,
        start_msg: str = "searching objects.",
        success_msg: str = "I found objects.",
        timeout_msg: str = "Sorry. I can not found objects.",
    ):
        """
        SAM3 ROS を使った物体認識状態。

        Parameters
        ----------
        node : Node
            ROS2ノードインスタンス。
        tts_say : TTS.say
            TTS（Text-to-Speech）の発話関数。
        robot_control : G1Control
            ロボット制御インスタンス。
        arm_control : Grasp
            アーム制御インスタンス。
        timeout_sec : float, optional
            認識のタイムアウト時間（秒）, by default 10.0
        start_msg : str, optional
            認識開始時の発話メッセージ, by default 'searching objects.'
        success_msg : str, optional
            認識成功時の発話メッセージ, by default 'I found objects.'
        timeout_msg : str, optional
            タイムアウト時の発話メッセージ, by default 'Sorry. I can not found objects.'

        Userdata
        --------
        Input Keys:
            objects_dict : dict
                物体名をキー、認識信頼度を値とする辞書。
                例: {"banana": 0.5, "apple": 0.8}

        Output Keys:
            object_poses_dict_list : list of dict
                検出された物体の情報リスト。
                各要素は以下の形式:
                {
                    'name': str,
                    'pose': {'ref_frame': str, 'xyz': list, 'rpy': list},
                    'conf': float,
                    'shape': str,
                    'size': list,
                    'grasp_approach': str
                }

        Outcomes
        --------
        success : str
            物体認識に成功した場合。
        timeout : str
            物体が見つからなかった、またはタイムアウトした場合。
        failure : str
            エラーが発生した場合。
        """
        # init smach
        smach.State.__init__(
            self,
            outcomes=["success", "timeout", "failure"],
            input_keys=["objects_dict"],
            output_keys=["object_poses_dict_list"],
        )

        # init values
        self.__node: Node = node
        self.__tts_say = tts_say
        self.__robot_control: G1Control = robot_control
        self.__arm_control: Grasp = arm_control
        self.__timeout_sec = timeout_sec
        self.__start_msg = start_msg
        self.__success_msg = success_msg
        self.__timeout_msg = timeout_msg
        self.__object_poses_dict_list = []
        self.__target_conf_map = {}

        # init sam3 service client
        self.__sam3_service_client = self.__node.create_client(
            ExecPredict, "/sam3/exec_predict"
        )
        while not self.__sam3_service_client.wait_for_service(timeout_sec=1.0):
            self.__node.get_logger().error(
                "sam3_ros service not available, waiting again..."
            )
            break

    def __send_sam3_request(self, execute: bool, objects_dict: dict):
        object_name_list = list(objects_dict.keys())
        object_conf_list = list(objects_dict.values())

        req = ExecPredict.Request()
        req.prompts = object_name_list
        req.conf = min(object_conf_list)
        req.execute = execute

        future = self.__sam3_service_client.call_async(req)
        rclpy.spin_until_future_complete(self.__node, future)
        if future.result() is None:
            self.__node.get_logger().error("Service call failed")
            raise RuntimeError("Service call failed")
        result = future.result()
        if result.success:
            self.__node.get_logger().info(
                "Object recognition successful"
            ) if execute else self.__node.get_logger().info("Object recognition stop.")
        else:
            self.__node.get_logger().error("Object recognition failed")
            raise RuntimeError("Object recognition failed")
        return result

    def pose_callback(
        self, predict_msg: PredictArray, depth_msg: Image, camera_info_msg: CameraInfo
    ):
        """
        SAM3の予測結果から物体の3Dポーズを算出し、リストに保存するコールバック関数。

        Parameters
        ----------
        predict_msg : PredictArray
            物体検出結果のメッセージ。
        depth_msg : Image
            深度画像メッセージ。
        camera_info_msg : CameraInfo
            カメラ情報メッセージ。
        """
        self.__object_poses_dict_list = []
        try:
            for predict in predict_msg.predicts:
                label = predict.label
                conf = predict.conf

                # Confidence フィルタ
                if label in self.__target_conf_map:
                    if conf < self.__target_conf_map[label]:
                        continue

                pose_stamped = predict.pose
                try:
                    tf_buffer = self.__arm_control.arm.tf_buffer
                    transform = tf_buffer.lookup_transform(
                        "base_link", pose_stamped.header.frame_id, rclpy.time.Time()
                    )

                    pose_transformed = tf2_geometry_msgs.do_transform_pose(
                        pose_stamped.pose, transform
                    )
                    xyz = [
                        pose_transformed.position.x,
                        pose_transformed.position.y,
                        pose_transformed.position.z,
                    ]

                    from scipy.spatial.transform import Rotation as R

                    quat = [
                        pose_transformed.orientation.x,
                        pose_transformed.orientation.y,
                        pose_transformed.orientation.z,
                        pose_transformed.orientation.w,
                    ]
                    rpy = R.from_quat(quat).as_euler("xyz").tolist()
                except Exception as e:
                    self.__node.get_logger().warn(
                        f"TF Transform failed for {label}: {e}"
                    )
                    continue

                size = [predict.size.x, predict.size.y, predict.size.z]

                self.__object_poses_dict_list.append(
                    {
                        "name": label,
                        "pose": {"ref_frame": "base_link", "xyz": xyz, "rpy": rpy},
                        "conf": float(conf),
                        "shape": "box",
                        "size": size,
                        "grasp_approach": "top",
                    }
                )
        except Exception as e:
            self.__node.get_logger().error(f"Error in pose_callback: {e}")

    def execute(self, userdata):
        """
        ステートの実行メソッド。

        Parameters
        ----------
        userdata : smach.user_data.Remapper
            共有データを管理する機構。

        Returns
        -------
        str
            'success', 'timeout', または 'failure'。
        """
        try:
            self.__arm_control.collision.clear_all()
            self.__robot_control.move_head(
                tilt=-0.5
            )  # stop robot before object detection
            self.__node.get_logger().info("""
            =================================
                OBJECT RECOGNITION START
            =================================
            """)
            self.__tts_say(self.__start_msg)
            print(userdata.objects_dict)

            self.__target_conf_map = userdata.objects_dict
            self.__send_sam3_request(execute=True, objects_dict=userdata.objects_dict)

            self.__object_poses_dict_list = []

            with TemporaryApproximateTimeSynchronizer(
                node=self.__node,
                sub_topics=[
                    (PredictArray, "/sam3/predicts"),
                    (Image, "/head_camera/d455/aligned_depth_to_color/image_raw"),
                    (
                        CameraInfo,
                        "/head_camera/d455/aligned_depth_to_color/camera_info",
                    ),
                ],
                qos_profile=10,
                slop=0.1,
                callback=self.pose_callback,
            ):
                it = time.time()
                while time.time() - it < self.__timeout_sec:
                    rclpy.spin_once(self.__node, timeout_sec=0.1)
                    if self.__object_poses_dict_list:
                        break

            self.__send_sam3_request(execute=False, objects_dict=userdata.objects_dict)

            if self.__object_poses_dict_list:
                userdata.object_poses_dict_list = self.__object_poses_dict_list
                self.__tts_say(self.__success_msg)
                import json

                self.__node.get_logger().info(
                    f"Detected objects: {json.dumps(self.__object_poses_dict_list)}"
                )
                return "success"
            else:
                self.__tts_say(self.__timeout_msg)
                return "timeout"

        except:
            self.__tts_say("Error is occured in SimpleObjectDetector", False)
            self.__node.get_logger().error(
                "Error is occured in SimpleObjectDetector\\n%s" % traceback.format_exc()
            )
            return "failure"
