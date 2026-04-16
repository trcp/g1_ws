#!/usr/bin/env python3

# ROS2
from rclpy_util.util import TemporarySubscriber
from rclpy.node import Node
from rclpy import qos
import rclpy

# TF
from tf2_ros import Buffer, TransformListener

# interfaces
from sensor_msgs.msg import Image, CameraInfo
from std_srvs.srv import SetBool
from std_msgs.msg import Int16MultiArray
from lor_interfaces.msg import Person3D, Persons3D # Light Weight Open Pose

# erasers API
from erasers_g1_api.tts import TTS

# state machine
import smach

# vision
from cv_bridge import CvBridge
import cv2

# whisper
from faster_whisper import WhisperModel

# preferences
from typing import List
import numpy as np
import traceback
import time


'''
人物認識
'''
class LOR(smach.StateMachine):
    def __init__(self,
                 node:Node,
                 tts_say:TTS.say,
                 start_msg:str='I will search the person. Please wait a moment.',
                 timeout_msg:str='The person was not found.',
                 failure_msg:str='An error occurred while detecting the person.',
                 success_msg:str='The person was detected.',
                 scan_sec:int = 5,
                 timeout_sec:int=10,
                 detect_condition:str='normal'):
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

        userdata
        --------
        person_poses : List[Person3D]
            見つかった人物の3Dポーズ情報。成功時に出力される。
        """
        
        # init smach
        smach.State.__init__(self,
                             outcomes=['success', 'timeout', 'failure'],
                             input_keys=[],
                             output_keys=['person_poses'])
        
        # init values
        self.__node:Node = node
        self.__say:TTS.say = tts_say
        self.__start_msg:str = start_msg
        self.__timeout_msg:str = timeout_msg
        self.__failure_msg:str = failure_msg
        self.__success_msg:str = success_msg
        self.__scan_sec:int = scan_sec
        self.__timeout_sec:int = timeout_sec
        self.__detect_condition:str = detect_condition # normal, hand_up 
        self.__person_poses:List[Person3D] = []

        # service
        self.__lor_cli = self.__node.create_client(SetBool, 'execute_person_detect')
        while not self.__lor_cli.wait_for_service(timeout_sec=5.0):
            self.__node.get_logger().error('lightweight_openpose_ros2 not available')
            raise RuntimeError('lightweight_openpose_ros2 not available')
    
    def __send_lor_req(self, req:SetBool.Request):
        future = self.__lor_cli.call_async(req)
        rclpy.spin_until_future_complete(self.__node, future)
        response:SetBool.Response = future.result()
        self.__node.get_logger().info('lightweight_openpose_ros2 response: %s'%response.message)
        return response.success
    

    def __person_cb(self, msg:Persons3D):
        if self.__detect_condition == 'normal':
            self.__person_poses = msg.persons
        elif self.__detect_condition == 'hand_up':
            self.__person_poses = []
            for person in msg.persons:
                # 0:nose, 1:neck, 
                # 2:r_sho, 3:r_elb, 4:r_wri, 
                # 5:l_sho, 6:l_elb, 7:l_wri
                
                # Check Right Hand
                r_wri = person.keypoints[4]
                r_sho = person.keypoints[2]
                is_r_hand_up = r_wri.z > r_sho.z

                # Check Left Hand
                l_wri = person.keypoints[7]
                l_sho = person.keypoints[5]
                is_l_hand_up = l_wri.z > l_sho.z

                if is_r_hand_up or is_l_hand_up:
                    self.__person_poses.append(person)
    

    def execute(self, userdata):
        try:
            # declare msg
            self.__node.get_logger().info('''
                WAIT PERSON RECOGNITION ....
            ''')
            self.__person_poses:List[Person3D] = []

            # request mic start
            request = SetBool.Request()
            request.data = True
            if not self.__send_lor_req(request):
                self.__node.get_logger().error('lightweight_openpose_ros2 request failed')
                self.__say(self.__failure_msg)
                return 'failure'
            
            self.__node.get_logger().info('''
            =================================
                PERSON RECOGNITION START ...
            =================================
            ''')

            # detect person pose
            with TemporarySubscriber(self.__node,
                                    Persons3D,
                                    f'/human_3d_poses',
                                    10,
                                    self.__person_cb,
                                    ):
                # declare msg
                self.__node.get_logger().info('''
                    SEARCHING ...
                ''')
                self.__say(self.__start_msg)

                init_time = time.time()
                while (time.time() - init_time < self.__scan_sec) or \
                      (time.time() - init_time < self.__timeout_sec and not self.__person_poses):
                    rclpy.spin_once(self.__node, timeout_sec=0.1)
            
            # request mic stop
            request = SetBool.Request()
            request.data = False
            if not self.__send_lor_req(request):
                self.__node.get_logger().error('lightweight_openpose_ros2 request failed')
                self.__say(self.__failure_msg)
                return 'failure'
            else:
                print(self.__person_poses)
                userdata.person_poses = self.__person_poses
                if not self.__person_poses:
                    self.__node.get_logger().warn('Person is not detected')
                    self.__say(self.__timeout_msg)
                    return 'timeout'
                else:
                    self.__node.get_logger().info('Person is detected')
                    self.__say(self.__success_msg)
                    return 'success'
        except:
            # Ensure lor is stopped on error
            try:
                request = SetBool.Request()
                request.data = False
                self.__send_lor_req(request)
            except:
                pass
                
            self.__say('Error is occured in PersonRecongnition')
            self.__node.get_logger().error('Error is occured in PersonRecongnition\n%s'%traceback.format_exc())
            return 'failure'



'''
音声認識
'''
class SpeechToText(smach.State):
    def __init__(self,
                 node:Node,
                 tts_say:TTS.say,
                 timeout_sec:float=10.0,
                 start_msg:str='Please task for me.',
                 success_msg:str='I can hear! Please wait.',
                 timeout_msg:str='Sorry. I can not hear.',
                 device:str='cpu',
                 model_size:str='small',
                 lang:str='en',
                 success_keywards:list=[],
                 speech_threshold:float=1000.0,
                 silence_duration:float=1.5,
                 max_record_duration:float=10.0,
                 max_challenge:int=3):
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
        success_keywards : list, optional
            認識結果に含まれている必要があるキーワードのリスト、デフォルトは []。
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
        num_challenge : int
            既に試行した認識リトライ回数。状態はこの値を読み書きする。
        stt_text : str
            認識結果のテキスト。成功時に出力される。
        """
        
        # init smach
        smach.State.__init__(self,
                             outcomes=['success', 'timeout', 'failure'],
                             input_keys=['num_challenge'],
                             output_keys=['num_challenge', 'stt_text'])
        
        # init values
        self.__node:Node = node
        self.__say:TTS.say = tts_say
        self.__timeout_sec = timeout_sec
        self.__start_msg = start_msg
        self.__success_msg = success_msg
        self.__timeout_msg = timeout_msg
        self.__lang:str = lang
        self.__whisper_model = WhisperModel(model_size, device=device) # init whisper
        # VAD parameters
        self.__speech_threshold = speech_threshold # Adjust based on mic sensitivity
        self.__silence_duration = silence_duration # seconds
        self.__max_record_duration = max_record_duration # seconds
        self.__max_challenge = max_challenge
        self.__success_keywards = success_keywards
        self.__max_challenge = max_challenge

        # mic service
        self.__mic_cli = self.__node.create_client(SetBool, 'mic_rec')
        while not self.__mic_cli.wait_for_service(timeout_sec=1.0):
            self.__node.get_logger().error('mic_service not available')
            raise RuntimeError('mic_service not available')
    

    def __send_mic_req(self, req:SetBool.Request):
        future = self.__mic_cli.call_async(req)
        rclpy.spin_until_future_complete(self.__node, future)
        response:SetBool.Response = future.result()
        self.__node.get_logger().info('mic_service response: %s'%response.message)
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
                self.__node.get_logger().info("Speech started (RMS: {:.2f})".format(rms))
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
                self.__node.get_logger().warn('Voice recong challenge is %d times. Remaining %d times.'%(num_challenge, self.__max_challenge - num_challenge))

            # bringup mic
            self.__say(self.__start_msg)
            request = SetBool.Request()
            request.data = True
            if not self.__send_mic_req(request):
                self.__node.get_logger().error('mic_service request failed')
                self.__say(self.__failure_msg)
                return 'failure'
            self.__node.get_logger().info('''
            =================================
                VOICE RECOGNITION START
            =================================
            ''')
            self.__audio_buffer = []
            self.__speech_started = False
            self.__silence_start_time = None
            self.__recording_finished = False

            # Subscribe to audio
            qos_profile = qos.QoSProfile(depth=10)
            with TemporarySubscriber(self.__node,
                                    Int16MultiArray,
                                    '/audio/raw',
                                    qos_profile,
                                    self.__audio_cb):
                
                start_time = time.time()
                while not self.__recording_finished:
                    if time.time() - start_time > self.__max_record_duration:
                        self.__node.get_logger().warn("Max recording duration reached")
                        break
                    rclpy.spin_once(self.__node, timeout_sec=0.1)

            # Stop mic
            self.__node.get_logger().info('''
            =================================
                VOICE RECOGNITION STOP...
            =================================
            ''')
            request.data = False
            self.__send_mic_req(request)

            # detect voice
            if not self.__audio_buffer:
                userdata.num_challenge = num_challenge + 1
                if userdata.num_challenge >= self.__max_challenge:
                    self.__node.get_logger().error("Voice recong challenge is %d times. challenge is over."%(num_challenge))
                    self.__say(self.__failure_msg)
                    userdata.num_challenge = 0  # init challenge count
                    return 'failure'
                else:
                    self.__node.get_logger().warn("No audio data recorded")
                    self.__say(self.__timeout_msg)
                    return 'timeout'
            audio_np = np.array(self.__audio_buffer, dtype=np.float32)
            audio_np = audio_np / 32768.0
            # Input is already 16kHz from G1Mic, so no downsampling needed
            segments, info = self.__whisper_model.transcribe(audio_np, beam_size=5, language=self.__lang)
            text_result = ""
            for segment in segments:
                text_result += segment.text
            self.__node.get_logger().info(f"Detected text: {text_result}")

            if not text_result:
                userdata.num_challenge = num_challenge + 1
                if userdata.num_challenge >= self.__max_challenge:
                    self.__node.get_logger().error("Voice recong challenge is %d times. challenge is over."%(num_challenge))
                    self.__say(self.__failure_msg)
                    userdata.num_challenge = 0  # init challenge count
                    return 'failure'
                else:
                    self.__node.get_logger().warn("Recong text is empty.")
                    self.__say(self.__timeout_msg)
                    return 'timeout'

            # check keywords if provided
            if self.__success_keywards:
                 if not any(keyword in text_result.lower() for keyword in self.__success_keywards):
                    userdata.num_challenge = num_challenge + 1
                    if userdata.num_challenge >= self.__max_challenge:
                        self.__node.get_logger().error("Voice recong challenge is %d times. challenge is over."%(num_challenge))
                        self.__say(self.__failure_msg)
                        userdata.num_challenge = 0  # init challenge count
                        return 'failure'
                    else:
                        self.__node.get_logger().warn(f"Keywords not detected in: {text_result}")
                        self.__say(self.__timeout_msg)
                        return 'timeout'

            userdata.stt_text = text_result
            userdata.num_challenge = 0  # init challenge count
            self.__say(self.__success_msg)
            return 'success'

        except:
            # Ensure mic is stopped on error
            try:
                request = SetBool.Request()
                request.data = False
                self.__send_mic_req(request)
            except:
                pass

            self.__node.get_logger().error('Error is occured in SpeechToText\n%s'%traceback.format_exc())
            return 'failure'


'''
Depth 画像から平面上に置かれた物体を検出する．
'''
class SimpleObjectDetector(smach.State):
    def __init__(self,
                 node:Node,
                 tts_say:TTS.say,
                 timeout_sec:float=10.0,
                 start_msg:str='searching objects.',
                 timeout_msg:str='Sorry. I can not found objects.'
                 ):
        # init smach
        smach.State.__init__(self,
                             outcomes=['success', 'timeout', 'failure'],
                             input_keys=[],
                             output_keys=[])
        
        # init values
        self.node:Node = node
        self.tts_say = tts_say
        self.timeout_sec = timeout_sec
        self.start_msg = start_msg
        self.timeout_msg = timeout_msg
        self.cv_bridge = CvBridge()
        self.depth_camera_info:CameraInfo = None
    

    def processing_cb(self, msg:Image):
        cv_depth = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    

    def execute(self, userdata):
        DEPTH_IMAGE_TOPIC = '/head_camera/d455/depth/image_rect_raw'
        DEPTH_INFO_TOPIC = '/head_camera/d455/depth/camera_info'

        def camera_info_cb(msg:CameraInfo):
            self.depth_camera_info = msg

        try:
            # Subscribe depth camera info
            with TemporarySubscriber(node=self.node,
                                     msg=CameraInfo,
                                     topic=DEPTH_INFO_TOPIC,
                                     qos_profile=10,
                                     cb=camera_info_cb):
                while not self.depth_camera_info:
                    rclpy.spin_once(self.node, timeout_sec=0.1)
                self.node.get_logger().info('Get depth camera info.')
            
            # Subscribe depth camera info
            with TemporarySubscriber(node=self.node,
                                     msg=Image,
                                     topic=DEPTH_IMAGE_TOPIC,
                                     qos_profile=10,
                                     cb=self.processing_cb):
                self.tts_say(self.start_msg)
                self.node.get_logger().info('Seaching objects ...')
                it = time.time()
                while time.time() - it < self.timeout_sec:
                    rclpy.spin_once(self.node, timeout_sec=0.1)

            self.tts_say(self.timeout_msg)
            self.node.get_logger().warn('Objects is not found.')
            return 'timeout'
        except: 
            self.tts_say('Error is occured in SimpleObjectDetector', False)
            self.node.get_logger().error('Error is occured in SimpleObjectDetector\n%s'%traceback.format_exc())
            return 'failure'