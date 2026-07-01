#!/usr/bin/env python3

# ROS2
from rclpy_util.util import TemporarySubscriber
from rclpy.node import Node
import rclpy
import smach

# interfaces
from sensor_msgs.msg import Image

# OpenCV
from PIL import Image as PILImage
from cv_bridge import CvBridge
import cv2

# erasers API
from erasers_g1_api.tts import TTS

# Google Gemini
from google import genai
from google.genai import types

# preferences
from typing import List
import numpy as np
import traceback
import os


MODEL_ID = "gemini-robotics-er-1.6-preview"


class GeminiVLMState(smach.State):
    def __init__(self,
                 node: Node,
                 model_id: str = MODEL_ID):
        
        smach.State.__init__(self,
                             outcomes=['success', 'failure'],
                             input_keys=['prompt_message', 'ud_prompt'],
                             output_keys=['result'])
        
        self.__node = node
        self.__model_id = model_id
        self.__cv_bridge = CvBridge()
        self.__image = None
        self.__gemini_client = genai.Client(api_key=os.environ.get("GEMINI_API_KEY"))
    

    def __image_callback(self, msg: Image):
        try:
            self.__image = self.__cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.__node.get_logger().error(f"Failed to convert image: {e}")
    

    def execute(self, userdata):
        """SMACH state を実行し、カメラ画像と prompt を Gemini に問い合わせる。

        Parameters
        ----------
        userdata : smach.UserData
            prompt_message と ud_prompt を入力として参照し、result に応答文字列を格納する。

        Returns
        -------
        str
            SMACH outcome。'success' または 'failure'。
        """
        try:
            self.__image = None
            with TemporarySubscriber(self.__node,
                                Image,
                                "/head_camera/d455/color/image_raw",
                                10,
                                self.__image_callback):
                while rclpy.ok() and self.__image is None:
                    rclpy.spin_once(self.__node, timeout_sec=0.1)
            
            # 画像がない場合はエラー
            if self.__image is None:
                return 'failure'
            
            # Gemini に互換性がある形式にする
            img_height, img_width = self.__image.shape[:2]
            rgb_image = cv2.cvtColor(self.__image, cv2.COLOR_BGR2RGB)
            pil_image = PILImage.fromarray(rgb_image)

            # Gemini に投げる
            prompt_message = userdata.prompt_message + "\nin English."
            ud_prompt_message = userdata.ud_prompt
            self.__node.get_logger().info("""
            ASK Gemini with image and text prompt...
            =========================================
            Prompt: %s
            """%userdata.prompt_message)
            response = self.__gemini_client.models.generate_content(
                model=self.__model_id,
                contents=[prompt_message, ud_prompt_message, pil_image]
            )
            self.__node.get_logger().info("""
            Gemini response is received!
            =========================================
            message: %s
            """%response.text)
            userdata.result = response.text
            
            return 'success'
                                
        except:
            self.__node.get_logger().error('Error is occured in RoboticsERState\n%s'%traceback.format_exc())
            return 'failure'

