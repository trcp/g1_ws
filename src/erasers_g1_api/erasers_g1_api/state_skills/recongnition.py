#!/usr/bin/env python3

# ROS2
from rclpy_util.util import TemporarySubscriber
from rclpy.node import Node
import rclpy

# TF
from tf2_ros import Buffer, TransformListener

# interfaces
from sensor_msgs.msg import Image, CameraInfo

# erasers API
from erasers_g1_api.tts import TTS

# state machine
import smach

# vision
from cv_bridge import CvBridge
import cv2

# preferences
import traceback
import time


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