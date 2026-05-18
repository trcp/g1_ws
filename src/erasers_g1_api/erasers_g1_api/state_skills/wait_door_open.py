#!/usr/bin/env python3
from rclpy.node import Node
from rclpy import qos
import rclpy

import smach

from sensor_msgs.msg import LaserScan

from rclpy_util.util import TemporarySubscriber
from erasers_g1_api.tts import TTS

import numpy as np
import traceback
import time
import math
import os


class WaitDoorOpen(smach.State):
    def __init__(self,
                 node:Node,
                 tts_say:TTS.say,
                 start_msg:str='Please open the door.',
                 success_msg:str='Thank you.',
                 timeout_msg:str='Time is out.',
                 failure_msg:str='Failure. error is ocurred in wait door open.',
                 timeout_sec:int=10,
                 threshold:float=1.0,
                 front_angle:float=0.0,
                 front_width:float=math.radians(20.0),):
        
        # init smach
        smach.State.__init__(self,
                             outcomes=['success', 'timeout', 'failure'])

        # init values
        self.__node:Node = node
        self.__say:TTS.say = tts_say
        self.__start_msg:str = start_msg
        self.__success_msg:str = success_msg
        self.__timeout_msg:str = timeout_msg
        self.__failure_msg:str = failure_msg
        self.__timeout_sec = timeout_sec
        self.__threshold = threshold
        self.__front_angle = front_angle
        self.__front_width = front_width
        self.__door_open:bool = False
        
    
    def __laserscan_cb(self, msg:LaserScan):
        try:
            # skip msg is empty
            if not msg.ranges:
                return
            
            arr = np.array(msg.ranges, dtype=float)
            angles = msg.angle_min + np.arange(len(arr)) * msg.angle_increment
            angle_diff = np.arctan2(
                np.sin(angles - self.__front_angle),
                np.cos(angles - self.__front_angle),
            )
            front_data = arr[np.abs(angle_diff) <= self.__front_width / 2.0]
            valid_data = front_data[np.isfinite(front_data)]
            valid_data = valid_data[
                (valid_data >= msg.range_min) & (valid_data <= msg.range_max)
            ]
            if len(valid_data) == 0:
                laser_value = float('inf')
            else:
                laser_value = np.min(valid_data)

            self.__node.get_logger().info('Current front min LaserScan: %f' % laser_value)
            
            if laser_value > self.__threshold:
                self.__door_open = True
            
        except Exception as e:
            self.__node.get_logger().warn(f"LaserScan callback error: {e}")
    

    def execute(self, userdata):
        try:
            laser_scan_qos_profile = qos.QoSProfile(
                reliability=qos.ReliabilityPolicy.BEST_EFFORT,
                history=qos.HistoryPolicy.KEEP_LAST,
                depth=10
            )

            # detect open door via LaserScan
            with TemporarySubscriber(self.__node,
                                    LaserScan,
                                    '/scan',
                                    laser_scan_qos_profile,
                                    self.__laserscan_cb):

                # declare msg
                self.__node.get_logger().info('''
                    WAIT DOOR OPEN ....
                ''')
                self.__say(self.__start_msg)

                init_time = time.time()
                while time.time() - init_time < self.__timeout_sec and not self.__door_open:
                    rclpy.spin_once(self.__node, timeout_sec=0.1)
            
            # Success
            if self.__door_open:
                self.__node.get_logger().info('''
                    DOOR OPEN !
                ''')
                self.__say(self.__success_msg)

                return 'success'

            # Timeout
            else:
                self.__node.get_logger().warn('''
                    TIMEOUT. DOOR IS NOT OPEN ...
                ''')
                self.__say(self.__timeout_msg)

                return 'timeout'
        
        except:
            self.__say(self.__failure_msg)
            self.__node.get_logger().error('Error is occured in WaitDoorOpen\n%s'%traceback.format_exc())
            return 'failure'
