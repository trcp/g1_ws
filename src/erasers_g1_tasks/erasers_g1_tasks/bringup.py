#!/usr/bin/env python3
from rclpy.node import Node
import rclpy

from erasers_g1_api.robot_control import G1Control, HeadControl
from erasers_g1_api.tts import VoicevoxTTS

import time
import traceback
import os


LED =[
    [255,100,0],
    [255, 0, 127],
    [255, 0, 255],
    [0, 0, 255],
    [0, 255, 0],
    [0, 130, 75],
    [143, 255, 0]
]
USE_HEAD = os.environ.get('USE_HEAD_CAMERA', 'false').lower().strip('\'"') == 'true'


def led_status_cb():
    for i in range(len(LED)):
        robot_control.led(LED[i][0], LED[i][1], LED[i][2])
        time.sleep(0.3 / len(LED))


def main():
    global \
        node, \
        robot_control
        
    # init ROS2
    rclpy.init()
    node = Node('system_bringup')
    led_status_timer = node.create_timer(0.3, led_status_cb)

    # init APIs
    try:
        tts = VoicevoxTTS(node)
        robot_control = G1Control(node)
        head_control = HeadControl(node)
    except Exception:
        node.get_logger().warn('Failure bringup API...\n%s'%traceback.format_exc())

    # LiDAR 再起動
    node.get_logger().info('Reboot LiDAR sensor ...')
    robot_control.robot_service_interface_setting('lidar_driver', False)
    robot_control.robot_service_interface_setting('lidar_driver', True)

    # 頭部センサーの状態を確認
    head_motor = [None, None]
    while head_motor[0] is None and head_motor[1] is None and USE_HEAD:
        head_motor = head_control.get_current_pose()

    # 起動成功時に発話
    led_status_timer.cancel()
    robot_control.led(255,255,255)
    tts.say('イレイサーズGワン，スタート！')

    # destroy
    node.destroy_node()
    rclpy.shutdown()
