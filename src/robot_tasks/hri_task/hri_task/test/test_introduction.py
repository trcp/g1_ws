#!/usr/bin/env python3
import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import rclpy
from rclpy.node import Node
import time

# API
from erasers_g1_api.tts import TTS
from erasers_g1_api.robot_control import G1Control

# Task modules
from direct_joint_control import DirectJointController
from main import introduce_guests_cb, describe_guest_1_cb

class DummyUserdata:
    def __init__(self):
        # ゲスト1（ホスト）のダミーデータ
        self.g1_name = "Alice"
        self.g1_drink = "Apple juice"
        self.g1_features = {"color": "red", "glasses": "wearing glasses"}
        
        # ゲスト2のダミーデータ
        self.g2_name = "Bob"
        self.g2_drink = "Beer"

def main():
    rclpy.init()
    node = Node('test_introduction_node')
    
    tts = TTS(node)
    SAY = tts.say
    CONTROL = G1Control(node)
    ARM = DirectJointController(node)
    
    # 仮想のuserdata
    ud = DummyUserdata()
    
    # 仮想的に、ゲスト2（見つけたときの腰の角度）をセット
    # 正面が 0.0、右がマイナス、左がプラス
    ARM.guest2_waist_yaw = -0.5
    
    node.get_logger().info("=== Testing describe_guest_1_cb ===")
    describe_guest_1_cb(ud, node, SAY)
    
    time.sleep(2)
    
    node.get_logger().info("=== Testing introduce_guests_cb ===")
    introduce_guests_cb(ud, node, SAY, CONTROL, ARM)
    
    node.get_logger().info("Test finished.")
    
    # 終了前にホームポジションに戻す
    ARM.go_home()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
