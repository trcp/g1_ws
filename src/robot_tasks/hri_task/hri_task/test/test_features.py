#!/usr/bin/env python3
import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import argparse

class TestFeaturesNode(Node):
    def __init__(self, feature_mode="online"):
        super().__init__('test_features_node')
        self.feature_mode = feature_mode
        
        # コマンドをパブリッシュするパブリッシャー
        self.cmd_pub = self.create_publisher(String, '/yolo_human/command', 10)
        self.res_sub = self.create_subscription(String, '/yolo_human/result', self.result_cb, 10)
        
        self.last_print_time = time.time()
        self.get_logger().info(f"Test Features Node started (Mode: {self.feature_mode}). Waiting for YOLO results...")
        
        # 起動後1回だけコマンドを送信する
        self.create_timer(1.0, self.send_start_cmd_once)
        self.cmd_sent = False

    def send_start_cmd_once(self):
        if self.cmd_sent:
            return
            
        msg = String()
        # extract_features: true を送って特徴抽出を有効化
        msg.data = json.dumps({
            "command": "start", 
            "classes": ["person"], 
            "extract_features": True,
            "feature_mode": self.feature_mode
        })
        self.cmd_pub.publish(msg)
        self.get_logger().info("Sent YOLO start command with extract_features=True")
        self.cmd_sent = True

    def result_cb(self, msg):
        now = time.time()
        # 5秒に1回だけログ出力する
        if now - self.last_print_time < 5.0:
            return
            
        try:
            detections = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        closest_person = None
        min_z = float('inf')
        
        for det in detections:
            if det.get('label') == 'person':
                z = det.get('distance_z', 999.0)
                if z < min_z:
                    min_z = z
                    closest_person = det
                    
        if closest_person:
            features = closest_person.get('features')
            self.get_logger().info(f"--- Closest Person at {min_z:.2f}m ---")
            if features is not None:
                # リストならオンライン(OpenAI)、辞書ならオフライン
                mode_str = "[ONLINE]" if isinstance(features, list) else "[OFFLINE]"
                self.get_logger().info(f"{mode_str} Extracted Features:\n{json.dumps(features, indent=2, ensure_ascii=False)}")
            else:
                self.get_logger().info("Features not found in detection. (Waiting for API processing...)")
            self.last_print_time = now
        else:
            self.get_logger().info("No person detected.")
            self.last_print_time = now

def main(args=None):
    import sys
    parser = argparse.ArgumentParser(description="Test feature extraction via YOLO node.")
    parser.add_argument('--mode', type=str, choices=['online', 'offline'], default='online', 
                        help="Feature extraction mode (default: online)")
    
    # rclpy.init用の引数と分離
    parsed_args, ros_args = parser.parse_known_args(sys.argv)
    
    rclpy.init(args=ros_args)
    node = TestFeaturesNode(feature_mode=parsed_args.mode)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
