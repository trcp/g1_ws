import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import json
import time

try:
    from unitree_api.msg import Request, RequestHeader, RequestIdentity
except ImportError:
    pass

class G1EnglishTest(Node):
    def __init__(self):
        super().__init__('g1_english_test')
        
        # QoSは Best Effort (これが重要)
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        
        self.publisher_ = self.create_publisher(Request, '/api/voice/request', qos_profile)
        self.timer = self.create_timer(1.0, self.send_request)
        self.get_logger().info("Waiting to send English TTS request...")

    def send_request(self):
        req = Request()
        req.header = RequestHeader()
        req.header.identity = RequestIdentity()
        req.header.identity.api_id = 1005  # TTS API
        
        # 【重要】英語、かつ非常に短い単語にする
        # ネットが遅くても処理できるようにする
        text = "Hello" 
        
        parameter_dict = {
            "cmd": 1,
            "text": text,
            "volume": 100
        }
        
        req.parameter = json.dumps(parameter_dict)
        self.publisher_.publish(req)
        self.get_logger().info(f"Sent: {text}")
        
        # 連続送信して気づかせる
        time.sleep(2) 
        
        # 2回目：少し長い文章
        text2 = "System online."
        parameter_dict["text"] = text2
        req.parameter = json.dumps(parameter_dict)
        self.publisher_.publish(req)
        self.get_logger().info(f"Sent: {text2}")

        self.timer.cancel()
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = G1EnglishTest()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
