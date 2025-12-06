import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy # 追加: QoS設定用
import json

try:
    from unitree_api.msg import Request, RequestHeader, RequestIdentity
except ImportError:
    print("Error: 'unitree_api' パッケージが見つかりません。")
    exit(1)

class G1Speaker(Node):
    def __init__(self):
        super().__init__('g1_speaker_node')

        # 【テスト1】まずは英語で試します（日本語非対応の可能性排除）
        self.text_to_speak = "Hello, G1 is ready." 

        # 【重要】QoSを Best Effort に設定
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

        # パブリッシャー作成時に QoS設定 を適用
        self.publisher_ = self.create_publisher(Request, '/api/voice/request', qos_profile)
        
        self.timer = self.create_timer(1.0, self.send_speech_request)
        self.get_logger().info('G1 Speaker Node has started (Best Effort QoS).')

    def send_speech_request(self):
        req = Request()
        req.header = RequestHeader()
        req.header.identity = RequestIdentity()
        req.header.identity.api_id = 1005 # echoで確認できたので1005で確定
        
        parameter_dict = {
            "cmd": 1,
            "text": self.text_to_speak,
            "volume": 100
        }
        
        req.parameter = json.dumps(parameter_dict)

        self.publisher_.publish(req)
        self.get_logger().info(f'Sent TTS request: "{self.text_to_speak}"')

        self.timer.cancel()
        self.destroy_timer(self.timer)
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = G1Speaker()

    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
