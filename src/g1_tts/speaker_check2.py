import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import json
import subprocess
import shutil
import os
import time

# メッセージ型のインポート試行
try:
    from unitree_api.msg import Request, RequestHeader, RequestIdentity
    MSG_IMPORT_SUCCESS = True
except ImportError:
    MSG_IMPORT_SUCCESS = False

class G1DeepDiagnosis(Node):
    def __init__(self):
        super().__init__('g1_deep_diagnosis')

        self.get_logger().info("========================================")
        self.get_logger().info("   Unitree G1 Audio Deep Diagnosis      ")
        self.get_logger().info("========================================")

        # 1. ライブラリチェック
        if not MSG_IMPORT_SUCCESS:
            self.get_logger().error("[CRITICAL] 'unitree_api' msg package missing!")
            self.get_logger().error(" -> Solution: source ~/colcon_ws/install/setup.bash")
            return

        # QoS設定 (Best Effort推奨)
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self.publisher_ = self.create_publisher(Request, '/api/voice/request', qos_profile)
        
        # 診断実行
        self.run_diagnostics()

        # 発話試行
        self.timer = self.create_timer(2.0, self.send_speech_request)

    def run_diagnostics(self):
        """ システム全体を診断する """
        
        # --- Check A: コンテナ環境とツール ---
        self.get_logger().info("[Check 1/4] Environment & Tools...")
        amixer_path = shutil.which('amixer')
        if amixer_path:
            self.get_logger().info(f"  [OK] 'amixer' found at: {amixer_path}")
        else:
            self.get_logger().warn("  [FAIL] 'amixer' command NOT found.")
            self.get_logger().warn("   -> Fix: Run 'sudo apt-get update && sudo apt-get install alsa-utils'")

        # --- Check B: ハードウェア認識 (/dev/snd) ---
        # amixerがなくても、Linuxのデバイスファイルを見ることでハードウェア認識を確認できる
        self.get_logger().info("[Check 2/4] Hardware Visibility...")
        if os.path.exists('/dev/snd'):
            self.get_logger().info("  [OK] /dev/snd directory exists.")
            # カードリストを直接読む
            try:
                with open('/proc/asound/cards', 'r') as f:
                    cards = f.read().strip()
                if cards:
                    self.get_logger().info(f"  [OK] Sound Cards detected:\n{cards}")
                else:
                    self.get_logger().error("  [FAIL] /proc/asound/cards is empty. No sound cards recognized!")
            except FileNotFoundError:
                self.get_logger().error("  [FAIL] /proc/asound/cards not readable.")
        else:
            self.get_logger().error("  [CRITICAL] /dev/snd does NOT exist.")
            self.get_logger().error("   -> Docker Error: Container launched without sound device access.")
            self.get_logger().error("   -> Fix: Add '--device /dev/snd' to your docker run command.")

        # --- Check C: ROS 2 通信トポロジー ---
        self.get_logger().info("[Check 3/4] ROS 2 Network Topology...")
        # 少し待ってトポロジー更新を待つ
        time.sleep(0.5)
        sub_count = self.publisher_.get_subscription_count()
        if sub_count > 0:
            self.get_logger().info(f"  [OK] Found {sub_count} subscriber(s) listening to /api/voice/request.")
            self.get_logger().info("   -> Robot service is ACTIVE and LISTENING.")
        else:
            self.get_logger().warn(f"  [WARNING] No subscribers found (Count: {sub_count}).")
            self.get_logger().warn("   -> Possibility 1: Robot internal service is down.")
            self.get_logger().warn("   -> Possibility 2: Namespace mismatch (check 'ros2 topic list').")
            self.get_logger().warn("   -> Possibility 3: Network isolation (check ROS_DOMAIN_ID).")

    def send_speech_request(self):
        self.get_logger().info("[Check 4/4] Sending TTS Request...")
        
        req = Request()
        req.header = RequestHeader()
        req.header.identity = RequestIdentity()
        req.header.identity.api_id = 1005 
        
        # 英語と日本語の両方を含めて送ってみる（エンジン依存回避）
        text_msg = "System diagnosis. 音声システム診断中。"
        
        parameter_dict = {
            "cmd": 1,
            "text": text_msg,
            "volume": 100
        }
        
        req.parameter = json.dumps(parameter_dict)

        self.publisher_.publish(req)
        self.get_logger().info(f"  -> Payload sent: {req.parameter}")
        self.get_logger().info("Diagnosis complete. Shutting down.")

        self.timer.cancel()
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = G1DeepDiagnosis()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
