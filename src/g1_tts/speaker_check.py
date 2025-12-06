import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import json
import subprocess # Linuxコマンドを実行するために追加

try:
    from unitree_api.msg import Request, RequestHeader, RequestIdentity
except ImportError:
    print("Error: 'unitree_api' パッケージが見つかりません。")
    exit(1)

class G1SpeakerDiagnosis(Node):
    def __init__(self):
        super().__init__('g1_speaker_diagnosis')

        self.text_to_speak = "Checking system volume. G1 audio is active." 

        # QoS設定 (Best Effort)
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.publisher_ = self.create_publisher(Request, '/api/voice/request', qos_profile)
        
        # 起動時にまず音量を診断する
        self.diagnose_system_volume()

        self.timer = self.create_timer(1.0, self.send_speech_request)
        self.get_logger().info('G1 Speaker Node (Diagnostic Mode) has started.')

    def diagnose_system_volume(self):
        """
        LinuxのALSAミキサー(amixer)を叩いて、現在の音量設定を調査する関数
        """
        self.get_logger().info("--- Start Volume Diagnosis ---")
        
        # チェックするターゲット (G1/Jetsonでは通常 Card 1 の MVC がマスター)
        card_id = "1" 
        control_name = "MVC" # Master Volume Control

        try:
            # コマンド: amixer -c 1 sget MVC
            # 状態を取得する
            result = subprocess.run(
                ['amixer', '-c', card_id, 'sget', control_name],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )

            if result.returncode != 0:
                self.get_logger().warn(f"Warning: Control '{control_name}' on Card {card_id} not found.")
                self.get_logger().warn("Trying generic check...")
                # 失敗した場合、デバイス一覧を表示してみる
                check_all = subprocess.run(['amixer', '-c', card_id, 'scontrols'], capture_output=True, text=True)
                self.get_logger().info(f"Available controls:\n{check_all.stdout}")
            else:
                # 出力結果を解析して表示
                output = result.stdout
                self.get_logger().info(f"Audio Control '{control_name}' Status:\n{output}")
                
                # [off] という文字列が含まれていたらミュート状態
                if "[off]" in output:
                    self.get_logger().error("!!! SYSTEM IS MUTED (ミュートされています) !!!")
                    self.get_logger().info("Attempting to unmute automatically...")
                    # 自動でミュート解除と音量最大化を試みる
                    subprocess.run(['amixer', '-c', card_id, 'sset', control_name, '100%', 'unmute'])
                else:
                    self.get_logger().info("System is UNMUTED (音が出せる状態です).")

        except Exception as e:
            self.get_logger().error(f"Failed to run diagnosis: {e}")
        
        self.get_logger().info("--- End Volume Diagnosis ---")

    def send_speech_request(self):
        req = Request()
        req.header = RequestHeader()
        req.header.identity = RequestIdentity()
        req.header.identity.api_id = 1005 
        
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
    node = G1SpeakerDiagnosis()

    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
