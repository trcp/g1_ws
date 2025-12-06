import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from unitree_go.msg import AudioData
import subprocess
import os

class G1TTSNode(Node):
    def __init__(self):
        super().__init__('g1_tts_node')

        # === 設定: Piperモデルのパス ===
        # 確実に存在するパスを指定してください
        self.model_path = "/home/unitree/en_US-amy-medium.onnx"
        # ============================

        # テキストを受け取る (Text -> Audio)
        self.subscription = self.create_subscription(
            String,
            '/audio_msg',
            self.listener_callback,
            10)
        
        # 音声データをUnitreeシステムに送る
        self.publisher_ = self.create_publisher(
            AudioData,
            '/audiosender',
            10)

        self.get_logger().info('G1 TTS Node is ready. Waiting for text on /audio_msg')

        if not os.path.exists(self.model_path):
            self.get_logger().error(f'Model file not found: {self.model_path}')

    def listener_callback(self, msg):
        text = msg.data
        self.get_logger().info(f'Reading: "{text}"')
        
        # 音声データを生成
        audio_data = self.generate_audio(text)

        if audio_data:
            self.publish_audio(audio_data)

    def generate_audio(self, text):
        """Piperを使ってテキストをRaw音声データに変換"""
        try:
            # Piperコマンド
            # --output_raw でヘッダなしのPCMデータを出力します
            cmd = [
                'piper',
                '--model', self.model_path,
                '--output_raw'
            ]

            process = subprocess.Popen(
                cmd,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            
            # 実行
            stdout_data, stderr_data = process.communicate(input=text.encode('utf-8'))

            if process.returncode != 0:
                self.get_logger().error(f'Piper Error: {stderr_data.decode()}')
                return None
            
            return stdout_data

        except Exception as e:
            self.get_logger().error(f'Generation Failed: {e}')
            return None

    def publish_audio(self, audio_bytes):
        """AudioDataメッセージを作成して配信"""
        msg = AudioData()
        
        # バイナリデータをuint8配列(リスト)に変換して格納
        msg.data = list(audio_bytes)
        
        # 必要であればIDなどを付与 (現状はデータのみで送信)
        msg.time_frame = 0 # タイムスタンプが必要な場合のプレースホルダー

        self.publisher_.publish(msg)
        self.get_logger().info(f'Sent {len(audio_bytes)} bytes to /audiosender')

def main(args=None):
    rclpy.init(args=args)
    node = G1TTSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
