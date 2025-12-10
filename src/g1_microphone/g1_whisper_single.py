import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyaudio
import numpy as np
from faster_whisper import WhisperModel
import threading
import time
import queue
import argparse
import sys
class G1WhisperNode(Node):
    def __init__(self, device_index=None, model_size='medium', device='cpu', energy_threshold=1000, silence_limit=1.0):
        super().__init__('g1_whisper_node')
        
        # Configuration
        self.device_index = device_index
        self.energy_threshold = energy_threshold
        self.silence_limit = silence_limit
        self.rate = 16000
        self.chunk = 4096
        self.format = pyaudio.paInt16
        
        # ROS Publisher
        self.publisher_ = self.create_publisher(String, '/g1/voice/text', 10)
        
        # Audio Queue and State
        self.audio_queue = queue.Queue()
        self.is_running = True
        
        # Initialize Whisper
        self.get_logger().info(f"Loading Whisper model '{model_size}' on '{device}'...")
        try:
            # compute_type="int8" is good for Jetson CPU/GPU
            self.model = WhisperModel(model_size, device=device, compute_type="int8")
            self.get_logger().info("Whisper model loaded.")
        except Exception as e:
            self.get_logger().error(f"Failed to load Whisper model: {e}")
            sys.exit(1)
        # Start Audio Thread
        self.audio_thread = threading.Thread(target=self.record_and_process_audio)
        self.audio_thread.daemon = True
        self.audio_thread.start()
        
        self.get_logger().info("G1 Whisper Node Started. Waiting for speech...")
    def record_and_process_audio(self):
        p = pyaudio.PyAudio()
        
        try:
            stream = p.open(format=self.format,
                            channels=1,
                            rate=self.rate,
                            input=True,
                            input_device_index=self.device_index,
                            frames_per_buffer=self.chunk)
        except Exception as e:
            self.get_logger().error(f"Failed to open microphone: {e}")
            self.get_logger().error("Check your device_index.")
            return
        speech_buffer = []
        is_speech = False
        last_speech_time = time.time()
        
        self.get_logger().info("Microphone stream opened.")
        while self.is_running and rclpy.ok():
            try:
                data = stream.read(self.chunk, exception_on_overflow=False)
                audio_chunk = np.frombuffer(data, dtype=np.int16)
                
                # Check energy
                # Convert to float for simpler math just for detection, keep int16 for whisper
                energy = np.sqrt(np.mean(audio_chunk.astype(np.float32)**2))
                
                if energy > self.energy_threshold:
                    if not is_speech:
                        self.get_logger().info("Speech detected...")
                        is_speech = True
                    last_speech_time = time.time()
                    speech_buffer.append(audio_chunk)
                
                elif is_speech:
                    # Append silence briefly to capture trailing dictation
                    speech_buffer.append(audio_chunk)
                    
                    if (time.time() - last_speech_time) > self.silence_limit:
                        self.get_logger().info("Silence detected, transcribing...")
                        
                        # Process buffer
                        full_audio = np.concatenate(speech_buffer)
                        
                        # Convert to float32 [-1, 1] for Whisper
                        full_audio_float = full_audio.astype(np.float32) / 32768.0
                        
                        self.transcribe(full_audio_float)
                        
                        # Reset
                        is_speech = False
                        speech_buffer = []
            except IOError as e:
                self.get_logger().warn(f"Audio overflow/error: {e}")
                continue
                
        stream.stop_stream()
        stream.close()
        p.terminate()
    def transcribe(self, audio_data):
        # Prevent transcribing extremely short noise
        if len(audio_data) < self.rate * 0.5:
            return
        try:
            segments, _ = self.model.transcribe(audio_data, beam_size=5)
            text = "".join([s.text for s in segments]).strip()
            
            if text:
                self.get_logger().info(f"TRANSCRIPTION: {text}")
                msg = String()
                msg.data = text
                self.publisher_.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Inference error: {e}")
def main():
    parser = argparse.ArgumentParser(description='G1 Whisper Node (Single Script)')
    parser.add_argument('--device-index', type=int, default=None, help='Microphone device index')
    parser.add_argument('--model', type=str, default='tiny', help='Whisper model size (tiny, base, small...)') # "tiny" is usually best for Latency on Jetson
    parser.add_argument('--compute-device', type=str, default='cpu', help='Inference device (cpu or cuda)')
    args, _ = parser.parse_known_args()
    rclpy.init()
    
    # Check if user passed arguments via --ros-args or standard args
    # Just use argparse values for simplicity in single script mode
    
    node = G1WhisperNode(
        device_index=args.device_index, 
        model_size=args.model,
        device=args.compute_device
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.is_running = False
        if node.audio_thread.is_alive():
            node.audio_thread.join(timeout=1.0)
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()
