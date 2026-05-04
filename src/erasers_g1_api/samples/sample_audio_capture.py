#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from erasers_g1_api.robot_control import G1Mic
import time
import os
import numpy as np

class AudioCaptureSample(Node):
    def __init__(self):
        super().__init__('sample_audio_capture')
        self.get_logger().info('Initializing G1 Audio Capture Sample (Service/Topic based)')
        
        # Audio configuration
        self.capture_duration = 5.0 # seconds
        self.mic = G1Mic(node=self)
        
        # Output configuration
        self.output_dir = '/home/gai/colcon_ws/src/erasers_g1'
        self.output_file = os.path.join(self.output_dir, 'captured_audio.wav')

    def run(self):
        self.get_logger().info(f'Starting audio capture for {self.capture_duration} seconds...')
        
        audio_buffer = []
        start_time = time.time()

        try:
            # G1Mic context manager handles service calls (mic_rec)
            with self.mic as mic:
                while rclpy.ok() and (time.time() - start_time) < self.capture_duration:
                    # Spin to allow subscriber callbacks to run
                    rclpy.spin_once(self, timeout_sec=0.1)
                    
                    # Periodically read data to show progress
                    chunk = mic.read()
                    if chunk.size > 0:
                        audio_buffer.append(chunk)
                        self.get_logger().info(f'Received {chunk.size} samples...')
                
                # Final read to get any remaining data
                final_chunk = mic.read()
                if final_chunk.size > 0:
                    audio_buffer.append(final_chunk)

        except Exception as e:
            self.get_logger().error(f'Error during audio capture: {e}')

        if audio_buffer:
            all_audio = np.concatenate(audio_buffer)
            self.get_logger().info(f'Capture finished. Total samples: {all_audio.size}')
            
            # Save the captured audio to WAV
            self.get_logger().info(f'Saving to {self.output_file}...')
            success = self.mic.save_wav(self.output_file, all_audio)
            if success:
                self.get_logger().info('WAV file saved successfully.')
            else:
                self.get_logger().error('Failed to save WAV file.')
        else:
            self.get_logger().warn('No audio data received. Skipping save.')

def main(args=None):
    rclpy.init(args=args)
    node = AudioCaptureSample()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
