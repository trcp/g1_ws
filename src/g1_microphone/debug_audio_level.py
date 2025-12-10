import pyaudio
import numpy as np
import time
import argparse
def visualize_audio(device_index):
    p = pyaudio.PyAudio()
    
    try:
        # Open stream
        stream = p.open(format=pyaudio.paInt16,
                        channels=1,
                        rate=48000,
                        input=True,
                        input_device_index=device_index,
                        frames_per_buffer=1024)
        print(f"Opening Device ID: {device_index} - {p.get_device_info_by_index(device_index).get('name')}")
        print("Speak now! (Press Ctrl+C to stop)")
        print("-" * 50)
        
        while True:
            data = stream.read(1024, exception_on_overflow=False)
            audio_data = np.frombuffer(data, dtype=np.int16)
            
            # RMS calculation
            rms = np.sqrt(np.mean(audio_data.astype(np.float32)**2))
            
            # Text visualization bar
            bars = int(rms / 100) # Sensitivity
            print(f"Volume: {int(rms):5d} |" + "#" * bars)
            
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error accessing device {device_index}: {e}")
    finally:
        if 'stream' in locals():
            stream.stop_stream()
            stream.close()
        p.terminate()
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--device-index", type=int, required=True)
    args = parser.parse_args()
    
    visualize_audio(args.device_index)
