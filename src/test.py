import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import evdev
import threading

class CustomInputNode(Node):
    def __init__(self, device_path='/dev/input/event1'):
        super().__init__('custom_input_node')
        self.publisher_ = self.create_publisher(Joy, 'joy', 10)
        self.device = evdev.InputDevice(device_path)
        self.thread = threading.Thread(target=self.read_loop, daemon=True)
        self.thread.start()

    def read_loop(self):
        joy_msg = Joy()
        joy_msg.buttons = [0] * 10 
        
        for event in self.device.read_loop():
            if event.type == evdev.ecodes.EV_KEY:
                joy_msg.header.stamp = self.get_clock().now().to_msg()
                self.publisher_.publish(joy_msg)

def main():
    rclpy.init()
    node = CustomInputNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
