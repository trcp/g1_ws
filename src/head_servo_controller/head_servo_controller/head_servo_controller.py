import sys
import math
import time  # timeモジュールを追加
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import JointState
from dynamixel_sdk import *

# ==========================================
#               ユーザー設定エリア
# ==========================================
DEVICENAME = '/dev/ttyUSB0'
BAUDRATE   = 1000000
PROTOCOL_VERSION = 2.0

# --- モーターID設定 ---
ID_PAN  = 1
ID_TILT = 0

# --- URDF上の関節名 ---
JOINT_NAME_PAN  = "xl330_joint"
JOINT_NAME_TILT = "d455_joint"

# --- キャリブレーション設定 ---
PAN_HOME_PULSE  = 2048
TILT_HOME_PULSE = 2048

# --- 回転方向 ---
PAN_DIR  = 1
TILT_DIR = 1

# --- Rviz表示位置の微調整オフセット ---
PAN_RAD_OFFSET  = 0.0
TILT_RAD_OFFSET = -0.6

# ==========================================

# XL430 アドレス
ADDR_OPERATING_MODE   = 11
ADDR_TORQUE_ENABLE    = 64
ADDR_GOAL_POSITION    = 116
ADDR_PRESENT_POSITION = 132

class PanTiltNode(Node):
    def __init__(self):
        super().__init__('pantilt_node')
        
        # SDK初期化
        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        if not self.portHandler.openPort():
            self.get_logger().error("Failed to open the port")
            sys.exit()

        if not self.portHandler.setBaudRate(BAUDRATE):
            self.get_logger().error("Failed to set the baudrate")
            sys.exit()

        self.get_logger().info(f"Connected to {DEVICENAME} at {BAUDRATE}bps")

        # モーター初期化 (少し時間を空けて実行)
        self.setup_motor(ID_PAN, "Pan")
        time.sleep(0.1) # 0.1秒待つ
        self.setup_motor(ID_TILT, "Tilt")

        # Subscribers
        self.create_subscription(Int32, '/pan/set_position', self.pan_callback, 10)
        self.create_subscription(Int32, '/tilt/set_position', self.tilt_callback, 10)

        # Publishers
        self.pub_pan_raw = self.create_publisher(Int32, '/pan/present_position', 10)
        self.pub_tilt_raw = self.create_publisher(Int32, '/tilt/present_position', 10)
        self.pub_joint_state = self.create_publisher(JointState, '/joint_states', 10)

        # Timer (20Hz)
        self.timer = self.create_timer(0.05, self.timer_callback)

    def setup_motor(self, dxl_id, name):
        # トルクOFF
        self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, 0)
        time.sleep(0.05) # 短い待機
        
        # Mode設定
        self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_OPERATING_MODE, 3)
        time.sleep(0.05) # 短い待機
        
        # トルクON (リトライ付き)
        for i in range(3):
            res, err = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, 1)
            if res == COMM_SUCCESS and err == 0:
                self.get_logger().info(f"[{name}] Torque ON Success.")
                return
            time.sleep(0.05)
        
        # 3回失敗したらログを出す
        self.get_logger().error(f"[{name}] Failed to enable Torque after 3 retries.")

    def pulse_to_rad(self, pulse, home_pulse, direction, offset_rad):
        base_rad = (pulse - home_pulse) * (2 * math.pi / 4096.0) * direction
        return base_rad + offset_rad

    def read_position(self, dxl_id):
        pos, res, err = self.packetHandler.read4ByteTxRx(self.portHandler, dxl_id, ADDR_PRESENT_POSITION)
        if res != COMM_SUCCESS:
            return None
        return pos

    def write_position(self, dxl_id, position):
        position = max(0, min(4095, position))
        res, err = self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, ADDR_GOAL_POSITION, position)
        if res != COMM_SUCCESS:
             self.get_logger().warn(f"Failed to write position to ID {dxl_id}")

    # --- Callbacks ---
    def pan_callback(self, msg):
        self.write_position(ID_PAN, msg.data)

    def tilt_callback(self, msg):
        self.write_position(ID_TILT, msg.data)

    def timer_callback(self):
        pan_pulse = self.read_position(ID_PAN)
        tilt_pulse = self.read_position(ID_TILT)

        if pan_pulse is None or tilt_pulse is None:
            return

        # 生データ配信
        self.pub_pan_raw.publish(Int32(data=pan_pulse))
        self.pub_tilt_raw.publish(Int32(data=tilt_pulse))

        # JointState計算 & 配信
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = [JOINT_NAME_PAN, JOINT_NAME_TILT]
        
        pan_rad = self.pulse_to_rad(pan_pulse, PAN_HOME_PULSE, PAN_DIR, PAN_RAD_OFFSET)
        tilt_rad = self.pulse_to_rad(tilt_pulse, TILT_HOME_PULSE, TILT_DIR, TILT_RAD_OFFSET)
        
        joint_state.position = [pan_rad, tilt_rad]
        joint_state.velocity = []
        joint_state.effort = []

        self.pub_joint_state.publish(joint_state)

    def __del__(self):
        self.packetHandler.write1ByteTxRx(self.portHandler, ID_PAN, ADDR_TORQUE_ENABLE, 0)
        self.packetHandler.write1ByteTxRx(self.portHandler, ID_TILT, ADDR_TORQUE_ENABLE, 0)
        self.portHandler.closePort()

def main(args=None):
    rclpy.init(args=args)
    node = PanTiltNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
