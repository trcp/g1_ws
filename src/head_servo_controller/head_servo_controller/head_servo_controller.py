import sys
import math
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from dynamixel_sdk import *
from serial import SerialException

# ★ カスタムサービスのインポート
from g1_srvs.srv import MoveServo

# ==========================================
#               ユーザー設定エリア
# ==========================================
DEVICENAME = '/dev/ttyUSB0'
BAUDRATE   = 1000000
PROTOCOL_VERSION = 2.0

ID_PAN  = 1
ID_TILT = 0

JOINT_NAME_PAN  = "xl330_joint"
JOINT_NAME_TILT = "d455_joint"

PAN_HOME_PULSE  = 2500
TILT_HOME_PULSE = 2048

PAN_DIR  = 1
TILT_DIR = 1

PAN_RAD_OFFSET  = 0.0
TILT_RAD_OFFSET = -0.6

# --- 移動速度の設定 (Profile Velocity) ---
# 0     : 制限なし（最大速度・急発進）
# 30    : 超ゆっくり (安全・低負荷)
# 100   : 普通
# 300   : 速い
# 目安: XL430の最大値は約 260 です。まずは 50-100 くらい推奨。
MOVING_SPEED = 60 

# --- 制御パラメータ ---
CONTROL_PERIOD_SEC = 0.05  # 20Hz
VEL_TIMEOUT_SEC    = 0.5
SERVICE_TIMEOUT    = 5.0   # ゆっくり動くのでタイムアウトを少し延長
GOAL_TOLERANCE     = 0.08

# ==========================================

ADDR_OPERATING_MODE   = 11
ADDR_TORQUE_ENABLE    = 64
ADDR_PROFILE_VELOCITY = 112 # ★速度制限用アドレス
ADDR_GOAL_POSITION    = 116
ADDR_PRESENT_POSITION = 132

class PanTiltNode(Node):
    def __init__(self):
        super().__init__('head_servo_controller')
        
        self.target_pan_rad = 0.0
        self.target_tilt_rad = 0.0
        self.vel_cmd_pan = 0.0
        self.vel_cmd_tilt = 0.0
        self.last_vel_time = 0.0
        self.is_connected = False

        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        self.try_connect()

        self.pub_pan_raw = self.create_publisher(Int32, '/pan/present_position', 10)
        self.pub_tilt_raw = self.create_publisher(Int32, '/tilt/present_position', 10)
        self.pub_joint_state = self.create_publisher(JointState, '/joint_states', 10)

        self.create_subscription(Twist, '/servo_vel', self.vel_callback, 10)
        self.create_service(MoveServo, '/move_servo', self.move_servo_callback)

        self.timer = self.create_timer(CONTROL_PERIOD_SEC, self.timer_callback)

    def try_connect(self):
        self.get_logger().info(f"Connecting to {DEVICENAME}...")
        try:
            if self.portHandler.is_open:
                self.portHandler.closePort()
            
            if self.portHandler.openPort():
                if self.portHandler.setBaudRate(BAUDRATE):
                    self.get_logger().info(f"Connected. Initializing motors...")
                    # 接続時にPan/Tilt両方の設定を試みる
                    if self.setup_motor(ID_PAN, "Pan") and self.setup_motor(ID_TILT, "Tilt"):
                        self.get_logger().info("Motors Ready!")
                        self.is_connected = True
                        return True
            self.get_logger().error("Failed to open port or set baudrate.")
        except Exception as e:
            self.get_logger().error(f"Connection Exception: {e}")
        
        self.is_connected = False
        return False

    def setup_motor(self, dxl_id, name):
        try:
            # 1. トルクOFF
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, 0)
            time.sleep(0.02)
            
            # 2. モード設定 (Position Control Mode)
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_OPERATING_MODE, 3)
            time.sleep(0.02)
            
            # 3. ★速度制限 (Profile Velocity) の設定
            # これにより「set_position」命令時も、指定した速度で滑らかに動きます
            self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, ADDR_PROFILE_VELOCITY, MOVING_SPEED)
            time.sleep(0.02)
            
            # 4. トルクON
            res, err = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, 1)
            if res == COMM_SUCCESS and err == 0:
                self.get_logger().info(f"[{name}] Velocity Profile set to {MOVING_SPEED}. Torque ON.")
                return True
        except Exception:
            pass
        self.get_logger().warn(f"[{name}] Setup failed. Will retry.")
        return False

    def safe_read_pulse(self, dxl_id):
        if not self.is_connected: return None
        try:
            pos, res, err = self.packetHandler.read4ByteTxRx(self.portHandler, dxl_id, ADDR_PRESENT_POSITION)
            if res == COMM_SUCCESS:
                return pos
        except (SerialException, OSError) as e:
            self.handle_disconnect(e)
        return None

    def safe_write_pulse(self, dxl_id, pulse):
        if not self.is_connected: return
        pulse = max(0, min(4095, int(pulse)))
        try:
            res, err = self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, ADDR_GOAL_POSITION, pulse)
        except (SerialException, OSError) as e:
            self.handle_disconnect(e)

    def handle_disconnect(self, e):
        if self.is_connected:
            self.get_logger().error(f"CRITICAL: Communication lost! ({e})")
            self.is_connected = False

    def rad_to_pulse(self, rad, home_pulse, direction):
        return int(home_pulse + (rad * (4096.0 / (2 * math.pi)) * direction))

    def pulse_to_rad(self, pulse, home_pulse, direction, offset_rad):
        return (pulse - home_pulse) * (2 * math.pi / 4096.0) * direction + offset_rad

    def write_position_rad(self, dxl_id, rad, home_pulse, direction):
        pulse = self.rad_to_pulse(rad, home_pulse, direction)
        self.safe_write_pulse(dxl_id, pulse)

    # --- 共通処理: 状態の取得と配信 ---
    def publish_current_state(self):
        pan_pulse = self.safe_read_pulse(ID_PAN)
        tilt_pulse = self.safe_read_pulse(ID_TILT)

        if pan_pulse is not None and tilt_pulse is not None:
            self.pub_pan_raw.publish(Int32(data=pan_pulse))
            self.pub_tilt_raw.publish(Int32(data=tilt_pulse))

            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.name = [JOINT_NAME_PAN, JOINT_NAME_TILT]
            
            p_rad = self.pulse_to_rad(pan_pulse, PAN_HOME_PULSE, PAN_DIR, PAN_RAD_OFFSET)
            t_rad = self.pulse_to_rad(tilt_pulse, TILT_HOME_PULSE, TILT_DIR, TILT_RAD_OFFSET)
            
            joint_state.position = [p_rad, t_rad]
            joint_state.velocity = []
            joint_state.effort = []
            self.pub_joint_state.publish(joint_state)
            
            return p_rad, t_rad
        return None, None

    # --- サービスコールバック ---
    def move_servo_callback(self, request, response):
        target_pan = request.pan
        target_tilt = request.tilt
        
        self.get_logger().info(f"MoveServo: Pan={target_pan:.2f}, Tilt={target_tilt:.2f}")
        
        self.target_pan_rad = target_pan
        self.target_tilt_rad = target_tilt
        
        self.write_position_rad(ID_PAN, target_pan, PAN_HOME_PULSE, PAN_DIR)
        self.write_position_rad(ID_TILT, target_tilt, TILT_HOME_PULSE, TILT_DIR)
        
        success = self.wait_for_both_arrival(target_pan, target_tilt)
        
        response.success = success
        if success:
            self.get_logger().info(" -> Target Reached.")
        else:
            self.get_logger().warn(" -> Timeout or Stalled.")
            
        return response

    def wait_for_both_arrival(self, target_pan, target_tilt):
        start_time = time.time()
        while (time.time() - start_time) < SERVICE_TIMEOUT:
            if not self.is_connected: return False
            
            curr_pan, curr_tilt = self.publish_current_state()
            
            if curr_pan is not None and curr_tilt is not None:
                pure_curr_pan = curr_pan - PAN_RAD_OFFSET
                pure_curr_tilt = curr_tilt - TILT_RAD_OFFSET
                
                err_pan = abs(target_pan - pure_curr_pan)
                err_tilt = abs(target_tilt - pure_curr_tilt)
                
                if err_pan < GOAL_TOLERANCE and err_tilt < GOAL_TOLERANCE:
                    return True
            time.sleep(0.05)
        return False

    def vel_callback(self, msg):
        self.vel_cmd_pan = msg.angular.z
        self.vel_cmd_tilt = msg.angular.y
        self.last_vel_time = time.time()

    def timer_callback(self):
        if not self.is_connected:
            self.get_logger().warn("Connection lost. Retrying...", throttle_duration_sec=2.0)
            self.try_connect()
            return

        if (time.time() - self.last_vel_time) < VEL_TIMEOUT_SEC:
            self.target_pan_rad  += self.vel_cmd_pan  * CONTROL_PERIOD_SEC
            self.target_tilt_rad += self.vel_cmd_tilt * CONTROL_PERIOD_SEC
            self.write_position_rad(ID_PAN, self.target_pan_rad, PAN_HOME_PULSE, PAN_DIR)
            self.write_position_rad(ID_TILT, self.target_tilt_rad, TILT_HOME_PULSE, TILT_DIR)

        curr_pan, curr_tilt = self.publish_current_state()
        
        if curr_pan is not None and curr_tilt is not None:
            if (time.time() - self.last_vel_time) >= VEL_TIMEOUT_SEC:
                 self.target_pan_rad = curr_pan - PAN_RAD_OFFSET
                 self.target_tilt_rad = curr_tilt - TILT_RAD_OFFSET

    def __del__(self):
        try:
            if self.is_connected:
                self.packetHandler.write1ByteTxRx(self.portHandler, ID_PAN, ADDR_TORQUE_ENABLE, 0)
                self.packetHandler.write1ByteTxRx(self.portHandler, ID_TILT, ADDR_TORQUE_ENABLE, 0)
            self.portHandler.closePort()
        except:
            pass

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
