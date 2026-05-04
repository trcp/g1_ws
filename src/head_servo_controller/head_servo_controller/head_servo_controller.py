# src/head_servo_controller/head_servo_controller/head_servo_controller.py

import sys
import math
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Float32, Int32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from dynamixel_sdk import *

# Import custom service
from g1_srvs.srv import MoveServo

# ==========================================
#               Default Constants (Fallback)
# ==========================================
DEFAULT_DEVICENAME = '/dev/ttyUSB0'
DEFAULT_BAUDRATE   = 1000000
PROTOCOL_VERSION   = 2.0

ADDR_OPERATING_MODE   = 11
ADDR_TORQUE_ENABLE    = 64
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION    = 116
ADDR_PRESENT_LOAD     = 126
ADDR_PRESENT_POSITION = 132

class PanTiltNode(Node):
    def __init__(self):
        super().__init__('head_servo_controller')
        
        self.lock = threading.Lock()
        self.callback_group = ReentrantCallbackGroup()
        
        # YAMLのデフォルト値を安全な値（1791, 2470）にフォールバック
        self.dx_path = self.declare_parameter('dx_path', DEFAULT_DEVICENAME).value
        self.pan_id = self.declare_parameter('pan_id', 1).value
        self.tilt_id = self.declare_parameter('tilt_id', 0).value
        self.pan_home_pulse = self.declare_parameter('pan_home_pulse', 1791).value
        self.tilt_home_pulse = self.declare_parameter('tilt_home_pulse', 2470).value
        self.pan_rad_offset = self.declare_parameter('pan_rad_offset', 0.0).value
        self.tilt_rad_offset = self.declare_parameter('tilt_rad_offset', 0.0).value
        self.pan_dir = self.declare_parameter('pan_dir', 1).value
        self.tilt_dir = self.declare_parameter('tilt_dir', 1).value
        self.pan_min_pulse = self.declare_parameter('pan_min_pulse', 0).value
        self.pan_max_pulse = self.declare_parameter('pan_max_pulse', 4000).value
        self.tilt_min_pulse = self.declare_parameter('tilt_min_pulse', 1455).value
        self.tilt_max_pulse = self.declare_parameter('tilt_max_pulse', 4000).value
        self.moving_speed = self.declare_parameter('moving_speed', 60).value
        self.calib_speed = self.declare_parameter('calib_speed', 20).value
        self.control_period_sec = self.declare_parameter('control_period_sec', 0.05).value
        self.vel_timeout_sec = self.declare_parameter('vel_timeout_sec', 0.5).value

        self.get_logger().info(f"--- Head Servo Parameters ---")
        self.get_logger().info(f"dx_path: {self.dx_path}")
        self.get_logger().info(f"pan_home_pulse: {self.pan_home_pulse}")
        self.get_logger().info(f"tilt_home_pulse: {self.tilt_home_pulse}")
        self.get_logger().info(f"------------------------------")

        self.target_pan_rad = 0.0
        self.target_tilt_rad = 0.0
        self.vel_cmd_pan = 0.0
        self.vel_cmd_tilt = 0.0
        self.last_vel_time = 0.0
        
        self.is_connected = False
        self.has_homed = False  # 起動時のソフトホーミング完了フラグ

        self.portHandler = PortHandler(self.dx_path)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        self.try_connect()

        self.pub_pan_raw = self.create_publisher(Int32, '/pan/present_position', 10)
        self.pub_tilt_raw = self.create_publisher(Int32, '/tilt/present_position', 10)
        self.pub_joint_state = self.create_publisher(JointState, '/joint_states', 10)

        self.create_subscription(Twist, '/servo_vel', self.vel_callback, 10, callback_group=self.callback_group)
        self.create_subscription(JointState, '/upper_joints_control', self.joint_control_callback, 10, callback_group=self.callback_group)
        
        self.create_service(MoveServo, '/move_servo', self.move_servo_callback, callback_group=self.callback_group)
        self.create_service(Trigger, '/calibrate_head', self.calibrate_callback, callback_group=self.callback_group)
        self.create_service(Trigger, '/manual_calibration', self.manual_calibration_callback, callback_group=self.callback_group)

        self.timer = self.create_timer(self.control_period_sec, self.timer_callback, callback_group=self.callback_group)

    def try_connect(self):
        """ 通信とトルクの初期化のみ（オリジナルの安定した挙動） """
        with self.lock:
            self.get_logger().info(f"Connecting to {self.dx_path}...")
            try:
                if self.portHandler.is_open:
                    self.portHandler.closePort()
                
                if self.portHandler.openPort():
                    if self.portHandler.setBaudRate(DEFAULT_BAUDRATE):
                        self.get_logger().info("Connected. Initializing motors...")
                        if self.setup_motor_locked(self.pan_id, "Pan") and self.setup_motor_locked(self.tilt_id, "Tilt"):
                            self.get_logger().info("Motors Ready!")
                            self.is_connected = True
                            return True
                self.get_logger().error("Failed to open port or set baudrate.")
            except Exception as e:
                self.get_logger().error(f"Connection Exception: {e}")
            
            self.is_connected = False
            return False

    def setup_motor_locked(self, dxl_id, name):
        try:
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, 0)
            time.sleep(0.02)
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_OPERATING_MODE, 3)
            time.sleep(0.02)
            self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, ADDR_PROFILE_VELOCITY, self.moving_speed)
            time.sleep(0.02)
            res, err = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, 1)
            if res == COMM_SUCCESS and err == 0:
                self.get_logger().info(f"[{name}] Velocity Profile set to {self.moving_speed}. Torque ON.")
                return True
        except Exception:
            pass
        self.get_logger().warn(f"[{name}] Setup failed. Will retry.")
        return False

    def set_torque(self, dxl_id, enable):
        if not self.is_connected: return False
        with self.lock:
            try:
                res, err = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, 1 if enable else 0)
                return res == COMM_SUCCESS and err == 0
            except Exception as e:
                self.handle_disconnect(e)
        return False

    def safe_read_pulse(self, dxl_id):
        if not self.is_connected: return None
        with self.lock:
            try:
                pos, res, err = self.packetHandler.read4ByteTxRx(self.portHandler, dxl_id, ADDR_PRESENT_POSITION)
                if res == COMM_SUCCESS:
                    return pos
            except Exception as e:
                self.handle_disconnect(e)
            return None

    def safe_write_pulse(self, dxl_id, pulse):
        if not self.is_connected: return
        pulse = int(pulse)
        # 上限下限での確実なクランプ
        if dxl_id == self.pan_id:
            pulse = max(self.pan_min_pulse, min(self.pan_max_pulse, pulse))
        elif dxl_id == self.tilt_id:
            pulse = max(self.tilt_min_pulse, min(self.tilt_max_pulse, pulse))
        
        with self.lock:
            try:
                self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, ADDR_GOAL_POSITION, pulse)
            except Exception as e:
                self.handle_disconnect(e)

    def handle_disconnect(self, e):
        if self.is_connected:
            self.get_logger().error(f"CRITICAL: Communication lost! ({e})")
            self.is_connected = False

    def rad_to_pulse(self, rad, home_pulse, direction, offset_rad):
        return int(home_pulse + ((rad - offset_rad) * (4096.0 / (2 * math.pi)) * direction))

    def pulse_to_rad(self, pulse, home_pulse, direction, offset_rad):
        return (pulse - home_pulse) * (2 * math.pi / 4096.0) * direction + offset_rad

    def write_position_rad(self, dxl_id, rad, home_pulse, direction, offset_rad):
        pulse = self.rad_to_pulse(rad, home_pulse, direction, offset_rad)
        self.safe_write_pulse(dxl_id, pulse)

    def publish_current_state(self):
        pan_pulse = self.safe_read_pulse(self.pan_id)
        tilt_pulse = self.safe_read_pulse(self.tilt_id)

        if pan_pulse is not None and tilt_pulse is not None:
            self.pub_pan_raw.publish(Int32(data=pan_pulse))
            self.pub_tilt_raw.publish(Int32(data=tilt_pulse))
            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.name = ["xl330_joint", "d455_joint"]
            p_rad = self.pulse_to_rad(pan_pulse, self.pan_home_pulse, self.pan_dir, self.pan_rad_offset)
            t_rad = self.pulse_to_rad(tilt_pulse, self.tilt_home_pulse, self.tilt_dir, self.tilt_rad_offset)
            joint_state.position = [p_rad, t_rad]
            self.pub_joint_state.publish(joint_state)
            return p_rad, t_rad
        return None, None

    def calibrate_callback(self, request, response):
        """ 物理破損を防ぐため自動キャリブレーションは無効化 """
        self.get_logger().warn("Auto-calibration is deprecated. Please use /manual_calibration.")
        response.success = False
        response.message = "Deprecated: Use /manual_calibration instead."
        return response

    def manual_calibration_callback(self, request, response):
        if not self.is_connected:
            response.success = False
            response.message = "Motor not connected"
            return response
        
        self.get_logger().info("Manual Calibration: Torque OFF. Please adjust the head to FACE FORWARD within 15 seconds.")
        self.set_torque(self.pan_id, False)
        self.set_torque(self.tilt_id, False)
        
        time.sleep(15.0)
        
        if not self.is_connected:
            response.success = False
            return response

        self.get_logger().info("Manual Calibration: Time is up. Fixing position...")
        pan_pos = self.safe_read_pulse(self.pan_id)
        tilt_pos = self.safe_read_pulse(self.tilt_id)
        
        if pan_pos is None or tilt_pos is None:
            response.success = False
            self.set_torque(self.pan_id, True)
            self.set_torque(self.tilt_id, True)
            return response
        
        self.set_parameters([
            Parameter('pan_home_pulse', Parameter.Type.INTEGER, pan_pos),
            Parameter('tilt_home_pulse', Parameter.Type.INTEGER, tilt_pos),
        ])
        
        self.set_torque(self.pan_id, True)
        self.set_torque(self.tilt_id, True)
        
        self.safe_write_pulse(self.pan_id, pan_pos)
        self.safe_write_pulse(self.tilt_id, tilt_pos)
        
        self.pan_home_pulse = pan_pos
        self.tilt_home_pulse = tilt_pos
        self.target_pan_rad = 0.0
        self.target_tilt_rad = 0.0
        
        result_msg = f"Calibration Finished. New zero points: Pan={pan_pos}, Tilt={tilt_pos}. PLEASE UPDATE YOUR YAML."
        self.get_logger().info(result_msg)
        response.success = True
        response.message = result_msg
        return response

    def move_servo_callback(self, request, response):
        if not self.has_homed:
            response.success = False
            return response
        self.target_pan_rad = request.pan
        self.target_tilt_rad = request.tilt
        self.write_position_rad(self.pan_id, self.target_pan_rad, self.pan_home_pulse, self.pan_dir, self.pan_rad_offset)
        self.write_position_rad(self.tilt_id, self.target_tilt_rad, self.tilt_home_pulse, self.tilt_dir, self.tilt_rad_offset)
        response.success = True
        return response

    def vel_callback(self, msg):
        self.vel_cmd_pan = msg.angular.z
        self.vel_cmd_tilt = msg.angular.y
        self.last_vel_time = time.time()

    def joint_control_callback(self, msg):
        if not self.has_homed: return
        for i, name in enumerate(msg.name):
            if name == "xl330_joint":
                self.target_pan_rad = msg.position[i]
                self.last_vel_time = 0.0 
            elif name == "d455_joint":
                self.target_tilt_rad = msg.position[i]
                self.last_vel_time = 0.0
        if self.is_connected:
             self.write_position_rad(self.pan_id, self.target_pan_rad, self.pan_home_pulse, self.pan_dir, self.pan_rad_offset)
             self.write_position_rad(self.tilt_id, self.target_tilt_rad, self.tilt_home_pulse, self.tilt_dir, self.tilt_rad_offset)

    def timer_callback(self):
        if not self.is_connected:
            self.try_connect()
            return
        
        try:
            # 起動直後に「現在の物理的な位置」から「正面」へゆっくりと補間移動させる（急激な過負荷を防ぐ）
            if not self.has_homed:
                if not hasattr(self, 'startup_homing_active'):
                    pan_pulse = self.safe_read_pulse(self.pan_id)
                    tilt_pulse = self.safe_read_pulse(self.tilt_id)
                    if pan_pulse is None or tilt_pulse is None:
                        return
                    
                    # 起動時の現在位置を初期目標値とする
                    self.target_pan_rad = self.pulse_to_rad(pan_pulse, self.pan_home_pulse, self.pan_dir, self.pan_rad_offset)
                    self.target_tilt_rad = self.pulse_to_rad(tilt_pulse, self.tilt_home_pulse, self.tilt_dir, self.tilt_rad_offset)
                    self.startup_homing_active = True
                    self.get_logger().info("Soft Homing Started: Moving to front smoothly...")

                # 毎秒0.5radの安全な速度で正面(0.0)へ近づける
                step = 0.5 * self.control_period_sec
                dp = 0.0 - self.target_pan_rad
                dtilt = 0.0 - self.target_tilt_rad
                
                if abs(dp) < step and abs(dtilt) < step:
                    self.target_pan_rad = 0.0
                    self.target_tilt_rad = 0.0
                    self.has_homed = True
                    self.last_vel_time = 0.0
                    self.get_logger().info("Soft Homing Complete. Ready for commands.")
                else:
                    self.target_pan_rad += max(-step, min(step, dp))
                    self.target_tilt_rad += max(-step, min(step, dtilt))
                
                self.write_position_rad(self.pan_id, self.target_pan_rad, self.pan_home_pulse, self.pan_dir, self.pan_rad_offset)
                self.write_position_rad(self.tilt_id, self.target_tilt_rad, self.tilt_home_pulse, self.tilt_dir, self.tilt_rad_offset)
                self.publish_current_state()
                return  # ホーミング中は外部からの指令を無視する
            
            # 通常時の動作
            if (time.time() - self.last_vel_time) < self.vel_timeout_sec:
                self.target_pan_rad  += self.vel_cmd_pan  * self.control_period_sec
                self.target_tilt_rad += self.vel_cmd_tilt * self.control_period_sec
                self.write_position_rad(self.pan_id, self.target_pan_rad, self.pan_home_pulse, self.pan_dir, self.pan_rad_offset)
                self.write_position_rad(self.tilt_id, self.target_tilt_rad, self.tilt_home_pulse, self.tilt_dir, self.tilt_rad_offset)
            
            self.publish_current_state()
        except Exception as e:
            self.handle_disconnect(e)

    def __del__(self):
        try:
            if self.is_connected:
                self.packetHandler.write1ByteTxRx(self.portHandler, self.pan_id, ADDR_TORQUE_ENABLE, 0)
                self.packetHandler.write1ByteTxRx(self.portHandler, self.tilt_id, ADDR_TORQUE_ENABLE, 0)
            self.portHandler.closePort()
        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = PanTiltNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Node crashed with exception: {e}", file=sys.stderr)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
