# src/erasers_g1/head_servo_controller/head_servo_controller/head_servo_controller.py

import sys
import math
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from dynamixel_sdk import *
from std_msgs.msg import Float32MultiArray

# ==========================================
#               Dynamixel Constants
# ==========================================
PROTOCOL_VERSION = 2.0

# XL330 Control Table Addresses (Protocol 2.0)
ADDR_OPERATING_MODE         = 11    # RW, 1 byte, EEPROM
ADDR_HOMING_OFFSET          = 20    # RW, 4 byte, EEPROM
ADDR_CURRENT_LIMIT          = 38    # RW, 2 byte, EEPROM
ADDR_SHUTDOWN               = 63    # RW, 1 byte, EEPROM
ADDR_TORQUE_ENABLE          = 64    # RW, 1 byte, RAM
ADDR_HARDWARE_ERROR_STATUS  = 70    # RO, 1 byte, RAM
ADDR_BUS_WATCHDOG           = 98    # RW, 2 byte, RAM
ADDR_PROFILE_ACCELERATION   = 108   # RW, 4 byte, RAM
ADDR_PROFILE_VELOCITY       = 112   # RW, 4 byte, RAM
ADDR_GOAL_POSITION          = 116   # RW, 4 byte, RAM
ADDR_MOVING                 = 122   # RO, 1 byte, RAM
ADDR_MOVING_STATUS          = 123   # RO, 1 byte, RAM
ADDR_PRESENT_CURRENT        = 126   # RO, 2 byte, RAM
ADDR_PRESENT_POSITION       = 132   # RO, 4 byte, RAM
ADDR_PRESENT_VOLTAGE        = 144   # RO, 2 byte, RAM
ADDR_PRESENT_TEMPERATURE    = 146   # RO, 1 byte, RAM

# Operating Modes
OPMODE_POSITION = 3

# Torque values
TORQUE_OFF = 0
TORQUE_ON = 1

# XL330 mechanical specs
XL330_PULSE_PER_REV = 4096
XL330_RAD_PER_PULSE = 2.0 * math.pi / XL330_PULSE_PER_REV
XL330_CURRENT_UNIT_mA = 1.0

# Dynamixel Protocol 2.0 Error Codes
DXL_ERRORS = {
    0x01: "InputVoltage",
    0x02: "HallSensor",
    0x04: "Overheat",
    0x08: "MotorShaft",
    0x10: "ElectronicShock",
    0x20: "ControlTable",
    0x40: "Overload",
}

def decode_dxl_error(err_code: int) -> str:
    if err_code == 0:
        return "OK"
    errors = [name for bit, name in DXL_ERRORS.items() if err_code & bit]
    return f"{err_code}(0x{err_code:02X}): {', '.join(errors)}"

def to_signed_16(value: int) -> int:
    return value - 65536 if value > 32767 else value

class PanTiltNode(Node):
    def __init__(self):
        super().__init__('head_servo_controller')

        self.lock = threading.RLock()
        self.callback_group = ReentrantCallbackGroup()

        # ===== Parameter Declaration =====
        self.declare_parameter('dx_path', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('pan_id', 1)
        self.declare_parameter('tilt_id', 2)
        
        self.declare_parameter('pan_home_pulse', 2048)
        self.declare_parameter('tilt_home_pulse', 2048)
        
        self.declare_parameter('pan_rad_offset', 0.0)
        self.declare_parameter('tilt_rad_offset', 0.0)
        self.declare_parameter('pan_dir', 1)
        self.declare_parameter('tilt_dir', 1)
        
        self.declare_parameter('pan_min_pulse', 1024)
        self.declare_parameter('pan_max_pulse', 3072)
        self.declare_parameter('tilt_min_pulse', 1024)
        self.declare_parameter('tilt_max_pulse', 3072)
        
        self.declare_parameter('moving_speed', 200)
        self.declare_parameter('moving_accel', 50)
        self.declare_parameter('current_limit_mA', 400)
        
        self.declare_parameter('control_period_sec', 0.02)
        self.declare_parameter('vel_timeout_sec', 0.5)
        self.declare_parameter('bus_watchdog_ms', 100)
        
        self.declare_parameter('debug_power_monitor', True)
        self.declare_parameter('power_monitor_period_sec', 0.1)

        # ===== Load Parameters =====
        self.dx_path = self.get_parameter('dx_path').value
        self.baudrate = self.get_parameter('baudrate').value
        self.pan_id = self.get_parameter('pan_id').value
        self.tilt_id = self.get_parameter('tilt_id').value
        self.pan_home_pulse = self.get_parameter('pan_home_pulse').value
        self.tilt_home_pulse = self.get_parameter('tilt_home_pulse').value
        self.pan_rad_offset = self.get_parameter('pan_rad_offset').value
        self.tilt_rad_offset = self.get_parameter('tilt_rad_offset').value
        self.pan_dir = self.get_parameter('pan_dir').value
        self.tilt_dir = self.get_parameter('tilt_dir').value
        self.pan_min_pulse = self.get_parameter('pan_min_pulse').value
        self.pan_max_pulse = self.get_parameter('pan_max_pulse').value
        self.tilt_min_pulse = self.get_parameter('tilt_min_pulse').value
        self.tilt_max_pulse = self.get_parameter('tilt_max_pulse').value
        
        self.moving_speed = self.get_parameter('moving_speed').value
        self.moving_accel = self.get_parameter('moving_accel').value
        self.current_limit_mA = self.get_parameter('current_limit_mA').value
        self.current_limit_units = max(1, min(1750, int(self.current_limit_mA / XL330_CURRENT_UNIT_mA)))
        
        self.control_period_sec = self.get_parameter('control_period_sec').value
        self.vel_timeout_sec = self.get_parameter('vel_timeout_sec').value
        self.bus_watchdog_ms = self.get_parameter('bus_watchdog_ms').value
        
        self.debug_power_monitor = self.get_parameter('debug_power_monitor').value
        self.power_monitor_period = self.get_parameter('power_monitor_period_sec').value

        self.get_logger().info(f"=== Head Servo Controller Initialized ===")
        self.get_logger().info(f"Device: {self.dx_path}, Baud: {self.baudrate}")
        self.get_logger().info(f"Home Pulse -> Pan: {self.pan_home_pulse}, Tilt: {self.tilt_home_pulse}")

        self.target_pan_rad = 0.0
        self.target_tilt_rad = 0.0
        self.vel_cmd_pan = 0.0
        self.vel_cmd_tilt = 0.0
        self.last_vel_time = 0.0
        self.is_connected = False
        self.has_initialized_pos = False

        self.portHandler = PortHandler(self.dx_path)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        self.groupSyncRead = None

        connect_res = self.try_connect()
        self.get_logger().info(f"connection result: {connect_res}")

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )

        self.pub_joint_state = self.create_publisher(JointState, '/joint_states', sensor_qos)
        
        if self.debug_power_monitor:
            self.pub_power_debug = self.create_publisher(Float32MultiArray, '/servo_power_debug', 10)

        self.create_subscription(JointState, '/upper_joints_control', self.joint_control_callback, 10, callback_group=self.callback_group)
        self.create_subscription(Twist, '/servo_vel', self.vel_callback, 10, callback_group=self.callback_group)
        self.create_service(Trigger, '/calibrate_head', self.calibrate_callback, callback_group=self.callback_group)
        self.create_service(Trigger, '/check_servo_power', self.check_power_callback, callback_group=self.callback_group)

        self.timer = self.create_timer(self.control_period_sec, self.timer_callback, callback_group=self.callback_group)
        
        if self.debug_power_monitor:
            self.power_timer = self.create_timer(self.power_monitor_period, self.power_monitor_callback, callback_group=self.callback_group)

        self.get_logger().info("=== Ready ===")

    def get_detailed_power_status(self) -> str:
        status_lines = []
        pan_pulse, tilt_pulse = self.read_positions_sync()
        
        # [FIXED] Tuple unpacking error resolved by adding pan_pulse and tilt_pulse
        for dxl_id, name, pulse in [(self.pan_id, "Pan", pan_pulse), (self.tilt_id, "Tilt", tilt_pulse)]:
            if not self.is_connected:
                return "Not connected"
            with self.lock:
                voltage, res, _ = self.packetHandler.read2ByteTxRx(self.portHandler, dxl_id, ADDR_PRESENT_VOLTAGE)
                temp, res2, _ = self.packetHandler.read1ByteTxRx(self.portHandler, dxl_id, ADDR_PRESENT_TEMPERATURE)
                current_raw, res3, _ = self.packetHandler.read2ByteTxRx(self.portHandler, dxl_id, ADDR_PRESENT_CURRENT)
                error, res4, _ = self.packetHandler.read1ByteTxRx(self.portHandler, dxl_id, ADDR_HARDWARE_ERROR_STATUS)
            
            voltage_v = voltage * 0.1 if res == COMM_SUCCESS else -1
            current_ma = to_signed_16(current_raw) * XL330_CURRENT_UNIT_mA if res3 == COMM_SUCCESS else -1
            
            home_pulse = self.pan_home_pulse if name == "Pan" else self.tilt_home_pulse
            dir_val = self.pan_dir if name == "Pan" else self.tilt_dir
            offset = self.pan_rad_offset if name == "Pan" else self.tilt_rad_offset
            current_rad = self.pulse_to_rad(pulse, home_pulse, dir_val, offset) if pulse is not None else 0.0
            
            status_lines.append(f"\n=== {name} (ID:{dxl_id}) ===")
            status_lines.append(f"Position: {current_rad:.3f} rad (raw pulse: {pulse})")
            status_lines.append(f"Voltage: {voltage_v:.1f}V")
            status_lines.append(f"Temperature: {temp}°C")
            status_lines.append(f"Current: {current_ma:.0f}mA")
            status_lines.append(f"Error Status: {decode_dxl_error(error) if error else 'OK'}")
            status_lines.append(f"Current Limit: {self.current_limit_units} units")
        return "\n".join(status_lines)

    def power_monitor_callback(self):
        if not self.is_connected:
            return
        status_msg = self.get_detailed_power_status()
        if "Not connected" in status_msg:
            return
            
        if int(time.time()) % 5 == 0:
            self.get_logger().info(f"\n{status_msg}")

    def check_power_callback(self, request, response):
        response.success = True
        response.message = self.get_detailed_power_status()
        return response

    def try_connect(self):
        with self.lock:
            try:
                if self.portHandler.is_open:
                    self.portHandler.closePort()
                if not self.portHandler.openPort():
                    return False
                if not self.portHandler.setBaudRate(self.baudrate):
                    return False

                if not (self.setup_motor(self.pan_id, "Pan") and self.setup_motor(self.tilt_id, "Tilt")):
                    return False

                for dxl_id in [self.pan_id, self.tilt_id]:
                    self.packetHandler.write2ByteTxRx(self.portHandler, dxl_id, ADDR_BUS_WATCHDOG, max(1, self.bus_watchdog_ms // 20))

                self.groupSyncRead = GroupSyncRead(self.portHandler, self.packetHandler, ADDR_PRESENT_POSITION, 4)
                self.groupSyncRead.addParam(self.pan_id)
                self.groupSyncRead.addParam(self.tilt_id)

                self.is_connected = True
                self.get_logger().info("All motors ready!")
                return True
            except Exception as e:
                self.get_logger().error(f"Connection error: {e}")
                self.is_connected = False
                return False

    def setup_motor(self, dxl_id, name):
        try:
            with self.lock:
                self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_OFF)
                time.sleep(0.02) 

                error_reg, _, _ = self.packetHandler.read1ByteTxRx(self.portHandler, dxl_id, ADDR_HARDWARE_ERROR_STATUS)
                if error_reg != 0:
                    self.get_logger().warn(f"[{name}] Hardware errors present: {decode_dxl_error(error_reg)}")
                    self.packetHandler.reboot(self.portHandler, dxl_id)
                    time.sleep(1.0)
                    self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_OFF)
                    time.sleep(0.02)

                op_mode, _, _ = self.packetHandler.read1ByteTxRx(self.portHandler, dxl_id, ADDR_OPERATING_MODE)
                if op_mode != OPMODE_POSITION:
                    self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_OPERATING_MODE, OPMODE_POSITION)
                    time.sleep(0.02)

                current_limit_val, _, _ = self.packetHandler.read2ByteTxRx(self.portHandler, dxl_id, ADDR_CURRENT_LIMIT)
                if current_limit_val != self.current_limit_units:
                    res, err = self.packetHandler.write2ByteTxRx(self.portHandler, dxl_id, ADDR_CURRENT_LIMIT, self.current_limit_units)
                    if res != COMM_SUCCESS:
                        self.get_logger().error(f"[{name}] Failed to write Current Limit: {res}")
                    time.sleep(0.02)

                self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, ADDR_PROFILE_ACCELERATION, self.moving_accel)
                self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, ADDR_PROFILE_VELOCITY, self.moving_speed)
                time.sleep(0.01)

                self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ON)
                time.sleep(0.01)

            self.get_logger().info(f"[{name}] Setup complete. Torque ON.")
            return True
        except Exception as e:
            self.get_logger().error(f"[{name}] Setup failed: {e}")
            return False

    def _to_signed_32(self, value):
        if value > 0x7FFFFFFF:  # 2147483647 (32ビットの正の最大値) を超える場合はマイナス値
            value -= 0x100000000  # 4294967296 を引いて負の値に補正
        return value
        
    def read_positions_sync(self):
        if not self.is_connected or self.groupSyncRead is None:
            return None, None
        with self.lock:
            try:
                result = self.groupSyncRead.txRxPacket()
                if result != COMM_SUCCESS:
                    return None, None
                raw_pan = self.groupSyncRead.getData(self.pan_id, ADDR_PRESENT_POSITION, 4)
                raw_tilt = self.groupSyncRead.getData(self.tilt_id, ADDR_PRESENT_POSITION, 4)
                pan_pulse = self._to_signed_32(raw_pan)
                tilt_pulse = self._to_signed_32(raw_tilt)
                return pan_pulse, tilt_pulse
            except Exception:
                self.is_connected = False
                return None, None

    def write_position_pulse(self, dxl_id, pulse):
        if not self.is_connected:
            return
        
        if dxl_id == self.pan_id:
            pulse = max(self.pan_min_pulse, min(self.pan_max_pulse, int(pulse)))
        elif dxl_id == self.tilt_id:
            pulse = max(self.tilt_min_pulse, min(self.tilt_max_pulse, int(pulse)))

        with self.lock:
            try:
                self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, ADDR_GOAL_POSITION, pulse)
            except Exception:
                self.is_connected = False

    def rad_to_pulse(self, rad, home_pulse, direction, offset_rad):
        return int(home_pulse + ((rad - offset_rad) / XL330_RAD_PER_PULSE) * direction)

    def pulse_to_rad(self, pulse, home_pulse, direction, offset_rad):
        return ((pulse - home_pulse) * XL330_RAD_PER_PULSE * direction) + offset_rad

    def publish_joint_state(self, pan_pulse, tilt_pulse):
        if pan_pulse is None or tilt_pulse is None:
            return
        pan_rad = self.pulse_to_rad(pan_pulse, self.pan_home_pulse, self.pan_dir, self.pan_rad_offset)
        tilt_rad = self.pulse_to_rad(tilt_pulse, self.tilt_home_pulse, self.tilt_dir, self.tilt_rad_offset)
        
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ["xl330_joint", "d455_joint"]
        joint_state.position = [pan_rad, tilt_rad]
        self.pub_joint_state.publish(joint_state)

    def vel_callback(self, msg: Twist):
        self.vel_cmd_pan = msg.angular.z
        self.vel_cmd_tilt = msg.angular.y
        self.last_vel_time = time.time()

    def joint_control_callback(self, msg: JointState):
        updated = False
        for i, name in enumerate(msg.name):
            if name == "xl330_joint" and i < len(msg.position):
                self.target_pan_rad = msg.position[i]
                updated = True
            elif name == "d455_joint" and i < len(msg.position):
                self.target_tilt_rad = msg.position[i]
                updated = True
        if updated:
            self.last_vel_time = 0.0

    def calibrate_callback(self, request, response):
        pass

    def timer_callback(self):
        if not self.is_connected:
            self.try_connect()
            return
        try:
            if not self.has_initialized_pos:
                pan_pulse, tilt_pulse = self.read_positions_sync()
                if pan_pulse is None or tilt_pulse is None: 
                    return
                
                self.target_pan_rad = self.pulse_to_rad(pan_pulse, self.pan_home_pulse, self.pan_dir, self.pan_rad_offset)
                self.target_tilt_rad = self.pulse_to_rad(tilt_pulse, self.tilt_home_pulse, self.tilt_dir, self.tilt_rad_offset)
                
                self.has_initialized_pos = True
                self.get_logger().info(f"Startup position held: Pan={self.target_pan_rad:.3f} rad, Tilt={self.target_tilt_rad:.3f} rad")
                return

            if (time.time() - self.last_vel_time) < self.vel_timeout_sec:
                self.target_pan_rad += self.vel_cmd_pan * self.control_period_sec
                self.target_tilt_rad += self.vel_cmd_tilt * self.control_period_sec

            self.write_position_pulse(self.pan_id, self.rad_to_pulse(self.target_pan_rad, self.pan_home_pulse, self.pan_dir, self.pan_rad_offset))
            self.write_position_pulse(self.tilt_id, self.rad_to_pulse(self.target_tilt_rad, self.tilt_home_pulse, self.tilt_dir, self.tilt_rad_offset))

            pan_pulse, tilt_pulse = self.read_positions_sync()
            self.publish_joint_state(pan_pulse, tilt_pulse)

        except Exception as e:
            self.get_logger().error(f"Control loop error: {e}")
            self.is_connected = False

    def __del__(self):
        try:
            if self.is_connected:
                with self.lock:
                    self.packetHandler.write1ByteTxRx(self.portHandler, self.pan_id, ADDR_TORQUE_ENABLE, TORQUE_OFF)
                    self.packetHandler.write1ByteTxRx(self.portHandler, self.tilt_id, ADDR_TORQUE_ENABLE, TORQUE_OFF)
            if self.portHandler.is_open:
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
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
