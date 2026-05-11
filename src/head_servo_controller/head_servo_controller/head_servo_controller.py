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
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from dynamixel_sdk import *

# ==========================================
#               Dynamixel Constants
# ==========================================
PROTOCOL_VERSION = 2.0

# XL330/XC330 Control Table Addresses (Protocol 2.0)
ADDR_OPERATING_MODE    = 11    # RW, 1 byte
ADDR_TORQUE_ENABLE     = 64    # RW, 1 byte
ADDR_BUS_WATCHDOG      = 98    # RW, 2 byte
ADDR_PROFILE_VELOCITY  = 112   # RW, 4 byte
ADDR_GOAL_POSITION     = 116   # RW, 4 byte
ADDR_MOVING            = 122   # RO, 1 byte
ADDR_MOVING_STATUS     = 123   # RO, 1 byte
ADDR_PRESENT_POSITION  = 132   # RO, 4 byte
ADDR_HOMING_OFFSET     = 20    # RW, 4 byte, EEPROM
ADDR_CURRENT_LIMIT     = 38    # RW, 2 byte, EEPROM
ADDR_PRESENT_CURRENT   = 126   # RO, 2 byte
ADDR_PRESENT_VOLTAGE   = 144   # RO, 2 byte  ← [NEW]
ADDR_PRESENT_TEMPERATURE = 146 # RO, 1 byte  ← [NEW]

# Operating Modes
OPMODE_CURRENT = 0
OPMODE_VELOCITY = 1
OPMODE_POSITION = 3

# Torque values
TORQUE_OFF = 0
TORQUE_ON = 1

# XL330/XC330 mechanical specs
XL330_PULSE_PER_REV = 4096
XL330_RAD_PER_PULSE = 2.0 * math.pi / XL330_PULSE_PER_REV

# [FIX] XC330-M288: Current Limit unit = 2.69mA (NOT 1.17mA!)
XC330_CURRENT_UNIT_mA = 2.69

# Dynamixel Protocol 2.0 Error Codes (for debugging)
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
    """Decode Dynamixel hardware error code to human-readable string"""
    if err_code == 0:
        return "OK"
    errors = [name for bit, name in DXL_ERRORS.items() if err_code & bit]
    return f"{err_code}(0x{err_code:02X}): {', '.join(errors)}"


class PanTiltNode(Node):
    def __init__(self):
        super().__init__('head_servo_controller')

        self.lock = threading.Lock()
        self.callback_group = ReentrantCallbackGroup()

        # ===== Parameter Declaration =====
        self.declare_parameter('dx_path', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('protocol_version', 2.0)
        self.declare_parameter('pan_id', 1)
        self.declare_parameter('tilt_id', 2)
        self.declare_parameter('pan_home_pulse', 0)
        self.declare_parameter('tilt_home_pulse', 0)
        self.declare_parameter('pan_rad_offset', 0.0)
        self.declare_parameter('tilt_rad_offset', 0.0)
        self.declare_parameter('pan_dir', 1)
        self.declare_parameter('tilt_dir', 1)
        self.declare_parameter('pan_min_pulse', 0)
        self.declare_parameter('pan_max_pulse', 4095)
        self.declare_parameter('tilt_min_pulse', 0)
        self.declare_parameter('tilt_max_pulse', 4095)
        
        # [FIX] Conservative defaults for testing
        self.declare_parameter('moving_speed', 300)
        # [FIX] Current limit in mA (XC330: max recommended 2700mA)
        self.declare_parameter('current_limit_mA', 1500)  # ← 安全値に引き下げ
        
        self.declare_parameter('control_period_sec', 0.02)
        self.declare_parameter('vel_timeout_sec', 0.5)
        self.declare_parameter('bus_watchdog_ms', 100)
        self.declare_parameter('use_sync_read', True)
        # [NEW] Enable hardware error monitoring
        self.declare_parameter('monitor_hardware', True)

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
        self.current_limit_mA = self.get_parameter('current_limit_mA').value
        # [FIX] Correct unit conversion for XC330-M288
        self.current_limit_units = max(1, min(1008, int(self.current_limit_mA / XC330_CURRENT_UNIT_mA)))
        
        self.control_period_sec = self.get_parameter('control_period_sec').value
        self.vel_timeout_sec = self.get_parameter('vel_timeout_sec').value
        self.bus_watchdog_ms = self.get_parameter('bus_watchdog_ms').value
        self.use_sync_read = self.get_parameter('use_sync_read').value
        self.monitor_hardware = self.get_parameter('monitor_hardware').value

        self.get_logger().info(f"=== Head Servo Controller Initialized ===")
        self.get_logger().info(f"Device: {self.dx_path}, Baud: {self.baudrate}")
        self.get_logger().info(f"Pan ID: {self.pan_id}, Tilt ID: {self.tilt_id}")
        self.get_logger().info(f"Profile Velocity: {self.moving_speed} pulse/s, Current Limit: {self.current_limit_mA} mA ({self.current_limit_units} units)")

        # ===== State Variables =====
        self.target_pan_rad = 0.0
        self.target_tilt_rad = 0.0
        self.vel_cmd_pan = 0.0
        self.vel_cmd_tilt = 0.0
        self.last_vel_time = 0.0
        self.is_connected = False
        self.has_homed = False

        # ===== Dynamixel SDK Setup =====
        self.portHandler = PortHandler(self.dx_path)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        self.groupSyncRead = None

        # ===== Connection & Initialization =====
        self.try_connect()

        # ===== ROS Publishers =====
        self.pub_joint_state = self.create_publisher(JointState, '/joint_states', 10)

        # ===== ROS Subscribers =====
        self.create_subscription(
            JointState, '/upper_joints_control',
            self.joint_control_callback, 10,
            callback_group=self.callback_group
        )
        self.create_subscription(
            Twist, '/servo_vel',
            self.vel_callback, 10,
            callback_group=self.callback_group
        )

        # ===== ROS Services =====
        self.create_service(
            Trigger, '/calibrate_head',
            self.calibrate_callback,
            callback_group=self.callback_group
        )

        # ===== Timer =====
        self.timer = self.create_timer(
            self.control_period_sec, self.timer_callback,
            callback_group=self.callback_group
        )

        self.get_logger().info("=== Ready ===")

    def try_connect(self):
        """Initialize port, baudrate, and motor setup"""
        with self.lock:
            try:
                if self.portHandler.is_open:
                    self.portHandler.closePort()

                if not self.portHandler.openPort():
                    self.get_logger().error(f"Failed to open port: {self.dx_path}")
                    return False

                if not self.portHandler.setBaudRate(self.baudrate):
                    self.get_logger().error(f"Failed to set baudrate: {self.baudrate}")
                    return False

                self.get_logger().info("Port opened. Initializing servos...")
                
                if not (self.setup_motor(self.pan_id, "Pan") and 
                        self.setup_motor(self.tilt_id, "Tilt")):
                    return False

                # Setup Bus Watchdog for safety
                for dxl_id, name in [(self.pan_id, "Pan"), (self.tilt_id, "Tilt")]:
                    result, error = self.packetHandler.write2ByteTxRx(
                        self.portHandler, dxl_id, ADDR_BUS_WATCHDOG,
                        max(1, self.bus_watchdog_ms // 20)
                    )
                    if result != COMM_SUCCESS:
                        self.get_logger().warn(f"[{name}] Bus Watchdog setup failed: result={result}")

                # Re-initialize GroupSyncRead after successful connection
                self.groupSyncRead = GroupSyncRead(
                    self.portHandler, self.packetHandler,
                    ADDR_PRESENT_POSITION, 4
                )
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
        """Configure a single motor with hardware error checking"""
        try:
            # 1. Torque OFF
            result, error = self.packetHandler.write1ByteTxRx(
                self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_OFF
            )
            if result != COMM_SUCCESS:
                raise RuntimeError(f"Failed to disable torque: result={result}, err={decode_dxl_error(error)}")
            time.sleep(0.01)

            # [NEW] Check hardware status before configuration
            if self.monitor_hardware:
                voltage, res, err = self.packetHandler.read2ByteTxRx(
                    self.portHandler, dxl_id, ADDR_PRESENT_VOLTAGE
                )
                if res == COMM_SUCCESS:
                    # Voltage unit: 0.1V, so 50 = 5.0V
                    self.get_logger().info(f"[{name}] Present Voltage: {voltage * 0.1:.1f}V")
                    if voltage < 47 or voltage > 53:  # Outside 4.7V-5.3V
                        self.get_logger().warn(f"[{name}] Voltage out of range! Check power supply.")
                
                temp, res, err = self.packetHandler.read1ByteTxRx(
                    self.portHandler, dxl_id, ADDR_PRESENT_TEMPERATURE
                )
                if res == COMM_SUCCESS:
                    self.get_logger().info(f"[{name}] Present Temperature: {temp}°C")
                    if temp > 70:
                        self.get_logger().warn(f"[{name}] Temperature high! Allow to cool.")

            # 2. Set Operating Mode to Extended Position Control
            result, error = self.packetHandler.write1ByteTxRx(
                self.portHandler, dxl_id, ADDR_OPERATING_MODE, OPMODE_POSITION
            )
            if result != COMM_SUCCESS:
                raise RuntimeError(f"Failed to set position mode: result={result}, err={decode_dxl_error(error)}")
            time.sleep(0.01)

            # 3. Set Profile Velocity
            result, error = self.packetHandler.write4ByteTxRx(
                self.portHandler, dxl_id, ADDR_PROFILE_VELOCITY, self.moving_speed
            )
            if result != COMM_SUCCESS:
                raise RuntimeError(f"Failed to set profile velocity: result={result}, err={decode_dxl_error(error)}")
            time.sleep(0.01)

            # 4. Set Current Limit [FIX: correct unit conversion + safe default]
            result, error = self.packetHandler.write2ByteTxRx(
                self.portHandler, dxl_id, ADDR_CURRENT_LIMIT, self.current_limit_units
            )
            if result != COMM_SUCCESS:
                self.get_logger().warn(f"[{name}] Failed to set current limit: result={result}, err={decode_dxl_error(error)}")
            else:
                actual_mA = self.current_limit_units * XC330_CURRENT_UNIT_mA
                self.get_logger().info(f"[{name}] Current Limit set: {self.current_limit_units} units ≈ {actual_mA:.0f} mA")
            time.sleep(0.01)

            # 5. Torque ON
            result, error = self.packetHandler.write1ByteTxRx(
                self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ON
            )
            if result != COMM_SUCCESS:
                raise RuntimeError(f"Failed to enable torque: result={result}, err={decode_dxl_error(error)}")
            if error != 0:
                self.get_logger().warn(f"[{name}] Torque ON returned error: {decode_dxl_error(error)}")
                # Continue anyway - some errors may be transient

            self.get_logger().info(f"[{name}] Setup complete. Torque ON.")
            return True

        except Exception as e:
            self.get_logger().error(f"[{name}] Setup failed: {e}")
            return False

    def set_torque(self, dxl_id, enable):
        """Safely toggle torque with lock"""
        if not self.is_connected:
            return False
        with self.lock:
            try:
                result, error = self.packetHandler.write1ByteTxRx(
                    self.portHandler, dxl_id, ADDR_TORQUE_ENABLE,
                    TORQUE_ON if enable else TORQUE_OFF
                )
                if error != 0:
                    self.get_logger().debug(f"Torque toggle error for ID {dxl_id}: {decode_dxl_error(error)}")
                return result == COMM_SUCCESS
            except Exception as e:
                self.get_logger().error(f"Torque toggle failed: {e}")
                self.is_connected = False
                return False

    def read_positions_sync(self):
        """Read present positions using GroupSyncRead"""
        if not self.is_connected or self.groupSyncRead is None:
            return None, None

        with self.lock:
            try:
                result = self.groupSyncRead.txRxPacket()
                if result != COMM_SUCCESS:
                    for dxl_id in [self.pan_id, self.tilt_id]:
                        if not self.groupSyncRead.isAvailable(dxl_id, ADDR_PRESENT_POSITION, 4):
                            self.get_logger().debug(f"SyncRead failed for ID {dxl_id}")
                    return None, None

                pan_pulse = self.groupSyncRead.getData(
                    self.pan_id, ADDR_PRESENT_POSITION, 4
                )
                tilt_pulse = self.groupSyncRead.getData(
                    self.tilt_id, ADDR_PRESENT_POSITION, 4
                )
                return pan_pulse, tilt_pulse

            except Exception as e:
                self.get_logger().error(f"SyncRead exception: {e}")
                self.is_connected = False
                return None, None

    def write_position_pulse(self, dxl_id, pulse):
        """Write goal position with limit clamping and error handling"""
        if not self.is_connected:
            return

        original_pulse = pulse
        if dxl_id == self.pan_id:
            pulse = max(self.pan_min_pulse, min(self.pan_max_pulse, int(pulse)))
        elif dxl_id == self.tilt_id:
            pulse = max(self.tilt_min_pulse, min(self.tilt_max_pulse, int(pulse)))
        
        if pulse != original_pulse:
            self.get_logger().debug(f"[ID:{dxl_id}] Pulse clamped: {original_pulse} → {pulse}")

        with self.lock:
            try:
                result, error = self.packetHandler.write4ByteTxRx(
                    self.portHandler, dxl_id, ADDR_GOAL_POSITION, pulse
                )
                if result != COMM_SUCCESS:
                    self.get_logger().warn(f"Write COMM error for ID {dxl_id}: result={result}")
                elif error != 0:
                    # [IMPORTANT] Log hardware errors for debugging
                    self.get_logger().warn(f"Write hardware error for ID {dxl_id}: {decode_dxl_error(error)}")
                    # Don't set is_connected=False for hardware errors - may be transient
                else:
                    self.get_logger().debug(f"[ID:{dxl_id}] Position written: {pulse} pulse")
            except Exception as e:
                self.get_logger().error(f"Write exception: {e}")
                self.is_connected = False

    def rad_to_pulse(self, rad, home_pulse, direction, offset_rad):
        """Convert radian to pulse count"""
        raw_pulse = (rad - offset_rad) / XL330_RAD_PER_PULSE
        return int(home_pulse + raw_pulse * direction)

    def pulse_to_rad(self, pulse, home_pulse, direction, offset_rad):
        """Convert pulse count to radian"""
        raw_rad = (pulse - home_pulse) * XL330_RAD_PER_PULSE * direction
        return raw_rad + offset_rad

    def publish_joint_state(self, pan_pulse, tilt_pulse):
        """Publish current joint state to /joint_states"""
        if pan_pulse is None or tilt_pulse is None:
            return

        pan_rad = self.pulse_to_rad(
            pan_pulse, self.pan_home_pulse, self.pan_dir, self.pan_rad_offset
        )
        tilt_rad = self.pulse_to_rad(
            tilt_pulse, self.tilt_home_pulse, self.tilt_dir, self.tilt_rad_offset
        )

        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ["xl330_pan_joint", "xl330_tilt_joint"]
        joint_state.position = [pan_rad, tilt_rad]
        joint_state.velocity = []
        joint_state.effort = []
        self.pub_joint_state.publish(joint_state)

    # ===== Callbacks =====

    def vel_callback(self, msg: Twist):
        """Velocity command callback"""
        self.vel_cmd_pan = msg.angular.z
        self.vel_cmd_tilt = msg.angular.y
        self.last_vel_time = time.time()

    def joint_control_callback(self, msg: JointState):
        """Position command via JointState message"""
        updated = False
        for i, name in enumerate(msg.name):
            if name == "xl330_pan_joint" and i < len(msg.position):
                self.target_pan_rad = msg.position[i]
                updated = True
            elif name == "xl330_tilt_joint" and i < len(msg.position):
                self.target_tilt_rad = msg.position[i]
                updated = True
        
        if updated:
            self.last_vel_time = 0.0

    def calibrate_callback(self, request, response):
        """Manual calibration service"""
        if not self.is_connected:
            response.success = False
            response.message = "Motors not connected"
            return response

        self.get_logger().info("=== Manual Calibration Started ===")
        self.get_logger().info("Step 1: Torque OFF. Position head FORWARD within 15 seconds.")
        
        self.set_torque(self.pan_id, False)
        self.set_torque(self.tilt_id, False)
        time.sleep(0.1)
        time.sleep(15.0)

        if not self.is_connected:
            response.success = False
            response.message = "Connection lost during calibration"
            return response

        self.get_logger().info("Step 2: Reading current positions as new zero points...")
        
        pan_pulse, tilt_pulse = self.read_positions_sync()
        if pan_pulse is None or tilt_pulse is None:
            response.success = False
            response.message = "Failed to read positions"
            self.set_torque(self.pan_id, True)
            self.set_torque(self.tilt_id, True)
            return response

        self.get_logger().info(f"Step 3: Writing Homing Offset to EEPROM...")
        
        success = True
        for dxl_id, name, current_pulse in [
            (self.pan_id, "Pan", pan_pulse),
            (self.tilt_id, "Tilt", tilt_pulse)
        ]:
            old_offset, res, err = self.packetHandler.read4ByteTxRx(
                self.portHandler, dxl_id, ADDR_HOMING_OFFSET
            )
            if res != COMM_SUCCESS:
                self.get_logger().warn(f"[{name}] Failed to read old offset")
                old_offset = 0
            
            new_offset = old_offset - current_pulse
            
            self.packetHandler.write1ByteTxRx(
                self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_OFF
            )
            time.sleep(0.01)
            
            res, err = self.packetHandler.write4ByteTxRx(
                self.portHandler, dxl_id, ADDR_HOMING_OFFSET, new_offset
            )
            if res != COMM_SUCCESS or err != 0:
                self.get_logger().error(f"[{name}] Failed to write Homing Offset: {decode_dxl_error(err)}")
                success = False
            else:
                self.get_logger().info(f"[{name}] Homing Offset updated: {old_offset} → {new_offset}")
            
            time.sleep(0.01)
            self.packetHandler.write1ByteTxRx(
                self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ON
            )

        if not success:
            response.success = False
            response.message = "Failed to write Homing Offset to EEPROM"
            return response

        self.set_parameters([
            Parameter('pan_home_pulse', Parameter.Type.INTEGER, 0),
            Parameter('tilt_home_pulse', Parameter.Type.INTEGER, 0),
        ])
        self.pan_home_pulse = 0
        self.tilt_home_pulse = 0

        self.get_logger().info("Step 4: Rebooting servos to apply changes...")
        self.packetHandler.reboot(self.portHandler, self.pan_id)
        self.packetHandler.reboot(self.portHandler, self.tilt_id)
        time.sleep(1.0)

        self.setup_motor(self.pan_id, "Pan")
        self.setup_motor(self.tilt_id, "Tilt")

        self.target_pan_rad = 0.0
        self.target_tilt_rad = 0.0
        self.has_homed = True

        response.success = True
        response.message = "Calibration complete!"
        return response

    def timer_callback(self):
        """Main control loop"""
        if not self.is_connected:
            self.try_connect()
            return

        try:
            if not self.has_homed:
                if not hasattr(self, '_startup_homing_init'):
                    pan_pulse, tilt_pulse = self.read_positions_sync()
                    if pan_pulse is None or tilt_pulse is None:
                        return
                    
                    self.target_pan_rad = self.pulse_to_rad(
                        pan_pulse, self.pan_home_pulse, self.pan_dir, self.pan_rad_offset
                    )
                    self.target_tilt_rad = self.pulse_to_rad(
                        tilt_pulse, self.tilt_home_pulse, self.tilt_dir, self.tilt_rad_offset
                    )
                    self._startup_homing_init = True
                    self.get_logger().info("Soft homing: moving to neutral position...")
                
                step_rad = 0.5 * self.control_period_sec
                dp = 0.0 - self.target_pan_rad
                dt = 0.0 - self.target_tilt_rad
                
                if abs(dp) < step_rad and abs(dt) < step_rad:
                    self.target_pan_rad = 0.0
                    self.target_tilt_rad = 0.0
                    self.has_homed = True
                    self.last_vel_time = 0.0
                    self.get_logger().info("Soft homing complete. Ready for commands.")
                else:
                    self.target_pan_rad += max(-step_rad, min(step_rad, dp))
                    self.target_tilt_rad += max(-step_rad, min(step_rad, dt))
                
                self.write_position_pulse(self.pan_id, self.rad_to_pulse(
                    self.target_pan_rad, self.pan_home_pulse, self.pan_dir, self.pan_rad_offset))
                self.write_position_pulse(self.tilt_id, self.rad_to_pulse(
                    self.target_tilt_rad, self.tilt_home_pulse, self.tilt_dir, self.tilt_rad_offset))
                
                pan_pulse, tilt_pulse = self.read_positions_sync()
                self.publish_joint_state(pan_pulse, tilt_pulse)
                return

            # Normal operation
            if (time.time() - self.last_vel_time) < self.vel_timeout_sec:
                self.target_pan_rad += self.vel_cmd_pan * self.control_period_sec
                self.target_tilt_rad += self.vel_cmd_tilt * self.control_period_sec
            
            pan_pulse = self.rad_to_pulse(
                self.target_pan_rad, self.pan_home_pulse, self.pan_dir, self.pan_rad_offset)
            tilt_pulse = self.rad_to_pulse(
                self.target_tilt_rad, self.tilt_home_pulse, self.tilt_dir, self.tilt_rad_offset)
            
            self.write_position_pulse(self.pan_id, pan_pulse)
            self.write_position_pulse(self.tilt_id, tilt_pulse)

            pan_pulse, tilt_pulse = self.read_positions_sync()
            self.publish_joint_state(pan_pulse, tilt_pulse)

        except Exception as e:
            self.get_logger().error(f"Control loop error: {e}", stack_info=True)
            self.is_connected = False

    def __del__(self):
        """Cleanup"""
        try:
            if self.is_connected:
                self.set_torque(self.pan_id, False)
                self.set_torque(self.tilt_id, False)
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
    except Exception as e:
        node.get_logger().error(f"Fatal error: {e}", stack_info=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
