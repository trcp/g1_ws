import sys
import math
import time
import os
import yaml
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
from serial import SerialException

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
        
        # Communication lock for thread-safety
        self.lock = threading.Lock()
        
        # Callback Group for concurrent execution
        self.callback_group = ReentrantCallbackGroup()
        
        # Declare Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('dx_path', DEFAULT_DEVICENAME),
                ('pan_id', 1),
                ('tilt_id', 0),
                ('pan_home_pulse', 2535),
                ('tilt_home_pulse', 1895),
                ('pan_rad_offset', 0.0),
                ('tilt_rad_offset', 0.0),
                ('pan_dir', 1),
                ('tilt_dir', 1),
                ('pan_min_pulse', 0),
                ('pan_max_pulse', 4000),
                ('tilt_min_pulse', 1455),
                ('tilt_max_pulse', 4000),
                ('moving_speed', 60),
                ('calib_speed', 20),
                ('control_period_sec', 0.05),
                ('vel_timeout_sec', 0.5),
                ('load_threshold', 400),
                ('load_count_limit', 5)
            ]
        )

        self.target_pan_rad = 0.0
        self.target_tilt_rad = 0.0
        self.vel_cmd_pan = 0.0
        self.vel_cmd_tilt = 0.0
        self.last_vel_time = 0.0
        self.is_connected = False

        self.portHandler = PortHandler(self.get_parameter('dx_path').value)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        self.try_connect()

        self.pub_pan_raw = self.create_publisher(Int32, '/pan/present_position', 10)
        self.pub_tilt_raw = self.create_publisher(Int32, '/tilt/present_position', 10)
        self.pub_joint_state = self.create_publisher(JointState, '/joint_states', 10)

        self.create_subscription(Twist, '/servo_vel', self.vel_callback, 10, callback_group=self.callback_group)
        self.create_subscription(JointState, '/upper_joints_control', self.joint_control_callback, 10, callback_group=self.callback_group)
        
        self.create_service(MoveServo, '/move_servo', self.move_servo_callback, callback_group=self.callback_group)
        self.create_service(Trigger, '/calibrate_head', self.calibrate_callback, callback_group=self.callback_group)
        
        # Integrated Manual calibration service
        self.create_service(Trigger, '/manual_calibration', self.manual_calibration_callback, callback_group=self.callback_group)

        self.timer = self.create_timer(self.get_parameter('control_period_sec').value, self.timer_callback, callback_group=self.callback_group)

    def try_connect(self):
        with self.lock:
            dx_path = self.get_parameter('dx_path').value
            self.get_logger().info(f"Connecting to {dx_path}...")
            try:
                if self.portHandler.is_open:
                    self.portHandler.closePort()
                
                if self.portHandler.openPort():
                    if self.portHandler.setBaudRate(DEFAULT_BAUDRATE):
                        self.get_logger().info(f"Connected. Initializing motors...")
                        pan_id = self.get_parameter('pan_id').value
                        tilt_id = self.get_parameter('tilt_id').value
                        if self.setup_motor_locked(pan_id, "Pan") and self.setup_motor_locked(tilt_id, "Tilt"):
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
            moving_speed = self.get_parameter('moving_speed').value
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, 0)
            time.sleep(0.02)
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_OPERATING_MODE, 3)
            time.sleep(0.02)
            self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, ADDR_PROFILE_VELOCITY, moving_speed)
            time.sleep(0.02)
            res, err = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, 1)
            if res == COMM_SUCCESS and err == 0:
                self.get_logger().info(f"[{name}] Velocity Profile set to {moving_speed}. Torque ON.")
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

    def safe_read_load(self, dxl_id):
        if not self.is_connected: return 0
        with self.lock:
            try:
                load, res, err = self.packetHandler.read2ByteTxRx(self.portHandler, dxl_id, ADDR_PRESENT_LOAD)
                if res == COMM_SUCCESS:
                    if load > 32767: load -= 65536
                    return load
            except Exception as e:
                self.handle_disconnect(e)
            return 0

    def safe_write_pulse(self, dxl_id, pulse):
        if not self.is_connected: return
        pulse = int(pulse)
        if dxl_id == self.get_parameter('pan_id').value:
            pulse = max(self.get_parameter('pan_min_pulse').value, min(self.get_parameter('pan_max_pulse').value, pulse))
        elif dxl_id == self.get_parameter('tilt_id').value:
            pulse = max(self.get_parameter('tilt_min_pulse').value, min(self.get_parameter('tilt_max_pulse').value, pulse))
        
        with self.lock:
            try:
                self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, ADDR_GOAL_POSITION, pulse)
            except Exception as e:
                self.handle_disconnect(e)
    
    def set_profile_velocity(self, dxl_id, velocity):
        if not self.is_connected: return
        with self.lock:
            try:
                 self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, ADDR_PROFILE_VELOCITY, velocity)
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
        pan_id = self.get_parameter('pan_id').value
        tilt_id = self.get_parameter('tilt_id').value
        pan_pulse = self.safe_read_pulse(pan_id)
        tilt_pulse = self.safe_read_pulse(tilt_id)

        if pan_pulse is not None and tilt_pulse is not None:
            self.pub_pan_raw.publish(Int32(data=pan_pulse))
            self.pub_tilt_raw.publish(Int32(data=tilt_pulse))
            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.name = ["xl330_joint", "d455_joint"]
            p_rad = self.pulse_to_rad(pan_pulse, self.get_parameter('pan_home_pulse').value, self.get_parameter('pan_dir').value, self.get_parameter('pan_rad_offset').value)
            t_rad = self.pulse_to_rad(tilt_pulse, self.get_parameter('tilt_home_pulse').value, self.get_parameter('tilt_dir').value, self.get_parameter('tilt_rad_offset').value)
            joint_state.position = [p_rad, t_rad]
            self.pub_joint_state.publish(joint_state)
            return p_rad, t_rad
        return None, None

    def execute_calibration_for_joint(self, dxl_id, name):
        self.get_logger().info(f"Calibrating {name} (ID: {dxl_id})...")
        calib_speed = self.get_parameter('calib_speed').value
        load_threshold = self.get_parameter('load_threshold').value
        load_count_limit = self.get_parameter('load_count_limit').value
        
        if not self.is_connected: return 0, 0
        self.set_profile_velocity(dxl_id, calib_speed)
        self.get_logger().info(f"[{name}] Searching Upper Limit...")
        self.safe_write_pulse(dxl_id, 4000) 
        
        overload_count = 0
        detected_upper_pulse = 4000
        for _ in range(200):
            if not self.is_connected: break
            load = abs(self.safe_read_load(dxl_id))
            pos  = self.safe_read_pulse(dxl_id)
            if pos is None: continue
            if load > load_threshold:
                overload_count += 1
            else:
                overload_count = 0
            if overload_count > load_count_limit:
                detected_upper_pulse = pos
                self.get_logger().info(f"[{name}] Upper Limit Detected at {pos} (Load: {load})")
                self.safe_write_pulse(dxl_id, pos)
                break
            time.sleep(0.05)
            
        if not self.is_connected: return 0, 0
        time.sleep(1.0)
        self.safe_write_pulse(dxl_id, detected_upper_pulse - 200)
        time.sleep(2.0)
        
        self.get_logger().info(f"[{name}] Searching Lower Limit...")
        self.safe_write_pulse(dxl_id, 100)
        overload_count = 0
        detected_lower_pulse = 0
        for _ in range(200):
            if not self.is_connected: break
            load = abs(self.safe_read_load(dxl_id))
            pos  = self.safe_read_pulse(dxl_id)
            if pos is None: continue
            if load > load_threshold:
                overload_count += 1
            else:
                overload_count = 0
            if overload_count > load_count_limit:
                detected_lower_pulse = pos
                self.get_logger().info(f"[{name}] Lower Limit Detected at {pos} (Load: {load})")
                self.safe_write_pulse(dxl_id, pos)
                break
            time.sleep(0.05)
            
        self.set_profile_velocity(dxl_id, self.get_parameter('moving_speed').value)
        center = int((detected_upper_pulse + detected_lower_pulse) / 2)
        self.safe_write_pulse(dxl_id, center)
        return detected_lower_pulse, detected_upper_pulse

    def calibrate_callback(self, request, response):
        if not self.is_connected:
            response.success = False
            response.message = "Motor not connected"
            return response
        try:
            tilt_min, tilt_max = self.execute_calibration_for_joint(self.get_parameter('tilt_id').value, "Tilt")
            if not self.is_connected: raise RuntimeError("Lost connection during Tilt calibration")
            pan_min, pan_max = self.execute_calibration_for_joint(self.get_parameter('pan_id').value, "Pan")
            if not self.is_connected: raise RuntimeError("Lost connection during Pan calibration")
            
            response.success = True
            response.message = f"Tilt [{tilt_min}:{tilt_max}], Pan [{pan_min}:{pan_max}]"
        except Exception as e:
            response.success = False
            response.message = f"Calibration failed: {e}"
        return response

    def manual_calibration_callback(self, request, response):
        if not self.is_connected:
            response.success = False
            response.message = "Motor not connected"
            return response
        
        self.get_logger().info("Manual Calibration: Torque OFF. You have 15 seconds to adjust the head.")
        pan_id = self.get_parameter('pan_id').value
        tilt_id = self.get_parameter('tilt_id').value
        
        # Torque OFF
        self.set_torque(pan_id, False)
        self.set_torque(tilt_id, False)
        
        # Wait 15 seconds
        time.sleep(15.0)
        
        if not self.is_connected:
            response.success = False
            response.message = "Connection lost during manual calibration"
            return response

        self.get_logger().info("Manual Calibration: Time is up. Fixing position...")
        pan_pos = self.safe_read_pulse(pan_id)
        tilt_pos = self.safe_read_pulse(tilt_id)
        
        if pan_pos is None or tilt_pos is None:
            response.success = False
            response.message = "Failed to read final position"
            # Try to turn torque back on anyway for safety
            self.set_torque(pan_id, True)
            self.set_torque(tilt_id, True)
            return response
        
        # Update parameters in memory
        self.set_parameters([
            Parameter('pan_home_pulse', Parameter.Type.INTEGER, pan_pos),
            Parameter('tilt_home_pulse', Parameter.Type.INTEGER, tilt_pos),
            Parameter('pan_rad_offset', Parameter.Type.DOUBLE, 0.0),
            Parameter('tilt_rad_offset', Parameter.Type.DOUBLE, 0.0)
        ])
        
        # Torque ON
        self.set_torque(pan_id, True)
        self.set_torque(tilt_id, True)
        
        # Maintain current position
        self.safe_write_pulse(pan_id, pan_pos)
        self.safe_write_pulse(tilt_id, tilt_pos)
        
        self.target_pan_rad = 0.0
        self.target_tilt_rad = 0.0
        
        result_msg = f"Calibration Finished. New zero points: Pan={pan_pos}, Tilt={tilt_pos}. Please update YAML manually."
        self.get_logger().info(result_msg)
        response.success = True
        response.message = result_msg
        return response

    def move_servo_callback(self, request, response):
        target_pan = request.pan
        target_tilt = request.tilt
        self.get_logger().info(f"MoveServo: Pan={target_pan:.2f}, Tilt={target_tilt:.2f}")
        self.target_pan_rad = target_pan
        self.target_tilt_rad = target_tilt
        
        pan_id = self.get_parameter('pan_id').value
        tilt_id = self.get_parameter('tilt_id').value
        
        self.write_position_rad(pan_id, target_pan, self.get_parameter('pan_home_pulse').value, self.get_parameter('pan_dir').value, self.get_parameter('pan_rad_offset').value)
        self.write_position_rad(tilt_id, target_tilt, self.get_parameter('tilt_home_pulse').value, self.get_parameter('tilt_dir').value, self.get_parameter('tilt_rad_offset').value)
        
        success = self.wait_for_both_arrival(target_pan, target_tilt)
        response.success = success
        return response

    def wait_for_both_arrival(self, target_pan, target_tilt):
        start_time = time.time()
        while (time.time() - start_time) < 5.0:
            if not self.is_connected: return False
            curr_pan, curr_tilt = self.publish_current_state()
            if curr_pan is not None and curr_tilt is not None:
                offset_pan = self.get_parameter('pan_rad_offset').value
                offset_tilt = self.get_parameter('tilt_rad_offset').value
                err_pan = abs(target_pan - (curr_pan - offset_pan))
                err_tilt = abs(target_tilt - (curr_tilt - offset_tilt))
                if err_pan < 0.08 and err_tilt < 0.08:
                    return True
            time.sleep(0.05)
        return False

    def vel_callback(self, msg):
        self.vel_cmd_pan = msg.angular.z
        self.vel_cmd_tilt = msg.angular.y
        self.last_vel_time = time.time()

    def joint_control_callback(self, msg):
        for i, name in enumerate(msg.name):
            if name == "xl330_joint":
                self.target_pan_rad = msg.position[i]
                self.last_vel_time = 0.0 
            elif name == "d455_joint":
                self.target_tilt_rad = msg.position[i]
                self.last_vel_time = 0.0
        if self.is_connected:
             self.write_position_rad(self.get_parameter('pan_id').value, self.target_pan_rad, self.get_parameter('pan_home_pulse').value, self.get_parameter('pan_dir').value, self.get_parameter('pan_rad_offset').value)
             self.write_position_rad(self.get_parameter('tilt_id').value, self.target_tilt_rad, self.get_parameter('tilt_home_pulse').value, self.get_parameter('tilt_dir').value, self.get_parameter('tilt_rad_offset').value)

    def timer_callback(self):
        if not self.is_connected:
            self.try_connect()
            return
        
        try:
            vel_timeout = self.get_parameter('vel_timeout_sec').value
            control_period = self.get_parameter('control_period_sec').value
            
            if (time.time() - self.last_vel_time) < vel_timeout:
                self.target_pan_rad  += self.vel_cmd_pan  * control_period
                self.target_tilt_rad += self.vel_cmd_tilt * control_period
                self.write_position_rad(self.get_parameter('pan_id').value, self.target_pan_rad, self.get_parameter('pan_home_pulse').value, self.get_parameter('pan_dir').value, self.get_parameter('pan_rad_offset').value)
                self.write_position_rad(self.get_parameter('tilt_id').value, self.target_tilt_rad, self.get_parameter('tilt_home_pulse').value, self.get_parameter('tilt_dir').value, self.get_parameter('tilt_rad_offset').value)
            
            self.publish_current_state()
        except Exception as e:
            self.handle_disconnect(e)

    def __del__(self):
        try:
            if self.is_connected:
                self.packetHandler.write1ByteTxRx(self.portHandler, self.get_parameter('pan_id').value, ADDR_TORQUE_ENABLE, 0)
                self.packetHandler.write1ByteTxRx(self.portHandler, self.get_parameter('tilt_id').value, ADDR_TORQUE_ENABLE, 0)
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
