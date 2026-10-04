#!/usr/bin/env python3

import math
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

# Dynamixel control table addresses
ADDR_TORQUE_ENABLE    = 64
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION    = 116
ADDR_PRESENT_POSITION = 132

DEG_PER_PULSE = 0.088   # [deg/pulse]
PROFILE_VELOCITY = 1000

PAN_MIN_RAD  = math.radians(-40.0)
PAN_MAX_RAD  = math.radians( 40.0)
TILT_MIN_RAD = math.radians(-80.0)
TILT_MAX_RAD = math.radians( 10.0)

class HeadServoNode(Node):
    def __init__(self):
        super().__init__('head_servo_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('pan_id', 1)
        self.declare_parameter('tilt_id', 2)
        self.declare_parameter(
            'calibration_file',
            os.path.join(os.path.dirname(__file__), 'calibration.txt'),
        )
        self.declare_parameter('publish_rate', 10.0)

        port     = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self.pan_id  = self.get_parameter('pan_id').value
        self.tilt_id = self.get_parameter('tilt_id').value
        calib_file   = self.get_parameter('calibration_file').value
        pub_rate     = self.get_parameter('publish_rate').value

        # --- Dynamixel setup ---
        self.port_handler   = PortHandler(port)
        self.packet_handler = PacketHandler(2.0)

        if not self.port_handler.openPort():
            self.get_logger().fatal(f'Failed to open port: {port}')
            raise RuntimeError('Port open failed')

        if not self.port_handler.setBaudRate(baudrate):
            self.get_logger().fatal(f'Failed to set baudrate: {baudrate}')
            raise RuntimeError('Baudrate set failed')

        self.get_logger().info(f'Opened {port} at {baudrate} bps')

        # --- Calibration ---
        self.pan_limit  = None
        self.tilt_limit = None
        self._load_calibration(calib_file)

        # --- Enable torque ---
        self._set_torque(self.pan_id,  1)
        self._set_torque(self.tilt_id, 1)

        # --- ROS interfaces ---
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.sub = self.create_subscription(
            JointState, '/upper_joints_control', self._cmd_callback, 10)

        self.create_timer(1.0 / pub_rate, self._publish_joint_states)

        self.get_logger().info('head_servo_node ready')

    # ------------------------------------------------------------------
    # Calibration
    # ------------------------------------------------------------------

    def _load_calibration(self, path: str):
        path = os.path.realpath(path)
        try:
            with open(path) as f:
                for line in f:
                    if 'pan' in line:
                        self.pan_limit = int(line.split('=')[-1])
                    elif 'tilt' in line:
                        self.tilt_limit = int(line.split('=')[-1])
        except FileNotFoundError:
            self.get_logger().fatal(f'calibration.txt not found: {path}')
            raise

        self.get_logger().info(
            f'Calibration loaded — pan_limit={self.pan_limit}, tilt_limit={self.tilt_limit}')

    # ------------------------------------------------------------------
    # Dynamixel helpers
    # ------------------------------------------------------------------

    def _set_torque(self, dxl_id: int, enable: int):
        result, error = self.packet_handler.write1ByteTxRx(
            self.port_handler, dxl_id, ADDR_TORQUE_ENABLE, enable)
        self._check_comm(result, error, f'set_torque id={dxl_id}')

    def _read_position(self, dxl_id: int):
        value, result, error = self.packet_handler.read4ByteTxRx(
            self.port_handler, dxl_id, ADDR_PRESENT_POSITION)
        if not self._check_comm(result, error, f'read_position id={dxl_id}'):
            return None
        # read4ByteTxRx returns unsigned; convert to signed 32-bit
        if value > 0x7FFFFFFF:
            value -= 0x100000000
        return value

    def _write_position(self, dxl_id: int, goal_pulse: int):
        self.packet_handler.write4ByteTxRx(
            self.port_handler, dxl_id, ADDR_PROFILE_VELOCITY, PROFILE_VELOCITY)
        result, error = self.packet_handler.write4ByteTxRx(
            self.port_handler, dxl_id, ADDR_GOAL_POSITION, goal_pulse)
        self._check_comm(result, error, f'write_position id={dxl_id}')

    def _check_comm(self, result, error, label: str) -> bool:
        if result != COMM_SUCCESS:
            self.get_logger().error(
                f'[{label}] {self.packet_handler.getTxRxResult(result)}')
            return False
        if error != 0:
            self.get_logger().error(
                f'[{label}] {self.packet_handler.getRxPacketError(error)}')
            return False
        return True

    # ------------------------------------------------------------------
    # Unit conversions
    # ------------------------------------------------------------------
    # Calibration positions:
    #   pan  limit = -50 deg  →  pan_limit  pulse
    #   tilt limit = -90 deg  →  tilt_limit pulse

    def _pan_pulse_to_rad(self, pulse: int) -> float:
        zero_pulse = 50.0 / DEG_PER_PULSE + self.pan_limit
        return math.radians((pulse - zero_pulse) * DEG_PER_PULSE)

    def _tilt_pulse_to_rad(self, pulse: int) -> float:
        zero_pulse = 90.0 / DEG_PER_PULSE + self.tilt_limit
        return math.radians((pulse - zero_pulse) * DEG_PER_PULSE)

    def _pan_rad_to_pulse(self, rad: float) -> int:
        zero_pulse = 50.0 / DEG_PER_PULSE + self.pan_limit
        return int(zero_pulse + math.degrees(rad) / DEG_PER_PULSE)

    def _tilt_rad_to_pulse(self, rad: float) -> int:
        zero_pulse = 90.0 / DEG_PER_PULSE + self.tilt_limit
        return int(zero_pulse + math.degrees(rad) / DEG_PER_PULSE)

    # ------------------------------------------------------------------
    # Callbacks / timer
    # ------------------------------------------------------------------

    def _publish_joint_states(self):
        pan_pulse  = self._read_position(self.pan_id)
        tilt_pulse = self._read_position(self.tilt_id)
        if pan_pulse is None or tilt_pulse is None:
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        # msg.name     = ['pan_joint', 'tilt_joint']
        msg.name     = ['xl330_joint', 'd455_joint', 'xl330_pulse', 'd455_pulse']
        msg.position = [
            self._pan_pulse_to_rad(pan_pulse),
            self._tilt_pulse_to_rad(tilt_pulse),
        ]
        print("pan_pulse_pos: ", pan_pulse, "tilt_pulse_pos: ", tilt_pulse)
        self.pub.publish(msg)

    def _cmd_callback(self, msg: JointState):
        for name, pos_rad in zip(msg.name, msg.position):
            if name == 'xl330_joint':
                pos_rad = max(PAN_MIN_RAD, min(PAN_MAX_RAD, pos_rad))
                pulse = self._pan_rad_to_pulse(pos_rad)
                self._write_position(self.pan_id, pulse)
                self.get_logger().info(
                    f'pan  -> {math.degrees(pos_rad):.1f} deg (pulse {pulse})')
            elif name == 'd455_joint':
                pos_rad = max(TILT_MIN_RAD, min(TILT_MAX_RAD, pos_rad))
                pulse = self._tilt_rad_to_pulse(pos_rad)
                self._write_position(self.tilt_id, pulse)
                self.get_logger().info(
                    f'tilt -> {math.degrees(pos_rad):.1f} deg (pulse {pulse})')

    # ------------------------------------------------------------------

    def destroy_node(self):
        self._set_torque(self.pan_id,  0)
        self._set_torque(self.tilt_id, 0)
        self.port_handler.closePort()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HeadServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
