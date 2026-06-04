#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler
from g1_srvs.srv import MoveServo


PROTOCOL_VERSION = 2.0

ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132

OPERATING_MODE_POSITION = 3
TORQUE_OFF = 0
TORQUE_ON = 1

PULSE_PER_REV = 4096
RAD_PER_PULSE = 2.0 * math.pi / PULSE_PER_REV
MIN_POSITION_PULSE = 0
MAX_POSITION_PULSE = PULSE_PER_REV - 1
RECONNECT_PERIOD_SEC = 1.0

PAN_JOINT_NAME = 'xl330_joint'
TILT_JOINT_NAME = 'd455_joint'


class HeadServoNode(Node):
    def __init__(self):
        super().__init__('head_servo_node')

        self.declare_parameter('dx_path', '/dev/ttyCH341USB0')
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('pan_id', 1)
        self.declare_parameter('tilt_id', 2)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('pan_zero', 945)
        self.declare_parameter('tilt_zero', 1061)
        self.declare_parameter('pan_zero_rad', -0.8727)
        self.declare_parameter('tilt_zero_rad', -1.5708)
        self.declare_parameter('pan_min_rad', -0.8727)
        self.declare_parameter('pan_max_rad', 0.8727)
        self.declare_parameter('tilt_min_rad', -1.57)
        self.declare_parameter('tilt_max_rad', 1.0)
        self.declare_parameter('pan_dir', 1)
        self.declare_parameter('tilt_dir', 1)
        self.declare_parameter('profile_velocity', 1000)

        self.dx_path = self.get_parameter('dx_path').value
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.pan_id = int(self.get_parameter('pan_id').value)
        self.tilt_id = int(self.get_parameter('tilt_id').value)
        publish_rate = float(self.get_parameter('publish_rate').value)
        self.pan_zero = int(self.get_parameter('pan_zero').value)
        self.tilt_zero = int(self.get_parameter('tilt_zero').value)
        self.pan_zero_rad = float(self.get_parameter('pan_zero_rad').value)
        self.tilt_zero_rad = float(self.get_parameter('tilt_zero_rad').value)
        self.pan_min_rad = float(self.get_parameter('pan_min_rad').value)
        self.pan_max_rad = float(self.get_parameter('pan_max_rad').value)
        self.tilt_min_rad = float(self.get_parameter('tilt_min_rad').value)
        self.tilt_max_rad = float(self.get_parameter('tilt_max_rad').value)
        self.pan_dir = self._normalize_direction(
            int(self.get_parameter('pan_dir').value), 'pan_dir')
        self.tilt_dir = self._normalize_direction(
            int(self.get_parameter('tilt_dir').value), 'tilt_dir')
        self.profile_velocity = max(
            0, int(self.get_parameter('profile_velocity').value))

        if publish_rate <= 0.0:
            self.get_logger().warn(
                f'Invalid publish_rate={publish_rate}; using 10.0 Hz')
            publish_rate = 10.0

        self.port_handler = PortHandler(self.dx_path)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)
        self.port_open = False
        self.is_connected = False
        self.last_connect_attempt = 0.0

        self.joint_state_pub = self.create_publisher(
            JointState, '/joint_states', 10)
        self.create_subscription(
            JointState,
            '/upper_joints_control',
            self._joint_command_callback,
            10)
        self.create_service(
            MoveServo, '/move_servo', self._move_servo_callback)
        self.create_timer(1.0 / publish_rate, self._timer_callback)

        self.get_logger().info(
            'head_servo_node configured: '
            f'dx_path={self.dx_path}, baudrate={self.baudrate}, '
            f'pan_id={self.pan_id}, tilt_id={self.tilt_id}')
        self._connect(force=True)

    def _normalize_direction(self, value: int, name: str) -> int:
        if value not in (-1, 1):
            normalized = 1 if value >= 0 else -1
            self.get_logger().warn(
                f'{name} must be 1 or -1; '
                f'using {normalized} instead of {value}')
            return normalized
        return value

    def _connect(self, force: bool = False) -> bool:
        if self.is_connected:
            return True

        now = time.monotonic()
        if (
            not force
            and (now - self.last_connect_attempt) < RECONNECT_PERIOD_SEC
        ):
            return False
        self.last_connect_attempt = now

        self._close_port(disable_torque=False)

        if not self.port_handler.openPort():
            self.get_logger().warn(
                f'Failed to open Dynamixel port: {self.dx_path}')
            return False
        self.port_open = True

        if not self.port_handler.setBaudRate(self.baudrate):
            self.get_logger().warn(f'Failed to set baudrate: {self.baudrate}')
            self._close_port(disable_torque=False)
            return False

        if not self._setup_motor(self.pan_id, 'pan'):
            self._close_port(disable_torque=True)
            return False
        if not self._setup_motor(self.tilt_id, 'tilt'):
            self._close_port(disable_torque=True)
            return False

        self.is_connected = True
        self.get_logger().info(
            f'Dynamixel connected on {self.dx_path} at {self.baudrate} bps')
        return True

    def _setup_motor(self, dxl_id: int, label: str) -> bool:
        if not self._write1(
            dxl_id,
            ADDR_TORQUE_ENABLE,
            TORQUE_OFF,
            f'{label} torque off',
        ):
            return False
        time.sleep(0.02)

        if not self._write1(
            dxl_id, ADDR_OPERATING_MODE, OPERATING_MODE_POSITION,
            f'{label} position mode'
        ):
            return False
        time.sleep(0.02)

        if not self._write4(
            dxl_id, ADDR_PROFILE_VELOCITY, self.profile_velocity,
            f'{label} profile velocity'
        ):
            return False
        time.sleep(0.01)

        present_pulse = self._read_position_unchecked(dxl_id, label)
        if present_pulse is None:
            return False
        if not self._write4(
            dxl_id,
            ADDR_GOAL_POSITION,
            present_pulse,
            f'{label} hold present position',
        ):
            return False
        time.sleep(0.01)

        if not self._write1(
            dxl_id,
            ADDR_TORQUE_ENABLE,
            TORQUE_ON,
            f'{label} torque on',
        ):
            return False
        time.sleep(0.01)
        return True

    def _write1(
        self,
        dxl_id: int,
        address: int,
        value: int,
        label: str,
    ) -> bool:
        result, error = self.packet_handler.write1ByteTxRx(
            self.port_handler, dxl_id, address, value)
        return self._check_comm(result, error, label)

    def _write4(
        self,
        dxl_id: int,
        address: int,
        value: int,
        label: str,
    ) -> bool:
        result, error = self.packet_handler.write4ByteTxRx(
            self.port_handler, dxl_id, address, int(value))
        return self._check_comm(result, error, label)

    def _read_position_unchecked(self, dxl_id: int, label: str):
        value, result, error = self.packet_handler.read4ByteTxRx(
            self.port_handler, dxl_id, ADDR_PRESENT_POSITION)
        if not self._check_comm(
            result,
            error,
            f'{label} read present position',
        ):
            return None
        return self._to_signed_32(value)

    def _read_position(self, dxl_id: int):
        if not self.is_connected:
            return None
        value = self._read_position_unchecked(dxl_id, f'id={dxl_id}')
        if value is None:
            self.is_connected = False
            return None
        return value

    def _write_position(self, dxl_id: int, goal_pulse: int) -> bool:
        if not self._connect():
            return False

        goal_pulse = max(
            MIN_POSITION_PULSE, min(MAX_POSITION_PULSE, int(goal_pulse)))
        if not self._write4(
            dxl_id, ADDR_GOAL_POSITION, goal_pulse,
            f'write goal position id={dxl_id}'
        ):
            self.is_connected = False
            return False
        return True

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

    def _to_signed_32(self, value: int) -> int:
        if value > 0x7FFFFFFF:
            value -= 0x100000000
        return value

    def _pulse_to_rad(
        self,
        pulse: int,
        zero_pulse: int,
        zero_rad: float,
        direction: int,
    ) -> float:
        return ((pulse - zero_pulse) * RAD_PER_PULSE * direction) + zero_rad

    def _rad_to_pulse(
        self,
        rad: float,
        zero_pulse: int,
        zero_rad: float,
        direction: int,
    ) -> int:
        pulse_delta = direction * (rad - zero_rad) / RAD_PER_PULSE
        return int(round(zero_pulse + pulse_delta))

    def _clip_rad(
        self,
        value: float,
        lower: float,
        upper: float,
        axis: str,
    ) -> float:
        clipped = max(lower, min(upper, float(value)))
        if clipped != value:
            self.get_logger().warn(
                f'{axis} command clipped: {value:.4f} -> {clipped:.4f} rad')
        return clipped

    def _command_pan(self, pan_rad: float) -> bool:
        pan_rad = self._clip_rad(
            pan_rad, self.pan_min_rad, self.pan_max_rad, 'pan')
        pan_pulse = self._rad_to_pulse(
            pan_rad,
            self.pan_zero,
            self.pan_zero_rad,
            self.pan_dir)
        ok = self._write_position(self.pan_id, pan_pulse)
        # if ok:
        #     self.get_logger().info(
        #         f'pan -> {pan_rad:.4f} rad ({math.degrees(pan_rad):.1f} deg), '
        #         f'pulse={pan_pulse}')
        return ok

    def _command_tilt(self, tilt_rad: float) -> bool:
        tilt_rad = self._clip_rad(
            tilt_rad, self.tilt_min_rad, self.tilt_max_rad, 'tilt')
        tilt_pulse = self._rad_to_pulse(
            tilt_rad,
            self.tilt_zero,
            self.tilt_zero_rad,
            self.tilt_dir)
        ok = self._write_position(self.tilt_id, tilt_pulse)
        # if ok:
        #     self.get_logger().info(
        #         f'tilt -> {tilt_rad:.4f} rad '
        #         f'({math.degrees(tilt_rad):.1f} deg), '
        #         f'pulse={tilt_pulse}')
        return ok

    def _joint_command_callback(self, msg: JointState) -> None:
        success = True
        for name, pos_rad in zip(msg.name, msg.position):
            if name == PAN_JOINT_NAME:
                success = self._command_pan(pos_rad) and success
            elif name == TILT_JOINT_NAME:
                success = self._command_tilt(pos_rad) and success
        if not success:
            self.get_logger().warn(
                'Failed to apply one or more joint commands')

    def _move_servo_callback(self, request, response):
        pan_ok = self._command_pan(request.pan)
        tilt_ok = self._command_tilt(request.tilt)
        response.success = bool(pan_ok and tilt_ok)
        return response

    def _timer_callback(self) -> None:
        if not self._connect():
            return

        pan_pulse = self._read_position(self.pan_id)
        tilt_pulse = self._read_position(self.tilt_id)
        if pan_pulse is None or tilt_pulse is None:
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [PAN_JOINT_NAME, TILT_JOINT_NAME]
        msg.position = [
            self._pulse_to_rad(
                pan_pulse,
                self.pan_zero,
                self.pan_zero_rad,
                self.pan_dir),
            self._pulse_to_rad(
                tilt_pulse,
                self.tilt_zero,
                self.tilt_zero_rad,
                self.tilt_dir),
        ]
        self.joint_state_pub.publish(msg)

    def _close_port(self, disable_torque: bool = True) -> None:
        if self.port_open and disable_torque:
            for dxl_id in (self.pan_id, self.tilt_id):
                try:
                    self.packet_handler.write1ByteTxRx(
                        self.port_handler,
                        dxl_id,
                        ADDR_TORQUE_ENABLE,
                        TORQUE_OFF)
                except Exception as exc:
                    self.get_logger().debug(
                        f'Ignoring torque off failure for id={dxl_id}: {exc}')

        if self.port_open:
            try:
                self.port_handler.closePort()
            except Exception as exc:
                self.get_logger().debug(f'Ignoring port close failure: {exc}')

        self.port_open = False
        self.is_connected = False

    def destroy_node(self):
        self._close_port(disable_torque=True)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HeadServoNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
