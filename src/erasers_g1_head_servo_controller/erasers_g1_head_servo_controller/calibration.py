#!/usr/bin/env python3
"""
Calibration script for pan-tilt Dynamixel head.

Usage:
    python calibration.py

Torque is released so you can manually move each motor to its
mechanical zero position. Press Enter to record, then the raw
encoder value is saved as the zero-offset in calibration.txt.
"""

import sys
import time
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS
import calib_config


def open_port() -> tuple[PortHandler, PacketHandler]:
    port = PortHandler(calib_config.DEVICE_NAME)
    packet = PacketHandler(calib_config.PROTOCOL_VER)

    if not port.openPort():
        sys.exit("[ERROR] Failed to open port: " + calib_config.DEVICE_NAME)
    if not port.setBaudRate(calib_config.BAUDRATE):
        sys.exit("[ERROR] Failed to set baud rate: " + str(calib_config.BAUDRATE))

    print(f"Port {calib_config.DEVICE_NAME} opened at {calib_config.BAUDRATE} bps")
    return port, packet


def set_torque(port: PortHandler, packet: PacketHandler, dxl_id: int, enable: bool) -> None:
    val = calib_config.TORQUE_ENABLE if enable else calib_config.TORQUE_DISABLE
    result, error = packet.write1ByteTxRx(port, dxl_id, calib_config.ADDR_TORQUE_ENABLE, val)
    if result != COMM_SUCCESS:
        print(f"[WARN] ID {dxl_id} torque write failed: {packet.getTxRxResult(result)}")
    elif error:
        print(f"[WARN] ID {dxl_id} torque error: {packet.getRxPacketError(error)}")


def read_position(port: PortHandler, packet: PacketHandler, dxl_id: int) -> int:
    pos, result, error = packet.read4ByteTxRx(port, dxl_id, calib_config.ADDR_PRESENT_POSITION)
    if result != COMM_SUCCESS:
        sys.exit(f"[ERROR] ID {dxl_id} read position failed: {packet.getTxRxResult(result)}")
    if error:
        print(f"[WARN] ID {dxl_id} position error: {packet.getRxPacketError(error)}")
    return pos


def calibrate_motor(port: PortHandler, packet: PacketHandler, name: str, dxl_id: int) -> int:
    print(f"\n--- {name} (ID {dxl_id}) calibration ---")
    set_torque(port, packet, dxl_id, False)
    print(f"Torque released. Move the {name} motor to its MECHANICAL ZERO position.")
    input("Press Enter when ready...")

    set_torque(port, packet, dxl_id, True)
    time.sleep(5)
    pos = read_position(port, packet, dxl_id)
    print(f"Recorded zero offset: {pos} units  ({pos * calib_config.DEG_PER_UNIT:.2f} deg raw)")
    time.sleep(5)
    set_torque(port, packet, dxl_id, False)
    return pos


def save_calibration(pan_zero: int, tilt_zero: int) -> None:
    with open(calib_config.CALIB_FILE, "w") as f:
        f.write(f"pan_zero={pan_zero}\n")
        f.write(f"tilt_zero={tilt_zero}\n")
    print(f"\nCalibration saved to {calib_config.CALIB_FILE}")
    print(f"  pan_zero  = {pan_zero}")
    print(f"  tilt_zero = {tilt_zero}")


def main() -> None:
    print("=== Pan-Tilt Dynamixel Calibration ===")
    port, packet = open_port()

    set_torque(port, packet, 1, False)
    set_torque(port, packet, 2, False)

    try:
        pan_zero  = calibrate_motor(port, packet, "PAN",  calib_config.ID_PAN)
        tilt_zero = calibrate_motor(port, packet, "TILT", calib_config.ID_TILT)
        save_calibration(pan_zero, tilt_zero)

        print("\nCalibration complete. Motors are free.")
    finally:
        port.closePort()


if __name__ == "__main__":
    main()
