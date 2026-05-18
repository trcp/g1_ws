#!/usr/bin/env python3

import math
import sys
from dynamixel_sdk import *


portHandler = PortHandler("/dev/ttyCH341USB0")
packetHandler = PacketHandler(2.0)

if portHandler.openPort():
  print("Succeeded to open the port!")
else:
  print("Failed to open the port!")
  exit()

if portHandler.setBaudRate(1000000):
  print("Succeeded to change the baudrate!")
else:
  print("Failed to change the baudrate!")
  exit()

TORQUE_ON_ADDRESS = 64

pan_dxl_id = 1
tilt_dxl_id = 2

PAN_LIMIT = [4000]
TILT_LIMTI = []

TORQUE_ON = 1
TORQUE_OFF = 0

dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, pan_dxl_id, TORQUE_ON_ADDRESS, TORQUE_ON)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel has been successfully connected")

dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, tilt_dxl_id, TORQUE_ON_ADDRESS, TORQUE_ON)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel has been successfully connected")

while True:
    try:
        target_position = int(input("Enter target position (0 ~ 4095, -1 to exit): "))
    except ValueError:
        print("Please enter an integer.")
        continue

    if target_position == -1:
        break
    elif target_position < 0 or target_position > 4095:
        print("Position must be between 0 and 4095.")
        continue

    goal_position_address = 116
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, pan_dxl_id, goal_position_address, target_position)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    while True:
        present_position_address = 132
        present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, pan_dxl_id, present_position_address)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print(f"Current Position: {present_position}")

        if abs(target_position - present_position) <= 10:
            break
