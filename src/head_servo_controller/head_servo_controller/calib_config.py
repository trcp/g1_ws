#!/usr/bin/env python3

# --- Port / Protocol ---
DEVICE_NAME   = "/dev/ttyCH341USB0"
BAUDRATE      = 1000000
PROTOCOL_VER  = 2.0

# --- Motor IDs ---
ID_PAN  = 1
ID_TILT = 2

# --- Control table addresses (Protocol 2.0 / XM-XL series) ---
ADDR_OPERATING_MODE  = 11
ADDR_TORQUE_ENABLE   = 64
ADDR_GOAL_POSITION   = 116
ADDR_PRESENT_POSITION = 132

OPERATING_MODE_POSITION = 3   # position control mode
TORQUE_ENABLE  = 1
TORQUE_DISABLE = 0

# --- Position conversion ---
# 1 unit = 0.088 deg, centre (0 deg mechanical) = 2048
DEG_PER_UNIT = 0.088
UNIT_PER_DEG = 1.0 / DEG_PER_UNIT
CENTER_UNIT  = 2048

# --- Axis direction (+1: pulse increases with positive deg, -1: pulse decreases) ---
# Pan:  CW = negative deg, pulse increases CW → pulse decreases CCW (positive deg) → -1
# Tilt: to be confirmed
PAN_DIRECTION  = -1
TILT_DIRECTION =  1


# --- Physical range [deg] ---
PAN_PHYS_MIN  = -50.0
PAN_PHYS_MAX  =  50.0
TILT_PHYS_MIN = -90.0
TILT_PHYS_MAX =  22.7

# --- Software limits (10 deg margin) ---
SW_MARGIN     = 10.0
PAN_SW_MIN    = PAN_PHYS_MIN  + SW_MARGIN   # -40.0
PAN_SW_MAX    = PAN_PHYS_MAX  - SW_MARGIN   #  40.0
TILT_SW_MIN   = TILT_PHYS_MIN + SW_MARGIN   # -80.0
TILT_SW_MAX   = TILT_PHYS_MAX - SW_MARGIN   #  12.7

# --- Calibration file ---
CALIB_FILE = "calibration.txt"
