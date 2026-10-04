#!/usr/bin/env python3
"""Head servo controller module.

Provides HeadServoNode with automatic collision-based calibration
and position control, compatible with both head_servo_node and head_servo_controller.
"""

from .head_servo_node import (
    HeadServoNode,
    main,
)

__all__ = ['HeadServoNode', 'main']

if __name__ == '__main__':
    main()
