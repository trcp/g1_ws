import numpy as np

import rclpy
from rclpy.node import Node

from amazing_hand_interfaces.srv import HandCommand

from rustypot import Scs0009PyController


class AmazingHandControllerNode(Node):
    def __init__(self):
        super().__init__('amazing_hand_controller')

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 1000000)

        serial_port = self.get_parameter("serial_port").value
        baudrate = self.get_parameter("baudrate").value

        # -----------------------------
        # AmazingHand Controller
        # -----------------------------
        self.c = Scs0009PyController(
            serial_port=serial_port,
            baudrate=baudrate,
            timeout=0.05,
        )

        self.c.write_torque_enable(1, 1)

        # -----------------------------
        # ROS Service
        # -----------------------------
        self.srv = self.create_service(
            HandCommand,
            "hand_command",
            self.service_callback
        )

        # -----------------------------
        # Constants
        # -----------------------------
        self.max_speed = 7
        self.close_speed = 3
        self.close_finger_angles = (60, -60)
        self.close_thumb_angles = (75, -75)

        # calibration values
        self.middle_pos = {
            "right": [25, -20, 25, -15, 20, -25, 15, -15],
            "left": [20, -10, 33, -35, 30, -10, 20, -8],
        }

        # servo id map
        self.servo_map = {
            "right": {
                "index": (1, 2),
                "middle": (3, 4),
                "ring": (5, 6),
                "thumb": (7, 8),
            },
            "left": {
                "index": (11, 12),
                "middle": (13, 14),
                "ring": (15, 16),
                "thumb": (17, 18),
            },
        }

        self.get_logger().info("Hand controller node started")

    # ==================================================
    # Service callback
    # ==================================================

    def service_callback(self, request, response):
        cmd = request.command.lower()
        hand = request.hand.lower()

        if hand not in ["right", "left", "both"]:
            response.success = False
            response.message = f"Unknown hand: {hand}"
            return response

        self.get_logger().info(f"Command: {cmd}, Hand: {hand}")

        if cmd == "open":
            self.open_hand(hand)

        elif cmd == "close":
            self.close_hand(hand)

        elif cmd == "walk":
            self.walk_hand(hand)

        elif cmd == "progressive":
            self.open_hand_progressive(hand)

        else:
            response.success = False
            response.message = f"Unknown command: {cmd}"
            return response

        response.success = True
        response.message = "Executed"
        return response

    # ==================================================
    # Utility
    # ==================================================

    def target_hands(self, hand):
        if hand == "both":
            return ["right", "left"]
        return [hand]

    # ==================================================
    # Low level control
    # ==================================================

    def move_finger(self, finger, angle1, angle2, speed, hand):
        """
        finger: index / middle / ring / thumb
        hand: right / left
        """

        id1, id2 = self.servo_map[hand][finger]

        self.c.write_goal_speed(id1, speed)
        self.c.write_goal_speed(id2, speed)

        base = self.middle_pos[hand]

        finger_index = {
            "index": 0,
            "middle": 2,
            "ring": 4,
            "thumb": 6,
        }[finger]

        pos1 = np.deg2rad(base[finger_index] + angle1)
        pos2 = np.deg2rad(base[finger_index + 1] + angle2)

        self.c.write_goal_position(id1, pos1)
        self.c.write_goal_position(id2, pos2)

    # ==================================================
    # Gestures
    # ==================================================

    def walk_hand(self, hand):
        self.get_logger().info(f"Walk hand: {hand}")

        for h in self.target_hands(hand):
            for finger in ["index", "middle", "ring", "thumb"]:
                self.move_finger(finger, -35, 35, self.max_speed, h)

            self.move_finger("thumb", 100, -100, self.close_speed + 4, h)

    def open_hand(self, hand):
        self.get_logger().info(f"Open hand: {hand}")

        for h in self.target_hands(hand):
            for finger in ["index", "middle", "ring", "thumb"]:
                self.move_finger(finger, -35, 35, self.max_speed, h)

    def close_hand(self, hand):
        self.get_logger().info(f"Close hand: {hand}")

        for h in self.target_hands(hand):
            for finger in ["index", "middle", "ring"]:
                self.move_finger(finger, *self.close_finger_angles, self.close_speed, h)

            self.move_finger("thumb", *self.close_finger_angles, self.close_speed + 4, h)
            self.move_finger("thumb", *self.close_thumb_angles, self.close_speed + 4, h)

    def open_hand_progressive(self, hand):
        self.get_logger().info(f"Open progressive: {hand}")

        for finger in ["index", "middle", "ring", "thumb"]:
            for h in self.target_hands(hand):
                self.move_finger(finger, -35, 35, self.max_speed - 2, h)

def main(args=None):
    rclpy.init(args=args)

    node = AmazingHandControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
