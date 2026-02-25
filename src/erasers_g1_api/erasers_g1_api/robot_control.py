 #!/usr/bin/env python3
from rclpy.node import Node
import rclpy

# msgs
from geometry_msgs.msg import Twist
from amazing_hand_interfaces.srv import HandCommand
from g1_srvs.srv import MoveServo

# general
import time


class G1Control():
    def __init__(self, node:Node):
        self.__node = node

        self.__servo_cli = self.__node.create_client(MoveServo, '/move_servo')
        self.__hand_cli = self.__node.create_client(HandCommand, '/hand_command')

        while not self.__servo_cli.wait_for_service(timeout_sec=5.0) or not self.__hand_cli.wait_for_service(timeout_sec=5.0):
            self.__node.get_logger().error('Robot Service Servers are not running ...')
            raise RuntimeError('Robot Service Servers are not running ...')


    def __send_angle_req(self, req:MoveServo.Request):
        future = self.__servo_cli.call_async(req)
        rclpy.spin_until_future_complete(self.__node, future)
        response:MoveServo.Response = future.result()
        return response.success
    

    def __send_hand_req(self, req:HandCommand.Request):
        future = self.__hand_cli.call_async(req)
        rclpy.spin_until_future_complete(self.__node, future)
        response:HandCommand.Response = future.result()
        return response.success


    def move_head(self, tilt:float=0.0, pan:float=0.0):
        req = MoveServo.Request()
        req.tilt = -tilt
        req.pan = pan
        return self.__send_angle_req(req)


    def hand_control(self, command:str='walk', hand='both'):
        req = HandCommand.Request()
        req.command = command
        req.hand = hand
        return self.__send_hand_req(req)
