 #!/usr/bin/env python3
from rclpy.node import Node
import rclpy

# msgs
from geometry_msgs.msg import Twist
from g1_srvs.srv import MoveServo

# general
import time


class G1Control():
    def __init__(self, node:Node):
        self.__node = node

        self.__twist_pub = self.__node.create_publisher(Twist, '/cmd_vel', 10)
        self.__servo_cli = self.__node.create_client(MoveServo, '/move_servo')

        while not self.__servo_cli.wait_for_service(timeout_sec=5.0):
            self.__node.get_logger().error('Head Servo is not running ...')
            raise RuntimeError('Head Servo is not running ...')


    def move_stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = 0.0

        self.__node.get_logger().info('Stop move by velosity.')
        init_time = time.time()
        while rclpy.ok() and time.time() - init_time < 0.5:
            self.__twist_pub.publish(twist)
            rclpy.spin_once(self.__node, timeout_sec=0.1)


    def move_velosity(self, x:float=0.0, y:float=0.0, yaw:float=0.0, pub_once:bool=False, time_sec:float=1.0):
        twist = Twist()
        twist.linear.x = x
        twist.linear.y = y
        twist.angular.z = yaw

        if pub_once:
            self.__twist_pub.publish(twist)
            rclpy.spin_once(self.__node, timeout_sec=0.1)

        else:
            self.__node.get_logger().info('Move by velosity: x:%f y:%f yaw:%f time: %f sec.'%(x, y, yaw, time_sec))
            init_time = time.time()
            while rclpy.ok() and time.time() - init_time < time_sec:
                self.__twist_pub.publish(twist)
                rclpy.spin_once(self.__node, timeout_sec=0.1)
                time.sleep(0.05)

            self.move_stop()
            self.__node.get_logger().info('Move by velosity Finished.')


    def __send_angle_req(self, req:MoveServo.Request):
        future = self.__servo_cli.call_async(req)
        rclpy.spin_until_future_complete(self.__node, future)
        response:MoveServo.Response = future.result()
        return response.success


    def move_head(self, tilt:float=0.0, pan:float=0.0):
        req = MoveServo.Request()
        req.tilt = -tilt
        req.pan = pan

        return self.__send_angle_req(req)
