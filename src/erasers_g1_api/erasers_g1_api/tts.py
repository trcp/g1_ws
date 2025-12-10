#!/usr/bin/env python3

# ROS
from rclpy.node import Node
import rclpy

# TTS interface
from g1_srvs.srv import AudioClient


class TTS:
    def __init__(self, node:Node):
        self.__node = node

        # create TTS client
        self.__tts_cli = self.__node.create_client(AudioClient, '/play_audio')
        while not self.__tts_cli.wait_for_service(timeout_sec=5.0):
            self.__node.get_logger().error('May be erasers_g1 is not running ...')
            raise RuntimeError('May be erasers_g1 is not running ...')


    def __send_req(self, req:AudioClient.Request, wait:bool):
        future = self.__tts_cli.call_async(req)
        if wait:
            rclpy.spin_until_future_complete(self.__node, future)
            res:AudioClient.Response = future.result()
            return res.success


    def say(self, text:str, wait:bool=True) -> bool:
        req = AudioClient.Request()
        req.text = text
        return self.__send_req(req, wait)
