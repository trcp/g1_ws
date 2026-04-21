#!/usr/bin/env python3

# ROS
from rclpy.node import Node
import rclpy

# TTS interface
from g1_srvs.srv import AudioClient


class TTS:
    def __init__(self, node:Node):
        """TTS クラスの初期化。

        Parameters
        ----------
        node : Node
            サービス呼び出しとログ出力に使用する ROS ノードインスタンス。
        """
        self.__node = node

        # create TTS client
        self.__tts_cli = self.__node.create_client(AudioClient, '/play_audio')
        while not self.__tts_cli.wait_for_service(timeout_sec=5.0):
            self.__node.get_logger().error('May be erasers_g1 is not running ...')
            raise RuntimeError('May be erasers_g1 is not running ...')


    def __send_req(self, req:AudioClient.Request, logger:str, wait:bool):
        future = self.__tts_cli.call_async(req)
        if logger == "info":
            self.__node.get_logger().info("G1 SAY::::::::::::\n>> %s"%req.text)
        elif logger == "warn":
            self.__node.get_logger().warn("G1 SAY::::::::::::\n>> %s"%req.text)
        elif logger == "error":
            self.__node.get_logger().error("G1 SAY::::::::::::\n>> %s"%req.text)
        elif logger == "debug":
            self.__node.get_logger().debug("G1 SAY::::::::::::\n>> %s"%req.text)
        if wait:
            rclpy.spin_until_future_complete(self.__node, future)
            res:AudioClient.Response = future.result()
            return res.success


    def say(self, text:str, logger:str="info", wait:bool=True) -> bool:
        """テキストを音声で読み上げる。

        Parameters
        ----------
        text : str
            読み上げるテキスト。
        wait : bool, optional
            読み上げ完了を待つかどうか、デフォルトは True。

        Returns
        -------
        bool
            読み上げが成功したかどうか。
        """
        req = AudioClient.Request()
        req.text = text
        return self.__send_req(req, logger, wait)
    
    
    def audio(self, audio_path:str, logger:str="info", wait:bool=True) -> bool:
        req = AudioClient.Request()
        req.audio_path = audio_path
        return self.__send_req(req, logger, wait)
    