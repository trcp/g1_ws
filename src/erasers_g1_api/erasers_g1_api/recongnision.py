#!/usr/bin/env python3
# Copyright (c) 2026 Nakatogawa Laboratory. All rights reserved.

"""Unitree G1 のWhisper音声認識Actionクライアントです．"""

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from erasers_g1_api.vui_audio import wait_future

from erasers_g1_interfaces.action import WhisperRecongnition

from typing import Optional
import math


class WhisperSpeechToText():
    """Whisper音声認識Actionを利用する同期クライアントです．

    Parameters
    ----------
    node : Node
        Action通信に使用するROS 2ノード．

    Methods
    -------
    recongnition
        音声認識Goalを送信し，認識文を返します．
    cancel_recongnition_action
        実行中の音声認識Goalをキャンセルします．
    """

    __action_name = '/whisper_recongnition_action'
    __server_timeout_sec = 10.0
    __minimum_recording_time_sec = 1.0

    def __init__(self, node: Node):
        """Whisper音声認識Actionクライアントを初期化します．

        Parameters
        ----------
        node : Node
            Action通信に使用するROS 2ノード．

        Returns
        -------
        None

        Raises
        ------
        RuntimeError
            Actionサーバーへ接続できない場合．
        """
        self.__node = node
        self.__logger = node.get_logger()
        self.__action_client = ActionClient(
            self.__node,
            WhisperRecongnition,
            self.__action_name,
            callback_group=ReentrantCallbackGroup(),
        )
        self.__goal_handle = None
        self.__partial_result = ''

        while not self.__action_client.wait_for_server(
            timeout_sec=self.__server_timeout_sec
        ):
            self.__logger.error(
                f'Whisper action server is unavailable: {self.__action_name}'
            )
            raise RuntimeError('Whisper action server is unavailable.')

    def __cb_recongnition_feedback(self, feedback_message) -> None:
        """途中認識結果を保持します．

        Parameters
        ----------
        feedback_message : object
            Actionクライアントが受信したFeedbackメッセージ．

        Returns
        -------
        None

        Raises
        ------
        AttributeError
            Feedbackメッセージに認識結果フィールドがない場合．
        """
        self.__partial_result = feedback_message.feedback.partial_result

    @staticmethod
    def __validate_request(
        language: str,
        max_recongnition_time: float,
        minimum_scilent_time: float,
    ) -> None:
        """音声認識要求の引数を検証します．

        Parameters
        ----------
        language : str
            認識言語コード．
        max_recongnition_time : float
            最大録音時間［s］．
        minimum_scilent_time : float
            発話後の無音判定時間［s］．

        Returns
        -------
        None

        Raises
        ------
        ValueError
            引数がActionの制約を満たさない場合．
        """
        if language not in ('', 'en', 'ja'):
            raise ValueError("language must be '', 'en', or 'ja'.")
        if (
            not math.isfinite(max_recongnition_time)
            or max_recongnition_time < WhisperSpeechToText.__minimum_recording_time_sec
        ):
            raise ValueError('max_recongnition_time must be at least 1.0.')
        if not math.isfinite(minimum_scilent_time) or minimum_scilent_time <= 0.0:
            raise ValueError('minimum_scilent_time must be greater than zero.')

    def __send_recongnition_goal(self, goal: WhisperRecongnition.Goal) -> Optional[str]:
        """構築済みGoalを送信して結果を待機します．

        Parameters
        ----------
        goal : WhisperRecongnition.Goal
            送信する音声認識Goal．

        Returns
        -------
        Optional[str]
            成功時の認識文，失敗時はNone．

        Raises
        ------
        KeyboardInterrupt
            ユーザーが認識処理を中断した場合．
        """
        self.__partial_result = ''
        try:
            send_goal_future = self.__action_client.send_goal_async(
                goal,
                feedback_callback=self.__cb_recongnition_feedback,
            )
            if not wait_future(self.__node, send_goal_future, self.__server_timeout_sec):
                # 遅れて受理された録音要求を残さない。
                send_goal_future.add_done_callback(self.__cancel_late_goal)
                self.__logger.error('音声認識要求の受付がタイムアウトしました')
                return None
            self.__goal_handle = send_goal_future.result()
            if self.__goal_handle is None or not self.__goal_handle.accepted:
                self.__logger.warn('Whisper recognition goal was rejected.')
                return None

            result_future = self.__goal_handle.get_result_async()
            if not wait_future(self.__node, result_future, goal.maximum_recording_time + 120.0):
                self.cancel_recongnition_action()
                self.__logger.error('音声認識結果がタイムアウトしました')
                return None
            result_response = result_future.result()
            if result_response is None or not result_response.result.success:
                message = (
                    'no result'
                    if result_response is None
                    else result_response.result.message
                )
                self.__logger.warn(f'Whisper recognition failed: {message}')
                return None
            return result_response.result.result
        except KeyboardInterrupt:
            self.cancel_recongnition_action()
            raise
        except Exception as error:
            self.__logger.error(f'Whisper recognition client failed: {error}')
            return None
        finally:
            self.__goal_handle = None

    def recongnition(
        self,
        language: str = 'ja',
        max_recongnition_time: float = 30.0,
        minimum_scilent_time: float = 5.0,
    ) -> Optional[str]:
        """音声認識Goalを構築して送信します．

        Parameters
        ----------
        language : str, optional
            認識言語コード．空文字列は自動検出を指定します．
        max_recongnition_time : float, optional
            最大録音時間［s］．
        minimum_scilent_time : float, optional
            発話後の無音判定時間［s］．

        Returns
        -------
        Optional[str]
            成功時の認識文，失敗時はNone．

        Raises
        ------
        ValueError
            引数が不正な場合．
        """
        self.__validate_request(
            language,
            max_recongnition_time,
            minimum_scilent_time,
        )
        goal = WhisperRecongnition.Goal()
        goal.language = language
        goal.minimum_recording_time = self.__minimum_recording_time_sec
        goal.max_silence_time = minimum_scilent_time
        goal.maximum_recording_time = max_recongnition_time
        return self.__send_recongnition_goal(goal)

    @staticmethod
    def __cancel_late_goal(future) -> None:
        """受付待ちの期限後に受理された録音を停止する。"""
        if not future.cancelled() and future.exception() is None:
            handle = future.result()
            if handle is not None and handle.accepted:
                handle.cancel_goal_async()

    @property
    def partial_result(self) -> str:
        """Action フィードバックで受信した途中認識結果。"""
        return self.__partial_result

    def cancel_recongnition_action(self) -> bool:
        """実行中の音声認識Goalをキャンセルします．

        Returns
        -------
        bool
            キャンセル要求を送信できた場合はTrue，実行中Goalがない場合はFalse．

        Raises
        ------
        None
            通信失敗時はFalseを返します．
        """
        if self.__goal_handle is None:
            return False
        try:
            cancel_future = self.__goal_handle.cancel_goal_async()
            if not wait_future(self.__node, cancel_future, self.__server_timeout_sec):
                return False
            response = cancel_future.result()
            return response is not None and bool(response.goals_canceling)
        except Exception as error:
            self.__logger.error(f'Whisper recognition cancellation failed: {error}')
            return False
