#!/usr/bin/env python3
from rclpy_util.util import TemporarySubscriber
from rclpy.node import Node
from rclpy import qos
import rclpy

from sensor_msgs.msg import JointState

import smach
from typing import Optional

# erasers_api
from erasers_g1_api.robot_control import ArmControl
from erasers_g1_api.tts import TTS

# general
import numpy as np
import traceback
import threading
import time
import os


class WaitPushHand(smach.State):
    def __init__(
        self,
        node: Node,
        tts_say: TTS.say,
        arm_control: Optional[ArmControl] = None,
        start_msg: str = "I will up right arm. Please push my arm after up the arm.",
        success_msg: str = "Thank you.",
        timeout_msg: str = "Time is out.",
        failure_msg: str = "Failure. error is ocurred in wait push hand.",
        hand: str = "right",
        timeout_sec: int = 10,
        threthold: float = 1.5,
    ):
        """手を押されるまで待機する。

        Parameters
        ----------
        node : Node
            サブスクライバ、publisher、ログ出力に使用する ROS ノードインスタンス。
        tts_say : TTS.say
            音声応答に使うテキスト読み上げ関数。
        arm_control : Optional[ArmControl], optional
            右手を待機姿勢へ移動する ArmControl インスタンス。None の場合は
            /upper_joints_control に JointState を publish して待機姿勢を維持する。
        start_msg : str, optional
            手押し待機開始時に読み上げるメッセージ。
        success_msg : str, optional
            手押し検出成功時に読み上げるメッセージ。
        timeout_msg : str, optional
            タイムアウト時に読み上げるメッセージ。
        failure_msg : str, optional
            待機処理中にエラーが発生した場合に読み上げるメッセージ。
        hand : str, optional
            fallback publisher で動かす手の左右。'right' または 'left' を想定する。
        timeout_sec : int, optional
            手押し検出を待機する最大時間（秒）。
        threthold : float, optional
            手押しと判定する right_elbow_joint effort 差分のしきい値。

        userdata
        --------
        Input Keys:
            なし。
        Output Keys:
            なし。

        Outcomes
        ----------
        success:
            手押しが検出された場合。
        timeout:
            timeout_sec 内に手押しが検出されなかった場合。
        failure:
            待機処理中にエラーが発生した場合。
        """

        # init smach
        smach.State.__init__(self, outcomes=["success", "timeout", "failure"])

        # init values
        self.__node: Node = node
        self.__say: TTS.say = tts_say
        self.__arm_control: Optional[ArmControl] = arm_control
        self.__start_msg: str = start_msg
        self.__success_msg: str = success_msg
        self.__timeout_msg: str = timeout_msg
        self.__failure_msg: str = failure_msg
        self.__timeout_sec = timeout_sec
        self.__threthold: float = threthold
        self.__hand: str = hand
        self.__push_hend: bool = False
        self.__init_effort: float = None
        self.__fallback_hand_pose_pub = self.__node.create_publisher(
            JointState, "/upper_joints_control", 10
        )
        self.__fallback_publish_stop = threading.Event()
        self.__fallback_publish_thread = None

    def __publish_fallback_hand_pose(self):
        msg = JointState()
        msg.header.stamp = self.__node.get_clock().now().to_msg()
        msg.name = [
            f"{self.__hand}_shoulder_pitch_joint",
            f"{self.__hand}_wrist_roll_joint",
        ]
        msg.position = [-0.735, 1.57]
        msg.velocity = [0.0, 0.0]
        self.__fallback_hand_pose_pub.publish(msg)

    def __fallback_publish_loop(self):
        while not self.__fallback_publish_stop.is_set():
            self.__publish_fallback_hand_pose()
            time.sleep(0.1)

    def __start_fallback_publish(self):
        if (
            self.__fallback_publish_thread is not None
            and self.__fallback_publish_thread.is_alive()
        ):
            return

        self.__fallback_publish_stop.clear()
        self.__fallback_publish_thread = threading.Thread(
            target=self.__fallback_publish_loop,
            daemon=True,
        )
        self.__fallback_publish_thread.start()

    def __stop_fallback_publish(self):
        self.__fallback_publish_stop.set()
        if self.__fallback_publish_thread is not None:
            self.__fallback_publish_thread.join(timeout=1.0)
            self.__fallback_publish_thread = None

    def __move_hand_to_wait_pose(self):
        if self.__arm_control is not None:
            self.__arm_control.enable_upper_body_control(True)
            return self.__arm_control.joint_control(
                right_shoulder_pitch_joint=-0.735,
                right_wrist_roll_joint=1.57,
                planning_group="upper_body",
            )

        self.__node.get_logger().info(
            "ArmControl is not set. Publishing wait pose to /upper_joints_control."
        )
        self.__start_fallback_publish()
        return True

    def __release_hand_pose(self):
        if self.__arm_control is None:
            self.__stop_fallback_publish()
            return

        self.__arm_control.move_groupstate(group_state="walk")
        self.__arm_control.enable_upper_body_control(False)

    def __joint_state_cb(self, msg: JointState):
        try:
            effort = msg.effort[msg.name.index("right_elbow_joint")]
            if self.__init_effort is None and not np.isnan(effort):
                self.__init_effort = effort
            else:
                if not np.isnan(self.__init_effort) and not np.isnan(effort):
                    #print(f"effort: {self.__init_effort - effort}")
                    if self.__init_effort - effort >= self.__threthold:
                        self.__push_hend = True
        except Exception as e:
            self.__node.get_logger().warn(f"JointState callback error: {e}")

    def execute(self, userdata):
        """SMACH state を実行し、手押し入力を待機する。

        Parameters
        ----------
        userdata : smach.UserData
            この state では参照しない。

        Returns
        -------
        str
            SMACH outcome。'success'、'timeout'、'failure' のいずれか。
        """
        try:
            self.__push_hend = False
            self.__init_effort = None

            # enable move hand
            self.__say(self.__start_msg)
            self.__move_hand_to_wait_pose()

            joint_state_qos_profile = qos.QoSProfile(
                reliability=qos.ReliabilityPolicy.BEST_EFFORT,
                history=qos.HistoryPolicy.KEEP_LAST,
                depth=10,
            )
            # detect open door via LaserScan
            with TemporarySubscriber(
                self.__node,
                JointState,
                "/joint_states",
                joint_state_qos_profile,
                self.__joint_state_cb,
            ):
                # declare msg
                self.__node.get_logger().info("""
                    WAIT PUSH MY HAND ....
                """)
                init_time = time.time()
                while (
                    time.time() - init_time < self.__timeout_sec
                    and not self.__push_hend
                ):
                    rclpy.spin_once(self.__node, timeout_sec=0.1)

            self.__release_hand_pose()

            # Success
            if self.__push_hend:
                self.__node.get_logger().info("""
                    DOOR OPEN !
                """)
                self.__say(self.__success_msg)

                return "success"

            # Timeout
            else:
                self.__node.get_logger().warn("""
                    TIMEOUT. DOOR IS NOT OPEN ...
                """)
                self.__say(self.__timeout_msg)

                return "timeout"

        except Exception as e:
            self.__release_hand_pose()
            self.__node.get_logger().warn(f"WaitPushHand error: {e}")
            self.__node.get_logger().warn(traceback.format_exc())
            return "failure"
