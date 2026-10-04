import numpy as np

try:
    np.float = float
except AttributeError:
    pass

#!/usr/bin/env python3
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
import random
import rclpy

# msgs
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from amazing_hand_interfaces.srv import HandCommand
from erasers_g1_interfaces.srv import MoveServo, PosePolicy, RobotPose, ArmAction
from erasers_g1_api.vui_audio import wait_future
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    PositionConstraint,
    OrientationConstraint,
    MoveItErrorCodes,
    PlanningScene,
    CollisionObject,
    AttachedCollisionObject,
    PlanningSceneComponents,
)
from moveit_msgs.srv import GetPositionIK, GetPlanningScene
from std_msgs.msg import Int16MultiArray

# tf
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformListener, Buffer
from tf_transformations import euler_from_quaternion, quaternion_from_euler

# general
import time
import math
import copy
import os
import xml.etree.ElementTree as ET
from rclpy.action import ActionClient
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)

# ArmControl specific imports
from std_srvs.srv import SetBool, Trigger
import tf_transformations
import threading

# G1Mic specific imports
import socket
import struct
import numpy as np
import wave

try:
    import netifaces
except ImportError:
    netifaces = None

from scipy.spatial.transform import Rotation as R


# マニピュレーション API の結果型と安全な ROS 待機処理。
from dataclasses import dataclass, replace
from enum import Enum
from functools import wraps
from pathlib import Path
from typing import Any, Optional, Union

from geometry_msgs.msg import Quaternion
from moveit_msgs.msg import AllowedCollisionEntry
from moveit_msgs.srv import ApplyPlanningScene, GetPositionFK
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy._rclpy_pybind11 import InvalidHandle, RCLError
from tf2_ros import TransformException

class G1Control:
    def __init__(self, node: Node):
        """
        G1Control クラスのコンストラクタ

        Parameters
        ----------
        node : Node
            ROS2 ノードオブジェクト。サービスクライアントの作成と呼び出しに使用する。
        """
        self.node = node

        self.__servo_cli = self.node.create_client(MoveServo, "/move_servo")
        self.__pose_cli = self.node.create_client(PosePolicy, "/pose_policy")
        self.__robot_pose_cli = self.node.create_client(RobotPose, "/robot_pose")
        self.__arm_action_cli = self.node.create_client(ArmAction, "/arm_action")

        while not self.__servo_cli.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error("Servo Service Servers are not running ...")
            break
        while not self.__pose_cli.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error("Pose Service Servers are not running ...")
            break

    def __send_angle_req(self, req: MoveServo.Request):
        """
        サーボ角度移動リクエストを送信する内部メソッド

        Parameters
        ----------
        req : MoveServo.Request
            サーボ移動の要求メッセージ。

        Returns
        -------
        bool
            サービス呼び出しが成功した場合は True、失敗した場合は False。
        """
        future = self.__servo_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        response: MoveServo.Response = future.result()
        return response.success

    def __send_pose_req(self, req: PosePolicy.Request):
        """
        ポーズポリシー要求を送信する内部メソッド

        Parameters
        ----------
        req : PosePolicy.Request
            ポーズポリシーの要求メッセージ。

        Returns
        -------
        bool
            サービス呼び出しが成功した場合は True、失敗した場合は False。
        """
        future = self.__pose_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        response: PosePolicy.Response = future.result()
        return response.success

    def robot_pose(self, mode: int, timeout_sec: float = 25.0) -> bool:
        """FSM の遷移結果を確認する姿勢サービスを呼ぶ。"""
        request = RobotPose.Request()
        request.mode = mode
        return self.__bounded_control_call(self.__robot_pose_cli, request, timeout_sec)

    def arm_action(self, action_id: int, timeout_sec: float = 40.0) -> bool:
        """上半身の関節制御を無効化した後に、腕動作とその完了を要求する。"""
        request = ArmAction.Request()
        request.mode = action_id
        return self.__bounded_control_call(self.__arm_action_cli, request, timeout_sec)

    def __bounded_control_call(self, client, request, timeout_sec):
        if not math.isfinite(timeout_sec) or timeout_sec <= 0:
            raise ValueError("timeout_sec は正の有限値で指定してください")
        if not client.wait_for_service(timeout_sec=2.0):
            return False
        future = client.call_async(request)
        if not wait_future(self.node, future, timeout_sec):
            client.remove_pending_request(future)
            return False
        response = future.result()
        return response is not None and response.success

    def move_head(self, tilt: float = 0.0, pan: float = 0.0):
        """
        頭部を傾けて旋回させる。

        Parameters
        ----------
        tilt : float, optional
            頭部の上下角度(rad)。
        pan : float, optional
            頭部の左右角度(rad)。

        Returns
        -------
        bool
            サーボコマンド送信に成功した場合は True、失敗した場合は False。
        """
        req = MoveServo.Request()
        req.tilt = -tilt
        req.pan = pan
        return self.__send_angle_req(req)

    def pose_policy(self, pose: str):
        """
        ポーズポリシーを設定する。

        Parameters
        ----------
        pose : str
            適用するポーズポリシーの識別子。PosePolicy.srv で定義されている
            対応姿勢は 'damp'、'start'、'squat'、'sit'、'stand_up'、
            'zero_torque'、'stop_move'、'high_stand'、'low_stand'、
            'balance_stand'、'shake_hand'、'wave_hand'、
            'wave_hand_with_turn'、'running'。

        Returns
        -------
        bool
            サービス呼び出しが成功した場合は True、失敗した場合は False。
        """
        req = PosePolicy.Request()
        req.pose = pose
        return self.__send_pose_req(req)


class G1Navigation:
    
    GET_BY_TOPIC = True

    def __init__(
        self,
        node: Node,
        wait_time: int = 10,
        tf_buffer: Buffer = None,
        debug_goal_topic: str = "/api_goal",
    ):
        """
        G1Navigation クラスのコンストラクタ

        Parameters
        ----------
        node : Node
            ROS2 ノードオブジェクト
        wait_time : int, optional
            アクションサーバー接続待機時間(秒)。デフォルトは 10。
        tf_buffer : Buffer, optional
            TF2 バッファオブジェクト。None の場合は新規作成。デフォルトは None。
        debug_goal_topic : str, optional
            Nav2 に送る最終ゴール PoseStamped を publish するデバッグ用トピック。
            デフォルトは '/api_goal'。
        """
        self.node = node
        self.TIMEOUT_SEC = 60.0
        self.FACE_GOAL_TIMEOUT_SEC = 30.0
        self.__current_goal_handle = None
        self.__latest_odom_pose = None
        self.__odom_lock = threading.Lock()
        self.__latest_localization_pose = None
        self.__localization_lock = threading.Lock()

        # TF2 Setup
        self.tf_buffer = tf_buffer or Buffer()
        self.__tf_listener = TransformListener(self.tf_buffer, self.node)

        # Action Client Setup
        self.__action_client = ActionClient(
            self.node, NavigateToPose, "/navigate_to_pose"
        )
        if not self.__action_client.wait_for_server(timeout_sec=wait_time):
            self.node.get_logger().fatal("Nav2 action server not available...")
            # raise RuntimeError("Nav2 action server not available")

        # Initial pose publisher
        self.__initial_pose_pub = self.node.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self.__debug_goal_pub = self.node.create_publisher(
            PoseStamped, debug_goal_topic, 10
        )
        self.__cmd_vel_pub = self.node.create_publisher(Twist, "/cmd_vel", 10)
        self.__odom_sub = self.node.create_subscription(
            Odometry, "/odom", self.__odom_callback, 10
        )
        localization_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.__localization_pose_sub = self.node.create_subscription(
            PoseWithCovarianceStamped,
            "/localization/pose_with_covariance",
            self.__localization_pose_callback,
            localization_qos,
        )

    def get_current_pose(self, simple: bool = False):
        """
        現在のロボットの位置姿勢を取得する．

        Parameters
        ----------
        simple : bool, optional
            True の場合、[x, y, yaw] の1次元リストとして現在位置を出力する。
            False の場合、PoseStamped 型で現在位置を出力する。デフォルトは False。
        use_topic : bool, optional
            True の場合、/localization/pose_with_covariance の最新値から現在位置を取得する。
            False の場合、TF の map -> base_link 変換から現在位置を取得する。
            デフォルトは True。

        Returns
        -------
        PoseStamped or list of float
            simple=False の場合はマップ座標系基準の PoseStamped。
            simple=True の場合は [x, y, yaw] を格納したリスト。
        """
        if self.GET_BY_TOPIC:
            last_warn_time = 0.0
            while rclpy.ok():
                rclpy.spin_once(self.node, timeout_sec=0.05)
                pose = self.__get_current_localization_pose()
                if pose is not None:
                    return self.__format_current_pose(pose, simple)

                now = time.time()
                if now - last_warn_time >= 2.0:
                    self.node.get_logger().warn(
                        "Waiting for /localization/pose_with_covariance ..."
                    )
                    last_warn_time = now
            return None

        last_warn_time = 0.0
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.01)
            try:
                if not self.tf_buffer.can_transform(
                    "map", "base_link", rclpy.time.Time()
                ):
                    now = time.time()
                    if now - last_warn_time >= 2.0:
                        self.node.get_logger().warn(
                            "Waiting for TF transform map -> base_link ..."
                        )
                        last_warn_time = now
                    continue

                transform = self.tf_buffer.lookup_transform(
                    "map", "base_link", rclpy.time.Time()
                )
                pose = PoseStamped()
                pose.header = transform.header
                pose.pose.position.x = transform.transform.translation.x
                pose.pose.position.y = transform.transform.translation.y
                pose.pose.position.z = transform.transform.translation.z
                pose.pose.orientation = transform.transform.rotation
                return self.__format_current_pose(pose, simple)
            except Exception as e:
                self.node.get_logger().warn(f"TF Lookup failed: {str(e)}")
                continue

        return None

    def __format_current_pose(self, pose: PoseStamped, simple: bool = False):
        if simple:
            q = pose.pose.orientation
            (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
            return [
                pose.pose.position.x,
                pose.pose.position.y,
                yaw,
            ]

        return copy.deepcopy(pose)

    def __get_pose_yaw(self, pose: PoseStamped) -> float:
        q = pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return yaw

    def __localization_pose_callback(self, msg: PoseWithCovarianceStamped):
        pose = PoseStamped()
        pose.header = copy.deepcopy(msg.header)
        pose.pose = copy.deepcopy(msg.pose.pose)
        with self.__localization_lock:
            self.__latest_localization_pose = pose

    def __get_current_localization_pose(self):
        with self.__localization_lock:
            if self.__latest_localization_pose is None:
                return None
            return copy.deepcopy(self.__latest_localization_pose)

    def __odom_callback(self, msg: Odometry):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )
        with self.__odom_lock:
            self.__latest_odom_pose = (
                float(position.x),
                float(position.y),
                float(yaw),
            )

    def __get_current_odom_pose(self):
        with self.__odom_lock:
            if self.__latest_odom_pose is None:
                return None
            return tuple(self.__latest_odom_pose)

    def __wait_for_odom_pose(self, timeout: float = None):
        timeout_sec = self.TIMEOUT_SEC if timeout is None else timeout
        start_time = time.time()
        while rclpy.ok():
            current_pose = self.__get_current_odom_pose()
            if current_pose is not None:
                return current_pose

            if timeout_sec is not None and timeout_sec > 0:
                if time.time() - start_time > timeout_sec:
                    return None

            rclpy.spin_once(self.node, timeout_sec=0.05)

        return None

    def __normalize_angle(self, angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def __clamp(self, value: float, min_value: float, max_value: float) -> float:
        return max(min_value, min(max_value, value))

    def __publish_stop_cmd(self, repeat: int = 5):
        stop_cmd = Twist()
        for _ in range(max(1, int(repeat))):
            self.__cmd_vel_pub.publish(stop_cmd)
            rclpy.spin_once(self.node, timeout_sec=0.0)
            time.sleep(0.02)

    def __move_to_pose_odom_only(
        self,
        goal_pose: PoseStamped,
        tolerance: float = 0.0,
        wait: bool = True,
        timeout: float = None,
    ) -> bool:
        if not wait:
            self.node.get_logger().warn(
                "use_odom_only=True does not support wait=False."
            )
            return False

        if goal_pose.header.frame_id != "odom":
            self.node.get_logger().warn(
                "use_odom_only=True treats goal pose as odom frame, "
                f"but received frame '{goal_pose.header.frame_id}'."
            )

        goal_x = float(goal_pose.pose.position.x)
        goal_y = float(goal_pose.pose.position.y)
        goal_yaw = self.__get_pose_yaw(goal_pose)

        debug_goal = copy.deepcopy(goal_pose)
        debug_goal.header.frame_id = "odom"
        debug_goal.header.stamp = self.node.get_clock().now().to_msg()
        self.__debug_goal_pub.publish(debug_goal)

        return self.__move_to_odom_goal(
            goal_x=goal_x,
            goal_y=goal_y,
            goal_yaw=goal_yaw,
            tolerance=tolerance,
            timeout=timeout,
        )

    def __move_to_odom_goal(
        self,
        goal_x: float,
        goal_y: float,
        goal_yaw: float,
        tolerance: float = 0.0,
        timeout: float = None,
    ) -> bool:
        timeout_sec = self.TIMEOUT_SEC if timeout is None else timeout
        xy_tolerance = max(float(tolerance or 0.0), 0.05)
        yaw_tolerance = 0.08
        control_period = 1.0 / 20.0
        max_linear = 0.25
        max_angular = 0.6
        k_linear = 0.8
        k_angular = 1.5
        heading_gate = 0.25

        start_time = time.time()
        if self.__wait_for_odom_pose(timeout=timeout_sec) is None:
            self.node.get_logger().error("No /odom received for odom-only navigation.")
            self.__publish_stop_cmd()
            return False

        self.node.get_logger().info(
            "Starting odom-only navigation to "
            f"({goal_x:.3f}, {goal_y:.3f}, {goal_yaw:.3f})"
        )

        try:
            while rclpy.ok():
                loop_start = time.time()
                if timeout_sec is not None and timeout_sec > 0:
                    if loop_start - start_time > timeout_sec:
                        self.node.get_logger().error("TIMEOUT ODOM-ONLY NAVIGATION!")
                        return False

                current_pose = self.__get_current_odom_pose()
                if current_pose is None:
                    rclpy.spin_once(self.node, timeout_sec=0.01)
                    continue

                current_x, current_y, current_yaw = current_pose
                dx = goal_x - current_x
                dy = goal_y - current_y
                distance = math.hypot(dx, dy)
                cmd = Twist()

                if distance > xy_tolerance:
                    target_heading = math.atan2(dy, dx)
                    heading_error = self.__normalize_angle(target_heading - current_yaw)
                    cmd.angular.z = self.__clamp(
                        k_angular * heading_error, -max_angular, max_angular
                    )
                    if abs(heading_error) <= heading_gate:
                        cmd.linear.x = self.__clamp(
                            k_linear * distance, 0.2, max_linear
                        )
                else:
                    yaw_error = self.__normalize_angle(goal_yaw - current_yaw)
                    if abs(yaw_error) <= yaw_tolerance:
                        self.node.get_logger().info(
                            "Odom-only navigation reached goal: "
                            f"position_error={distance:.3f} m, "
                            f"yaw_error={yaw_error:.3f} rad"
                        )
                        return True

                    cmd.angular.z = self.__clamp(
                        k_angular * yaw_error, -max_angular, max_angular
                    )

                self.__cmd_vel_pub.publish(cmd)
                rclpy.spin_once(self.node, timeout_sec=0.0)

                sleep_time = control_period - (time.time() - loop_start)
                if sleep_time > 0.0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            self.node.get_logger().warn(
                "KeyboardInterrupt: Stopping odom-only navigation..."
            )
            return False
        except Exception as e:
            self.node.get_logger().error(f"Odom-only navigation error: {str(e)}")
            return False
        finally:
            self.__publish_stop_cmd()

        return False

    def __move_rel_by_odom_displacement(
        self,
        x: float = 0.0,
        y: float = 0.0,
        yaw: float = 0.0,
        tolerance: float = 0.0,
        timeout: float = None,
    ) -> bool:
        timeout_sec = self.TIMEOUT_SEC if timeout is None else timeout
        xy_tolerance = max(float(tolerance or 0.0), 0.05)
        yaw_tolerance = 0.08
        control_period = 1.0 / 20.0
        max_linear = 0.25
        max_angular = 0.6
        k_linear = 0.8
        k_angular = 1.5
        target_x = float(x)
        target_y = float(y)
        target_yaw_delta = float(yaw)

        start_pose = self.__wait_for_odom_pose(timeout=timeout_sec)
        if start_pose is None:
            self.node.get_logger().error(
                "Could not get /odom pose for relative odom-only movement"
            )
            self.__publish_stop_cmd()
            return False

        start_x, start_y, start_yaw = start_pose
        cos_start = math.cos(start_yaw)
        sin_start = math.sin(start_yaw)
        start_time = time.time()

        self.node.get_logger().info(
            "Starting odom-only relative movement by displacement "
            f"(x={target_x:.3f}, y={target_y:.3f}, yaw={target_yaw_delta:.3f})"
        )

        try:
            while rclpy.ok():
                loop_start = time.time()
                if timeout_sec is not None and timeout_sec > 0:
                    if loop_start - start_time > timeout_sec:
                        self.node.get_logger().error(
                            "TIMEOUT ODOM-ONLY RELATIVE MOVEMENT!"
                        )
                        return False

                current_pose = self.__get_current_odom_pose()
                if current_pose is None:
                    rclpy.spin_once(self.node, timeout_sec=0.01)
                    continue

                current_x, current_y, current_yaw = current_pose
                odom_dx = current_x - start_x
                odom_dy = current_y - start_y

                moved_x = cos_start * odom_dx + sin_start * odom_dy
                moved_y = -sin_start * odom_dx + cos_start * odom_dy
                remaining_x = target_x - moved_x
                remaining_y = target_y - moved_y
                distance = math.hypot(remaining_x, remaining_y)
                yaw_delta = self.__normalize_angle(current_yaw - start_yaw)
                yaw_error = self.__normalize_angle(target_yaw_delta - yaw_delta)

                cmd = Twist()
                if distance > xy_tolerance:
                    linear_speed = min(max_linear, k_linear * distance)
                    cmd.linear.x = linear_speed * remaining_x / distance
                    cmd.linear.x = self.__clamp(cmd.linear.x, 0.2, max_linear)
                    cmd.linear.y = linear_speed * remaining_y / distance
                    # cmd.linear.y = self.__clamp(cmd.linear.y, 0.2, max_linear)

                elif (
                    abs(target_yaw_delta) > yaw_tolerance
                    and abs(yaw_error) > yaw_tolerance
                ):
                    cmd.angular.z = self.__clamp(
                        k_angular * yaw_error, -max_angular, max_angular
                    )
                else:
                    self.node.get_logger().info(
                        "Odom-only relative movement reached target displacement: "
                        f"moved=({moved_x:.3f}, {moved_y:.3f}), "
                        f"position_error={distance:.3f} m, yaw_delta={yaw_delta:.3f} rad"
                    )
                    return True

                self.__cmd_vel_pub.publish(cmd)
                rclpy.spin_once(self.node, timeout_sec=0.0)

                sleep_time = control_period - (time.time() - loop_start)
                if sleep_time > 0.0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            self.node.get_logger().warn(
                "KeyboardInterrupt: Stopping odom-only relative movement..."
            )
            return False
        except Exception as e:
            self.node.get_logger().error(f"Odom-only relative movement error: {str(e)}")
            return False
        finally:
            self.__publish_stop_cmd()

        return False

    def __face_goal_pose(self, goal_pose: PoseStamped) -> bool:
        current_pose = self.get_current_pose(simple=True)
        if current_pose is None:
            self.node.get_logger().warn("Could not get current pose to face goal")
            return False

        goal_x = goal_pose.pose.position.x
        goal_y = goal_pose.pose.position.y
        dx = goal_x - current_pose[0]
        dy = goal_y - current_pose[1]
        distance = math.hypot(dx, dy)
        target_yaw = (
            math.atan2(dy, dx) if distance > 1e-3 else self.__get_pose_yaw(goal_pose)
        )

        face_pose = PoseStamped()
        face_pose.header.frame_id = "map"
        face_pose.header.stamp = self.node.get_clock().now().to_msg()
        face_pose.pose.position.x = current_pose[0]
        face_pose.pose.position.y = current_pose[1]
        face_pose.pose.position.z = 0.0

        q = quaternion_from_euler(0, 0, target_yaw)
        face_pose.pose.orientation.x = q[0]
        face_pose.pose.orientation.y = q[1]
        face_pose.pose.orientation.z = q[2]
        face_pose.pose.orientation.w = q[3]

        self.node.get_logger().info(
            f"Facing original goal pose before stopping (yaw={target_yaw:.3f})."
        )
        return self.move_to_pose(
            face_pose,
            tolerance=0.0,
            reference_frame="map",
            wait=True,
            timeout=self.FACE_GOAL_TIMEOUT_SEC,
        )

    def move_to_pose(
        self,
        pose,
        tolerance: float = 0.0,
        reference_frame: str = "map",
        wait: bool = True,
        timeout: float = None,
        use_odom_only: bool = False,
        retry_on_feedback_timeout: bool = True,
        feedback_timeout_sec: float = 5.0,
    ) -> bool:
        """
        与えられた目標姿勢に基づいてロボットを自律移動させる．
        すべてのナビゲーションの中核となるメソッドであり、KeyboardInterrupt 発生時には即座にアクションをキャンセルする。

        Parameters
        ----------
        pose : PoseStamped or Pose
            目標とする姿勢情報。Pose メッセージの場合、reference_frame の座標系基準として扱われる。
        tolerance : float, optional
            目標から指定された距離(m)以内に到達した場合、その時点でナビゲーションを成功として終了する。デフォルトは 0.5。
        reference_frame : str, optional
            pose が Pose 型の場合の基準フレーム。デフォルトは 'map'。
        wait : bool, optional
            移動完了まで処理をブロックするかどうか。デフォルトは True。
        timeout : float, optional
            ナビゲーションのタイムアウト時間(秒)。指定時間を超えた場合はキャンセルして False を返す。
            デフォルトは None (self.TIMEOUT_SEC を使用)。0 以下の場合はタイムアウトなし。
        use_odom_only : bool, optional
            True の場合、erasers_g1_machida_navigation を使わず /odom と /cmd_vel による簡易移動を行う。
        retry_on_feedback_timeout : bool, optional
            True の場合、Action goal が accept された後に feedback_timeout_sec 秒以内に feedback が
            返らなければ、現在の goal を cancel して同じ goal を再送する。デフォルトは False。
        feedback_timeout_sec : float, optional
            retry_on_feedback_timeout が True の場合の feedback 待機時間。デフォルトは 5.0 秒。

        Returns
        -------
        bool
            ナビゲーションが成功（または tolerance 以内に到達）した場合は True、失敗またはキャンセルされた場合は False。
        """
        goal_pose = PoseStamped()

        if isinstance(pose, PoseStamped):
            goal_pose = pose
        elif isinstance(pose, Pose):
            goal_pose.header.frame_id = reference_frame
            goal_pose.header.stamp = self.node.get_clock().now().to_msg()
            goal_pose.pose = pose
        else:
            self.node.get_logger().error("pose must be PoseStamped or Pose")
            return False

        if use_odom_only:
            if retry_on_feedback_timeout:
                self.node.get_logger().warn(
                    "retry_on_feedback_timeout is ignored when use_odom_only=True."
                )
            return self.__move_to_pose_odom_only(
                goal_pose,
                tolerance=tolerance,
                wait=wait,
                timeout=timeout,
            )

        # Transform to map frame if not already in map frame
        if goal_pose.header.frame_id != "map":
            try:
                transform = self.tf_buffer.lookup_transform(
                    "map",
                    goal_pose.header.frame_id,
                    rclpy.time.Time(),
                    rclpy.duration.Duration(seconds=1.0),
                )

                import tf2_geometry_msgs

                goal_pose = tf2_geometry_msgs.do_transform_pose_stamped(
                    goal_pose, transform
                )
            except Exception as e:
                self.node.get_logger().error(
                    f"Failed to transform pose to map frame: {str(e)}"
                )
                return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        feedback_lock = threading.Lock()
        last_feedback_time = None
        goal_accept_time = time.monotonic()

        def feedback_callback(_feedback_msg):
            nonlocal last_feedback_time
            with feedback_lock:
                last_feedback_time = time.monotonic()

        def send_navigation_goal():
            nonlocal last_feedback_time
            with feedback_lock:
                last_feedback_time = None
            self.__debug_goal_pub.publish(goal_pose)
            return self.__action_client.send_goal_async(
                goal_msg,
                feedback_callback=feedback_callback,
            )

        def wait_for_goal_accept(goal_future):
            nonlocal goal_accept_time, last_feedback_time
            rclpy.spin_until_future_complete(
                self.node, goal_future, timeout_sec=10.0
            )
            if not goal_future.done():
                self.node.get_logger().error("Send goal timed out")
                return None

            accepted_goal_handle = goal_future.result()
            self.__current_goal_handle = accepted_goal_handle

            if accepted_goal_handle is None:
                self.node.get_logger().error("Goal response is empty")
                return None

            if not accepted_goal_handle.accepted:
                self.node.get_logger().error("Goal rejected by server")
                return None

            with feedback_lock:
                last_feedback_time = None
                goal_accept_time = time.monotonic()

            return accepted_goal_handle

        if retry_on_feedback_timeout and not wait:
            self.node.get_logger().warn(
                "retry_on_feedback_timeout requires wait=True and is ignored."
            )

        feedback_retry_enabled = (
            retry_on_feedback_timeout
            and wait
            and feedback_timeout_sec is not None
            and feedback_timeout_sec > 0.0
        )

        future = send_navigation_goal()

        if not wait:
            # 非同期モードの場合は送信完了まで少し待機して終了とする
            try:
                rclpy.spin_until_future_complete(self.node, future, timeout_sec=0.5)
            except KeyboardInterrupt:
                pass
            return True

        # 同期モード (wait=True)
        try:
            goal_handle = wait_for_goal_accept(future)
            if goal_handle is None:
                return False

            result_future = goal_handle.get_result_async()

            nav_success = False
            timeout_sec = self.TIMEOUT_SEC if timeout is None else timeout
            start_time = time.time()
            retry_count = 0
            while rclpy.ok() and not result_future.done():
                if timeout_sec is not None and timeout_sec > 0:
                    if time.time() - start_time > timeout_sec:
                        self.node.get_logger().error("TIMEOUT NAVIGATION!")
                        cancel_future = goal_handle.cancel_goal_async()
                        rclpy.spin_until_future_complete(
                            self.node, cancel_future, timeout_sec=5.0
                        )
                        return False

                rclpy.spin_once(self.node, timeout_sec=0.1)

                if feedback_retry_enabled:
                    with feedback_lock:
                        feedback_reference_time = (
                            last_feedback_time
                            if last_feedback_time is not None
                            else goal_accept_time
                        )

                    if (
                        time.monotonic() - feedback_reference_time
                        >= feedback_timeout_sec
                    ):
                        retry_count += 1
                        self.node.get_logger().warn(
                            "No navigation feedback for "
                            f"{feedback_timeout_sec:.1f} sec after goal accept. "
                            f"Canceling and resending goal (retry={retry_count})."
                        )

                        cancel_future = goal_handle.cancel_goal_async()
                        rclpy.spin_until_future_complete(
                            self.node, cancel_future, timeout_sec=2.0
                        )
                        if not cancel_future.done():
                            self.node.get_logger().warn(
                                "Cancel goal timed out before resend; resending anyway."
                            )

                        future = send_navigation_goal()
                        goal_handle = wait_for_goal_accept(future)
                        if goal_handle is None:
                            return False
                        result_future = goal_handle.get_result_async()
                        continue

                if tolerance is not None and tolerance > 0.0:
                    current_pose = self.get_current_pose(simple=True)
                    if current_pose is not None:
                        goal_x = goal_pose.pose.position.x
                        goal_y = goal_pose.pose.position.y
                        dist = math.sqrt(
                            (current_pose[0] - goal_x) ** 2
                            + (current_pose[1] - goal_y) ** 2
                        )

                        if dist <= tolerance:
                            self.node.get_logger().info(
                                f"Reached tolerance limit ({dist:.3f} <= {tolerance:.3f}). Canceling Nav2."
                            )
                            cancel_future = goal_handle.cancel_goal_async()
                            rclpy.spin_until_future_complete(
                                self.node, cancel_future, timeout_sec=5.0
                            )
                            self.__current_goal_handle = None
                            return self.__face_goal_pose(goal_pose)

            if not nav_success:
                result = result_future.result()
                if result.status == GoalStatus.STATUS_SUCCEEDED:
                    nav_success = True
                else:
                    self.node.get_logger().warn(
                        f"Navigation failed with status: {result.status}"
                    )
                    nav_success = False

            if nav_success:
                return True

            return False

        except KeyboardInterrupt:
            self.node.get_logger().warn(
                "KeyboardInterrupt: Canceling navigation goal..."
            )
            if self.__current_goal_handle:
                cancel_future = self.__current_goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(
                    self.node, cancel_future, timeout_sec=5.0
                )
                self.node.get_logger().info("Navigation goal canceled.")
            self.__current_goal_handle = None
            return False
        except Exception as e:
            self.node.get_logger().error(f"Navigation error: {str(e)}")
            return False

    def move_abs(
        self,
        x: float = 0.0,
        y: float = 0.0,
        yaw: float = 0.0,
        tolerance: float = 0.0,
        reference_frame: str = "map",
        wait: bool = True,
        timeout: float = None,
        use_odom_only: bool = False,
        retry_on_feedback_timeout: bool = True,
        feedback_timeout_sec: float = 5.0,
    ) -> bool:
        """
        基準フレームでの絶対座標を指定してロボットを自律移動させる．
        内部で move_to_pose() を呼び出す。

        Parameters
        ----------
        x : float, optional
            目標位置のX座標。デフォルトは 0.0。
        y : float, optional
            目標位置のY座標。デフォルトは 0.0。
        yaw : float, optional
            目標姿勢のヨー角（ラジアン）。デフォルトは 0.0。
        tolerance : float, optional
            目標からの許容誤差半径(m)。指定値以内に到達すれば終了する。デフォルトは 0.5。
        reference_frame : str, optional
            座標系の基準フレーム。デフォルトは 'map'。
        wait : bool, optional
            移動完了まで処理をブロックするかどうか。デフォルトは True。
        timeout : float, optional
            ナビゲーションのタイムアウト時間(秒)。デフォルトは None (タイムアウトなし)。
        use_odom_only : bool, optional
            True の場合、x, y, yaw を odom 座標系の絶対目標として扱い簡易移動する。
        retry_on_feedback_timeout : bool, optional
            True の場合、Action goal accept 後に feedback が一定時間返らないとき goal を再送する。
        feedback_timeout_sec : float, optional
            feedback 未受信時の再送判定時間。デフォルトは 5.0 秒。

        Returns
        -------
        bool
            ナビゲーションが成功した場合は True、失敗・キャンセルされた場合は False。
        """
        pose = PoseStamped()
        pose.header.frame_id = "odom" if use_odom_only else reference_frame
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0

        q = quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        return self.move_to_pose(
            pose,
            tolerance=tolerance,
            reference_frame=reference_frame,
            wait=wait,
            timeout=timeout,
            use_odom_only=use_odom_only,
            retry_on_feedback_timeout=retry_on_feedback_timeout,
            feedback_timeout_sec=feedback_timeout_sec,
        )

    def move_rel(
        self,
        x: float = 0.0,
        y: float = 0.0,
        yaw: float = 0.0,
        tolerance: float = 0.0,
        wait: bool = True,
        timeout: float = None,
        use_odom_only: bool = False,
        retry_on_feedback_timeout: bool = True,
        feedback_timeout_sec: float = 5.0,
    ) -> bool:
        """
        ロボットの現在の位置・姿勢からの相対座標で自律移動させる．
        内部で move_abs() を呼び出す。

        Parameters
        ----------
        x : float, optional
            ロボット前方への相対移動量(m)。デフォルトは 0.0。
        y : float, optional
            ロボット左方向への相対移動量(m)。デフォルトは 0.0。
        yaw : float, optional
            ロボットの現在角度からの相対的な反時計回りの回転量（ラジアン）。デフォルトは 0.0。
        tolerance : float, optional
            目標からの許容誤差半径(m)。指定値以内に到達すれば終了する。デフォルトは 0.5。
        wait : bool, optional
            移動完了まで処理をブロックするかどうか。デフォルトは True。
        timeout : float, optional
            ナビゲーションのタイムアウト時間(秒)。デフォルトは None (タイムアウトなし)。
        use_odom_only : bool, optional
            True の場合、現在 odom 姿勢からのロボット座標系相対量として簡易移動する。
        retry_on_feedback_timeout : bool, optional
            True の場合、Action goal accept 後に feedback が一定時間返らないとき goal を再送する。
        feedback_timeout_sec : float, optional
            feedback 未受信時の再送判定時間。デフォルトは 5.0 秒。

        Returns
        -------
        bool
            ナビゲーションが成功した場合は True、失敗・キャンセルされた場合は False。
        """
        if use_odom_only:
            if retry_on_feedback_timeout:
                self.node.get_logger().warn(
                    "retry_on_feedback_timeout is ignored when use_odom_only=True."
                )
            if not wait:
                self.node.get_logger().warn(
                    "use_odom_only=True does not support wait=False."
                )
                return False

            return self.__move_rel_by_odom_displacement(
                x=x,
                y=y,
                yaw=yaw,
                tolerance=tolerance,
                timeout=timeout,
            )

        current_pose = self.get_current_pose(simple=True)
        if current_pose is None:
            self.node.get_logger().error(
                "Could not get current pose for relative movement"
            )
            return False

        current_x, current_y, current_yaw = current_pose

        new_x = current_x + x * math.cos(current_yaw) - y * math.sin(current_yaw)
        new_y = current_y + x * math.sin(current_yaw) + y * math.cos(current_yaw)
        new_yaw = current_yaw + yaw

        return self.move_abs(
            x=new_x,
            y=new_y,
            yaw=new_yaw,
            tolerance=tolerance,
            reference_frame="map",
            wait=wait,
            timeout=timeout,
            retry_on_feedback_timeout=retry_on_feedback_timeout,
            feedback_timeout_sec=feedback_timeout_sec,
        )

    def set_initialpose(
        self,
        pose,
        reference_frame: str = "map",
        xyy: bool = True,
        tolerance: float = 0.3,
        max_attempts: int = 1,
        settle_time: float = 1.5,
    ) -> bool:
        """
        ロボットの初期位置（Initial Pose）を設定する．
        ローカライゼーションノードに対して /initialpose トピックをパブリッシュする。

        Parameters
        ----------
        pose : list of float or PoseWithCovarianceStamped
            xyy=True の場合は [x, y, yaw] の形式。
            xyy=False の場合は PoseWithCovarianceStamped 形式。
        reference_frame : str, optional
            基準となる座標フレーム。デフォルトは 'map'。
        xyy : bool, optional
            True の場合、pose を [x, y, yaw] として扱う。False の場合、pose を PoseWithCovarianceStamped として扱う。
        tolerance : float, optional
            初期位置反映後の現在位置と指定位置の許容距離[m]。デフォルトは 0.3。
        max_attempts : int, optional
            初期位置 publish と確認を繰り返す最大回数。デフォルトは 1。
        settle_time : float, optional
            publish 後に localization の反映を待つ時間[秒]。デフォルトは 1.5。

        Returns
        -------
        bool
            初期位置が tolerance 内に反映された場合は True、失敗した場合は False。
        """
        if xyy:
            if not (isinstance(pose, list) and len(pose) == 3):
                self.node.get_logger().error(
                    "Invalid pose format for set_initialpose. Use [x, y, yaw] when xyy=True."
                )
                return False

            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = reference_frame
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.pose.pose.position.x = float(pose[0])
            msg.pose.pose.position.y = float(pose[1])
            # TODO: 変数として受け取るような仕様のほうがいいかも？
            # 1.3 is unitree g1 lidar height from ground level
            # msg.pose.pose.position.z = 0.75
            msg.pose.pose.position.z = 0.0

            q = quaternion_from_euler(0, 0, pose[2])
            msg.pose.pose.orientation.x = q[0]
            msg.pose.pose.orientation.y = q[1]
            msg.pose.pose.orientation.z = q[2]
            msg.pose.pose.orientation.w = q[3]

            # Covariance - typical reasonable defaults for a manual reset
            msg.pose.covariance[0] = 0.25
            msg.pose.covariance[7] = 0.25
            msg.pose.covariance[35] = 0.06853891945200942

            target_x = float(pose[0])
            target_y = float(pose[1])
            target_yaw = float(pose[2])
        else:
            if not isinstance(pose, PoseWithCovarianceStamped):
                self.node.get_logger().error(
                    "Invalid pose format for set_initialpose. Use PoseWithCovarianceStamped when xyy=False."
                )
                return False

            msg = copy.deepcopy(pose)
            target_x = float(msg.pose.pose.position.x)
            target_y = float(msg.pose.pose.position.y)
            q = msg.pose.pose.orientation
            (_, _, target_yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])

        if msg.header.frame_id != "map":
            self.node.get_logger().warn(
                "set_initialpose verification compares against get_current_pose() in map frame, "
                f"but initial pose frame is '{msg.header.frame_id}'."
            )

        attempts = max(1, int(max_attempts))
        tolerance = max(0.0, float(tolerance))
        settle_time = max(0.0, float(settle_time))

        for attempt in range(1, attempts + 1):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.__initial_pose_pub.publish(msg)
            self.node.get_logger().info(
                "Published initial pose to /initialpose "
                f"(attempt {attempt}/{attempts}, frame={msg.header.frame_id})"
            )

            time.sleep(settle_time)

            current_pose = self.get_current_pose(simple=True)
            if current_pose is None:
                self.node.get_logger().warn(
                    f"Initial pose verification failed on attempt {attempt}: current pose unavailable."
                )
                continue

            current_x, current_y, current_yaw = current_pose
            distance_error = math.hypot(current_x - target_x, current_y - target_y)
            yaw_error = math.atan2(
                math.sin(current_yaw - target_yaw),
                math.cos(current_yaw - target_yaw),
            )

            if distance_error <= tolerance:
                self.node.get_logger().info(
                    "Initial pose verified: "
                    f"position_error={distance_error:.3f} m <= {tolerance:.3f} m, cx {current_x:.3f} cy {current_y:.3f}"
                    f"yaw_error={yaw_error:.3f} rad"
                )
                return True

            self.node.get_logger().warn(
                "Initial pose is outside tolerance after publish: "
                f"position_error={distance_error:.3f} m > {tolerance:.3f} m, "
                f"yaw_error={yaw_error:.3f} rad "
                f"cx {current_x:.3f} cy {current_y:.3f}"
            )

        self.node.get_logger().error(
            "Failed to verify initial pose after "
            f"{attempts} attempts: target=({target_x:.3f}, {target_y:.3f}, {target_yaw:.3f}), "
            f"tolerance={tolerance:.3f} m"
        )
        return False


class ArmControlStatus(str, Enum):
    """マニピュレーション要求の結果。"""

    SUCCEEDED = "succeeded"
    DEGRADED = "degraded"
    SUBMITTED = "submitted"
    INVALID_ARGUMENT = "invalid_argument"
    NOT_READY = "not_ready"
    STATE_UNAVAILABLE = "state_unavailable"
    TF_UNAVAILABLE = "tf_unavailable"
    GOAL_REJECTED = "goal_rejected"
    MOVEIT_FAILED = "moveit_failed"
    CANCELLED = "cancelled"
    TIMED_OUT = "timed_out"
    ROS_SHUTDOWN = "ros_shutdown"
    INTERNAL_ERROR = "internal_error"


@dataclass(frozen=True)
class ArmControlResult:
    """受付と完了、通常成功と位置のみの縮退成功を区別する。"""

    status: ArmControlStatus
    message: str = ""
    moveit_error_code: Optional[int] = None
    moveit_error_name: Optional[str] = None
    action_status: Optional[int] = None
    plan_only: bool = False
    used_ik: bool = False
    used_position_only_fallback: bool = False

    @property
    def succeeded(self) -> bool:
        return self.status in (
            ArmControlStatus.SUCCEEDED, ArmControlStatus.DEGRADED)

    @property
    def strict_success(self) -> bool:
        return self.status == ArmControlStatus.SUCCEEDED


class IkStatus(str, Enum):
    """IK の失敗原因。"""

    SUCCEEDED = "succeeded"
    NO_SOLUTION = "no_solution"
    SERVICE_UNAVAILABLE = "service_unavailable"
    TIMED_OUT = "timed_out"
    INVALID_REQUEST = "invalid_request"
    INVALID_RESPONSE = "invalid_response"
    STATE_UNAVAILABLE = "state_unavailable"
    CANCELLED = "cancelled"
    ROS_SHUTDOWN = "ros_shutdown"
    INTERNAL_ERROR = "internal_error"


@dataclass(frozen=True)
class IkResult:
    status: IkStatus
    joints: Optional[dict[str, float]] = None
    message: str = ""
    moveit_error_code: Optional[int] = None


class _FutureWaitStatus(str, Enum):
    COMPLETED = "completed"
    CANCELLED = "cancelled"
    TIMED_OUT = "timed_out"
    ROS_SHUTDOWN = "ros_shutdown"


def _finite(value: Any) -> bool:
    """bool、非数値、非有限値を除外する。"""
    if isinstance(value, (bool, str, bytes)):
        return False
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError, OverflowError):
        return False


def _positive(name: str, value: Any) -> float:
    if not _finite(value) or float(value) <= 0.0:
        raise ValueError(f"{name} は正の有限数で指定してください")
    return float(value)


def _name(value: Any) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError("名前・座標系は空でない文字列が必要です")
    return value


def _unit_quaternion(value: Quaternion) -> Quaternion:
    if not isinstance(value, Quaternion):
        raise ValueError("Quaternion が必要です")
    values = (value.x, value.y, value.z, value.w)
    if not all(_finite(v) for v in values):
        raise ValueError("Quaternion に非有限値があります")
    norm = math.hypot(*values)
    if not math.isfinite(norm) or norm <= 1e-12:
        raise ValueError("Quaternion の長さが不正です")
    return Quaternion(**dict(zip(
        ("x", "y", "z", "w"), (v / norm for v in values))))


def _quaternion_product(a: Quaternion, b: Quaternion) -> Quaternion:
    a, b = _unit_quaternion(a), _unit_quaternion(b)
    return _unit_quaternion(Quaternion(
        x=a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        y=a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        z=a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w,
        w=a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z))


def _quaternion_euler(roll: float, pitch: float, yaw: float) -> Quaternion:
    if not all(_finite(v) for v in (roll, pitch, yaw)):
        raise ValueError("姿勢角は有限数で指定してください")
    values = quaternion_from_euler(float(roll), float(pitch), float(yaw))
    return _unit_quaternion(Quaternion(
        x=float(values[0]), y=float(values[1]),
        z=float(values[2]), w=float(values[3])))


def _pose_valid(pose: Pose) -> Pose:
    if not isinstance(pose, Pose):
        raise ValueError("Pose が必要です")
    if not all(_finite(v) for v in (
            pose.position.x, pose.position.y, pose.position.z)):
        raise ValueError("位置は有限数で指定してください")
    result = copy.deepcopy(pose)
    result.orientation = _unit_quaternion(pose.orientation)
    return result


def _compose_pose(parent: Pose, child: Pose) -> Pose:
    """親座標の回転も含めて Pose を合成する。"""
    parent, child = _pose_valid(parent), _pose_valid(child)
    q = parent.orientation
    matrix = tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
    vector = matrix[:3, :3].dot([
        child.position.x, child.position.y, child.position.z])
    result = Pose()
    result.position.x = float(parent.position.x + vector[0])
    result.position.y = float(parent.position.y + vector[1])
    result.position.z = float(parent.position.z + vector[2])
    result.orientation = _quaternion_product(
        parent.orientation, child.orientation)
    return _pose_valid(result)


class _ArmRosSupport:
    """外部 executor を奪わず、実時間で期限を管理する。"""

    _SPIN_INTERVAL = 0.02

    def _context_ok(self) -> bool:
        try:
            return bool(self.node.context.ok())
        except (AttributeError, ExternalShutdownException, RCLError,
                InvalidHandle):
            return False

    def _spin_once(self, timeout: float) -> bool:
        if not self._context_ok():
            return False
        timeout = max(0.0, min(timeout, self._SPIN_INTERVAL))
        executor = self.node.executor
        if executor is not None:
            time.sleep(timeout)
            return self._context_ok()
        if not self._spin_lock.acquire(blocking=False):
            time.sleep(timeout)
            return self._context_ok()
        try:
            if executor is None:
                rclpy.spin_once(self.node, timeout_sec=timeout)
            else:
                executor.spin_once(timeout_sec=timeout)
            return self._context_ok()
        except (ExternalShutdownException, RCLError, InvalidHandle):
            return False
        finally:
            self._spin_lock.release()

    def _safe_service_ready(
        self, timeout: float = 0.0, client: Any = None,
    ) -> bool:
        client = self._ik_client if client is None else client
        if not self._context_ok():
            return False
        try:
            return bool(client.wait_for_service(timeout_sec=timeout))
        except (ExternalShutdownException, RCLError, InvalidHandle):
            return False

    def _wait_for_future(
        self, future: Any, timeout: float, *,
        cancel_event: Optional[threading.Event] = None,
        observe_cancel: bool = True,
    ) -> _FutureWaitStatus:
        deadline = time.monotonic() + _positive("timeout", timeout)
        while self._context_ok():
            if observe_cancel and (
                    self._cancel_requested.is_set()
                    or (cancel_event is not None and cancel_event.is_set())):
                return _FutureWaitStatus.CANCELLED
            if future.done():
                return _FutureWaitStatus.COMPLETED
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return _FutureWaitStatus.TIMED_OUT
            if not self._spin_once(remaining):
                return _FutureWaitStatus.ROS_SHUTDOWN
        return _FutureWaitStatus.ROS_SHUTDOWN

    def _call_service(
        self, client: Any, request: Any, timeout: float, *,
        cancel_event: Optional[threading.Event] = None,
    ) -> tuple[_FutureWaitStatus, Any]:
        """発見待ちと応答待ちで一つの期限を共有する。"""
        if not _finite(timeout):
            raise ValueError("timeout は有限数で指定してください")
        if timeout <= 0.0:
            return _FutureWaitStatus.TIMED_OUT, None
        deadline = time.monotonic() + float(timeout)
        while not self._safe_service_ready(0.0, client):
            if not self._context_ok():
                return _FutureWaitStatus.ROS_SHUTDOWN, None
            if self._cancel_requested.is_set() or (
                    cancel_event is not None and cancel_event.is_set()):
                return _FutureWaitStatus.CANCELLED, None
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return _FutureWaitStatus.TIMED_OUT, None
            self._spin_once(remaining)
        if self._cancel_requested.is_set() or (
                cancel_event is not None and cancel_event.is_set()):
            return _FutureWaitStatus.CANCELLED, None
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return _FutureWaitStatus.TIMED_OUT, None
        future = client.call_async(request)
        status = self._wait_for_future(
            future, remaining, cancel_event=cancel_event)
        if status != _FutureWaitStatus.COMPLETED:
            # サービス要求の実行取消ではなく、ローカルの応答追跡を除去する。
            client.remove_pending_request(future)
            return status, None
        return status, future.result()


class ArmCollision(_ArmRosSupport):
    """PlanningScene の操作を応答確認付きで実施する。"""

    def __init__(
        self, node: Node, *, timeout: float = 3.0,
        tf_buffer: Optional[Buffer] = None,
        spin_lock: Optional[Any] = None,
    ) -> None:
        self.node = node
        self.timeout = _positive("timeout", timeout)
        self._spin_lock = spin_lock or threading.Lock()
        self._cancel_requested = threading.Event()
        self._scene_lock = threading.Lock()
        self._get_client = node.create_client(
            GetPlanningScene, "/get_planning_scene")
        self._apply_client = node.create_client(
            ApplyPlanningScene, "/apply_planning_scene")
        self.tf_buffer = tf_buffer if tf_buffer is not None else Buffer()
        self._tf_listener = (
            TransformListener(self.tf_buffer, node)
            if tf_buffer is None else None)

    def _deadline(self, timeout: Optional[float]) -> float:
        return time.monotonic() + _positive(
            "timeout", self.timeout if timeout is None else timeout)

    def _scene(
        self, deadline: float, components: int,
    ) -> Optional[PlanningScene]:
        request = GetPlanningScene.Request()
        request.components.components = components
        _, response = self._call_service(
            self._get_client, request, deadline - time.monotonic())
        return None if response is None else response.scene

    def _apply(self, scene: PlanningScene, deadline: float) -> bool:
        scene.is_diff = True
        scene.robot_state.is_diff = True
        request = ApplyPlanningScene.Request()
        request.scene = scene
        _, response = self._call_service(
            self._apply_client, request, deadline - time.monotonic())
        return response is not None and bool(response.success)

    def _error(self, error: Exception) -> None:
        if self._context_ok():
            self.node.get_logger().error(f"PlanningScene 操作失敗: {error}")

    def _add(
        self, name: str, ref: str, values: tuple,
        shape: int, dimensions: list, timeout: Optional[float],
    ) -> bool:
        try:
            deadline = self._deadline(timeout)
            _name(name)
            _name(ref)
            if not all(_finite(v) for v in values):
                raise ValueError("位置・姿勢は有限数で指定してください")
            obj = CollisionObject()
            obj.id, obj.header.frame_id = name, ref
            obj.operation = CollisionObject.ADD
            obj.pose.orientation.w = 1.0
            primitive = SolidPrimitive()
            primitive.type = shape
            primitive.dimensions = [
                _positive("dimension", v) for v in dimensions]
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = map(
                float, values[:3])
            pose.orientation = _quaternion_euler(*values[3:])
            obj.primitives = [primitive]
            obj.primitive_poses = [pose]
            scene = PlanningScene()
            scene.world.collision_objects = [obj]
            return self._apply(scene, deadline)
        except Exception as error:
            self._error(error)
            return False

    def add_box(
        self, name: str, ref: str = "base_link", x: float = 0.0,
        y: float = 0.0, z: float = 0.0, roll: float = 0.0,
        pitch: float = 0.0, yaw: float = 0.0,
        size: tuple = (0.1, 0.1, 0.1), *,
        timeout: Optional[float] = None,
    ) -> bool:
        try:
            if len(size) != 3:
                raise ValueError("箱の寸法は 3 要素が必要です")
            return self._add(
                name, ref, (x, y, z, roll, pitch, yaw),
                SolidPrimitive.BOX, list(size), timeout)
        except (TypeError, ValueError) as error:
            self._error(error)
            return False

    def add_sphere(
        self, name: str, ref: str = "base_link", x: float = 0.0,
        y: float = 0.0, z: float = 0.0, radius: float = 0.05, *,
        timeout: Optional[float] = None,
    ) -> bool:
        return self._add(
            name, ref, (x, y, z, 0.0, 0.0, 0.0),
            SolidPrimitive.SPHERE, [radius], timeout)

    def add_cylinder(
        self, name: str, ref: str = "base_link", x: float = 0.0,
        y: float = 0.0, z: float = 0.0, roll: float = 0.0,
        pitch: float = 0.0, yaw: float = 0.0,
        height: float = 0.1, radius: float = 0.05, *,
        timeout: Optional[float] = None,
    ) -> bool:
        return self._add(
            name, ref, (x, y, z, roll, pitch, yaw),
            SolidPrimitive.CYLINDER, [height, radius], timeout)

    @staticmethod
    def _removal(name: str) -> PlanningScene:
        scene = PlanningScene()
        obj = CollisionObject()
        obj.id, obj.operation = _name(name), CollisionObject.REMOVE
        scene.world.collision_objects = [obj]
        attached = AttachedCollisionObject()
        attached.object = copy.deepcopy(obj)
        scene.robot_state.attached_collision_objects = [attached]
        return scene

    def remove_collision(
        self, name: str, *, timeout: Optional[float] = None,
    ) -> bool:
        try:
            return self._apply(self._removal(name), self._deadline(timeout))
        except Exception as error:
            self._error(error)
            return False

    def get_object(
        self, name: str, *, timeout: Optional[float] = None,
    ) -> Optional[CollisionObject]:
        try:
            _name(name)
            scene = self._scene(
                self._deadline(timeout),
                PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
                | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
            if scene is None:
                return None
            objects = list(scene.world.collision_objects) + [
                a.object for a in scene.robot_state.attached_collision_objects]
            return next((copy.deepcopy(o) for o in objects if o.id == name),
                        None)
        except Exception as error:
            self._error(error)
            return None

    @staticmethod
    def _object_pose(obj: CollisionObject) -> Pose:
        poses = obj.primitive_poses or obj.mesh_poses or obj.plane_poses
        return (_compose_pose(obj.pose, poses[0]) if poses
                else _pose_valid(obj.pose))

    def get_object_pose(
        self, name: str, *, timeout: Optional[float] = None,
    ) -> Optional[tuple[float, ...]]:
        obj = self.get_object(name, timeout=timeout)
        if obj is None:
            return None
        try:
            pose = self._object_pose(obj)
            q = pose.orientation
            return (pose.position.x, pose.position.y, pose.position.z,
                    *euler_from_quaternion([q.x, q.y, q.z, q.w]))
        except ValueError as error:
            self._error(error)
            return None

    def _in_frame(
        self, pose: Pose, source: str, target: str, deadline: float,
    ) -> Pose:
        if source == target:
            return pose
        while self._context_ok() and time.monotonic() < deadline:
            try:
                tf = self.tf_buffer.lookup_transform(
                    target, source, rclpy.time.Time())
                parent = Pose()
                p = tf.transform.translation
                parent.position.x, parent.position.y = p.x, p.y
                parent.position.z = p.z
                parent.orientation = tf.transform.rotation
                return _compose_pose(parent, pose)
            except TransformException:
                self._spin_once(deadline - time.monotonic())
        raise ValueError(f"TF を取得できません: {source} -> {target}")

    def remove_near_objects(
        self, x: float, y: float, z: float, radius: float = 0.05, *,
        reference_frame: str = "base_link",
        timeout: Optional[float] = None,
    ) -> bool:
        try:
            if not all(_finite(v) for v in (x, y, z)):
                raise ValueError("位置は有限数で指定してください")
            radius = _positive("radius", radius)
            _name(reference_frame)
            deadline = self._deadline(timeout)
            scene = self._scene(
                deadline, PlanningSceneComponents.WORLD_OBJECT_GEOMETRY)
            if scene is None:
                return False
            diff = PlanningScene()
            for obj in scene.world.collision_objects:
                pose = self._in_frame(
                    self._object_pose(obj), _name(obj.header.frame_id),
                    reference_frame, deadline)
                p = pose.position
                if math.dist((float(x), float(y), float(z)),
                             (p.x, p.y, p.z)) <= radius:
                    diff.world.collision_objects.extend(
                        self._removal(obj.id).world.collision_objects)
            return not diff.world.collision_objects or self._apply(
                diff, deadline)
        except Exception as error:
            self._error(error)
            return False

    def clear_all(self, *, timeout: Optional[float] = None) -> bool:
        try:
            deadline = self._deadline(timeout)
            scene = self._scene(
                deadline, PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
                | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
            if scene is None:
                return False
            diff = PlanningScene()
            names = {o.id for o in scene.world.collision_objects}
            names.update(a.object.id
                         for a in scene.robot_state.attached_collision_objects)
            for name in names:
                removal = self._removal(name)
                diff.world.collision_objects.extend(
                    removal.world.collision_objects)
                diff.robot_state.attached_collision_objects.extend(
                    removal.robot_state.attached_collision_objects)
            return not names or self._apply(diff, deadline)
        except Exception as error:
            self._error(error)
            return False

    def attach_collision(
        self, name: str, link_name: str,
        touch_links: Optional[list] = None,
        collision_object: Optional[CollisionObject] = None, *,
        timeout: Optional[float] = None,
    ) -> bool:
        try:
            deadline = self._deadline(timeout)
            _name(name)
            _name(link_name)
            if touch_links is not None and not isinstance(
                    touch_links, (list, tuple)):
                raise ValueError("touch_links はリンク名の列が必要です")
            links = [_name(v) for v in (touch_links or [link_name])]
            obj = (self.get_object(
                name, timeout=deadline - time.monotonic())
                if collision_object is None else copy.deepcopy(
                    collision_object))
            if not isinstance(obj, CollisionObject):
                raise ValueError("アタッチ対象が見つかりません")
            _name(obj.header.frame_id)
            obj.pose = _pose_valid(obj.pose)
            if not (obj.primitives or obj.meshes or obj.planes):
                raise ValueError("アタッチ対象の形状がありません")
            for shapes, poses in (
                    (obj.primitives, obj.primitive_poses),
                    (obj.meshes, obj.mesh_poses),
                    (obj.planes, obj.plane_poses)):
                if len(shapes) != len(poses):
                    raise ValueError("形状と姿勢の数が一致しません")
                for index, pose in enumerate(poses):
                    poses[index] = _pose_valid(pose)
            for primitive in obj.primitives:
                counts = {SolidPrimitive.BOX: 3, SolidPrimitive.SPHERE: 1,
                          SolidPrimitive.CYLINDER: 2, SolidPrimitive.CONE: 2}
                if len(primitive.dimensions) != counts.get(primitive.type):
                    raise ValueError("プリミティブ寸法が不正です")
                for dimension in primitive.dimensions:
                    _positive("dimension", dimension)
            obj.id, obj.operation = name, CollisionObject.ADD
            attached = AttachedCollisionObject()
            attached.link_name, attached.touch_links = link_name, links
            attached.object = obj
            diff = self._removal(name)
            diff.robot_state.attached_collision_objects = [attached]
            return self._apply(diff, deadline)
        except Exception as error:
            self._error(error)
            return False

    def allow_collision(
        self, name1: str, name2: str, *,
        timeout: Optional[float] = None,
    ) -> bool:
        try:
            _name(name1)
            _name(name2)
            deadline = self._deadline(timeout)
            if not self._scene_lock.acquire(
                    timeout=max(0.0, deadline - time.monotonic())):
                return False
            try:
                scene = self._scene(
                    deadline, PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)
                if scene is None:
                    return False
                acm = scene.allowed_collision_matrix
                size = len(acm.entry_names)
                if len(acm.entry_values) != size or any(
                        len(e.enabled) != size for e in acm.entry_values):
                    raise ValueError("衝突許可行列の次元が不正です")
                for name in (name1,) if name2 == "all" else (name1, name2):
                    if name not in acm.entry_names:
                        acm.entry_names.append(name)
                        for row in acm.entry_values:
                            row.enabled.append(False)
                        row = AllowedCollisionEntry()
                        row.enabled = [False] * len(acm.entry_names)
                        acm.entry_values.append(row)
                first = acm.entry_names.index(name1)
                targets = (range(len(acm.entry_names)) if name2 == "all"
                           else [acm.entry_names.index(name2)])
                for second in targets:
                    acm.entry_values[first].enabled[second] = True
                    acm.entry_values[second].enabled[first] = True
                if name2 == "all":
                    if name1 in acm.default_entry_names:
                        acm.default_entry_values[
                            acm.default_entry_names.index(name1)] = True
                    else:
                        acm.default_entry_names.append(name1)
                        acm.default_entry_values.append(True)
                diff = PlanningScene()
                diff.allowed_collision_matrix = acm
                return self._apply(diff, deadline)
            finally:
                self._scene_lock.release()
        except Exception as error:
            self._error(error)
            return False


@dataclass
class _ArmGoal:
    """遅延した受付応答でも取消要求を失わないゴール単位の記録。"""

    future: Any
    goal: Any
    response_deadline: float
    result_timeout: float
    cancel_event: Optional[threading.Event]
    used_ik: bool = False
    position_only: bool = False
    handle: Any = None
    result_future: Any = None
    cancel_future: Any = None
    result_deadline: Optional[float] = None
    abort_status: Optional[ArmControlStatus] = None
    result: Optional[ArmControlResult] = None


def _arm_result_guard(method: Any) -> Any:
    """公開詳細 API の例外を構造化結果へ変換する。"""
    @wraps(method)
    def guarded(self: Any, *args: Any, **kwargs: Any) -> ArmControlResult:
        try:
            if not self._context_ok():
                result = ArmControlResult(ArmControlStatus.ROS_SHUTDOWN)
            elif self._cancel_requested.is_set():
                result = ArmControlResult(ArmControlStatus.CANCELLED)
            else:
                result = method(self, *args, **kwargs)
        except (ValueError, TypeError, OverflowError) as error:
            result = ArmControlResult(
                ArmControlStatus.INVALID_ARGUMENT, str(error))
        except (ExternalShutdownException, RCLError, InvalidHandle):
            result = ArmControlResult(ArmControlStatus.ROS_SHUTDOWN)
        except Exception as error:
            result = ArmControlResult(
                ArmControlStatus.INTERNAL_ERROR, str(error))
        if kwargs.get("execute") is False and not result.plan_only:
            result = replace(result, plan_only=True)
        return result
    return guarded


class ArmControl(_ArmRosSupport):
    """G1 の既存 bool API と詳細結果 API を提供する。

    wait=False の完了監視には node の executor を継続して spin する。
    同じ executor の単一スレッド callback から同期 API は呼ばない。
    """

    _GROUPS = (
        "arm_left", "arm_left_with_waist", "arm_right",
        "arm_right_with_waist", "arm_both", "arm_both_with_waist", "head",
    )
    _SINGLE_GROUPS = _GROUPS[:4]
    _PLAN_FAILURES = (
        MoveItErrorCodes.PLANNING_FAILED,
        MoveItErrorCodes.INVALID_MOTION_PLAN,
        MoveItErrorCodes.NO_IK_SOLUTION,
        MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED,
    )

    def __init__(
        self, node: Node, wait_time: int = 5, tf_buffer: Buffer = None,
    ) -> None:
        if not _finite(wait_time) or wait_time < 0:
            raise ValueError("wait_time は非負の有限数が必要です")
        self.node = node
        self.base_frame = "base_link"
        self.position_tolerance = 0.001
        self.orientation_tolerance = 0.1
        self.joint_tolerance = 0.01
        self.joint_state_max_age = 1.0
        self.goal_response_timeout = 5.0
        self.result_timeout = 30.0
        self.cancel_timeout = 2.0
        self._velocity_scale = 1.0
        self._acceleration_scale = 1.0
        self._spin_lock = threading.Lock()
        self._joint_state_lock = threading.RLock()
        self._goal_lock = threading.RLock()
        self._cancel_requested = threading.Event()
        self._joint_states: dict[str, float] = {}
        self._joint_state_updated_at: dict[str, float] = {}
        self._goals: dict[Any, _ArmGoal] = {}
        self.last_result: Optional[ArmControlResult] = None
        self._groups: dict[str, tuple[str, ...]] = {}
        self._srdf_group_states: Optional[dict] = None
        self.tf_buffer = tf_buffer if tf_buffer is not None else Buffer()
        self._tf_listener = (
            TransformListener(self.tf_buffer, node)
            if tf_buffer is None else None)
        self._move_group_client = ActionClient(node, MoveGroup, "/move_action")
        self._ik_client = node.create_client(GetPositionIK, "/compute_ik")
        self._fk_client = node.create_client(GetPositionFK, "/compute_fk")
        self._hand_client = node.create_client(HandCommand, "/hand_command")
        self._ubc_client = node.create_client(
            SetBool, "/enable_upper_body_control")
        self._joint_sub = node.create_subscription(
            JointState, "/joint_states", self._joint_state_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.collision = ArmCollision(
            node, tf_buffer=self.tf_buffer, spin_lock=self._spin_lock)
        # ROS 時刻が停止しても非同期ゴールの期限を監視する。
        self._goal_timer = node.create_timer(
            self._SPIN_INTERVAL, self._monitor_goals,
            clock=Clock(clock_type=ClockType.STEADY_TIME))
        self._load_srdf_group_states()
        if wait_time > 0:
            self.wait_until_ready(float(wait_time), require_ik=False)

    @staticmethod
    def _canonical_group(name: str) -> str:
        return {
            "upper_body": "arm_both_with_waist",
            "__pure_arm_left": "arm_left",
            "__pure_arm_right": "arm_right",
        }.get(name, name)

    @staticmethod
    def _scale(name: str, value: Any) -> float:
        value = _positive(name, value)
        if value > 1.0:
            raise ValueError(f"{name} は 1.0 以下が必要です")
        return value

    @property
    def velocity_scale(self) -> float:
        return self._velocity_scale

    @velocity_scale.setter
    def velocity_scale(self, value: float) -> None:
        self._velocity_scale = self._scale("velocity_scale", value)

    @property
    def acceleration_scale(self) -> float:
        return self._acceleration_scale

    @acceleration_scale.setter
    def acceleration_scale(self, value: float) -> None:
        self._acceleration_scale = self._scale("acceleration_scale", value)

    def set_motion_scaling(
        self, velocity_scale: Optional[float] = None,
        acceleration_scale: Optional[float] = None,
    ) -> None:
        """両方を検証してから反映し、片方だけの変更を防ぐ。"""
        velocity = (self.velocity_scale if velocity_scale is None
                    else self._scale("velocity_scale", velocity_scale))
        acceleration = (self.acceleration_scale if acceleration_scale is None
                        else self._scale(
                            "acceleration_scale", acceleration_scale))
        self._velocity_scale, self._acceleration_scale = velocity, acceleration

    @staticmethod
    def _to_legacy_bool(result: ArmControlResult) -> bool:
        return result.succeeded or result.status == ArmControlStatus.SUBMITTED

    @staticmethod
    def _result(
        status: ArmControlStatus, message: str = "", **fields: Any,
    ) -> ArmControlResult:
        return ArmControlResult(status, message, **fields)

    def _safe_action_ready(self, timeout: float = 0.0) -> bool:
        if not self._context_ok():
            return False
        try:
            return bool(self._move_group_client.wait_for_server(
                timeout_sec=timeout))
        except (ExternalShutdownException, RCLError, InvalidHandle):
            return False

    def is_move_group_ready(self) -> bool:
        return self._safe_action_ready()

    def is_ik_ready(self) -> bool:
        return self._safe_service_ready()

    def wait_until_ready(
        self, timeout: float = 5.0, require_ik: bool = True,
    ) -> ArmControlResult:
        try:
            deadline = time.monotonic() + _positive("timeout", timeout)
            while self._context_ok():
                if self._cancel_requested.is_set():
                    return self._result(ArmControlStatus.CANCELLED)
                if self.is_move_group_ready() and (
                        not require_ik or self.is_ik_ready()):
                    return self._result(ArmControlStatus.SUCCEEDED)
                if time.monotonic() >= deadline:
                    return self._result(
                        ArmControlStatus.NOT_READY, "MoveIt が未起動です")
                self._spin_once(deadline - time.monotonic())
            return self._result(ArmControlStatus.ROS_SHUTDOWN)
        except (TypeError, ValueError) as error:
            return self._result(ArmControlStatus.INVALID_ARGUMENT, str(error))

    def _joint_state_callback(self, message: JointState) -> None:
        if (len(message.name) != len(message.position)
                or len(set(message.name)) != len(message.name)):
            return
        received = time.monotonic()
        with self._joint_state_lock:
            for name, position in zip(message.name, message.position):
                if name and _finite(position):
                    self._joint_states[name] = float(position)
                    self._joint_state_updated_at[name] = received
                else:
                    self._joint_state_updated_at.pop(name, None)

    def get_current_joints_pose(
        self, planning_group: str = "arm_both_with_waist",
    ) -> dict[str, float]:
        # 旧仕様どおり受信済み全関節のコピーを返す。
        with self._joint_state_lock:
            return self._joint_states.copy()

    def get_joint_state_age(
        self, joint_name: Optional[str] = None,
    ) -> Optional[float]:
        with self._joint_state_lock:
            stamps = ([self._joint_state_updated_at[joint_name]]
                      if joint_name in self._joint_state_updated_at
                      else [] if joint_name is not None
                      else list(self._joint_state_updated_at.values()))
        return time.monotonic() - min(stamps) if stamps else None

    def _fresh_joint_snapshot(
        self, joints: tuple[str, ...],
    ) -> tuple[Optional[dict[str, float]], list[str]]:
        now = time.monotonic()
        with self._joint_state_lock:
            missing = [
                name for name in joints
                if name not in self._joint_states
                or name not in self._joint_state_updated_at
                or now - self._joint_state_updated_at[name]
                > self.joint_state_max_age]
            return (None, missing) if missing else (
                {name: self._joint_states[name] for name in joints}, [])

    def wait_for_joint_states(
        self, planning_group: str = "arm_both_with_waist",
        timeout: float = 2.0,
    ) -> ArmControlResult:
        try:
            joints = self._joints_for_group(planning_group)
            deadline = time.monotonic() + _positive("timeout", timeout)
            while self._context_ok():
                if self._cancel_requested.is_set():
                    return self._result(ArmControlStatus.CANCELLED)
                snapshot, missing = self._fresh_joint_snapshot(joints)
                if snapshot is not None:
                    return self._result(ArmControlStatus.SUCCEEDED)
                if time.monotonic() >= deadline:
                    return self._result(
                        ArmControlStatus.STATE_UNAVAILABLE,
                        f"関節状態が未受信または古いです: {missing}")
                self._spin_once(deadline - time.monotonic())
            return self._result(ArmControlStatus.ROS_SHUTDOWN)
        except (TypeError, ValueError) as error:
            return self._result(ArmControlStatus.INVALID_ARGUMENT, str(error))

    def _load_srdf_group_states(self) -> Optional[dict]:
        if self._srdf_group_states is not None:
            return self._srdf_group_states
        try:
            path = Path(get_package_share_directory(
                "erasers_g1_moveit")) / "config" / "g1.srdf"
            root = ET.parse(path).getroot()
            groups = {g.get("name") for g in root.findall("group")}
            states = {}
            for state in root.findall("group_state"):
                group, name = state.get("group"), state.get("name")
                if group not in self._GROUPS:
                    continue
                if not name or group not in groups or (group, name) in states:
                    raise ValueError("SRDF のグループ・姿勢名が不正です")
                joints = {}
                for joint in state.findall("joint"):
                    key, value = _name(joint.get("name")), float(
                        joint.get("value"))
                    if key in joints or not _finite(value):
                        raise ValueError("SRDF の関節値が不正です")
                    joints[key] = value
                if not joints:
                    raise ValueError("SRDF の姿勢が空です")
                states[(group, name)] = joints
            definitions = {
                group: tuple(states[(group, "home")])
                for group in self._GROUPS if (group, "home") in states}
            if not all(group in definitions for group in self._GROUPS[:6]):
                raise ValueError("G1 の 6 グループの home 定義が必要です")
            left, right = (set(definitions["arm_left"]),
                           set(definitions["arm_right"]))
            if left & right:
                raise ValueError("左右腕の関節が重複しています")
            expected = {
                "arm_left_with_waist": left | {"waist_yaw_joint"},
                "arm_right_with_waist": right | {"waist_yaw_joint"},
                "arm_both": left | right,
                "arm_both_with_waist": left | right | {"waist_yaw_joint"},
            }
            if any(set(definitions[g]) != values
                   for g, values in expected.items()):
                raise ValueError("G1 の腕・腰のグループ構成が不正です")
            for (group, _), joints in states.items():
                if not set(joints) <= set(definitions.get(group, ())):
                    raise ValueError("グループ外の関節が SRDF にあります")
            self._groups, self._srdf_group_states = definitions, states
            return states
        except (OSError, ET.ParseError, ValueError, TypeError,
                PackageNotFoundError) as error:
            if self._context_ok():
                self.node.get_logger().error(f"SRDF 読み込み失敗: {error}")
            return None

    def _joints_for_group(self, group: str) -> tuple[str, ...]:
        group = self._canonical_group(_name(group))
        if self._load_srdf_group_states() is None:
            raise ValueError("SRDF を読み込めません")
        if group not in self._groups:
            raise ValueError(f"未対応のグループです: {group}")
        return self._groups[group]

    def get_supported_joints(
        self, planning_group: str = "arm_both_with_waist",
    ) -> tuple[str, ...]:
        try:
            return self._joints_for_group(planning_group)
        except ValueError:
            return ()

    def list_group_states(
        self, planning_group: str = "arm_both_with_waist",
    ) -> tuple[str, ...]:
        states = self._load_srdf_group_states() or {}
        group = self._canonical_group(planning_group)
        return tuple(sorted(s for g, s in states if g == group))

    def get_group_state(
        self, group_state: str,
        planning_group: str = "arm_both_with_waist",
    ) -> Optional[dict[str, float]]:
        states = self._load_srdf_group_states() or {}
        joints = states.get((
            self._canonical_group(planning_group), group_state))
        return None if joints is None else joints.copy()

    def reload_group_states(self) -> ArmControlResult:
        self._srdf_group_states, self._groups = None, {}
        return self._result(
            ArmControlStatus.SUCCEEDED if self._load_srdf_group_states()
            is not None else ArmControlStatus.INTERNAL_ERROR)

    def _goal_result(
        self, operation: _ArmGoal, status: ArmControlStatus,
        message: str = "", **fields: Any,
    ) -> ArmControlResult:
        return self._result(
            status, message,
            plan_only=operation.goal.planning_options.plan_only,
            used_ik=operation.used_ik,
            used_position_only_fallback=operation.position_only, **fields)

    def _finish_goal(
        self, operation: _ArmGoal, result: ArmControlResult,
    ) -> None:
        with self._goal_lock:
            operation.result = result
            self.last_result = result
            self._goals.pop(operation.future, None)

    def _cancel_operation(self, operation: _ArmGoal) -> None:
        if (operation.handle is None or operation.cancel_future is not None
                or not self._context_ok()):
            return
        try:
            operation.cancel_future = operation.handle.cancel_goal_async()
        except Exception as error:
            # 終了確認ができないゴールは追跡から外さない。
            self.last_result = self._goal_result(
                operation, ArmControlStatus.INTERNAL_ERROR,
                f"取消要求に失敗しました。終了状態は未確認です: {error}")

    def _abort_goal(
        self, operation: _ArmGoal, status: ArmControlStatus,
    ) -> None:
        with self._goal_lock:
            if operation.result is not None:
                return
            if operation.abort_status is None:
                operation.abort_status = status
            self._cancel_operation(operation)
            self.last_result = self._goal_result(
                operation, operation.abort_status, "取消要求中です")

    def request_cancel(self) -> None:
        """待機中と送信済みの全ゴールに非同期で取消を要求する。"""
        self._cancel_requested.set()
        with self._goal_lock:
            for operation in list(self._goals.values()):
                self._abort_goal(operation, ArmControlStatus.CANCELLED)

    def reset_cancel_request(self) -> None:
        """新規要求の取消フラグだけを解除する。送信済み取消は維持する。"""
        self._cancel_requested.clear()

    def _monitor_goals(self) -> None:
        with self._goal_lock:
            for operation in list(self._goals.values()):
                if self._cancel_requested.is_set() or (
                        operation.cancel_event is not None
                        and operation.cancel_event.is_set()):
                    self._abort_goal(operation, ArmControlStatus.CANCELLED)
                deadline = (operation.response_deadline
                            if operation.result_deadline is None
                            else operation.result_deadline)
                if time.monotonic() >= deadline:
                    self._abort_goal(operation, ArmControlStatus.TIMED_OUT)

    def _goal_response_done(self, operation: _ArmGoal, future: Any) -> None:
        """callback 内では待機せず、結果と取消要求を登録する。"""
        try:
            handle = future.result()
            if handle is None or not handle.accepted:
                self._finish_goal(operation, self._goal_result(
                    operation, operation.abort_status
                    or ArmControlStatus.GOAL_REJECTED,
                    "MoveGroup がゴールを拒否しました"))
                return
            with self._goal_lock:
                operation.handle = handle
                operation.result_deadline = (
                    time.monotonic() + operation.result_timeout)
                operation.result_future = handle.get_result_async()
                operation.result_future.add_done_callback(
                    lambda result: self._goal_finished(operation, result))
                if (operation.abort_status is not None
                        or self._cancel_requested.is_set()
                        or (operation.cancel_event is not None
                            and operation.cancel_event.is_set())):
                    self._abort_goal(
                        operation, operation.abort_status
                        or ArmControlStatus.CANCELLED)
        except Exception as error:
            # 受理済みか不明な通信失敗は、新しい動作を開始しない。
            self._abort_goal(operation, ArmControlStatus.INTERNAL_ERROR)
            self.last_result = self._goal_result(
                operation, ArmControlStatus.INTERNAL_ERROR, str(error))

    @staticmethod
    def _moveit_error_name(code: int) -> str:
        return next((name for name in dir(MoveItErrorCodes)
                     if name.isupper()
                     and getattr(MoveItErrorCodes, name) == code),
                    f"UNKNOWN_{code}")

    def _goal_finished(self, operation: _ArmGoal, future: Any) -> None:
        try:
            wrapped = future.result()
            code = wrapped.result.error_code.val
            action_status = wrapped.status
            if action_status == GoalStatus.STATUS_CANCELED:
                status = ArmControlStatus.CANCELLED
            elif (action_status == GoalStatus.STATUS_SUCCEEDED
                  and code == MoveItErrorCodes.SUCCESS):
                status = (ArmControlStatus.DEGRADED if operation.position_only
                          else ArmControlStatus.SUCCEEDED)
            else:
                status = ArmControlStatus.MOVEIT_FAILED
            status = operation.abort_status or status
            result = self._goal_result(
                operation, status, self._moveit_error_name(code),
                moveit_error_code=code,
                moveit_error_name=self._moveit_error_name(code),
                action_status=action_status)
            self._finish_goal(operation, result)
        except Exception as error:
            # 終了を確認できない場合も handle を保持して取消を可能にする。
            self._abort_goal(operation, ArmControlStatus.INTERNAL_ERROR)
            self.last_result = self._goal_result(
                operation, ArmControlStatus.INTERNAL_ERROR, str(error))

    def cancel_current_goal(
        self, timeout: Optional[float] = None,
    ) -> ArmControlResult:
        try:
            deadline = time.monotonic() + _positive(
                "timeout", self.cancel_timeout if timeout is None else timeout)
        except ValueError as error:
            return self._result(ArmControlStatus.INVALID_ARGUMENT, str(error))
        with self._goal_lock:
            operations = list(self._goals.values())
        self.request_cancel()
        if not operations:
            return self._result(
                ArmControlStatus.INVALID_ARGUMENT, "実行中ゴールがありません")
        while self._context_ok():
            if all(op.result is not None for op in operations):
                if all(op.result.action_status == GoalStatus.STATUS_CANCELED
                       for op in operations):
                    return self._result(
                        ArmControlStatus.CANCELLED, "取消完了を確認しました")
                return self._result(
                    ArmControlStatus.GOAL_REJECTED,
                    "取消より先にゴールが終了したか、取消が拒否されました")
            if time.monotonic() >= deadline:
                return self._result(
                    ArmControlStatus.TIMED_OUT,
                    "取消後の終了は未確認です。ゴールの追跡を継続します")
            self._spin_once(deadline - time.monotonic())
        return self._result(ArmControlStatus.ROS_SHUTDOWN)

    def cancel(self, wait: bool = True, timeout: float = 5.0) -> bool:
        if not wait:
            self.request_cancel()
            return True
        return self.cancel_current_goal(timeout).status == (
            ArmControlStatus.CANCELLED)

    def _send_move_group_goal_detailed(
        self, goal: MoveGroup.Goal, *, wait: bool = True,
        execute: bool = True, goal_response_timeout: Optional[float] = None,
        result_timeout: Optional[float] = None,
        cancel_timeout: Optional[float] = None,
        cancel_event: Optional[threading.Event] = None,
        used_ik: bool = False, position_only: bool = False,
    ) -> ArmControlResult:
        try:
            response_limit = _positive(
                "goal_response_timeout", self.goal_response_timeout
                if goal_response_timeout is None else goal_response_timeout)
            result_limit = _positive(
                "result_timeout", self.result_timeout
                if result_timeout is None else result_timeout)
            cancel_limit = _positive(
                "cancel_timeout", self.cancel_timeout
                if cancel_timeout is None else cancel_timeout)
            if not isinstance(wait, bool) or not isinstance(execute, bool):
                raise ValueError("wait と execute は bool が必要です")
            if cancel_event is not None and not isinstance(
                    cancel_event, threading.Event):
                raise ValueError("cancel_event は threading.Event が必要です")
            if not self._context_ok():
                return self._result(ArmControlStatus.ROS_SHUTDOWN)
            if self._cancel_requested.is_set() or (
                    cancel_event is not None and cancel_event.is_set()):
                return self._result(ArmControlStatus.CANCELLED)
            if not self.is_move_group_ready():
                return self._result(ArmControlStatus.NOT_READY)
            goal.planning_options.plan_only = not execute
            with self._goal_lock:
                if self._goals:
                    return self._result(
                        ArmControlStatus.NOT_READY,
                        "前のゴールが終了していません")
                future = self._move_group_client.send_goal_async(goal)
                operation = _ArmGoal(
                    future, goal, time.monotonic() + response_limit,
                    result_limit, cancel_event, used_ik, position_only)
                self._goals[future] = operation
                future.add_done_callback(
                    lambda done: self._goal_response_done(operation, done))
            if not wait:
                return operation.result or self._goal_result(
                    operation, ArmControlStatus.SUBMITTED, "ゴールを送信しました")
            while self._context_ok():
                self._monitor_goals()
                with self._goal_lock:
                    if operation.result is not None:
                        return operation.result
                    abort_status = operation.abort_status
                if abort_status is not None:
                    # 取消応答待ちも有限。未終了ゴールは後続 callback が監視する。
                    deadline = time.monotonic() + cancel_limit
                    while (self._context_ok() and operation.result is None
                           and time.monotonic() < deadline):
                        self._spin_once(deadline - time.monotonic())
                    return operation.result or self._goal_result(
                        operation, abort_status,
                        "要求を中断しました。取消後の終了状態は未確認です")
                self._spin_once(self._SPIN_INTERVAL)
            return self._goal_result(
                operation, ArmControlStatus.ROS_SHUTDOWN)
        except KeyboardInterrupt:
            self.request_cancel()
            raise
        except (ValueError, TypeError) as error:
            return self._result(ArmControlStatus.INVALID_ARGUMENT, str(error))
        except (ExternalShutdownException, RCLError, InvalidHandle):
            return self._result(ArmControlStatus.ROS_SHUTDOWN)
        except Exception as error:
            if "operation" in locals():
                self._abort_goal(operation, ArmControlStatus.INTERNAL_ERROR)
            return self._result(ArmControlStatus.INTERNAL_ERROR, str(error))

    def _send_move_group_goal(
        self,
        goal_msg: MoveGroup.Goal,
        wait: bool,
    ) -> bool:
        return self._to_legacy_bool(self._send_move_group_goal_detailed(
            goal_msg, wait=wait,
            execute=not goal_msg.planning_options.plan_only))

    def _tip(self, group: str, tip_link: Optional[str] = None) -> str:
        group = self._canonical_group(_name(group))
        self._joints_for_group(group)
        if group == "head":
            raise ValueError("頭部には関節制御を使用してください")
        left, right = "left_amazing_hand", "right_amazing_hand"
        allowed = ((left,) if "left" in group else (right,)
                   if "right" in group else (left, right))
        link = tip_link or (left if "left" in group else right)
        if link not in allowed:
            raise ValueError(f"{group} の手先リンクではありません: {link}")
        return link

    def _current_pose(
        self, group: str, reference_frame: str,
        tip_link: Optional[str] = None, timeout: float = 2.0,
    ) -> Optional[PoseStamped]:
        link, frame = self._tip(group, tip_link), _name(reference_frame)
        deadline = time.monotonic() + _positive("pose_timeout", timeout)
        while self._context_ok() and not self._cancel_requested.is_set():
            try:
                tf = self.tf_buffer.lookup_transform(
                    frame, link, rclpy.time.Time())
                pose = PoseStamped()
                pose.header = tf.header
                pose.header.frame_id = frame
                p = tf.transform.translation
                pose.pose.position.x, pose.pose.position.y = p.x, p.y
                pose.pose.position.z = p.z
                pose.pose.orientation = tf.transform.rotation
                pose.pose = _pose_valid(pose.pose)
                return pose
            except TransformException:
                if time.monotonic() >= deadline:
                    break
                self._spin_once(deadline - time.monotonic())
        return None

    def get_current_pose(
        self, simple: bool = False,
        planning_group: str = "arm_both_with_waist",
        reference_frame: str = "base_link",
    ) -> Optional[Union[PoseStamped, list[float]]]:
        try:
            pose = self._current_pose(planning_group, reference_frame)
            if pose is None or not simple:
                return pose
            p, q = pose.pose.position, pose.pose.orientation
            return [p.x, p.y, p.z,
                    *euler_from_quaternion([q.x, q.y, q.z, q.w])]
        except (ValueError, TypeError, ExternalShutdownException,
                RCLError, InvalidHandle):
            return None

    def _valid_pose(
        self, pose: Union[Pose, PoseStamped],
    ) -> PoseStamped:
        if isinstance(pose, PoseStamped):
            target = copy.deepcopy(pose)
            _name(target.header.frame_id)
        elif isinstance(pose, Pose):
            target = PoseStamped()
            target.header.frame_id = self.base_frame
            target.pose = copy.deepcopy(pose)
        else:
            raise ValueError("Pose または PoseStamped が必要です")
        target.pose = _pose_valid(target.pose)
        return target

    def _absolute_pose(
        self, values: tuple, reference_frame: str,
    ) -> PoseStamped:
        if not all(_finite(v) for v in values):
            raise ValueError("目標位置・姿勢は有限数で指定してください")
        pose = PoseStamped()
        pose.header.frame_id = _name(reference_frame)
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = (
            float(v) for v in values[:3])
        pose.pose.orientation = _quaternion_euler(*values[3:])
        return pose

    def _relative_pose(
        self, current: PoseStamped, values: tuple,
    ) -> PoseStamped:
        if not all(_finite(v) for v in values):
            raise ValueError("相対位置・姿勢は有限数で指定してください")
        pose = copy.deepcopy(current)
        pose.pose.position.x += float(values[0])
        pose.pose.position.y += float(values[1])
        pose.pose.position.z += float(values[2])
        pose.pose.orientation = _quaternion_product(
            current.pose.orientation, _quaternion_euler(*values[3:]))
        return self._valid_pose(pose)

    def _options(self, kwargs: dict) -> dict:
        defaults = {
            "planning_attempts": 10, "planning_time": 5.0,
            "execute": True,
            "goal_response_timeout": self.goal_response_timeout,
            "result_timeout": self.result_timeout,
            "cancel_timeout": self.cancel_timeout, "cancel_event": None,
            "velocity_scale": self.velocity_scale,
            "acceleration_scale": self.acceleration_scale,
        }
        unknown = set(kwargs) - set(defaults)
        if unknown:
            raise ValueError(f"不明な引数です: {sorted(unknown)}")
        defaults.update(kwargs)
        attempts = defaults["planning_attempts"]
        if not isinstance(attempts, int) or isinstance(attempts, bool) or (
                attempts < 1):
            raise ValueError("planning_attempts は正の整数が必要です")
        for name in ("planning_time", "goal_response_timeout",
                     "result_timeout", "cancel_timeout"):
            defaults[name] = _positive(name, defaults[name])
        for name in ("velocity_scale", "acceleration_scale"):
            defaults[name] = self._scale(name, defaults[name])
        if not isinstance(defaults["execute"], bool):
            raise ValueError("execute は bool が必要です")
        event = defaults["cancel_event"]
        if event is not None and not isinstance(event, threading.Event):
            raise ValueError("cancel_event は threading.Event が必要です")
        return defaults

    def _new_goal(self, group: str, options: dict) -> MoveGroup.Goal:
        group = self._canonical_group(_name(group))
        self._joints_for_group(group)
        goal = MoveGroup.Goal()
        goal.request.group_name = group
        goal.request.start_state.is_diff = True
        goal.request.num_planning_attempts = options["planning_attempts"]
        goal.request.allowed_planning_time = options["planning_time"]
        goal.request.max_velocity_scaling_factor = options["velocity_scale"]
        goal.request.max_acceleration_scaling_factor = options[
            "acceleration_scale"]
        goal.planning_options.plan_only = not options["execute"]
        return goal

    def _submit(
        self, goal: MoveGroup.Goal, wait: bool, options: dict, *,
        used_ik: bool = False, position_only: bool = False,
    ) -> ArmControlResult:
        return self._send_move_group_goal_detailed(
            goal, wait=wait, execute=options["execute"],
            goal_response_timeout=options["goal_response_timeout"],
            result_timeout=options["result_timeout"],
            cancel_timeout=options["cancel_timeout"],
            cancel_event=options["cancel_event"],
            used_ik=used_ik, position_only=position_only)

    def _create_pose_constraints(
        self, target_pose: PoseStamped, tip_link: str,
    ) -> tuple[PositionConstraint, OrientationConstraint]:
        pose = self._valid_pose(target_pose)
        pc = PositionConstraint()
        pc.header, pc.link_name, pc.weight = pose.header, tip_link, 1.0
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [
            _positive("position_tolerance", self.position_tolerance)]
        pc.constraint_region.primitives = [primitive]
        pc.constraint_region.primitive_poses = [pose.pose]
        oc = OrientationConstraint()
        oc.header, oc.link_name, oc.weight = pose.header, tip_link, 1.0
        oc.orientation = pose.pose.orientation
        tolerance = _positive(
            "orientation_tolerance", self.orientation_tolerance)
        oc.absolute_x_axis_tolerance = tolerance
        oc.absolute_y_axis_tolerance = tolerance
        oc.absolute_z_axis_tolerance = tolerance
        return pc, oc

    def _pose_goal(
        self, targets: list[tuple[PoseStamped, str]], group: str,
        options: dict, include_orientation: bool = True,
    ) -> MoveGroup.Goal:
        goal, constraints = self._new_goal(group, options), Constraints()
        for pose, tip in targets:
            pc, oc = self._create_pose_constraints(pose, tip)
            constraints.position_constraints.append(pc)
            if include_orientation:
                constraints.orientation_constraints.append(oc)
        # 両手の条件を一つの Constraints にまとめ、AND 条件で解く。
        goal.request.goal_constraints = [constraints]
        return goal

    def _joint_goal(
        self, targets: dict[str, float], group: str, options: dict,
    ) -> MoveGroup.Goal:
        goal, constraints = self._new_goal(group, options), Constraints()
        for name, target in targets.items():
            jc = JointConstraint()
            jc.joint_name, jc.position, jc.weight = name, float(target), 1.0
            jc.tolerance_above = _positive(
                "joint_tolerance", self.joint_tolerance)
            jc.tolerance_below = jc.tolerance_above
            constraints.joint_constraints.append(jc)
        goal.request.goal_constraints = [constraints]
        return goal

    def _solve_ik_detailed(
        self, pose_stamped: PoseStamped, group_name: str,
        tip_link: Optional[str] = None, *,
        timeout: Optional[float] = None,
        cancel_event: Optional[threading.Event] = None,
    ) -> IkResult:
        try:
            group = self._canonical_group(_name(group_name))
            if group not in self._SINGLE_GROUPS:
                return IkResult(
                    IkStatus.INVALID_REQUEST, message="単腕グループが必要です")
            target = self._valid_pose(pose_stamped)
            tip = self._tip(group, tip_link)
            joints = self._joints_for_group(group)
            deadline = time.monotonic() + _positive(
                "timeout", self.goal_response_timeout
                if timeout is None else timeout)
            if not self._context_ok():
                return IkResult(IkStatus.ROS_SHUTDOWN)
            if self._cancel_requested.is_set() or (
                    cancel_event is not None and cancel_event.is_set()):
                return IkResult(IkStatus.CANCELLED)
            current, missing = self._fresh_joint_snapshot(joints)
            if current is None:
                return IkResult(
                    IkStatus.STATE_UNAVAILABLE,
                    message=f"関節状態が未受信または古いです: {missing}")
            if not self.is_ik_ready():
                return IkResult(IkStatus.SERVICE_UNAVAILABLE)
            # rotation_scale=0 の解を姿勢まで一致した成功と誤認しない。
            if not self._safe_service_ready(0.0, self._fk_client):
                return IkResult(
                    IkStatus.NO_SOLUTION, message="FK で姿勢を確認できません")
            req = GetPositionIK.Request()
            req.ik_request.group_name, req.ik_request.ik_link_name = group, tip
            req.ik_request.pose_stamped = target
            req.ik_request.timeout.sec = 1
            req.ik_request.avoid_collisions = True
            req.ik_request.robot_state.is_diff = True
            req.ik_request.robot_state.joint_state.name = list(current)
            req.ik_request.robot_state.joint_state.position = list(
                current.values())
            status, response = self._call_service(
                self._ik_client, req, deadline - time.monotonic(),
                cancel_event=cancel_event)
            if status != _FutureWaitStatus.COMPLETED:
                return IkResult(IkStatus(status.value))
            if response is None:
                return IkResult(IkStatus.INVALID_RESPONSE)
            code = response.error_code.val
            if code != MoveItErrorCodes.SUCCESS:
                return IkResult(IkStatus.NO_SOLUTION, moveit_error_code=code)
            state = response.solution.joint_state
            if (len(state.name) != len(state.position)
                    or len(set(state.name)) != len(state.name)
                    or not all(_finite(v) for v in state.position)):
                return IkResult(IkStatus.INVALID_RESPONSE)
            solution = dict(zip(state.name, state.position))
            if not set(joints) <= solution.keys():
                return IkResult(IkStatus.INVALID_RESPONSE)
            fk = GetPositionFK.Request()
            fk.header = target.header
            fk.fk_link_names = [tip]
            fk.robot_state = response.solution
            fk.robot_state.is_diff = True
            status, checked = self._call_service(
                self._fk_client, fk, deadline - time.monotonic(),
                cancel_event=cancel_event)
            if status != _FutureWaitStatus.COMPLETED:
                return IkResult(IkStatus(status.value))
            if (checked is None
                    or checked.error_code.val != MoveItErrorCodes.SUCCESS
                    or tip not in checked.fk_link_names):
                return IkResult(IkStatus.INVALID_RESPONSE)
            achieved = checked.pose_stamped[checked.fk_link_names.index(tip)]
            if achieved.header.frame_id != target.header.frame_id:
                return IkResult(IkStatus.INVALID_RESPONSE)
            a, b = _pose_valid(achieved.pose), target.pose
            position_error = math.dist(
                (a.position.x, a.position.y, a.position.z),
                (b.position.x, b.position.y, b.position.z))
            # OrientationConstraint と同じ軸ごとの誤差を評価する。
            inverse = Quaternion(
                x=-b.orientation.x, y=-b.orientation.y,
                z=-b.orientation.z, w=b.orientation.w)
            error = _quaternion_product(inverse, a.orientation)
            angles = euler_from_quaternion(
                [error.x, error.y, error.z, error.w])
            if (position_error > self.position_tolerance
                    or any(abs(v) > self.orientation_tolerance
                           for v in angles)):
                return IkResult(
                    IkStatus.NO_SOLUTION,
                    message="IK 解の位置または姿勢が許容値を超えています")
            return IkResult(
                IkStatus.SUCCEEDED,
                joints={name: float(solution[name]) for name in joints},
                moveit_error_code=code)
        except (TypeError, ValueError) as error:
            return IkResult(IkStatus.INVALID_REQUEST, message=str(error))
        except (ExternalShutdownException, RCLError, InvalidHandle):
            return IkResult(IkStatus.ROS_SHUTDOWN)
        except Exception as error:
            return IkResult(IkStatus.INTERNAL_ERROR, message=str(error))

    def _solve_ik(
        self, pose_stamped: PoseStamped, group_name: str,
        tip_link: str = None,
    ) -> Optional[dict]:
        result = self._solve_ik_detailed(pose_stamped, group_name, tip_link)
        return result.joints if result.status == IkStatus.SUCCEEDED else None

    def _fallback_allowed(self, result: ArmControlResult) -> bool:
        return (result.status == ArmControlStatus.MOVEIT_FAILED
                and result.moveit_error_code in self._PLAN_FAILURES
                and not self._cancel_requested.is_set())

    @_arm_result_guard
    def move_to_pose_detailed(
        self, pose: Union[Pose, PoseStamped],
        planning_group: str = "arm_both_with_waist", wait: bool = True,
        tip_link: str = None, *, allow_position_only_fallback: bool = False,
        use_ik: bool = True, **kwargs: Any,
    ) -> ArmControlResult:
        if not isinstance(wait, bool) or not isinstance(
                allow_position_only_fallback, bool):
            raise ValueError("wait と fallback は bool が必要です")
        if not wait and allow_position_only_fallback:
            raise ValueError(
                "位置のみのフォールバックには wait=True が必要です")
        if not isinstance(use_ik, bool):
            raise ValueError("use_ik は bool が必要です")
        group = self._canonical_group(_name(planning_group))
        target, tip = self._valid_pose(pose), self._tip(group, tip_link)
        options = self._options(kwargs)
        if use_ik and group in self._SINGLE_GROUPS:
            ik = self._solve_ik_detailed(
                target, group, tip, timeout=options["goal_response_timeout"],
                cancel_event=options["cancel_event"])
            if ik.status == IkStatus.SUCCEEDED:
                result = self._submit(
                    self._joint_goal(ik.joints, group, options),
                    wait, options, used_ik=True)
                if not self._fallback_allowed(result):
                    return result
            elif ik.status not in (
                    IkStatus.NO_SOLUTION, IkStatus.SERVICE_UNAVAILABLE):
                status = {
                    IkStatus.TIMED_OUT: ArmControlStatus.TIMED_OUT,
                    IkStatus.CANCELLED: ArmControlStatus.CANCELLED,
                    IkStatus.ROS_SHUTDOWN: ArmControlStatus.ROS_SHUTDOWN,
                    IkStatus.STATE_UNAVAILABLE:
                        ArmControlStatus.STATE_UNAVAILABLE,
                    IkStatus.INVALID_REQUEST:
                        ArmControlStatus.INVALID_ARGUMENT,
                }.get(ik.status, ArmControlStatus.INTERNAL_ERROR)
                return self._result(
                    status, ik.message, moveit_error_code=ik.moveit_error_code)
        targets = [(target, tip)]
        result = self._submit(
            self._pose_goal(targets, group, options), wait, options)
        if allow_position_only_fallback and self._fallback_allowed(result):
            return self._submit(
                self._pose_goal(targets, group, options, False),
                wait, options, position_only=True)
        return result

    @_arm_result_guard
    def move_abs_detailed(
        self, x: float = 0.0, y: float = 0.0, z: float = 0.0,
        roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0,
        planning_group: str = "arm_both_with_waist", wait: bool = True,
        reference_frame: str = "base_link", tip_link: str = None,
        **kwargs: Any,
    ) -> ArmControlResult:
        pose = self._absolute_pose(
            (x, y, z, roll, pitch, yaw), reference_frame)
        return self.move_to_pose_detailed(
            pose, planning_group, wait, tip_link, **kwargs)

    @_arm_result_guard
    def move_rel_detailed(
        self, x: float = 0.0, y: float = 0.0, z: float = 0.0,
        roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0,
        planning_group: str = "arm_both_with_waist", wait: bool = True,
        **kwargs: Any,
    ) -> ArmControlResult:
        values = (x, y, z, roll, pitch, yaw)
        if not all(_finite(v) for v in values):
            raise ValueError("相対位置・姿勢は有限数で指定してください")
        frame = kwargs.pop("reference_frame", self.base_frame)
        tip = kwargs.pop("tip_link", None)
        timeout = kwargs.pop("pose_timeout", 2.0)
        current = self._current_pose(planning_group, frame, tip, timeout)
        if current is None:
            status = (ArmControlStatus.ROS_SHUTDOWN if not self._context_ok()
                      else ArmControlStatus.CANCELLED
                      if self._cancel_requested.is_set()
                      else ArmControlStatus.TF_UNAVAILABLE)
            return self._result(status)
        return self.move_to_pose_detailed(
            self._relative_pose(current, values), planning_group, wait,
            tip, **kwargs)

    @_arm_result_guard
    def move_dual_abs_detailed(
        self, lx: float = 0.0, ly: float = 0.0, lz: float = 0.0,
        lr: float = 0.0, lp: float = 0.0, lyaw: float = 0.0,
        rx: float = 0.0, ry: float = 0.0, rz: float = 0.0,
        rr: float = 0.0, rp: float = 0.0, ryaw: float = 0.0,
        wait: bool = True, reference_frame: str = "base_link",
        **kwargs: Any,
    ) -> ArmControlResult:
        group = self._canonical_group(kwargs.pop(
            "planning_group", "arm_both_with_waist"))
        if group not in ("arm_both", "arm_both_with_waist"):
            raise ValueError("双腕グループを指定してください")
        fallback = kwargs.pop("allow_position_only_fallback", False)
        if not isinstance(fallback, bool) or (fallback and not wait):
            raise ValueError("位置のみのフォールバックには同期指定が必要です")
        options = self._options(kwargs)
        targets = [
            (self._absolute_pose((lx, ly, lz, lr, lp, lyaw), reference_frame),
             "left_amazing_hand"),
            (self._absolute_pose((rx, ry, rz, rr, rp, ryaw), reference_frame),
             "right_amazing_hand")]
        result = self._submit(
            self._pose_goal(targets, group, options), wait, options)
        if fallback and self._fallback_allowed(result):
            return self._submit(
                self._pose_goal(targets, group, options, False),
                wait, options, position_only=True)
        return result

    @_arm_result_guard
    def move_dual_rel_detailed(
        self, lx: float = 0.0, ly: float = 0.0, lz: float = 0.0,
        lr: float = 0.0, lp: float = 0.0, lyaw: float = 0.0,
        rx: float = 0.0, ry: float = 0.0, rz: float = 0.0,
        rr: float = 0.0, rp: float = 0.0, ryaw: float = 0.0,
        wait: bool = True, **kwargs: Any,
    ) -> ArmControlResult:
        values = ((lx, ly, lz, lr, lp, lyaw), (rx, ry, rz, rr, rp, ryaw))
        if not all(_finite(v) for pose in values for v in pose):
            raise ValueError("相対位置・姿勢は有限数で指定してください")
        frame = kwargs.pop("reference_frame", self.base_frame)
        deadline = time.monotonic() + _positive(
            "pose_timeout", kwargs.pop("pose_timeout", 2.0))
        poses = []
        for group, delta in zip(("arm_left", "arm_right"), values):
            current = self._current_pose(
                group, frame, timeout=deadline - time.monotonic())
            if current is None:
                status = (ArmControlStatus.CANCELLED
                          if self._cancel_requested.is_set()
                          else ArmControlStatus.ROS_SHUTDOWN
                          if not self._context_ok()
                          else ArmControlStatus.TF_UNAVAILABLE)
                return self._result(status)
            target = self._relative_pose(current, delta)
            p, q = target.pose.position, target.pose.orientation
            poses.extend((p.x, p.y, p.z, *euler_from_quaternion(
                [q.x, q.y, q.z, q.w])))
        return self.move_dual_abs_detailed(
            *poses, wait=wait, reference_frame=frame, **kwargs)

    @_arm_result_guard
    def joint_control_detailed(
        self, rlt: bool = False, wait: bool = True,
        planning_group: str = "arm_both_with_waist", **kwargs: Any,
    ) -> ArmControlResult:
        if not isinstance(rlt, bool):
            raise ValueError("rlt は bool が必要です")
        group = self._canonical_group(_name(planning_group))
        joints = self._joints_for_group(group)
        targets = {name: kwargs.pop(name) for name in joints if name in kwargs}
        options = self._options(kwargs)
        if not all(_finite(v) for v in targets.values()):
            raise ValueError("関節値は有限数で指定してください")
        current = {}
        if rlt or set(targets) != set(joints):
            current, missing = self._fresh_joint_snapshot(joints)
            if current is None:
                return self._result(
                    ArmControlStatus.STATE_UNAVAILABLE,
                    f"関節状態が未受信または古いです: {missing}")
        resolved = {
            name: (float(targets[name]) + (current[name] if rlt else 0.0)
                   if name in targets else current[name])
            for name in joints}
        if not all(_finite(v) for v in resolved.values()):
            raise ValueError("計算後の関節値が非有限です")
        return self._submit(self._joint_goal(
            resolved, group, options), wait, options)

    @_arm_result_guard
    def move_groupstate_detailed(
        self, group_name: str = "arm_both_with_waist",
        group_state: str = "home", wait: bool = True, **kwargs: Any,
    ) -> ArmControlResult:
        group_name = self._canonical_group(_name(group_name))
        group_state = self._canonical_group(_name(group_state))
        states = self._load_srdf_group_states()
        if states is None:
            return self._result(
                ArmControlStatus.NOT_READY, "SRDF を読み込めません")
        groups, names = {g for g, _ in states}, {s for _, s in states}
        if group_name in names and group_state in groups:
            group_name, group_state = group_state, group_name
        if (group_state == "home" and group_name not in groups
                and group_name in names):
            group_name, group_state = "arm_both_with_waist", group_name
        joints = states.get((group_name, group_state))
        if joints is None:
            raise ValueError("指定された SRDF 姿勢がありません")
        return self.joint_control_detailed(
            planning_group=group_name, wait=wait, **kwargs, **joints)

    @_arm_result_guard
    def place_detailed(
        self, x: float, y: float, z: float,
        planning_group: str = "arm_right", wait: bool = True,
        **kwargs: Any,
    ) -> ArmControlResult:
        options = self._options(kwargs)
        pose = self._absolute_pose((x, y, z, 0.0, 0.0, 0.0), self.base_frame)
        goal = self._pose_goal(
            [(pose, self._tip(planning_group))], planning_group,
            options, False)
        region = goal.request.goal_constraints[0].position_constraints[0]
        region.constraint_region.primitives[0].type = SolidPrimitive.BOX
        region.constraint_region.primitives[0].dimensions = [0.05] * 3
        return self._submit(goal, wait, options)

    @_arm_result_guard
    def enable_upper_body_control_detailed(
        self, enable: bool = True, *, timeout: float = 5.0,
    ) -> ArmControlResult:
        if not isinstance(enable, bool):
            raise ValueError("enable は bool が必要です")
        request = SetBool.Request()
        request.data = enable
        return self._external_service(self._ubc_client, request, timeout)

    def enable_upper_body_control(self, enable: bool = True) -> bool:
        return self._to_legacy_bool(
            self.enable_upper_body_control_detailed(enable))

    @_arm_result_guard
    def hand_control_detailed(
        self, command: str = "walk", hand: str = "both", *,
        timeout: float = 5.0,
    ) -> ArmControlResult:
        request = HandCommand.Request()
        request.command, request.hand = _name(command), _name(hand)
        if hand not in ("left", "right", "both"):
            raise ValueError("hand は left、right、both のいずれかです")
        return self._external_service(self._hand_client, request, timeout)

    def hand_control(self, command: str = "walk", hand: str = "both") -> bool:
        return self._to_legacy_bool(
            self.hand_control_detailed(command, hand))

    def _external_service(
        self, client: Any, request: Any, timeout: float,
    ) -> ArmControlResult:
        status, response = self._call_service(client, request, timeout)
        if status != _FutureWaitStatus.COMPLETED:
            return self._result(
                ArmControlStatus(status.value),
                "サービス要求の実行結果は未確認です")
        if response is None:
            return self._result(ArmControlStatus.INTERNAL_ERROR)
        return self._result(
            ArmControlStatus.SUCCEEDED if response.success
            else ArmControlStatus.GOAL_REJECTED,
            getattr(response, "message", ""))

    def move_to_pose(
        self,
        pose: Union[Pose, PoseStamped],
        planning_group: str = 'arm_both_with_waist',
        wait: bool = True,
        tip_link: str = None,
        **kwargs: Any,
    ) -> bool:
        """既存の引数と bool 戻り値を維持する。"""
        kwargs.setdefault("allow_position_only_fallback", bool(wait))
        return self._to_legacy_bool(self.move_to_pose_detailed(
            pose=pose,
            planning_group=planning_group,
            wait=wait,
            tip_link=tip_link,
            **kwargs))

    def place(
        self,
        x: float,
        y: float,
        z: float,
        planning_group: str = 'arm_right',
        wait: bool = True,
    ) -> bool:
        """既存の引数と bool 戻り値を維持する。"""
        return self._to_legacy_bool(self.place_detailed(
            x=x,
            y=y,
            z=z,
            planning_group=planning_group,
            wait=wait))

    def move_abs(
        self,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
        planning_group: str = 'arm_both_with_waist',
        wait: bool = True,
        reference_frame: str = 'base_link',
        tip_link: str = None,
        **kwargs: Any,
    ) -> bool:
        """既存の引数と bool 戻り値を維持する。"""
        kwargs.setdefault("allow_position_only_fallback", bool(wait))
        return self._to_legacy_bool(self.move_abs_detailed(
            x=x,
            y=y,
            z=z,
            roll=roll,
            pitch=pitch,
            yaw=yaw,
            planning_group=planning_group,
            wait=wait,
            reference_frame=reference_frame,
            tip_link=tip_link,
            **kwargs))

    def move_rel(
        self,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
        planning_group: str = 'arm_both_with_waist',
        wait: bool = True,
        **kwargs: Any,
    ) -> bool:
        """既存の引数と bool 戻り値を維持する。"""
        kwargs.setdefault("allow_position_only_fallback", bool(wait))
        return self._to_legacy_bool(self.move_rel_detailed(
            x=x,
            y=y,
            z=z,
            roll=roll,
            pitch=pitch,
            yaw=yaw,
            planning_group=planning_group,
            wait=wait,
            **kwargs))

    def move_dual_abs(
        self,
        lx: float = 0.0,
        ly: float = 0.0,
        lz: float = 0.0,
        lr: float = 0.0,
        lp: float = 0.0,
        lyaw: float = 0.0,
        rx: float = 0.0,
        ry: float = 0.0,
        rz: float = 0.0,
        rr: float = 0.0,
        rp: float = 0.0,
        ryaw: float = 0.0,
        wait: bool = True,
        reference_frame: str = 'base_link',
        **kwargs: Any,
    ) -> bool:
        """既存の引数と bool 戻り値を維持する。"""
        return self._to_legacy_bool(self.move_dual_abs_detailed(
            lx=lx,
            ly=ly,
            lz=lz,
            lr=lr,
            lp=lp,
            lyaw=lyaw,
            rx=rx,
            ry=ry,
            rz=rz,
            rr=rr,
            rp=rp,
            ryaw=ryaw,
            wait=wait,
            reference_frame=reference_frame,
            **kwargs))

    def move_dual_rel(
        self,
        lx: float = 0.0,
        ly: float = 0.0,
        lz: float = 0.0,
        lr: float = 0.0,
        lp: float = 0.0,
        lyaw: float = 0.0,
        rx: float = 0.0,
        ry: float = 0.0,
        rz: float = 0.0,
        rr: float = 0.0,
        rp: float = 0.0,
        ryaw: float = 0.0,
        wait: bool = True,
        **kwargs: Any,
    ) -> bool:
        """既存の引数と bool 戻り値を維持する。"""
        return self._to_legacy_bool(self.move_dual_rel_detailed(
            lx=lx,
            ly=ly,
            lz=lz,
            lr=lr,
            lp=lp,
            lyaw=lyaw,
            rx=rx,
            ry=ry,
            rz=rz,
            rr=rr,
            rp=rp,
            ryaw=ryaw,
            wait=wait,
            **kwargs))

    def move_groupstate(
        self,
        group_name: str = 'arm_both_with_waist',
        group_state: str = 'home',
        wait: bool = True,
    ) -> bool:
        """既存の引数と bool 戻り値を維持する。"""
        return self._to_legacy_bool(self.move_groupstate_detailed(
            group_name=group_name,
            group_state=group_state,
            wait=wait))

    def joint_control(
        self,
        rlt: bool = False,
        wait: bool = True,
        planning_group: str = 'arm_both_with_waist',
        **kwargs: Any,
    ) -> bool:
        """既存の引数と bool 戻り値を維持する。"""
        return self._to_legacy_bool(self.joint_control_detailed(
            rlt=rlt,
            wait=wait,
            planning_group=planning_group,
            **kwargs))


class G1Mic:
    """
    Unitree G1 robot microphone audio receiver class.
    Communicates with mic_server node via ROS 2 services and topics.
    """

    def __init__(self, node: Node, sample_rate: int = 16000, channels: int = 1):
        """
        G1Mic クラスのコンストラクタ

        Parameters
        ----------
        node : Node
            ROS2 ノードオブジェクト
        sample_rate : int, optional
            サンプリングレート。
        channels : int, optional
            チャンネル数。
        """
        self.node = node
        self.__sample_rate = sample_rate
        self.__channels = channels

        self.__audio_buffer = []
        self.__buffer_lock = threading.Lock()

        # Service client for control
        self.__mic_rec_cli = self.node.create_client(SetBool, "mic_rec")

        # Subscriber for audio data
        self.__audio_sub = self.node.create_subscription(
            Int16MultiArray, "/audio/raw", self.__audio_callback, 10
        )

    def __audio_callback(self, msg: Int16MultiArray):
        """
        音声データを受信した際のコールバック関数。
        """
        with self.__buffer_lock:
            # Convert Int16MultiArray data to numpy array
            self.__audio_buffer.append(np.array(msg.data, dtype=np.int16))

    def __enter__(self) -> "G1Mic":
        """
        コンテキストマネージャの開始。音声配信を有効化します。
        """
        if not self.__mic_rec_cli.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error("mic_server (mic_rec service) is not running.")
            raise RuntimeError("mic_server is not running.")

        # 録音開始のリクエスト
        req = SetBool.Request()
        req.data = True

        future = self.__mic_rec_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)

        if future.done():
            res = future.result()
            if res.success:
                self.node.get_logger().info(
                    "Microphone recording enabled via mic_server."
                )
            else:
                self.node.get_logger().error(
                    f"Failed to enable recording: {res.message}"
                )
        else:
            self.node.get_logger().error("Service call timed out.")

        with self.__buffer_lock:
            self.__audio_buffer = []  # Clear buffer on start

        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """
        コンテキストマネージャの終了。音声配信を無効化します。
        """
        req = SetBool.Request()
        req.data = False

        future = self.__mic_rec_cli.call_async(req)
        # We don't necessarily need to wait long here, but it's good practice
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=1.0)

        self.node.get_logger().info("Microphone recording disabled.")

    def read(self) -> np.ndarray:
        """
        前回の呼び出しから現在までに蓄積された全ての音声データを取得します。

        Returns
        -------
        np.ndarray
            蓄積された音声データ（int16）を結合したもの。データがない場合は空の配列。
        """
        with self.__buffer_lock:
            if not self.__audio_buffer:
                return np.array([], dtype=np.int16)

            # Concatenate all chunks in buffer
            full_data = np.concatenate(self.__audio_buffer)
            self.__audio_buffer = []  # Clear buffer after reading
            return full_data

    def save_wav(self, file_path: str, audio_data: np.ndarray) -> bool:
        """
        取得した音声データを WAV ファイルとして保存します。

        Parameters
        ----------
        file_path : str
            保存先のファイルパス。
        audio_data : np.ndarray
            保存する音声データ。int16 の numpy 配列。

        Returns
        -------
        bool
            保存に成功した場合は True。
        """
        if audio_data.size == 0:
            self.node.get_logger().warn("No audio data to save.")
            return False

        try:
            with wave.open(file_path, "wb") as wf:
                wf.setnchannels(self.__channels)
                wf.setsampwidth(2)  # 16-bit
                wf.setframerate(self.__sample_rate)
                wf.writeframes(audio_data.tobytes())

            self.node.get_logger().info(f"Successfully saved audio to {file_path}")
            return True
        except Exception as e:
            self.node.get_logger().error(f"Failed to save WAV file: {e}")
            return False
