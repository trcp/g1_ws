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
from g1_srvs.srv import MoveServo, PosePolicy
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
            True の場合、machida_navigation を使わず /odom と /cmd_vel による簡易移動を行う。
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


class Collision:
    """
    MoveItのプランニングシーンにおけるコリジョン(障害物)を管理するクラス。
    """

    def __init__(self, node: Node):
        self.node = node

        # Publisher for PlanningScene (more robust for MoveIt diffs)
        self.__scene_pub = self.node.create_publisher(
            PlanningScene, "/planning_scene", 10
        )

        # Service client for getting planning scene
        self.__get_scene_cli = self.node.create_client(
            GetPlanningScene, "/get_planning_scene"
        )

    def _publish_scene(self, co: CollisionObject):
        scene_msg = PlanningScene()
        scene_msg.is_diff = True
        scene_msg.world.collision_objects.append(co)
        self.__scene_pub.publish(scene_msg)

    def _create_collision_object(
        self,
        name: str,
        ref: str,
        x: float,
        y: float,
        z: float,
        roll: float,
        pitch: float,
        yaw: float,
        shape_type: int,
        dimensions: list,
        operation: int = CollisionObject.ADD,
    ) -> CollisionObject:
        co = CollisionObject()
        co.id = name
        co.header.frame_id = ref
        co.header.stamp = self.node.get_clock().now().to_msg()
        co.operation = operation

        if operation == CollisionObject.REMOVE:
            return co

        primitive = SolidPrimitive()
        primitive.type = shape_type
        primitive.dimensions = dimensions
        co.primitives.append(primitive)

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        q = quaternion_from_euler(roll, pitch, yaw)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        co.primitive_poses.append(pose)

        return co

    def add_box(
        self,
        name: str,
        ref: str = "base_link",
        x=0.0,
        y=0.0,
        z=0.0,
        roll=0.0,
        pitch=0.0,
        yaw=0.0,
        size=(0.1, 0.1, 0.1),
    ):
        """箱形状の CollisionObject をプランニングシーンに追加する。

        Parameters
        ----------
        name : str
            追加する CollisionObject の名前。
        ref : str, optional
            オブジェクト姿勢の基準フレーム。
        x : float, optional
            オブジェクト中心の x 座標(m)。
        y : float, optional
            オブジェクト中心の y 座標(m)。
        z : float, optional
            オブジェクト中心の z 座標(m)。
        roll : float, optional
            オブジェクト姿勢の roll 角(rad)。
        pitch : float, optional
            オブジェクト姿勢の pitch 角(rad)。
        yaw : float, optional
            オブジェクト姿勢の yaw 角(rad)。
        size : tuple, optional
            箱の寸法。x、y、z 方向の長さ(m)。
        """
        co = self._create_collision_object(
            name, ref, x, y, z, roll, pitch, yaw, SolidPrimitive.BOX, list(size)
        )
        self._publish_scene(co)

    def add_sphere(
        self,
        name: str,
        ref: str = "base_link",
        x=0.0,
        y=0.0,
        z=0.0,
        radius=0.05,
    ):
        """球形状の CollisionObject をプランニングシーンに追加する。

        Parameters
        ----------
        name : str
            追加する CollisionObject の名前。
        ref : str, optional
            オブジェクト姿勢の基準フレーム。
        x : float, optional
            球中心の x 座標(m)。
        y : float, optional
            球中心の y 座標(m)。
        z : float, optional
            球中心の z 座標(m)。
        radius : float, optional
            球の半径(m)。
        """
        co = self._create_collision_object(
            name, ref, x, y, z, 0, 0, 0, SolidPrimitive.SPHERE, [radius]
        )
        self._publish_scene(co)

    def add_cylinder(
        self,
        name: str,
        ref: str = "base_link",
        x=0.0,
        y=0.0,
        z=0.0,
        roll=0.0,
        pitch=0.0,
        yaw=0.0,
        height=0.1,
        radius=0.05,
    ):
        """円柱形状の CollisionObject をプランニングシーンに追加する。

        Parameters
        ----------
        name : str
            追加する CollisionObject の名前。
        ref : str, optional
            オブジェクト姿勢の基準フレーム。
        x : float, optional
            円柱中心の x 座標(m)。
        y : float, optional
            円柱中心の y 座標(m)。
        z : float, optional
            円柱中心の z 座標(m)。
        roll : float, optional
            オブジェクト姿勢の roll 角(rad)。
        pitch : float, optional
            オブジェクト姿勢の pitch 角(rad)。
        yaw : float, optional
            オブジェクト姿勢の yaw 角(rad)。
        height : float, optional
            円柱の高さ(m)。
        radius : float, optional
            円柱の半径(m)。
        """
        co = self._create_collision_object(
            name,
            ref,
            x,
            y,
            z,
            roll,
            pitch,
            yaw,
            SolidPrimitive.CYLINDER,
            [height, radius],
        )
        self._publish_scene(co)

    def remove_near_objects(self, x: float, y: float, z: float, radius: float = 0.05):
        """指定された座標の近くにあるすべてのオブジェクトを削除する。"""
        if not self.__get_scene_cli.wait_for_service(timeout_sec=1.0):
            return
        req = GetPlanningScene.Request()
        req.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            | PlanningSceneComponents.WORLD_OBJECT_NAMES
        )
        future = self.__get_scene_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
        if not future.done():
            return

        res = future.result()
        removed_count = 0
        for co in res.scene.world.collision_objects:
            # global_pose check
            gp = co.pose.position
            dist = math.sqrt((x - gp.x) ** 2 + (y - gp.y) ** 2 + (z - gp.z) ** 2)
            if dist < radius:
                self.remove_collision(co.id)
                removed_count += 1

        if removed_count > 0:
            self.node.get_logger().info(
                f"Removed {removed_count} near objects (ghosts) around ({x:.3f}, {y:.3f}, {z:.3f})"
            )

    def remove_collision(self, name: str):
        """指定した CollisionObject をプランニングシーンから削除する。

        Parameters
        ----------
        name : str
            削除する CollisionObject の名前。
        """
        co = CollisionObject()
        co.id = name
        co.operation = CollisionObject.REMOVE

        scene_msg = PlanningScene()
        scene_msg.is_diff = True
        scene_msg.world.collision_objects.append(co)

        # Also remove if attached
        aco = AttachedCollisionObject()
        aco.object.id = name
        aco.object.operation = CollisionObject.REMOVE
        scene_msg.robot_state.attached_collision_objects.append(aco)
        scene_msg.robot_state.is_diff = True

        self.__scene_pub.publish(scene_msg)

    def get_object_pose(self, name: str):
        """指定した CollisionObject の姿勢を取得する。

        Parameters
        ----------
        name : str
            姿勢を取得する CollisionObject の名前。

        Returns
        -------
        tuple or None
            オブジェクト姿勢を (x, y, z, roll, pitch, yaw) で返す。
            見つからない場合、または planning scene を取得できない場合は None。
        """
        if not self.__get_scene_cli.wait_for_service(timeout_sec=1.0):
            return None
        req = GetPlanningScene.Request()
        req.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            | PlanningSceneComponents.WORLD_OBJECT_NAMES
        )
        future = self.__get_scene_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
        if not future.done():
            return None
        res = future.result()
        for co in res.scene.world.collision_objects:
            if co.id == name:
                global_pose = co.pose
                if len(co.primitive_poses) > 0:
                    p = co.primitive_poses[0]
                    # Simplified combine
                    fx = global_pose.position.x + p.position.x
                    fy = global_pose.position.y + p.position.y
                    fz = global_pose.position.z + p.position.z
                    r, pt, y = euler_from_quaternion(
                        [
                            global_pose.orientation.x,
                            global_pose.orientation.y,
                            global_pose.orientation.z,
                            global_pose.orientation.w,
                        ]
                    )
                    return (fx, fy, fz, r, pt, y)
        return None

    def get_object(self, name: str):
        """指定した CollisionObject を取得する。

        Parameters
        ----------
        name : str
            取得する CollisionObject の名前。

        Returns
        -------
        CollisionObject or None
            見つかった CollisionObject。見つからない場合、または planning scene を
            取得できない場合は None。
        """
        if not self.__get_scene_cli.wait_for_service(timeout_sec=1.0):
            return None
        req = GetPlanningScene.Request()
        req.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            | PlanningSceneComponents.WORLD_OBJECT_NAMES
        )
        future = self.__get_scene_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
        if not future.done():
            return None
        res = future.result()
        for co in res.scene.world.collision_objects:
            if co.id == name:
                return co
        return None

    def attach_collision(
        self,
        name: str,
        link_name: str,
        touch_links: list = None,
        collision_object: CollisionObject = None,
    ):
        """CollisionObject をロボットリンクに attach する。

        Parameters
        ----------
        name : str
            attach する CollisionObject の名前。
        link_name : str
            CollisionObject を attach するリンク名。
        touch_links : list, optional
            接触を許可するリンク名のリスト。None の場合は link_name のみを使用する。
        collision_object : CollisionObject, optional
            attach に使用する CollisionObject。None の場合は name のみを持つ
            CollisionObject を attach する。
        """
        co = CollisionObject()
        co.id = name
        co.operation = CollisionObject.REMOVE

        aco = AttachedCollisionObject()
        aco.link_name = link_name
        if collision_object:
            aco.object = collision_object
            aco.object.id = name
            aco.object.operation = CollisionObject.ADD
        else:
            aco.object.id = name
            aco.object.operation = CollisionObject.ADD

        aco.touch_links = touch_links or [link_name]

        scene_msg = PlanningScene()
        scene_msg.is_diff = True
        scene_msg.world.collision_objects.append(co)
        scene_msg.robot_state.attached_collision_objects.append(aco)
        scene_msg.robot_state.is_diff = True

        self.__scene_pub.publish(scene_msg)

    def allow_collision(self, name1: str, name2: str):
        """ACMを更新して衝突を許可する"""
        if not self.__get_scene_cli.wait_for_service(timeout_sec=1.0):
            return
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        future = self.__get_scene_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
        if not future.done():
            return
        acm = future.result().scene.allowed_collision_matrix

        # Add if missing, and set enabled=False (allow collision)
        from moveit_msgs.msg import AllowedCollisionEntry

        def ensure_entry(name):
            if name not in acm.entry_names:
                acm.entry_names.append(name)
                for entry in acm.entry_values:
                    entry.enabled.append(True)
                new_entry = AllowedCollisionEntry()
                new_entry.enabled = [True] * len(acm.entry_names)
                acm.entry_values.append(new_entry)

        if name2 == "all":
            ensure_entry(name1)
            for existing in acm.entry_names:
                if existing != name1:
                    idx1 = acm.entry_names.index(name1)
                    idx2 = acm.entry_names.index(existing)
                    acm.entry_values[idx1].enabled[idx2] = False
                    acm.entry_values[idx2].enabled[idx1] = False
        else:
            ensure_entry(name1)
            ensure_entry(name2)
            idx1 = acm.entry_names.index(name1)
            idx2 = acm.entry_names.index(name2)
            acm.entry_values[idx1].enabled[idx2] = False
            acm.entry_values[idx2].enabled[idx1] = False

        msg = PlanningScene()
        msg.is_diff = True
        msg.allowed_collision_matrix = acm
        self.__scene_pub.publish(msg)

    def clear_all(self):
        """全てのオブジェクトをプランニングシーンから削除する"""
        if not self.__get_scene_cli.wait_for_service(timeout_sec=1.0):
            return
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        future = self.__get_scene_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
        if not future.done():
            return

        scene = future.result().scene
        msg = PlanningScene()
        msg.is_diff = True

        for obj in scene.world.collision_objects:
            co = CollisionObject()
            co.id = obj.id
            co.header.frame_id = obj.header.frame_id
            co.operation = CollisionObject.REMOVE
            msg.world.collision_objects.append(co)

        self.__scene_pub.publish(msg)
        self.node.get_logger().info(
            f"Cleared {len(scene.world.collision_objects)} objects from planning scene."
        )


class Grasp:
    """
    把持動作シーケンスを管理するクラス。
    """

    def __init__(self, arm, collision: Collision):
        self.arm = arm
        self.collision = collision

    def grasp(self, target_name: str, arm: str = None) -> bool:
        """
        指定されたオブジェクトを把持する。複数の姿勢（角度）を試行し、到達可能な解を探す。
        """
        # 1. オブジェクト情報を取得
        pose = self.collision.get_object_pose(target_name)
        if not pose:
            self.arm.node.get_logger().error(
                f"Object {target_name} not found in planning scene"
            )
            return False

        ox, oy, oz, _, _, _ = pose
        self.arm.node.get_logger().info(
            f"Grasp target: {target_name} at {ox:.3f}, {oy:.3f}, {oz:.3f}"
        )

        # 2. 腕の選択
        if not arm:
            arm = "arm_right" if oy < 0 else "arm_left"
        tip_link = "right_amazing_hand" if arm == "arm_right" else "left_amazing_hand"
        hand_side = "right" if arm == "arm_right" else "left"

        # 3. アプローチ方向の計算 (Robot-to-Object)
        dist_xy = math.sqrt(ox**2 + oy**2)
        nx, ny = (ox / dist_xy, oy / dist_xy) if dist_xy > 1e-6 else (1.0, 0.0)
        base_yaw = math.atan2(ny, nx)

        # 試行する姿勢リスト (ピッチ角: 0.0=水平, 0.8=斜め, 1.57=真上)
        # G1のリーチ制約（腰ピッチなし）のため、水平に近いほうが届きやすい
        pitches = [1.0, 0.5, 0.0, 1.57]

        self.arm.hand_control(command="open", hand=hand_side)
        time.sleep(0.5)

        for pitch in pitches:
            self.arm.node.get_logger().info(f"Trying grasp with pitch={pitch:.2f}")

            # 手をオブジェクトにぶつけないためのプリポーズ (15cm手前)
            pre_offset = 0.15
            px = ox - nx * pre_offset
            py = oy - ny * pre_offset
            pz = oz + 0.15

            # 姿勢 (AmazingHandのロール軸: Right=1.57, Left=-1.57)
            roll = 1.57 if arm == "arm_right" else -1.57
            yaw = base_yaw

            # # A. Pre-grasp (到達可能性確認)
            # self.arm.node.get_logger().info(f"Step A: Pre-grasp move to {px:.3f}, {py:.3f}, {pz:.3f}")
            # # arm_left/arm_right グループは腰(waist_yaw)を含むため、こちらでIKを解く
            # if not self.arm.move_abs(px, py, pz, roll, pitch, yaw, planning_group=arm, tip_link=tip_link, position_only=True):
            #     continue

            # B. コリジョン一時無効化
            self.collision.allow_collision(tip_link, "all")
            self.collision.remove_near_objects(ox, oy, oz, radius=0.1)
            self.collision.remove_collision(target_name)
            time.sleep(0.3)

            # ターゲット位置の計算 (ピッチに合わせて手前オフセットを微調整)
            grasp_offset = 0.03
            tx = ox - nx * grasp_offset * math.sin(pitch) if pitch > 0.1 else ox - 0.06
            ty = oy - ny * grasp_offset * math.sin(pitch) if pitch > 0.1 else oy
            tz = oz + grasp_offset * math.cos(pitch) if pitch > 0.1 else oz

            # C. Final Grasp
            self.arm.node.get_logger().info(
                f"Step C: Final grasp move to {tx:.3f}, {ty:.3f}, {tz:.3f}"
            )
            if not self.arm.move_abs(
                tx,
                ty,
                tz,
                roll,
                pitch,
                yaw,
                planning_group=arm,
                tip_link=tip_link,
                orientation_tolerance=0.5,
            ):
                # 失敗したらコリジョンを戻して次へ
                self.arm.node.get_logger().warn(
                    f"Final grasp failed at pitch {pitch:.2f}, trying next..."
                )
                continue

            # D. Close & Attach
            self.arm.hand_control(command="close", hand=hand_side)
            time.sleep(1.0)
            self.collision.attach_collision(target_name, link_name=tip_link)

            # E. Lift
            self.arm.move_rel(z=0.1, planning_group="upper_body")
            self.arm.node.get_logger().info(f"Grasp success at pitch {pitch:.2f}!")
            return True

        self.arm.node.get_logger().error("Grasp failed with all pitch candidates.")
        return False

        return False


class ArmControl:
    def __init__(self, node: Node, wait_time: int = 5, tf_buffer: Buffer = None):
        """
        ArmControl クラスのコンストラクタ

        Parameters
        ----------
        node : Node
            ROS2 ノードオブジェクト
        wait_time : int, optional
            アクションサーバーの接続待機時間(秒)。デフォルトは 5。
        tf_buffer : Buffer, optional
            TF2 バッファオブジェクト。None の場合は新規作成。デフォルトは None。
        """
        self.node = node
        self.__current_goal_handles = []

        # TF2 Setup
        self.tf_buffer = tf_buffer or Buffer()
        self.__tf_listener = TransformListener(self.tf_buffer, self.node)

        # MoveGroup Action Client
        self.__move_group_client = ActionClient(self.node, MoveGroup, "/move_action")

        if not self.__move_group_client.wait_for_server(timeout_sec=wait_time):
            self.node.get_logger().error("MoveGroup action server not available...")

        # IK Service Client
        self.__ik_cli = self.node.create_client(GetPositionIK, "/compute_ik")
        if not self.__ik_cli.wait_for_service(timeout_sec=wait_time):
            self.node.get_logger().error("IK service /compute_ik not available...")

        # Joint states storage
        self.__joint_states = {}
        self.__srdf_group_states = None
        self.__joint_sub = self.node.create_subscription(
            JointState, "/joint_states", self.__joint_state_callback, 10
        )

        # Amazing Hand
        self.__hand_cli = self.node.create_client(HandCommand, "/hand_command")
        while not self.__hand_cli.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error("Hand Service Servers are not running ...")
            break

        # upper body control
        self.__ubc_cli = self.node.create_client(SetBool, "/enable_upper_body_control")
        while not self.__ubc_cli.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error(
                "eR@sers G1 Service Servers are not running ..."
            )
            break

        # New Infrastructure
        self.collision = Collision(self.node)
        self.grasp_manager = Grasp(self, self.collision)

    def __send_ubc_req(self, req: SetBool.Request):
        future = self.__ubc_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        response: MoveServo.Response = future.result()
        return response.success

    def __joint_state_callback(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self.__joint_states[name] = pos

    def __load_srdf_group_states(self):
        if self.__srdf_group_states is not None:
            return self.__srdf_group_states

        try:
            g1_moveit_share = get_package_share_directory("g1_moveit")
        except PackageNotFoundError:
            self.node.get_logger().error("Package 'g1_moveit' was not found.")
            return None

        srdf_path = os.path.join(g1_moveit_share, "config", "g1.srdf")
        try:
            root = ET.parse(srdf_path).getroot()
        except (OSError, ET.ParseError) as err:
            self.node.get_logger().error(f"Failed to load SRDF '{srdf_path}': {err}")
            return None

        group_states = {}
        for group_state in root.findall("group_state"):
            state_name = group_state.get("name")
            group_name = group_state.get("group")
            if not state_name or not group_name:
                continue

            joints = {}
            for joint in group_state.findall("joint"):
                joint_name = joint.get("name")
                joint_value = joint.get("value")
                if not joint_name or joint_value is None:
                    continue

                try:
                    joints[joint_name] = float(joint_value)
                except ValueError:
                    self.node.get_logger().error(
                        f"Invalid SRDF value for {group_name}/{state_name}: "
                        f"{joint_name}={joint_value}"
                    )
                    return None

            group_states[(group_name, state_name)] = joints

        self.__srdf_group_states = group_states
        return self.__srdf_group_states

    def __get_srdf_group_state_joints(self, group_name: str, group_state: str):
        group_states = self.__load_srdf_group_states()
        if group_states is None:
            return None

        joints = group_states.get((group_name, group_state))
        if joints is None:
            available = ", ".join(
                f"{group}/{state}" for group, state in sorted(group_states.keys())
            )
            self.node.get_logger().error(
                f"SRDF group_state '{group_state}' for group '{group_name}' "
                f"was not found. Available states: {available}"
            )
            return None

        if not joints:
            self.node.get_logger().error(
                f"SRDF group_state '{group_state}' for group '{group_name}' "
                "does not define any joints."
            )
            return None

        return joints

    def enable_upper_body_control(self, enable: bool = True):
        """上半身制御の有効/無効を切り替える。

        Parameters
        ----------
        enable : bool, optional
            True の場合は上半身制御を有効化し、False の場合は無効化する。

        Returns
        -------
        bool
            サービス呼び出しが成功した場合は True。
        """
        req = SetBool.Request()
        req.data = enable
        return self.__send_ubc_req(req)

    def get_current_joints_pose(self, planning_group: str = "upper_body"):
        """
        現在の各ジョイントの角度を取得します。

        Returns
        -------
        dict
            各ジョイント名をキー、角度(rad)を値とする辞書
        """
        return self.__joint_states.copy()

    def get_current_pose(
        self,
        simple: bool = False,
        planning_group: str = "upper_body",
        reference_frame: str = "base_link",
    ):
        """
        指定されたエンドエフェクタの現在位置姿勢を取得する．

        Parameters
        ----------
        simple : bool, optional
            True の場合、[x, y, z, roll, pitch, yaw] の1次元リストとして現在位置を出力する。
            False の場合、PoseStamped 型で現在位置を出力する。デフォルトは False。
        planning_group : str, optional
            SRDFで定義されたプランニンググループ名。デフォルトは 'upper_body'。
        reference_frame : str, optional
            基準となる座標フレーム。デフォルトは 'base_link'。

        Returns
        -------
        PoseStamped or list of float
            現在の姿勢データ。
        """
        tip_link = (
            "left_amazing_hand" if "left" in planning_group else "right_amazing_hand"
        )

        start_time = self.node.get_clock().now()
        while (
            rclpy.ok() and (self.node.get_clock().now() - start_time).nanoseconds < 2e9
        ):  # 2s timeout
            rclpy.spin_once(self.node, timeout_sec=0.1)
            try:
                transform = self.tf_buffer.lookup_transform(
                    reference_frame, tip_link, rclpy.time.Time()
                )
                pos = transform.transform.translation
                rot = transform.transform.rotation

                if simple:
                    (roll, pitch, yaw) = euler_from_quaternion(
                        [rot.x, rot.y, rot.z, rot.w]
                    )
                    return [pos.x, pos.y, pos.z, roll, pitch, yaw]
                else:
                    pose = PoseStamped()
                    pose.header.frame_id = reference_frame
                    pose.header.stamp = self.node.get_clock().now().to_msg()
                    pose.pose.position.x = pos.x
                    pose.pose.position.y = pos.y
                    pose.pose.position.z = pos.z
                    pose.pose.orientation = rot
                    return pose
            except Exception as e:
                self.node.get_logger().debug(
                    f"TF Lookup failed for {tip_link}: {str(e)}"
                )
                continue
        return None

    def move_to_pose(
        self,
        pose,
        planning_group: str = "upper_body",
        wait: bool = True,
        tip_link: str = None,
        **kwargs,
    ) -> bool:
        """
        与えられた目標姿勢に向けてエンドエフェクタを自律移動させる．
        """
        # Determine tip link (assuming standard names for G1)
        if tip_link is None:
            tip_link = (
                "left_amazing_hand"
                if "left" in planning_group
                else "right_amazing_hand"
            )

        # Formulate goal constraints
        target_pose = pose
        if isinstance(pose, Pose):
            target_pose = PoseStamped()
            target_pose.header.frame_id = "base_link"
            target_pose.pose = pose

        # Deletage to joint goal by solving IK first, as it is more robust than task-space planning in G1
        joints = self._solve_ik(target_pose, planning_group, tip_link=tip_link)
        if joints:
            return self.joint_control(
                **joints,
                wait=wait,
                planning_group=planning_group,
                planning_attempts=kwargs.get("planning_attempts", 10),
                planning_time=kwargs.get("planning_time", 5.0),
            )

        # Fallback to Task-space planning if IK solver fails
        # Try position + orientation first
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = planning_group
        goal_msg.request.num_planning_attempts = kwargs.get("planning_attempts", 15)
        goal_msg.request.allowed_planning_time = kwargs.get("planning_time", 5.0)

        l_pc, l_oc = self._create_pose_constraints(target_pose, tip_link)
        goal_msg.request.goal_constraints.append(
            Constraints(position_constraints=[l_pc], orientation_constraints=[l_oc])
        )
        if self._send_move_group_goal(goal_msg, wait):
            return True

        # Last resort: Position-only planning (for 1.5x larger gripper)
        self.node.get_logger().warn(
            f"Trying position-only planning for {planning_group}"
        )
        goal_msg.request.goal_constraints[0].orientation_constraints = []
        return self._send_move_group_goal(goal_msg, wait)

    def place(
        self,
        x: float,
        y: float,
        z: float,
        planning_group: str = "arm_right",
        wait: bool = True,
    ) -> bool:
        """
        指定された位置に物体を配置する（Position Constraintのみを使用）。
        """
        self.node.get_logger().info(f"Placing object at {x:.3f}, {y:.3f}, {z:.3f}")

        tip_link = (
            "left_amazing_hand" if "left" in planning_group else "right_amazing_hand"
        )

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = planning_group
        goal_msg.request.num_planning_attempts = 15
        goal_msg.request.allowed_planning_time = 5.0

        constraints = Constraints()
        pc = PositionConstraint()
        pc.header.frame_id = "base_link"
        pc.link_name = tip_link

        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        pc.constraint_region.primitive_poses.append(target_pose)

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [
            0.05,
            0.05,
            0.05,
        ]  # 5cm tolerance (Relaxed for G1 reliability)
        pc.constraint_region.primitives.append(box)
        pc.weight = 1.0

        constraints.position_constraints.append(pc)
        goal_msg.request.goal_constraints.append(constraints)

        return self._send_move_group_goal(goal_msg, wait)

    def _create_pose_constraints(self, target_pose: PoseStamped, tip_link: str):
        """
        PoseStamped から PositionConstraint と OrientationConstraint を生成する内部ヘルパー。
        (pc, oc) のタプルを返す。
        """
        # Position Constraint
        pc = PositionConstraint()
        pc.header.frame_id = target_pose.header.frame_id
        pc.link_name = tip_link
        pc.constraint_region.primitive_poses.append(target_pose.pose)

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.05, 0.05, 0.05]
        pc.constraint_region.primitives.append(box)
        pc.weight = 1.0

        # Orientation Constraint
        oc = OrientationConstraint()
        oc.header.frame_id = target_pose.header.frame_id
        oc.link_name = tip_link
        oc.orientation = target_pose.pose.orientation
        oc.absolute_x_axis_tolerance = 0.1
        oc.absolute_y_axis_tolerance = 0.1
        oc.absolute_z_axis_tolerance = 0.1
        oc.weight = 1.0

        return pc, oc

    def _solve_ik(
        self, pose_stamped: PoseStamped, group_name: str, tip_link: str = None
    ) -> dict:
        """
        MoveIt の /compute_ik サービスを使用して特定のグループの逆運動学を解く。
        """
        self.node.get_logger().info(
            f"Solving IK for {group_name} (tip: {tip_link}) at pose: {pose_stamped.pose.position.x:.3f}, {pose_stamped.pose.position.y:.3f}, {pose_stamped.pose.position.z:.3f}"
        )

        leg_joints = [
            "left_hip_pitch_joint",
            "left_hip_roll_joint",
            "left_hip_yaw_joint",
            "left_knee_joint",
            "left_ankle_pitch_joint",
            "left_ankle_roll_joint",
            "right_hip_pitch_joint",
            "right_hip_roll_joint",
            "right_hip_yaw_joint",
            "right_knee_joint",
            "right_ankle_pitch_joint",
            "right_ankle_roll_joint",
        ]

        def call_ik(seed_joint_positions=None):
            req = GetPositionIK.Request()
            req.ik_request.group_name = group_name
            if tip_link:
                req.ik_request.ik_link_name = tip_link
            req.ik_request.pose_stamped = pose_stamped
            req.ik_request.timeout.sec = 1
            req.ik_request.timeout.nanosec = 0
            req.ik_request.avoid_collisions = (
                False  # RELAXED: Handle collisions in planning
            )

            # Populate joint states (including missing legs to avoid MoveIt warnings/errors)
            all_joint_names = list(self.__joint_states.keys())
            all_joint_positions = list(self.__joint_states.values())

            for lj in leg_joints:
                if lj not in self.__joint_states:
                    all_joint_names.append(lj)
                    all_joint_positions.append(0.0)

            if seed_joint_positions:
                # Override positions with seed (keep names same)
                pos_dict = dict(zip(all_joint_names, all_joint_positions))
                for name, pos in seed_joint_positions.items():
                    if name in pos_dict:
                        pos_dict[name] = pos
                all_joint_names = list(pos_dict.keys())
                all_joint_positions = list(pos_dict.values())

            req.ik_request.robot_state.joint_state.name = all_joint_names
            req.ik_request.robot_state.joint_state.position = all_joint_positions

            future = self.__ik_cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=3.0)
            return future.result()

        # Attempt 1: Current state
        res = call_ik()
        if res and res.error_code.val == MoveItErrorCodes.SUCCESS:
            return dict(
                zip(res.solution.joint_state.name, res.solution.joint_state.position)
            )

        # Attempt 2: Relaxed timeout and internal search
        # MoveIt solver usually handles seeds better internally if given time.
        # We also increase the timeout for the internal solver.
        req = GetPositionIK.Request()
        req.ik_request.group_name = group_name
        if tip_link:
            req.ik_request.ik_link_name = tip_link
        req.ik_request.pose_stamped = pose_stamped
        req.ik_request.timeout.sec = 2  # Increase to 2s
        req.ik_request.timeout.nanosec = 0
        req.ik_request.avoid_collisions = True

        # Populate joint states
        all_joint_names = list(self.__joint_states.keys())
        all_joint_positions = list(self.__joint_states.values())
        for lj in leg_joints:
            if lj not in self.__joint_states:
                all_joint_names.append(lj)
                all_joint_positions.append(0.0)
        req.ik_request.robot_state.joint_state.name = all_joint_names
        req.ik_request.robot_state.joint_state.position = all_joint_positions

        future = self.__ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=3.0)
        res = future.result() if future.done() else None

        if res and res.error_code.val == MoveItErrorCodes.SUCCESS:
            return dict(
                zip(res.solution.joint_state.name, res.solution.joint_state.position)
            )

        if res:
            self.node.get_logger().error(
                f"IK failed for {group_name} with error: {res.error_code.val}"
            )
        else:
            self.node.get_logger().error(f"IK service call timed out for {group_name}")
        return None

        if res:
            self.node.get_logger().error(
                f"IK failed for {group_name} with error: {res.error_code.val}"
            )
        else:
            self.node.get_logger().error(f"IK service call timed out for {group_name}")
        return None

    def move_abs(
        self,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
        planning_group: str = "upper_body",
        wait: bool = True,
        reference_frame: str = "base_link",
        tip_link: str = None,
        **kwargs,
    ) -> bool:
        """
        絶対座標指定でロボットを移動させる．

        Parameters
        ----------
        x, y, z : float
            目標位置。
        roll, pitch, yaw : float
            目標姿勢（ラジアン）。
        planning_group : str, optional
            プランニンググループ名。デフォルトは 'upper_body'。
        wait : bool, optional
            完了待機の有無。デフォルトは True。
        reference_frame : str, optional
            基準座標系。デフォルトは 'base_link'。

        Returns
        -------
        bool
            成功時は True。
        """
        pose = PoseStamped()
        pose.header.frame_id = reference_frame
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)

        q = quaternion_from_euler(roll, pitch, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        return self.move_to_pose(
            pose, planning_group=planning_group, wait=wait, tip_link=tip_link, **kwargs
        )

    def move_rel(
        self,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
        planning_group: str = "upper_body",
        wait: bool = True,
        **kwargs,
    ) -> bool:
        """
        現在位置姿勢からの相対移動．
        """
        current_pose = self.get_current_pose(simple=True, planning_group=planning_group)
        if current_pose is None:
            self.node.get_logger().error(
                "Could not get current pose for relative movement"
            )
            return False

        cx, cy, cz, croll, cpitch, cyaw = current_pose
        self.node.get_logger().info(
            f"Current {planning_group} pose: {cx:.3f}, {cy:.3f}, {cz:.3f}"
        )

        new_x = cx + x
        new_y = cy + y
        new_z = cz + z

        self.node.get_logger().info(
            f"Target {planning_group} pose: {new_x:.3f}, {new_y:.3f}, {new_z:.3f} (rel x={x})"
        )

        q_curr = tf_transformations.quaternion_from_euler(croll, cpitch, cyaw)
        q_rel = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        q_new = tf_transformations.quaternion_multiply(q_curr, q_rel)

        (nr, np, ny) = euler_from_quaternion(q_new)

        return self.move_abs(
            x=new_x,
            y=new_y,
            z=new_z,
            roll=nr,
            pitch=np,
            yaw=ny,
            planning_group=planning_group,
            wait=wait,
            **kwargs,
        )

    def move_dual_abs(
        self,
        lx=0.0,
        ly=0.0,
        lz=0.0,
        lr=0.0,
        lp=0.0,
        lyaw=0.0,
        rx=0.0,
        ry=0.0,
        rz=0.0,
        rr=0.0,
        rp=0.0,
        ryaw=0.0,
        wait=True,
        reference_frame="base_link",
        **kwargs,
    ) -> bool:
        """
        左右の手の目標座標を同時に指定して移動させる。
        planning_group は強制的に 'upper_body' が使用されます。
        """
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "upper_body"
        goal_msg.request.num_planning_attempts = kwargs.get("planning_attempts", 10)
        goal_msg.request.allowed_planning_time = kwargs.get("planning_time", 5.0)

        # Left Arm Pose
        l_pose = PoseStamped()
        l_pose.header.frame_id = reference_frame
        l_pose.header.stamp = self.node.get_clock().now().to_msg()
        l_pose.pose.position.x = float(lx)
        l_pose.pose.position.y = float(ly)
        l_pose.pose.position.z = float(lz)
        l_q = quaternion_from_euler(lr, lp, lyaw)
        l_pose.pose.orientation.x = l_q[0]
        l_pose.pose.orientation.y = l_q[1]
        l_pose.pose.orientation.z = l_q[2]
        l_pose.pose.orientation.w = l_q[3]
        l_pc, l_oc = self._create_pose_constraints(l_pose, "left_amazing_hand")

        # Right Arm Pose
        r_pose = PoseStamped()
        r_pose.header.frame_id = reference_frame
        r_pose.header.stamp = self.node.get_clock().now().to_msg()
        r_pose.pose.position.x = float(rx)
        r_pose.pose.position.y = float(ry)
        r_pose.pose.position.z = float(rz)
        r_q = quaternion_from_euler(rr, rp, ryaw)
        r_pose.pose.orientation.x = r_q[0]
        r_pose.pose.orientation.y = r_q[1]
        r_pose.pose.orientation.z = r_q[2]
        r_pose.pose.orientation.w = r_q[3]
        # Right Arm IK
        r_pc, r_oc = self._create_pose_constraints(r_pose, "right_amazing_hand")

        # Solve IK for both arms to get joint targets
        l_joints = self._solve_ik(l_pose, "arm_left")
        r_joints = self._solve_ik(r_pose, "arm_right")

        if l_joints is None or r_joints is None:
            self.node.get_logger().error(
                f"Dual IK solving failed. L: {'Success' if l_joints else 'Fail'}, R: {'Success' if r_joints else 'Fail'}"
            )
            return False

        self.node.get_logger().info(
            "Dual IK solved successfully. Proceeding with joint-space planning."
        )

        # Build joint constraints for upper_body
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "upper_body"
        goal_msg.request.num_planning_attempts = kwargs.get("planning_attempts", 10)
        goal_msg.request.allowed_planning_time = kwargs.get("planning_time", 5.0)

        constraints = Constraints()
        # combine joints (L & R)
        target_joints = {**l_joints, **r_joints}

        # Define target joints for upper_body (Waist + Both Arms)
        upper_body_joints = [
            "waist_yaw_joint",
            "left_shoulder_pitch_joint",
            "left_shoulder_roll_joint",
            "left_shoulder_yaw_joint",
            "left_elbow_joint",
            "left_wrist_roll_joint",
            "right_shoulder_pitch_joint",
            "right_shoulder_roll_joint",
            "right_shoulder_yaw_joint",
            "right_elbow_joint",
            "right_wrist_roll_joint",
        ]

        for j_name in upper_body_joints:
            jc = JointConstraint()
            jc.joint_name = j_name
            # IK結果があればそれを使用、なければ現在値を保持
            jc.position = target_joints.get(
                j_name, self.__joint_states.get(j_name, 0.0)
            )
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        goal_msg.request.goal_constraints.append(constraints)
        return self._send_move_group_goal(goal_msg, wait)

    def move_dual_rel(
        self,
        lx=0.0,
        ly=0.0,
        lz=0.0,
        lr=0.0,
        lp=0.0,
        lyaw=0.0,
        rx=0.0,
        ry=0.0,
        rz=0.0,
        rr=0.0,
        rp=0.0,
        ryaw=0.0,
        wait=True,
        **kwargs,
    ) -> bool:
        """
        現在位置からの左右同時の相対移動。
        """
        l_curr = self.get_current_pose(simple=True, planning_group="arm_left")
        r_curr = self.get_current_pose(simple=True, planning_group="arm_right")

        if l_curr is None or r_curr is None:
            self.node.get_logger().error(
                "Could not get current pose for dual relative movement"
            )
            return False

        # Left Relative
        clx, cly, clz, clr, clp, clyaw = l_curr
        nlx, nly, nlz = clx + lx, cly + ly, clz + lz
        ql_curr = tf_transformations.quaternion_from_euler(clr, clp, clyaw)
        ql_rel = tf_transformations.quaternion_from_euler(lr, lp, lyaw)
        ql_new = tf_transformations.quaternion_multiply(ql_curr, ql_rel)
        new_lr, new_lp, new_lyaw = euler_from_quaternion(ql_new)

        # Right Relative
        crx, cry, crz, crr, crp, cryaw = r_curr
        nrx, nry, nrz = crx + rx, cry + ry, crz + rz
        qr_curr = tf_transformations.quaternion_from_euler(crr, crp, cryaw)
        qr_rel = tf_transformations.quaternion_from_euler(rr, rp, ryaw)
        qr_new = tf_transformations.quaternion_multiply(qr_curr, qr_rel)
        new_rr, new_rp, new_ryaw = euler_from_quaternion(qr_new)

        return self.move_dual_abs(
            lx=nlx,
            ly=nly,
            lz=nlz,
            lr=new_lr,
            lp=new_lp,
            lyaw=new_lyaw,
            rx=nrx,
            ry=nry,
            rz=nrz,
            rr=new_rr,
            rp=new_rp,
            ryaw=new_ryaw,
            wait=wait,
            **kwargs,
        )

    def move_groupstate(
        self,
        group_name: str = "upper_body",
        group_state: str = "home",
        wait: bool = True,
    ) -> bool:
        """
        SRDF に定義された MoveIt group_state へ遷移する。

        `src/g1_moveit/config/g1.srdf` の group_state 定義を読み取り、
        指定グループの全ジョイントに JointConstraint を設定して MoveGroup に送る。

        Parameters
        ----------
        group_name : str, optional
            遷移対象の MoveIt planning group 名。選択可能な主なグループは
            `upper_body`, `arm_left`, `arm_right`。

            SRDF には内部用として `__pure_arm_left`, `__pure_arm_right`,
            `__left_hand`, `__right_hand` も定義されているが、この API では
            通常 `upper_body`, `arm_left`, `arm_right` を指定する。
        group_state : str, optional
            SRDF に定義された遷移姿勢名。

            選択可能な group_state は次の通り。

            - `upper_body`: `home`, `walk`
            - `arm_left`: `home`, `walk`
            - `arm_right`: `home`, `walk`
            - `__pure_arm_left`: `walk`（内部用）
            - `__pure_arm_right`: `walk`（内部用）

            `home` はゼロ姿勢、`walk` は歩行時に近い腕姿勢を表す。
        wait : bool, optional
            True の場合は MoveGroup action の完了を待つ。False の場合は goal
            送信後に結果を待たずに戻る。

        Returns
        -------
        bool
            MoveGroup goal の送信または実行が成功した場合 True。
            指定した `group_name` と `group_state` の組み合わせが SRDF に存在しない、
            または MoveGroup goal が失敗した場合 False。
        """
        group_states = self.__load_srdf_group_states()
        if group_states is None:
            return False

        available_groups = {group for group, _ in group_states.keys()}
        available_states = {state for _, state in group_states.keys()}
        if group_name in available_states and group_state in available_groups:
            group_name, group_state = group_state, group_name
        if (
            group_state == "home"
            and group_name not in available_groups
            and group_name in available_states
        ):
            group_state = group_name
            group_name = "upper_body"

        joints = self.__get_srdf_group_state_joints(group_name, group_state)
        if joints is None:
            return False

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = group_name

        constraints = Constraints()
        for joint_name, joint_value in joints.items():
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = joint_value
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        goal_msg.request.goal_constraints.append(constraints)
        return self._send_move_group_goal(goal_msg, wait)

    def joint_control(
        self,
        rlt: bool = False,
        wait: bool = True,
        planning_group: str = "upper_body",
        **kwargs,
    ) -> bool:
        """
        MoveIt planning group に対してジョイント角度を直接指定する。

        `src/g1_moveit/config/g1.srdf` の planning group 定義に対応する
        ジョイントへ JointConstraint を設定して MoveGroup に送る。
        `kwargs` には制御したいジョイント名をキー、目標角度(rad)を値として渡す。
        指定されないジョイントは現在角度を保持する。

        Parameters
        ----------
        rlt : bool, optional
            True の場合、`kwargs` の値を現在角度からの相対角度(rad)として扱う。
            False の場合、`kwargs` の値を絶対角度(rad)として扱う。
        wait : bool, optional
            True の場合は MoveGroup action の完了を待つ。False の場合は goal
            送信後に結果を待たずに戻る。
        planning_group : str, optional
            制御対象の planning group 名。選択可能なグループは
            `upper_body`, `arm_left`, `arm_right`。

            SRDF には `__pure_arm_left`, `__pure_arm_right`,
            `__left_hand`, `__right_hand` も定義されているが、
            このメソッドの実装では上記 3 グループのみ対応する。
        **kwargs : float
            ジョイント名と目標角度(rad)の対応。

            `upper_body` で制御可能なジョイント:

            - `waist_yaw_joint`
            - `left_shoulder_pitch_joint`
            - `left_shoulder_roll_joint`
            - `left_shoulder_yaw_joint`
            - `left_elbow_joint`
            - `left_wrist_roll_joint`
            - `right_shoulder_pitch_joint`
            - `right_shoulder_roll_joint`
            - `right_shoulder_yaw_joint`
            - `right_elbow_joint`
            - `right_wrist_roll_joint`

            `arm_left` で制御可能なジョイント:

            - `left_shoulder_pitch_joint`
            - `left_shoulder_roll_joint`
            - `left_shoulder_yaw_joint`
            - `left_elbow_joint`
            - `left_wrist_roll_joint`

            `arm_right` で制御可能なジョイント:

            - `right_shoulder_pitch_joint`
            - `right_shoulder_roll_joint`
            - `right_shoulder_yaw_joint`
            - `right_elbow_joint`
            - `right_wrist_roll_joint`

        Returns
        -------
        bool
            MoveGroup goal の送信または実行が成功した場合 True。
            未対応の `planning_group` が指定された場合、または MoveGroup goal が
            失敗した場合 False。

        Examples
        --------
        >>> arm.joint_control(
        ...     planning_group="upper_body",
        ...     right_shoulder_pitch_joint=-0.7,
        ...     right_elbow_joint=1.0,
        ... )
        """
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = planning_group

        joints = []
        if planning_group == "arm_left":
            joints = [
                "left_shoulder_pitch_joint",
                "left_shoulder_roll_joint",
                "left_shoulder_yaw_joint",
                "left_elbow_joint",
                "left_wrist_roll_joint",
            ]
        elif planning_group == "arm_right":
            joints = [
                "right_shoulder_pitch_joint",
                "right_shoulder_roll_joint",
                "right_shoulder_yaw_joint",
                "right_elbow_joint",
                "right_wrist_roll_joint",
            ]
        elif planning_group == "upper_body":
            joints = [
                "waist_yaw_joint",
                "left_shoulder_pitch_joint",
                "left_shoulder_roll_joint",
                "left_shoulder_yaw_joint",
                "left_elbow_joint",
                "left_wrist_roll_joint",
                "right_shoulder_pitch_joint",
                "right_shoulder_roll_joint",
                "right_shoulder_yaw_joint",
                "right_elbow_joint",
                "right_wrist_roll_joint",
            ]
        else:
            self.node.get_logger().error(f"Unsupported group: {planning_group}")
            return False

        constraints = Constraints()
        for j_name in joints:
            jc = JointConstraint()
            jc.joint_name = j_name
            if j_name in kwargs:
                val = kwargs[j_name]
                if rlt:
                    jc.position = self.__joint_states.get(j_name, 0.0) + val
                else:
                    jc.position = val
            else:
                jc.position = self.__joint_states.get(j_name, 0.0)

            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        goal_msg.request.goal_constraints.append(constraints)
        return self._send_move_group_goal(goal_msg, wait)

    def _send_move_group_goal(self, goal_msg, wait):
        self.node.get_logger().info(
            f"Sending MoveGroup goal for group: {goal_msg.request.group_name}"
        )
        future = self.__move_group_client.send_goal_async(goal_msg)

        if not wait:
            return True

        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        if not future.done():
            self.node.get_logger().error("MoveGroup goal sending timed out")
            return False

        handle = future.result()
        if not handle.accepted:
            self.node.get_logger().error("MoveGroup goal rejected")
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        result = result_future.result()

        if result.result.error_code.val == MoveItErrorCodes.SUCCESS:
            return True
        else:
            self.node.get_logger().error(
                f"MoveGroup failed with error code: {result.result.error_code.val}"
            )
            return False

    def __send_hand_req(self, req: HandCommand.Request):
        """
        ハンド操作リクエストを送信する内部メソッド

        Parameters
        ----------
        req : HandCommand.Request
            ハンド操作の要求メッセージ。

        Returns
        -------
        bool
            サービス呼び出しが成功した場合は True、失敗した場合は False。
        """
        future = self.__hand_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        response: HandCommand.Response = future.result()
        return response.success

    def hand_control(self, command: str = "walk", hand="both"):
        """
        ハンド操作コマンドを送信する。

        Parameters
        ----------
        command : str, optional
            HandCommand サービスに渡す命令文字列。
        hand : str, optional
            操作対象の手。'left', 'right', 'both' などを指定する。

        Returns
        -------
        bool
            サービス呼び出しが成功した場合は True、失敗した場合は False。
        """
        req = HandCommand.Request()
        req.command = command
        req.hand = hand
        return self.__send_hand_req(req)


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
