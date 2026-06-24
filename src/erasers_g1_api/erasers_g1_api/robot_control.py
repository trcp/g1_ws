import numpy as np

try:
    np.float = float
except AttributeError:
    pass

#!/usr/bin/env python3
from rclpy.node import Node
import random
import rclpy

# msgs
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose
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
    PlanningSceneComponents
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
            適用するポーズポリシーの識別子。

        Returns
        -------
        bool
            サービス呼び出しが成功した場合は True、失敗した場合は False。
        """
        req = PosePolicy.Request()
        req.pose = pose
        return self.__send_pose_req(req)


class G1Navigation:
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
        self.FACE_GOAL_TIMEOUT_SEC = 10.0
        self.__current_goal_handle = None

        # TF2 Setup
        self.tf_buffer = tf_buffer or Buffer()
        self.__tf_listener = TransformListener(self.tf_buffer, self.node)

        # Action Client Setup
        self.__action_client = ActionClient(
            self.node, NavigateToPose, "/navigate_to_pose"
        )
        if not self.__action_client.wait_for_server(timeout_sec=wait_time):
            self.node.get_logger().fatal("Nav2 action server not available...")
            #raise RuntimeError("Nav2 action server not available")

        # Initial pose publisher
        self.__initial_pose_pub = self.node.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self.__debug_goal_pub = self.node.create_publisher(
            PoseStamped, debug_goal_topic, 10
        )

    def get_current_pose(self, simple: bool = False):
        """
        現在のロボットの位置姿勢を取得する．

        Parameters
        ----------
        simple : bool, optional
            True の場合、[x, y, yaw] の1次元リストとして現在位置を出力する。
            False の場合、PoseStamped 型で現在位置を出力する。デフォルトは False。

        Returns
        -------
        PoseStamped or list of float
            simple=False の場合はマップ座標系基準の PoseStamped。
            simple=True の場合は [x, y, yaw] を格納したリスト。
        """
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            try:
                transform = self.tf_buffer.lookup_transform(
                    "map", "base_link", rclpy.time.Time()
                )

                if simple:
                    x = transform.transform.translation.x
                    y = transform.transform.translation.y
                    q = transform.transform.rotation
                    (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
                    return [x, y, yaw]
                else:
                    pose = PoseStamped()
                    pose.header = transform.header
                    pose.pose.position.x = transform.transform.translation.x
                    pose.pose.position.y = transform.transform.translation.y
                    pose.pose.position.z = transform.transform.translation.z
                    pose.pose.orientation = transform.transform.rotation
                    return pose
            except Exception as e:
                self.node.get_logger().debug(f"TF Lookup failed: {str(e)}")
                continue

    def __get_pose_yaw(self, pose: PoseStamped) -> float:
        q = pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return yaw

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
            math.atan2(dy, dx)
            if distance > 1e-3
            else self.__get_pose_yaw(goal_pose)
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
        tolerance: float = 0.5,
        reference_frame: str = "map",
        wait: bool = True,
        timeout: float = None,
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

                goal_pose = tf2_geometry_msgs.do_transform_pose_stamped(goal_pose, transform)
            except Exception as e:
                self.node.get_logger().error(
                    f"Failed to transform pose to map frame: {str(e)}"
                )
                return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        self.__debug_goal_pub.publish(goal_pose)

        future = self.__action_client.send_goal_async(goal_msg)

        if not wait:
            # 非同期モードの場合は送信完了まで少し待機して終了とする
            try:
                rclpy.spin_until_future_complete(self.node, future, timeout_sec=0.5)
            except KeyboardInterrupt:
                pass
            return True

        # 同期モード (wait=True)
        try:
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
            if not future.done():
                self.node.get_logger().error("Send goal timed out")
                return False

            goal_handle = future.result()
            self.__current_goal_handle = goal_handle

            if not goal_handle.accepted:
                self.node.get_logger().error("Goal rejected by server")
                return False

            result_future = goal_handle.get_result_async()

            nav_success = False
            timeout_sec = self.TIMEOUT_SEC if timeout is None else timeout
            start_time = time.time()
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
        tolerance: float = 0.5,
        reference_frame: str = "map",
        wait: bool = True,
        timeout: float = None,
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

        Returns
        -------
        bool
            ナビゲーションが成功した場合は True、失敗・キャンセルされた場合は False。
        """
        pose = PoseStamped()
        pose.header.frame_id = reference_frame
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
            pose, tolerance=tolerance, reference_frame=reference_frame, wait=wait, timeout=timeout
        )

    def move_rel(
        self,
        x: float = 0.0,
        y: float = 0.0,
        yaw: float = 0.0,
        tolerance: float = 0.5,
        wait: bool = True,
        timeout: float = None,
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

        Returns
        -------
        bool
            ナビゲーションが成功した場合は True、失敗・キャンセルされた場合は False。
        """
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
        )

    def set_initialpose(self, pose, reference_frame: str = "map", xyy: bool = True):
        """
        ロボットの初期位置（Initial Pose）を設定する．
        AMCL等のローカライゼーションノードに対して /initialpose トピックをパブリッシュする。

        Parameters
        ----------
        pose : list of float or PoseWithCovarianceStamped
            xyy=True の場合は [x, y, yaw] の形式。
            xyy=False の場合は PoseWithCovarianceStamped 形式。
        reference_frame : str, optional
            基準となる座標フレーム。デフォルトは 'map'。
        xyy : bool, optional
            True の場合、pose を [x, y, yaw] として扱う。False の場合、pose を PoseWithCovarianceStamped として扱う。

        Returns
        -------
        None
        """
        if xyy:
            if not (isinstance(pose, list) and len(pose) == 3):
                self.node.get_logger().error(
                    "Invalid pose format for set_initialpose. Use [x, y, yaw] when xyy=True."
                )
                return

            msg = PoseWithCovarianceStamped()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = reference_frame
            msg.pose.pose.position.x = float(pose[0])
            msg.pose.pose.position.y = float(pose[1])
            # TODO: 変数として受け取るような仕様のほうがいいかも？
            # 1.3 is unitree g1 lidar height from ground level
            msg.pose.pose.position.z = 1.0

            q = quaternion_from_euler(0, 0, pose[2])
            msg.pose.pose.orientation.x = q[0]
            msg.pose.pose.orientation.y = q[1]
            msg.pose.pose.orientation.z = q[2]
            msg.pose.pose.orientation.w = q[3]

            # Covariance - typical reasonable defaults for a manual reset
            msg.pose.covariance[0] = 0.25
            msg.pose.covariance[7] = 0.25
            msg.pose.covariance[35] = 0.06853891945200942
        else:
            if not isinstance(pose, PoseWithCovarianceStamped):
                self.node.get_logger().error(
                    "Invalid pose format for set_initialpose. Use PoseWithCovarianceStamped when xyy=False."
                )
                return

            msg = pose

        self.__initial_pose_pub.publish(msg)
        self.node.get_logger().info(
            f"Published initial pose to /initialpose in frame: {msg.header.frame_id}"
        )


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
        co = self._create_collision_object(
            name, ref, x, y, z, roll, pitch, yaw, SolidPrimitive.CYLINDER, [height, radius]
        )
        self._publish_scene(co)

    def remove_near_objects(self, x: float, y: float, z: float, radius: float = 0.05):
        """指定された座標の近くにあるすべてのオブジェクトを削除する。"""
        if not self.__get_scene_cli.wait_for_service(timeout_sec=1.0):
            return
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY | PlanningSceneComponents.WORLD_OBJECT_NAMES
        future = self.__get_scene_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
        if not future.done():
            return
        
        res = future.result()
        removed_count = 0
        for co in res.scene.world.collision_objects:
            # global_pose check
            gp = co.pose.position
            dist = math.sqrt((x - gp.x)**2 + (y - gp.y)**2 + (z - gp.z)**2)
            if dist < radius:
                self.remove_collision(co.id)
                removed_count += 1
        
        if removed_count > 0:
            self.node.get_logger().info(f"Removed {removed_count} near objects (ghosts) around ({x:.3f}, {y:.3f}, {z:.3f})")

    def remove_collision(self, name: str):
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
        if not self.__get_scene_cli.wait_for_service(timeout_sec=1.0):
            return None
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY | PlanningSceneComponents.WORLD_OBJECT_NAMES
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
                    r, pt, y = euler_from_quaternion([global_pose.orientation.x, global_pose.orientation.y, global_pose.orientation.z, global_pose.orientation.w])
                    return (fx, fy, fz, r, pt, y)
        return None

    def get_object(self, name: str):
        if not self.__get_scene_cli.wait_for_service(timeout_sec=1.0):
            return None
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY | PlanningSceneComponents.WORLD_OBJECT_NAMES
        future = self.__get_scene_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
        if not future.done():
            return None
        res = future.result()
        for co in res.scene.world.collision_objects:
            if co.id == name:
                return co
        return None

    def attach_collision(self, name: str, link_name: str, touch_links: list = None, collision_object: CollisionObject = None):
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
        self.node.get_logger().info(f"Cleared {len(scene.world.collision_objects)} objects from planning scene.")


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
            self.arm.node.get_logger().error(f"Object {target_name} not found in planning scene")
            return False

        ox, oy, oz, _, _, _ = pose
        self.arm.node.get_logger().info(f"Grasp target: {target_name} at {ox:.3f}, {oy:.3f}, {oz:.3f}")

        # 2. 腕の選択
        if not arm:
            arm = "arm_right" if oy < 0 else "arm_left"
        tip_link = "right_amazing_hand" if arm == "arm_right" else "left_amazing_hand"
        hand_side = "right" if arm == "arm_right" else "left"

        # 3. アプローチ方向の計算 (Robot-to-Object)
        dist_xy = math.sqrt(ox**2 + oy**2)
        nx, ny = (ox/dist_xy, oy/dist_xy) if dist_xy > 1e-6 else (1.0, 0.0)
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
            self.arm.node.get_logger().info(f"Step C: Final grasp move to {tx:.3f}, {ty:.3f}, {tz:.3f}")
            if not self.arm.move_abs(tx, ty, tz, roll, pitch, yaw, planning_group=arm, tip_link=tip_link, orientation_tolerance=0.5):
                # 失敗したらコリジョンを戻して次へ
                self.arm.node.get_logger().warn(f"Final grasp failed at pitch {pitch:.2f}, trying next...")
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
            self.node.get_logger().error("eR@sers G1 Service Servers are not running ...")
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
    
    def enable_upper_body_control(self, enable:bool=True):
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
            rclpy.ok()
            and (self.node.get_clock().now() - start_time).nanoseconds < 2e9
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
        self, pose, planning_group: str = "upper_body", wait: bool = True, tip_link: str = None, **kwargs
    ) -> bool:
        """
        与えられた目標姿勢に向けてエンドエフェクタを自律移動させる．
        """
        # Determine tip link (assuming standard names for G1)
        if tip_link is None:
            tip_link = (
                "left_amazing_hand" if "left" in planning_group else "right_amazing_hand"
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
        self.node.get_logger().warn(f"Trying position-only planning for {planning_group}")
        goal_msg.request.goal_constraints[0].orientation_constraints = []
        return self._send_move_group_goal(goal_msg, wait)

    def place(self, x: float, y: float, z: float, planning_group: str = "arm_right", wait: bool = True) -> bool:
        """
        指定された位置に物体を配置する（Position Constraintのみを使用）。
        """
        self.node.get_logger().info(f"Placing object at {x:.3f}, {y:.3f}, {z:.3f}")
        
        tip_link = "left_amazing_hand" if "left" in planning_group else "right_amazing_hand"
        
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
        box.dimensions = [0.05, 0.05, 0.05] # 5cm tolerance (Relaxed for G1 reliability)
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

    def _solve_ik(self, pose_stamped: PoseStamped, group_name: str, tip_link: str = None) -> dict:
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
            req.ik_request.avoid_collisions = False # RELAXED: Handle collisions in planning

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
        req.ik_request.timeout.sec = 2 # Increase to 2s
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
            self.node.get_logger().error(
                f"IK service call timed out for {group_name}"
            )
        return None

        if res:
            self.node.get_logger().error(
                f"IK failed for {group_name} with error: {res.error_code.val}"
            )
        else:
            self.node.get_logger().error(
                f"IK service call timed out for {group_name}"
            )
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
        定義済み状態への遷移．
        """
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
        ジョイント角度制御．
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
            self.node.get_logger().error(
                "mic_server (mic_rec service) is not running."
            )
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
