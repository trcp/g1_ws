"""Unitree G1 ロボット制御のための高水準 Python API スイート．

ROS 2 インターフェース（Topic / Service / Action）をラップし，
上位のタスクスクリプトからロボットを直感的かつ安全に制御する機能を提供します．
"""

import numpy as np

try:
    np.float = float
except AttributeError:
    pass

# rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.action.client import ClientGoalHandle
from rclpy.action import ActionClient
from rclpy.node import Node
import rclpy

# ROS 2 interfaces
from erasers_g1_interfaces.srv import RobotServiceClient, RobotPose, MoveServo, ArmAction
from moveit_msgs.msg import (
    PlanningScene,
    CollisionObject,
    AttachedCollisionObject,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    PositionConstraint,
    OrientationConstraint,
    PlanningSceneComponents,
    AllowedCollisionMatrix,
    AllowedCollisionEntry,
    RobotState,
)
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, Twist
from moveit_msgs.srv import (
    ApplyPlanningScene, GetPlanningScene, GetPositionIK, GetPositionFK, GetCartesianPath,
)
from action_msgs.msg import GoalStatus
from shape_msgs.msg import SolidPrimitive
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import JointState
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int16MultiArray
from std_srvs.srv import SetBool
from std_msgs.msg import Int32

# API
from rclpy_util.util import TemporarySubscriber

# General
from tf_transformations import (
    quaternion_from_euler,
    euler_from_quaternion,
    quaternion_matrix,
    quaternion_from_matrix,
)
from ament_index_python.packages import get_package_share_directory
from typing import Optional, Union, List, Dict, Any, Tuple
from tf2_ros import TransformListener, Buffer
from tf2_ros import TransformException
import xml.etree.ElementTree as ET
import numpy as np
import threading
import math
import copy
import wave
import time


def _quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """クォータニオンからヨー角（rad）を算出する内部ヘルパー関数．"""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def _pose_matrix(pose: Pose) -> np.ndarray:
    """Pose を同次変換にする．未設定の四元数は単位回転として扱う．"""
    q = pose.orientation
    matrix = quaternion_matrix([q.x, q.y, q.z, q.w])
    matrix[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return matrix


def _matrix_pose(matrix: np.ndarray) -> Pose:
    """同次変換を Pose にする．"""
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = map(float, matrix[:3, 3])
    q = quaternion_from_matrix(matrix)
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, q)
    return pose


def _object_reference_pose(obj: CollisionObject) -> Pose:
    """先頭形状の中心を把持・配置の基準とする．複合形状の相対配置は維持する．"""
    matrix = _pose_matrix(obj.pose)
    poses = obj.primitive_poses or obj.mesh_poses or obj.plane_poses
    if poses:
        matrix = matrix @ _pose_matrix(poses[0])
    return _matrix_pose(matrix)


def _positive_y_path_valid(
    poses: List[np.ndarray], start: np.ndarray, distance: float,
    angle_tolerance: float = 0.1, position_tolerance: float = 0.005,
) -> bool:
    """順運動学で得た接近経路の方向，逆走，直線性，到達位置を確認する．"""
    if not poses or not math.isfinite(distance) or distance <= 0.0:
        return False
    axis, origin = start[:3, 1], start[:3, 3]
    cosine = math.cos(angle_tolerance)
    previous = start
    previous_progress = 0.0
    for pose in poses:
        if not np.isfinite(pose).all() or float(pose[:3, 1] @ axis) < cosine:
            return False
        displacement = pose[:3, 3] - origin
        progress = float(displacement @ axis)
        if (progress < previous_progress - 1e-5 or progress > distance + position_tolerance
                or np.linalg.norm(displacement - progress * axis) > position_tolerance):
            return False
        step = pose[:3, 3] - previous[:3, 3]
        length = np.linalg.norm(step)
        if length > 1e-6:
            direction = step / length
            if min(float(direction @ previous[:3, 1]), float(direction @ pose[:3, 1])) < cosine:
                return False
        previous, previous_progress = pose, progress
    return bool(np.linalg.norm(poses[-1][:3, 3] - origin - distance * axis)
                <= position_tolerance)


def _sample_joint_trajectory(trajectory) -> Optional[List[List[float]]]:
    """位置・速度・加速度に対応する補間を 10 ms 以下で検査用に標本化する．"""
    points, count = trajectory.points, len(trajectory.joint_names)
    if count == 0 or len(points) < 2 or len(set(trajectory.joint_names)) != count:
        return None
    if any(len(p.positions) != count or len(p.velocities) not in (0, count)
           or len(p.accelerations) not in (0, count) for p in points):
        return None
    if (points[0].time_from_start.sec != 0 or points[0].time_from_start.nanosec != 0
            or any(not np.isfinite(list(p.positions) + list(p.velocities)
                                   + list(p.accelerations)).all() for p in points)
            or len({len(p.velocities) for p in points}) != 1
            or len({len(p.accelerations) for p in points}) != 1):
        return None
    samples = [list(points[0].positions)]
    for first, last in zip(points, points[1:]):
        duration = ((last.time_from_start.sec - first.time_from_start.sec)
                    + (last.time_from_start.nanosec - first.time_from_start.nanosec) * 1e-9)
        p0, p1 = np.array(first.positions), np.array(last.positions)
        if duration <= 0 or not math.isfinite(duration) or np.max(np.abs(p1 - p0)) > 0.25:
            return None
        steps = max(1, math.ceil(duration / 0.01), math.ceil(np.max(np.abs(p1 - p0)) / 0.01))
        if len(samples) + steps > 2000:
            return None
        if first.velocities and last.velocities:
            v0, v1 = np.array(first.velocities) * duration, np.array(last.velocities) * duration
            if first.accelerations and last.accelerations:
                a0 = np.array(first.accelerations) * duration**2
                a1 = np.array(last.accelerations) * duration**2
                delta, velocity, acceleration = p1 - p0 - v0 - a0 / 2, v1 - v0 - a0, a1 - a0
                coefficients = [p0, v0, a0 / 2, 10 * delta - 4 * velocity + acceleration / 2,
                                -15 * delta + 7 * velocity - acceleration,
                                6 * delta - 3 * velocity + acceleration / 2]
            else:
                coefficients = [p0, v0, 3 * (p1 - p0) - 2 * v0 - v1,
                                -2 * (p1 - p0) + v0 + v1]
        elif first.accelerations or last.accelerations:
            return None
        else:
            coefficients = [p0, p1 - p0]
        for step in range(1, steps + 1):
            ratio = step / steps
            value = sum(c * ratio**power for power, c in enumerate(coefficients))
            if not np.isfinite(value).all():
                return None
            samples.append(value.tolist())
    return samples


class G1Control:
    """Unitree G1 の基本状態およびシステム設定を制御する API クラス．

    Parameters
    ----------
    node : Node
        ROS 2 ノードインスタンス．
    timeout_sec : float, default 5.0
        通信待機のデフォルトタイムアウト秒数．

    Methods
    -------
    robot_pose(mode: int, safety: bool = True) -> bool
        ロボットの FSM 姿勢遷移を要求する．
    robot_service_interface_setting(name: str, enable: bool) -> bool
        指定されたロボット内部サービスの有効化・無効化を切り替える．
    get_robot_service_interfaces() -> List[str]
        現在ロボットで有効・登録されているサービス名一覧を取得する．
    get_current_robot_pose() -> int
        現在のロボットの FSM_ID を取得する．
    led(r: int, g: int, b: int) -> bool
        ロボット頭部の RGB LED の発色を変更する．
    """

    def __init__(self, node: Node, timeout_sec: float = 5.0) -> None:
        """API クラスのインスタンスを初期化する．

        Parameters
        ----------
        node : Node
            ROS 2 ノードインスタンス．
        timeout_sec : float, default 5.0
            通信待機のデフォルトタイムアウト秒数．

        Raises
        ------
        RuntimeError
            指定時間内に必須サービスサーバーが検出されなかった場合．
        """
        self.__node = node
        self.__logger = node.get_logger()
        self.__timeout_sec = timeout_sec

        self.__cb_group = MutuallyExclusiveCallbackGroup()

        self.__led_pub = self.__node.create_publisher(
            Int32MultiArray,
            "/robot_controller/led",
            10,
            callback_group=self.__cb_group,
        )

        self.__robot_pose_cli = self.__node.create_client(
            RobotPose,
            "/robot_pose",
            callback_group=self.__cb_group,
        )

        self.__service_setting_cli = self.__node.create_client(
            RobotServiceClient,
            "/robot_service_interface",
            callback_group=self.__cb_group,
        )

        while (
            not self.__robot_pose_cli.wait_for_service(timeout_sec=self.__timeout_sec)
            or not self.__service_setting_cli.wait_for_service(timeout_sec=self.__timeout_sec)
        ):
            self.__logger.error("Required services (/robot_pose, /robot_service_interface) are not available.")
            #raise RuntimeError("Failed to connect to required ROS 2 services for G1Control.")


    def __send_robot_pose_req(self, req: RobotPose.Request) -> bool:
        """RobotPose リクエストを同期送信し成否を判定する内部メソッド．"""
        future = self.__robot_pose_cli.call_async(req)
        rclpy.spin_until_future_complete(self.__node, future, timeout_sec=self.__timeout_sec)

        if future.done():
            res = future.result()
            if res is not None and res.success:
                return True
            self.__logger.warn(f"RobotPose request was rejected: {res.message if res else 'None'}")
            return False

        self.__logger.error("Timeout waiting for RobotPose service response.")
        return False


    def __send_service_setting_req(
        self, req: RobotServiceClient.Request
    ) -> Optional[RobotServiceClient.Response]:
        """RobotServiceClient リクエストを同期送信する内部メソッド．"""
        future = self.__service_setting_cli.call_async(req)
        rclpy.spin_until_future_complete(self.__node, future, timeout_sec=self.__timeout_sec)

        if future.done():
            return future.result()

        self.__logger.error("Timeout waiting for RobotServiceClient service response.")
        return None


    def robot_pose(self, mode: int, safety: bool = True) -> bool:
        """ロボットの姿勢（FSM モード）遷移を要求する．

        Parameters
        ----------
        mode : int
            目標姿勢モード番号（RobotPose.srv の定数を指定）．
        safety : bool, default True
            安全ガードを有効にするかどうか．有効時，走行中などの危険な状態からの
            脱力・着座・横臥への急激な遷移を拒否する．

        Returns
        -------
        bool
            姿勢遷移が正常に受理され完了した場合は True，それ以外は False．
        """
        if safety:
            current_fsm = self.get_current_robot_pose()
            # 走行中・移動中 FSM からの急停止・脱力・着座・横臥要求をブロック
            if current_fsm in (801, 501, 500) and mode in (0, 1, 2, 3, 5):
                self.__logger.warn(
                    f"Safety guard rejected transition: cannot transition to mode {mode} "
                    f"from active motion FSM {current_fsm}."
                )
                return False

        req = RobotPose.Request()
        req.mode = int(mode)
        return self.__send_robot_pose_req(req)


    def robot_service_interface_setting(self, name: str, enable: bool) -> bool:
        """指定された Unitree ロボット内部サービスの有効化・無効化を設定する．

        Parameters
        ----------
        name : str
            制御対象のサービス識別名．
        enable : bool
            有効化する場合は True，無効化する場合は False．

        Returns
        -------
        bool
            設定要求が成功した場合は True，それ以外は False．
        """
        req = RobotServiceClient.Request()
        req.name = str(name)
        req.enable = bool(enable)
        res = self.__send_service_setting_req(req)
        if res is not None and res.success:
            return True
        self.__logger.warn(
            f"Robot service setting for '{name}' failed: {res.message if res else 'None'}"
        )
        return False


    def get_robot_service_interfaces(self) -> List[str]:
        """ロボットで利用可能なサービス名の一覧を取得する．

        Returns
        -------
        List[str]
            登録されているサービス名の文字列リスト．取得失敗時は空リスト．
        """
        req = RobotServiceClient.Request()
        req.name = "NONE"
        req.enable = False
        res = self.__send_service_setting_req(req)
        if res is None or not res.success:
            self.__logger.warn("Failed to retrieve robot service interface list.")
            return []

        return [s.strip() for s in res.message.splitlines() if s.strip()]


    def get_current_robot_pose(self) -> int:
        """ロボットの現在の FSM_ID を取得する．

        Returns
        -------
        int
            現在の FSM_ID（整数値）．取得できなかった場合は -1．
        """
        latest_fsm: Optional[int] = None

        def __cb_fsm(msg: Int32) -> None:
            nonlocal latest_fsm
            latest_fsm = msg.data

        transient_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        with TemporarySubscriber(
            node=self.__node,
            msg=Int32,
            topic="/robot_controller/fsm_id",
            qos_profile=transient_qos,
            cb=__cb_fsm,
        ):
            start_time = time.monotonic()
            while rclpy.ok() and latest_fsm is None:
                if time.monotonic() - start_time > self.__timeout_sec:
                    self.__logger.warn("Timeout waiting for /robot_controller/fsm_id topic.")
                    break
                rclpy.spin_once(self.__node, timeout_sec=0.05)

        return latest_fsm if latest_fsm is not None else -1


    def led(self, r: int, g: int, b: int) -> bool:
        """ロボット頭部の RGB LED の発色を変更する．

        Parameters
        ----------
        r : int
            赤色の輝度値（0 〜 255）．
        g : int
            緑色の輝度値（0 〜 255）．
        b : int
            青色の輝度値（0 〜 255）．

        Returns
        -------
        bool
            送信に成功した場合は True，値が範囲外の場合は False．
        """
        for val, name in ((r, "R"), (g, "G"), (b, "B")):
            if not (0 <= val <= 255):
                self.__logger.error(f"LED {name} value out of range [0, 255]: {val}")
                return False

        msg = Int32MultiArray()
        msg.data = [int(r), int(g), int(b)]
        self.__led_pub.publish(msg)
        return True


class HeadControl:
    """頭部カメラのパン・チルトサーボモーターおよび視線追従を制御する API クラス．

    Parameters
    ----------
    node : Node
        ROS 2 ノードインスタンス．
    timeout_sec : float, default 5.0
        通信待機のデフォルトタイムアウト秒数．
    tf_buffer : Buffer, optional
        TF2 バッファインスタンス．未指定時は新規作成．

    Methods
    -------
    move_to_pose(pan: float = 0.0, tilt: float = 0.0, wait: bool = True) -> bool
        指定した角度へ頭部カメラを回転させる．
    move_to_vel(pan: float = 0.0, tilt: float = 0.0, time_sec: float = 0.1) -> bool
        指定した角速度で頭部サーボを駆動する．
    get_current_pose() -> List[float]
        現在の頭部サーボ角度 [pan, tilt] を取得する．
    gaze(target: Union[str, PoseStamped], wait: bool = True) -> bool
        指定されたリンクまたは目標座標の方向を頭部カメラで注視する．
    """

    def __init__(
        self,
        node: Node,
        use_sim_time: bool=False,
        timeout_sec: float = 5.0,
        tf_buffer: Optional[Buffer] = None,
    ) -> None:
        """API クラスのインスタンスを初期化する．

        Parameters
        ----------
        node : Node
            ROS 2 ノードインスタンス．
        timeout_sec : float, default 5.0
            通信待機のデフォルトタイムアウト秒数．
        tf_buffer : Buffer, optional
            TF2 バッファインスタンス．

        Raises
        ------
        RuntimeError
            指定時間内に /move_servo サービスが検出されなかった場合．
        """
        self.__node = node
        self.__logger = node.get_logger()
        self.__timeout_sec = timeout_sec

        self.__cb_group = MutuallyExclusiveCallbackGroup()

        self.__servo_cli = self.__node.create_client(
            MoveServo,
            "/move_servo",
            callback_group=self.__cb_group,
        )

        self.__tf_buffer = tf_buffer or Buffer()
        self.__tf_listener = TransformListener(self.__tf_buffer, self.__node)

        while not self.__servo_cli.wait_for_service(timeout_sec=self.__timeout_sec):
            self.__logger.error("Required service /move_servo is not available.")
            #raise RuntimeError("Failed to connect to /move_servo service for HeadControl.")


    def __send_servo_req(self, req: MoveServo.Request) -> bool:
        """MoveServo リクエストを同期送信する内部メソッド．"""
        future = self.__servo_cli.call_async(req)
        rclpy.spin_until_future_complete(self.__node, future, timeout_sec=self.__timeout_sec)

        if future.done():
            res = future.result()
            if res is not None and res.success:
                return True
            self.__logger.warn("MoveServo request was rejected by service server.")
            return False

        self.__logger.error("Timeout waiting for MoveServo service response.")
        return False


    def move_to_pose(self, pan: float = 0.0, tilt: float = 0.0, wait: bool = True) -> bool:
        """指定した角度へ頭部カメラを回転させる．

        Parameters
        ----------
        pan : float, default 0.0
            パン角度（左右，ラジアン）．
        tilt : float, default 0.0
            チルト角度（上下，ラジアン）．
        wait : bool, default True
            動作完了を待機するかどうか．

        Returns
        -------
        bool
            移動要求が正常に完了した場合は True，それ以外は False．
        """
        req = MoveServo.Request()
        req.pan = float(pan)
        req.tilt = -float(tilt)

        ok = self.__send_servo_req(req)
        if not ok:
            return False

        if not wait:
            return True

        start_time = time.monotonic()
        while rclpy.ok():
            if time.monotonic() - start_time > self.__timeout_sec:
                self.__logger.warn("Timeout waiting for head servo to reach target pose.")
                return False

            cur = self.get_current_pose()
            if abs(cur[0] - pan) < 0.08 and abs(cur[1] - tilt) < 0.08:
                return True
            rclpy.spin_once(self.__node, timeout_sec=0.05)

        return False


    def move_to_vel(self, pan: float = 0.0, tilt: float = 0.0, time_sec: float = 0.1) -> bool:
        """指定した角速度で頭部サーボを駆動する．

        Parameters
        ----------
        pan : float, default 0.0
            パン方向の目標角速度（rad/s）．
        tilt : float, default 0.0
            チルト方向の目標角速度（rad/s）．
        time_sec : float, default 0.1
            指令反映時間（秒）．

        Returns
        -------
        bool
            移動指令が正常に発行された場合は True，それ以外は False．
        """
        cur = self.get_current_pose()
        target_pan = cur[0] + float(pan) * float(time_sec)
        target_tilt = cur[1] + float(tilt) * float(time_sec)
        return self.move_to_pose(pan=target_pan, tilt=target_tilt, wait=False)


    def get_current_pose(self) -> List[float]:
        """現在の頭部サーボ角度 [pan, tilt] を取得する．

        Returns
        -------
        List[float]
            現在の [pan, tilt] 角度（ラジアン）．取得失敗時は [0.0, 0.0]．
        """
        angles: Optional[List[float]] = None

        def __cb_joint(msg: JointState) -> None:
            nonlocal angles
            pan_val = 0.0
            tilt_val = 0.0
            found_pan = False
            found_tilt = False
            for name, pos in zip(msg.name, msg.position):
                if name == "xl330_joint":
                    pan_val = pos
                    found_pan = True
                elif name == "d455_joint":
                    tilt_val = -pos
                    found_tilt = True
            if found_pan or found_tilt:
                angles = [pan_val, tilt_val]

        with TemporarySubscriber(
            node=self.__node,
            msg=JointState,
            topic="/joint_states",
            qos_profile=10,
            cb=__cb_joint,
        ):
            start_time = time.monotonic()
            while rclpy.ok() and angles is None:
                if time.monotonic() - start_time > self.__timeout_sec:
                    self.__logger.warn("Timeout waiting for /joint_states for head pose.")
                    break
                rclpy.spin_once(self.__node, timeout_sec=0.05)

        return angles if angles is not None else [0.0, 0.0]


    def gaze(self, target: Union[str, PoseStamped], wait: bool = True) -> bool:
        """指定されたリンクまたは目標座標の方向を頭部カメラで注視する．

        Parameters
        ----------
        target : Union[str, PoseStamped]
            注視対象．TF フレーム名（文字列）または PoseStamped 目標位置．
        wait : bool, default True
            注視完了を待機するかどうか．

        Returns
        -------
        bool
            注視制御に成功した場合は True，それ以外は False．
        """
        ref_frame = "head_servo_link"
        dx: float = 0.0
        dy: float = 0.0
        dz: float = 0.0

        try:
            if isinstance(target, str):
                tf_stamped = self.__tf_buffer.lookup_transform(
                    ref_frame,
                    target,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=2.0),
                )
                dx = tf_stamped.transform.translation.x
                dy = tf_stamped.transform.translation.y
                dz = tf_stamped.transform.translation.z
            elif isinstance(target, PoseStamped):
                target_in_ref = self.__tf_buffer.transform(
                    target,
                    ref_frame,
                    timeout=rclpy.duration.Duration(seconds=2.0),
                )
                dx = target_in_ref.pose.position.x
                dy = target_in_ref.pose.position.y
                dz = target_in_ref.pose.position.z
            else:
                self.__logger.error("Target must be a frame name (str) or PoseStamped.")
                return False
        except TransformException as exc:
            self.__logger.error(f"TF transformation error during gaze: {exc}")
            return False

        dist_xy = math.hypot(dx, dy)
        if dist_xy < 1e-4 and abs(dz) < 1e-4:
            self.__logger.warn("Target is at head origin; cannot compute gaze angle.")
            return False

        target_pan = math.atan2(dy, dx)
        target_tilt = -math.atan2(dz, dist_xy)

        return self.move_to_pose(pan=target_pan, tilt=target_tilt, wait=wait)


class ArmCollision:
    """MoveIt PlanningScene に対する干渉オブジェクトの登録・削除を管理する API クラス．

    Parameters
    ----------
    node : Node
        ROS 2 ノードインスタンス．
    timeout_sec : float, default 5.0
        通信待機のデフォルトタイムアウト秒数．
    apply_scene_cli : Any, optional
        後方互換性のための非推奨引数（省略時は内部で自動生成）．

    Methods
    -------
    get_object(name: str) -> Optional[CollisionObject]
        指定名のコリジョンオブジェクトを取得する．
    get_object_pose(name: str) -> Optional[Tuple[float, float, float, float, float, float, str]]
        指定オブジェクトの 6DoF ポーズ (x, y, z, roll, pitch, yaw, frame_id) を取得する．
    get_attached_object(name: str) -> Optional[AttachedCollisionObject]
        指定オブジェクトのアタッチ状態を取得する．
    allow_collision(name: str, support_name: str) -> bool
        ACM (AllowedCollisionMatrix) を更新し特定オブジェクト・リンク間の衝突判定を無効化する．
    add_box(...) -> bool
        直方体の干渉オブジェクトを登録する．
    add_cylinder(...) -> bool
        円柱の干渉オブジェクトを登録する．
    add_sphere(...) -> bool
        球体の干渉オブジェクトを登録する．
    attach(name: str, attach_frame: str = 'left_amazing_hand', touch_links: Optional[List[str]] = None, ...) -> bool
        指定した干渉オブジェクトを指定リンクへ把持結合（アタッチ）する．
    detach(name: str, attach_frame: Optional[str] = None) -> bool
        把持中の干渉オブジェクトを解放（デタッチ）する．
    remove_collision(name: str) -> bool
        指定名の干渉オブジェクトを削除する．
    remove_all_collisions() -> bool / all_remove_collisions() -> bool
        すべての干渉オブジェクトを削除する．
    """

    def __init__(
        self,
        node: Node,
        timeout_sec: float = 5.0,
        apply_scene_cli: Optional[Any] = None,
    ) -> None:
        """干渉オブジェクト管理クラスのインスタンスを初期化する．"""
        self.__node = node
        self.__logger = node.get_logger()
        self.__cb_group = MutuallyExclusiveCallbackGroup()

        # 第2引数に apply_scene_cli が位置引数で渡された場合の後方互換対応
        if not isinstance(timeout_sec, (int, float)):
            actual_cli = timeout_sec
            actual_timeout = 5.0 if apply_scene_cli is None else float(apply_scene_cli)
        else:
            actual_cli = apply_scene_cli
            actual_timeout = float(timeout_sec)

        self.__timeout_sec = actual_timeout
        if actual_cli is not None:
            self.__apply_scene_cli = actual_cli
        else:
            self.__apply_scene_cli = self.__node.create_client(
                ApplyPlanningScene,
                "/apply_planning_scene",
                callback_group=self.__cb_group,
            )

        self.__scene_pub = self.__node.create_publisher(
            PlanningScene,
            "/planning_scene",
            10,
            callback_group=self.__cb_group,
        )
        self.__get_scene_cli = self.__node.create_client(
            GetPlanningScene,
            "/get_planning_scene",
            callback_group=self.__cb_group,
        )
        self.__registered_objects: List[str] = []
        self.__attached_objects: List[str] = []


    def __apply(self, scene: PlanningScene) -> bool:
        """PlanningScene を非同期送信し反映完了を待機する内部メソッド．"""
        scene.is_diff = True
        scene.robot_state.is_diff = True

        if self.__apply_scene_cli.wait_for_service(timeout_sec=self.__timeout_sec):
            req = ApplyPlanningScene.Request()
            req.scene = scene

            future = self.__apply_scene_cli.call_async(req)
            rclpy.spin_until_future_complete(self.__node, future, timeout_sec=self.__timeout_sec)

            if future.done():
                res = future.result()
                if res is not None and res.success:
                    return True
                self.__logger.warn("ApplyPlanningScene service rejected scene update.")
                return False

            future.cancel()
        self.__logger.error("PlanningScene の更新完了を確認できませんでした．")
        return False


    def get_object(self, name: str) -> Optional[CollisionObject]:
        """指定された名前のコリジョンオブジェクト全体を取得する．

        Parameters
        ----------
        name : str
            オブジェクト名．

        Returns
        -------
        Optional[CollisionObject]
            見つかった場合は CollisionObject，存在しない場合は None．
        """
        if not self.__get_scene_cli.wait_for_service(timeout_sec=self.__timeout_sec):
            self.__logger.warn("GetPlanningScene service unavailable.")
            return None

        req = GetPlanningScene.Request()
        req.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            | PlanningSceneComponents.WORLD_OBJECT_NAMES
        )
        future = self.__get_scene_cli.call_async(req)
        rclpy.spin_until_future_complete(self.__node, future, timeout_sec=self.__timeout_sec)

        if not future.done() or future.result() is None:
            return None

        for co in future.result().scene.world.collision_objects:
            if co.id == str(name):
                return co
        return None


    def get_object_pose(
        self, name: str
    ) -> Optional[Tuple[float, float, float, float, float, float, str]]:
        """指定されたオブジェクトの 6DoF ポーズ (x, y, z, roll, pitch, yaw, frame_id) を取得する．

        Parameters
        ----------
        name : str
            オブジェクト名．

        Returns
        -------
        Optional[Tuple[float, float, float, float, float, float, str]]
            (x, y, z, roll, pitch, yaw, frame_id) のタプル．見つからない場合は None．
        """
        co = self.get_object(name)
        if co is None:
            self.__logger.warn(f"Object '{name}' not found in planning scene.")
            return None

        pose = _object_reference_pose(co)
        q = pose.orientation
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return (pose.position.x, pose.position.y, pose.position.z,
                roll, pitch, yaw, co.header.frame_id)


    def get_attached_object(
        self, name: str = "", link_name: Optional[str] = None,
    ) -> Optional[AttachedCollisionObject]:
        """指定された名前の AttachedCollisionObject を取得する．

        Parameters
        ----------
        name : str, default ""
            オブジェクト名（空文字列の場合は最初に見つかったアタッチオブジェクト）．
        link_name : Optional[str], default None
            指定時はこのリンクにアタッチされた物体だけを対象とする．

        Returns
        -------
        Optional[AttachedCollisionObject]
            見つかった場合は AttachedCollisionObject，存在しない場合は None．
        """
        if not self.__get_scene_cli.wait_for_service(timeout_sec=self.__timeout_sec):
            self.__logger.warn("GetPlanningScene service not available.")
            return None

        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS

        future = self.__get_scene_cli.call_async(req)
        rclpy.spin_until_future_complete(self.__node, future, timeout_sec=self.__timeout_sec)

        if not future.done() or future.result() is None:
            return None

        for aco in future.result().scene.robot_state.attached_collision_objects:
            if ((not name or aco.object.id == str(name))
                    and (link_name is None or aco.link_name == link_name)):
                return aco
        return None


    def get_allowed_collision_matrix(self) -> Optional[AllowedCollisionMatrix]:
        """一時的な接触許可の保存・復元に使う現在の行列を取得する．"""
        if not self.__get_scene_cli.wait_for_service(timeout_sec=self.__timeout_sec):
            return None
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        future = self.__get_scene_cli.call_async(req)
        rclpy.spin_until_future_complete(self.__node, future, timeout_sec=self.__timeout_sec)
        if not future.done() or future.result() is None:
            return None
        return copy.deepcopy(future.result().scene.allowed_collision_matrix)


    def get_robot_state(self) -> Optional[RobotState]:
        """MoveIt が認識する関節と把持物体を含む現在状態を取得する．"""
        if not self.__get_scene_cli.wait_for_service(timeout_sec=self.__timeout_sec):
            return None
        req = GetPlanningScene.Request()
        req.components.components = (
            PlanningSceneComponents.ROBOT_STATE
            | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        )
        future = self.__get_scene_cli.call_async(req)
        rclpy.spin_until_future_complete(self.__node, future, timeout_sec=self.__timeout_sec)
        if not future.done() or future.result() is None:
            return None
        return copy.deepcopy(future.result().scene.robot_state)


    def apply_allowed_collision_matrix(self, matrix: AllowedCollisionMatrix) -> bool:
        """接触許可行列の適用完了を確認する．"""
        scene = PlanningScene()
        scene.allowed_collision_matrix = matrix
        return self.__apply(scene)


    @staticmethod
    def _set_allowed_pair(acm: AllowedCollisionMatrix, name: str, other: str) -> None:
        """既存の規則と既定値を維持し，指定した組だけを接触許可にする．"""
        defaults = dict(zip(acm.default_entry_names, acm.default_entry_values))

        def default_value(a: str, b: str) -> bool:
            values = [defaults[n] for n in (a, b) if n in defaults]
            return all(values) if values else False

        for item in (name, other):
            if item not in acm.entry_names:
                for entry, existing in zip(acm.entry_values, acm.entry_names):
                    entry.enabled.append(default_value(item, existing))
                acm.entry_names.append(item)
                acm.entry_values.append(AllowedCollisionEntry(
                    enabled=[default_value(item, existing) for existing in acm.entry_names]))
        i, j = acm.entry_names.index(name), acm.entry_names.index(other)
        acm.entry_values[i].enabled[j] = True
        acm.entry_values[j].enabled[i] = True


    def allow_collision(self, name: str, support_name: str) -> bool:
        """指定された2つのオブジェクト（またはオブジェクトとロボットリンク）間の衝突判定を無効化（接触許可）する．

        Parameters
        ----------
        name : str
            オブジェクト名1．
        support_name : str
            オブジェクト名2（作業台や把持ハンドリンクなど）．

        Returns
        -------
        bool
            ACM 更新が成功した場合は True，失敗した場合は False．
        """
        acm = self.get_allowed_collision_matrix()
        if acm is None:
            return False
        self._set_allowed_pair(acm, name, support_name)
        return self.apply_allowed_collision_matrix(acm)


    def add_box(
        self,
        name: str = "box",
        ref_frame: str = "torso_link",
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
        scale_x: float = 0.1,
        scale_y: float = 0.1,
        scale_z: float = 0.1,
        ref: Optional[str] = None,
    ) -> bool:
        """直方体の干渉オブジェクトを PlanningScene に登録する．"""
        target_ref = ref if ref is not None else ref_frame
        obj = CollisionObject()
        obj.id = str(name)
        obj.header.frame_id = str(target_ref)
        obj.operation = CollisionObject.ADD

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [float(scale_x), float(scale_y), float(scale_z)]

        q = quaternion_from_euler(roll, pitch, yaw)
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        obj.primitives = [primitive]
        obj.primitive_poses = [pose]

        scene = PlanningScene()
        scene.world.collision_objects = [obj]

        ok = self.__apply(scene)
        if ok and name not in self.__registered_objects:
            self.__registered_objects.append(name)
        return ok


    def add_cylinder(
        self,
        name: str = "cylinder",
        ref_frame: str = "torso_link",
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
        radius: float = 0.05,
        height: float = 0.1,
        ref: Optional[str] = None,
    ) -> bool:
        """円柱の干渉オブジェクトを PlanningScene に登録する．"""
        target_ref = ref if ref is not None else ref_frame
        obj = CollisionObject()
        obj.id = str(name)
        obj.header.frame_id = str(target_ref)
        obj.operation = CollisionObject.ADD

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.CYLINDER
        primitive.dimensions = [float(height), float(radius)]

        q = quaternion_from_euler(roll, pitch, yaw)
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        obj.primitives = [primitive]
        obj.primitive_poses = [pose]

        scene = PlanningScene()
        scene.world.collision_objects = [obj]

        ok = self.__apply(scene)
        if ok and name not in self.__registered_objects:
            self.__registered_objects.append(name)
        return ok


    def add_sphere(
        self,
        name: str = "sphere",
        ref_frame: str = "torso_link",
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
        radius: float = 0.05,
        ref: Optional[str] = None,
    ) -> bool:
        """球体の干渉オブジェクトを PlanningScene に登録する．"""
        target_ref = ref if ref is not None else ref_frame
        obj = CollisionObject()
        obj.id = str(name)
        obj.header.frame_id = str(target_ref)
        obj.operation = CollisionObject.ADD

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [float(radius)]

        q = quaternion_from_euler(roll, pitch, yaw)
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        obj.primitives = [primitive]
        obj.primitive_poses = [pose]

        scene = PlanningScene()
        scene.world.collision_objects = [obj]

        ok = self.__apply(scene)
        if ok and name not in self.__registered_objects:
            self.__registered_objects.append(name)
        return ok


    def attach(
        self,
        name: str,
        attach_frame: str = "left_amazing_hand",
        touch_links: Optional[List[str]] = None,
        collision_object: Optional[CollisionObject] = None,
        link_name: Optional[str] = None,
    ) -> bool:
        """指定した干渉オブジェクトを指定リンクへアタッチ（把持結合）する．

        Parameters
        ----------
        name : str
            アタッチする干渉オブジェクトの識別名．
        attach_frame : str, default "left_amazing_hand"
            把持・アタッチ対象のロボットリンク名．
        touch_links : Optional[List[str]], default None
            把持対象オブジェクトとの接触を許可するリンク名のリスト．
            None の場合，attach_frame および手首リンク群が自動設定される．
        collision_object : Optional[CollisionObject], default None
            明示的なオブジェクトジオメトリ定義（指定時はワールド外でも形状を保持してアタッチ）．
        link_name : Optional[str], default None
            attach_frame の Piper 互換エイリアス．

        Returns
        -------
        bool
            適用に成功した場合は True，失敗した場合は False．
        """
        target_frame = str(link_name) if link_name is not None else str(attach_frame)

        if touch_links is None:
            touch_links = [target_frame]
            if "left" in target_frame:
                touch_links.extend([
                    "left_wrist_roll_rubber_hand",
                ])
            elif "right" in target_frame:
                touch_links.extend([
                    "right_wrist_roll_rubber_hand",
                ])

        attached_obj = AttachedCollisionObject()
        attached_obj.link_name = target_frame
        if collision_object is not None:
            attached_obj.object = copy.deepcopy(collision_object)
            attached_obj.object.id = str(name)
            attached_obj.object.operation = CollisionObject.ADD
        else:
            attached_obj.object.id = str(name)
            attached_obj.object.operation = CollisionObject.ADD
        attached_obj.touch_links = list(set(touch_links))

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [attached_obj]

        ok = self.__apply(scene)
        if ok and name not in self.__attached_objects:
            self.__attached_objects.append(name)
        return ok


    def detach(
        self,
        name: str,
        attach_frame: Optional[str] = None,
    ) -> bool:
        """指定した干渉オブジェクトをロボットリンクからデタッチ（解放）する．

        Parameters
        ----------
        name : str
            デタッチする干渉オブジェクトの識別名．
        attach_frame : Optional[str], default None
            デタッチ元のリンク名（指定がある場合）．

        Returns
        -------
        bool
            適用に成功した場合は True，失敗した場合は False．
        """
        attached_obj = AttachedCollisionObject()
        if attach_frame is not None:
            attached_obj.link_name = str(attach_frame)
        attached_obj.object.id = str(name)
        attached_obj.object.operation = CollisionObject.REMOVE

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [attached_obj]

        ok = self.__apply(scene)
        if ok and name in self.__attached_objects:
            self.__attached_objects.remove(name)
        return ok


    def attach_collision(
        self,
        name: str,
        link_name: str = "left_amazing_hand",
        touch_links: Optional[List[str]] = None,
        collision_object: Optional[CollisionObject] = None,
        attach_frame: Optional[str] = None,
    ) -> bool:
        """指定した干渉オブジェクトを指定リンクへアタッチ（把持結合）する（attach のエイリアス）．"""
        target_frame = attach_frame if attach_frame is not None else link_name
        return self.attach(
            name,
            attach_frame=target_frame,
            touch_links=touch_links,
            collision_object=collision_object,
        )


    def detach_collision(
        self,
        name: str,
        attach_frame: Optional[str] = None,
    ) -> bool:
        """指定した干渉オブジェクトをロボットリンクからデタッチ（解放）する（detach のエイリアス）．"""
        return self.detach(name, attach_frame=attach_frame)


    def all_remove_collisions(self) -> bool:
        """作成したすべてのコリジョンを削除する（Piper 命名互換．remove_all_collisions と同等）．"""
        return self.remove_all_collisions()


    def remove_collision(self, name: str) -> bool:
        """指定名の干渉オブジェクトを PlanningScene から削除する．"""
        obj = CollisionObject()
        obj.id = str(name)
        obj.operation = CollisionObject.REMOVE

        attached = AttachedCollisionObject()
        attached.object = copy.deepcopy(obj)

        scene = PlanningScene()
        scene.world.collision_objects = [obj]
        scene.robot_state.attached_collision_objects = [attached]

        ok = self.__apply(scene)
        if ok:
            if name in self.__registered_objects:
                self.__registered_objects.remove(name)
            if name in self.__attached_objects:
                self.__attached_objects.remove(name)
        return ok


    def remove_all_collisions(self) -> bool:
        """登録済みおよびアタッチ済みのすべての干渉オブジェクトを PlanningScene から削除する．"""
        all_success = True
        all_names = set(self.__registered_objects + self.__attached_objects)

        # MoveIt が動作している場合は現在の PlanningScene から全オブジェクトを自動検出
        if self.__get_scene_cli.service_is_ready():
            req = GetPlanningScene.Request()
            req.components.components = 1 | 2 | 4 | 8 | 16
            future = self.__get_scene_cli.call_async(req)
            rclpy.spin_until_future_complete(self.__node, future, timeout_sec=self.__timeout_sec)
            if future.done() and future.result() is not None:
                scene = future.result().scene
                for o in scene.world.collision_objects:
                    all_names.add(o.id)
                for o in scene.robot_state.attached_collision_objects:
                    all_names.add(o.object.id)

        for name in list(all_names):
            if not self.remove_collision(name):
                all_success = False
        return all_success


class ArmControl:
    """Unitree G1 の上半身・双腕マニピュレーションを制御する API クラス．

    Parameters
    ----------
    node : Node
        ROS 2 ノードインスタンス．
    timeout_sec : float, default 10.0
        通信待機のデフォルトタイムアウト秒数．

    Methods
    -------
    move_groupstate(group_name: str = 'arm_both_with_waist', state: str = 'walk') -> bool
        事前定義された姿勢グループステートへ遷移させる．
    move_abs(x: float, y: float, z: float, ref_frame: str = 'torso_link', group_name: str = 'arm_left_with_waist', wait: bool = True) -> bool
        指定座標系における絶対座標へエンドエフェクタを移動させる．
    move_rel(x: float, y: float, z: float, ref_frame: str = '*_left_amazing_hand', group_name: str = 'arm_left_with_waist', wait: bool = True) -> bool
        指定グループのエンドエフェクタを相対移動させる．
    joint_control(rel: bool = False, wait: bool = True, **kwargs: float) -> bool
        指定された関節を指定角度へ直接駆動する．
    get_current_joint_pose() -> Dict[str, float]
        現在の全関節名と関節角度の辞書を取得する．
    arm_action(mode: int) -> bool
        Unitree プリセット腕動作を実行する．
    upper_body_control(enable: bool) -> bool
        上半身の関節制御権限の有効・無効を切り替える．
    """

    def __init__(
        self,
        node: Node,
        timeout_sec: float = 10.0,
        tf_buffer: Optional[Buffer] = None,
        use_sim_time: Optional[bool] = None,
    ) -> None:
        """API クラスのインスタンスを初期化する．

        Parameters
        ----------
        node : Node
            ROS 2 ノードインスタンス．
        timeout_sec : float, default 10.0
            通信待機のデフォルトタイムアウト秒数．
        tf_buffer : Optional[Buffer], optional
            共有 TF2 バッファインスタンス．省略時は内部で生成．
        use_sim_time : Optional[bool], optional
            省略時は node パラメータから自動判定．明示指定時は node の時刻設定も一致させる．

        Raises
        ------
        RuntimeError
            実機環境（use_sim_time=False）において指定時間内に必須サービスが検出されなかった場合．
        """
        self.__node = node
        self.__logger = node.get_logger()
        self.__timeout_sec = timeout_sec

        if use_sim_time is not None:
            self.__use_sim_time = bool(use_sim_time)
            # フラグと TF・サービス要求の時刻を一致させる．
            if not node.has_parameter("use_sim_time"):
                node.declare_parameter("use_sim_time", self.__use_sim_time)
            result = node.set_parameters([
                rclpy.parameter.Parameter("use_sim_time", value=self.__use_sim_time)
            ])[0]
            if not result.successful:
                raise RuntimeError(f"use_sim_time を設定できません: {result.reason}")
        else:
            self.__use_sim_time = False
            try:
                if self.__node.has_parameter("use_sim_time"):
                    self.__use_sim_time = bool(self.__node.get_parameter("use_sim_time").value)
                else:
                    self.__use_sim_time = bool(
                        self.__node.declare_parameter("use_sim_time", False).value
                    )
            except Exception:
                try:
                    self.__use_sim_time = bool(self.__node.get_parameter("use_sim_time").value)
                except Exception:
                    self.__use_sim_time = False

        self.__cb_group = MutuallyExclusiveCallbackGroup()

        self.__arm_action_cli = self.__node.create_client(
            ArmAction,
            "/arm_action",
            callback_group=self.__cb_group,
        )

        self.__upper_enable_cli = self.__node.create_client(
            SetBool,
            "/enable_upper_body_control",
            callback_group=self.__cb_group,
        )
        self.__fallback_upper_enable_cli = self.__node.create_client(
            SetBool,
            "/robot_controller/upper_body/enable",
            callback_group=self.__cb_group,
        )

        self.__ik_cli = self.__node.create_client(
            GetPositionIK,
            "/compute_ik",
            callback_group=self.__cb_group,
        )

        self.__fk_cli = self.__node.create_client(
            GetPositionFK, "/compute_fk", callback_group=self.__cb_group)
        self.__cartesian_cli = self.__node.create_client(
            GetCartesianPath, "/compute_cartesian_path", callback_group=self.__cb_group)
        self.__execute_cli = ActionClient(
            self.__node, ExecuteTrajectory, "/execute_trajectory", callback_group=self.__cb_group)

        self.__move_group_cli = ActionClient(
            self.__node,
            MoveGroup,
            "/move_action",
            callback_group=self.__cb_group,
        )

        self.collision = ArmCollision(
            self.__node, timeout_sec=self.__timeout_sec
        )

        self.__tf_buffer = tf_buffer or Buffer()
        self.__tf_listener = TransformListener(self.__tf_buffer, self.__node) if tf_buffer is None else None

        if self.__use_sim_time:
            self.__logger.info(
                "ArmControl initialized with use_sim_time=True: hardware arm services are skipped."
            )
        else:
            start_wait = time.monotonic()
            while rclpy.ok():
                arm_ok = self.__arm_action_cli.wait_for_service(timeout_sec=0.1)
                enable_ok = (
                    self.__upper_enable_cli.wait_for_service(timeout_sec=0.1)
                    or self.__fallback_upper_enable_cli.wait_for_service(timeout_sec=0.1)
                )
                if arm_ok and enable_ok:
                    break
                if time.monotonic() - start_wait > self.__timeout_sec:
                    self.__logger.error("Required arm services are not available.")
                    raise RuntimeError("Failed to connect to required arm services.")
                rclpy.spin_once(self.__node, timeout_sec=0.05)

        self.__srdf_states = self.__load_srdf_group_states()


    def __load_srdf_group_states(self) -> Dict[str, Dict[str, Dict[str, float]]]:
        """SRDF ファイルから group_state の定義を読み込む内部メソッド．"""
        states: Dict[str, Dict[str, Dict[str, float]]] = {}
        try:
            pkg_path = get_package_share_directory("erasers_g1_moveit")
            srdf_path = f"{pkg_path}/config/g1.srdf"
            tree = ET.parse(srdf_path)
            root = tree.getroot()
            for gs in root.findall("group_state"):
                name = gs.get("name")
                group = gs.get("group")
                if not name or not group:
                    continue
                joint_values: Dict[str, float] = {}
                for j in gs.findall("joint"):
                    j_name = j.get("name")
                    j_val = float(j.get("value", 0.0))
                    if j_name:
                        joint_values[j_name] = j_val
                if group not in states:
                    states[group] = {}
                states[group][name] = joint_values
        except Exception as exc:
            self.__logger.warn(f"Failed to load SRDF group states: {exc}")
        return states


    def __send_arm_action_req(self, req: ArmAction.Request) -> bool:
        """ArmAction リクエストを同期送信する内部メソッド．"""
        if not self.__arm_action_cli.service_is_ready():
            self.__logger.error("ArmAction service (/arm_action) is not available.")
            return False
        future = self.__arm_action_cli.call_async(req)
        rclpy.spin_until_future_complete(self.__node, future, timeout_sec=self.__timeout_sec)

        if future.done():
            res = future.result()
            if res is not None and res.success:
                return True
            self.__logger.warn(f"ArmAction rejected: {res.message if res else 'None'}")
            return False

        self.__logger.error("Timeout waiting for ArmAction response.")
        return False


    def __send_upper_enable_req(self, req: SetBool.Request) -> bool:
        """SetBool リクエストを同期送信する内部メソッド．"""
        cli = (
            self.__upper_enable_cli
            if self.__upper_enable_cli.service_is_ready()
            else self.__fallback_upper_enable_cli
        )
        if not cli.service_is_ready():
            self.__logger.error("Upper-body enable service is not available.")
            return False
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self.__node, future, timeout_sec=self.__timeout_sec)

        if future.done():
            res = future.result()
            if res is not None and res.success:
                return True
            self.__logger.warn(f"Upper-body enable request failed: {res.message if res else 'None'}")
            return False

        self.__logger.error("Timeout waiting for upper-body enable response.")
        return False


    def __send_move_group_goal(
        self,
        goal: MoveGroup.Goal,
        wait: bool = True,
    ) -> bool:
        """MoveGroup アクションゴールを同期/非同期送信する内部メソッド．"""
        if not self.__move_group_cli.wait_for_server(timeout_sec=self.__timeout_sec):
            self.__logger.error("MoveGroup action server (/move_action) is not available.")
            return False

        future = self.__move_group_cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.__node, future, timeout_sec=self.__timeout_sec)

        if not future.done():
            self.__logger.error("Timeout waiting for MoveGroup goal response.")
            return False

        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.__logger.warn("MoveGroup goal was rejected by server.")
            return False

        if not wait:
            return True

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self.__node, result_future, timeout_sec=self.__timeout_sec * 4.0
        )

        if not result_future.done():
            self.__logger.error("Timeout waiting for MoveGroup execution result.")
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(
                self.__node, cancel_future, timeout_sec=self.__timeout_sec
            )
            rclpy.spin_until_future_complete(
                self.__node, result_future, timeout_sec=self.__timeout_sec
            )
            if not result_future.done():
                self.__logger.error("MoveGroup のキャンセル後の終了を確認できませんでした．")
            return False

        wrapped_result = result_future.result()
        if wrapped_result is None or wrapped_result.result is None:
            self.__logger.error("MoveGroup returned null result wrapper.")
            return False

        error_code = wrapped_result.result.error_code.val
        if (wrapped_result.status == GoalStatus.STATUS_SUCCEEDED
                and error_code == MoveItErrorCodes.SUCCESS):
            return True

        self.__logger.warn(f"MoveGroup execution finished with MoveIt error code: {error_code}")
        return False


    def _create_move_group_goal(
        self,
        group_name: str,
    ) -> MoveGroup.Goal:
        """MoveGroup.Goal の共通設定インスタンスを生成する内部メソッド．"""
        goal = MoveGroup.Goal()
        goal.request.group_name = group_name
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 10.0
        goal.request.max_velocity_scaling_factor = 0.5
        goal.request.max_acceleration_scaling_factor = 0.5
        goal.request.start_state.is_diff = True

        # 空の差分により MoveIt の現在状態を使う．/joint_states には
        # モデル外の関節が含まれ得るため，開始状態へそのまま転送しない．

        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 5
        return goal


    def move_groupstate(
        self,
        group_name: str = "arm_both_with_waist",
        state: str = "walk",
        wait: bool = True,
    ) -> bool:
        """SRDF に定義された名前付き姿勢グループステートへ遷移させる．

        Parameters
        ----------
        group_name : str, default 'arm_both_with_waist'
            対象の planning group 名．
        state : str, default 'walk'
            目標の group_state 名（'walk', 'home' 等）．
        wait : bool, default True
            動作完了を待機するかどうか．

        Returns
        -------
        bool
            姿勢遷移が正常に完了した場合は True，それ以外は False．
        """
        if group_name not in self.__srdf_states:
            self.__logger.error(f"Planning group '{group_name}' not found in SRDF.")
            return False

        if state not in self.__srdf_states[group_name]:
            self.__logger.error(f"State '{state}' not found for group '{group_name}'.")
            return False

        joints = self.__srdf_states[group_name][state]
        return self.joint_control(rel=False, wait=wait, planning_group=group_name, **joints)


    def get_current_endeffector_pose(
        self,
        ref: str = "torso_link",
        arm_side: str = "left",
    ) -> Optional[Any]:
        """指定された腕のエンドエフェクタ（amazing_hand）の現在位置姿勢を取得する．

        Parameters
        ----------
        ref : str, default 'torso_link'
            基準座標系名．
        arm_side : str, default 'left'
            腕の指定（'left' または 'right'，あるいは直接リンク名）．

        Returns
        -------
        Optional[geometry_msgs.msg.Transform]
            Transform メッセージ（translation, rotation を持つ）．取得失敗時は None．
        """
        link_name = arm_side if arm_side.endswith("_amazing_hand") else f"{arm_side}_amazing_hand"
        deadline = time.monotonic() + self.__timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            try:
                ts = self.__tf_buffer.lookup_transform(
                    ref,
                    link_name,
                    rclpy.time.Time(),
                )
                return ts.transform
            except TransformException:
                rclpy.spin_once(self.__node, timeout_sec=0.1)

        self.__logger.warn(f"TF lookup failed for get_current_endeffector_pose ({link_name} to {ref}).")
        return None


    def transform_pose(self, pose: Pose, source_frame: str,
                       target_frame: str = "pelvis") -> Optional[Pose]:
        """最新 TF で姿勢を変換する．腰の回転前に目標を pelvis へ固定できる．"""
        if source_frame == target_frame:
            return copy.deepcopy(pose)
        deadline = time.monotonic() + self.__timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            try:
                transform = self.__tf_buffer.lookup_transform(
                    target_frame, source_frame, rclpy.time.Time()).transform
                frame_pose = Pose()
                frame_pose.position.x = transform.translation.x
                frame_pose.position.y = transform.translation.y
                frame_pose.position.z = transform.translation.z
                frame_pose.orientation = transform.rotation
                return _matrix_pose(_pose_matrix(frame_pose) @ _pose_matrix(pose))
            except TransformException:
                rclpy.spin_once(self.__node, timeout_sec=0.05)
        self.__logger.error(f"TF を取得できません: {source_frame} -> {target_frame}")
        return None


    def _fk_pose(self, state: RobotState, link: str) -> Optional[Pose]:
        """MoveIt と同じモデルで手先姿勢を求める．モデル外の関節は追加しない．"""
        request = GetPositionFK.Request()
        request.header.frame_id = "pelvis"
        request.fk_link_names = [link]
        request.robot_state = state
        future = self.__fk_cli.call_async(request)
        rclpy.spin_until_future_complete(self.__node, future, timeout_sec=self.__timeout_sec)
        if not future.done():
            future.cancel()
            return None
        response = future.result()
        if (response is None or response.error_code.val != MoveItErrorCodes.SUCCESS
                or len(response.pose_stamped) != 1):
            return None
        return response.pose_stamped[0].pose


    def _execute_checked_trajectory(self, trajectory) -> bool:
        """検査済みの軌道を再計画せずに実行し，終了またはキャンセルを確認する．"""
        if not self.__execute_cli.wait_for_server(timeout_sec=self.__timeout_sec):
            return False
        future = self.__execute_cli.send_goal_async(ExecuteTrajectory.Goal(trajectory=trajectory))
        rclpy.spin_until_future_complete(self.__node, future, timeout_sec=self.__timeout_sec)
        if not future.done():
            # 遅れて受理されたゴールも，次のスピン時にキャンセルする．
            def cancel_late_goal(completed):
                handle = completed.result()
                if handle is not None and handle.accepted:
                    handle.cancel_goal_async()
            future.add_done_callback(cancel_late_goal)
            self.__logger.error("接近軌道の受理を確認できませんでした．")
            return False
        handle = future.result()
        if handle is None or not handle.accepted:
            return False
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(
            self.__node, result_future, timeout_sec=self.__timeout_sec * 4)
        if not result_future.done():
            cancellation = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(
                self.__node, cancellation, timeout_sec=self.__timeout_sec)
            rclpy.spin_until_future_complete(
                self.__node, result_future, timeout_sec=self.__timeout_sec)
            self.__logger.error("接近軌道が時間内に完了しませんでした．")
            return False
        result = result_future.result()
        return bool(result is not None and result.status == GoalStatus.STATUS_SUCCEEDED
                    and result.result.error_code.val == MoveItErrorCodes.SUCCESS)


    def _align_grasp_y(
        self, center: np.ndarray, distance: float, group_name: str,
    ) -> Optional[np.ndarray]:
        """物体を手先 +Y 軸上の指定距離へ置く接近前姿勢を求め，衝突判定付きで移動する．

        手先位置だけの IK は向きを変えてしまうため，手先から +Y へ距離を加えた
        点の誤差を，MoveIt の FK による減衰最小二乗法で解く．関節限界と経路の
        衝突判定は MoveGroup に委ね，計算中はロボットを動かさない．
        """
        side = self._arm_side(group_name)
        if (side is None or not np.isfinite(center).all()
                or not math.isfinite(distance) or distance <= 0.0
                or not self.__fk_cli.wait_for_service(timeout_sec=self.__timeout_sec)):
            return None
        state = self.collision.get_robot_state()
        if state is None:
            return None
        names = list(state.joint_state.name)
        group_joints = set()
        for values in self.__srdf_states.get(group_name, {}).values():
            group_joints.update(values)
        indices = [i for i, name in enumerate(names) if name in group_joints]
        if not indices or not group_joints.issubset(names):
            return None
        positions = np.array(state.joint_state.position)
        if len(positions) != len(names) or not np.isfinite(positions).all():
            return None
        link = f"{side}_amazing_hand"

        def evaluate(values):
            candidate = copy.deepcopy(state)
            candidate.joint_state.position = values.tolist()
            pose = self._fk_pose(candidate, link)
            return _pose_matrix(pose) if pose is not None else None

        for _ in range(30):
            matrix = evaluate(positions)
            if matrix is None:
                return None
            point = matrix[:3, 3] + distance * matrix[:3, 1]
            error = center - point
            if np.linalg.norm(error) <= 0.0001:
                break
            columns = []
            for index in indices:
                varied = positions.copy()
                varied[index] += 0.0001
                shifted = evaluate(varied)
                if shifted is None:
                    return None
                columns.append((shifted[:3, 3] + distance * shifted[:3, 1] - point) / 0.0001)
            jacobian = np.array(columns).T
            step = jacobian.T @ np.linalg.solve(
                jacobian @ jacobian.T + np.eye(3) * 1e-5, error)
            if not np.isfinite(step).all() or np.max(np.abs(step)) < 1e-8:
                return None
            positions[indices] += step * min(1.0, 0.15 / np.max(np.abs(step)))
        else:
            self.__logger.error("手先 +Y 軸上の接近前姿勢を求められませんでした．")
            return None

        goal = self._create_move_group_goal(group_name)
        constraints = Constraints()
        for index in indices:
            constraints.joint_constraints.append(JointConstraint(
                joint_name=names[index], position=float(positions[index]),
                tolerance_above=0.0001, tolerance_below=0.0001, weight=1.0))
        goal.request.goal_constraints = [constraints]
        if not self.__send_move_group_goal(goal):
            return None
        actual = self.collision.get_robot_state()
        pose = self._fk_pose(actual, link) if actual is not None else None
        if pose is None:
            return None
        matrix = _pose_matrix(pose)
        if np.linalg.norm(center - matrix[:3, 3] - distance * matrix[:3, 1]) > 0.0005:
            self.__logger.error("接近前の実際の手先姿勢が +Y 軸上の目標を満たしません．")
            return None
        return matrix


    def move_cartesian_y(
        self, distance: float, group_name: str = "arm_left_with_waist",
    ) -> bool:
        """現在の手先 +Y 方向へ直線接近する．距離は正の m 単位．

        +Y 軸と進行方向の許容角は 0.1 rad，直線・到達位置の許容差は 5 mm．
        部分経路は実行しない．時間補間後の経路も 10 ms 以下で順運動学検査する．
        条件を満たせない場合，位置だけの移動へ切り替えず False を返す．
        """
        side = self._arm_side(group_name)
        if side is None or not math.isfinite(distance) or distance <= 0.0:
            return False
        if not (self.__fk_cli.wait_for_service(timeout_sec=self.__timeout_sec)
                and self.__cartesian_cli.wait_for_service(timeout_sec=self.__timeout_sec)):
            return False
        state = self.collision.get_robot_state()
        if state is None or not state.joint_state.name:
            return False
        link = f"{side}_amazing_hand"
        pose = self._fk_pose(state, link)
        if pose is None:
            return False
        start = _pose_matrix(pose)
        target = start.copy()
        target[:3, 3] += start[:3, 1] * distance
        request = GetCartesianPath.Request()
        request.header.frame_id = "pelvis"
        request.start_state = state
        request.group_name = group_name
        request.link_name = link
        request.waypoints = [_matrix_pose(target)]
        request.max_step = 0.002
        # 相対ジャンプ判定は静止点が多い経路を誤検出するため，後段で絶対差を検査する．
        request.jump_threshold = 0.0
        request.avoid_collisions = True
        request.max_velocity_scaling_factor = 0.2
        request.max_acceleration_scaling_factor = 0.2
        constraint = OrientationConstraint()
        constraint.header.frame_id = "pelvis"
        constraint.link_name = link
        constraint.orientation = pose.orientation
        constraint.absolute_x_axis_tolerance = 0.1
        constraint.absolute_y_axis_tolerance = math.pi
        constraint.absolute_z_axis_tolerance = 0.1
        constraint.weight = 1.0
        request.path_constraints.orientation_constraints = [constraint]
        future = self.__cartesian_cli.call_async(request)
        rclpy.spin_until_future_complete(self.__node, future, timeout_sec=self.__timeout_sec)
        if not future.done():
            future.cancel()
            return False
        response = future.result()
        if (response is None or response.error_code.val != MoveItErrorCodes.SUCCESS
                or not math.isfinite(response.fraction) or response.fraction < 1.0 - 1e-9):
            self.__logger.error("+Y 方向の接近経路を全区間で生成できませんでした．")
            return False
        trajectory = response.solution.joint_trajectory
        samples = _sample_joint_trajectory(trajectory)
        model_names = list(state.joint_state.name)
        if (samples is None or not set(trajectory.joint_names).issubset(model_names)
                or response.solution.multi_dof_joint_trajectory.points):
            self.__logger.error("接近軌道の関節または時間情報が不正です．")
            return False
        initial = dict(zip(model_names, state.joint_state.position))
        if any(abs(initial[n] - q) > 0.01 for n, q in zip(trajectory.joint_names, samples[0])):
            return False
        poses = []
        for positions in samples:
            candidate = copy.deepcopy(state)
            values = dict(zip(trajectory.joint_names, positions))
            candidate.joint_state.position = [values.get(n, initial[n]) for n in model_names]
            sampled_pose = self._fk_pose(candidate, link)
            if sampled_pose is None:
                return False
            poses.append(_pose_matrix(sampled_pose))
        if not _positive_y_path_valid(poses, start, distance):
            self.__logger.error("接近軌道が手先 +Y 方向の条件を満たさないため実行しません．")
            return False
        current = self.collision.get_robot_state()
        if current is None:
            return False
        current_values = dict(zip(current.joint_state.name, current.joint_state.position))
        if any(n not in current_values or abs(current_values[n] - initial[n]) > 0.01
               for n in model_names):
            self.__logger.error("経路検査中に開始関節状態が変化しました．")
            return False
        if not self._execute_checked_trajectory(response.solution):
            return False
        current = self.collision.get_robot_state()
        final_pose = self._fk_pose(current, link) if current is not None else None
        return bool(final_pose is not None and _positive_y_path_valid(
            [start, _pose_matrix(final_pose)], start, distance))


    @staticmethod
    def _arm_side(group_name: str) -> Optional[str]:
        """単腕の計画グループだけを受け付ける．双腕の手先目標は曖昧なため拒否する．"""
        for side in ("left", "right"):
            if group_name in (f"arm_{side}", f"arm_{side}_with_waist"):
                return side
        return None


    def move_abs(
        self,
        x: float,
        y: float,
        z: float,
        roll: Optional[Union[float, str]] = None,
        pitch: Optional[Union[float, str]] = None,
        yaw: Optional[float] = None,
        ref_frame: str = "torso_link",
        group_name: str = "arm_left_with_waist",
        wait: bool = True,
    ) -> bool:
        """指定した基準座標系における絶対座標へエンドエフェクタを移動させる．

        Parameters
        ----------
        x : float
            目標 X 座標（m）．
        y : float
            目標 Y 座標（m）．
        z : float
            目標 Z 座標（m）．
        roll : Optional[Union[float, str]], default None
            目標ロール角（rad）．文字列の場合は ref_frame として扱われる（後方互換対応）．
        pitch : Optional[Union[float, str]], default None
            目標ピッチ角（rad）．文字列の場合は group_name として扱われる（後方互換対応）．
        yaw : Optional[float], default None
            目標ヨー角（rad）．
        ref_frame : str, default 'torso_link'
            基準座標系名．
        group_name : str, default 'arm_left_with_waist'
            制御対象の planning group 名．
        wait : bool, default True
            動作完了を待機するかどうか．

        Returns
        -------
        bool
            移動が正常に完了した場合は True，それ以外は False．
        """
        actual_roll: Optional[float] = None
        actual_pitch: Optional[float] = None
        actual_yaw: Optional[float] = yaw
        actual_ref: str = ref_frame
        actual_group: str = group_name

        if isinstance(roll, str):
            actual_ref = roll
            if isinstance(pitch, str):
                actual_group = pitch
                if isinstance(yaw, bool):
                    wait = yaw
                    actual_yaw = None
        elif isinstance(roll, (int, float)):
            actual_roll = float(roll)
        if isinstance(pitch, (int, float)):
            actual_pitch = float(pitch)

        side = self._arm_side(actual_group)
        if side is None:
            self.__logger.error(f"単腕の計画グループを指定してください: {actual_group}")
            return False

        if not self.__ik_cli.service_is_ready():
            self.__logger.warn("ComputeIK service is not ready.")
            return False

        tip_link = f"{side}_amazing_hand"
        specified = (actual_roll, actual_pitch, actual_yaw)
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = float(x), float(y), float(z)
        current = self.get_current_endeffector_pose(actual_ref, side)
        if current is None:
            return False
        q = current.rotation
        current_angles = euler_from_quaternion([q.x, q.y, q.z, q.w])
        angles = [old if new is None else float(new)
                  for old, new in zip(current_angles, specified)]
        qx, qy, qz, qw = quaternion_from_euler(*angles)
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(
            float, (qx, qy, qz, qw))
        target = self.transform_pose(pose, actual_ref)
        if target is None:
            return False

        req = GetPositionIK.Request()
        req.ik_request.group_name = actual_group
        req.ik_request.pose_stamped.header.frame_id = "pelvis"
        req.ik_request.pose_stamped.pose = target
        req.ik_request.ik_link_name = tip_link
        req.ik_request.timeout.sec = 2
        req.ik_request.avoid_collisions = True
        state = self.collision.get_robot_state()
        if state is None or not state.joint_state.name:
            self.__logger.error("IK の初期関節状態を取得できませんでした．")
            return False
        # IK サーバーが解釈可能なモデル内の関節だけを，シーンから取得する．
        req.ik_request.robot_state = state
        if any(angle is not None for angle in specified):
            constraint = OrientationConstraint()
            constraint.header.frame_id = "pelvis"
            constraint.link_name = tip_link
            constraint.orientation = target.orientation
            constraint.absolute_x_axis_tolerance = 0.1 if actual_roll is not None else math.pi
            constraint.absolute_y_axis_tolerance = 0.1 if actual_pitch is not None else math.pi
            constraint.absolute_z_axis_tolerance = 0.1 if actual_yaw is not None else math.pi
            constraint.weight = 1.0
            req.ik_request.constraints.orientation_constraints = [constraint]

        future = self.__ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.__node, future, timeout_sec=self.__timeout_sec)

        if future.done():
            res = future.result()
            if res is not None and res.error_code.val == 1:
                # 対象 planning group に属する関節名のみを抽出
                valid_joints = set()
                if actual_group in self.__srdf_states:
                    for state_joints in self.__srdf_states[actual_group].values():
                        valid_joints.update(state_joints.keys())
                if not valid_joints:
                    if "left" in actual_group:
                        valid_joints.update([
                            "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
                            "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint"
                        ])
                    if "right" in actual_group:
                        valid_joints.update([
                            "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
                            "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint"
                        ])
                    if "waist" in actual_group:
                        valid_joints.add("waist_yaw_joint")

                joint_targets: Dict[str, float] = {}
                for name, pos in zip(
                    res.solution.joint_state.name, res.solution.joint_state.position
                ):
                    if not valid_joints or name in valid_joints:
                        joint_targets[name] = pos
                return self.joint_control(
                    rel=False, wait=wait, planning_group=actual_group, **joint_targets
                )

        self.__logger.warn("IK solution failed for specified absolute pose.")
        return False


    def move_rel(
        self,
        x: float,
        y: float,
        z: float,
        ref_frame: str = "*_left_amazing_hand",
        group_name: str = "arm_left_with_waist",
        wait: bool = True,
    ) -> bool:
        """指定グループのエンドエフェクタを現在位置から相対移動させる．

        Parameters
        ----------
        x : float
            相対 X 変位（m）．
        y : float
            相対 Y 変位（m）．
        z : float
            相対 Z 変位（m）．
        ref_frame : str, default '*_left_amazing_hand'
            相対移動の基準フレーム．アスタリスクは group_name に基づき動的解決される．
        group_name : str, default 'arm_left_with_waist'
            制御対象の planning group 名．
        wait : bool, default True
            動作完了を待機するかどうか．

        Returns
        -------
        bool
            移動が正常に完了した場合は True，それ以外は False．
        """
        hand_side = self._arm_side(group_name)
        if hand_side is None:
            return False
        resolved_frame = ref_frame.replace("*_left", hand_side).replace("*", hand_side)
        current = self.get_current_endeffector_pose(ref=resolved_frame, arm_side=hand_side)
        if current is None:
            return False
        return self.move_abs(
            x=current.translation.x + x, y=current.translation.y + y,
            z=current.translation.z + z, ref_frame=resolved_frame,
            group_name=group_name, wait=wait
        )


    def joint_control(
        self,
        rel: bool = False,
        wait: bool = True,
        planning_group: str = "arm_both_with_waist",
        joint_tolerance: float = 0.01,
        **kwargs: float,
    ) -> bool:
        """MoveIt Action を通じて指定された関節角度へロボットを駆動する．

        Parameters
        ----------
        rel : bool, default False
            True の場合は現在の関節角度に対する相対変位として扱う．
        wait : bool, default True
            動作完了（目標到達）を待機するかどうか．
        planning_group : str, default 'arm_both_with_waist'
            対象の planning group 名．
        joint_tolerance : float, default 0.01
            目標関節角の許容差（rad，正の有限値）．狭い配置場所への姿勢復帰で指定する．
        **kwargs : float
            関節名と目標角度（ラジアン）のペア．

        Returns
        -------
        bool
            軌道計画および実行が正常に完了した場合は True，それ以外は False．
        """
        if not math.isfinite(joint_tolerance) or joint_tolerance <= 0:
            self.__logger.error("joint_tolerance は正の有限値を指定してください．")
            return False
        if not kwargs:
            self.__logger.warn("No joints specified for joint_control.")
            return False

        current_poses = self.get_current_joint_pose() if rel else {}
        targets: Dict[str, float] = {}
        for j_name, j_val in kwargs.items():
            final_val = float(j_val)
            if rel:
                final_val = current_poses.get(j_name, 0.0) + final_val
            targets[j_name] = final_val

        goal = self._create_move_group_goal(planning_group)
        constraints = Constraints()
        for j_name, target in targets.items():
            jc = JointConstraint()
            jc.joint_name = j_name
            jc.position = float(target)
            jc.tolerance_above = float(joint_tolerance)
            jc.tolerance_below = float(joint_tolerance)
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        goal.request.goal_constraints = [constraints]
        return self.__send_move_group_goal(goal, wait=wait)


    def get_current_joint_pose(self) -> Dict[str, float]:
        """現在の全関節名と関節角度の辞書を取得する．

        Returns
        -------
        Dict[str, float]
            関節名と角度（ラジアン）のマップ．取得失敗時は空の辞書．
        """
        latest_joints: Optional[Dict[str, float]] = None

        def __cb_joint(msg: JointState) -> None:
            nonlocal latest_joints
            latest_joints = dict(zip(msg.name, msg.position))

        with TemporarySubscriber(
            node=self.__node,
            msg=JointState,
            topic="/joint_states",
            qos_profile=10,
            cb=__cb_joint,
        ):
            start_time = time.monotonic()
            while rclpy.ok() and latest_joints is None:
                if time.monotonic() - start_time > self.__timeout_sec:
                    self.__logger.warn("Timeout waiting for /joint_states topic.")
                    break
                rclpy.spin_once(self.__node, timeout_sec=0.05)

        return latest_joints if latest_joints is not None else {}


    def arm_action(self, mode: int) -> bool:
        """Unitree プリセット腕動作を実行する．

        Parameters
        ----------
        mode : int
            動作モード番号（ArmAction.srv 定数）．

        Returns
        -------
        bool
            動作要求が正常に受理され完了した場合は True，それ以外は False．
        """
        if self.__use_sim_time and not self.__arm_action_cli.service_is_ready():
            self.__logger.warn(
                f"ArmAction mode={mode} is not supported in simulation/mock environment (service unavailable)."
            )
            return False

        req = ArmAction.Request()
        req.mode = int(mode)
        return self.__send_arm_action_req(req)


    def upper_body_control(self, enable: bool) -> bool:
        """上半身の関節制御権限の有効化・無効化を切り替える．

        Parameters
        ----------
        enable : bool
            制御を有効化する場合は True，無効化（脱力・保持解除）する場合は False．

        Returns
        -------
        bool
            切り替えが成功した場合は True，それ以外は False．
        """
        has_enable_service = (
            self.__upper_enable_cli.service_is_ready()
            or self.__fallback_upper_enable_cli.service_is_ready()
        )
        if self.__use_sim_time and not has_enable_service:
            self.__logger.info(
                f"use_sim_time is active. Skipping hardware upper body control request (enable={enable})."
            )
            return True

        req = SetBool.Request()
        req.data = bool(enable)
        ok = self.__send_upper_enable_req(req)
        # FAULT_LATCHED 等で有効化に失敗した場合は，一度 disable を送ってフォルトをリセットし再試行
        if not ok and enable:
            self.__logger.info("Attempting fault acknowledgment/reset before re-enabling upper body...")
            reset_req = SetBool.Request()
            reset_req.data = False
            self.__send_upper_enable_req(reset_req)
            time.sleep(0.5)
            ok = self.__send_upper_enable_req(req)
        return ok


class NavControl:
    """Nav2 を用いて Unitree G1 の自律移動および現在姿勢取得を制御する API クラス．

    Parameters
    ----------
    node : Node
        ROS 2 ノードインスタンス．
    timeout_sec : float, default 60.0
        ナビゲーションのデフォルトタイムアウト秒数．
    tf_buffer : Buffer, optional
        TF2 バッファインスタンス．未指定時は新規作成．

    Methods
    -------
    move_abs(x: float, y: float, yaw: float, ref_frame: str = 'map', wait: bool = True) -> bool
        指定座標系における絶対座標へ自律移動する．
    move_rel(x: float, y: float, yaw: float, ref_frame: str = 'base_link', wait: bool = True) -> bool
        ロボットの現在位置からの相対座標目標へ自律移動する．
    move_pose(pose: PoseStamped, wait: bool = True) -> bool
        PoseStamped 目標へ自律移動する．
    get_current_pose(ref_frame: str = 'map', use_xyy: bool = True) -> Union[List[float], PoseStamped]
        ロボットの現在位置姿勢を取得する．
    cancel_navigation_action() -> bool
        実行中のナビゲーション動作を明示的にキャンセルする．
    """

    def __init__(
        self,
        node: Node,
        timeout_sec: float = 60.0,
        tf_buffer: Optional[Buffer] = None,
    ) -> None:
        """API クラスのインスタンスを初期化する．

        Parameters
        ----------
        node : Node
            ROS 2 ノードインスタンス．
        timeout_sec : float, default 60.0
            ナビゲーション完了待機のデフォルトタイムアウト秒数．
        tf_buffer : Buffer, optional
            TF2 バッファインスタンス．

        Raises
        ------
        RuntimeError
            指定時間内に Nav2 アクションサーバーが検出されなかった場合．
        """
        self.__node = node
        self.__logger = node.get_logger()
        self.__timeout_sec = timeout_sec

        self.__cb_group = MutuallyExclusiveCallbackGroup()

        self.__nav_cli = ActionClient(
            self.__node,
            NavigateToPose,
            "/navigate_to_pose",
            callback_group=self.__cb_group,
        )

        self.__cmd_vel_pub = self.__node.create_publisher(
            Twist,
            "/cmd_vel",
            10,
            callback_group=self.__cb_group,
        )

        self.__tf_buffer = tf_buffer or Buffer()
        self.__tf_listener = TransformListener(self.__tf_buffer, self.__node)

        self.__current_goal_handle: Optional[ClientGoalHandle] = None

        while not self.__nav_cli.wait_for_server(timeout_sec=5.0):
            self.__logger.error("Required action server /navigate_to_pose is not available.")
            raise RuntimeError("Failed to connect to /navigate_to_pose action server.")


    def __cb_nav_feedback(self, feedback_msg: Any) -> None:
        """ナビゲーション進捗コールバック．"""
        pass


    def cancel_navigation_action(self) -> bool:
        """実行中の自律移動アクションを明示的にキャンセルする．

        Returns
        -------
        bool
            キャンセル要求が正常に送信された場合は True，それ以外は False．
        """
        if self.__current_goal_handle is None:
            return True

        self.__logger.warn("Canceling active navigation goal...")
        future = self.__current_goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self.__node, future, timeout_sec=5.0)

        # 停止コマンドを publish して惰性走行を抑制
        stop_cmd = Twist()
        self.__cmd_vel_pub.publish(stop_cmd)
        self.__current_goal_handle = None
        return True


    def move_pose(self, pose: PoseStamped, wait: bool = True) -> bool:
        """PoseStamped 目標へ自律移動を実行する．

        Parameters
        ----------
        pose : PoseStamped
            目標位置および目標姿勢．
        wait : bool, default True
            目標到達まで処理をブロックするかどうか．

        Returns
        -------
        bool
            ナビゲーションが成功した場合は True，失敗またはキャンセル時は False．
        """
        goal = NavigateToPose.Goal()
        goal.pose = pose

        send_future = self.__nav_cli.send_goal_async(
            goal, feedback_callback=self.__cb_nav_feedback
        )
        rclpy.spin_until_future_complete(self.__node, send_future, timeout_sec=self.__timeout_sec)

        if not send_future.done():
            self.__logger.error("Timeout sending navigation goal.")
            return False

        goal_handle: ClientGoalHandle = send_future.result()
        if not goal_handle.accepted:
            self.__logger.warn("Navigation goal was rejected by Nav2 server.")
            return False

        self.__current_goal_handle = goal_handle

        if not wait:
            return True

        result_future = goal_handle.get_result_async()
        try:
            rclpy.spin_until_future_complete(
                self.__node, result_future, timeout_sec=self.__timeout_sec
            )
        except KeyboardInterrupt:
            self.__logger.warn("Navigation interrupted by user. Canceling goal...")
            self.cancel_navigation_action()
            raise

        if not result_future.done():
            self.__logger.error("Navigation timed out. Canceling goal...")
            self.cancel_navigation_action()
            return False

        res = result_future.result()
        self.__current_goal_handle = None
        return res.status == GoalStatus.STATUS_SUCCEEDED


    def move_abs(
        self,
        x: float,
        y: float,
        yaw: float,
        ref_frame: str = "map",
        wait: bool = True,
    ) -> bool:
        """指定した基準座標系における絶対座標へ自律移動する．

        Parameters
        ----------
        x : float
            目標 X 座標（m）．
        y : float
            目標 Y 座標（m）．
        yaw : float
            目標ヨー角（rad）．
        ref_frame : str, default 'map'
            基準座標系名．
        wait : bool, default True
            目標到達まで待機するかどうか．

        Returns
        -------
        bool
            ナビゲーションが成功した場合は True，それ以外は False．
        """
        pose = PoseStamped()
        pose.header.frame_id = str(ref_frame)
        pose.header.stamp = self.__node.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0

        q = quaternion_from_euler(0.0, 0.0, float(yaw))
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        return self.move_pose(pose, wait=wait)


    def move_rel(
        self,
        x: float,
        y: float,
        yaw: float,
        ref_frame: str = "base_link",
        wait: bool = True,
    ) -> bool:
        """ロボットの現在位置からの相対座標目標へ自律移動する．

        Parameters
        ----------
        x : float
            前後方向の移動変位（m，前方が正）．
        y : float
            左右方向の移動変位（m，左方が正）．
        yaw : float
            回転角度変位（rad，反時計回りが正）．
        ref_frame : str, default 'base_link'
            相対移動の基準フレーム．
        wait : bool, default True
            目標到達まで待機するかどうか．

        Returns
        -------
        bool
            ナビゲーションが成功した場合は True，それ以外は False．
        """
        pose = PoseStamped()
        pose.header.frame_id = str(ref_frame)
        pose.header.stamp = self.__node.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0

        q = quaternion_from_euler(0.0, 0.0, float(yaw))
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        return self.move_pose(pose, wait=wait)


    def get_current_pose(
        self,
        ref_frame: str = "map",
        use_xyy: bool = True,
    ) -> Union[List[float], PoseStamped]:
        """ロボットの現在位置姿勢を取得する．

        Parameters
        ----------
        ref_frame : str, default 'map'
            基準座標系名．
        use_xyy : bool, default True
            True の場合は [x, y, yaw] の 1 次元リスト形式，
            False の場合は PoseStamped 型で返却する．

        Returns
        -------
        Union[List[float], PoseStamped]
            use_xyy=True の場合は [x, y, yaw]，False の場合は PoseStamped．
        """
        try:
            tf_stamped = self.__tf_buffer.lookup_transform(
                ref_frame,
                "base_link",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0),
            )
            px = tf_stamped.transform.translation.x
            py = tf_stamped.transform.translation.y
            pz = tf_stamped.transform.translation.z
            rot = tf_stamped.transform.rotation
            yaw = _quaternion_to_yaw(rot.x, rot.y, rot.z, rot.w)

            if use_xyy:
                return [px, py, yaw]

            pose = PoseStamped()
            pose.header = tf_stamped.header
            pose.pose.position.x = px
            pose.pose.position.y = py
            pose.pose.position.z = pz
            pose.pose.orientation = rot
            return pose

        except TransformException as exc:
            self.__logger.warn(f"TF lookup failed for get_current_pose: {exc}")

        # フォールバック: /localization/pose_with_covariance の最新値を購読
        latest_pose: Optional[PoseWithCovarianceStamped] = None

        def __cb_loc(msg: PoseWithCovarianceStamped) -> None:
            nonlocal latest_pose
            latest_pose = msg

        loc_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        with TemporarySubscriber(
            node=self.__node,
            msg=PoseWithCovarianceStamped,
            topic="/localization/pose_with_covariance",
            qos_profile=loc_qos,
            cb=__cb_loc,
        ):
            start_time = time.monotonic()
            while rclpy.ok() and latest_pose is None:
                if time.monotonic() - start_time > 2.0:
                    break
                rclpy.spin_once(self.__node, timeout_sec=0.05)

        if latest_pose is not None:
            px = latest_pose.pose.pose.position.x
            py = latest_pose.pose.pose.position.y
            pz = latest_pose.pose.pose.position.z
            rot = latest_pose.pose.pose.orientation
            yaw = _quaternion_to_yaw(rot.x, rot.y, rot.z, rot.w)
            if use_xyy:
                return [px, py, yaw]
            pose = PoseStamped()
            pose.header = latest_pose.header
            pose.pose = latest_pose.pose.pose
            return pose

        self.__logger.error("Failed to determine current robot pose.")
        return [0.0, 0.0, 0.0] if use_xyy else PoseStamped()


class G1Mic:
    """ロボット搭載マイクからの音声収録を管理するコンテキストマネージャクラス．

    Parameters
    ----------
    node : Node
        ROS 2 ノードインスタンス．
    sample_rate : int, default 16000
        サンプリングレート（Hz）．
    channels : int, default 1
        オーディオチャンネル数．
    """

    def __init__(self, node: Node, sample_rate: int = 16000, channels: int = 1) -> None:
        """マイク管理クラスのインスタンスを初期化する．"""
        self.__node = node
        self.__logger = node.get_logger()
        self.__sample_rate = sample_rate
        self.__channels = channels
        self.__audio_buffer: List[np.ndarray] = []
        self.__buffer_lock = threading.Lock()

        self.__mic_sub = self.__node.create_subscription(
            Int16MultiArray,
            "/mic_data",
            self.__audio_callback,
            10,
        )

        self.__mic_enable_cli = self.__node.create_client(
            SetBool,
            "/enable_mic",
        )


    def __audio_callback(self, msg: Int16MultiArray) -> None:
        """マイク音声データ受信コールバック．"""
        data = np.array(msg.data, dtype=np.int16)
        with self.__buffer_lock:
            self.__audio_buffer.append(data)


    def __enter__(self) -> "G1Mic":
        """コンテキストマネージャ開始時に音声配信を有効化する．"""
        req = SetBool.Request()
        req.data = True
        if self.__mic_enable_cli.wait_for_service(timeout_sec=2.0):
            future = self.__mic_enable_cli.call_async(req)
            rclpy.spin_until_future_complete(self.__node, future, timeout_sec=2.0)
        with self.__buffer_lock:
            self.__audio_buffer = []
        return self


    def __exit__(self, exc_type: Any, exc_val: Any, exc_tb: Any) -> None:
        """コンテキストマネージャ終了時に音声配信を無効化する．"""
        req = SetBool.Request()
        req.data = False
        if self.__mic_enable_cli.wait_for_service(timeout_sec=1.0):
            future = self.__mic_enable_cli.call_async(req)
            rclpy.spin_until_future_complete(self.__node, future, timeout_sec=1.0)


    def read(self) -> np.ndarray:
        """前回呼出から現在までに蓄積された全音声データを取得する．

        Returns
        -------
        np.ndarray
            結合された int16 音声配列．
        """
        with self.__buffer_lock:
            if not self.__audio_buffer:
                return np.array([], dtype=np.int16)
            full_data = np.concatenate(self.__audio_buffer)
            self.__audio_buffer = []
            return full_data


    def save_wav(self, file_path: str, audio_data: np.ndarray) -> bool:
        """音声データを WAV ファイルとして保存する．

        Parameters
        ----------
        file_path : str
            出力先ファイルパス．
        audio_data : np.ndarray
            保存対象の音声データ配列．

        Returns
        -------
        bool
            保存に成功した場合は True，それ以外は False．
        """
        if audio_data.size == 0:
            self.__logger.warn("No audio data to save.")
            return False

        try:
            with wave.open(file_path, "wb") as wf:
                wf.setnchannels(self.__channels)
                wf.setsampwidth(2)
                wf.setframerate(self.__sample_rate)
                wf.writeframes(audio_data.tobytes())
            return True
        except Exception as exc:
            self.__logger.error(f"Failed to save WAV file: {exc}")
            return False


class ArmGrasp:
    """単腕の物体把持・配置を管理する．

    先頭形状の中心を目標座標の基準とし，複合形状の配置を保つ．
    ラバーハンドの物理的な保持力は検出せず，PlanningScene の結合を管理する．
    呼び出しは逐次実行すること．
    """

    POSITION_TOLERANCE = 0.01
    ORIENTATION_TOLERANCE = 0.1

    def __init__(self, arm: ArmControl, collision: Optional[ArmCollision] = None) -> None:
        """アームとシーン管理を共有する．"""
        self.__arm = arm
        self.__node = arm._ArmControl__node
        self.__logger = self.__node.get_logger()
        self.__collision = collision or arm.collision
        self.__last_grasp_plan = None

    @property
    def arm(self) -> ArmControl:
        """アーム制御インスタンスを取得する．"""
        return self.__arm

    @property
    def collision(self) -> ArmCollision:
        """コリジョン管理インスタンスを取得する．"""
        return self.__collision

    @property
    def last_grasp_plan(self) -> Optional[Dict[str, Any]]:
        """最後に選択した方向・距離を返す．候補が成立しなかった場合は None．"""
        return copy.deepcopy(self.__last_grasp_plan)

    def _group(self, arm_side: str, group_name: Optional[str]) -> Optional[str]:
        """左右腕と計画グループの一致を確認する．"""
        group = group_name or f"arm_{arm_side}_with_waist"
        if arm_side not in ("left", "right") or ArmControl._arm_side(group) != arm_side:
            self.__logger.error(f"腕と計画グループが一致しません: {arm_side}, {group}")
            return None
        return group

    def _ee_matrix(self, arm_side: str, ref: str = "pelvis") -> Optional[np.ndarray]:
        """現在の手先を同次変換で取得する．"""
        transform = self.__arm.get_current_endeffector_pose(ref=ref, arm_side=arm_side)
        if transform is None:
            return None
        q = transform.rotation
        matrix = quaternion_matrix([q.x, q.y, q.z, q.w])
        p = transform.translation
        matrix[:3, 3] = [p.x, p.y, p.z]
        return matrix

    def grasp(
        self,
        object: str,
        approach_type: str = "auto",
        arm_side: str = "left",
        offset_dist: Optional[float] = None,
        pre_offset_dist: Optional[float] = None,
        lift: bool = True,
        lift_height: float = 0.10,
        group_name: Optional[str] = None,
        grasp_offset: Optional[float] = None,
        approach_frame: str = "base_link",
    ) -> bool:
        """手先のローカル +Y 方向へ接近し，物体を結合して持ち上げる．

        Parameters
        ----------
        object : str
            PlanningScene に登録された把持対象の ID．先頭形状の中心を
            物体基準点とする．
        approach_type : {'auto', 'side', 'front', 'top'}, default 'auto'
            接近方向．`approach_frame` 基準で，side は左腕が -Y，
            右腕が +Y，front は +X（ロボット側から），top は -Z．
            auto は三方向の姿勢・経路・衝突を移動前に評価し，成立した
            候補から関節移動量と関節限界までの余裕に基づいて選択する．
        arm_side : {'left', 'right'}, default 'left'
            使用する腕．
        offset_dist : float or None, default None
            把持時の物体基準点と手先原点の距離（m，非負）．None は物体と
            ハンドの形状から候補を作り，衝突・到達可能性により選択する．
            数値を指定した場合は自動変更しない．
        pre_offset_dist : float or None, default None
            最終直線接近の距離（m，正）．None は 0.025，0.05，0.10 m
            の候補から選択する．接近前の物体と手先の距離は，これと
            選択された `offset_dist` の合計になる．
        lift : bool, default True
            結合後に pelvis の +Z 方向へ持ち上げるかどうか．
        lift_height : float, default 0.10
            持ち上げる高さ（m，非負）．
        group_name : str or None, default None
            計画グループ．None は arm_left_with_waist または
            arm_right_with_waist．指定する場合は `arm_side` と一致させる．
        grasp_offset : float or None, default None
            `offset_dist` の互換引数．指定時は `offset_dist` より優先する．
        approach_frame : str, default 'base_link'
            接近方向の基準フレーム．開始時の方向を pelvis へ変換して固定し，
            腰の回転に追従させない．

        Returns
        -------
        bool
            把持および指定された持ち上げが成功した場合は True．入力不正，
            到達不能，衝突，通信失敗，動作失敗の場合は False．

        Notes
        -----
        すべての方向で手先 +Y 軸を対象へ向ける．最終接近の方向許容差は
        0.1 rad，直線経路・終点の検査許容差は 5 mm．補間後の軌道も検査し，
        部分経路は実行しない．side / front / top を明示した場合は，
        指定方向が不成立でも別方向へ切り替えない．

        自動距離は，接近側の物体表面とハンド形状が接する距離の近傍から
        5，15，25 mm 内側の候補を作る．基本形状とメッシュに対応する．
        平面など有限な寸法を得られない対象では距離の明示が必要となる．
        候補探索は有限であり，任意の物体配置での成功を保証するものではない．
        選択結果はログと `last_grasp_plan` で確認できる．

        対象と指定ハンド以外の接触は許可しない．結合前の失敗では物体を
        ワールドに残し，結合後のリフト失敗では手先への結合を維持する．
        接触許可は処理終了時に復元する．物理的な保持力の検出やハンド開閉は
        行わず，PlanningScene 上の把持を扱う．呼び出しは逐次実行すること．

        Examples
        --------
        >>> grasp.grasp('object_0', approach_type='auto')
        >>> grasp.grasp('object_0', approach_type='top', arm_side='right',
        ...             offset_dist=0.07, pre_offset_dist=0.025, lift=False)
        """
        from .grasp_planner import GraspPlanner

        self.__last_grasp_plan = None
        group = self._group(arm_side, group_name)
        if group is None or approach_type not in ("auto", "side", "front", "top"):
            self.__logger.error("腕・計画グループ・approach_type の指定を確認してください．")
            return False
        try:
            if grasp_offset is not None:
                offset_dist = float(grasp_offset)
            if (not approach_frame or not math.isfinite(lift_height) or lift_height < 0
                    or any(v is not None and (not math.isfinite(v) or v < 0)
                           for v in (offset_dist, pre_offset_dist))
                    or pre_offset_dist == 0):
                raise ValueError('距離・高さ・基準フレームの指定が不正です．')
        except (TypeError, ValueError) as exc:
            self.__logger.error(str(exc))
            return False
        target_object = self.__collision.get_object(object)
        if target_object is None:
            self.__logger.error(f"把持対象が見つかりません: {object}")
            return False
        tip = f"{arm_side}_amazing_hand"
        touch_links = [tip, f"{arm_side}_wrist_roll_rubber_hand"]
        ok = False
        original_acm = None
        restored = True
        try:
            with GraspPlanner(self.__arm, self.__collision) as planner:
                plan = planner.plan(target_object, approach_type, arm_side, group,
                                    offset_dist, pre_offset_dist, approach_frame)
                if plan is None:
                    return False
                self.__last_grasp_plan = {
                    "approach_type": plan.approach_type, "offset_dist": plan.offset,
                    "pre_offset_dist": plan.pre_offset, "approach_frame": approach_frame,
                    "direction_pelvis": plan.direction.tolist(),
                }
                if (not planner.matches_state(plan.initial_state)
                        or self.__collision.get_object(object) != target_object):
                    self.__logger.error("候補評価中に開始状態または対象物体が変化しました．")
                    return False
                if not self.__arm._execute_checked_trajectory(plan.pre_trajectory):
                    return False
                if (not planner.matches_state(plan.pre_state)
                        or self.__collision.get_object(object) != target_object):
                    return False
                # 移動後のシーンでも接近全区間を再確認する．
                trajectory = plan.approach_trajectory.joint_trajectory
                samples = _sample_joint_trajectory(trajectory)
                if samples is None or any(not planner.valid(
                        planner.state_at(plan.pre_state, trajectory.joint_names, values),
                        object, touch_links) for values in samples):
                    return False
                original_acm = self.__collision.get_allowed_collision_matrix()
                if original_acm is None:
                    return False
                approach_acm = copy.deepcopy(original_acm)
                for link in touch_links:
                    ArmCollision._set_allowed_pair(approach_acm, object, link)
                ok = self.__collision.apply_allowed_collision_matrix(approach_acm)
                if ok:
                    ok = self.__arm._execute_checked_trajectory(plan.approach_trajectory)
            if ok:
                state = self.__collision.get_robot_state()
                pose = self.__arm._fk_pose(state, tip) if state is not None else None
                reached = _pose_matrix(pose) if pose is not None else None
                target = plan.center - plan.direction * plan.offset
                ok = (reached is not None and
                      np.linalg.norm(reached[:3, 3] - target) <= self.POSITION_TOLERANCE)
                if ok:
                    relative = reached[:3, :3].T @ (plan.center - reached[:3, 3])
                    ok = (float(reached[:3, 1] @ plan.direction) >= math.cos(0.1)
                          and relative[1] >= -0.001)
                    if plan.offset > 0.0:
                        ok = ok and math.atan2(np.linalg.norm(relative[[0, 2]]), relative[1]) <= 0.1
                if not ok:
                    self.__logger.error("把持位置または +Y 接近の条件を満たさないため結合しません．")
            if ok:
                # ID による移管で物体原点・全形状・サブフレームの座標を MoveIt に保持させる．
                ok = self.__collision.attach(object, attach_frame=tip, touch_links=touch_links)
            if ok:
                attached = self.__collision.get_attached_object(object, link_name=tip)
                ok = attached is not None and attached.link_name == tip
        except Exception as exc:
            self.__logger.error(f"把持中に失敗しました: {exc}")
            ok = False
        finally:
            if original_acm is not None:
                restored = self.__collision.apply_allowed_collision_matrix(original_acm)
                if not restored:
                    self.__logger.error("一時的な接触許可を復元できませんでした．")
        if not ok or not restored:
            return False
        if lift:
            return self.__arm.move_rel(
                0.0, 0.0, lift_height, ref_frame="pelvis", group_name=group)
        return True

    def place(
        self,
        x: float,
        y: float,
        z: float,
        roll: Optional[float] = None,
        pitch: Optional[float] = None,
        yaw: Optional[float] = None,
        use_rlt: bool = False,
        ref: str = "torso_link",
        release: bool = True,
        detach: bool = True,
        dettach: Optional[bool] = None,
        object: Optional[str] = None,
        arm_side: str = "left",
        group_name: Optional[str] = None,
    ) -> bool:
        """物体基準点を指定座標へ移動し，実際の到達姿勢で解放する．

        x, y, z は先頭形状の中心（m）．use_rlt=True では現在の物体中心から
        ref 軸方向の変位とする．目標は開始時点の ref から pelvis に固定する．
        姿勢未指定時は位置優先で計画し，実際の手先回転に応じて中心位置を補正する．
        指定された姿勢を満たせない場合は False を返して結合を維持する．
        detach=False は結合を維持する．dettach は互換引数．
        release はハンド開閉未対応のため予約引数として受け付ける．
        """
        group = self._group(arm_side, group_name)
        if group is None:
            return False
        if dettach is not None:
            detach = dettach
        tip = f"{arm_side}_amazing_hand"
        attached = self.__collision.get_attached_object(object or "", link_name=tip)
        if attached is None or attached.link_name != tip:
            self.__logger.error(f"指定した腕に配置対象が結合されていません: {tip}")
            return False

        local_pose = self.__arm.transform_pose(
            _object_reference_pose(attached.object), attached.object.header.frame_id, tip)
        current = self._ee_matrix(arm_side, ref)
        if local_pose is None or current is None:
            return False
        local = _pose_matrix(local_pose)
        current_object = current @ local
        target = current_object.copy()
        target[:3, 3] = np.array([float(x), float(y), float(z)])
        if use_rlt:
            target[:3, 3] += current_object[:3, 3]
        specified = (roll, pitch, yaw)
        explicit_orientation = any(value is not None for value in specified)
        if explicit_orientation:
            current_angles = euler_from_quaternion(quaternion_from_matrix(current_object))
            angles = [old if new is None else float(new)
                      for old, new in zip(current_angles, specified)]
            target[:3, :3] = quaternion_matrix(quaternion_from_euler(*angles))[:3, :3]
        target_pose = self.__arm.transform_pose(_matrix_pose(target), ref, "pelvis")
        if target_pose is None:
            return False
        target = _pose_matrix(target_pose)
        if not np.isfinite(target).all():
            return False

        # 位置優先 IK は手先の回転を変え得るため，物体中心の実測誤差で補正する．
        for attempt in range(5):
            current = self._ee_matrix(arm_side)
            if current is None:
                return False
            actual = current @ local
            position_error = np.linalg.norm(actual[:3, 3] - target[:3, 3])
            rotation_error = math.acos(float(np.clip(
                (np.trace(target[:3, :3].T @ actual[:3, :3]) - 1.0) / 2.0, -1.0, 1.0)))
            if (position_error <= self.POSITION_TOLERANCE and
                    (not explicit_orientation or rotation_error <= self.ORIENTATION_TOLERANCE)):
                if detach:
                    # MoveIt が現在のロボット状態でワールドへ戻す．目標座標で上書きしない．
                    return self.__collision.detach(attached.object.id, attach_frame=tip)
                return True
            if attempt == 4:
                break
            rotation = (target[:3, :3] @ local[:3, :3].T
                        if explicit_orientation else current[:3, :3])
            ee_target = target[:3, 3] - rotation @ local[:3, 3]
            kwargs = {}
            if explicit_orientation:
                transform = np.eye(4)
                transform[:3, :3] = rotation
                angles = euler_from_quaternion(quaternion_from_matrix(transform))
                kwargs = dict(zip(("roll", "pitch", "yaw"), angles))
            if not self.__arm.move_abs(
                    *map(float, ee_target), ref_frame="pelvis", group_name=group, **kwargs):
                return False
        self.__logger.error(f"配置位置の誤差が許容値を超えています: {position_error:.4f} m")
        return False


# 後方互換性エイリアス / Piper 互換エイリアス
Collision = ArmCollision
Grasp = ArmGrasp
G1Navigation = NavControl
