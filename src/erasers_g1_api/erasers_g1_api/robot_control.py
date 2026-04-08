import numpy as np
try:
    np.float = float
except AttributeError:
    pass

#!/usr/bin/env python3
from rclpy.node import Node
import rclpy

# msgs
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped, Pose
from sensor_msgs.msg import Imu, JointState
from shape_msgs.msg import SolidPrimitive
from amazing_hand_interfaces.srv import HandCommand
from g1_srvs.srv import MoveServo
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
from std_msgs.msg import Int16MultiArray

# tf
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformListener, Buffer
from tf_transformations import euler_from_quaternion, quaternion_from_euler

# general
import time
import math
import copy
from rclpy.action import ActionClient
from rclpy_util.util import TemporarySubscriber

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

class G1Control():
    def __init__(self, node:Node):
        self.__node = node

        self.__servo_cli = self.__node.create_client(MoveServo, '/move_servo')
        self.__hand_cli = self.__node.create_client(HandCommand, '/hand_command')

        while not self.__servo_cli.wait_for_service(timeout_sec=5.0):
            self.__node.get_logger().error('Servo Service Servers are not running ...')
            break
        while not self.__hand_cli.wait_for_service(timeout_sec=5.0):
            self.__node.get_logger().error('Hand Service Servers are not running ...')
            break


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


class G1Navigation():
    def __init__(self, node: Node, wait_time: int = 10, tf_buffer: Buffer = None):
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
        """
        self.__node = node
        self.__current_goal_handle = None

        # TF2 Setup
        self.__tf_buffer = tf_buffer or Buffer()
        self.__tf_listener = TransformListener(self.__tf_buffer, self.__node)

        # Action Client Setup
        self.__action_client = ActionClient(self.__node, NavigateToPose, "/navigate_to_pose")
        if not self.__action_client.wait_for_server(timeout_sec=wait_time):
            self.__node.get_logger().fatal("Nav2 action server not available...")
            raise RuntimeError("Nav2 action server not available")

        # Initial pose publisher
        self.__initial_pose_pub = self.__node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        
        # Cmd Vel Publisher for Precision Correction
        self.__cmd_vel_pub = self.__node.create_publisher(Twist, '/cmd_vel', 10)


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
            rclpy.spin_once(self.__node, timeout_sec=0.1)
            try:
                transform = self.__tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                
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
                self.__node.get_logger().debug(f"TF Lookup failed: {str(e)}")
                continue


    def move_to_pose(self, pose, tolerance: float = 0.05, reference_frame: str = 'map', wait: bool = True) -> bool:
        """
        与えられた目標姿勢に基づいてロボットを自律移動させる．
        すべてのナビゲーションの中核となるメソッドであり、KeyboardInterrupt 発生時には即座にアクションをキャンセルする。

        Parameters
        ----------
        pose : PoseStamped or Pose
            目標とする姿勢情報。Pose メッセージの場合、reference_frame の座標系基準として扱われる。
        tolerance : float, optional
            目標から指定された距離(m)以内に到達した場合、その時点でナビゲーションを成功として終了する。デフォルトは 0.05。
        reference_frame : str, optional
            pose が Pose 型の場合の基準フレーム。デフォルトは 'map'。
        wait : bool, optional
            移動完了まで処理をブロックするかどうか。デフォルトは True。

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
            goal_pose.header.stamp = self.__node.get_clock().now().to_msg()
            goal_pose.pose = pose
        else:
            self.__node.get_logger().error("pose must be PoseStamped or Pose")
            return False

        # Transform to map frame if not already in map frame
        if goal_pose.header.frame_id != 'map':
             try:
                 transform = self.__tf_buffer.lookup_transform('map', goal_pose.header.frame_id, rclpy.time.Time(), rclpy.duration.Duration(seconds=1.0))
                 
                 import tf2_geometry_msgs
                 goal_pose = tf2_geometry_msgs.do_transform_pose(goal_pose, transform)
             except Exception as e:
                 self.__node.get_logger().error(f"Failed to transform pose to map frame: {str(e)}")
                 return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        future = self.__action_client.send_goal_async(goal_msg)

        if not wait:
            # 非同期モードの場合は送信完了まで少し待機して終了とする
            try:
                rclpy.spin_until_future_complete(self.__node, future, timeout_sec=0.5)
            except KeyboardInterrupt:
                pass
            return True

        # 同期モード (wait=True)
        try:
            rclpy.spin_until_future_complete(self.__node, future, timeout_sec=10.0)
            if not future.done():
                self.__node.get_logger().error("Send goal timed out")
                return False

            goal_handle = future.result()
            self.__current_goal_handle = goal_handle

            if not goal_handle.accepted:
                self.__node.get_logger().error("Goal rejected by server")
                return False

            result_future = goal_handle.get_result_async()
            
            nav_success = False
            while rclpy.ok() and not result_future.done():
                rclpy.spin_once(self.__node, timeout_sec=0.1)

                if tolerance is not None:
                    # 目標位置までの距離をチェック
                    current_pose = self.get_current_pose(simple=True)
                    if current_pose is not None:
                        goal_x = goal_pose.pose.position.x
                        goal_y = goal_pose.pose.position.y
                        dist = math.sqrt((current_pose[0] - goal_x)**2 + (current_pose[1] - goal_y)**2)
                        
                        if dist <= tolerance:
                            self.__node.get_logger().info(f"Reached tolerance limit ({dist:.3f} <= {tolerance:.3f}). Canceling Nav2 and starting precision phase.")
                            cancel_future = goal_handle.cancel_goal_async()
                            rclpy.spin_until_future_complete(self.__node, cancel_future, timeout_sec=5.0)
                            nav_success = True
                            break

            if not nav_success:
                result = result_future.result()
                if result.status == GoalStatus.STATUS_SUCCEEDED:
                    nav_success = True
                else:
                    self.__node.get_logger().warn(f"Navigation failed with status: {result.status}")
                    nav_success = False

            if nav_success:
                pos_tol = tolerance if tolerance is not None else 0.05
                self._precision_correction(goal_pose, pos_tol, 0.05)
                return True

            return False

        except KeyboardInterrupt:
            self.__node.get_logger().warn("KeyboardInterrupt: Canceling navigation goal...")
            if self.__current_goal_handle:
                cancel_future = self.__current_goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self.__node, cancel_future, timeout_sec=5.0)
                self.__node.get_logger().info("Navigation goal canceled.")
            self.__current_goal_handle = None
            return False
        except Exception as e:
            self.__node.get_logger().error(f"Navigation error: {str(e)}")
            return False

    def _precision_correction(self, goal_pose: PoseStamped, pos_tol: float, yaw_tol: float):
        imu_data = {'raw_yaw': None, 'yaw_offset': None}
        
        def imu_cb(msg: Imu):
            q = msg.orientation
            _, _, y = euler_from_quaternion([q.x, q.y, q.z, q.w])
            imu_data['raw_yaw'] = y
            
        self.__node.get_logger().debug("Starting precision correction using TF and /imu...")
        
        with TemporarySubscriber(self.__node, Imu, '/imu', 10, imu_cb):
            start_time = time.time()
            gx = goal_pose.pose.position.x
            gy = goal_pose.pose.position.y
            gq = goal_pose.pose.orientation
            _, _, gyaw = euler_from_quaternion([gq.x, gq.y, gq.z, gq.w])
            
            while rclpy.ok() and time.time() - start_time < 5.0:
                rclpy.spin_once(self.__node, timeout_sec=0.05)
                
                current_pose = self.get_current_pose(simple=True)
                if current_pose is None:
                    continue
                    
                cx, cy, cyaw = current_pose
                
                # Fuse high-freq IMU with low-freq TF
                if imu_data['raw_yaw'] is not None:
                    if imu_data['yaw_offset'] is None:
                        imu_data['yaw_offset'] = cyaw - imu_data['raw_yaw']
                    
                    current_yaw = imu_data['raw_yaw'] + imu_data['yaw_offset']
                    diff = cyaw - current_yaw
                    while diff > math.pi: diff -= 2*math.pi
                    while diff < -math.pi: diff += 2*math.pi
                    imu_data['yaw_offset'] += diff * 0.1
                else:
                    current_yaw = cyaw
                
                ex = gx - cx
                ey = gy - cy
                
                lex = ex * math.cos(current_yaw) + ey * math.sin(current_yaw)
                ley = -ex * math.sin(current_yaw) + ey * math.cos(current_yaw)
                
                eyaw = gyaw - current_yaw
                while eyaw > math.pi: eyaw -= 2.0 * math.pi
                while eyaw < -math.pi: eyaw += 2.0 * math.pi
                
                dist = math.sqrt(ex**2 + ey**2)
                
                if dist <= pos_tol and abs(eyaw) <= yaw_tol:
                    self.__node.get_logger().debug(f"Precision correction completed. Dist: {dist:.3f}, YawErr: {eyaw:.3f}")
                    break
                    
                def apply_min_max(err, p_gain, min_v, max_v, deadband):
                    if abs(err) < deadband: return 0.0
                    v = err * p_gain
                    if abs(v) < min_v:
                        return math.copysign(min_v, v)
                    return math.copysign(min(abs(v), max_v), v)
                
                vx = apply_min_max(lex, 2.0, 0.2, 0.2, pos_tol/2.0)
                vy = apply_min_max(ley, 2.0, 0.2, 0.2, pos_tol/2.0)
                vw = apply_min_max(eyaw, 1.5, 0.15, 1.0, yaw_tol/2.0)
                
                cmd = Twist()
                cmd.linear.x = vx
                cmd.linear.y = vy
                cmd.angular.z = vw
                self.__cmd_vel_pub.publish(cmd)
                
            self.__cmd_vel_pub.publish(Twist()) # Stop


    def move_abs(self, x: float = 0.0, y: float = 0.0, yaw: float = 0.0, tolerance: float = 0.05, reference_frame: str = 'map', wait: bool = True) -> bool:
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
            目標からの許容誤差半径(m)。指定値以内に到達すれば終了する。
        reference_frame : str, optional
            座標系の基準フレーム。デフォルトは 'map'。
        wait : bool, optional
            移動完了まで処理をブロックするかどうか。デフォルトは True。

        Returns
        -------
        bool
            ナビゲーションが成功した場合は True、失敗・キャンセルされた場合は False。
        """
        pose = PoseStamped()
        pose.header.frame_id = reference_frame
        pose.header.stamp = self.__node.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        
        q = quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        return self.move_to_pose(pose, tolerance=tolerance, reference_frame=reference_frame, wait=wait)


    def move_rel(self, x: float = 0.0, y: float = 0.0, yaw: float = 0.0, tolerance: float = 0.05, wait: bool = True) -> bool:
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
            目標からの許容誤差半径(m)。指定値以内に到達すれば終了する。
        wait : bool, optional
            移動完了まで処理をブロックするかどうか。デフォルトは True。

        Returns
        -------
        bool
            ナビゲーションが成功した場合は True、失敗・キャンセルされた場合は False。
        """
        current_pose = self.get_current_pose(simple=True)
        if current_pose is None:
            self.__node.get_logger().error("Could not get current pose for relative movement")
            return False
            
        current_x, current_y, current_yaw = current_pose

        new_x = current_x + x * math.cos(current_yaw) - y * math.sin(current_yaw)
        new_y = current_y + x * math.sin(current_yaw) + y * math.cos(current_yaw)
        new_yaw = current_yaw + yaw

        return self.move_abs(x=new_x, y=new_y, yaw=new_yaw, tolerance=tolerance, reference_frame='map', wait=wait)


    def set_initialpose(self, pose, reference_frame: str = 'map'):
        """
        ロボットの初期位置（Initial Pose）を設定する．
        AMCL等のローカライゼーションノードに対して /initialpose トピックをパブリッシュする。

        Parameters
        ----------
        pose : Pose or PoseStamped or list of float
            設定する初期姿勢。リストの場合は [x, y, yaw] の形式。
        reference_frame : str, optional
            基準となる座標フレーム。デフォルトは 'map'。

        Returns
        -------
        None
        """
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.__node.get_clock().now().to_msg()
        msg.header.frame_id = reference_frame

        if isinstance(pose, PoseStamped):
            msg.pose.pose = pose.pose
            msg.header.frame_id = pose.header.frame_id
        elif isinstance(pose, Pose):
            msg.pose.pose = pose
        elif isinstance(pose, list) and len(pose) == 3:
            msg.pose.pose.position.x = float(pose[0])
            msg.pose.pose.position.y = float(pose[1])
            msg.pose.pose.position.z = 0.0
            
            q = quaternion_from_euler(0, 0, pose[2])
            msg.pose.pose.orientation.x = q[0]
            msg.pose.pose.orientation.y = q[1]
            msg.pose.pose.orientation.z = q[2]
            msg.pose.pose.orientation.w = q[3]
        else:
            self.__node.get_logger().error("Invalid pose format for set_initialpose. Use Pose, PoseStamped, or [x, y, yaw].")
            return

        # Covariance - typical reasonable defaults for a manual reset
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.06853891945200942

        self.__initial_pose_pub.publish(msg)
        self.__node.get_logger().info(f"Published initial pose to /initialpose in frame: {msg.header.frame_id}")


class ArmControl():
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
        self.__node = node
        self.__current_goal_handles = []

        # TF2 Setup
        self.__tf_buffer = tf_buffer or Buffer()
        self.__tf_listener = TransformListener(self.__tf_buffer, self.__node)

        # MoveGroup Action Client
        self.__move_group_client = ActionClient(self.__node, MoveGroup, "/move_action")
        
        if not self.__move_group_client.wait_for_server(timeout_sec=wait_time):
            self.__node.get_logger().error("MoveGroup action server not available...")

        # IK Service Client
        self.__ik_cli = self.__node.create_client(GetPositionIK, "/compute_ik")
        if not self.__ik_cli.wait_for_service(timeout_sec=wait_time):
            self.__node.get_logger().error("IK service /compute_ik not available...")

        # Joint states storage
        self.__joint_states = {}
        self.__joint_sub = self.__node.create_subscription(
            JointState,
            "/joint_states",
            self.__joint_state_callback,
            10
        )

    def __joint_state_callback(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self.__joint_states[name] = pos

    def get_current_joints_pose(self, planning_group: str = 'upper_body'):
        """
        現在の各ジョイントの角度を取得します。

        Returns
        -------
        dict
            各ジョイント名をキー、角度(rad)を値とする辞書
        """
        return self.__joint_states.copy()

    def get_current_pose(self, simple: bool = False, planning_group: str = 'upper_body', reference_frame: str = 'base_link'):
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
        tip_link = "left_amazing_hand" if "left" in planning_group else "right_amazing_hand"
            
        start_time = self.__node.get_clock().now()
        while rclpy.ok() and (self.__node.get_clock().now() - start_time).nanoseconds < 2e9: # 2s timeout
            rclpy.spin_once(self.__node, timeout_sec=0.1)
            try:
                transform = self.__tf_buffer.lookup_transform(reference_frame, tip_link, rclpy.time.Time())
                pos = transform.transform.translation
                rot = transform.transform.rotation
                
                if simple:
                    (roll, pitch, yaw) = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
                    return [pos.x, pos.y, pos.z, roll, pitch, yaw]
                else:
                    pose = PoseStamped()
                    pose.header.frame_id = reference_frame
                    pose.header.stamp = self.__node.get_clock().now().to_msg()
                    pose.pose.position.x = pos.x
                    pose.pose.position.y = pos.y
                    pose.pose.position.z = pos.z
                    pose.pose.orientation = rot
                    return pose
            except Exception as e:
                self.__node.get_logger().debug(f"TF Lookup failed for {tip_link}: {str(e)}")
                continue
        return None

    def move_to_pose(self, pose, planning_group: str = 'upper_body', wait: bool = True, **kwargs) -> bool:
        """
        与えられた目標姿勢に向けてエンドエフェクタを自律移動させる．

        Parameters
        ----------
        pose : PoseStamped or Pose
            目標とする姿勢情報。
        planning_group : str, optional
            使用するプランニンググループ名。デフォルトは 'upper_body'。
        wait : bool, optional
            移動完了まで処理をブロックするかどうか。デフォルトは True。
        **kwargs
            planning_attempts: 計画試行回数 (default: 10)
            planning_time: 許容計画時間 (default: 5.0)

        Returns
        -------
        bool
            動作が成功した場合は True、失敗した場合は False。
        """
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = planning_group
        goal_msg.request.num_planning_attempts = kwargs.get('planning_attempts', 10)
        goal_msg.request.allowed_planning_time = kwargs.get('planning_time', 5.0)
        
        # Determine tip link (assuming standard names for G1)
        tip_link = "left_amazing_hand" if "left" in planning_group else "right_amazing_hand"
        
        # Formulate goal constraints
        target_pose = pose
        if isinstance(pose, Pose):
            target_pose = PoseStamped()
            target_pose.header.frame_id = "base_link"
            target_pose.pose = pose
            
        l_pc, l_oc = self._create_pose_constraints(target_pose, tip_link)
        goal_msg.request.goal_constraints.append(Constraints(
            position_constraints=[l_pc],
            orientation_constraints=[l_oc]
        ))
        
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
        box.dimensions = [0.01, 0.01, 0.01] # 1cm tolerance
        pc.constraint_region.primitives.append(box)
        pc.weight = 1.0
        
        # Orientation Constraint
        oc = OrientationConstraint()
        oc.header.frame_id = target_pose.header.frame_id
        oc.link_name = tip_link
        oc.orientation = target_pose.pose.orientation
        oc.absolute_x_axis_tolerance = 0.1 # 0.1rad tolerance
        oc.absolute_y_axis_tolerance = 0.1
        oc.absolute_z_axis_tolerance = 0.1
        oc.weight = 1.0
        
        return pc, oc

    def _solve_ik(self, pose_stamped: PoseStamped, group_name: str) -> dict:
        """
        MoveIt の /compute_ik サービスを使用して特定のグループの逆運動学を解く。
        """
        req = GetPositionIK.Request()
        req.ik_request.group_name = group_name
        req.ik_request.pose_stamped = pose_stamped
        req.ik_request.timeout.sec = 1
        
        # 現在の関節状態を反映
        if self.__joint_states:
            req.ik_request.robot_state.joint_state.name = list(self.__joint_states.keys())
            req.ik_request.robot_state.joint_state.position = list(self.__joint_states.values())
        
        future = self.__ik_cli.call_async(req)
        # IKサービスは比較的速いので短めのタイムアウトで spin
        rclpy.spin_until_future_complete(self.__node, future, timeout_sec=2.0)
        
        if future.done():
            res = future.result()
            if res.error_code.val == MoveItErrorCodes.SUCCESS:
                return dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
            else:
                self.__node.get_logger().error(f"IK failed for {group_name} with error: {res.error_code.val}")
        return None

    def move_abs(self, x: float = 0.0, y: float = 0.0, z: float = 0.0, roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0, planning_group: str = 'upper_body', wait: bool = True, reference_frame: str = 'base_link', **kwargs) -> bool:
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
        pose.header.stamp = self.__node.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        
        q = quaternion_from_euler(roll, pitch, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        return self.move_to_pose(pose, planning_group=planning_group, wait=wait, **kwargs)

    def move_rel(self, x: float = 0.0, y: float = 0.0, z: float = 0.0, roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0, planning_group: str = 'upper_body', wait: bool = True, **kwargs) -> bool:
        """
        現在位置姿勢からの相対移動．
        """
        current_pose = self.get_current_pose(simple=True, planning_group=planning_group)
        if current_pose is None:
            self.__node.get_logger().error("Could not get current pose for relative movement")
            return False
            
        cx, cy, cz, croll, cpitch, cyaw = current_pose

        new_x = cx + x
        new_y = cy + y
        new_z = cz + z
        
        q_curr = tf_transformations.quaternion_from_euler(croll, cpitch, cyaw)
        q_rel = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        q_new = tf_transformations.quaternion_multiply(q_curr, q_rel)
        
        (nr, np, ny) = euler_from_quaternion(q_new)

        return self.move_abs(x=new_x, y=new_y, z=new_z, roll=nr, pitch=np, yaw=ny, planning_group=planning_group, wait=wait, **kwargs)

    def move_dual_abs(self, lx=0.0, ly=0.0, lz=0.0, lr=0.0, lp=0.0, lyaw=0.0,
                      rx=0.0, ry=0.0, rz=0.0, rr=0.0, rp=0.0, ryaw=0.0,
                      wait=True, reference_frame='base_link', **kwargs) -> bool:
        """
        左右の手の目標座標を同時に指定して移動させる。
        planning_group は強制的に 'upper_body' が使用されます。
        """
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'upper_body'
        goal_msg.request.num_planning_attempts = kwargs.get('planning_attempts', 10)
        goal_msg.request.allowed_planning_time = kwargs.get('planning_time', 5.0)

        # Left Arm Pose
        l_pose = PoseStamped()
        l_pose.header.frame_id = reference_frame
        l_pose.header.stamp = self.__node.get_clock().now().to_msg()
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
        r_pose.header.stamp = self.__node.get_clock().now().to_msg()
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
            self.__node.get_logger().error(f"Dual IK solving failed. L: {'Success' if l_joints else 'Fail'}, R: {'Success' if r_joints else 'Fail'}")
            return False
        
        self.__node.get_logger().info("Dual IK solved successfully. Proceeding with joint-space planning.")

        # Build joint constraints for upper_body
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'upper_body'
        goal_msg.request.num_planning_attempts = kwargs.get('planning_attempts', 10)
        goal_msg.request.allowed_planning_time = kwargs.get('planning_time', 5.0)

        constraints = Constraints()
        # combine joints (L & R)
        target_joints = {**l_joints, **r_joints}
        
        # Define target joints for upper_body (Waist + Both Arms)
        upper_body_joints = [
            "waist_yaw_joint",
            "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint",
            "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint"
        ]

        for j_name in upper_body_joints:
            jc = JointConstraint()
            jc.joint_name = j_name
            # IK結果があればそれを使用、なければ現在値を保持
            jc.position = target_joints.get(j_name, self.__joint_states.get(j_name, 0.0))
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        goal_msg.request.goal_constraints.append(constraints)
        return self._send_move_group_goal(goal_msg, wait)

    def move_dual_rel(self, lx=0.0, ly=0.0, lz=0.0, lr=0.0, lp=0.0, lyaw=0.0,
                      rx=0.0, ry=0.0, rz=0.0, rr=0.0, rp=0.0, ryaw=0.0,
                      wait=True, **kwargs) -> bool:
        """
        現在位置からの左右同時の相対移動。
        """
        l_curr = self.get_current_pose(simple=True, planning_group='arm_left')
        r_curr = self.get_current_pose(simple=True, planning_group='arm_right')
        
        if l_curr is None or r_curr is None:
            self.__node.get_logger().error("Could not get current pose for dual relative movement")
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

        return self.move_dual_abs(lx=nlx, ly=nly, lz=nlz, lr=new_lr, lp=new_lp, lyaw=new_lyaw,
                                  rx=nrx, ry=nry, rz=nrz, rr=new_rr, rp=new_rp, ryaw=new_ryaw,
                                  wait=wait, **kwargs)

    def move_groupstate(self, group_name: str = 'upper_body', group_state: str = 'home', wait: bool = True) -> bool:
        """
        定義済み状態への遷移．
        """
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = group_name
        
        joints = []
        if group_name == "arm_left":
            joints = ["left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint"]
        elif group_name == "arm_right":
            joints = ["right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint"]
        elif group_name == "upper_body":
            joints = ["waist_yaw_joint", "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint", 
                      "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint"]
        else:
            self.__node.get_logger().error(f"Unsupported group: {group_name}")
            return False
            
        constraints = Constraints()
        for j in joints:
            jc = JointConstraint()
            jc.joint_name = j
            jc.position = 0.0 # 'home' assumes all zero
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
            
        goal_msg.request.goal_constraints.append(constraints)
        return self._send_move_group_goal(goal_msg, wait)

    def joint_control(self, rlt: bool = False, wait: bool = True, planning_group: str = 'upper_body', **kwargs) -> bool:
        """
        ジョイント角度制御．
        """
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = planning_group
        
        constraints = Constraints()
        for name, val in kwargs.items():
            jc = JointConstraint()
            jc.joint_name = name
            if rlt:
                jc.position = self.__joint_states.get(name, 0.0) + val
            else:
                jc.position = val
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
            
        goal_msg.request.goal_constraints.append(constraints)
        return self._send_move_group_goal(goal_msg, wait)

    def _send_move_group_goal(self, goal_msg, wait):
        self.__node.get_logger().info(f"Sending MoveGroup goal for group: {goal_msg.request.group_name}")
        future = self.__move_group_client.send_goal_async(goal_msg)
        
        if not wait:
            return True
            
        rclpy.spin_until_future_complete(self.__node, future, timeout_sec=10.0)
        if not future.done():
            self.__node.get_logger().error("MoveGroup goal sending timed out")
            return False
            
        handle = future.result()
        if not handle.accepted:
            self.__node.get_logger().error("MoveGroup goal rejected")
            return False
            
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self.__node, result_future)
        result = result_future.result()
        
        if result.result.error_code.val == MoveItErrorCodes.SUCCESS:
            return True
        else:
            self.__node.get_logger().error(f"MoveGroup failed with error code: {result.result.error_code.val}")
            return False

class G1Mic():
    """
    Unitree G1 robot microphone audio receiver class.
    Communicates with mic_server node via ROS 2 services and topics.
    """

    def __init__(
        self,
        node: Node,
        sample_rate: int = 16000,
        channels: int = 1
    ):
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
        self.__node = node
        self.__sample_rate = sample_rate
        self.__channels = channels
        
        self.__audio_buffer = []
        self.__buffer_lock = threading.Lock()
        
        # Service client for control
        self.__mic_rec_cli = self.__node.create_client(SetBool, 'mic_rec')
        
        # Subscriber for audio data
        self.__audio_sub = self.__node.create_subscription(
            Int16MultiArray,
            '/audio/raw',
            self.__audio_callback,
            10
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
            self.__node.get_logger().error('mic_server (mic_rec service) is not running.')
            raise RuntimeError('mic_server is not running.')

        # 録音開始のリクエスト
        req = SetBool.Request()
        req.data = True
        
        future = self.__mic_rec_cli.call_async(req)
        rclpy.spin_until_future_complete(self.__node, future, timeout_sec=2.0)
        
        if future.done():
            res = future.result()
            if res.success:
                self.__node.get_logger().info("Microphone recording enabled via mic_server.")
            else:
                self.__node.get_logger().error(f"Failed to enable recording: {res.message}")
        else:
            self.__node.get_logger().error("Service call timed out.")

        with self.__buffer_lock:
            self.__audio_buffer = [] # Clear buffer on start

        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """
        コンテキストマネージャの終了。音声配信を無効化します。
        """
        req = SetBool.Request()
        req.data = False
        
        future = self.__mic_rec_cli.call_async(req)
        # We don't necessarily need to wait long here, but it's good practice
        rclpy.spin_until_future_complete(self.__node, future, timeout_sec=1.0)
        
        self.__node.get_logger().info("Microphone recording disabled.")

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
            self.__audio_buffer = [] # Clear buffer after reading
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
            self.__node.get_logger().warn("No audio data to save.")
            return False

        try:
            with wave.open(file_path, 'wb') as wf:
                wf.setnchannels(self.__channels)
                wf.setsampwidth(2) # 16-bit
                wf.setframerate(self.__sample_rate)
                wf.writeframes(audio_data.tobytes())
            
            self.__node.get_logger().info(f"Successfully saved audio to {file_path}")
            return True
        except Exception as e:
            self.__node.get_logger().error(f"Failed to save WAV file: {e}")
            return False

