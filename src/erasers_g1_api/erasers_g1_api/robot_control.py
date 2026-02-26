#!/usr/bin/env python3
from rclpy.node import Node
import rclpy

# msgs
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped, Pose
from sensor_msgs.msg import Imu
from amazing_hand_interfaces.srv import HandCommand
from g1_srvs.srv import MoveServo
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

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
from g1_srvs.action import CartesianEE
from std_srvs.srv import SetBool, Trigger
import tf_transformations

class G1Control():
    def __init__(self, node:Node):
        self.__node = node

        self.__servo_cli = self.__node.create_client(MoveServo, '/move_servo')
        self.__hand_cli = self.__node.create_client(HandCommand, '/hand_command')

        while not self.__servo_cli.wait_for_service(timeout_sec=5.0) or not self.__hand_cli.wait_for_service(timeout_sec=5.0):
            self.__node.get_logger().error('Robot Service Servers are not running ...')
            raise RuntimeError('Robot Service Servers are not running ...')


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


    def move_to_pose(self, pose, tolerance: float = None, reference_frame: str = 'map', wait: bool = True) -> bool:
        """
        与えられた目標姿勢に基づいてロボットを自律移動させる．
        すべてのナビゲーションの中核となるメソッドであり、KeyboardInterrupt 発生時には即座にアクションをキャンセルする。

        Parameters
        ----------
        pose : PoseStamped or Pose
            目標とする姿勢情報。Pose メッセージの場合、reference_frame の座標系基準として扱われる。
        tolerance : float, optional
            目標から指定された距離(m)以内に到達した場合、その時点でナビゲーションを成功として終了する。
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


    def move_abs(self, x: float = 0.0, y: float = 0.0, yaw: float = 0.0, tolerance: float = None, reference_frame: str = 'map', wait: bool = True) -> bool:
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


    def move_rel(self, x: float = 0.0, y: float = 0.0, yaw: float = 0.0, tolerance: float = None, wait: bool = True) -> bool:
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
            アクションサーバーやサービスの接続待機時間(秒)。デフォルトは 5。
        tf_buffer : Buffer, optional
            TF2 バッファオブジェクト。None の場合は新規作成。デフォルトは None。
        """
        self.__node = node
        self.__current_goal_handles = []

        # TF2 Setup
        self.__tf_buffer = tf_buffer or Buffer()
        self.__tf_listener = TransformListener(self.__tf_buffer, self.__node)

        # Action Clients Setup
        self.__left_action_client = ActionClient(self.__node, CartesianEE, "/left_arm/cartesian_planner")
        self.__right_action_client = ActionClient(self.__node, CartesianEE, "/right_arm/cartesian_planner")
        
        # Service Clients Setup
        self.__enable_srv_client = self.__node.create_client(SetBool, "/enable_ee_control")
        self.__init_pose_srv_client = self.__node.create_client(Trigger, "/set_init_pose")

        if not self.__left_action_client.wait_for_server(timeout_sec=wait_time) or \
           not self.__right_action_client.wait_for_server(timeout_sec=wait_time):
            self.__node.get_logger().error("Cartesian planner action servers not available...")

    def init_pose(self, wait: bool = True) -> bool:
        """
        アームの現在位置（IKソリューション）をニュートラルな初期（ゼロ）姿勢にリセットする．
        /set_init_pose サービスを呼び出します。

        Parameters
        ----------
        wait : bool, optional
            遷移完了まで待機するかどうか。デフォルトは True。

        Returns
        -------
        bool
            成功時は True、失敗時は False。
        """
        if not self.__init_pose_srv_client.wait_for_service(timeout_sec=2.0):
            self.__node.get_logger().error("Init pose service not available")
            return False
            
        req = Trigger.Request()
        future = self.__init_pose_srv_client.call_async(req)
        rclpy.spin_until_future_complete(self.__node, future, timeout_sec=5.0)
        
        if future.done():
            success = future.result().success
            if success and wait:
                start_req = time.time()
                while time.time() - start_req < 5.0 and rclpy.ok():
                    rclpy.spin_once(self.__node, timeout_sec=0.1)
            return success
        return False

    def enable(self, state: bool, wait: bool = True) -> bool:
        """
        アーム制御の有効/無効を切り替える．
        C++側で自動的に遷移が行われるため、Python側では状態の送信と待機のみを行います。
        /enable_ee_control サービスを呼び出します。

        Parameters
        ----------
        state : bool
            True で制御有効化、False で無効化（現在姿勢の維持）。
        wait : bool, optional
            遷移完了まで待機するかどうか。デフォルトは True。

        Returns
        -------
        bool
            成功時は True、失敗時は False。
        """
        if not self.__enable_srv_client.wait_for_service(timeout_sec=2.0):
            self.__node.get_logger().error("Enable service not available")
            return False

        req = SetBool.Request()
        req.data = state
        future = self.__enable_srv_client.call_async(req)
        rclpy.spin_until_future_complete(self.__node, future, timeout_sec=5.0)
        
        if future.done():
            success = future.result().success
            if state and success and wait:
                start_req = time.time()
                while time.time() - start_req < 5.0 and rclpy.ok():
                    rclpy.spin_once(self.__node, timeout_sec=0.1)
            return success
        return False

    def get_current_pose(self, simple: bool = False, arm: str = 'left', reference_frame: str = 'base_link'):
        """
        指定されたエンドエフェクタの現在位置姿勢を取得する．

        Parameters
        ----------
        simple : bool, optional
            True の場合、[x, y, z, roll, pitch, yaw] の1次元リストとして現在位置を出力する。
            False の場合、PoseStamped 型で現在位置を出力する。デフォルトは False。
        arm : str, optional
            'left', 'right', 'both' のいずれかを指定する。デフォルトは 'left'。
        reference_frame : str, optional
            基準となる座標フレーム。デフォルトは 'base_link'。

        Returns
        -------
        PoseStamped or list of float or list
            arm='both' の場合は、左右両方の姿勢データのリストを返す（例: `[left_pose, right_pose]`）。
        """
        if arm == 'both':
            return [
                self.get_current_pose(simple, 'left', reference_frame),
                self.get_current_pose(simple, 'right', reference_frame)
            ]

        while rclpy.ok():
            rclpy.spin_once(self.__node, timeout_sec=0.1)
            try:
                # ターゲットフレームはIK制御で使用される手先の基準フレーム
                target_frame = 'left_wrist_roll_rubber_hand' if arm == 'left' else 'right_wrist_roll_rubber_hand'
                transform = self.__tf_buffer.lookup_transform(reference_frame, target_frame, rclpy.time.Time())
                
                t_x = transform.transform.translation.x
                t_y = transform.transform.translation.y
                t_z = transform.transform.translation.z
                q = transform.transform.rotation

                # Apply 0.2m offset for true IK target (Offset along local X)
                T_mat = tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
                offset = [0.2, 0.0, 0.0, 1.0]
                world_offset = [sum(a * b for a, b in zip(row, offset)) for row in T_mat]

                target_x = t_x + world_offset[0]
                target_y = t_y + world_offset[1]
                target_z = t_z + world_offset[2]

                if simple:
                    (roll, pitch, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
                    return [target_x, target_y, target_z, roll, pitch, yaw]
                else:
                    pose = PoseStamped()
                    pose.header.frame_id = reference_frame
                    pose.header.stamp = self.__node.get_clock().now().to_msg()
                    pose.pose.position.x = target_x
                    pose.pose.position.y = target_y
                    pose.pose.position.z = target_z
                    pose.pose.orientation = q
                    return pose
            except Exception as e:
                self.__node.get_logger().debug(f"TF Lookup failed for {arm} arm: {str(e)}")
                continue

    def move_to_pose(self, pose, duration: float = 2.0, arm: str = 'left', wait: bool = True) -> bool:
        """
        与えられた目標姿勢に向けてエンドエフェクタを自律移動させる．
        KeyboardInterrupt 発生時には即座にアクションをキャンセルする。

        Parameters
        ----------
        pose : PoseStamped or list of PoseStamped or Pose or list of Pose
            目標とする姿勢情報。arm='both' の場合、左右それぞれの Goal が格納された長さ2のリストを受け付ける
            （単一のオブジェクトが渡された場合は両腕を同じ目的座標に移動させる）。
        duration : float, optional
            移動にかける時間(秒)。デフォルトは 2.0。
        arm : str, optional
            'left', 'right', 'both' のいずれか。デフォルトは 'left'。
        wait : bool, optional
            移動完了まで処理をブロックするかどうか。デフォルトは True。

        Returns
        -------
        bool
            動作が成功した場合は True、失敗・キャンセルされた場合は False。
        """
        self.__current_goal_handles.clear()

        # Handle Pose to PoseStamped conversion
        def cast_to_stamped(p):
            if isinstance(p, Pose):
                ps = PoseStamped()
                ps.header.frame_id = 'base_link'
                ps.header.stamp = self.__node.get_clock().now().to_msg()
                ps.pose = p
                return ps
            return p

        if arm == 'both':
            if isinstance(pose, list) and len(pose) == 2:
                pose_left = cast_to_stamped(pose[0])
                pose_right = cast_to_stamped(pose[1])
            else:
                pose_left = cast_to_stamped(pose)
                pose_right = cast_to_stamped(pose)

            future_l = self._send_goal('left', pose_left, duration)
            future_r = self._send_goal('right', pose_right, duration)
            return self._wait_for_futures([future_l, future_r], wait, duration)
        else:
            p = cast_to_stamped(pose)
            future = self._send_goal(arm, p, duration)
            return self._wait_for_futures([future], wait, duration)

    def _send_goal(self, arm: str, pose: PoseStamped, duration: float):
        client = self.__left_action_client if arm == 'left' else self.__right_action_client
        goal_msg = CartesianEE.Goal()
        goal_msg.pose = pose
        goal_msg.duration = duration
        return client.send_goal_async(goal_msg)

    def _wait_for_futures(self, futures: list, wait: bool, duration: float) -> bool:
        if not wait:
            # wait_for_futures non-blocking is just a small wait to ensure goal is accepted
            try:
                start_req = time.time()
                while time.time() - start_req < 0.2 and rclpy.ok():
                    rclpy.spin_once(self.__node, timeout_sec=0.1)
            except KeyboardInterrupt:
                pass
            return True

        # Blocking wait mechanism
        try:
            handles = []
            for f in futures:
                rclpy.spin_until_future_complete(self.__node, f, timeout_sec=10.0)
                if f.done():
                    handle = f.result()
                    if handle.accepted:
                        handles.append(handle)
                        self.__current_goal_handles.append(handle)

            if not handles:
                self.__node.get_logger().error("All Action goals were rejected or timed out")
                return False

            result_futures = [h.get_result_async() for h in handles]
            
            # Spin until all results are in, or KeyboardInterrupt occurs
            start_time = time.time()
            all_done = False
            while rclpy.ok() and not all_done:
                all_done = all([rf.done() for rf in result_futures])
                rclpy.spin_once(self.__node, timeout_sec=0.1)
                
                # Fallback timeout in case the action server gets stuck
                if time.time() - start_time > duration + 5.0:
                    self.__node.get_logger().warn("Action server timeout.")
                    break
            
            success = True
            for rf in result_futures:
                if rf.done() and rf.result().status != GoalStatus.STATUS_SUCCEEDED:
                    success = False
                    
            return success

        except KeyboardInterrupt:
            self.__node.get_logger().warn("KeyboardInterrupt: Canceling arm trajectory...")
            for h in self.__current_goal_handles:
                cancel_f = h.cancel_goal_async()
                rclpy.spin_until_future_complete(self.__node, cancel_f, timeout_sec=3.0)
            self.__current_goal_handles.clear()
            return False
            
    def move_abs(self, x: float = 0.0, y: float = 0.0, z: float = 0.0, roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0, duration: float = 2.0, reference_frame: str = 'base_link', arm: str = 'left', wait: bool = True) -> bool:
        """
        基準フレームでの絶対座標を指定してアームを移動させる．
        指定されたエンドエフェクタの位置が範囲外でも，アームはその方向へ最大限伸ばすように動作する（C++側での自動緩和）。

        Parameters
        ----------
        x, y, z : float
            目標位置の座標。
        roll, pitch, yaw : float
            目標姿勢のオイラー角（ラジアン）。
        duration : float, optional
            移動時間。デフォルトは 2.0。
        reference_frame : str, optional
            基準座標系。デフォルトは 'base_link'。
        arm : str, optional
            'left', 'right', 'both' のいずれか。
        wait : bool, optional
            移動完了まで待機するかどうか。

        Returns
        -------
        bool
            成功時は True、失敗時は False。
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

        return self.move_to_pose(pose, duration=duration, arm=arm, wait=wait)

    def move_rel(self, x: float = 0.0, y: float = 0.0, z: float = 0.0, roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0, duration: float = 2.0, arm: str = 'left', wait: bool = True) -> bool:
        """
        現在位置姿勢からの相対座標でアームを移動させる．

        Parameters
        ----------
        x, y, z : float
            現在位置からの相対移動量。
        roll, pitch, yaw : float
            現在姿勢からの相対的な回転量（ラジアン）。
        duration : float, optional
            移動時間。
        arm : str, optional
            対象とするアーム。
        wait : bool, optional
            完了待機の有無。

        Returns
        -------
        bool
            成功時は True、失敗時は False。
        """
        if arm == 'both':
            succ_l = self.move_rel(x, y, z, roll, pitch, yaw, duration, 'left', wait)
            succ_r = self.move_rel(x, y, z, roll, pitch, yaw, duration, 'right', wait)
            return succ_l and succ_r
            
        current_pose = self.get_current_pose(simple=True, arm=arm, reference_frame='base_link')
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
        
        (new_roll, new_pitch, new_yaw) = euler_from_quaternion(q_new)

        return self.move_abs(x=new_x, y=new_y, z=new_z, roll=new_roll, pitch=new_pitch, yaw=new_yaw, duration=duration, reference_frame='base_link', arm=arm, wait=wait)
