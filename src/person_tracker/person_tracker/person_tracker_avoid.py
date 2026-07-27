#!/usr/bin/env python3

# person_tracker.py をベースに、cmd_vel を直接出す代わりに
# machida_navigation の DWA local planner へ「人に向かう経路」を流して
# ローカル障害物回避をさせる版 (アプローチB: NavigationManager を経由しない)。
#
# 出力:
#   /execute_path_plan     (nav_msgs/Path) : ロボット現在地 -> 人の手前 までの直線経路 (map 系)
#   /execute_local_planner (std_msgs/Bool) : DWA を有効化 (検出ロスト時は false で停止)
#
# 認識部 (DBSCAN + パーティクルフィルタ) は person_tracker.py から流用。
# DWA は経路追従 + local_costmap によるローカル回避のみを行うため、
# 大きく塞がれた場合 (max_path_deviation 超) は回避できず停止する点に注意。

import math

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Quaternion
from nav_msgs.msg import Path
from std_msgs.msg import Bool, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from sensor_msgs_py import point_cloud2

from tf2_ros import Buffer, TransformListener, TransformException
from tf2_sensor_msgs import do_transform_cloud

from sklearn.cluster import DBSCAN


def yaw_from_quat(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def quat_from_yaw(yaw):
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class ParticleFilter:
    """CTRV (Constant Turn Rate and Velocity) モデルのパーティクルフィルタ.

    状態ベクトルは [x, y, theta, v, omega]:
        x, y   : 位置 [m]        (base_link 系)
        theta  : 進行方向 [rad]  (x軸正方向が 0, 反時計回りが正)
        v      : 並進速度 [m/s]
        omega  : 旋回レート [rad/s]
    速度と旋回レートを一定と仮定しつつ、向きに沿ってカーブを描く動きを予測できる。
    観測は位置 (x, y) のみで、theta / v / omega はフィルタが内部推定する。
    """

    def __init__(self, num_particles=500):

        self.N = num_particles

        # 列: 0:x 1:y 2:theta 3:v 4:omega
        self.particles = np.zeros((self.N, 5))

        # 初期分布: 前方 1m あたりに位置、向き・速度・旋回はばらつかせて
        # 動き出したときに素早く収束できるようにする。
        self.particles[:, 0] = 1.0                                   # x
        self.particles[:, 1] = 0.0                                   # y
        self.particles[:, 2] = np.random.uniform(-np.pi, np.pi, self.N)  # theta
        self.particles[:, 3] = np.random.uniform(0.0, 1.0, self.N)       # v
        self.particles[:, 4] = np.random.uniform(-0.5, 0.5, self.N)      # omega

        self.weights = np.ones(self.N) / self.N

    def predict(self, dt):

        theta = self.particles[:, 2]
        v = self.particles[:, 3]
        omega = self.particles[:, 4]

        theta_new = theta + omega * dt

        # CTRV の運動方程式。omega が 0 付近だと v/omega が発散するため、
        # 旋回している場合と直進している場合で式を分ける。
        turning = np.abs(omega) > 1e-5

        # --- 旋回時: 円弧に沿って進む ---
        # x += (v/omega) * ( sin(theta+omega*dt) - sin(theta) )
        # y += (v/omega) * (-cos(theta+omega*dt) + cos(theta) )
        # omega=0 の要素は 0除算を避けるため where で 1 に置換 (結果は straight 側で上書き)
        safe_omega = np.where(turning, omega, 1.0)
        dx_turn = (v / safe_omega) * (np.sin(theta_new) - np.sin(theta))
        dy_turn = (v / safe_omega) * (-np.cos(theta_new) + np.cos(theta))

        # --- 直進時: 等速直線 ---
        dx_straight = v * np.cos(theta) * dt
        dy_straight = v * np.sin(theta) * dt

        self.particles[:, 0] += np.where(turning, dx_turn, dx_straight)
        self.particles[:, 1] += np.where(turning, dy_turn, dy_straight)
        self.particles[:, 2] = theta_new

        # プロセスノイズ。各状態に平均0のガウス雑音を足してばらつかせ、
        # 人の予測しきれない動き (加減速・向き変え) を表現する。
        noise = np.random.normal(
            0.0,
            [0.2, 0.2, 0.05, 0.1, 0.2],
            size=(self.N, 5)
        )

        self.particles += noise

        # theta を [-pi, pi] に正規化
        self.particles[:, 2] = np.arctan2(
            np.sin(self.particles[:, 2]),
            np.cos(self.particles[:, 2])
        )

    def update(self, observation):

        # 観測 (x, y) との距離が近いパーティクルほど高い重みを与える。
        dist = np.linalg.norm(
            self.particles[:, :2] - observation,
            axis=1
        )

        sigma = 0.3

        self.weights = np.exp(
            -0.5 * (dist / sigma) ** 2
        )

        self.weights += 1e-300
        self.weights /= np.sum(self.weights)

    def resample(self):

        idx = np.random.choice(
            self.N,
            self.N,
            p=self.weights
        )

        self.particles = self.particles[idx]

        self.weights.fill(1.0 / self.N)

    def estimate(self):

        # x, y, v, omega は単純平均、theta は角度なので円周平均をとる。
        mean = np.mean(self.particles, axis=0)
        mean[2] = np.arctan2(
            np.mean(np.sin(self.particles[:, 2])),
            np.mean(np.cos(self.particles[:, 2]))
        )
        return mean


class PersonTrackerAvoid(Node):

    def __init__(self):

        super().__init__('person_tracker_avoid')

        # 認識用の変換先座標系 (人の位置はまずこの系で推定する)
        self.target_frame = 'base_link'

        # 経路 / 自己位置に使う座標系 (DWA local planner と一致させる)
        self.map_frame = 'map'
        self.robot_base_frame = 'base_footprint'

        # -------------------
        # ROI (base_link 座標系での探索範囲 [m])
        #   x: ロボット前方が正
        #   y: ロボット左方向が正 (左右対称に探索)
        #   z: 上方向が正 (地面〜頭上)
        # -------------------
        self.roi_x_min = 0.3    # 前方このより手前は無視 (足元・本体)
        self.roi_x_max = 1.5    # 前方このより遠くは無視
        self.roi_y_abs = 0.8    # 左右それぞれこの範囲まで
        self.roi_z_min = 0.3    # これより低い点は無視 (地面の下)
        self.roi_z_max = 2.0    # これより高い点は無視 (天井など)

        # tf2 (ポイントクラウドを base_link に変換 / 人とロボットを map に変換)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(
            PointCloud2,
            "/utlidar/cloud_livox_mid360",
            self.pointcloud_callback,
            10
        )

        self.pub = self.create_publisher(
            PointStamped,
            '/tracked_person',
            10
        )

        self.viz_pub = self.create_publisher(
            MarkerArray,
            '/tracker/markers',
            10
        )

        # --- DWA local planner への出力 ---
        # path は DWA 側が QoS(1).transient_local().reliable() で購読しているので合わせる。
        path_qos = QoSProfile(depth=1)
        path_qos.reliability = QoSReliabilityPolicy.RELIABLE
        path_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        exec_qos = QoSProfile(depth=1)
        exec_qos.reliability = QoSReliabilityPolicy.RELIABLE

        self.path_pub = self.create_publisher(
            Path,
            '/execute_path_plan',
            path_qos
        )

        self.execute_pub = self.create_publisher(
            Bool,
            '/execute_local_planner',
            exec_qos
        )

        self.pf = ParticleFilter()

        self.last_time = self.get_clock().now()

        # -------------------
        # 追従パラメータ
        # -------------------
        self.follow_distance = 0.5    # 人の手前これだけ残してゴールを置く [m]
        self.path_resolution = 0.1    # 経路点の間隔 [m]
        self.detection_timeout = 0.5  # この時間検出が無ければ DWA を止める [s]

        # 最新の推定位置 (base_link 系)。未検出のうちは None
        self.last_estimate = None
        self.last_estimate_time = self.get_clock().now()

        # DWA を止めている間に何度も false を送らないためのフラグ
        self.executing = False

        # 一定周期で経路を出す制御ループ (10Hz)
        self.control_timer = self.create_timer(0.1, self.control_loop)

    def pointcloud_callback(self, msg):

        now = self.get_clock().now()

        dt = (
            now - self.last_time
        ).nanoseconds * 1e-9

        self.last_time = now

        # -------------------
        # base_link 座標系へ変換
        # -------------------

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                rclpy.time.Time(),          # 利用可能な最新の変換を使う
                timeout=Duration(seconds=0.1)
            )
        except TransformException as ex:
            self.get_logger().warn(
                f"transform {msg.header.frame_id} -> "
                f"{self.target_frame} failed: {ex}"
            )
            return

        cloud = do_transform_cloud(msg, transform)

        points = []

        for p in point_cloud2.read_points(
            cloud,
            field_names=("x", "y", "z"),
            skip_nans=True
        ):
            points.append([p[0], p[1], p[2]])

        if len(points) < 10:
            return

        points = np.array(points)

        # -------------------
        # ROI (探索範囲内の点だけ残す)
        # -------------------

        x = points[:, 0]
        y = points[:, 1]
        z = points[:, 2]

        in_front = (x > self.roi_x_min) & (x < self.roi_x_max)
        in_side = np.abs(y) < self.roi_y_abs
        in_height = (z > self.roi_z_min) & (z < self.roi_z_max)

        in_roi = in_front & in_side & in_height

        points = points[in_roi]

        if len(points) < 10:
            return

        # -------------------
        # DBSCAN
        # -------------------

        clustering = DBSCAN(
            eps=0.15,
            min_samples=5
        ).fit(points[:, :2])

        labels = clustering.labels_

        best_cluster = None
        best_size = 0

        for label in set(labels):

            if label == -1:
                continue

            cluster = points[labels == label]

            if len(cluster) > best_size:

                best_size = len(cluster)
                best_cluster = cluster

        if best_cluster is None:
            return

        observation = np.mean(
            best_cluster[:, :2],
            axis=0
        )

        # -------------------
        # Particle Filter
        # -------------------

        self.pf.predict(dt)

        self.pf.update(observation)

        self.pf.resample()

        state = self.pf.estimate()

        # -------------------
        # Publish
        # -------------------

        msg_out = PointStamped()

        msg_out.header = cloud.header

        msg_out.point.x = float(state[0])
        msg_out.point.y = float(state[1])
        msg_out.point.z = 0.0

        self.pub.publish(msg_out)

        # 制御ループ用に最新の推定位置を保存 (base_link 系)
        self.last_estimate = (float(state[0]), float(state[1]))
        self.last_estimate_time = now

        self.publish_markers(cloud.header, observation, state)

        self.get_logger().info(
            f"target=({state[0]:.2f}, {state[1]:.2f})"
        )

    def publish_execute(self, value):

        msg = Bool()
        msg.data = value
        self.execute_pub.publish(msg)
        self.executing = value

    def control_loop(self):

        # まだ一度も検出していなければ DWA を止める
        if self.last_estimate is None:
            if self.executing:
                self.publish_execute(False)
            return

        # 検出が古すぎる (見失った) 場合は安全のため DWA を止める
        age = (
            self.get_clock().now() - self.last_estimate_time
        ).nanoseconds * 1e-9

        if age > self.detection_timeout:
            if self.executing:
                self.publish_execute(False)
            return

        # --- ロボットの現在地 (map 系) ---
        try:
            robot_tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1)
            )
        except TransformException as ex:
            self.get_logger().warn(
                f"transform {self.map_frame} -> "
                f"{self.robot_base_frame} failed: {ex}"
            )
            return

        rx = robot_tf.transform.translation.x
        ry = robot_tf.transform.translation.y

        # --- 人の位置を base_link -> map に変換 ---
        try:
            person_tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.target_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1)
            )
        except TransformException as ex:
            self.get_logger().warn(
                f"transform {self.map_frame} -> "
                f"{self.target_frame} failed: {ex}"
            )
            return

        ptx = person_tf.transform.translation.x
        pty = person_tf.transform.translation.y
        pyaw = yaw_from_quat(person_tf.transform.rotation)
        cos_p = math.cos(pyaw)
        sin_p = math.sin(pyaw)

        ex_bl, ey_bl = self.last_estimate
        person_x = cos_p * ex_bl - sin_p * ey_bl + ptx
        person_y = sin_p * ex_bl + cos_p * ey_bl + pty

        # --- 人の手前 follow_distance にゴールを置く ---
        dx = person_x - rx
        dy = person_y - ry
        dist = math.hypot(dx, dy)
        goal_yaw = math.atan2(dy, dx)   # 常に人の方を向く

        if dist > self.follow_distance:
            ratio = (dist - self.follow_distance) / dist
            gx = rx + dx * ratio
            gy = ry + dy * ratio
        else:
            # 既に十分近い: その場で人の方を向くだけ
            gx = rx
            gy = ry

        # --- ロボット現在地 -> ゴール の直線経路を生成 (map 系) ---
        path = Path()
        path.header.frame_id = self.map_frame
        path.header.stamp = self.get_clock().now().to_msg()

        seg = math.hypot(gx - rx, gy - ry)
        n = max(1, int(seg / self.path_resolution))

        for i in range(n + 1):
            t = i / n
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = rx + (gx - rx) * t
            pose.pose.position.y = ry + (gy - ry) * t
            pose.pose.orientation = quat_from_yaw(goal_yaw)
            path.poses.append(pose)

        self.path_pub.publish(path)
        self.publish_execute(True)

    def publish_markers(self, header, observation, state):

        array = MarkerArray()

        # --- パーティクル (SPHERE_LIST、重みで赤→緑グラデーション) ---
        pm = Marker()
        pm.header = header
        pm.ns = 'particles'
        pm.id = 0
        pm.type = Marker.SPHERE_LIST
        pm.action = Marker.ADD
        pm.scale.x = 0.05
        pm.scale.y = 0.05
        pm.scale.z = 0.05
        pm.pose.orientation.w = 1.0

        max_w = float(np.max(self.pf.weights))

        for p, w in zip(self.pf.particles, self.pf.weights):
            pt = Point()
            pt.x = float(p[0])
            pt.y = float(p[1])
            pt.z = 0.0
            pm.points.append(pt)

            t = float(w / max_w) if max_w > 0.0 else 0.0
            c = ColorRGBA()
            c.r = 1.0 - t
            c.g = t
            c.b = 0.0
            c.a = 0.7
            pm.colors.append(c)

        array.markers.append(pm)

        # --- 観測点 (DBSCANクラスタ重心、黄色) ---
        om = Marker()
        om.header = header
        om.ns = 'observation'
        om.id = 1
        om.type = Marker.SPHERE
        om.action = Marker.ADD
        om.pose.position.x = float(observation[0])
        om.pose.position.y = float(observation[1])
        om.pose.position.z = 0.0
        om.pose.orientation.w = 1.0
        om.scale.x = 0.15
        om.scale.y = 0.15
        om.scale.z = 0.15
        om.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.9)
        array.markers.append(om)

        # --- 推定位置 (パーティクルフィルタ出力、水色) ---
        em = Marker()
        em.header = header
        em.ns = 'estimate'
        em.id = 2
        em.type = Marker.SPHERE
        em.action = Marker.ADD
        em.pose.position.x = float(state[0])
        em.pose.position.y = float(state[1])
        em.pose.position.z = 0.0
        em.pose.orientation.w = 1.0
        em.scale.x = 0.30
        em.scale.y = 0.30
        em.scale.z = 0.30
        em.color = ColorRGBA(r=0.0, g=0.5, b=1.0, a=0.9)
        array.markers.append(em)

        self.viz_pub.publish(array)


def main():

    rclpy.init()

    print("start")

    node = PersonTrackerAvoid()

    print("node constract")

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
