#!/usr/bin/env python3
from rclpy_util.util import TemporaryApproximateTimeSynchronizer
from rclpy.duration import Duration
from rclpy.node import Node
import rclpy
import smach

from nakalab_ultralytics_interfaces.msg import (
    PersonPose2DArray,
    PersonPose3DArray,
)
from nakalab_ultralytics_interfaces.srv import Detect

import copy
import math
import traceback


class PersonDetector():
    """
    人物姿勢検出ノードの起動と停止を制御するクライアント.

    Parameters
    ----------
    node : Node
        `/nu_ros2/detect_person` サービス呼び出しに使用する ROS ノード。

    Attributes
    ----------
    SERVICE_NAME : str
        人物検出の起動・停止を制御するサービス名。
    CONNECTION_TIMEOUT_SEC : float
        サービス接続待機のタイムアウト秒数。

    """

    SERVICE_NAME = '/nu_ros2/detect_person'
    CONNECTION_TIMEOUT_SEC = 5.0

    def __init__(self, node: Node):
        self.__node = node

        # create service client
        self.person_pose_cli = self.__node.create_client(
            Detect,
            self.SERVICE_NAME
        )

        while not self.person_pose_cli.wait_for_service(
            timeout_sec=self.CONNECTION_TIMEOUT_SEC
        ):
            err_msg = (
                'Service %s is not running! '
                'Did you execute nakalab_ultralytics_ros2 ?'
            ) % self.SERVICE_NAME
            self.__node.get_logger().error(err_msg)
            raise RuntimeError(err_msg)

    def __send_person_pose_req(self, req: Detect.Request):
        future = self.person_pose_cli.call_async(req)
        rclpy.spin_until_future_complete(self.__node, future)
        response: Detect.Response = future.result()
        self.__node.get_logger().info(response.message)
        return response.success

    def execute(self, run: bool, confedence: float = 0.5) -> bool:
        """
        人物姿勢検出モデルの起動または停止を要求する.

        Parameters
        ----------
        run : bool
            True の場合はモデルをロードして検出を開始し、False の場合はモデルを破棄する。
        confedence : float, optional
            検出信頼度のしきい値。

        Returns
        -------
        bool
            サービス要求が成功した場合 True。

        """
        try:
            req = Detect.Request()
            req.run = run
            req.confidence = confedence
            return self.__send_person_pose_req(req)
        except Exception:
            err_msg = traceback.format_exc()
            self.__node.get_logger().error(
                'Error is occured in PersonDetector.execute ======\n%s' %
                err_msg
            )
            raise RuntimeError(err_msg)


class PersonDetectorState(smach.State, PersonDetector):

    TOPIC_INFO_LIST = [
        (PersonPose2DArray, '/nu_ros2/person_pose_2d'),
        (PersonPose3DArray, '/nu_ros2/person_pose_3d'),
    ]
    MAX_SCAN_TIME_SEC = 10.0
    HAND_UP_2D_MARGIN_PX = 20.0
    HAND_UP_3D_MARGIN_M = 0.08
    MIN_POSITION_KEYPOINTS = 2

    def __init__(self,
                 node: Node,
                 timeout_sec: float = 10.0,
                 scan_time_sec: float = 1.0,
                 confedence: float = 0.5,
                 condition: str = 'detect',
                 cluster_radius_m: float = 0.35,
                 min_matched_samples: int = 2,
                 torso_keypoint_indices=(5, 6, 11, 12),
                 min_keypoint_confidence: float = 0.1,
                 max_valid_depth_m: float = 6.0,
                 use_3d_hand_check: bool = False,
                 selection_policy: str = 'densest'):
        """
        人物姿勢を検出し、SMACH userdata に辞書形式で格納する状態.

        Parameters
        ----------
        node : Node
            サービス呼び出し、購読、ログ出力に使用する ROS ノード。
        timeout_sec : float, optional
            人物検出を待機する最大秒数。
        scan_time_sec : float, optional
            condition に合致しても最低限検出に当てる秒数。timeout_sec と 10.0 秒を
            上限として自動調整される。
        confedence : float, optional
            YOLO pose 推論に渡す検出信頼度のしきい値。
        condition : str, optional
            検出成功条件。`detect` は 1 人以上の人物検出、`hand_up` は手上げ人物検出。
        cluster_radius_m : float, optional
            scan_time_sec 中に蓄積した候補人物位置をクラスタリングする半径 [m]。
        min_matched_samples : int, optional
            成功判定と最終クラスタに必要な最小サンプル数。
        torso_keypoint_indices : sequence of int, optional
            人物代表点の推定で優先する torso keypoint index。
        min_keypoint_confidence : float, optional
            代表点推定と 2D hand_up 判定で使う keypoint confidence の下限。
        max_valid_depth_m : float, optional
            代表点推定で有効とみなす 3D keypoint の最大 z 距離 [m]。
        use_3d_hand_check : bool, optional
            True の場合、有効な 3D keypoint があるときだけ hand_up の補助確認に使う。
            3D keypoint が無効な場合は 2D 判定を維持する。
        selection_policy : str, optional
            複数候補の選択方針。`densest`, `nearest`, `center`,
            `highest_confidence` から選択する。

        userdata
        --------
        Output Keys:
            person_poses : list[dict]
                検出人物ごとの 2D bbox/keypoints、3D keypoints、人物位置を
                格納した辞書配列。

        person_poses structure:
            ```json
            [
                {
                    "frame_id": "camera_frame_name",
                    "confedence": 0.0,
                    "pose_2d": {
                    "confedence": 0.0,
                    "bbox": [0.0, 0.0, 0.0, 0.0],
                    "keypoints": [
                        [0.0, 0.0]
                    ],
                    "keypoints_confidence": [0.0]
                    },
                    "pose_3d": {
                    "pose": [0.0, 0.0, 0.0],
                    "keypoints": [
                        [0.0, 0.0, 0.0]
                    ]
                    }
                }
            ]
            ```

        Outcomes
        --------
        success
            condition に合致した人物が検出された場合。
        timeout
            timeout_sec 内に condition に合致した人物が検出されなかった場合。
        failure
            サービス呼び出し、購読、変換処理のいずれかで例外が発生した場合。

        """

        smach.State.__init__(self, outcomes=['success', 'timeout', 'failure'],
                             output_keys=['person_poses'])
        PersonDetector.__init__(self, node=node)

        self.__node = node
        self.__confedence = confedence
        self.__timeout_sec = timeout_sec
        self.__scan_time_sec = self.__normalize_scan_time_sec(scan_time_sec)
        self.___condition = condition
        self.__cluster_radius_m = max(0.0, float(cluster_radius_m))
        self.__min_matched_samples = max(1, int(min_matched_samples))
        self.__torso_keypoint_indices = tuple(
            int(index) for index in torso_keypoint_indices
        )
        self.__min_keypoint_confidence = max(
            0.0, float(min_keypoint_confidence)
        )
        self.__max_valid_depth_m = max(0.0, float(max_valid_depth_m))
        self.__use_3d_hand_check = bool(use_3d_hand_check)
        self.__selection_policy = selection_policy
        self.__detect_person = False
        self.__matched_pose_samples = []
        self.__first_match_time = None
        self.__unknown_condition_logged = False

        # userdata.person_poses structure
        self.__person_poses = [
            # {
            #     'frame_id': 'camera_frame_name',
            #     'confedence': 0.0,
            #     'pose_2d': {
            #         'confedence': 0.0,
            #         'bbox': [0.0, 0.0],
            #         'keypoints': [
            #             [x, y],
            #             ...
            #         ],
            #     },
            #     'pose_3d': {
            #         'pose': [x, y, z],
            #         'keypoints': [
            #             [x, y, z],
            #             ...
            #         ],
            #     }
            # },
            # ...
        ]

    def __normalize_scan_time_sec(self, scan_time_sec: float) -> float:
        max_scan_time_sec = min(self.__timeout_sec, self.MAX_SCAN_TIME_SEC)
        if scan_time_sec > self.__timeout_sec:
            self.__node.get_logger().warning(
                'scan_time_sec %.2f exceeds timeout_sec %.2f. '
                'Adjusting scan_time_sec to %.2f.' %
                (scan_time_sec, self.__timeout_sec, max_scan_time_sec)
            )
            return max_scan_time_sec

        if scan_time_sec > self.MAX_SCAN_TIME_SEC:
            self.__node.get_logger().warning(
                'scan_time_sec %.2f exceeds max value %.2f. '
                'Adjusting scan_time_sec to %.2f.' %
                (scan_time_sec, self.MAX_SCAN_TIME_SEC, self.MAX_SCAN_TIME_SEC)
            )
            return self.MAX_SCAN_TIME_SEC

        if scan_time_sec < 0.0:
            self.__node.get_logger().warning(
                'scan_time_sec %.2f is negative. Adjusting to 0.0.' %
                scan_time_sec
            )
            return 0.0

        return scan_time_sec

    def __cb(self,
             person_pose_2d_msg: PersonPose2DArray,
             person_pose_3d_msg: PersonPose3DArray):
        current_person_poses = self.__make_person_pose_dicts(
            person_pose_2d_msg,
            person_pose_3d_msg,
        )
        self.__person_poses = current_person_poses

        matched_person_poses = self.__filter_matching_person_poses(
            current_person_poses
        )
        now = self.__node.get_clock().now()
        appended_sample = False
        for person_pose in matched_person_poses:
            position = self.__estimate_person_position(person_pose)
            if position is None:
                continue

            sample_pose = copy.deepcopy(person_pose)
            sample_pose['pose_3d']['pose'] = position
            self.__matched_pose_samples.append({
                'person_pose': sample_pose,
                'position': position,
                'confidence': self.__person_pose_confidence(sample_pose),
                'stamp_ns': now.nanoseconds,
            })
            appended_sample = True

        if appended_sample and self.__first_match_time is None:
            self.__first_match_time = now

        self.__detect_person = bool(self.__matched_pose_samples)

    def __make_person_pose_dicts(self,
                                 person_pose_2d_msg: PersonPose2DArray,
                                 person_pose_3d_msg: PersonPose3DArray):
        person_poses = []
        pose_count = min(
            len(person_pose_2d_msg.poses),
            len(person_pose_3d_msg.poses)
        )
        frame_id = (
            person_pose_3d_msg.header.frame_id
            or person_pose_2d_msg.header.frame_id
        )

        for index in range(pose_count):
            pose_2d = person_pose_2d_msg.poses[index]
            pose_3d = person_pose_3d_msg.poses[index]
            keypoints_3d = [
                [float(point.x), float(point.y), float(point.z)]
                for point in pose_3d.keypoints
            ]
            keypoint_confidences = [
                float(point.confidence)
                for point in pose_2d.keypoints
            ]

            pose_dict = {
                'frame_id': frame_id,
                'confedence': float(pose_2d.confidence),
                'pose_2d': {
                    'confedence': float(pose_2d.confidence),
                    'bbox': [
                        float(pose_2d.bounding_box.top_left.x),
                        float(pose_2d.bounding_box.top_left.y),
                        float(pose_2d.bounding_box.bottom_right.x),
                        float(pose_2d.bounding_box.bottom_right.y),
                    ],
                    'keypoints': [
                        [float(point.x), float(point.y)]
                        for point in pose_2d.keypoints
                    ],
                    'keypoints_confidence': keypoint_confidences,
                },
                'pose_3d': {
                    'pose': self.__estimate_position_from_keypoints(
                        keypoints_3d,
                        keypoint_confidences,
                    ) or [float('nan'), float('nan'), float('nan')],
                    'keypoints': keypoints_3d,
                }
            }
            person_poses.append(pose_dict)

        return person_poses

    def __filter_matching_person_poses(self, person_poses):
        if self.___condition == 'detect':
            return list(person_poses)

        if self.___condition == 'hand_up':
            return [
                person_pose
                for person_pose in person_poses
                if self.__is_hand_up(person_pose)
            ]

        if not self.__unknown_condition_logged:
            self.__node.get_logger().warning(
                'Unknown condition "%s". Treating detection as failed.' %
                self.___condition
            )
            self.__unknown_condition_logged = True
        return []

    def __estimate_person_position(self, person_pose):
        return self.__estimate_position_from_keypoints(
            person_pose['pose_3d']['keypoints'],
            person_pose['pose_2d'].get('keypoints_confidence', []),
        )

    def __estimate_position_from_keypoints(self, keypoints_3d,
                                           keypoint_confidences):
        torso_points = self.__valid_3d_keypoints(
            keypoints_3d,
            keypoint_confidences,
            self.__torso_keypoint_indices,
        )
        if len(torso_points) >= self.MIN_POSITION_KEYPOINTS:
            return self.__median_point(torso_points)

        all_points = self.__valid_3d_keypoints(
            keypoints_3d,
            keypoint_confidences,
            range(len(keypoints_3d)),
        )
        if len(all_points) < self.MIN_POSITION_KEYPOINTS:
            return None

        cluster = self.__densest_position_cluster(all_points)
        if len(cluster) < self.MIN_POSITION_KEYPOINTS:
            return None
        return self.__median_point(cluster)

    def __valid_3d_keypoints(self, keypoints_3d, keypoint_confidences,
                             indices):
        points = []
        for index in indices:
            if index >= len(keypoints_3d):
                continue
            confidence = (
                keypoint_confidences[index]
                if index < len(keypoint_confidences)
                else 1.0
            )
            if confidence < self.__min_keypoint_confidence:
                continue
            point = keypoints_3d[index]
            if not self.__is_valid_point(point):
                continue
            if point[2] <= 0.0 or point[2] > self.__max_valid_depth_m:
                continue
            points.append(point)
        return points

    def __densest_position_cluster(self, points):
        if not points:
            return []

        radius_sq = self.__cluster_radius_m * self.__cluster_radius_m
        best_cluster = []
        best_z_variance = float('inf')
        for point in points:
            cluster = [
                other
                for other in points
                if self.__squared_distance(point, other) <= radius_sq
            ]
            z_variance = self.__z_variance(cluster)
            if (
                len(cluster) > len(best_cluster)
                or (
                    len(cluster) == len(best_cluster)
                    and z_variance < best_z_variance
                )
            ):
                best_cluster = cluster
                best_z_variance = z_variance
        return best_cluster

    def __finalize_person_poses(self):
        clusters = self.__cluster_matched_samples()
        if not clusters:
            return []

        finalized_person_poses = []
        for cluster in clusters:
            positions = [sample['position'] for sample in cluster]
            representative_position = self.__median_point(positions)
            best_sample = max(
                cluster,
                key=lambda sample: (
                    sample['confidence'],
                    sample['stamp_ns'],
                )
            )
            person_pose = copy.deepcopy(best_sample['person_pose'])
            person_pose['pose_3d']['pose'] = representative_position
            person_pose['sample_count'] = len(cluster)
            person_pose['position_variance'] = self.__position_variance(
                positions
            )
            person_pose['last_observed_stamp_ns'] = max(
                sample['stamp_ns'] for sample in cluster
            )
            finalized_person_poses.append(person_pose)

        finalized_person_poses.sort(key=self.__person_pose_sort_key)
        return finalized_person_poses

    def __cluster_matched_samples(self):
        if len(self.__matched_pose_samples) < self.__min_matched_samples:
            return []

        radius_sq = self.__cluster_radius_m * self.__cluster_radius_m
        clusters = []
        seen_clusters = set()
        for index, sample in enumerate(self.__matched_pose_samples):
            cluster_indices = tuple(
                candidate_index
                for candidate_index, candidate in enumerate(
                    self.__matched_pose_samples
                )
                if self.__squared_distance(
                    sample['position'],
                    candidate['position']
                ) <= radius_sq
            )
            if (
                len(cluster_indices) < self.__min_matched_samples
                or cluster_indices in seen_clusters
            ):
                continue
            seen_clusters.add(cluster_indices)
            clusters.append([
                self.__matched_pose_samples[candidate_index]
                for candidate_index in cluster_indices
            ])

        return clusters

    def __person_pose_sort_key(self, person_pose):
        pose = person_pose['pose_3d']['pose']
        sample_count = person_pose.get('sample_count', 0)
        confidence = self.__person_pose_confidence(person_pose)
        variance = person_pose.get('position_variance', float('inf'))
        recency = person_pose.get('last_observed_stamp_ns', 0)

        if self.__selection_policy == 'nearest':
            return (
                self.__distance_from_origin(pose),
                -sample_count,
                -confidence,
                variance,
                -recency,
            )
        if self.__selection_policy == 'center':
            return (
                abs(pose[0]),
                -sample_count,
                -confidence,
                variance,
                -recency,
            )
        if self.__selection_policy == 'highest_confidence':
            return (
                -confidence,
                -sample_count,
                variance,
                -recency,
            )

        return (
            -sample_count,
            variance,
            -confidence,
            -recency,
        )

    def __is_hand_up(self, person_pose: dict) -> bool:
        return (
            self.__is_hand_up_side(person_pose, shoulder_index=5,
                                   elbow_index=7, wrist_index=9)
            or self.__is_hand_up_side(person_pose, shoulder_index=6,
                                      elbow_index=8, wrist_index=10)
        )

    def __is_hand_up_side(self, person_pose: dict, shoulder_index: int,
                          elbow_index: int, wrist_index: int) -> bool:
        keypoints_2d = person_pose['pose_2d']['keypoints']
        keypoints_3d = person_pose['pose_3d']['keypoints']
        keypoint_confidences = person_pose['pose_2d'].get(
            'keypoints_confidence', []
        )
        try:
            shoulder_2d = keypoints_2d[shoulder_index]
            elbow_2d = keypoints_2d[elbow_index]
            wrist_2d = keypoints_2d[wrist_index]
            shoulder_3d = keypoints_3d[shoulder_index]
            elbow_3d = keypoints_3d[elbow_index]
            wrist_3d = keypoints_3d[wrist_index]
        except IndexError:
            return False

        if not (
            self.__is_valid_point(shoulder_2d)
            and self.__is_valid_point(elbow_2d)
            and self.__is_valid_point(wrist_2d)
            and self.__has_enough_keypoint_confidence(
                keypoint_confidences,
                shoulder_index,
                elbow_index,
                wrist_index,
            )
        ):
            return False

        wrist_above_shoulder_2d = (
            wrist_2d[1] < shoulder_2d[1] - self.HAND_UP_2D_MARGIN_PX
        )
        wrist_above_elbow_2d = wrist_2d[1] < elbow_2d[1]
        if not (wrist_above_shoulder_2d and wrist_above_elbow_2d):
            return False

        if not self.__use_3d_hand_check:
            return True

        if self.__is_valid_point(shoulder_3d) and self.__is_valid_point(
            wrist_3d
        ):
            wrist_above_shoulder_3d = (
                wrist_3d[1] < shoulder_3d[1] - self.HAND_UP_3D_MARGIN_M
            )
            if not wrist_above_shoulder_3d:
                return False

            if (
                self.__is_valid_point(elbow_3d)
                and not wrist_3d[1] < elbow_3d[1]
            ):
                return False

        return True

    def __has_enough_keypoint_confidence(self, keypoint_confidences, *indices):
        for index in indices:
            if index >= len(keypoint_confidences):
                continue
            if keypoint_confidences[index] < self.__min_keypoint_confidence:
                return False
        return True

    @staticmethod
    def __is_valid_point(point) -> bool:
        return all(math.isfinite(value) for value in point)

    @staticmethod
    def __median_point(points):
        return [
            PersonDetectorState.__median([point[axis] for point in points])
            for axis in range(3)
        ]

    @staticmethod
    def __median(values):
        sorted_values = sorted(values)
        value_count = len(sorted_values)
        midpoint = value_count // 2
        if value_count % 2:
            return sorted_values[midpoint]
        return (
            sorted_values[midpoint - 1]
            + sorted_values[midpoint]
        ) * 0.5

    @staticmethod
    def __squared_distance(first, second):
        dx = first[0] - second[0]
        dy = first[1] - second[1]
        dz = first[2] - second[2]
        return dx * dx + dy * dy + dz * dz

    @staticmethod
    def __z_variance(points):
        if not points:
            return float('inf')
        z_mean = sum(point[2] for point in points) / len(points)
        return sum((point[2] - z_mean) ** 2 for point in points) / len(points)

    @staticmethod
    def __position_variance(points):
        if not points:
            return float('inf')
        center = PersonDetectorState.__median_point(points)
        return sum(
            PersonDetectorState.__squared_distance(point, center)
            for point in points
        ) / len(points)

    @staticmethod
    def __distance_from_origin(point):
        return math.sqrt(point[0] ** 2 + point[1] ** 2 + point[2] ** 2)

    @staticmethod
    def __person_pose_confidence(person_pose):
        return float(
            person_pose.get(
                'confedence',
                person_pose.get('pose_2d', {}).get('confedence', 0.0)
            )
        )

    def execute(self, userdata):
        """
        人物検出を実行し、検出結果を userdata.person_poses に格納する.

        Parameters
        ----------
        userdata : smach.UserData
            `person_poses` output key を受け渡す SMACH userdata。

        Returns
        -------
        str
            `success`, `timeout`, `failure` のいずれか。

        """
        detector_started = False
        try:
            self.__person_poses = []
            self.__detect_person = False
            self.__matched_pose_samples = []
            self.__first_match_time = None
            self.__unknown_condition_logged = False
            with TemporaryApproximateTimeSynchronizer(
                node=self.__node,
                sub_topics=self.TOPIC_INFO_LIST,
                qos_profile=10,
                slop=1.0,
                callback=self.__cb
            ):
                self.__node.get_logger().info('''
	================================================
	PERSON DETECTOR START!
	------------------------------------------------
	condition: %s
	conference: %f
	scan_time_sec: %f
	timeout_sec: %f
	cluster_radius_m: %f
	min_matched_samples: %d
	================================================
                ''' % (
                    self.___condition,
                    self.__confedence,
                    self.__scan_time_sec,
                    self.__timeout_sec,
                    self.__cluster_radius_m,
                    self.__min_matched_samples,
                ))
                if not PersonDetector.execute(self, True, self.__confedence):
                    raise RuntimeError('Failure bringup detector.')
                detector_started = True
                ct = self.__node.get_clock().now()
                timeout_duration = Duration(seconds=self.__timeout_sec)
                scan_duration = Duration(seconds=self.__scan_time_sec)
                while self.__node.get_clock().now() - ct < timeout_duration:
                    rclpy.spin_once(self.__node, timeout_sec=0.1)
                    if self.__first_match_time is None:
                        continue
                    now = self.__node.get_clock().now()
                    if now - self.__first_match_time < scan_duration:
                        continue
                    self.__person_poses = self.__finalize_person_poses()
                    self.__detect_person = bool(self.__person_poses)
                    if self.__detect_person:
                        break

            self.__node.get_logger().info('''
================================================
PERSON DETECTOR STOP...
================================================
            ''')
            if detector_started and not PersonDetector.execute(self, False):
                raise RuntimeError('Failure disarm detector.')

            self.__person_poses = self.__finalize_person_poses()
            self.__detect_person = bool(self.__person_poses)

            if not self.__detect_person:
                userdata.person_poses = []
                self.__node.get_logger().warning(
                    'TIMEOUT! The person is not found...'
                )
                return 'timeout'

            userdata.person_poses = self.__person_poses
            self.__node.get_logger().info('''
================================================
PERSON DETECTION SUCCESSFLLY !
SERCH CONDITION: %s
================================================
            '''%self.___condition)
            return 'success'

        except Exception:
            err_msg = traceback.format_exc()
            self.__node.get_logger().error(
                'Error is occured in PersonDetectorState.execute ======\n%s' %
                err_msg
            )
            if detector_started:
                try:
                    PersonDetector.execute(self, False)
                except Exception:
                    self.__node.get_logger().error(
                        'Failure disarm detector during error handling.'
                    )
            return 'failure'
