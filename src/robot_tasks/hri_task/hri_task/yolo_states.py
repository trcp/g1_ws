#!/usr/bin/env python3
"""
YOLO Docker ノードとのトピック通信を使った SMACH ステート群。
yolo_human_node が別コンテナで起動していることを前提とする。
"""
import sys, os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.node import Node
import smach
import json
import time
import math
import subprocess
import threading
import cv2
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from direct_joint_control import HOME_POSE
from bag_grasp_ik import calculate_bag_grasp_joints
from yolo_track import YoloHumanTracker

try:
    from yolo_human.extract_person_features_off import match_reference_person
except Exception:
    match_reference_person = None

ENABLE_OCCUPANT_CONTEXT_SPEECH = True

# ============================================================
# ベースクラス: YOLO コマンド送信 + 結果受信
# ============================================================
class BaseYoloState(smach.State):
    """YOLOノードへの start/stop コマンド送信と結果受信の共通処理。"""

    def __init__(self, node: Node, target_classes,
                 command_topic='/yolo_human/command',
                 result_topic='/yolo_human/result',
                 timeout=5.0,
                 input_keys=[],
                 output_keys=[]):
        smach.State.__init__(self, outcomes=['success', 'failure', 'timeout'],
                             input_keys=input_keys, output_keys=output_keys)
        self.node = node
        self.target_classes = target_classes
        self.timeout = timeout

        # Publisher（コマンド送信用）
        self.cmd_pub = self.node.create_publisher(String, command_topic, 10)

        # Subscriber（結果受信用）— 永続的に購読してキャッシュ
        self.latest_msg = None
        self._result_sub = self.node.create_subscription(
            String, result_topic, self._result_callback, 10)

    def _result_callback(self, msg):
        self.latest_msg = msg.data

    def start_yolo(self):
        """YOLOノードに推論開始コマンドを送信する。"""
        cmd = {"command": "start", "classes": self.target_classes}
        msg = String()
        msg.data = json.dumps(cmd)
        self.cmd_pub.publish(msg)
        self.node.get_logger().info(f"[YOLO] START classes={self.target_classes}")

    def stop_yolo(self):
        """YOLOノードに推論停止コマンドを送信する。"""
        cmd = {"command": "stop"}
        msg = String()
        msg.data = json.dumps(cmd)
        self.cmd_pub.publish(msg)
        self.node.get_logger().info("[YOLO] STOP")

    def wait_for_result(self, timeout=None, spin=True):
        """最新の結果が届くまで待機する。タイムアウトしたら None を返す。"""
        self.latest_msg = None
        t = timeout if timeout is not None else self.timeout
        start_time = time.time()
        while rclpy.ok() and self.latest_msg is None:
            if time.time() - start_time > t:
                return None
            if spin:
                rclpy.spin_once(self.node, timeout_sec=0.1)
            else:
                time.sleep(0.1)
        return self.latest_msg

    def get_closest_person(self, detections: list):
        """複数の人物の中から一番近い(distance_zが最小の)ものを探す"""
        people = [d for d in detections if d.get('label') == 'person']
        if not people:
            return None
        return min(people, key=lambda p: p.get('distance_z', 999.0))

    def parse_yolo_target(self, msg_str):
        """JSON文字列から一番近いターゲットを抽出し、基本情報を返す"""
        try:
            detections = json.loads(msg_str)
            target = self.get_closest_person(detections)
            if target:
                z = target.get('distance_z', 999.0)
                # YOLO出力は右が正。ジョイント座標系（左が正）に統一するため反転
                angle_rad = -target.get('angle_rad', 0.0)
                w_ratio = target.get('bbox_width_ratio', 0.0)
                return target, z, angle_rad, w_ratio
        except json.JSONDecodeError:
            pass
        return None, 999.0, 0.0, 0.0


# ============================================================
# 人物トラッキング: 腰の回転で一番近い人の方を向く
# ============================================================
class YoloTrackingState(BaseYoloState):
    """
    YOLO で一番近い person を検出し、腰（waist_yaw_joint）の回転で
    その人の方向を向くステート。

    - use_waist=True: 腰を回して追従（インタラクション時）
    - use_waist=False: cmd_vel 的なログ出力のみ（将来のフォロー用）
    - max_loops: 追従ループ回数（デフォルト3）
    """

    def __init__(self, node, target_classes=None, timeout=30.0,
                 direct_arm=None, use_waist=True, distance_threshold=1.05,
                 ideal_distance=0.7, center_threshold=0.45,
                 stationary_angle_threshold=0.08, stationary_depth_threshold=0.18,
                 consecutive_frames=3, max_loops=3):
        if target_classes is None:
            target_classes = ["person"]
        super().__init__(node, target_classes, timeout=timeout)
        self.direct_arm = direct_arm
        self.use_waist = use_waist
        self.distance_threshold = distance_threshold
        self.ideal_distance = ideal_distance
        self.center_threshold = center_threshold
        self.stationary_angle_threshold = stationary_angle_threshold
        self.stationary_depth_threshold = stationary_depth_threshold
        self.consecutive_frames = consecutive_frames
        self.max_loops = max_loops

    def _person_candidates(self, detections):
        candidates = []
        for det in detections:
            if det.get('label') != 'person':
                continue

            z = float(det.get('distance_z', 999.0))
            if not (0.1 < z < 10.0):
                continue

            angle_rad = -float(det.get('angle_rad', 0.0))
            w_ratio = float(det.get('bbox_width_ratio', 0.0))
            score = (
                abs(z - self.ideal_distance) * 1.5
                + abs(angle_rad) * 2.0
                + max(0.0, z - self.distance_threshold) * 3.0
            )
            candidates.append({
                'det': det,
                'z': z,
                'angle_rad': angle_rad,
                'w_ratio': w_ratio,
                'score': score,
            })

        return sorted(candidates, key=lambda c: c['score'])

    def _is_good_guest_candidate(self, candidate):
        return (
            0.1 < candidate['z'] < self.distance_threshold
            and abs(candidate['angle_rad']) < self.center_threshold
        )

    def _is_stationary_candidate(self, candidate, previous):
        if previous is None:
            return False
        return (
            abs(candidate['angle_rad'] - previous['angle_rad']) <= self.stationary_angle_threshold
            and abs(candidate['z'] - previous['z']) <= self.stationary_depth_threshold
        )

    def _turn_waist_to_candidate(self, candidate):
        if not (self.use_waist and self.direct_arm):
            return
        angle_rad = candidate['angle_rad']
        w_ratio = candidate['w_ratio']
        if (
            w_ratio < 0.6
            and candidate['z'] <= self.distance_threshold + 0.25
            and abs(angle_rad) > 0.2
            and abs(angle_rad) < 0.75
        ):
            try:
                self.direct_arm.turn_waist_towards(angle_rad, hold_sec=0)
            except Exception as e:
                self.node.get_logger().warn(f"  -> Waist move failed: {e}")

    def execute(self, userdata):
        self.node.get_logger().info("[YOLO TRACKING] Starting...")
        self.start_yolo()
        
        start_time = time.time()
        close_frames = 0
        previous_candidate = None
        best_seen = None
        
        while rclpy.ok():
            if time.time() - start_time > self.timeout:
                if best_seen:
                    self.node.get_logger().info(
                        "  -> Tracking timeout. Using best guest candidate "
                        f"z={best_seen['z']:.2f}, angle={best_seen['angle_rad']:.2f}, "
                        f"score={best_seen['score']:.2f}."
                    )
                    self._turn_waist_to_candidate(best_seen)
                else:
                    self.node.get_logger().info("  -> Tracking timeout with no person candidate. Forcing success.")
                break

            msg = self.wait_for_result(timeout=1.0)
            if msg:
                try:
                    detections = json.loads(msg)
                except json.JSONDecodeError:
                    self.node.get_logger().warn("  -> Invalid YOLO JSON while tracking.")
                    detections = []

                candidates = self._person_candidates(detections)
                if candidates:
                    candidate = candidates[0]
                    if best_seen is None or candidate['score'] < best_seen['score']:
                        best_seen = candidate

                    if self._is_good_guest_candidate(candidate):
                        if self._is_stationary_candidate(candidate, previous_candidate):
                            close_frames += 1
                        else:
                            close_frames = 1

                        self._turn_waist_to_candidate(candidate)

                        if close_frames >= self.consecutive_frames:
                            self.node.get_logger().info(
                                "  -> Guest candidate is close, centered, and stable. "
                                f"z={candidate['z']:.2f}, angle={candidate['angle_rad']:.2f}. Ending tracking."
                            )
                            break
                    else:
                        close_frames = 0
                        self._turn_waist_to_candidate(candidate)

                    previous_candidate = candidate
                else:
                    close_frames = 0
                    previous_candidate = None
            else:
                self.node.get_logger().info("  -> No YOLO result (timeout)")
            
            time.sleep(0.1)

        self.stop_yolo()

        if self.use_waist and self.direct_arm:
            self.node.get_logger().info("  -> Resetting waist to 0.0")
            try:
                self.direct_arm.send_joints({'waist_yaw_joint': 0.0}, hold_sec=1.0)
            except Exception as e:
                self.node.get_logger().warn(f"  -> Waist reset failed: {e}")

        return 'success'


# ============================================================
# 空席検出: person と seating object の位置関係から空いている席を見つける
# ============================================================
class YoloEmptyChairState(BaseYoloState):
    """
    YOLO で person と chair/sofa を同時に検出し、検出物体を座席候補へ
    正規化してから空席を判定する。ソファーが1つのbboxになるケースでは、
    必要に応じてbboxを左右方向に分割して仮想的な席として扱う。
    """

    SEATING_LABELS = {"chair", "sofa", "couch", "bench"}
    SPLIT_LABELS = {"sofa", "couch"}

    def __init__(self, node, direct_arm=None, guest_index=1, timeout=5.0,
                 expected_seat_count=3, image_width=640,
                 seat_stable_seconds=2.0, seat_stable_frames=3,
                 min_split_seat_width_px=90.0):
        super().__init__(
            node,
            target_classes=["person", "chair", "sofa", "couch", "bench"],
            timeout=timeout,
            output_keys=['empty_seat_index'])
        self.direct_arm = direct_arm
        self.guest_index = guest_index
        self.expected_seat_count = expected_seat_count
        self.image_width = image_width
        self.seat_stable_seconds = seat_stable_seconds
        self.seat_stable_frames = seat_stable_frames
        self.min_split_seat_width_px = min_split_seat_width_px

    def _det_conf(self, det):
        return float(det.get('confidence', det.get('score', 1.0)))

    def _bbox(self, item):
        bbox = item.get('bbox', [0, 0, 0, 0])
        if len(bbox) != 4:
            return [0, 0, 0, 0]
        return [float(v) for v in bbox]

    def _bbox_center_x(self, item):
        x1, _, x2, _ = self._bbox(item)
        return (x1 + x2) / 2.0

    def _bbox_width(self, item):
        x1, _, x2, _ = self._bbox(item)
        return max(1.0, x2 - x1)

    def _bbox_height(self, item):
        _, y1, _, y2 = self._bbox(item)
        return max(1.0, y2 - y1)

    def _bbox_center_y(self, item):
        _, y1, _, y2 = self._bbox(item)
        return (y1 + y2) / 2.0

    def _bbox_overlap_ratio(self, a_box, b_box):
        ax1, ay1, ax2, ay2 = a_box
        bx1, by1, bx2, by2 = b_box
        x_left = max(ax1, bx1)
        y_top = max(ay1, by1)
        x_right = min(ax2, bx2)
        y_bottom = min(ay2, by2)
        if x_right <= x_left or y_bottom <= y_top:
            return 0.0
        inter = (x_right - x_left) * (y_bottom - y_top)
        area = max(1.0, (ax2 - ax1) * (ay2 - ay1))
        return inter / area

    def _bbox_iou(self, a_box, b_box):
        ax1, ay1, ax2, ay2 = a_box
        bx1, by1, bx2, by2 = b_box
        x_left = max(ax1, bx1)
        y_top = max(ay1, by1)
        x_right = min(ax2, bx2)
        y_bottom = min(ay2, by2)
        if x_right <= x_left or y_bottom <= y_top:
            return 0.0
        inter = (x_right - x_left) * (y_bottom - y_top)
        a_area = max(1.0, (ax2 - ax1) * (ay2 - ay1))
        b_area = max(1.0, (bx2 - bx1) * (by2 - by1))
        return inter / max(1.0, a_area + b_area - inter)

    def _dedupe_seating_objects(self, seating_objects):
        objects = sorted(
            seating_objects,
            key=lambda o: (self._bbox_width(o) * self._bbox_height(o), self._det_conf(o)),
            reverse=True,
        )
        kept = []
        for obj in objects:
            box = self._bbox(obj)
            duplicate = False
            for existing in kept:
                existing_box = self._bbox(existing)
                iou = self._bbox_iou(box, existing_box)
                overlap_smaller = max(
                    self._bbox_overlap_ratio(box, existing_box),
                    self._bbox_overlap_ratio(existing_box, box),
                )
                if iou > 0.45 or overlap_smaller > 0.70:
                    duplicate = True
                    break
            if not duplicate:
                kept.append(obj)
        return sorted(kept, key=self._bbox_center_x)

    def _is_splittable_seating_object(self, obj, frame_width):
        label = obj.get('label', '')
        return label in self.SPLIT_LABELS

    def _part_name(self, part_index, part_count):
        if part_count == 2:
            return ["left side", "right side"][part_index]
        if part_count == 3:
            return ["left side", "middle", "right side"][part_index]
        if part_index == 0:
            return "left side"
        if part_index == part_count - 1:
            return "right side"
        return f"part {part_index + 1}"

    def _source_name(self, obj):
        label = obj.get('label', 'seat')
        if label == "couch":
            return "sofa"
        if label == "sofa":
            return "sofa"
        return "chair"

    def _seat_from_bbox(self, bbox, source_label='inferred_seat',
                        source_bbox=None, split_count=1, split_part=1,
                        description=None, inferred=False):
        x1, y1, x2, y2 = bbox
        if source_bbox is None:
            source_bbox = [x1, y1, x2, y2]
        return {
            'bbox': [x1, y1, x2, y2],
            'center_x': (x1 + x2) / 2.0,
            'source_label': source_label,
            'source_bbox': source_bbox,
            'split_count': split_count,
            'split_part': split_part,
            'description': description,
            'occupied': False,
            'occupant': None,
            'inferred': inferred,
        }

    def _snapshot_from_seats(self, seats):
        snapshot = []
        for seat in sorted(seats, key=lambda s: s['center_x']):
            snapshot.append({
                'center_x': seat['center_x'],
                'bbox': list(seat['bbox']),
                'description': seat['description'],
                'source_label': seat['source_label'],
                'source_bbox': list(seat.get('source_bbox', seat['bbox'])),
                'split_count': seat.get('split_count', 1),
                'split_part': seat.get('split_part', 1),
                'inferred': seat.get('inferred', False),
            })
        return snapshot

    def _seats_from_snapshot(self, snapshot):
        seats = []
        for item in snapshot or []:
            bbox = item.get('bbox')
            if not isinstance(bbox, list) or len(bbox) != 4:
                continue
            seats.append(self._seat_from_bbox(
                bbox,
                source_label=item.get('source_label', 'snapshot_seat'),
                source_bbox=item.get('source_bbox', bbox),
                split_count=item.get('split_count', 1),
                split_part=item.get('split_part', 1),
                description=item.get('description'),
                inferred=item.get('inferred', False),
            ))
        return self._finalize_seat_indices(seats)

    def _snapshot_seats_available(self):
        if not self.direct_arm or self.guest_index < 2:
            return None
        snapshot = getattr(self.direct_arm, 'seat_layout_snapshot', None)
        if not snapshot:
            return None
        seats = self._seats_from_snapshot(snapshot)
        return seats or None

    def _split_object_into_seats(self, obj, split_count):
        x1, y1, x2, y2 = self._bbox(obj)
        width = max(1.0, x2 - x1)
        source_name = self._source_name(obj)
        seats = []
        for i in range(split_count):
            sx1 = x1 + width * i / split_count
            sx2 = x1 + width * (i + 1) / split_count
            part = self._part_name(i, split_count)
            seats.append(self._seat_from_bbox(
                [sx1, y1, sx2, y2],
                source_label=obj.get('label', 'seat'),
                source_bbox=[x1, y1, x2, y2],
                split_count=split_count,
                split_part=i + 1,
                description=f"the {part} of the {source_name}",
            ))
        return seats

    def _is_wide_enough_to_split(self, obj, split_count):
        slot_width = self._bbox_width(obj) / max(1, split_count)
        return slot_width >= self.min_split_seat_width_px, slot_width

    def _select_primary_sofa(self, objects, frame_width):
        sofas = [o for o in objects if o.get('label') in self.SPLIT_LABELS]
        if not sofas:
            return None
        image_center = frame_width / 2.0
        return min(
            sofas,
            key=lambda o: (
                abs(self._bbox_center_x(o) - image_center),
                -self._bbox_width(o),
                -self._det_conf(o),
            )
        )

    def _expand_sofa_bbox_with_seated_people(self, sofa, people, frame_width):
        x1, y1, x2, y2 = self._bbox(sofa)
        sofa_width = max(1.0, x2 - x1)
        left_people = []
        right_people = []

        for person in people:
            p_x1, p_y1, p_x2, p_y2 = self._bbox(person)
            px = (p_x1 + p_x2) / 2.0
            near_x = (x1 - sofa_width * 1.2) <= px <= (x2 + sofa_width * 1.2)
            bottom_near_seat = p_y2 >= y1 - 20.0
            torso_reaches_seat = p_y1 <= y2 + 30.0
            lower_body_near_sofa = p_y2 <= y2 + max(80.0, (y2 - y1) * 0.45)

            if not (near_x and bottom_near_seat and torso_reaches_seat and lower_body_near_sofa):
                continue
            if px < x1:
                left_people.append((p_x1, p_x2))
            elif px > x2:
                right_people.append((p_x1, p_x2))

        if not left_people or not right_people:
            return sofa

        expanded_x1 = min([x1] + [p[0] for p in left_people])
        expanded_x2 = max([x2] + [p[1] for p in right_people])
        used_people = len(left_people) + len(right_people)

        expanded = dict(sofa)
        expanded['bbox'] = [
            max(0.0, expanded_x1),
            y1,
            min(float(frame_width), expanded_x2),
            y2,
        ]
        expanded['raw_sofa_bbox'] = [x1, y1, x2, y2]
        expanded['expanded_with_people_count'] = used_people
        return expanded

    def _median(self, values, default):
        values = sorted(values)
        if not values:
            return default
        mid = len(values) // 2
        if len(values) % 2:
            return values[mid]
        return (values[mid - 1] + values[mid]) / 2.0

    def _person_depth(self, person):
        z = float(person.get('distance_z', 999.0))
        return z if 0.1 < z < 20.0 else None

    def _filter_foreground_people(self, people):
        if not people:
            return [], []

        with_geometry = [p for p in people if self._bbox_width(p) > 5 and self._bbox_height(p) > 10]
        if not with_geometry:
            return [], people

        depths = [self._person_depth(p) for p in with_geometry]
        valid_depths = [z for z in depths if z is not None]
        max_bottom = max(self._bbox(p)[3] for p in with_geometry)
        max_height = max(self._bbox_height(p) for p in with_geometry)

        kept = []
        rejected = []
        nearest_z = min(valid_depths) if valid_depths else None
        for person in with_geometry:
            z = self._person_depth(person)
            _, _, _, y2 = self._bbox(person)
            height = self._bbox_height(person)

            near_by_depth = nearest_z is None or z is None or z <= nearest_z + 1.2
            low_enough = y2 >= max_bottom - 130.0
            large_enough = height >= max_height * 0.55

            if near_by_depth and (low_enough or large_enough):
                kept.append(person)
            else:
                rejected.append(person)

        return kept, rejected

    def _row_bounds_from_detections(self, seating_objects, people, frame_width):
        boxes = [self._bbox(o) for o in seating_objects]
        person_boxes = [self._bbox(p) for p in people]

        if not boxes and not person_boxes:
            return None

        all_boxes = boxes if boxes else person_boxes
        x1 = min(b[0] for b in all_boxes)
        x2 = max(b[2] for b in all_boxes)
        y1 = min(b[1] for b in all_boxes)
        y2 = max(b[3] for b in all_boxes)

        for p_box in person_boxes:
            x1 = min(x1, p_box[0])
            x2 = max(x2, p_box[2])
            y2 = max(y2, p_box[3])

        object_widths = [self._bbox_width(o) for o in seating_objects]
        person_widths = [self._bbox_width(p) for p in people]
        slot_width = self._median(object_widths, frame_width * 0.18)
        if object_widths and len(seating_objects) < self.expected_seat_count:
            slot_width = max(slot_width, self._median(person_widths, slot_width) * 0.85)
        elif person_widths:
            slot_width = max(slot_width, self._median(person_widths, slot_width) * 0.9)

        min_row_width = slot_width * self.expected_seat_count
        current_width = max(1.0, x2 - x1)
        if current_width < min_row_width:
            center = (x1 + x2) / 2.0
            x1 = center - min_row_width / 2.0
            x2 = center + min_row_width / 2.0

        x1 = max(0.0, x1)
        x2 = min(float(frame_width), x2)
        if x2 - x1 < min_row_width * 0.7:
            center = (x1 + x2) / 2.0
            x1 = max(0.0, center - min_row_width / 2.0)
            x2 = min(float(frame_width), center + min_row_width / 2.0)

        return [x1, y1, x2, y2]

    def _build_uniform_seat_slots(self, row_bbox, source_label, inferred):
        x1, y1, x2, y2 = row_bbox
        width = max(1.0, x2 - x1)
        seats = []
        for i in range(self.expected_seat_count):
            sx1 = x1 + width * i / self.expected_seat_count
            sx2 = x1 + width * (i + 1) / self.expected_seat_count
            seats.append(self._seat_from_bbox(
                [sx1, y1, sx2, y2],
                source_label=source_label,
                source_bbox=row_bbox,
                split_count=self.expected_seat_count,
                split_part=i + 1,
                description=f"the {self._part_name(i, self.expected_seat_count)} of the seating row",
                inferred=inferred,
            ))
        return seats

    def _build_seat_candidates(self, seating_objects, frame_width, people=None):
        people = people or []
        objects = self._dedupe_seating_objects(seating_objects)
        if not objects and not people:
            return []

        split_target = self._select_primary_sofa(objects, frame_width)
        if split_target:
            split_target = self._expand_sofa_bbox_with_seated_people(
                split_target, people, frame_width)
            can_split, slot_width = self._is_wide_enough_to_split(
                split_target, self.expected_seat_count)
            if not can_split:
                x1, y1, x2, y2 = self._bbox(split_target)
                if hasattr(self, 'node') and self.node:
                    self.node.get_logger().info(
                        f"[YOLO SEAT] Sofa too narrow to split -> "
                        f"width={self._bbox_width(split_target):.1f}, "
                        f"slot_width={slot_width:.1f}, "
                        f"min_slot_width={self.min_split_seat_width_px:.1f}"
                    )
                return self._finalize_seat_indices([
                    self._seat_from_bbox(
                        [x1, y1, x2, y2],
                        source_label=split_target.get('label', 'sofa'),
                        source_bbox=[x1, y1, x2, y2],
                        description=f"the {self._source_name(split_target)}",
                    )
                ])
            seats = self._split_object_into_seats(split_target, self.expected_seat_count)
            split_box = self._bbox(split_target)
            for obj in objects:
                if obj is split_target:
                    continue
                if obj.get('label') in self.SPLIT_LABELS:
                    continue
                obj_center = self._bbox_center_x(obj)
                obj_box = self._bbox(obj)
                if split_box[0] <= obj_center <= split_box[2] and self._bbox_iou(obj_box, split_box) > 0.05:
                    continue
                x1, y1, x2, y2 = obj_box
                seats.append(self._seat_from_bbox(
                    [x1, y1, x2, y2],
                    source_label=obj.get('label', 'seat'),
                    source_bbox=[x1, y1, x2, y2],
                ))
            return self._finalize_seat_indices(seats)

        chairs = [o for o in objects if o.get('label') not in self.SPLIT_LABELS]
        if chairs:
            seats = []
            for obj in chairs:
                x1, y1, x2, y2 = self._bbox(obj)
                seats.append(self._seat_from_bbox(
                    [x1, y1, x2, y2],
                    source_label=obj.get('label', 'seat'),
                    source_bbox=[x1, y1, x2, y2],
                ))
            return self._finalize_seat_indices(seats)

        seats = []
        for obj in objects:
            x1, y1, x2, y2 = self._bbox(obj)
            seats.append(self._seat_from_bbox(
                [x1, y1, x2, y2],
                source_label=obj.get('label', 'seat'),
                source_bbox=[x1, y1, x2, y2],
            ))

        return self._finalize_seat_indices(seats)

    def _finalize_seat_indices(self, seats):
        seats.sort(key=lambda s: s['center_x'])
        for i, seat in enumerate(seats):
            if not seat['description']:
                seat['description'] = f"the {self._ordinal(i + 1)} seat from the left"
            seat['index'] = i + 1
        return seats

    def _ordinal(self, n):
        return {1: "first", 2: "second", 3: "third", 4: "fourth", 5: "fifth"}.get(n, f"{n}th")

    def _assign_people_to_seats(self, people, seats):
        assignments = []
        for p_idx, person in enumerate(people):
            p_box = self._bbox(person)
            px = (p_box[0] + p_box[2]) / 2.0
            py = (p_box[1] + p_box[3]) / 2.0
            candidates = []
            for seat in seats:
                s_box = seat['bbox']
                source_overlap = self._bbox_overlap_ratio(p_box, seat['source_bbox'])
                vertical_overlap = not (p_box[3] < s_box[1] or p_box[1] > s_box[3])
                slot_margin = max(25.0, (s_box[2] - s_box[0]) * 0.25)
                contains_x = (s_box[0] - slot_margin) <= px <= (s_box[2] + slot_margin)
                distance = abs(px - seat['center_x'])
                max_distance = max(45.0, (s_box[2] - s_box[0]) * 0.75)
                seat_h = max(1.0, s_box[3] - s_box[1])
                lower_band_top = s_box[1] + seat_h * 0.15
                lower_band_bottom = s_box[3] + max(40.0, seat_h * 0.35)
                lower_body_on_seat = lower_band_top <= p_box[3] <= lower_band_bottom
                person_large_enough = self._bbox_height(person) >= max(55.0, seat_h * 0.65)
                if (
                    contains_x
                    and distance <= max_distance
                    and person_large_enough
                    and lower_body_on_seat
                    and (vertical_overlap or source_overlap > 0.02 or seat.get('inferred'))
                ):
                    candidates.append((distance, seat, source_overlap))

            if not candidates:
                continue

            _, seat, overlap = min(candidates, key=lambda x: x[0])
            z = self._person_depth(person)
            priority = (
                z if z is not None else 999.0,
                -p_box[3],
                abs(px - seat['center_x']),
            )
            assignments.append((seat, priority, p_idx, px, py, overlap, person))

        best_by_seat = {}
        for seat, priority, p_idx, px, py, overlap, person in assignments:
            seat_index = seat['index']
            if seat_index not in best_by_seat or priority < best_by_seat[seat_index][0]:
                best_by_seat[seat_index] = (priority, p_idx, px, py, overlap, person)

        for seat in seats:
            selected = best_by_seat.get(seat['index'])
            if not selected:
                continue
            _, p_idx, px, py, overlap, person = selected
            seat['occupied'] = True
            seat['occupant'] = {
                'person_index': p_idx + 1,
                'person_x': px,
                'person_y': py,
                'overlap': overlap,
                'distance_z': person.get('distance_z', 999.0),
                'crop_path': person.get('crop_path'),
                'bbox': person.get('bbox'),
            }

        self._mark_person_overlap_unsafe_seats(people, seats)

    def _mark_person_overlap_unsafe_seats(self, people, seats):
        """人物bboxがかかる座席を保守的に塞ぎ、誤って人を指さすのを防ぐ。"""
        for p_idx, person in enumerate(people):
            p_box = self._bbox(person)
            p_w = self._bbox_width(person)
            p_h = self._bbox_height(person)
            if p_w <= 5.0 or p_h <= 40.0:
                continue

            p_x1, _, p_x2, p_y2 = p_box
            px = (p_x1 + p_x2) / 2.0

            for seat in seats:
                s_box = seat['bbox']
                s_x1, s_y1, s_x2, s_y2 = s_box
                seat_w = max(1.0, s_x2 - s_x1)
                seat_h = max(1.0, s_y2 - s_y1)

                x_left = max(p_x1, s_x1)
                x_right = min(p_x2, s_x2)
                overlap_w = max(0.0, x_right - x_left)
                slot_overlap = overlap_w / seat_w
                person_overlap = overlap_w / max(1.0, p_w)

                lower_band_top = s_y1 + seat_h * 0.10
                lower_band_bottom = s_y2 + max(55.0, seat_h * 0.45)
                lower_body_near_seat = lower_band_top <= p_y2 <= lower_band_bottom
                person_large_enough = p_h >= max(55.0, seat_h * 0.55)
                center_near_slot = (s_x1 - seat_w * 0.10) <= px <= (s_x2 + seat_w * 0.10)

                if not (lower_body_near_seat and person_large_enough):
                    continue

                if (
                    slot_overlap >= 0.28
                    or person_overlap >= 0.25
                    or (center_near_slot and slot_overlap >= 0.12)
                ):
                    if not seat.get('occupied'):
                        seat['occupied'] = True
                        seat['occupant'] = {
                            'person_index': f"unsafe_{p_idx + 1}",
                            'person_x': px,
                            'person_y': (p_box[1] + p_box[3]) / 2.0,
                            'overlap': max(slot_overlap, person_overlap),
                            'distance_z': person.get('distance_z', 999.0),
                            'crop_path': person.get('crop_path'),
                            'bbox': person.get('bbox'),
                            'unsafe_overlap': True,
                        }
                    else:
                        occupant = seat.get('occupant') or {}
                        occupant['unsafe_overlap'] = True
                        occupant['overlap'] = max(
                            float(occupant.get('overlap', 0.0)),
                            slot_overlap,
                            person_overlap,
                        )
                        seat['occupant'] = occupant

    def _recover_seated_people(self, people, seating_objects, foreground_people):
        recovered = list(foreground_people)
        seen = {id(p) for p in recovered}
        objects = self._dedupe_seating_objects(seating_objects)
        for person in people:
            if id(person) in seen:
                continue
            p_box = self._bbox(person)
            px = (p_box[0] + p_box[2]) / 2.0
            p_height = self._bbox_height(person)
            for obj in objects:
                o_box = self._bbox(obj)
                o_h = max(1.0, o_box[3] - o_box[1])
                x_margin = max(35.0, self._bbox_width(obj) * 0.20)
                x_near = (o_box[0] - x_margin) <= px <= (o_box[2] + x_margin)
                lower_near = p_box[3] >= o_box[1] + o_h * 0.15
                overlaps = self._bbox_overlap_ratio(p_box, o_box) > 0.015
                if x_near and lower_near and overlaps and p_height >= max(55.0, o_h * 0.55):
                    recovered.append(person)
                    seen.add(id(person))
                    break
        return recovered

    def _mark_saved_seat_occupied(self, seats, attr_name, label):
        if not self.direct_arm or not hasattr(self.direct_arm, attr_name):
            return
        saved_x = getattr(self.direct_arm, attr_name)
        if saved_x is None or not seats:
            return
        seat = min(seats, key=lambda s: abs(s['center_x'] - saved_x))
        slot_width = max(40.0, seat['bbox'][2] - seat['bbox'][0])
        if abs(seat['center_x'] - saved_x) <= slot_width * 0.65:
            seat['occupied'] = True
            seat['occupant'] = seat.get('occupant') or {
                'person_index': label,
                'person_x': saved_x,
                'person_y': None,
                'overlap': 0.0,
                'distance_z': 999.0,
            }

    def _save_host_if_needed(self, occupied_seats, seat_waist_yaws):
        if not self.direct_arm or self.guest_index != 1:
            return
        if hasattr(self.direct_arm, 'host_seat_center_x'):
            return
        if not occupied_seats:
            return
        host_seat = min(occupied_seats, key=lambda s: s['center_x'])
        self.direct_arm.host_seat_index = host_seat['index']
        self.direct_arm.host_seat_center_x = host_seat['center_x']
        self.direct_arm.host_waist_yaw = seat_waist_yaws[host_seat['index']]
        self.node.get_logger().info(
            f"[YOLO SEAT] Fixed host seat -> index={host_seat['index']}, "
            f"desc='{host_seat['description']}', center={host_seat['center_x']:.1f}, "
            f"waist={self.direct_arm.host_waist_yaw:.3f}"
        )

    def _save_guest_seat(self, empty_seat, seat_waist_yaws):
        if not self.direct_arm:
            return
        prefix = f"guest{self.guest_index}"
        setattr(self.direct_arm, f"{prefix}_seat_index", empty_seat['index'])
        setattr(self.direct_arm, f"{prefix}_seat_center_x", empty_seat['center_x'])
        setattr(self.direct_arm, f"{prefix}_waist_yaw", seat_waist_yaws[empty_seat['index']])

    def _refresh_guest1_seat_if_needed(self, occupied_seats, seat_waist_yaws):
        if not self.direct_arm or self.guest_index < 2 or not occupied_seats:
            return
        old_x = getattr(self.direct_arm, 'guest1_seat_center_x', None)
        if old_x is None:
            return

        seat = min(occupied_seats, key=lambda s: abs(s['center_x'] - old_x))
        slot_width = max(40.0, seat['bbox'][2] - seat['bbox'][0])
        if abs(seat['center_x'] - old_x) > slot_width * 1.5:
            return

        self.direct_arm.guest1_seat_index = seat['index']
        self.direct_arm.guest1_seat_center_x = seat['center_x']
        self.direct_arm.guest1_waist_yaw = seat_waist_yaws[seat['index']]
        if hasattr(self, 'node') and self.node:
            self.node.get_logger().info(
                f"[YOLO SEAT] Refreshed Guest 1 seat for introduction -> "
                f"index={seat['index']}, center={seat['center_x']:.1f}, "
                f"waist={self.direct_arm.guest1_waist_yaw:.3f}"
            )

    def _select_empty_seat(self, empty_seats):
        if not empty_seats:
            return None

        sofa_seats = [s for s in empty_seats if s.get('source_label') in self.SPLIT_LABELS]
        if sofa_seats:
            edge_order = [1, self.expected_seat_count]
            if self.guest_index >= 2:
                edge_order.reverse()
            for part in edge_order:
                for seat in sofa_seats:
                    if seat.get('split_part') == part:
                        return seat
            return min(
                sofa_seats,
                key=lambda s: (
                    1 if s.get('split_part') == 2 else 0,
                    s.get('split_part', s['index']),
                )
            )

        return min(empty_seats, key=lambda s: s['index'])

    def _seat_stability_key(self, seat):
        if seat.get('source_label') in self.SPLIT_LABELS:
            return ('sofa', seat.get('split_part'))
        return (seat.get('source_label'), seat.get('index'))

    def _seat_side_name(self, seat):
        if seat.get('split_count') == 3:
            return {
                1: "left side",
                2: "middle",
                3: "right side",
            }.get(seat.get('split_part'), seat.get('description', 'seat'))
        return seat.get('description', 'seat')

    def _build_selected_seat_speech(self, selected_seat, seats):
        if not selected_seat:
            return None

        if selected_seat.get('source_label') in self.SPLIT_LABELS:
            if selected_seat.get('split_count', 1) <= 1:
                return (
                    "From my point of view, the sofa in front of me is empty. "
                    "Please sit there."
                )

            selected_side = self._seat_side_name(selected_seat)
            base = (
                f"From my point of view, the {selected_side} of the sofa "
                f"in front of me is empty. Please sit there."
            )
            if not ENABLE_OCCUPANT_CONTEXT_SPEECH:
                return base

            occupied_sofa = [
                s for s in seats
                if s.get('source_label') in self.SPLIT_LABELS
                and s.get('occupied')
                and s.get('index') != selected_seat.get('index')
            ]
            if not occupied_sofa:
                return base

            occupied_sides = [self._seat_side_name(s) for s in occupied_sofa]
            if len(occupied_sides) == 1:
                occupied_text = f"Someone is on the {occupied_sides[0]} of the sofa"
            else:
                occupied_text = (
                    "People are on "
                    + " and ".join([f"the {side}" for side in occupied_sides])
                    + " of the sofa"
                )
            return (
                f"{occupied_text}. From my point of view, the {selected_side} of the sofa "
                f"is empty. Please sit there."
            )

        chair_seats = [s for s in seats if s.get('source_label') not in self.SPLIT_LABELS]
        chair_order = 1
        for i, seat in enumerate(sorted(chair_seats, key=lambda s: s['center_x']), start=1):
            if seat is selected_seat or seat.get('index') == selected_seat.get('index'):
                chair_order = i
                break
        return (
            f"From my point of view, the {self._ordinal(chair_order)} chair "
            f"from the left is empty. Please sit there."
        )

    def _waist_yaw_for_x(self, x_center, current_waist):
        angle_rad = (self.image_width / 2.0 - x_center) * (87.0 / self.image_width) * math.pi / 180.0
        target_turn = angle_rad * 1.5
        target_waist = current_waist + (target_turn * 0.8)
        return max(-1.2, min(1.2, target_waist)), target_turn

    def _start_yolo_for_seats(self):
        cmd = {
            "command": "start",
            "classes": list(self.target_classes),
            "save_crops": self.guest_index >= 2,
        }
        msg = String()
        msg.data = json.dumps(cmd)
        self.cmd_pub.publish(msg)
        self.node.get_logger().info(f"[YOLO] START classes={self.target_classes}, save_crops={cmd['save_crops']}")

    def _maybe_start_guest1_vlm_match(self, occupied_seats, seat_waist_yaws):
        if not self.direct_arm or self.guest_index < 2 or match_reference_person is None:
            return
        if getattr(self.direct_arm, 'guest1_vlm_match_running', False):
            return

        ref_path = getattr(self.direct_arm, 'guest1_reference_crop_path', None)
        if not ref_path or not os.path.exists(ref_path):
            return

        candidates = []
        for seat in occupied_seats:
            occupant = seat.get('occupant') or {}
            crop_path = occupant.get('crop_path')
            if not crop_path or not os.path.exists(crop_path):
                continue
            bbox = occupant.get('bbox') or seat.get('bbox')
            if not bbox or len(bbox) != 4:
                continue
            bbox_h = max(1.0, float(bbox[3]) - float(bbox[1]))
            if bbox_h < 55.0:
                continue
            candidates.append({
                'seat': seat,
                'crop_path': crop_path,
                'waist_yaw': seat_waist_yaws.get(seat['index']),
            })

        if len(candidates) < 2:
            return

        old_x = getattr(self.direct_arm, 'guest1_seat_center_x', None)
        position_candidate = None
        if old_x is not None:
            position_candidate = min(candidates, key=lambda c: abs(c['seat']['center_x'] - old_x))

        candidates = sorted(candidates, key=lambda c: c['seat']['center_x'])[:2]
        self.direct_arm.guest1_vlm_match_running = True
        self.direct_arm.guest1_vlm_match_status = "running"

        def worker():
            try:
                ref_img = cv2.imread(ref_path)
                a_img = cv2.imread(candidates[0]['crop_path'])
                b_img = cv2.imread(candidates[1]['crop_path'])
                result = match_reference_person(ref_img, a_img, b_img, timeout=30.0)
                match = str(result.get('match', 'uncertain')).strip().upper()
                try:
                    confidence = float(result.get('confidence', 0.0))
                except (TypeError, ValueError):
                    confidence = 0.0

                vlm_candidate = None
                if match == 'A':
                    vlm_candidate = candidates[0]
                elif match == 'B':
                    vlm_candidate = candidates[1]

                selected = None
                if vlm_candidate is not None:
                    if position_candidate is None and confidence >= 0.70:
                        selected = vlm_candidate
                    elif vlm_candidate is position_candidate:
                        selected = vlm_candidate
                    elif confidence >= 0.85:
                        selected = vlm_candidate
                    else:
                        selected = position_candidate
                elif position_candidate is not None:
                    selected = position_candidate

                if selected and selected.get('waist_yaw') is not None:
                    seat = selected['seat']
                    self.direct_arm.guest1_seat_index = seat['index']
                    self.direct_arm.guest1_seat_center_x = seat['center_x']
                    self.direct_arm.guest1_waist_yaw = selected['waist_yaw']
                    self.direct_arm.guest1_vlm_match_status = (
                        f"selected seat {seat['index']} by "
                        f"{'vlm' if selected is vlm_candidate else 'position'} "
                        f"(vlm={match}, conf={confidence:.2f})"
                    )
                    self.node.get_logger().info(
                        f"[YOLO SEAT] Guest1 VLM match -> {self.direct_arm.guest1_vlm_match_status}"
                    )
                else:
                    self.direct_arm.guest1_vlm_match_status = f"uncertain (vlm={match}, conf={confidence:.2f})"
            except Exception as e:
                self.direct_arm.guest1_vlm_match_status = f"failed: {e}"
                self.node.get_logger().warn(f"[YOLO SEAT] Guest1 VLM match failed: {e}")
            finally:
                self.direct_arm.guest1_vlm_match_running = False

        threading.Thread(target=worker, daemon=True).start()

    def execute(self, userdata):
        self.node.get_logger().info("[YOLO SEAT] Searching for empty seat...")
        self._start_yolo_for_seats()
        stable_key = None
        stable_since = None
        stable_frames = 0

        for _ in range(30):
            msg = self.wait_for_result(timeout=min(self.timeout, 1.0))
            if msg:
                try:
                    detections = json.loads(msg)
                    people = [
                        d for d in detections
                        if d.get('label') == 'person' and self._det_conf(d) >= 0.7
                    ]
                    foreground_people, background_people = self._filter_foreground_people(people)
                    seating_objects = [
                        d for d in detections
                        if d.get('label') in self.SEATING_LABELS and self._det_conf(d) >= 0.55
                    ]
                    visible_sofa_count = sum(1 for d in seating_objects if d.get('label') in self.SPLIT_LABELS)
                    visible_chair_count = sum(1 for d in seating_objects if d.get('label') not in self.SPLIT_LABELS)
                    seating_people = self._recover_seated_people(
                        people, seating_objects, foreground_people)

                    max_x = max([self.image_width] + [self._bbox(d)[2] for d in detections if d.get('bbox')])
                    frame_width = max(self.image_width, max_x)
                    snapshot_seats = self._snapshot_seats_available()
                    if snapshot_seats:
                        seats = snapshot_seats
                        self.node.get_logger().info(
                            f"[YOLO SEAT] Using saved seat layout snapshot: seats={len(seats)}"
                        )
                    else:
                        seats = self._build_seat_candidates(seating_objects, frame_width, seating_people)
                    self._assign_people_to_seats(seating_people, seats)
                    if self.guest_index >= 2:
                        self._mark_saved_seat_occupied(seats, 'host_seat_center_x', 'host')
                        self._mark_saved_seat_occupied(seats, 'guest1_seat_center_x', 'guest1')

                    if not seats:
                        self.node.get_logger().warn(
                            f"[YOLO SEAT] No seating objects. detections={len(detections)}, people={len(people)}"
                        )
                        continue

                    if self.guest_index == 1 and self.direct_arm:
                        self.direct_arm.saved_chair_x_centers = [s['center_x'] for s in seats]
                        self.direct_arm.seat_layout_snapshot = self._snapshot_from_seats(seats)
                        self.node.get_logger().info(
                            f"[YOLO SEAT] Saved initial seat centers: {self.direct_arm.saved_chair_x_centers}"
                        )
                        self.node.get_logger().info(
                            f"[YOLO SEAT] Saved seat layout snapshot: seats={len(self.direct_arm.seat_layout_snapshot)}"
                        )

                    empty_seats = [s for s in seats if not s['occupied']]
                    occupied_seats = [s for s in seats if s['occupied']]
                    raw_log = [
                        f"{d.get('label')} conf={self._det_conf(d):.2f} bbox={[int(v) for v in self._bbox(d)]}"
                        for d in detections
                        if d.get('label') in self.SEATING_LABELS or d.get('label') == 'person'
                    ]
                    seat_status_log = []
                    for s in seats:
                        occupant = s.get('occupant')
                        status = "Occupied" if s['occupied'] else "Empty"
                        detail = ""
                        if occupant:
                            detail = (
                                f", person={occupant['person_index']}, "
                                f"px={occupant['person_x']:.1f}, "
                                f"z={occupant['distance_z']:.2f}, overlap={occupant['overlap']:.2f}"
                            )
                        seat_status_log.append(
                            f"{s['index']}:{status} center={s['center_x']:.1f} "
                            f"src={s['source_label']} split={s['split_part']}/{s['split_count']} "
                            f"inferred={s.get('inferred', False)} desc='{s['description']}'{detail}"
                        )

                    self.node.get_logger().info(
                        f"[YOLO SEAT] Raw detections -> {', '.join(raw_log) if raw_log else 'none'}"
                    )
                    self.node.get_logger().info(
                        f"[YOLO SEAT] Summary -> seating_objects={len(seating_objects)}, "
                        f"visible_sofas={visible_sofa_count}, visible_chairs={visible_chair_count}, "
                        f"people={len(people)}, foreground_people={len(foreground_people)}, "
                        f"seating_people={len(seating_people)}, "
                        f"background_people={len(background_people)}, seats={len(seats)}, empty={len(empty_seats)}"
                    )
                    if background_people:
                        bg_log = [
                            f"bbox={[int(v) for v in self._bbox(p)]} z={p.get('distance_z', 999.0):.2f}"
                            for p in background_people[:6]
                        ]
                        self.node.get_logger().info(
                            f"[YOLO SEAT] Ignored background people -> {', '.join(bg_log)}"
                        )
                    self.node.get_logger().info(
                        f"[YOLO SEAT] Seat status left-to-right -> {' | '.join(seat_status_log)}"
                    )

                    if empty_seats:
                        empty_seat = self._select_empty_seat(empty_seats)
                        candidate_key = self._seat_stability_key(empty_seat)
                        now = time.time()
                        if candidate_key != stable_key:
                            stable_key = candidate_key
                            stable_since = now
                            stable_frames = 1
                        else:
                            stable_frames += 1

                        stable_elapsed = 0.0 if stable_since is None else now - stable_since
                        if (
                            stable_frames < self.seat_stable_frames
                            or stable_elapsed < self.seat_stable_seconds
                        ):
                            self.node.get_logger().info(
                                f"[YOLO SEAT] Waiting for stable empty seat -> "
                                f"candidate={candidate_key}, frames={stable_frames}/"
                                f"{self.seat_stable_frames}, elapsed={stable_elapsed:.1f}/"
                                f"{self.seat_stable_seconds:.1f}s"
                            )
                            time.sleep(0.25)
                            continue

                        empty_idx = empty_seat['index']
                        userdata.empty_seat_index = empty_idx

                        if self.direct_arm:
                            current_waist = self.direct_arm.current_joints.get('waist_yaw_joint', 0.0)
                            seat_waist_yaws = {}
                            seat_turns = {}
                            for seat in seats:
                                waist_yaw, turn = self._waist_yaw_for_x(seat['center_x'], current_waist)
                                seat_waist_yaws[seat['index']] = waist_yaw
                                seat_turns[seat['index']] = turn

                            self.direct_arm.empty_seat_index = empty_idx
                            self.direct_arm.selected_seat_description = empty_seat['description']
                            self.direct_arm.selected_seat_speech = self._build_selected_seat_speech(empty_seat, seats)
                            self.direct_arm.selected_seat_source_label = empty_seat.get('source_label')
                            self.direct_arm.selected_seat_safe = True
                            self.direct_arm.seat_waist_yaws = seat_waist_yaws
                            self.direct_arm.seat_candidates_debug = seats

                            self._save_host_if_needed(occupied_seats, seat_waist_yaws)
                            self._refresh_guest1_seat_if_needed(occupied_seats, seat_waist_yaws)
                            self._save_guest_seat(empty_seat, seat_waist_yaws)
                            self._maybe_start_guest1_vlm_match(occupied_seats, seat_waist_yaws)

                            current_guest_waist = seat_waist_yaws[empty_idx]
                            target_turn = seat_turns[empty_idx]
                            self.node.get_logger().info(
                                f"[YOLO SEAT] Selected empty seat -> index={empty_idx}, "
                                f"desc='{empty_seat['description']}', center={empty_seat['center_x']:.1f}, "
                                f"turn={target_turn:.3f}, waist={current_guest_waist:.3f}"
                            )
                            self.direct_arm.turn_waist_towards(target_turn, hold_sec=0.0)

                        self.stop_yolo()
                        return 'success'

                    self.node.get_logger().warn("[YOLO SEAT] No empty seat found in current frame.")

                except Exception as e:
                    self.node.get_logger().warn(f"  -> Parse error: {e}")
                    # 例外が出てもクラッシュさせずリトライする
            time.sleep(1.0)

        self.stop_yolo()
        if self.direct_arm:
            # フォールバック時もインデックスの更新を確実に行う
            fallback_idx = 1 if self.guest_index == 1 else 2
            userdata.empty_seat_index = fallback_idx
            self.direct_arm.empty_seat_index = fallback_idx
            self.direct_arm.selected_seat_description = None
            self.direct_arm.selected_seat_speech = None
            self.direct_arm.selected_seat_source_label = None
            self.direct_arm.selected_seat_safe = False
            
            self.node.get_logger().warn(
                "[YOLO SEAT] No safe empty seat confirmed. Skipping fallback pointing."
            )
        return 'failure'


# ============================================================
# バッグ検出
# ============================================================
class YoloFindBagState(BaseYoloState):
    """YOLO で bag を検出するステート。"""

    def __init__(self, node, timeout=5.0):
        super().__init__(node, target_classes=["bag"], timeout=timeout)

    def execute(self, userdata):
        self.node.get_logger().info("[YOLO BAG] Searching for bag...")
        self.start_yolo()

        start_time = time.time()
        detected = False
        while rclpy.ok() and time.time() - start_time < self.timeout:
            msg = self.wait_for_result(timeout=1.0)
            if not msg:
                continue
            try:
                detections = json.loads(msg)
            except json.JSONDecodeError:
                self.node.get_logger().warn("  -> Invalid YOLO JSON while searching for bag.")
                continue

            bags = [d for d in detections if d.get('label') == 'bag']
            if bags:
                summary = [
                    f"conf={float(b.get('confidence', b.get('score', 0.0))):.2f}, "
                    f"z={float(b.get('distance_z', 999.0)):.2f}, bbox={b.get('bbox', [])}"
                    for b in bags[:3]
                ]
                self.node.get_logger().info(
                    f"  -> Bag detected: {' | '.join(summary)}"
                )
                detected = True
                break

            self.node.get_logger().info("  -> No bag in current YOLO result.")

        if not detected:
            self.node.get_logger().info("  -> No bag detected (timeout), continuing anyway")

        self.stop_yolo()
        return 'success'


# ============================================================
# ホスト追従: 止まったら終了
# ============================================================
class YoloFollowHostState(BaseYoloState):
    """
    YOLO で person を追従し、ホストが止まったことを検知して終了する。
    """

    def __init__(self, node, tts_say=None, direct_arm=None, control=None,
                 timeout=3.0, max_duration=60.0, stop_threshold=0.05,
                 stop_count_required=10, stop_distance=0.8):
        super().__init__(node, target_classes=["person"], timeout=timeout)
        self.tts_say = tts_say
        self.direct_arm = direct_arm
        self.control = control
        self.max_duration = max_duration
        self.stop_threshold = stop_threshold
        self.stop_count_required = stop_count_required
        self.stop_distance = stop_distance

    def execute(self, userdata):
        self.node.get_logger().info("[FOLLOW HOST] Starting host follow...")
        if self.tts_say:
            self.tts_say("Host, please guide me to your destination.")
            time.sleep(1.0)
            
        self.start_yolo()

        if self.control:
            try:
                self.control.pose_policy('running')
                self.control.move_head(tilt=0.0)
            except Exception as e:
                self.node.get_logger().warn(f"  -> Pose policy failed: {e}")

        prev_z = None
        prev_angle = None
        stop_count = 0
        start_time = time.time()
        
        # (以前は移動中の干渉を防ぐために direct_arm を pause していましたが、
        # 現在は10Hz化により両立できるため pause せず、腕の剛性を保ちます)
        if self.direct_arm:
            self.node.get_logger().info("  -> direct_arm remains active to keep joint stiffness.")

        # サブプロセスではなく、スレッド内でトラッカー(YoloHumanTracker)を起動
        self.tracker_node = None
        self.tracker_executor = None
        self.tracker_thread = None
        try:
            self.node.get_logger().info("  -> Starting YoloHumanTracker in background thread...")
            self.tracker_node = YoloHumanTracker()
            self.tracker_executor = SingleThreadedExecutor()
            self.tracker_executor.add_node(self.tracker_node)
            self.tracker_thread = threading.Thread(target=self.tracker_executor.spin, daemon=True)
            self.tracker_thread.start()
        except Exception as e:
            self.node.get_logger().error(f"  -> Failed to start tracker thread: {e}")

        try:
            if self.tts_say:
                self.tts_say("I am following you now.")
                time.sleep(1.0)
                
            while rclpy.ok():
                if time.time() - start_time > self.max_duration:
                    if self.tts_say:
                        self.tts_say("I have followed you for the maximum duration.")
                        time.sleep(1.0)
                    break

                msg = self.wait_for_result(timeout=2.0)
                if msg:
                    # _handle_tracking内で自己完結的に REACHED に移行したかをチェック
                    if self.tracker_node and self.tracker_node.state == 'REACHED':
                        self.node.get_logger().info("  -> Tracker node reports REACHED! Stop condition met.")
                        if self.tts_say:
                            self.tts_say("It seems we have arrived.")
                            time.sleep(1.0)
                        break
                        
                    target, z, angle, w_ratio = self.parse_yolo_target(msg)
                    if target:
                        self.node.get_logger().info(f"  -> Following: Z={z:.2f}m, rad={angle:.2f}")
                
        finally:
            if self.tracker_executor is not None:
                self.node.get_logger().info("  -> Shutting down YoloHumanTracker thread...")
                self.tracker_executor.shutdown()
            if self.tracker_node is not None:
                self.tracker_node.destroy_node()
            if self.tracker_thread is not None:
                self.tracker_thread.join(timeout=2.0)

            # 剛性保持のため pause していないので resume も不要
            if self.direct_arm:
                self.node.get_logger().info("  -> Finished tracking.")

        self.stop_yolo()
        if self.control:
            try:
                self.control.pose_policy('start')
                self.control.move_head(tilt=0.0)
            except Exception as e:
                self.node.get_logger().warn(f"  -> Pose reset failed: {e}")

        return 'success'

# ============================================================
# 右手のみのバッグ把持 (層的インタラクション)
# ============================================================
class YoloBagGraspInteractionState(BaseYoloState):
    """
    ゲストに特徴を伝えた直後に実行されるバッグ把持ステート。
    ルールに基づき、カメラを下に向け、足回りでかばんに近づき（画面中央・50cm）、右手のみで把持位置（上部少し右）へ腕を伸ばす。
    """

    def __init__(self, node, tts_say=None, direct_arm=None, control=None, timeout=8.0):
        super().__init__(node, target_classes=["bag"], timeout=timeout)
        self.tts_say = tts_say
        self.direct_arm = direct_arm
        self.control = control
        self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)

    def _say(self, text):
        if self.tts_say:
            self.tts_say(text)

    def _try_hand_control(self, command, hand="right"):
        if not self.control:
            return False
        try:
            return bool(self.control.hand_control(command=command, hand=hand))
        except Exception as e:
            self.node.get_logger().warn(f"  -> hand_control({command}, {hand}) failed: {e}")
            return False

    def _detect_bag_grasp_target(self, timeout):
        start_time = time.time()
        while rclpy.ok() and (time.time() - start_time < timeout):
            msg = self.wait_for_result(timeout=1.0)
            if not msg:
                continue

            try:
                detections = json.loads(msg)
                bags = [d for d in detections if d.get('label') == 'bag']
                if not bags:
                    continue

                target_bag = min(bags, key=lambda b: b.get('distance_z', 999.0))
                bag_z = target_bag.get('distance_z', 0.3)
                bbox = target_bag.get('bbox', [320, 240, 320, 240])
                if (
                    not isinstance(bbox, list)
                    or len(bbox) != 4
                    or bbox[2] <= bbox[0]
                    or bbox[3] <= bbox[1]
                ):
                    self.node.get_logger().warn(
                        f"  -> Bag bbox invalid; using default image center: {bbox}"
                    )
                    bbox = [320, 240, 320, 240]

                bbox_width = bbox[2] - bbox[0]
                bbox_height = bbox[3] - bbox[1]
                bag_cx = (bbox[0] + bbox[2]) / 2.0

                return {
                    'cx': bag_cx + (bbox_width * 0.15),
                    'cy': bbox[1] + (bbox_height * 0.05),
                    'z': bag_z,
                    'bbox': bbox,
                }

            except Exception as e:
                self.node.get_logger().error(f"  -> Error parsing bag: {e}")

        return None

    def execute(self, userdata):
        self.node.get_logger().info("[YOLO BAG GRASP] Starting visual servoing for bag grasp...")

        try:
            # 0. 把持前ホームポジションへ（右肘を曲げて体にくっつけることでバッグへの接触を防ぐ）
            bag_home = HOME_POSE.copy()
            bag_home['right_elbow_joint'] = -1.0
            
            if self.direct_arm:
                self.node.get_logger().info("  -> Moving to bag grasp home posture...")
                self.direct_arm.send_joints(bag_home, hold_sec=1.5)

            # 1. 頭を下に向ける (かばんを探すため)
            head_tilt_val = -0.5
            if self.control:
                try:
                    self.node.get_logger().info("  -> Tilting head down...")
                    self.control.move_head(tilt=head_tilt_val)
                except Exception as e:
                    self.node.get_logger().warn(f"  -> Head tilt failed: {e}")
            
            time.sleep(1.0)
            self.start_yolo()

            # 2. バッグを一度認識し、前に出してもらってから再認識する。
            final_bag_cx = 320.0
            final_bag_cy = 240.0
            final_bag_z = 0.3

            first_target = self._detect_bag_grasp_target(self.timeout)
            if first_target:
                self.node.get_logger().info(
                    f"  -> Initial bag detected: bbox={first_target['bbox']}, z={first_target['z']}"
                )
                self._say("I will take your bag first. Please hold it out in front of me.")
                time.sleep(3.0)
                second_target = self._detect_bag_grasp_target(self.timeout)
                if second_target:
                    final_bag_cx = second_target['cx']
                    final_bag_cy = second_target['cy']
                    final_bag_z = second_target['z']
                    self.node.get_logger().info(
                        f"  -> Re-detected bag for grasp: bbox={second_target['bbox']}, z={final_bag_z}"
                    )
                else:
                    self.node.get_logger().warn(
                        "  -> Could not re-detect bag. Attempting grasp at default position."
                    )
            else:
                self.node.get_logger().warn(
                    "  -> Could not find bag. Attempting grasp at default position."
                )

            # 3. 把持姿勢を取る
            self.node.get_logger().info(f"  -> Calculating IK for bag at cx={final_bag_cx}, cy={final_bag_cy}, z={final_bag_z}")
            
            target_joints = calculate_bag_grasp_joints(final_bag_cx, final_bag_cy, final_bag_z, head_tilt=head_tilt_val)
            
            if self.direct_arm:
                self.node.get_logger().info(f"  -> Sending Grasp Joints: {target_joints}")
                if not self._try_hand_control("open", "right"):
                    self.node.get_logger().warn(
                        "  -> Opening right hand failed; continuing grasp posture anyway."
                    )
                self.direct_arm.send_joints(target_joints, hold_sec=2.0)
                
                # 手を閉じる前の案内と待機
                self._say("I will close my grip now.")
                
                self.node.get_logger().info("  -> Waiting 3 seconds before closing hand...")
                time.sleep(3.0)
                
                if not self._try_hand_control("close", "right"):
                    self.node.get_logger().warn(
                        "  -> Closing right hand failed; continuing task after grasp posture."
                    )
                time.sleep(1.0)
                
                # 腕を戻す（まずは把持ホームポジションを経由）
                self.node.get_logger().info("  -> Returning via bag grasp home posture...")
                self.direct_arm.send_joints(bag_home, hold_sec=1.5)

            return 'success'

        except Exception as e:
            self.node.get_logger().error(f"  -> Unexpected error in execute: {e}")
            return 'failure'

        finally:
            # 安全のためのクリーンアップ処理（例外発生時も必ず実行）
            try:
                self.stop_yolo()
            except Exception:
                pass
            
            try:
                self.cmd_vel_pub.publish(Twist()) # ベース停止
            except Exception:
                pass
            
            # 頭のピッチを戻す
            if self.control:
                try:
                    self.node.get_logger().info("  -> Resetting head tilt to 0.0...")
                    self.control.move_head(tilt=0.0)
                except Exception as e:
                    self.node.get_logger().warn(f"  -> Head reset failed: {e}")
