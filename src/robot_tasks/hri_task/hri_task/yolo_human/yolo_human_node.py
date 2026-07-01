#!/usr/bin/env python3
"""
YOLO Human Detection Node (Docker用)
/upper_joints_controlではなく、直接カメラトピックから画像を取得してYOLO推論を行う。

使い方:
  - Docker起動時にモデルをロードし、待機状態で起動
  - /yolo_human/command に {"command": "start"} を送ると検出開始
  - /yolo_human/command に {"command": "stop"} を送ると検出停止
  - START_ACTIVE = True にするとコマンドなしで即開始
"""

# ============================================================
#  設定変数（コード内で書き換えて動作を切り替える）
# ============================================================
# True にすると起動直後から検出処理を開始する（commandトピック不要）
START_ACTIVE = True
# True にすると cv2.imshow でリアルタイム表示する（Docker側でX11転送が必要）
ENABLE_IMSHOW = False
# ============================================================

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import math
import sys

from ultralytics import YOLOE

try:
    from extract_person_features import extract_person_features
except Exception as e:
    extract_person_features = None
    print(f"Failed to import extract_person_features: {e}")

try:
    from extract_person_features_off import extract_person_features as extract_person_features_offline
except Exception as e:
    extract_person_features_offline = None
    print(f"Failed to import extract_person_features_offline: {e}")

class YoloHumanNode(Node):
    def __init__(self):
        super().__init__('yolo_human_node')

        # --- カメラトピック（直書き） ---
        rgb_topic = '/head_camera/d455/color/image_raw'
        depth_topic = '/head_camera/d455/depth/image_rect_raw'

        # --- Load YOLO model ---
        model_path = 'yoloe-26x-seg.pt'
        import torch
        if torch.cuda.is_available():
            self.get_logger().info(f"CUDA is available! Using GPU ({torch.cuda.get_device_name(0)})")
        else:
            self.get_logger().warn("CUDA is NOT available! Falling back to CPU.")

        self.get_logger().info(f"Loading YOLO model from {model_path}...")
        self.model = YOLOE(model_path)
        
        # Open-Vocabulary configuration (言語司令)
        self.target_classes = ["person"]
        try:
            self.model.set_classes(self.target_classes)
            self.get_logger().info(f"Initialized with classes: {self.target_classes}")
        except AttributeError:
            self.get_logger().info("Model does not support set_classes, continuing with standard classes.")

        self.get_logger().info("Model loaded successfully.")

        self.bridge = CvBridge()

        # State
        self.is_active = START_ACTIVE
        self.extract_features = False
        self.feature_mode = "online"  # Default
        self.target_classes = ["person"]
        
        # Async Feature Extraction State
        self.online_features_cache = None
        self.api_fetching = False
        self.api_disabled = False

        if self.is_active:
            self.get_logger().info("START_ACTIVE=True: 起動直後から検出開始")
        else:
            self.get_logger().info("START_ACTIVE=False: commandトピックで開始してください")

        # Data cache
        self.latest_rgb = None
        self.latest_depth = None
        self.rgb_received = False
        self.depth_received = False

        # --- QoS: カメラのRELIABLE publisher に合わせる ---
        # RealSenseのデフォルト: RELIABLE, KEEP_LAST(1)
        camera_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.rgb_sub = self.create_subscription(
            Image, rgb_topic, self.rgb_callback, camera_qos)
        self.depth_sub = self.create_subscription(
            Image, depth_topic, self.depth_callback, camera_qos)

        self.get_logger().info(f"Subscribed to RGB: {rgb_topic}")
        self.get_logger().info(f"Subscribed to Depth: {depth_topic}")

        # Command Subscriber
        self.cmd_sub = self.create_subscription(
            String, '/yolo_human/command', self.command_callback, 10)

        # Publishers
        self.debug_pub = self.create_publisher(
            CompressedImage, '/yolo_human/debug_image/compressed', 10)
        self.result_pub = self.create_publisher(
            String, '/yolo_human/result', 10)

        # Timer for processing (10 Hz)
        self.timer = self.create_timer(0.1, self.process_frame)
        self.get_logger().info("Node initialized. Waiting for camera data...")

    def command_callback(self, msg):
        try:
            cmd_data = json.loads(msg.data)
            if "command" in cmd_data:
                if cmd_data["command"] == "start":
                    self.is_active = True
                    self.get_logger().info("YOLO processing STARTED")
                elif cmd_data["command"] == "stop":
                    self.is_active = False
                    self.get_logger().info("YOLO processing STOPPED")

            if "extract_features" in cmd_data:
                new_extract = bool(cmd_data["extract_features"])
                if new_extract and not self.extract_features:
                    self.online_features_cache = None  # Reset cache when turned ON
                self.extract_features = new_extract
                self.get_logger().info(f"Extract features set to {self.extract_features}")
            elif "command" in cmd_data and cmd_data["command"] == "start":
                # Default to False if command is start but no extract_features flag
                self.extract_features = False
                self.online_features_cache = None

            if "feature_mode" in cmd_data:
                self.feature_mode = cmd_data["feature_mode"]
                self.get_logger().info(f"Feature mode set to {self.feature_mode}")

            if "classes" in cmd_data and isinstance(cmd_data["classes"], list):
                self.target_classes = cmd_data["classes"]
                try:
                    self.model.set_classes(self.target_classes)
                except AttributeError:
                    pass
                self.get_logger().info(
                    f"Target classes updated: {self.target_classes}, model.names={self.model.names}"
                )
        except json.JSONDecodeError:
            self.get_logger().error("Invalid command format. Expected JSON.")

    def rgb_callback(self, msg):
        self.latest_rgb = msg
        if not self.rgb_received:
            self.rgb_received = True
            self.get_logger().info("First RGB image received!")

    def depth_callback(self, msg):
        self.latest_depth = msg
        if not self.depth_received:
            self.depth_received = True
            self.get_logger().info("First Depth image received!")

    def _depth_to_meters(self, depth_values):
        depth = np.asarray(depth_values, dtype=np.float32)
        depth = np.where(depth > 100.0, depth / 1000.0, depth)
        return depth

    def _estimate_depth(self, cv_depth, x1, y1, x2, y2, u_center, v_center):
        h, w = cv_depth.shape[:2]
        u_center = max(0, min(u_center, w - 1))
        v_center = max(0, min(v_center, h - 1))

        center_z = float(self._depth_to_meters(cv_depth[v_center, u_center]))
        if np.isfinite(center_z) and 0.1 < center_z < 10.0:
            return center_z, True

        # RealSense の中心画素だけ 0 になる場合があるため、bbox 中央付近の中央値で補完する。
        bx1 = max(0, min(int(x1), w - 1))
        bx2 = max(0, min(int(x2), w))
        by1 = max(0, min(int(y1), h - 1))
        by2 = max(0, min(int(y2), h))
        if bx2 <= bx1 or by2 <= by1:
            return 999.0, False

        margin_x = max(1, int((bx2 - bx1) * 0.25))
        margin_y = max(1, int((by2 - by1) * 0.25))
        roi = cv_depth[by1 + margin_y:by2 - margin_y, bx1 + margin_x:bx2 - margin_x]
        if roi.size == 0:
            roi = cv_depth[by1:by2, bx1:bx2]

        roi_m = self._depth_to_meters(roi).reshape(-1)
        valid = roi_m[np.isfinite(roi_m) & (roi_m > 0.1) & (roi_m < 10.0)]
        if valid.size == 0:
            return 999.0, False

        return float(np.median(valid)), True

    def process_frame(self):
        if not self.is_active:
            return

        if self.latest_rgb is None:
            self.get_logger().warn("Waiting for RGB image...", throttle_duration_sec=2.0)
            return
        if self.latest_depth is None:
            self.get_logger().warn("Waiting for Depth image...", throttle_duration_sec=2.0)
            return

        try:
            cv_rgb = self.bridge.imgmsg_to_cv2(self.latest_rgb, "bgr8")
            cv_depth = self.bridge.imgmsg_to_cv2(self.latest_depth, "passthrough")
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        # Perform inference with bytetrack tracker (GPU指定)
        results = self.model.track(cv_rgb, device=0, verbose=False,
                                   persist=True, tracker="bytetrack.yaml")

        # Fallback: if tracker dropped all boxes, try predict directly
        if len(results) > 0 and len(results[0].boxes) == 0:
            pred_res = self.model.predict(cv_rgb, device=0, verbose=False)
            if len(pred_res) > 0 and len(pred_res[0].boxes) > 0:
                results = pred_res

        detections = []
        debug_img = cv_rgb.copy()

        closest_det_idx = -1
        min_z = float('inf')
        raw_detections_debug = []

        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0])
                label = self.model.names[cls_id]
                conf = float(box.conf[0])
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                raw_detections_debug.append(
                    f"id={cls_id} label={label} conf={conf:.2f} bbox={[x1, y1, x2, y2]}"
                )

                # Filter by target classes
                if label not in self.target_classes:
                    continue

                # BBox coordinates
                u_center = int((x1 + x2) / 2)
                v_center = int((y1 + y2) / 2)

                h, w = cv_depth.shape[:2]
                u_center = max(0, min(u_center, w - 1))
                v_center = max(0, min(v_center, h - 1))

                z, valid_depth = self._estimate_depth(
                    cv_depth, x1, y1, x2, y2, u_center, v_center)

                # FOV-based angle calculation (~69deg = 1.2rad)
                angle_rad = ((u_center - (w / 2.0)) / float(w)) * 1.2
                x_offset = z * math.tan(angle_rad) if valid_depth else 0.0
                bbox_width_ratio = (x2 - x1) / float(w)

                # トラッキングIDを取得
                track_id = -1
                if box.id is not None:
                    track_id = int(box.id[0])

                det_info = {
                    'label': label,
                    'confidence': conf,
                    'bbox': [x1, y1, x2, y2],
                    'bbox_width_ratio': bbox_width_ratio,
                    'distance_z': z,
                    'valid_depth': valid_depth,
                    'angle_rad': angle_rad,
                    'offset_x': x_offset,
                    'track_id': track_id
                }

                detections.append(det_info)
                
                # Track the closest person
                if label == "person" and valid_depth and z < min_z:
                    min_z = z
                    closest_det_idx = len(detections) - 1

                # Draw on debug image
                cv2.rectangle(debug_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                text = f"{label} ID:{track_id} {conf:.2f} Z:{z:.2f}m rad:{angle_rad:.2f}"
                cv2.putText(debug_img, text, (x1, max(y1 - 10, 0)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.circle(debug_img, (u_center, v_center), 5, (0, 0, 255), -1)

        if raw_detections_debug and not detections:
            self.get_logger().info(
                "YOLO raw detections filtered out: "
                f"targets={self.target_classes}, names={self.model.names}, "
                f"raw={'; '.join(raw_detections_debug[:8])}",
                throttle_duration_sec=2.0)

        # Handle Feature Extraction ONLY for the closest person
        if self.extract_features and closest_det_idx >= 0:
            det_info = detections[closest_det_idx]
            x1, y1, x2, y2 = det_info['bbox']
            features = None

            if self.feature_mode in ["online", "offline"]:
                if self.online_features_cache is None and not self.api_fetching:
                    self.api_fetching = True
                    crop = cv_rgb[y1:y2, x1:x2].copy()

                    def fetch():
                        try:
                            if self.feature_mode == "online" and not self.api_disabled and extract_person_features:
                                self.get_logger().info("Calling OpenAI API for the closest person (timeout 8.0s)...")
                                res = extract_person_features(crop, timeout=8.0)
                            elif extract_person_features_offline:
                                self.get_logger().info("Calling Local VLM for the closest person (timeout 30.0s)...")
                                res = extract_person_features_offline(crop, timeout=30.0)
                            else:
                                res = {"error": "NO_EXTRACTOR_AVAILABLE"}
                            
                            self.online_features_cache = res
                            self.get_logger().info(f"{self.feature_mode.upper()} VLM call succeeded. Caching results.")
                        except Exception as e:
                            self.get_logger().error(f"VLM extraction failed: {e}. Returning failure status.")
                            self.online_features_cache = {"error": "API_FAILED"}
                        finally:
                            self.api_fetching = False

                    import threading
                    threading.Thread(target=fetch).start()

                if self.online_features_cache is not None:
                    features = self.online_features_cache


            if features is not None:
                detections[closest_det_idx]['features'] = features

        # Publish results
        res_msg = String()
        res_msg.data = json.dumps(detections)
        self.result_pub.publish(res_msg)

        # Publish debug image (compressed)
        try:
            debug_msg = CompressedImage()
            debug_msg.header.stamp = self.get_clock().now().to_msg()
            debug_msg.format = "jpeg"
            _, encoded = cv2.imencode('.jpg', debug_img,
                                       [cv2.IMWRITE_JPEG_QUALITY, 50])
            debug_msg.data = encoded.tobytes()
            self.debug_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f"Debug image publish error: {e}")

        # Real-time visualization (imshow)
        if ENABLE_IMSHOW:
            cv2.imshow("YOLO Detections", debug_img)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = YoloHumanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"Unexpected error: {e}")
    finally:
        if ENABLE_IMSHOW:
            cv2.destroyAllWindows()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
