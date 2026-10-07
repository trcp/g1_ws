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

RGB_TOPIC = '/head_camera/d455/color/image_raw'
ALIGNED_DEPTH_TOPIC = '/head_camera/d455/aligned_depth_to_color/image_raw'
COLOR_CAMERA_INFO_TOPIC = '/head_camera/d455/color/camera_info'
CAMERA_PARAMETER_SERVICE = '/head_camera/d455/set_parameters'
ALIGNMENT_CHECK_PERIOD_SEC = 2.0
ALIGNED_DEPTH_SILENCE_SEC = 3.0
ALIGNMENT_REQUEST_TIMEOUT_SEC = 5.0
MAX_ALIGNMENT_ATTEMPTS = 3
# ============================================================

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import math
import sys
import os
import shutil
import time
from pathlib import Path

# リポジトリから直接実行する場合は隣のobject/を追加する。Dockerでは
# rgbd_geometry.pyを/appへマウントするため、スクリプトと同じ場所から見つかる。
object_module_dir = Path(__file__).resolve().parent.parent / "object"
if object_module_dir.is_dir():
    sys.path.insert(0, str(object_module_dir))
from rgbd_geometry import FramePairs, camera_from_info, depth_in_meters, stamp_ns

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


def prepare_yolo_weight(model_path: str, logger) -> str:
    """Ensure the YOLO weight file exists before constructing YOLOE."""
    path = Path(model_path)

    if path.is_dir():
        logger.warn(f"{model_path} is a directory. Removing it before model download.")
        shutil.rmtree(path)

    if path.is_file():
        logger.info(f"YOLO weight already exists: {model_path}")
        return str(path)

    logger.info(f"YOLO weight not found. Downloading: {model_path}")
    try:
        from ultralytics.utils.downloads import attempt_download_asset

        downloaded_path = Path(attempt_download_asset(model_path))
        if downloaded_path.is_file():
            logger.info(f"YOLO weight downloaded: {downloaded_path}")
            return str(downloaded_path)
    except Exception as e:
        logger.warn(f"Explicit YOLO weight download failed, falling back to YOLOE loader: {e}")

    return str(path)


def remove_corrupt_torchscript_asset(asset_name: str, logger) -> None:
    """Delete a cached TorchScript asset if PyTorch cannot load it."""
    try:
        import torch
    except Exception as e:
        logger.warn(f"Could not validate {asset_name} before loading: {e}")
        return

    for path in cached_asset_candidates(asset_name):
        if path.is_dir():
            logger.warn(f"{path} is a directory. Removing it before text encoder download.")
            shutil.rmtree(path)
            continue
        if not path.is_file():
            continue

        try:
            torch.jit.load(str(path), map_location="cpu")
        except Exception as e:
            logger.warn(f"Cached {asset_name} is invalid at {path}. Removing it: {e}")
            path.unlink()
        else:
            logger.info(f"Cached {asset_name} is valid: {path}")


def cached_asset_candidates(asset_name: str):
    cache_root = Path.home() / ".cache" / "ultralytics"
    return [
        cache_root / "weights" / asset_name,
        cache_root / "assets" / asset_name,
        Path(asset_name),
        Path("/app") / asset_name,
    ]


def cached_asset_exists(asset_name: str) -> bool:
    return any(path.is_file() and path.stat().st_size > 0 for path in cached_asset_candidates(asset_name))


def ensure_local_cached_asset(asset_name: str, logger) -> bool:
    """Expose a cached asset under /app for libraries that resolve relative paths."""
    app_path = Path("/app") / asset_name
    if app_path.is_file() and app_path.stat().st_size > 0:
        return True

    for path in cached_asset_candidates(asset_name):
        if path == app_path or not path.is_file() or path.stat().st_size <= 0:
            continue
        try:
            if app_path.exists() or app_path.is_symlink():
                app_path.unlink()
            app_path.symlink_to(path)
            logger.info(f"Linked cached {asset_name}: {app_path} -> {path}")
            return True
        except Exception as e:
            logger.warn(f"Could not link cached {asset_name} from {path}: {e}")

    return False


class YoloHumanNode(Node):
    def __init__(self):
        super().__init__('yolo_human_node')

        # --- Load YOLO model ---
        model_path = 'yoloe-26x-seg.pt'
        import torch
        if torch.cuda.is_available():
            self.inference_device = 0
            self.get_logger().info(f"CUDA is available! Using GPU ({torch.cuda.get_device_name(0)})")
        else:
            self.inference_device = "cpu"
            self.get_logger().warn("CUDA is NOT available! Falling back to CPU.")

        model_path = prepare_yolo_weight(model_path, self.get_logger())
        self.get_logger().info(f"Loading YOLO model from {model_path}...")
        self.model = YOLOE(model_path)
        
        # Open-Vocabulary configuration (言語司令)
        self.target_classes = ["person"]
        text_encoder_asset = "mobileclip2_b.ts"
        remove_corrupt_torchscript_asset(text_encoder_asset, self.get_logger())
        self.can_set_classes = ensure_local_cached_asset(text_encoder_asset, self.get_logger())
        if self.can_set_classes:
            try:
                self.model.set_classes(self.target_classes)
                self.get_logger().info(f"Initialized with classes: {self.target_classes}")
            except AttributeError:
                self.get_logger().info("Model does not support set_classes, continuing with standard classes.")
            except Exception as e:
                self.can_set_classes = False
                self.get_logger().warn(
                    f"set_classes failed. Continuing with standard model classes: {e}"
                )
        else:
            self.get_logger().warn(
                f"{text_encoder_asset} is not cached. Skipping set_classes to allow offline startup."
            )

        self.get_logger().info("Model loaded successfully.")

        self.bridge = CvBridge()

        # State
        self.is_active = START_ACTIVE
        self.extract_features = False
        self.feature_mode = "online"  # Default
        self.target_classes = ["person"]
        self.save_crops = False
        self.crop_dir = "/tmp/hri_yolo_crops"
        os.makedirs(self.crop_dir, exist_ok=True)
        
        # Async Feature Extraction State
        self.online_features_cache = None
        self.api_fetching = False
        self.api_disabled = False
        self.feature_crop_path = None

        if self.is_active:
            self.get_logger().info("START_ACTIVE=True: 起動直後から検出開始")
        else:
            self.get_logger().info("START_ACTIVE=False: commandトピックで開始してください")

        # Data cache
        self.frame_pairs = FramePairs()
        self.camera_info = None
        self.alignment_future = None
        self.alignment_requested_at = 0.0
        self.last_depth_received_at = 0.0
        self.alignment_attempts = 0
        self.alignment_client = self.create_client(
            SetParameters,
            CAMERA_PARAMETER_SERVICE,
        )
        self.alignment_timer = self.create_timer(
            ALIGNMENT_CHECK_PERIOD_SEC,
            self.ensure_aligned_depth,
        )
        self.rgb_received = False
        self.depth_received = False

        # --- BEST_EFFORTでRELIABLE/BEST_EFFORTの両カメラ配信に対応 ---
        camera_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.rgb_sub = self.create_subscription(
            Image, RGB_TOPIC, self.rgb_callback, camera_qos)
        self.depth_sub = self.create_subscription(
            Image, ALIGNED_DEPTH_TOPIC, self.depth_callback, camera_qos)

        self.info_sub = self.create_subscription(
            CameraInfo, COLOR_CAMERA_INFO_TOPIC,
            self.camera_info_callback, camera_qos)

        self.get_logger().info(f"Subscribed to RGB: {RGB_TOPIC}")
        self.get_logger().info(f"Subscribed to Depth: {ALIGNED_DEPTH_TOPIC}")

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
                    self.feature_crop_path = None
                self.extract_features = new_extract
                self.get_logger().info(f"Extract features set to {self.extract_features}")
            elif "command" in cmd_data and cmd_data["command"] == "start":
                # Default to False if command is start but no extract_features flag
                self.extract_features = False
                self.online_features_cache = None
                self.feature_crop_path = None

            if "feature_mode" in cmd_data:
                self.feature_mode = cmd_data["feature_mode"]
                self.get_logger().info(f"Feature mode set to {self.feature_mode}")

            if "save_crops" in cmd_data:
                self.save_crops = bool(cmd_data["save_crops"])
                self.get_logger().info(f"Save person crops set to {self.save_crops}")
            elif "command" in cmd_data and cmd_data["command"] == "start":
                self.save_crops = False

            if "classes" in cmd_data and isinstance(cmd_data["classes"], list):
                self.target_classes = cmd_data["classes"]
                if self.can_set_classes:
                    try:
                        self.model.set_classes(self.target_classes)
                    except AttributeError:
                        self.can_set_classes = False
                    except Exception as e:
                        self.can_set_classes = False
                        self.get_logger().warn(
                            f"set_classes failed after command. "
                            f"Continuing with standard model classes: {e}"
                        )
                self.get_logger().info(
                    f"Target classes updated: {self.target_classes}, model.names={self.model.names}"
                )
        except json.JSONDecodeError:
            self.get_logger().error("Invalid command format. Expected JSON.")

    def rgb_callback(self, msg):
        received_ns = self.get_clock().now().nanoseconds
        self.frame_pairs.add_rgb(msg, received_ns)
        if not self.rgb_received:
            self.rgb_received = True
            self.get_logger().info("First RGB image received!")

    def depth_callback(self, msg):
        received_ns = self.get_clock().now().nanoseconds
        self.frame_pairs.add_depth(msg, received_ns)
        self.last_depth_received_at = time.monotonic()
        if not self.depth_received:
            self.depth_received = True
            self.get_logger().info("First Depth image received!")

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def ensure_aligned_depth(self):
        """bringupを書き換えず、必要時だけ実行中カメラの整列を有効化。"""
        now = time.monotonic()
        if self.alignment_future is not None:
            if self.alignment_future.done():
                try:
                    response = self.alignment_future.result()
                    succeeded = (
                        response
                        and response.results
                        and all(result.successful for result in response.results)
                    )
                    if succeeded:
                        self.get_logger().info(
                            'Camera depth alignment enabled; '
                            'waiting for aligned frames.'
                        )
                    else:
                        reasons = (
                            [result.reason for result in response.results]
                            if response
                            else ['no response']
                        )
                        self.get_logger().error(
                            f'Cannot enable depth alignment: {reasons}'
                        )
                except Exception as error:
                    self.get_logger().error(
                        f'Depth alignment request failed: {error}'
                    )
                self.alignment_future = None
            elif now - self.alignment_requested_at > ALIGNMENT_REQUEST_TIMEOUT_SEC:
                self.alignment_client.remove_pending_request(self.alignment_future)
                self.alignment_future.cancel()
                self.alignment_future = None
                self.get_logger().error('Depth alignment parameter request timed out.')
            return
        if now - self.last_depth_received_at < ALIGNED_DEPTH_SILENCE_SEC:
            return
        if self.alignment_attempts >= MAX_ALIGNMENT_ATTEMPTS:
            self.get_logger().error(
                'Aligned depth unavailable. Check camera align_depth.enable; '
                'raw depth will not be used.',
                throttle_duration_sec=10.0,
            )
            return
        if not self.alignment_client.service_is_ready():
            self.get_logger().warn(
                f'Waiting for camera set_parameters service {CAMERA_PARAMETER_SERVICE}.',
                throttle_duration_sec=10.0)
            return
        request = SetParameters.Request()
        request.parameters = [Parameter(
            name='align_depth.enable',
            value=ParameterValue(
                type=ParameterType.PARAMETER_BOOL,
                bool_value=True,
            ),
        )]
        self.alignment_attempts += 1
        self.alignment_requested_at = now
        self.alignment_future = self.alignment_client.call_async(request)

    def _estimate_depth(self, cv_depth, x1, y1, x2, y2, u_center, v_center):
        h, w = cv_depth.shape[:2]
        u_center = max(0, min(u_center, w - 1))
        v_center = max(0, min(v_center, h - 1))

        center_z = float(cv_depth[v_center, u_center])
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

        roi_m = roi.reshape(-1)
        valid = roi_m[np.isfinite(roi_m) & (roi_m > 0.1) & (roi_m < 10.0)]
        if valid.size == 0:
            return 999.0, False

        return float(np.median(valid)), True

    def _save_person_crop(self, image, bbox, prefix):
        x1, y1, x2, y2 = bbox
        crop = image[y1:y2, x1:x2].copy()
        if crop.size == 0:
            return None
        path = os.path.join(
            self.crop_dir,
            f"{prefix}_{int(time.time() * 1000)}.jpg"
        )
        if cv2.imwrite(path, crop):
            return path
        return None

    def process_frame(self):
        if not self.is_active:
            return

        try:
            pair = self.frame_pairs.take(self.get_clock().now().nanoseconds)
            if pair is None:
                self.get_logger().warn(
                    'Waiting for fresh synchronized RGB/aligned depth.',
                    throttle_duration_sec=2.0,
                )
                return
            rgb_msg, depth_msg = pair
            frame_received_ns = self.frame_pairs.last_pair_received_ns
            camera_model = camera_from_info(self.camera_info, rgb_msg)
            if camera_model is None:
                self.get_logger().warn(
                    'CameraInfo unavailable: object grasp will use fixed '
                    'intrinsics.',
                    throttle_duration_sec=10.0,
                )
            cv_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            cv_depth = depth_in_meters(
                self.bridge.imgmsg_to_cv2(depth_msg, "passthrough"),
                depth_msg.encoding,
            )
        except Exception as error:
            self.get_logger().error(
                f'RGBD frame rejected: {error}',
                throttle_duration_sec=2.0,
            )
            return

        # Trackerが検出を落とした場合だけ通常推論へフォールバックする。
        results = self.model.track(
            cv_rgb,
            device=self.inference_device,
            verbose=False,
            persist=True,
            tracker="bytetrack.yaml",
        )

        # Fallback: if tracker dropped all boxes, try predict directly
        if len(results) > 0 and len(results[0].boxes) == 0:
            pred_res = self.model.predict(
                cv_rgb, device=self.inference_device, verbose=False
            )
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
                    'track_id': track_id,
                    'rgb_stamp_ns': stamp_ns(rgb_msg),
                    'depth_stamp_ns': stamp_ns(depth_msg),
                    'frame_received_ns': frame_received_ns,
                    'result_stamp_ns': self.get_clock().now().nanoseconds,
                    'depth_aligned': True,
                    'image_width': int(rgb_msg.width),
                    'image_height': int(rgb_msg.height),
                    'camera_frame': rgb_msg.header.frame_id,
                    'camera_model': camera_model
                }
                if self.save_crops and label == "person":
                    crop_path = self._save_person_crop(
                        cv_rgb, (x1, y1, x2, y2), f"person_{len(detections)}")
                    if crop_path:
                        det_info['crop_path'] = crop_path

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
                    crop_path = self._save_person_crop(
                        cv_rgb, (x1, y1, x2, y2), "feature_ref")
                    if crop_path:
                        self.feature_crop_path = crop_path
                        detections[closest_det_idx]['feature_crop_path'] = crop_path

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
                            self.online_features_cache = {
                                "error": "API_FAILED",
                                "mode": self.feature_mode,
                                "message": str(e),
                                "extractor": (
                                    "online_openai"
                                    if self.feature_mode == "online"
                                    else "local_vlm"
                                ),
                            }
                        finally:
                            self.api_fetching = False

                    import threading
                    threading.Thread(target=fetch).start()

                if self.online_features_cache is not None:
                    features = self.online_features_cache


            if features is not None:
                detections[closest_det_idx]['features'] = features
                if self.feature_crop_path:
                    detections[closest_det_idx]['feature_crop_path'] = self.feature_crop_path

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
