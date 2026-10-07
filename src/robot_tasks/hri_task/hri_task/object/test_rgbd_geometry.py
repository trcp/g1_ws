"""実画像・ROS通信・ロボット動作なしでRGBDと安定判定を検証する。"""

import ast
import importlib.util
import json
import math
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import MagicMock, patch

import numpy as np

from rgbd_geometry import FramePairs, camera_from_info, deproject, depth_in_meters, stamp_ns
from check_rgbd_geometry import projection_rows, validate_rows


def image(t, width=640, frame="color_optical"):
    ns = round(t * 1e9)
    return NS(width=width, height=480, header=NS(
        frame_id=frame, stamp=NS(sec=ns // 10**9, nanosec=ns % 10**9)))


def calibration(fx=500):
    return dict(width=640, height=480, fx=fx, fy=500., cx=320., cy=240.,
                d=[], distortion_model="plumb_bob", frame_id="color_optical", source="camera_info")


def load_object_module():
    ros = ModuleType("rclpy")
    ros.ok = lambda: True
    spec = importlib.util.spec_from_file_location("object_geometry_test", Path(__file__).with_name("object_grasp.py"))
    app = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, {
        "rclpy": ros, "rclpy.node": NS(Node=object),
        "std_msgs.msg": NS(String=NS), spec.name: app,
    }):
        spec.loader.exec_module(app)
    return app, ros


class RGBDTests(unittest.TestCase):
    def test_depth_units_are_encoding_based(self):
        np.testing.assert_allclose(depth_in_meters([50, 500], "16UC1"), [.05, .5])
        np.testing.assert_allclose(depth_in_meters([.5, 101], "32FC1"), [.5, 101])
        with self.assertRaises(ValueError):
            depth_in_meters([1], "8UC1")

    def test_pair_is_synchronized_and_never_reused(self):
        pairs = FramePairs()
        pairs.add_rgb(image(10), 10_150_000_000)
        pairs.add_rgb(image(10.1), 10_150_000_000)
        pairs.add_depth(image(10.11), 10_150_000_000)
        self.assertIsNotNone(pairs.take(10_200_000_000))
        self.assertIsNone(pairs.take(10_200_000_000))
        pairs.add_rgb(image(10.12), 10_200_000_000)
        self.assertIsNone(pairs.take(10_200_000_000))

    def test_stale_future_and_unsynchronized_frames_rejected(self):
        cases = [
            (1, 1, 1),
            (11, 11, 11),
            (10, 10.1, 10),
        ]
        for rgb, depth, received in cases:
            pairs = FramePairs()
            pairs.add_rgb(image(rgb), round(received * 1e9))
            pairs.add_depth(image(depth), round(received * 1e9))
            self.assertIsNone(pairs.take(10_000_000_000))

    def test_pair_dimensions_and_frames_must_match(self):
        for bad_depth in [image(10, width=320), image(10, frame="depth_optical")]:
            pairs = FramePairs()
            pairs.add_rgb(image(10), 10_000_000_000)
            pairs.add_depth(bad_depth, 10_000_000_000)
            with self.assertRaises(ValueError):
                pairs.take(10_000_000_000)

    def test_camera_clock_offset_does_not_make_live_frames_stale(self):
        pairs = FramePairs()
        pairs.add_rgb(image(5.0), 10_000_000_000)
        pairs.add_depth(image(5.01), 10_000_000_000)

        self.assertIsNotNone(pairs.take(10_050_000_000))
        self.assertEqual(pairs.last_pair_received_ns, 10_000_000_000)

    def test_live_intrinsics_change_off_center_projection(self):
        self.assertEqual(deproject(320, 240, .5, calibration()), (0., 0., .5))
        self.assertAlmostEqual(deproject(420, 240, .5, calibration())[0], .1)
        self.assertAlmostEqual(deproject(420, 240, .5, calibration(1000))[0], .05)

    def test_distortion_round_trip(self):
        import cv2
        model = calibration()
        model["d"] = [.1, -.02, .001, .002, 0.]
        k = np.array([[500., 0, 320], [0, 500, 240], [0, 0, 1]])
        point = np.array([[[.12, -.08, .5]]])
        pixels, _ = cv2.projectPoints(point, np.zeros(3), np.zeros(3), k, np.array(model["d"]))
        np.testing.assert_allclose(deproject(*pixels[0, 0], .5, model), point[0, 0], atol=1e-7)

    def test_info_missing_and_mismatch(self):
        rgb = image(10)
        self.assertIsNone(camera_from_info(None, rgb))
        info = NS(width=640, height=480, k=[500., 0, 320, 0, 500, 240, 0, 0, 1],
                  d=[], distortion_model="plumb_bob", header=rgb.header)
        self.assertEqual(camera_from_info(info, rgb)["fx"], 500.)
        info.width = 320
        with self.assertRaises(ValueError):
            camera_from_info(info, rgb)

    def test_offline_left_center_right_check(self):
        rows = projection_rows(NS(
            width=640, height=480, fx=615., fy=615., cx=320., cy=240.,
            depth=.5, horizontal_span=160.,
        ))
        validate_rows(rows)
        self.assertLess(rows[0][3], 0.)
        self.assertEqual(rows[1][3], 0.)
        self.assertGreater(rows[2][3], 0.)


class StabilityTests(unittest.TestCase):
    def setUp(self):
        self.app, self.ros = load_object_module()

    def detection(self, t, shift=0, depth=.5, track=1, model=None):
        return self.app.ObjectDetection(
            "orange", .9, [300+shift, 220, 340+shift, 260], depth, True,
            camera_model=model, camera_frame="color_optical", rgb_stamp_ns=round(t*1e9),
            depth_stamp_ns=round(t*1e9), frame_received_ns=round(t*1e9),
            result_stamp_ns=round(t*1e9), track_id=track)

    def test_fixed_intrinsics_remain_and_size_mismatch_is_rejected(self):
        d = self.detection(1)
        self.assertEqual(self.app.detection_camera(d)["fx"], 615.)
        self.assertEqual(self.app.detection_camera(d)["source"], "fixed")
        d.image_width = 1280
        with self.assertRaises(ValueError):
            self.app.detection_camera(d)

    def test_stability_requires_five_frames_and_half_second(self):
        stable = self.app.StableDetections()
        for i in range(4):
            self.assertIsNone(stable.add(self.detection(1+i*.125)))
        result = stable.add(self.detection(1.5, depth=.51))
        self.assertIsNotNone(result)
        self.assertEqual(result.depth_m, .5)

    def test_five_fast_frames_do_not_satisfy_duration(self):
        stable = self.app.StableDetections()
        for i in range(5):
            self.assertIsNone(stable.add(self.detection(1+i*.01)))

    def test_duplicate_frames_cannot_fake_stability(self):
        stable = self.app.StableDetections()
        for _ in range(10):
            self.assertIsNone(stable.add(self.detection(1)))
        self.assertEqual(len(stable.samples), 1)

    def test_motion_and_depth_jumps_do_not_stabilize(self):
        for moving in (True, False):
            stable = self.app.StableDetections()
            for i in range(15):
                d = self.detection(1+i*.125, shift=(i*10 if moving else 0),
                                   depth=(.5 if moving else .5+(i % 2)*.05))
                self.assertIsNone(stable.add(d))

    def test_target_change_gap_and_calibration_change_reset_window(self):
        for change in [dict(track=2), dict(t=2), dict(model=calibration())]:
            stable = self.app.StableDetections()
            for i in range(4):
                stable.add(self.detection(1+i*.125))
            values = dict(t=1.5)
            values.update(change)
            self.assertIsNone(stable.add(self.detection(**values)))
            self.assertEqual(len(stable.samples), 1)

    def test_grasp_plan_uses_live_intrinsics_but_preserves_fixed_fallback(self):
        d = self.detection(1, shift=100)
        fixed = self.app.calculate_object_grasp_plan(d)
        d.camera_model = calibration()
        live = self.app.calculate_object_grasp_plan(d)
        self.assertAlmostEqual(fixed.camera_xyz[0], .5*100/615)
        self.assertAlmostEqual(live.camera_xyz[0], .1)

    def test_camera_tf_replaces_manual_camera_offsets(self):
        transform = self.app.CameraTransform(
            target_frame="base_link",
            source_frame="color_optical",
            translation=(0.1, 0.2, 0.3),
            rotation_xyzw=(0.0, 0.0, 0.0, 1.0),
        )
        plan = self.app.calculate_object_grasp_plan(
            self.detection(1),
            head_tilt=-1.0,
            camera_transform=transform,
        )

        self.assertEqual(plan.camera_xyz, (0.0, 0.0, 0.5))
        self.assertEqual(plan.robot_xyz, (0.1, 0.2, 0.8))
        self.assertEqual(plan.coordinate_source, "tf:base_link<-color_optical")

    def test_camera_tf_applies_quaternion_rotation(self):
        half_sqrt = math.sqrt(0.5)
        transform = self.app.CameraTransform(
            target_frame="base_link",
            source_frame="color_optical",
            translation=(1.0, 2.0, 3.0),
            rotation_xyzw=(0.0, 0.0, half_sqrt, half_sqrt),
        )

        result = self.app.transform_camera_point((1.0, 0.0, 0.0), transform)

        for actual, expected in zip(result, (1.0, 3.0, 3.0)):
            self.assertAlmostEqual(actual, expected)

    def test_downward_camera_tilt_lowers_forward_optical_point(self):
        # 水平時の optical(X=右,Y=下,Z=前)→base(X=前,Y=左,Z=上)。
        optical_to_base = (-0.5, 0.5, -0.5, 0.5)
        tilt = math.radians(7.0)
        down_about_base_y = (0.0, math.sin(tilt / 2.0), 0.0,
                             math.cos(tilt / 2.0))

        def multiply(left, right):
            lx, ly, lz, lw = left
            rx, ry, rz, rw = right
            return (
                lw * rx + lx * rw + ly * rz - lz * ry,
                lw * ry - lx * rz + ly * rw + lz * rx,
                lw * rz + lx * ry - ly * rx + lz * rw,
                lw * rw - lx * rx - ly * ry - lz * rz,
            )

        transform = self.app.CameraTransform(
            target_frame="base_link",
            source_frame="d455_color_optical_frame",
            translation=(0.0, 0.0, 0.0),
            rotation_xyzw=multiply(down_about_base_y, optical_to_base),
        )

        x, y, z = self.app.transform_camera_point((0.0, 0.0, 0.5), transform)

        self.assertAlmostEqual(x, 0.5 * math.cos(tilt))
        self.assertAlmostEqual(y, 0.0)
        self.assertAlmostEqual(z, -0.5 * math.sin(tilt))

    def test_left_arm_uses_mirrored_roll_wrist_and_shoulder(self):
        transform = self.app.CameraTransform(
            target_frame="base_link",
            source_frame="color_optical",
            translation=(0.25, 0.0, 0.2),
            rotation_xyzw=(0.0, 0.0, 0.0, 1.0),
        )
        detection = self.detection(1)
        left = self.app.calculate_object_grasp_plan(
            detection,
            arm="left",
            wrist_roll=1.2,
            camera_transform=transform,
        )
        right = self.app.calculate_object_grasp_plan(
            detection,
            arm="right",
            wrist_roll=-1.2,
            camera_transform=transform,
        )

        self.assertEqual(left.arm, "left")
        self.assertIn("left_elbow_joint", left.joints)
        self.assertNotIn("right_elbow_joint", left.joints)
        self.assertEqual(left.joints["left_shoulder_roll_joint"], 0.2)
        self.assertEqual(right.joints["right_shoulder_roll_joint"], -0.2)
        self.assertEqual(left.joints["left_wrist_roll_joint"], 1.2)
        self.assertEqual(right.joints["right_wrist_roll_joint"], -1.2)
        self.assertAlmostEqual(
            left.joints["waist_yaw_joint"],
            -right.joints["waist_yaw_joint"],
        )

    def test_joint_limit_margin_rejects_near_limit_target(self):
        violations = self.app.joint_margin_violations({
            "left_wrist_roll_joint": 1.90,
            "right_wrist_roll_joint": 0.0,
        })

        self.assertEqual(len(violations), 1)
        self.assertIn("left_wrist_roll_joint", violations[0])

    def test_hard_joint_limits_accept_values_inside_urdf_range(self):
        self.assertEqual(
            self.app.joint_limit_violations({
                "right_wrist_roll_joint": -1.899,
                "left_wrist_roll_joint": 1.899,
            }),
            [],
        )

    def test_object_side_is_closer_to_same_side_shoulder(self):
        detection = self.detection(1)
        for target_y, expected_arm in ((0.25, "left"), (-0.25, "right")):
            transform = self.app.CameraTransform(
                target_frame="base_link",
                source_frame="color_optical",
                translation=(0.2, target_y, -0.3),
                rotation_xyzw=(0.0, 0.0, 0.0, 1.0),
            )
            plans = {
                arm: self.app.calculate_object_grasp_plan(
                    detection,
                    arm=arm,
                    camera_transform=transform,
                )
                for arm in ("left", "right")
            }

            other_arm = "right" if expected_arm == "left" else "left"
            self.assertLess(
                plans[expected_arm].shoulder_distance_at_zero_m,
                plans[other_arm].shoulder_distance_at_zero_m,
            )

    def run_detector(self, messages):
        node = MagicMock()
        current = [10.]
        node.get_clock().now.side_effect = lambda: NS(nanoseconds=round(current[0]*1e9))
        detector = self.app.YoloObjectDetector(node)
        detector.wait_until_ready = lambda **kw: True
        messages = iter(messages)
        def spin(*args, **kwargs):
            current[0] += .125
            item = next(messages, None)
            if item is not None:
                detector._result_callback(NS(data=json.dumps(item)))
        self.ros.spin_once = spin
        with patch.object(self.app.time, "monotonic", side_effect=lambda: current[0]):
            result = detector.detect("orange", timeout=2.)
        return result, detector

    def payload(self, t):
        item = self.app.asdict(self.detection(t))
        item["distance_z"] = item.pop("depth_m")
        item["depth_aligned"] = True
        return item

    def test_detector_waits_for_fresh_stable_frames(self):
        result, _ = self.run_detector([[self.payload(10+(i+1)*.125)] for i in range(5)])
        self.assertIsNotNone(result)
        self.assertEqual(result.rgb_stamp_ns, 10_625_000_000)

    def test_detector_rejects_old_unaligned_or_invalid_metadata(self):
        for changes in [dict(frame_received_ns=1), dict(result_stamp_ns=1),
                        dict(depth_aligned=False), dict(image_width=1280),
                        dict(depth_stamp_ns=0), dict(distance_z=float("nan"))]:
            frames = []
            for i in range(10):
                item = self.payload(10+(i+1)*.125)
                item.update(changes)
                frames.append([item])
            result, detector = self.run_detector(frames)
            self.assertIsNone(result)
            self.assertIn("invalid", detector.last_failure_reason)

    def test_edge_clipped_detection_is_kept_for_grasp(self):
        frames = []
        for i in range(10):
            item = self.payload(10+(i+1)*.125)
            item["bbox"] = [479.0, 448.0, 558.5, 479.0]
            frames.append([item])

        result, detector = self.run_detector(frames)

        self.assertIsNotNone(result)
        self.assertEqual(result.bbox, [479.0, 448.0, 558.5, 479.0])
        self.assertEqual(detector.last_failure_reason, "")

    def test_detector_accepts_fixed_camera_clock_offset(self):
        frames = []
        for i in range(5):
            item = self.payload(10+(i+1)*.125)
            item["rgb_stamp_ns"] -= 5_000_000_000
            item["depth_stamp_ns"] -= 5_000_000_000
            frames.append([item])

        result, _ = self.run_detector(frames)

        self.assertIsNotNone(result)

    def test_detector_unstable_is_not_reported_as_object_absent(self):
        frames = []
        for i in range(12):
            item = self.payload(10+(i+1)*.125)
            item["distance_z"] = .5+(i % 2)*.05
            frames.append([item])
        result, detector = self.run_detector(frames)
        self.assertIsNone(result)
        self.assertIn("did not remain stable", detector.last_failure_reason)


class AlignmentRequestTests(unittest.TestCase):
    def setUp(self):
        # YOLOモデルのロードやコンストラクタのROS通信は実行しない。
        path = Path(__file__).parent.parent / 'yolo_human/yolo_human_node.py'
        cls = next(n for n in ast.parse(path.read_text()).body if isinstance(n, ast.ClassDef) and n.name == 'YoloHumanNode')
        namespace = dict(Node=object, time=NS(monotonic=lambda: 100.),
                         SetParameters=NS(Request=NS), Parameter=NS, ParameterValue=NS,
                         ParameterType=NS(PARAMETER_BOOL=1),
                         FramePairs=FramePairs, camera_from_info=camera_from_info,
                         depth_in_meters=depth_in_meters, stamp_ns=stamp_ns,
                         np=np, math=math, json=json, String=NS,
                         CompressedImage=lambda: NS(header=NS()), ENABLE_IMSHOW=False,
                         ALIGNMENT_REQUEST_TIMEOUT_SEC=5.0,
                         ALIGNED_DEPTH_SILENCE_SEC=3.0,
                         MAX_ALIGNMENT_ATTEMPTS=3,
                         CAMERA_PARAMETER_SERVICE='/head_camera/d455/set_parameters')
        import cv2
        namespace['cv2'] = cv2
        exec(compile(ast.Module(body=[cls], type_ignores=[]), str(path), 'exec'), namespace)
        self.node = namespace['YoloHumanNode'].__new__(namespace['YoloHumanNode'])
        self.node.alignment_future = None
        self.node.alignment_requested_at = 0.
        self.node.last_depth_received_at = 0.
        self.node.alignment_attempts = 0
        self.node.alignment_client = MagicMock()
        self.node.get_logger = MagicMock()
        self.node.inference_device = "cpu"

    def test_runtime_alignment_request_without_bringup_change(self):
        self.node.ensure_aligned_depth()
        request = self.node.alignment_client.call_async.call_args.args[0]
        self.assertEqual(request.parameters[0].name, 'align_depth.enable')
        self.assertTrue(request.parameters[0].value.bool_value)

    def test_existing_aligned_frames_need_no_request(self):
        self.node.last_depth_received_at = 99.
        self.node.ensure_aligned_depth()
        self.node.alignment_client.call_async.assert_not_called()

    def test_request_failure_and_timeout_are_reported(self):
        self.node.alignment_future = MagicMock()
        self.node.alignment_future.result.return_value = NS(results=[NS(successful=False, reason='read only')])
        self.node.ensure_aligned_depth()
        self.node.get_logger().error.assert_called()
        self.assertIsNone(self.node.alignment_future)
        future = MagicMock()
        future.done.return_value = False
        self.node.alignment_future = future
        self.node.ensure_aligned_depth()
        future.cancel.assert_called_once()

    def test_yolo_frame_contains_matched_metadata_and_meter_depth(self):
        node = self.node
        node.is_active = True
        node.frame_pairs = FramePairs()
        rgb, depth = image(10), image(10.01)
        depth.encoding = '16UC1'
        node.frame_pairs.add_rgb(rgb, 10_020_000_000)
        node.frame_pairs.add_depth(depth, 10_020_000_000)
        node.get_clock = MagicMock()
        node.get_clock().now.return_value = NS(nanoseconds=10_020_000_000, to_msg=lambda: NS())
        node.camera_info = NS(width=640, height=480,
                              k=[500., 0, 320, 0, 500, 240, 0, 0, 1],
                              d=[], distortion_model='plumb_bob', header=rgb.header)
        node.bridge = MagicMock()
        node.bridge.imgmsg_to_cv2.side_effect = lambda msg, _: (
            np.zeros((480, 640, 3), dtype=np.uint8) if msg is rgb
            else np.full((480, 640), 500, dtype=np.uint16))
        node.model = MagicMock()
        node.model.names = {0: 'orange'}
        node.model.track.return_value = [NS(boxes=[NS(
            cls=[0], conf=[.9], xyxy=[[300, 220, 340, 260]], id=[7])])]
        node.target_classes = ['orange']
        node.save_crops = False
        node.extract_features = False
        node.result_pub = MagicMock()
        node.debug_pub = MagicMock()
        node.process_frame()
        result = json.loads(node.result_pub.publish.call_args.args[0].data)[0]
        self.assertEqual(result['distance_z'], .5)
        self.assertTrue(result['depth_aligned'])
        self.assertEqual(result['rgb_stamp_ns'], 10_000_000_000)
        self.assertEqual(result['depth_stamp_ns'], 10_010_000_000)
        self.assertEqual(result['frame_received_ns'], 10_020_000_000)
        self.assertEqual(result['result_stamp_ns'], 10_020_000_000)
        self.assertEqual(result['camera_model']['fx'], 500.)
        self.assertEqual(result['track_id'], 7)
        node.process_frame()
        node.result_pub.publish.assert_called_once()


if __name__ == '__main__':
    unittest.main()
