"""ROSや実機を使わず、復帰順序と音声による対象選択を確認する。"""

import importlib.util
import os
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import MagicMock, call, patch


def module(name, **attributes):
    result = ModuleType(name)
    result.__dict__.update(attributes)
    return result


class ObjectGraspTests(unittest.TestCase):
    def setUp(self):
        self.node = MagicMock()
        self.base_arm = type("ArmControl", (), {"_send_move_group_goal": MagicMock(return_value=True)})
        self.ros = module(
            "rclpy", init=MagicMock(), shutdown=MagicMock(),
            ok=lambda: True, spin_once=MagicMock(),
        )
        self.modules = {
            "rclpy": self.ros,
            "rclpy.node": module("rclpy.node", Node=MagicMock(return_value=self.node)),
            "erasers_g1_api": module("erasers_g1_api"),
            "erasers_g1_api.robot_control": module(
                "erasers_g1_api.robot_control", ArmControl=self.base_arm, G1Control=MagicMock()
            ),
            "object_grasp": module(
                "object_grasp",
                CameraTransform=lambda **values: SimpleNamespace(**values),
                YoloObjectDetector=MagicMock(),
                calculate_object_grasp_plan=MagicMock(),
                joint_limit_violations=lambda joints: [],
                joint_margin_violations=lambda joints: [],
            ),
        }
        spec = importlib.util.spec_from_file_location(
            "grasp_under_test", Path(__file__).with_name("run_object_grasp.py")
        )
        self.app = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, self.modules), patch.dict(os.environ):
            spec.loader.exec_module(self.app)
        self.control_class = self.app.GraspArmControl
        self.app.GraspArmControl = MagicMock()
        self.real_pause = self.app.pause_after_motion
        self.app.pause_after_motion = MagicMock(return_value=True)

    def make_arm(self, outcomes=()):
        arm = MagicMock()
        current = self.app.WALK_POSE.copy()
        current["right_shoulder_pitch_joint"] = -0.6
        current["right_elbow_joint"] = 0.2
        arm.get_current_joints_pose.side_effect = lambda: current.copy()
        results = iter(outcomes)

        def move(**kwargs):
            self.assertEqual(kwargs.pop("planning_group"), "upper_body")
            self.assertTrue(kwargs.pop("wait"))
            result = next(results, True)
            if result:
                current.update(kwargs)
            return result

        arm.joint_control.side_effect = move
        arm.hand_control.return_value = True
        return arm

    def test_return_shoulder_and_elbow_in_one_goal(self):
        arm = self.make_arm()
        home = self.app.top_grasp_home_pose(
            arm.get_current_joints_pose(), "right"
        )
        self.assertTrue(
            self.app.return_to_top_home(self.node, arm, home, "right")
        )
        arm.joint_control.assert_called_once()
        goal = arm.joint_control.call_args
        for name, angle in home.items():
            self.assertEqual(goal.kwargs[name], angle)

    def test_return_failure_never_sends_another_goal(self):
        arm = self.make_arm([False])
        home = self.app.top_grasp_home_pose(
            arm.get_current_joints_pose(), "left"
        )
        self.assertFalse(
            self.app.return_to_top_home(self.node, arm, home, "left")
        )
        self.assertEqual(arm.joint_control.call_count, 1)

    def test_missing_feedback_never_sends_return_goal(self):
        arm = MagicMock()
        arm.get_current_joints_pose.return_value = {"right_elbow_joint": 0.2}
        self.assertFalse(
            self.app.return_to_top_home(self.node, arm, {}, "right")
        )
        arm.joint_control.assert_not_called()

    def test_hand_defaults_to_real_and_modes_remain_available(self):
        with patch.object(sys, "argv", ["run_object_grasp.py"]):
            args = self.app.parse_args()
            self.assertEqual(args.hand_mode, "real")
            self.assertFalse(args.plan_only)
        arm = self.make_arm()
        hand = self.app.OptionalHandController(self.node, arm)
        hand.command("open")
        hand.command("close")
        self.assertEqual(
            [c.kwargs for c in arm.hand_control.call_args_list],
            [{"command": "open", "hand": "right"}, {"command": "close", "hand": "right"}],
        )
        arm.hand_control.reset_mock()
        for mode in ("dummy", "disabled"):
            self.assertTrue(self.app.OptionalHandController(self.node, arm, mode).command("open"))
        arm.hand_control.assert_not_called()

    def setup_main(self, outcomes=(), detected=True):
        arm = self.make_arm(outcomes)
        self.app.GraspArmControl.return_value = arm
        self.app.parse_args = lambda: SimpleNamespace(
            target="orange",
            hand_mode="real",
            plan_only=False,
        )
        self.app.wait_for_arm_ready = lambda *args: self.app.WALK_POSE.copy()
        self.app.wait_for_head_tilt_feedback = lambda *args: None
        self.app.print_grasp_plan = MagicMock()
        detector = self.app.YoloObjectDetector.return_value
        detector.wait_until_ready.return_value = True
        detector.detect.return_value = (
            SimpleNamespace(camera_frame="d455_color_optical_frame")
            if detected else None
        )
        detector.last_failure_reason = "Detection unavailable"
        self.app.wait_for_camera_transform = MagicMock(
            return_value=SimpleNamespace()
        )
        pose = arm.get_current_joints_pose()
        self.app.calculate_object_grasp_plan.side_effect = lambda *args, **kwargs: SimpleNamespace(
            arm=kwargs["arm"], joints=pose, reachable=True, reason="",
            arm_distance_m=(0.30 if kwargs["arm"] == "right" else 0.35),
            shoulder_distance_at_zero_m=(
                0.30 if kwargs["arm"] == "right" else 0.40
            ),
            robot_xyz=(
                0.4,
                -0.15 if kwargs["arm"] == "right" else 0.15,
                kwargs["offset_z_m"],
            ),
            kinematics_source="simplified_2link",
        )
        return arm

    def test_main_normal_return_and_hand_order(self):
        arm = self.setup_main()
        self.app.recognize_target_name = MagicMock()
        self.app.main()
        self.app.recognize_target_name.assert_not_called()
        events = [c[0] for c in arm.method_calls if c[0] in ("joint_control", "hand_control")]
        self.assertEqual(events, [
            "hand_control", "joint_control", "hand_control", "joint_control",
            "joint_control", "hand_control", "joint_control", "joint_control",
            "joint_control",
        ])
        goals = [c.kwargs for c in arm.joint_control.call_args_list]
        self.assertEqual(
            goals[0]["right_wrist_roll_joint"],
            self.app.WALK_POSE["right_wrist_roll_joint"],
        )
        self.assertEqual(
            goals[4]["right_elbow_joint"],
            self.app.TOP_GRASP_ELBOW_RAD,
        )
        self.assertEqual(self.app.pause_after_motion.call_count, 9)
        self.assertEqual(
            [c.kwargs for c in arm.hand_control.call_args_list],
            [
                {"command": "close", "hand": "both"},
                {"command": "open", "hand": "right"},
                {"command": "close", "hand": "right"},
            ],
        )
        arm.solve_grasp_ik.assert_not_called()

    def test_left_arm_is_selected_only_when_right_is_unreachable(self):
        arm = self.setup_main()
        pose = arm.get_current_joints_pose()
        self.app.calculate_object_grasp_plan.side_effect = lambda *args, **kwargs: SimpleNamespace(
            arm=kwargs["arm"], joints=pose,
            reachable=(kwargs["arm"] == "left"),
            reason=("" if kwargs["arm"] == "left" else "right unreachable"),
            arm_distance_m=(0.25 if kwargs["arm"] == "left" else 0.35),
            shoulder_distance_at_zero_m=(
                0.25 if kwargs["arm"] == "left" else 0.45
            ),
            robot_xyz=(
                0.4,
                0.15 if kwargs["arm"] == "left" else -0.15,
                kwargs["offset_z_m"],
            ),
            kinematics_source="simplified_2link",
        )

        self.app.main()

        hand_calls = [c.kwargs for c in arm.hand_control.call_args_list]
        self.assertEqual(hand_calls, [
            {"command": "close", "hand": "both"},
            {"command": "open", "hand": "left"},
            {"command": "close", "hand": "left"},
        ])
        first_goal = arm.joint_control.call_args_list[0].kwargs
        self.assertEqual(
            first_goal["left_elbow_joint"],
            self.app.TOP_GRASP_ELBOW_RAD,
        )
        self.assertEqual(
            first_goal["left_wrist_roll_joint"],
            self.app.WALK_POSE["left_wrist_roll_joint"],
        )
        self.assertEqual(
            first_goal["right_elbow_joint"],
            self.app.WALK_POSE["right_elbow_joint"],
        )
        arm.solve_grasp_ik.assert_not_called()

    def test_main_return_failure_does_not_retry_or_restore_initial_pose(self):
        arm = self.setup_main([True, True, True, True, False])
        self.app.main()
        self.assertEqual(arm.joint_control.call_count, 5)
        self.assertEqual(
            arm.enable_upper_body_control.call_args_list,
            [call(True)],
        )

    def test_main_return_exception_does_not_restore_initial_pose(self):
        arm = self.setup_main()
        self.app.return_to_top_home = MagicMock(side_effect=RuntimeError("return failed"))
        with self.assertRaisesRegex(RuntimeError, "return failed"):
            self.app.main()
        self.assertEqual(arm.joint_control.call_count, 4)
        self.app.return_to_top_home.assert_called_once()
        self.node.destroy_node.assert_called_once()

    def test_lift_failure_never_drags_hand_back_to_home(self):
        arm = self.setup_main([True, True, True, False])

        self.app.main()

        self.assertEqual(arm.joint_control.call_count, 4)
        self.assertEqual(
            arm.enable_upper_body_control.call_args_list,
            [call(True)],
        )
        arm.hand_control.assert_any_call(command="close", hand="right")

    def test_both_simplified_unreachable_use_legacy_clamped_right(self):
        arm = self.setup_main()
        pose = arm.get_current_joints_pose()
        self.app.calculate_object_grasp_plan.side_effect = lambda *args, **kwargs: SimpleNamespace(
            arm=kwargs["arm"], joints=pose,
            reachable=False, reason="target is too far",
            arm_distance_m=0.6,
            shoulder_distance_at_zero_m=(
                0.3 if kwargs["arm"] == "right" else 0.4
            ),
            robot_xyz=(0.4, -0.15, kwargs["offset_z_m"]),
            kinematics_source="simplified_2link",
        )

        self.app.main()

        arm.solve_grasp_ik.assert_not_called()
        self.assertEqual(arm.joint_control.call_count, 6)
        arm.hand_control.assert_any_call(command="open", hand="right")

    def test_right_arm_is_preferred_by_default(self):
        arm = self.setup_main()

        self.app.main()

        self.assertEqual(
            [c.kwargs for c in arm.hand_control.call_args_list],
            [
                {"command": "close", "hand": "both"},
                {"command": "open", "hand": "right"},
                {"command": "close", "hand": "right"},
            ],
        )

    def test_hard_limit_failure_falls_back_to_left_arm(self):
        arm = self.setup_main()
        self.app.joint_limit_violations = lambda joints: (
            ["right joint outside URDF limit"]
            if "right_elbow_joint" in joints else []
        )

        self.app.main()

        self.assertEqual(
            [c.kwargs for c in arm.hand_control.call_args_list],
            [
                {"command": "close", "hand": "both"},
                {"command": "open", "hand": "left"},
                {"command": "close", "hand": "left"},
            ],
        )

    def test_missing_camera_tf_stops_before_object_approach(self):
        arm = self.setup_main()
        self.app.wait_for_camera_transform.return_value = None

        self.app.main()

        self.assertEqual(arm.joint_control.call_count, 0)
        self.app.calculate_object_grasp_plan.assert_not_called()

    def test_home_joint_margin_violation_stops_before_arm_motion(self):
        arm = self.setup_main()
        self.app.joint_margin_violations = MagicMock(
            return_value=["left_wrist_roll_joint near limit"]
        )

        self.app.main()

        arm.joint_control.assert_not_called()
        arm.hand_control.assert_called_once_with(command="close", hand="both")

    def test_finally_return_uses_same_order(self):
        arm = self.setup_main([True, False, True])
        self.app.main()
        # selected HOME、pre-grasp失敗、finallyのselected HOME、初期姿勢。
        self.assertEqual(arm.joint_control.call_count, 4)
        self.assertEqual(
            arm.joint_control.call_args_list[2].kwargs["right_elbow_joint"],
            self.app.TOP_GRASP_ELBOW_RAD,
        )
        arm.hand_control.assert_has_calls([
            call(command="close", hand="both"),
            call(command="open", hand="right"),
            call(command="close", hand="right"),
        ])

    def test_finally_return_failure_skips_initial_pose(self):
        arm = self.setup_main([True, False, False])
        self.app.main()
        self.assertEqual(arm.joint_control.call_count, 3)

    def voice_modules(self, responses):
        speech = MagicMock()
        responses = iter(responses)

        def execute(userdata):
            outcome, text = next(responses)
            userdata.stt_text = text
            return outcome

        speech.execute.side_effect = execute
        factory = MagicMock(return_value=speech)
        tts = MagicMock()
        mods = {
            "smach": module("smach", UserData=SimpleNamespace),
            "erasers_g1_api.tts": module("erasers_g1_api.tts", TTS=MagicMock(return_value=tts)),
            "erasers_g1_api.state_skills.recongnition": module(
                "erasers_g1_api.state_skills.recongnition", SpeechToText=factory
            ),
            "g1_srvs.srv": module("g1_srvs.srv", AudioClient=object),
            "std_srvs.srv": module("std_srvs.srv", SetBool=object),
        }
        return mods, factory, speech, tts

    def test_each_voice_object_and_english_prompt(self):
        for name in self.app.OBJECT_LIST:
            with self.subTest(name=name):
                mods, factory, speech, tts = self.voice_modules([
                    ("success", f"Please pick up the {name.upper()}.")
                ])
                with patch.dict(sys.modules, mods):
                    self.assertEqual(self.app.recognize_target_name(self.node), name)
                self.assertEqual(factory.call_args.kwargs["lang"], "en")
                self.assertIn("after the beep", factory.call_args.kwargs["start_msg"])
                tts.say.assert_called_with(f"I will grasp the {name}.")

    def test_ambiguous_and_substring_matches_retry(self):
        mods, _, speech, _ = self.voice_modules([
            ("success", "apple or bottle"),
            ("success", "pineapple"),
            ("success", "Bananas please"),
        ])
        with patch.dict(sys.modules, mods):
            self.assertEqual(self.app.recognize_target_name(self.node), "banana")
        self.assertEqual(speech.execute.call_count, 3)

    def test_voice_timeout_limit_and_failure(self):
        for responses, count in [([("timeout", "")] * 3, 3), ([("failure", "")], 1)]:
            mods, _, speech, _ = self.voice_modules(responses)
            with patch.dict(sys.modules, mods):
                self.assertIsNone(self.app.recognize_target_name(self.node))
            self.assertEqual(speech.execute.call_count, count)

    def test_voice_service_missing_returns_before_loading_model(self):
        mods, factory, _, _ = self.voice_modules([])
        self.node.create_client.return_value.wait_for_service.return_value = False
        with patch.dict(sys.modules, mods):
            self.assertIsNone(self.app.recognize_target_name(self.node))
        factory.assert_not_called()
        self.node.destroy_client.assert_called_once()

    def test_voice_failure_main_never_constructs_arm(self):
        self.app.parse_args = lambda: SimpleNamespace(
            target=None,
            hand_mode="real",
            plan_only=False,
        )
        for effect in (None, RuntimeError("speech dependency missing")):
            self.node.reset_mock()
            self.ros.shutdown.reset_mock()
            self.app.recognize_target_name = MagicMock(return_value=None)
            if effect:
                self.app.recognize_target_name.side_effect = effect
            self.app.main()
            self.app.GraspArmControl.assert_not_called()
            self.app.YoloObjectDetector.assert_not_called()
            self.node.destroy_node.assert_called_once()
            self.ros.shutdown.assert_called_once()

    def test_voice_selection_is_passed_to_detector(self):
        self.setup_main()
        self.app.parse_args = lambda: SimpleNamespace(
            target=None,
            hand_mode="real",
            plan_only=False,
        )
        self.app.recognize_target_name = MagicMock(return_value="apple")
        self.app.main()
        self.app.YoloObjectDetector.return_value.detect.assert_called_once_with(
            "apple", timeout=self.app.DETECTION_TIMEOUT_SEC
        )

    def test_plan_only_never_constructs_robot_control(self):
        self.app.parse_args = lambda: SimpleNamespace(
            target="orange",
            hand_mode="real",
            plan_only=True,
        )
        detector = self.app.YoloObjectDetector.return_value
        detector.wait_until_ready.return_value = True
        detection = SimpleNamespace(camera_frame="d455_color_optical_frame")
        detector.detect.return_value = detection
        plan = SimpleNamespace(
            to_dict=lambda: {},
            joints={"waist_yaw_joint": 0.0},
            reachable=True,
            arm_distance_m=0.3,
            shoulder_distance_at_zero_m=0.3,
        )
        self.app.calculate_object_grasp_plan.return_value = plan
        self.app.print_grasp_plan = MagicMock()
        self.app.create_tf_buffer = MagicMock(
            return_value=(MagicMock(), MagicMock())
        )
        self.app.wait_for_camera_transform = MagicMock(
            return_value=SimpleNamespace()
        )

        self.app.main()

        self.app.GraspArmControl.assert_not_called()
        self.modules["erasers_g1_api.robot_control"].G1Control.assert_not_called()
        detector.detect.assert_called_once_with(
            "orange", timeout=self.app.DETECTION_TIMEOUT_SEC
        )
        self.assertEqual(self.app.calculate_object_grasp_plan.call_count, 6)
        self.assertEqual(self.app.print_grasp_plan.call_count, 6)
        offsets = [
            call.kwargs["offset_z_m"]
            for call in self.app.calculate_object_grasp_plan.call_args_list
        ]
        expected_one_arm = [
            self.app.GRASP_OFFSET_Z_M,
            self.app.GRASP_OFFSET_Z_M,
            self.app.GRASP_OFFSET_Z_M + self.app.POST_GRASP_LIFT_M,
        ]
        self.assertEqual(offsets, expected_one_arm * 2)
        x_offsets = [
            call.kwargs["offset_x_m"]
            for call in self.app.calculate_object_grasp_plan.call_args_list
        ]
        expected_x_one_arm = [
            self.app.GRASP_OFFSET_X_M - self.app.SIDE_APPROACH_CLEARANCE_M,
            self.app.GRASP_OFFSET_X_M,
            self.app.GRASP_OFFSET_X_M,
        ]
        self.assertEqual(x_offsets, expected_x_one_arm * 2)
        wrist_rolls = [
            call.kwargs["wrist_roll"]
            for call in self.app.calculate_object_grasp_plan.call_args_list
        ]
        self.assertEqual(
            wrist_rolls,
            [self.app.SIDE_GRASP_WRIST_ROLL_RAD["right"]] * 3
            + [self.app.SIDE_GRASP_WRIST_ROLL_RAD["left"]] * 3,
        )
        self.assertEqual(
            [c.kwargs["arm"] for c in self.app.calculate_object_grasp_plan.call_args_list],
            ["right"] * 3 + ["left"] * 3,
        )
        for grasp_call in self.app.calculate_object_grasp_plan.call_args_list:
            self.assertIsNotNone(grasp_call.kwargs["camera_transform"])
        detector.stop.assert_called_once()
        self.node.destroy_node.assert_called_once()
        self.ros.shutdown.assert_called_once()

    def test_speed_limits_reach_moveit_goal_without_changing_target(self):
        arm = self.control_class()
        request = SimpleNamespace(goal_constraints=["original target"])
        goal = SimpleNamespace(request=request)
        self.assertTrue(arm._send_move_group_goal(goal, True))
        self.assertEqual(request.max_velocity_scaling_factor, 0.5)
        self.assertEqual(request.max_acceleration_scaling_factor, 0.5)
        self.assertEqual(request.goal_constraints, ["original target"])
        self.base_arm._send_move_group_goal.assert_called_once_with(goal, True)

    def test_invalid_speed_scale_cannot_fall_back_to_full_speed(self):
        for setting in ("ARM_VELOCITY_SCALE", "ARM_ACCELERATION_SCALE"):
            for value in (0.0, -0.1, 1.1, float("nan"), float("inf")):
                with self.subTest(setting=setting, value=value), patch.object(self.app, setting, value):
                    with self.assertRaises(ValueError):
                        self.control_class()._send_move_group_goal(SimpleNamespace(request=SimpleNamespace()), True)
        self.base_arm._send_move_group_goal.assert_not_called()

    def test_pause_waits_one_second_while_processing_ros_callbacks(self):
        now = [10.0]
        def spin(node, timeout_sec):
            self.assertIs(node, self.node)
            self.assertLessEqual(timeout_sec, 0.1)
            now[0] += timeout_sec
        self.ros.spin_once.side_effect = spin
        with patch.object(self.app.time, "monotonic", side_effect=lambda: now[0]):
            self.assertTrue(self.real_pause(self.node))
        self.assertAlmostEqual(now[0], 11.0)
        self.assertGreaterEqual(self.ros.spin_once.call_count, 10)

    def test_pause_exits_on_ros_shutdown(self):
        self.ros.ok = lambda: False
        self.assertFalse(self.real_pause(self.node))
        self.ros.spin_once.assert_not_called()

    def test_failed_motion_does_not_add_success_pause(self):
        arm = self.make_arm([False])
        home = self.app.top_grasp_home_pose(
            arm.get_current_joints_pose(), "right"
        )
        self.assertFalse(self.app.move_arm_joints(self.node, arm, home))
        arm.hand_control.return_value = False
        self.assertFalse(self.app.OptionalHandController(self.node, arm).command("close"))
        self.app.pause_after_motion.assert_not_called()

    def test_head_settle_is_followed_by_pause(self):
        arm, robot = MagicMock(), MagicMock()
        arm.get_current_joints_pose.return_value = {"d455_joint": 0.2}
        self.assertTrue(self.app.move_head_tilt_and_wait(self.node, robot, arm, 0.2))
        self.app.pause_after_motion.assert_called_once_with(self.node)


if __name__ == "__main__":
    unittest.main()
