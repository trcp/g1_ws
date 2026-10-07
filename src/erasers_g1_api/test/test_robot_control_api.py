# Copyright (c) 2026 Nakatogawa Laboratory. All rights reserved.

import unittest
from unittest.mock import MagicMock, patch
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from erasers_g1_interfaces.srv import RobotPose, RobotServiceClient, MoveServo
from erasers_g1_api.robot_control import (
    G1Control,
    HeadControl,
    ArmControl,
    ArmCollision,
    ArmGrasp,
    NavControl,
    G1Navigation,
    Collision,
    Grasp,
)


class TestG1ControlApi(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = Node("test_g1_control_api_node")

    def tearDown(self):
        self.node.destroy_node()

    def test_g1_control_safety_guard_blocks_dangerous_transition(self):
        # サービスの待機をモックして初期化
        with patch.object(rclpy.client.Client, "wait_for_service", return_value=True):
            ctrl = G1Control(self.node, timeout_sec=1.0)

            # current_fsm = 801 (RUNNING) をモック
            with patch.object(ctrl, "get_current_robot_pose", return_value=801):
                # mode = 0 (ZERO_TORQUE) はブロックされて False になること
                result = ctrl.robot_pose(mode=0, safety=True)
                self.assertFalse(result)

                # mode = 1 (DAMP) はブロックされること
                result = ctrl.robot_pose(mode=1, safety=True)
                self.assertFalse(result)

                # mode = 3 (LOCKED_SEAT) はブロックされること
                result = ctrl.robot_pose(mode=3, safety=True)
                self.assertFalse(result)

    def test_g1_control_safety_guard_disabled_allows_call(self):
        with patch.object(rclpy.client.Client, "wait_for_service", return_value=True):
            ctrl = G1Control(self.node, timeout_sec=1.0)

            with patch.object(ctrl, "get_current_robot_pose", return_value=801):
                with patch.object(ctrl, "_G1Control__send_robot_pose_req", return_value=True) as mock_send:
                    result = ctrl.robot_pose(mode=0, safety=False)
                    self.assertTrue(result)
                    mock_send.assert_called_once()

    def test_g1_control_led_validation(self):
        with patch.object(rclpy.client.Client, "wait_for_service", return_value=True):
            ctrl = G1Control(self.node, timeout_sec=1.0)

            # 正常値
            self.assertTrue(ctrl.led(255, 128, 0))
            self.assertTrue(ctrl.led(0, 0, 0))

            # 範囲外値
            self.assertFalse(ctrl.led(-1, 0, 0))
            self.assertFalse(ctrl.led(0, 256, 0))
            self.assertFalse(ctrl.led(0, 0, 300))

    def test_g1_control_get_robot_service_interfaces_parsing(self):
        with patch.object(rclpy.client.Client, "wait_for_service", return_value=True):
            ctrl = G1Control(self.node, timeout_sec=1.0)

            mock_res = RobotServiceClient.Response()
            mock_res.success = True
            mock_res.message = "service_a\nservice_b\nservice_c\n"

            with patch.object(ctrl, "_G1Control__send_service_setting_req", return_value=mock_res):
                services = ctrl.get_robot_service_interfaces()
                self.assertEqual(services, ["service_a", "service_b", "service_c"])

    def test_backward_compatibility_aliases(self):
        self.assertIs(G1Navigation, NavControl)
        self.assertIs(Collision, ArmCollision)
        self.assertIs(Grasp, ArmGrasp)

    def test_arm_control_move_groupstate_and_joint_control(self):
        with patch.object(rclpy.client.Client, "wait_for_service", return_value=True), \
             patch.object(ArmControl, "_ArmControl__load_srdf_group_states", return_value={"arm_both_with_waist": {"walk": {"waist_yaw_joint": 0.0, "left_shoulder_pitch_joint": 0.1}}}):
            arm = ArmControl(self.node, timeout_sec=1.0)

            sent_goals = []
            def mock_send_goal(goal, wait=True):
                sent_goals.append((goal, wait))
                return True

            with patch.object(arm, "_ArmControl__send_move_group_goal", side_effect=mock_send_goal):
                res = arm.move_groupstate(group_name="arm_both_with_waist", state="walk", wait=True)
                self.assertTrue(res)
                self.assertEqual(len(sent_goals), 1)
                goal, wait = sent_goals[0]
                self.assertTrue(wait)
                self.assertEqual(goal.request.group_name, "arm_both_with_waist")
                self.assertEqual(len(goal.request.goal_constraints), 1)
                jc_list = goal.request.goal_constraints[0].joint_constraints
                self.assertEqual(len(jc_list), 2)
                joint_names = {jc.joint_name for jc in jc_list}
                self.assertEqual(joint_names, {"waist_yaw_joint", "left_shoulder_pitch_joint"})

    def test_arm_control_use_sim_time_true_skips_hardware_services(self):
        # use_sim_time=True では実機サービスの wait_for_service が呼ばれず即座に初期化されること
        with patch.object(rclpy.client.Client, "wait_for_service", return_value=False) as mock_w, \
             patch.object(ArmControl, "_ArmControl__load_srdf_group_states", return_value={}):
            arm = ArmControl(self.node, timeout_sec=0.5, use_sim_time=True)
            mock_w.assert_not_called()

            # サービス未稼働時でも use_sim_time 下では upper_body_control は True を返すこと
            self.assertTrue(arm.upper_body_control(True))
            self.assertTrue(arm.upper_body_control(False))

            # サービス未稼働時かつ use_sim_time 下では arm_action は False を返すこと
            self.assertFalse(arm.arm_action(1))

    def test_arm_control_inherits_node_sim_time_when_omitted(self):
        # 引数省略時にノードの時刻設定を上書きせず，自動判定する．
        self.node.set_parameters([
            rclpy.parameter.Parameter("use_sim_time", value=True)])
        with patch.object(rclpy.client.Client, "wait_for_service",
                          return_value=False) as wait_service:
            with patch.object(ArmControl, "_ArmControl__load_srdf_group_states",
                              return_value={}):
                arm = ArmControl(self.node, timeout_sec=0.1)
        wait_service.assert_not_called()
        self.assertTrue(self.node.get_parameter("use_sim_time").value)
        self.assertTrue(arm.upper_body_control(True))

    def test_arm_control_use_sim_time_false_raises_runtime_error_on_timeout(self):
        # use_sim_time=False ではサービスが未起動の場合に RuntimeError を送出すること
        with patch.object(rclpy.client.Client, "wait_for_service", return_value=False), \
             patch.object(ArmControl, "_ArmControl__load_srdf_group_states", return_value={}):
            with self.assertRaises(RuntimeError):
                ArmControl(self.node, timeout_sec=0.1, use_sim_time=False)

    def test_arm_collision_without_apply_scene_cli(self):
        # apply_scene_cli 引数なしで初期化
        collision = ArmCollision(self.node, timeout_sec=1.0)
        
        with patch.object(collision, "_ArmCollision__apply", return_value=True):
            self.assertTrue(collision.add_box("box_1", ref_frame="torso_link"))
            self.assertTrue(collision.add_cylinder("cyl_1", ref_frame="torso_link"))
            self.assertTrue(collision.add_sphere("sph_1", ref_frame="torso_link"))
            self.assertTrue(collision.remove_collision("box_1"))
            self.assertTrue(collision.remove_all_collisions())

    def test_arm_collision_attach_and_detach(self):
        collision = ArmCollision(self.node, timeout_sec=1.0)

        applied_scenes = []
        def mock_apply(scene):
            applied_scenes.append(scene)
            return True

        with patch.object(collision, "_ArmCollision__apply", side_effect=mock_apply):
            # attach
            res = collision.attach("cylinder_obj", attach_frame="left_amazing_hand")
            self.assertTrue(res)
            self.assertEqual(len(applied_scenes), 1)
            scene = applied_scenes[0]
            self.assertEqual(len(scene.robot_state.attached_collision_objects), 1)
            att = scene.robot_state.attached_collision_objects[0]
            self.assertEqual(att.link_name, "left_amazing_hand")
            self.assertEqual(att.object.id, "cylinder_obj")
            from moveit_msgs.msg import CollisionObject
            self.assertEqual(att.object.operation, CollisionObject.ADD)
            self.assertIn("left_amazing_hand", att.touch_links)
            self.assertIn("left_wrist_roll_rubber_hand", att.touch_links)

            # detach
            res = collision.detach("cylinder_obj")
            self.assertTrue(res)
            self.assertEqual(len(applied_scenes), 2)
            scene2 = applied_scenes[1]
            self.assertEqual(len(scene2.robot_state.attached_collision_objects), 1)
            att2 = scene2.robot_state.attached_collision_objects[0]
            self.assertEqual(att2.object.id, "cylinder_obj")
            self.assertEqual(att2.object.operation, CollisionObject.REMOVE)

            # aliases
            self.assertTrue(collision.attach_collision("cylinder_obj"))
            self.assertTrue(collision.detach_collision("cylinder_obj"))

            # remove_all_collisions cleans up
            self.assertTrue(collision.remove_all_collisions())

    def test_arm_collision_queries_and_acm(self):
        from moveit_msgs.msg import CollisionObject, PlanningScene, AllowedCollisionEntry
        from moveit_msgs.srv import GetPlanningScene
        from geometry_msgs.msg import Pose

        collision = ArmCollision(self.node, timeout_sec=1.0)

        co = CollisionObject()
        co.id = "target_box"
        co.header.frame_id = "torso_link"
        co.pose = Pose()
        co.pose.position.x = 0.5
        co.pose.position.y = 0.2
        co.pose.position.z = 0.1
        co.pose.orientation.w = 1.0

        mock_scene = PlanningScene()
        mock_scene.world.collision_objects.append(co)
        mock_scene.allowed_collision_matrix.entry_names = ["target_box", "table"]
        e1 = AllowedCollisionEntry()
        e1.enabled = [True, True]
        e2 = AllowedCollisionEntry()
        e2.enabled = [True, True]
        mock_scene.allowed_collision_matrix.entry_values = [e1, e2]

        mock_res = GetPlanningScene.Response()
        mock_res.scene = mock_scene

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_res

        with patch.object(collision._ArmCollision__get_scene_cli, "wait_for_service", return_value=True), \
             patch.object(collision._ArmCollision__get_scene_cli, "call_async", return_value=mock_future), \
             patch.object(rclpy, "spin_until_future_complete", return_value=None), \
             patch.object(collision, "_ArmCollision__apply", return_value=True) as mock_apply:

            # get_object
            obj = collision.get_object("target_box")
            self.assertIsNotNone(obj)
            self.assertEqual(obj.id, "target_box")

            not_obj = collision.get_object("non_existent")
            self.assertIsNone(not_obj)

            # get_object_pose
            pose = collision.get_object_pose("target_box")
            self.assertIsNotNone(pose)
            x, y, z, roll, pitch, yaw, frame_id = pose
            self.assertAlmostEqual(x, 0.5)
            self.assertAlmostEqual(y, 0.2)
            self.assertAlmostEqual(z, 0.1)
            self.assertEqual(frame_id, "torso_link")

            # allow_collision
            ok = collision.allow_collision("target_box", "table")
            self.assertTrue(ok)
            mock_apply.assert_called_once()
            applied_scene = mock_apply.call_args[0][0]
            acm = applied_scene.allowed_collision_matrix
            idx1 = acm.entry_names.index("target_box")
            idx2 = acm.entry_names.index("table")
            self.assertTrue(acm.entry_values[idx1].enabled[idx2])

    def test_arm_control_endeffector_pose_and_move_abs_orientation(self):
        from geometry_msgs.msg import TransformStamped
        from moveit_msgs.srv import GetPositionIK
        from tf2_ros import Buffer

        mock_buffer = MagicMock(spec=Buffer)
        ts = TransformStamped()
        ts.transform.translation.x = 0.3
        ts.transform.translation.y = 0.2
        ts.transform.translation.z = -0.1
        ts.transform.rotation.w = 1.0
        mock_buffer.lookup_transform.return_value = ts

        with patch.object(rclpy.client.Client, "wait_for_service", return_value=True), \
             patch.object(ArmControl, "_ArmControl__load_srdf_group_states", return_value={}):
            arm = ArmControl(self.node, timeout_sec=1.0, tf_buffer=mock_buffer)

            # get_current_endeffector_pose
            cur_pose = arm.get_current_endeffector_pose(ref="torso_link", arm_side="left")
            self.assertIsNotNone(cur_pose)
            self.assertAlmostEqual(cur_pose.translation.x, 0.3)
            self.assertAlmostEqual(cur_pose.rotation.w, 1.0)

            # move_abs with roll, pitch, yaw
            mock_ik_res = GetPositionIK.Response()
            mock_ik_res.error_code.val = 1
            mock_ik_res.solution.joint_state.name = ["left_shoulder_pitch_joint"]
            mock_ik_res.solution.joint_state.position = [0.2]

            mock_future = MagicMock()
            mock_future.done.return_value = True
            mock_future.result.return_value = mock_ik_res

            from moveit_msgs.msg import RobotState
            seed = RobotState()
            seed.joint_state.name = ["left_shoulder_pitch_joint"]
            seed.joint_state.position = [0.2]
            with patch.object(arm._ArmControl__ik_cli, "service_is_ready", return_value=True), \
                 patch.object(arm._ArmControl__ik_cli, "call_async", return_value=mock_future) as mock_ik_call, \
                 patch.object(rclpy, "spin_until_future_complete", return_value=None), \
                 patch.object(arm, "transform_pose", side_effect=lambda pose, *args: pose), \
                 patch.object(arm.collision, "get_robot_state", return_value=seed), \
                 patch.object(arm, "joint_control", return_value=True):

                ok = arm.move_abs(0.4, 0.2, 0.0, roll=0.0, pitch=1.57, yaw=0.0)
                self.assertTrue(ok)
                mock_ik_call.assert_called_once()
                ik_req = mock_ik_call.call_args[0][0]
                self.assertAlmostEqual(ik_req.ik_request.pose_stamped.pose.position.x, 0.4)
                self.assertNotEqual(ik_req.ik_request.pose_stamped.pose.orientation.y, 0.0)

    def test_arm_grasp_flow(self):
        import numpy as np
        from types import SimpleNamespace
        from geometry_msgs.msg import TransformStamped
        from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, RobotState, RobotTrajectory
        from geometry_msgs.msg import Pose
        from tf2_ros import Buffer
        from trajectory_msgs.msg import JointTrajectoryPoint

        mock_buffer = MagicMock(spec=Buffer)
        ts = TransformStamped()
        ts.transform.translation.x = 0.25
        ts.transform.translation.y = 0.20
        ts.transform.translation.z = 0.0
        ts.transform.rotation.w = 1.0
        mock_buffer.lookup_transform.return_value = ts

        with patch.object(rclpy.client.Client, "wait_for_service", return_value=True), \
             patch.object(ArmControl, "_ArmControl__load_srdf_group_states", return_value={}):
            arm = ArmControl(self.node, timeout_sec=1.0, tf_buffer=mock_buffer)
            collision = ArmCollision(self.node, timeout_sec=1.0)
            grasp = ArmGrasp(arm, collision)

            self.assertIs(grasp.arm, arm)
            self.assertIs(grasp.collision, collision)

            co = CollisionObject()
            co.id = "cylinder_obj"
            co.header.frame_id = "torso_link"
            co.pose = Pose()
            co.pose.position.x = 0.40
            co.pose.position.y = 0.20
            co.pose.position.z = 0.0
            co.pose.orientation.w = 1.0

            aco = AttachedCollisionObject()
            aco.object = co
            aco.link_name = "left_amazing_hand"

            def move_abs(x, y, z, **kwargs):
                ts.transform.translation.x = float(x)
                ts.transform.translation.y = float(y)
                ts.transform.translation.z = float(z)
                return True

            def move_rel(x, y, z, **kwargs):
                ts.transform.translation.x += float(x)
                ts.transform.translation.y += float(y)
                ts.transform.translation.z += float(z)
                return True

            def move_cartesian_y(distance, **kwargs):
                ts.transform.translation.y += float(distance)
                return True

            def align_grasp_y(center, distance, group):
                matrix = np.eye(4)
                matrix[:3, 3] = center - matrix[:3, 1] * distance
                move_abs(*matrix[:3, 3])
                return matrix

            trajectory = RobotTrajectory()
            trajectory.joint_trajectory.joint_names = ['left_elbow_joint']
            first = JointTrajectoryPoint(positions=[0.0])
            last = JointTrajectoryPoint(positions=[0.02])
            last.time_from_start.sec = 1
            trajectory.joint_trajectory.points = [first, last]
            selected = SimpleNamespace(
                approach_type='side', offset=0.03, pre_offset=0.1,
                center=np.array([0.4, 0.2, 0.0]), direction=np.array([0., 1., 0.]),
                initial_state=RobotState(), pre_state=RobotState(),
                pre_trajectory=RobotTrajectory(), approach_trajectory=trajectory)

            def execute(trajectory):
                if trajectory is selected.pre_trajectory:
                    arm._align_grasp_y(selected.center, 0.13, 'arm_left_with_waist')
                    return True
                return move_cartesian_y(0.1)

            def fk(*args):
                pose = Pose()
                pose.position.x = ts.transform.translation.x
                pose.position.y = ts.transform.translation.y
                pose.position.z = ts.transform.translation.z
                pose.orientation = ts.transform.rotation
                return pose

            from moveit_msgs.msg import AllowedCollisionMatrix
            with patch.object(collision, "get_object", return_value=co), \
                 patch.object(collision, "get_attached_object", return_value=aco), \
                 patch.object(collision, "get_robot_state", return_value=RobotState()), \
                 patch.object(collision, "attach", return_value=True) as mock_attach, \
                 patch.object(collision, "detach", return_value=True) as mock_detach, \
                 patch.object(collision, "get_allowed_collision_matrix", return_value=AllowedCollisionMatrix()), \
                 patch.object(collision, "apply_allowed_collision_matrix", return_value=True), \
                 patch.object(arm, "transform_pose", side_effect=lambda pose, *args: pose), \
                 patch.object(arm, "move_abs", side_effect=move_abs), \
                 patch.object(arm, "_align_grasp_y", side_effect=align_grasp_y) as mock_align, \
                 patch.object(arm, "_execute_checked_trajectory", side_effect=execute), \
                 patch.object(arm, "_fk_pose", side_effect=fk), \
                 patch("erasers_g1_api.grasp_planner.GraspPlanner") as planner_class, \
                 patch.object(arm, "move_rel", side_effect=move_rel) as mock_move_rel:
                planner = planner_class.return_value.__enter__.return_value
                planner.plan.return_value = selected
                planner.matches_state.return_value = True
                planner.valid.return_value = True
                # 1. grasp
                res_grasp = grasp.grasp("cylinder_obj", approach_type="auto", arm_side="left", lift=True)
                self.assertTrue(res_grasp)
                self.assertTrue(mock_align.called)
                self.assertTrue(mock_attach.called)
                self.assertTrue(mock_move_rel.called)

                # 2. place
                res_place = grasp.place(0.35, 0.25, 0.0, arm_side="left", detach=True)
                self.assertTrue(res_place)
                self.assertTrue(mock_detach.called)


if __name__ == "__main__":
    unittest.main()
