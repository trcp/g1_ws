# Copyright (c) 2026 Nakatogawa Laboratory. All rights reserved.

"""把持の座標・PlanningScene・失敗時の状態を検証する回帰テスト．"""

import copy
import math
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import rclpy
import numpy as np
from geometry_msgs.msg import Pose, TransformStamped
from moveit_msgs.msg import (
    AllowedCollisionEntry, AllowedCollisionMatrix, AttachedCollisionObject,
    CollisionObject, PlanningScene, RobotState, RobotTrajectory,
)
from moveit_msgs.srv import GetPlanningScene, GetPositionIK, GetCartesianPath
from rclpy.node import Node
from tf2_ros import Buffer
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from erasers_g1_api.robot_control import (
    ArmCollision, ArmControl, ArmGrasp, _matrix_pose, _object_reference_pose, _pose_matrix,
    _positive_y_path_valid, _sample_joint_trajectory,
)


class TestArmConsistency(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = Node(self._testMethodName)
        self.addCleanup(self.node.destroy_node)
        self.buffer = MagicMock(spec=Buffer)
        self.transform = TransformStamped()
        self.transform.transform.rotation.w = 1.0
        self.transform.transform.translation.x = 0.3
        self.transform.transform.translation.y = 0.2
        self.buffer.lookup_transform.return_value = self.transform
        self.arm = ArmControl(self.node, tf_buffer=self.buffer, use_sim_time=True)
        state = RobotState()
        state.joint_state.name = ['left_elbow_joint', 'right_elbow_joint']
        state.joint_state.position = [0.0, 0.0]
        self.enterContext(patch.object(self.arm.collision, 'get_robot_state', return_value=state))
        self.collision = ArmCollision(self.node)

    def scene_response(self, scene):
        response = GetPlanningScene.Response()
        response.scene = scene
        future = MagicMock()
        future.done.return_value = True
        future.result.return_value = response
        self.enterContext(patch.object(
            self.collision._ArmCollision__get_scene_cli,
            'wait_for_service', return_value=True))
        self.enterContext(patch.object(
            self.collision._ArmCollision__get_scene_cli,
            'call_async', return_value=future))
        self.enterContext(patch.object(rclpy, 'spin_until_future_complete'))

    # Python 3.10 の unittest でも後処理を一元化する．
    def enterContext(self, manager):
        result = manager.__enter__()
        self.addCleanup(manager.__exit__, None, None, None)
        return result

    def test_rotated_object_pose_composes_shape_transform(self):
        obj = CollisionObject()
        obj.header.frame_id = 'pelvis'
        obj.pose.position.x = 1.0
        obj.pose.orientation.z = math.sin(math.pi / 4)
        obj.pose.orientation.w = math.cos(math.pi / 4)
        shape = Pose()
        shape.position.x = 0.2
        shape.orientation.z = math.sin(math.pi / 4)
        shape.orientation.w = math.cos(math.pi / 4)
        obj.primitive_poses = [shape]
        with patch.object(self.collision, 'get_object', return_value=obj):
            x, y, _, _, _, yaw, frame = self.collision.get_object_pose('target')
        self.assertAlmostEqual(x, 1.0)
        self.assertAlmostEqual(y, 0.2)
        self.assertAlmostEqual(abs(yaw), math.pi)
        self.assertEqual(frame, 'pelvis')

    def test_allow_collision_changes_only_requested_pair(self):
        scene = PlanningScene()
        scene.allowed_collision_matrix.entry_names = ['table', 'body']
        scene.allowed_collision_matrix.entry_values = [
            AllowedCollisionEntry(enabled=[False, False]),
            AllowedCollisionEntry(enabled=[False, False]),
        ]
        self.scene_response(scene)
        with patch.object(self.collision, '_ArmCollision__apply', return_value=True) as apply:
            self.assertTrue(self.collision.allow_collision('target', 'hand'))
        acm = apply.call_args.args[0].allowed_collision_matrix
        index = acm.entry_names.index
        self.assertTrue(acm.entry_values[index('target')].enabled[index('hand')])
        self.assertFalse(acm.entry_values[index('target')].enabled[index('table')])
        self.assertFalse(acm.entry_values[index('body')].enabled[index('hand')])

    def test_relative_motion_in_fixed_frame_adds_current_position(self):
        with patch.object(self.arm, 'move_abs', return_value=True) as move:
            self.assertTrue(self.arm.move_rel(0.01, 0.0, 0.1, ref_frame='pelvis'))
        target = move.call_args.kwargs
        self.assertAlmostEqual(target['x'], 0.31)
        self.assertAlmostEqual(target['y'], 0.2)
        self.assertAlmostEqual(target['z'], 0.1)

    def test_legacy_right_group_is_used_for_ik_and_execution(self):
        response = GetPositionIK.Response()
        response.error_code.val = 1
        response.solution.joint_state.name = ['left_elbow_joint', 'right_elbow_joint']
        response.solution.joint_state.position = [0.4, 0.5]
        future = MagicMock()
        future.done.return_value = True
        future.result.return_value = response
        with patch.object(self.arm._ArmControl__ik_cli, 'service_is_ready', return_value=True), \
                patch.object(self.arm._ArmControl__ik_cli, 'call_async', return_value=future), \
                patch.object(rclpy, 'spin_until_future_complete'), \
                patch.object(self.arm, 'joint_control', return_value=True) as move:
            self.assertTrue(self.arm.move_abs(0.3, -0.2, 0.0, 'pelvis', 'arm_right'))
            self.assertTrue(self.arm.move_abs(0.3, -0.2, 0.0, 'pelvis', 'arm_right', False))
        self.assertEqual(move.call_args.kwargs['planning_group'], 'arm_right')
        self.assertFalse(move.call_args.kwargs['wait'])
        self.assertNotIn('left_elbow_joint', move.call_args.kwargs)
        self.assertEqual(move.call_args.kwargs['right_elbow_joint'], 0.5)

    def test_model_external_joints_never_enter_ik_or_move_group_start(self):
        response = GetPositionIK.Response()
        response.error_code.val = 1
        response.solution.joint_state.name = ['left_elbow_joint']
        response.solution.joint_state.position = [0.3]
        future = MagicMock()
        future.done.return_value = True
        future.result.return_value = response
        client = self.arm._ArmControl__ik_cli
        # 実際の停止原因になった，MoveIt モデル外のトピック関節を再現する．
        with patch.object(self.arm, 'get_current_joint_pose', return_value={
                'left_elbow_joint': 0.0, 'left_wrist_pitch_joint': 0.0,
                'left_wrist_yaw_joint': 0.0}) as topic_state, \
                patch.object(client, 'service_is_ready', return_value=True), \
                patch.object(client, 'call_async', return_value=future) as ik, \
                patch.object(rclpy, 'spin_until_future_complete'), \
                patch.object(self.arm, 'joint_control', return_value=True):
            self.assertTrue(self.arm.move_abs(0.3, 0.2, 0.0, ref_frame='pelvis'))
            goal = self.arm._create_move_group_goal('arm_left')
        self.assertEqual(ik.call_args.args[0].ik_request.robot_state.joint_state.name,
                         ['left_elbow_joint', 'right_elbow_joint'])
        self.assertTrue(goal.request.start_state.is_diff)
        self.assertEqual(goal.request.start_state.joint_state.name, [])
        topic_state.assert_not_called()

    def prepare_cartesian_response(self, fraction=1.0):
        response = GetCartesianPath.Response()
        response.fraction = fraction
        response.error_code.val = 1
        trajectory = JointTrajectory(joint_names=['left_elbow_joint'])
        first = JointTrajectoryPoint(positions=[0.0])
        last = JointTrajectoryPoint(positions=[0.02])
        last.time_from_start.sec = 1
        trajectory.points = [first, last]
        response.solution.joint_trajectory = trajectory
        future = MagicMock()
        future.done.return_value = True
        future.result.return_value = response
        self.enterContext(patch.object(self.arm._ArmControl__fk_cli,
                                       'wait_for_service', return_value=True))
        self.enterContext(patch.object(self.arm._ArmControl__cartesian_cli,
                                       'wait_for_service', return_value=True))
        call = self.enterContext(patch.object(self.arm._ArmControl__cartesian_cli,
                                              'call_async', return_value=future))
        self.enterContext(patch.object(rclpy, 'spin_until_future_complete'))

        def fk(state, link):
            pose = Pose()
            pose.orientation.w = 1.0
            values = dict(zip(state.joint_state.name, state.joint_state.position))
            pose.position.y = values['left_elbow_joint']
            return pose

        self.enterContext(patch.object(self.arm, '_fk_pose', side_effect=fk))
        execute = self.enterContext(patch.object(
            self.arm, '_execute_checked_trajectory', return_value=True))
        return response, call, execute

    def test_partial_cartesian_path_is_never_executed(self):
        _, _, execute = self.prepare_cartesian_response(fraction=0.5)
        self.assertFalse(self.arm.move_cartesian_y(0.02))
        execute.assert_not_called()

    def test_joint_tolerance_is_not_forwarded_as_a_joint(self):
        with patch.object(self.arm, '_ArmControl__send_move_group_goal',
                          return_value=True) as send:
            self.assertTrue(self.arm.joint_control(
                planning_group='arm_left', joint_tolerance=0.0001, left_elbow_joint=0.5))
            constraint = send.call_args.args[0].request.goal_constraints[0].joint_constraints
            self.assertEqual(len(constraint), 1)
            self.assertEqual(constraint[0].joint_name, 'left_elbow_joint')
            self.assertEqual(constraint[0].tolerance_above, 0.0001)
            self.assertEqual(constraint[0].tolerance_below, 0.0001)
            send.reset_mock()
            self.assertFalse(self.arm.joint_control(joint_tolerance=0., left_elbow_joint=0.5))
            send.assert_not_called()

    def test_verified_cartesian_path_executes_exact_returned_trajectory(self):
        response, call, execute = self.prepare_cartesian_response()
        initial = self.arm.collision.get_robot_state()
        reached = copy.deepcopy(initial)
        reached.joint_state.position[0] = 0.02
        self.arm.collision.get_robot_state.side_effect = [initial, initial, reached]
        self.assertTrue(self.arm.move_cartesian_y(0.02))
        execute.assert_called_once_with(response.solution)
        request = call.call_args.args[0]
        self.assertTrue(request.avoid_collisions)
        self.assertEqual(request.link_name, 'left_amazing_hand')
        self.assertEqual(len(request.path_constraints.orientation_constraints), 1)
        self.assertAlmostEqual(request.waypoints[0].position.y, 0.02)

    def test_cartesian_trajectory_cannot_introduce_unknown_joint(self):
        response, _, execute = self.prepare_cartesian_response()
        response.solution.joint_trajectory.joint_names = ['left_wrist_pitch_joint']
        self.assertFalse(self.arm.move_cartesian_y(0.02))
        execute.assert_not_called()

    def test_pregrasp_solver_checks_reached_virtual_point(self):
        self.arm._ArmControl__srdf_states = {'arm_left': {'home': {'left_elbow_joint': 0.0}}}
        initial = self.arm.collision.get_robot_state()
        achieved = copy.deepcopy(initial)

        def fk(state, link):
            pose = Pose()
            pose.orientation.w = 1.0
            pose.position.x = float(state.joint_state.position[0])
            return pose

        def execute(goal):
            constraint = goal.request.goal_constraints[0].joint_constraints[0]
            achieved.joint_state.position[0] = constraint.position
            self.arm.collision.get_robot_state.return_value = achieved
            return True

        with patch.object(self.arm._ArmControl__fk_cli, 'wait_for_service', return_value=True), \
                patch.object(self.arm, '_fk_pose', side_effect=fk), \
                patch.object(self.arm, '_ArmControl__send_move_group_goal',
                             side_effect=execute) as move:
            reached = self.arm._align_grasp_y(np.array([0.02, 0.13, 0.0]), 0.13, 'arm_left')
        self.assertIsNotNone(reached)
        np.testing.assert_allclose(reached[:3, 3] + reached[:3, 1] * 0.13,
                                   [0.02, 0.13, 0.0], atol=0.0005)
        move.assert_called_once()

    def test_unreachable_pregrasp_does_not_move(self):
        self.arm._ArmControl__srdf_states = {'arm_left': {'home': {'left_elbow_joint': 0.0}}}
        pose = Pose()
        pose.orientation.w = 1.0
        with patch.object(self.arm._ArmControl__fk_cli, 'wait_for_service', return_value=True), \
                patch.object(self.arm, '_fk_pose', return_value=pose), \
                patch.object(self.arm, '_ArmControl__send_move_group_goal') as move:
            self.assertIsNone(self.arm._align_grasp_y(
                np.array([1.0, 0.0, 0.0]), 0.13, 'arm_left'))
        move.assert_not_called()


class TestGraspState(unittest.TestCase):
    """手先の回転とシーンの移管を模擬し，外部から見える物体状態を検証する．"""

    def setUp(self):
        self.ee = np.eye(4)
        self.ee[:3, 3] = [0.25, 0.2, 0.0]
        self.events = []
        self.world = CollisionObject(id='target')
        self.world.header.frame_id = 'pelvis'
        self.world.pose.orientation.w = 1.0
        shape = Pose()
        shape.position.x, shape.position.y = 0.4, 0.2
        shape.orientation.w = 1.0
        self.world.primitive_poses = [shape]
        self.attached = None
        self.arm = MagicMock(spec=ArmControl)
        self.arm._ArmControl__node = MagicMock()
        self.arm.get_current_endeffector_pose.side_effect = self.current_pose
        self.arm.transform_pose.side_effect = self.transform_pose
        self.arm.move_abs.side_effect = self.move_abs
        self.arm._align_grasp_y.side_effect = self.align_grasp_y
        self.arm.move_cartesian_y.side_effect = self.move_cartesian_y
        self.arm.move_rel.side_effect = self.move_rel
        self.arm._fk_pose.side_effect = lambda *args: _matrix_pose(self.ee)
        self.arm._execute_checked_trajectory.side_effect = self.execute
        self.collision = MagicMock(spec=ArmCollision)
        self.collision.get_object.side_effect = lambda name: copy.deepcopy(self.world)
        self.collision.get_attached_object.side_effect = (
            lambda *a, **k: copy.deepcopy(self.attached))
        self.collision.attach.side_effect = self.attach
        self.collision.detach.side_effect = self.detach
        self.original_acm = AllowedCollisionMatrix(
            entry_names=['table'], entry_values=[AllowedCollisionEntry(enabled=[False])])
        self.collision.get_allowed_collision_matrix.return_value = self.original_acm
        self.collision.apply_allowed_collision_matrix.return_value = True
        self.grasp = ArmGrasp(self.arm, self.collision)
        planner_patch = patch('erasers_g1_api.grasp_planner.GraspPlanner')
        self.planner = planner_patch.start().return_value.__enter__.return_value
        self.addCleanup(planner_patch.stop)
        self.planner.plan.side_effect = self.plan
        self.planner.matches_state.return_value = True
        self.planner.valid.return_value = True

    def plan(self, obj, mode, side, group, offset, pre_offset, ref):
        trajectory = RobotTrajectory()
        trajectory.joint_trajectory.joint_names = ['joint']
        first, last = JointTrajectoryPoint(positions=[0.0]), JointTrajectoryPoint(positions=[0.02])
        last.time_from_start.sec = 1
        trajectory.joint_trajectory.points = [first, last]
        self.selected = SimpleNamespace(
            approach_type=mode if mode != 'auto' else 'side',
            offset=0.03 if offset is None else offset,
            pre_offset=0.10 if pre_offset is None else pre_offset,
            direction=self.ee[:3, 1].copy(), center=np.array([0.4, 0.2, 0.0]),
            initial_state=RobotState(), pre_state=RobotState(),
            pre_trajectory=RobotTrajectory(), approach_trajectory=trajectory, group=group)
        return self.selected

    def execute(self, trajectory):
        if trajectory is self.selected.pre_trajectory:
            return self.arm._align_grasp_y(
                self.selected.center, self.selected.offset + self.selected.pre_offset,
                self.selected.group) is not None
        self.assertIs(trajectory, self.selected.approach_trajectory)
        return self.arm.move_cartesian_y(self.selected.pre_offset)

    def current_pose(self, ref='pelvis', arm_side='left'):
        pose = _matrix_pose(self.ee)
        transform = TransformStamped().transform
        transform.translation.x = pose.position.x
        transform.translation.y = pose.position.y
        transform.translation.z = pose.position.z
        transform.rotation = pose.orientation
        return transform

    def transform_pose(self, pose, source_frame, target_frame='pelvis'):
        if source_frame == target_frame:
            return copy.deepcopy(pose)
        if source_frame.endswith('_amazing_hand') and target_frame == 'pelvis':
            return _matrix_pose(self.ee @ _pose_matrix(pose))
        if source_frame == 'pelvis' and target_frame.endswith('_amazing_hand'):
            return _matrix_pose(np.linalg.inv(self.ee) @ _pose_matrix(pose))
        self.fail(f'未定義のテスト座標変換: {source_frame} -> {target_frame}')

    def move_abs(self, x, y, z, **kwargs):
        self.assertEqual(kwargs['ref_frame'], 'pelvis')
        self.events.append('move')
        self.ee[:3, 3] = [x, y, z]
        return True

    def align_grasp_y(self, center, distance, group_name):
        self.events.append('align')
        self.ee[:3, 3] = center - self.ee[:3, 1] * distance
        return self.ee.copy()

    def move_rel(self, x, y, z, **kwargs):
        self.assertIsNotNone(self.attached)
        self.assertEqual(kwargs['ref_frame'], 'pelvis')
        self.events.append('lift')
        self.ee[:3, 3] += [x, y, z]
        return True

    def move_cartesian_y(self, distance, **kwargs):
        self.events.append('approach_y')
        self.assertGreater(distance, 0.0)
        self.ee[:3, 3] += self.ee[:3, 1] * distance
        return True

    def attach(self, name, attach_frame, **kwargs):
        self.events.append('attach')
        before = _pose_matrix(_object_reference_pose(self.world))
        obj = copy.deepcopy(self.world)
        obj.pose = _matrix_pose(np.linalg.inv(self.ee) @ _pose_matrix(obj.pose))
        obj.header.frame_id = attach_frame
        self.attached = AttachedCollisionObject(link_name=attach_frame, object=obj)
        self.world = None
        np.testing.assert_allclose(
            self.ee @ _pose_matrix(_object_reference_pose(obj)), before, atol=1e-12)
        return True

    def detach(self, name, attach_frame):
        self.events.append('detach')
        self.world = copy.deepcopy(self.attached.object)
        self.world.pose = _matrix_pose(self.ee @ _pose_matrix(self.world.pose))
        self.world.header.frame_id = 'pelvis'
        self.attached = None
        return True

    def assert_restored(self):
        self.assertEqual(
            self.collision.apply_allowed_collision_matrix.call_args.args[0], self.original_acm)
        self.collision.remove_collision.assert_not_called()

    def test_attach_precedes_vertical_lift_without_object_jump(self):
        self.assertTrue(self.grasp.grasp('target'))
        self.assertEqual(self.events, ['align', 'approach_y', 'attach', 'lift'])
        actual = self.ee @ _pose_matrix(_object_reference_pose(self.attached.object))
        np.testing.assert_allclose(actual[:3, 3], [0.4, 0.2, 0.1], atol=1e-12)
        self.assert_restored()

    def test_approach_failure_preserves_world_object_and_acm(self):
        self.arm.move_cartesian_y.side_effect = None
        self.arm.move_cartesian_y.return_value = False
        self.assertFalse(self.grasp.grasp('target'))
        self.assertIsNotNone(self.world)
        self.collision.attach.assert_not_called()
        self.arm.move_rel.assert_not_called()
        self.assert_restored()

    def test_exception_restores_acm_without_deleting_object(self):
        self.arm.move_cartesian_y.side_effect = RuntimeError('通信失敗')
        self.assertFalse(self.grasp.grasp('target'))
        self.assertIsNotNone(self.world)
        self.assert_restored()

    def test_lift_failure_keeps_attached_object_and_returns_false(self):
        self.arm.move_rel.side_effect = None
        self.arm.move_rel.return_value = False
        self.assertFalse(self.grasp.grasp('target'))
        self.assertIsNotNone(self.attached)
        self.assertIsNone(self.world)
        self.assert_restored()

    def test_attach_failure_does_not_lift_or_remove_world_object(self):
        self.collision.attach.side_effect = None
        self.collision.attach.return_value = False
        self.assertFalse(self.grasp.grasp('target'))
        self.assertIsNotNone(self.world)
        self.arm.move_rel.assert_not_called()
        self.assert_restored()

    def test_acm_restore_failure_prevents_lift(self):
        self.collision.apply_allowed_collision_matrix.side_effect = [True, False]
        self.assertFalse(self.grasp.grasp('target'))
        self.assertIsNotNone(self.attached)
        self.arm.move_rel.assert_not_called()

    def test_arm_group_mismatch_is_rejected_before_motion(self):
        self.assertFalse(self.grasp.grasp('target', arm_side='right', group_name='arm_left'))
        self.arm.move_abs.assert_not_called()
        self.collision.apply_allowed_collision_matrix.assert_not_called()

    def test_right_arm_attaches_to_right_hand(self):
        self.assertTrue(self.grasp.grasp('target', arm_side='right', lift=False))
        self.assertEqual(self.attached.link_name, 'right_amazing_hand')
        for call in self.arm.move_abs.call_args_list:
            self.assertEqual(call.kwargs['group_name'], 'arm_right_with_waist')

    def test_corrected_left_frame_uses_its_positive_y(self):
        self.ee[:3, :3] = np.diag([1.0, -1.0, -1.0])
        self.assertTrue(self.grasp.grasp('target', arm_side='left', lift=False))
        center, distance, group = self.arm._align_grasp_y.call_args.args
        np.testing.assert_allclose(center, [0.4, 0.2, 0.0], atol=1e-12)
        self.assertAlmostEqual(distance, 0.13)
        self.assertEqual(group, 'arm_left_with_waist')
        np.testing.assert_allclose(self.ee[:3, 3], [0.4, 0.23, 0.0], atol=1e-12)
        local = _pose_matrix(_object_reference_pose(self.attached.object))[:3, 3]
        np.testing.assert_allclose(local, [0.0, 0.03, 0.0], atol=1e-12)

    def test_preposition_failure_does_not_allow_contact(self):
        self.arm._align_grasp_y.side_effect = None
        self.arm._align_grasp_y.return_value = None
        self.assertFalse(self.grasp.grasp('target'))
        self.collision.apply_allowed_collision_matrix.assert_not_called()
        self.arm.move_cartesian_y.assert_not_called()

    def test_no_feasible_candidate_does_not_move_or_modify_scene(self):
        self.planner.plan.side_effect = None
        self.planner.plan.return_value = None
        self.assertFalse(self.grasp.grasp('target'))
        self.assertIsNone(self.grasp.last_grasp_plan)
        self.arm._execute_checked_trajectory.assert_not_called()
        self.collision.apply_allowed_collision_matrix.assert_not_called()

    def test_changed_start_state_prevents_execution(self):
        self.planner.matches_state.return_value = False
        self.assertFalse(self.grasp.grasp('target'))
        self.arm._execute_checked_trajectory.assert_not_called()
        self.collision.apply_allowed_collision_matrix.assert_not_called()

    def test_invalid_approach_and_distances_are_rejected_before_planning(self):
        for parameters in ({'approach_type': 'unknown'}, {'pre_offset_dist': 0.0},
                           {'offset_dist': -0.1}, {'lift_height': float('nan')}):
            with self.subTest(parameters=parameters):
                self.assertFalse(self.grasp.grasp('target', **parameters))
        self.planner.plan.assert_not_called()

    def test_selection_metadata_does_not_allow_external_mutation(self):
        self.assertTrue(self.grasp.grasp('target', lift=False, approach_type='top',
                                         offset_dist=0.05, pre_offset_dist=0.025))
        selected = self.grasp.last_grasp_plan
        self.assertEqual(selected['approach_type'], 'top')
        self.assertEqual(selected['offset_dist'], 0.05)
        self.assertEqual(selected['pre_offset_dist'], 0.025)
        selected['direction_pelvis'][1] = -100.0
        self.assertEqual(self.grasp.last_grasp_plan['direction_pelvis'][1], 1.0)

    def test_cartesian_failure_has_no_position_only_fallback(self):
        self.arm.move_cartesian_y.side_effect = None
        self.arm.move_cartesian_y.return_value = False
        self.assertFalse(self.grasp.grasp('target'))
        self.arm.move_abs.assert_not_called()
        self.collision.attach.assert_not_called()
        self.assert_restored()

    def test_reversed_actual_hand_direction_prevents_attachment(self):
        def reversed_approach(distance, **kwargs):
            self.move_cartesian_y(distance, **kwargs)
            self.ee[:3, :3] = np.diag([1.0, -1.0, -1.0])
            return True
        self.arm.move_cartesian_y.side_effect = reversed_approach
        self.assertFalse(self.grasp.grasp('target'))
        self.collision.attach.assert_not_called()
        self.assert_restored()

    def test_detach_false_keeps_object_attached(self):
        self.assertTrue(self.grasp.grasp('target', lift=False))
        self.assertTrue(self.grasp.place(0.35, 0.2, 0.0, ref='pelvis', detach=False))
        self.assertIsNotNone(self.attached)
        self.collision.detach.assert_not_called()
        self.assert_restored()

    def test_detach_failure_is_not_reported_as_success(self):
        self.assertTrue(self.grasp.grasp('target', lift=False))
        self.collision.detach.side_effect = None
        self.collision.detach.return_value = False
        self.assertFalse(self.grasp.place(0.35, 0.2, 0.0, ref='pelvis'))
        self.assertIsNotNone(self.attached)

    def test_place_compensates_actual_wrist_rotation(self):
        self.assertTrue(self.grasp.grasp('target', lift=False))
        self.arm.move_abs.reset_mock()

        def rotate_and_move(x, y, z, **kwargs):
            self.ee[:3, :3] = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
            return self.move_abs(x, y, z, **kwargs)

        self.arm.move_abs.side_effect = rotate_and_move
        self.assertTrue(self.grasp.place(0.35, 0.2, 0.05, ref='pelvis'))
        self.assertEqual(self.arm.move_abs.call_count, 2)
        actual = _pose_matrix(_object_reference_pose(self.world))
        np.testing.assert_allclose(actual[:3, 3], [0.35, 0.2, 0.05], atol=1e-12)

    def test_unreached_place_never_teleports_object(self):
        self.assertTrue(self.grasp.grasp('target', lift=False))
        self.arm.move_abs.side_effect = None
        self.arm.move_abs.return_value = True
        self.assertFalse(self.grasp.place(0.8, 0.2, 0.0, ref='pelvis'))
        self.collision.detach.assert_not_called()
        self.assertIsNotNone(self.attached)

    def test_relative_place_offsets_object_center(self):
        self.assertTrue(self.grasp.grasp('target', lift=False))
        self.assertTrue(self.grasp.place(0.05, 0.0, 0.0, ref='pelvis', use_rlt=True))
        actual = _pose_matrix(_object_reference_pose(self.world))
        np.testing.assert_allclose(actual[:3, 3], [0.45, 0.2, 0.0], atol=1e-12)

    def test_wrong_attachment_side_is_rejected(self):
        self.assertTrue(self.grasp.grasp('target', lift=False))
        self.arm.move_abs.reset_mock()
        self.assertFalse(self.grasp.place(0.35, 0.2, 0.0, arm_side='right', ref='pelvis'))
        self.arm.move_abs.assert_not_called()
        self.collision.detach.assert_not_called()


class TestPositiveYPath(unittest.TestCase):
    def path(self, positions):
        poses = []
        for xyz in positions:
            pose = np.eye(4)
            pose[:3, 3] = xyz
            poses.append(pose)
        return poses

    def test_accepts_positive_y_path_for_both_frame_orientations(self):
        for rotation in (np.eye(3), np.diag([1., -1., -1.])):
            start = np.eye(4)
            start[:3, :3] = rotation
            poses = []
            for distance in (0.0, 0.01, 0.02):
                pose = start.copy()
                pose[:3, 3] += rotation[:, 1] * distance
                poses.append(pose)
            self.assertTrue(_positive_y_path_valid(poses, start, 0.02))

    def test_rejects_sideways_motion_even_with_correct_endpoint(self):
        poses = self.path([[0, 0, 0], [0.004, 0.01, 0], [0, 0.02, 0]])
        self.assertFalse(_positive_y_path_valid(poses, np.eye(4), 0.02))

    def test_rejects_backward_or_incomplete_motion(self):
        for positions in ([[0, 0, 0], [0, -0.01, 0], [0, 0.02, 0]],
                          [[0, 0, 0], [0, 0.01, 0]]):
            self.assertFalse(_positive_y_path_valid(self.path(positions), np.eye(4), 0.02))

    def test_rejects_hand_rotation_away_from_motion_direction(self):
        poses = self.path([[0, 0, 0], [0, 0.02, 0]])
        poses[-1][:3, :3] = np.diag([1., -1., -1.])
        self.assertFalse(_positive_y_path_valid(poses, np.eye(4), 0.02))

    def test_quintic_interpolation_detects_reverse_between_valid_endpoints(self):
        trajectory = JointTrajectory(joint_names=['joint'])
        first = JointTrajectoryPoint(positions=[0.0], velocities=[-0.1], accelerations=[0.0])
        last = JointTrajectoryPoint(positions=[0.02], velocities=[0.0], accelerations=[0.0])
        last.time_from_start.sec = 1
        trajectory.points = [first, last]
        samples = _sample_joint_trajectory(trajectory)
        self.assertIsNotNone(samples)
        self.assertAlmostEqual(samples[-1][0], 0.02)
        self.assertLess(min(p[0] for p in samples), 0.0)
        poses = self.path([[0, p[0], 0] for p in samples])
        self.assertFalse(_positive_y_path_valid(poses, np.eye(4), 0.02))

    def test_malformed_trajectory_is_rejected_before_fk(self):
        for problem in ('nan', 'delayed_start', 'mixed_velocity', 'acceleration_only'):
            with self.subTest(problem=problem):
                trajectory = JointTrajectory(joint_names=['joint'])
                first = JointTrajectoryPoint(positions=[0.0])
                last = JointTrajectoryPoint(positions=[0.02])
                last.time_from_start.sec = 1
                if problem == 'nan':
                    first.positions[0] = float('nan')
                elif problem == 'delayed_start':
                    first.time_from_start.nanosec = 1
                elif problem == 'mixed_velocity':
                    last.velocities = [0.0]
                else:
                    first.accelerations = last.accelerations = [0.0]
                trajectory.points = [first, last]
                self.assertIsNone(_sample_joint_trajectory(trajectory))


if __name__ == '__main__':
    unittest.main()
