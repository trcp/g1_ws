# Copyright (c) 2026 Nakatogawa Laboratory. All rights reserved.

"""方向・自動距離・衝突に基づく候補選択の回帰試験．"""

from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

import numpy as np
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, ContactInformation, RobotState, RobotTrajectory
from moveit_msgs.srv import GetCartesianPath, GetStateValidity
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectoryPoint

from erasers_g1_api.robot_control import _pose_matrix
from erasers_g1_api.grasp_geometry import HandChain, distance_candidates, object_near_extent
from erasers_g1_api.grasp_planner import GraspPlanner


class TestGraspGeometry(unittest.TestCase):
    URDF = '''<robot name="test"><link name="pelvis"/>
    <link name="wrist"><collision><geometry><box size="0.2 0.08 0.1"/>
    </geometry></collision></link><link name="left_amazing_hand"/>
    <joint name="yaw" type="revolute"><parent link="pelvis"/><child link="wrist"/>
    <axis xyz="0 0 1"/><limit lower="-2" upper="2"/></joint>
    <joint name="tip" type="fixed"><parent link="wrist"/><child link="left_amazing_hand"/>
    <origin xyz="0.16 0 0" rpy="3.141592653589793 0 0"/></joint></robot>'''

    def test_corrected_hand_axis_and_analytic_jacobian(self):
        chain = HandChain(self.URDF, 'left_amazing_hand', ['yaw'])
        pose, jacobian = chain.forward([np.pi / 2], {}, jacobian=True)
        np.testing.assert_allclose(pose[:3, 3], [0., .16, 0.], atol=1e-12)
        np.testing.assert_allclose(pose[:3, 1], [1., 0., 0.], atol=1e-12)
        self.assertAlmostEqual(chain.hand_reach, .04)
        varied = chain.forward([np.pi / 2 + 1e-6], {})
        difference = np.r_[varied[:3, 3] - pose[:3, 3],
                           .15 * (varied[:3, 1] - pose[:3, 1])] / 1e-6
        np.testing.assert_allclose(jacobian[:, 0], difference, atol=1e-6)

    def test_axis_constrained_solver_respects_joint_limits(self):
        chain = HandChain(self.URDF, 'left_amazing_hand', ['yaw'])
        solved = chain.solve(np.array([0., .16, 0.]), np.array([1., 0., 0.]), [.2], {})
        self.assertIsNotNone(solved)
        self.assertAlmostEqual(solved[0], np.pi / 2, places=3)
        self.assertIsNone(chain.solve(np.array([1., 1., 1.]), np.array([0., 0., -1.]), [0.], {}))

    def test_rotated_compound_object_extent(self):
        obj = CollisionObject()
        shape = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[.2, .04, .06])
        pose = Pose()
        pose.orientation.z, pose.orientation.w = np.sin(np.pi / 4), np.cos(np.pi / 4)
        obj.primitives = [shape]
        obj.primitive_poses = [pose]
        self.assertAlmostEqual(object_near_extent(
            obj, np.eye(4), np.zeros(3), np.array([1., 0., 0.]), _pose_matrix), .02)
        self.assertAlmostEqual(object_near_extent(
            obj, np.eye(4), np.zeros(3), np.array([0., 1., 0.]), _pose_matrix), .1)

    def test_explicit_distances_are_not_adjusted(self):
        self.assertEqual(distance_candidates(.2, .04, .03, .12), [(.03, .12)])
        candidates = distance_candidates(.04, .04, None, None)
        self.assertEqual(len(candidates), 9)
        self.assertAlmostEqual(max(o for o, _ in candidates), .075)
        self.assertEqual({p for _, p in candidates}, {.025, .05, .10})


class TestCandidateSelection(unittest.TestCase):
    def setUp(self):
        self.arm = MagicMock()
        self.arm._ArmControl__timeout_sec = 1.
        self.arm._ArmControl__srdf_states = {'arm_left': {'home': {'x': 0., 'y': 0., 'z': 0.}}}
        self.arm.transform_pose.side_effect = lambda pose, *args: pose
        origin = Pose()
        origin.orientation.w = 1.
        self.arm._fk_pose.return_value = origin
        self.state = RobotState()
        self.state.joint_state.name = ['x', 'y', 'z']
        self.state.joint_state.position = [0., 0., 0.]
        self.collision = MagicMock()
        self.collision.get_robot_state.return_value = self.state
        self.planner = GraspPlanner(self.arm, self.collision)
        self.addCleanup(self.planner.__exit__)
        self.planner.call = MagicMock(return_value=SimpleNamespace(
            values=[SimpleNamespace(string_value='robot')]))
        self.planner.valid = MagicMock(return_value=True)
        chain = MagicMock()
        chain.names = ['x', 'y', 'z']
        chain.lower, chain.upper = np.full(3, -2.), np.full(3, 2.)
        chain.hand_reach = .04
        chain.forward.return_value = np.eye(4)
        chain.seeds.side_effect = lambda current: [current]
        chain.solve.side_effect = lambda position, *args: position
        manager = patch('erasers_g1_api.grasp_planner.HandChain', return_value=chain)
        manager.start()
        self.addCleanup(manager.stop)
        self.obj = CollisionObject(id='target')
        self.obj.header.frame_id = 'pelvis'
        self.obj.pose.orientation.w = 1.
        self.obj.pose.position.x = .4
        shape_pose = Pose()
        shape_pose.orientation.w = 1.
        self.obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[.08] * 3)]
        self.obj.primitive_poses = [shape_pose]
        self.planner.plan_pre = MagicMock(side_effect=self.plan_pre)
        self.planner.plan_approach = MagicMock(return_value=RobotTrajectory())

    def plan_pre(self, initial, target, names, group):
        trajectory = RobotTrajectory()
        trajectory.joint_trajectory.joint_names = names
        trajectory.joint_trajectory.points = [JointTrajectoryPoint(
            positions=list(target.joint_state.position))]
        return trajectory

    def plan(self, mode='auto', offset=None, pre=None):
        return self.planner.plan(self.obj, mode, 'left', 'arm_left', offset, pre, 'base_link')

    def test_auto_avoids_blocked_side_and_front_without_moving(self):
        # 左側面と正面の候補を障害物で塞ぎ，上方だけを残す．
        self.planner.valid.side_effect = lambda state, *args: (
            state is self.state or state.joint_state.position[2] > .05)
        selected = self.plan()
        self.assertEqual(selected.approach_type, 'top')
        np.testing.assert_allclose(selected.direction, [0., 0., -1.])
        self.arm._execute_checked_trajectory.assert_not_called()
        self.collision.apply_allowed_collision_matrix.assert_not_called()

    def test_manual_direction_does_not_fall_back(self):
        self.planner.valid.side_effect = lambda state, *args: state is self.state
        self.assertIsNone(self.plan('front'))
        self.planner.plan_approach.assert_not_called()

    def test_distance_changes_when_pregrasp_is_blocked(self):
        # 接近前の距離が 12 cm 以上なら，上方の空間へ抜けられる状況を模擬する．
        self.planner.valid.side_effect = lambda state, *args: (
            state is self.state or state.joint_state.position[2] >= .12)
        selected = self.plan('top')
        self.assertIsNotNone(selected)
        self.assertGreater(selected.offset + selected.pre_offset, .12 - 1e-9)
        self.assertIsNone(self.plan('top', .03, .025))

    def test_explicit_distances_reach_the_planner_unchanged(self):
        selected = self.plan('front', .08, .04)
        self.assertEqual((selected.offset, selected.pre_offset), (.08, .04))
        np.testing.assert_allclose(selected.direction, [1., 0., 0.])

    def test_right_side_uses_positive_reference_y(self):
        self.arm._ArmControl__srdf_states['arm_right'] = {'home': {'x': 0., 'y': 0., 'z': 0.}}
        selected = self.planner.plan(self.obj, 'side', 'right', 'arm_right', .08, .04, 'base_link')
        np.testing.assert_allclose(selected.direction, [0., 1., 0.])

    def test_reference_rotation_is_applied_to_direction(self):
        def transform(pose, source, destination):
            if source == 'rotated_reference':
                result = Pose()
                result.orientation.z, result.orientation.w = np.sin(np.pi / 4), np.cos(np.pi / 4)
                return result
            return pose

        self.arm.transform_pose.side_effect = transform
        selected = self.planner.plan(
            self.obj, 'front', 'left', 'arm_left', .08, .04, 'rotated_reference')
        np.testing.assert_allclose(selected.direction, [0., 1., 0.], atol=1e-12)

    def test_all_blocked_does_not_execute_or_modify_scene(self):
        self.planner.valid.side_effect = lambda state, *args: state is self.state
        self.assertIsNone(self.plan())
        self.arm._execute_checked_trajectory.assert_not_called()
        self.collision.apply_allowed_collision_matrix.assert_not_called()

    def test_allowed_target_contact_does_not_hide_table_contact(self):
        planner = GraspPlanner(self.arm, self.collision)
        self.addCleanup(planner.__exit__)
        response = GetStateValidity.Response(valid=False)
        response.contacts = [ContactInformation(
            contact_body_1='target', contact_body_2='left_wrist_roll_rubber_hand')]
        planner.call = MagicMock(return_value=response)
        self.assertTrue(planner.valid(self.state, 'target', ['left_wrist_roll_rubber_hand']))
        response.contacts.append(ContactInformation(
            contact_body_1='table', contact_body_2='left_wrist_roll_rubber_hand'))
        self.assertFalse(planner.valid(self.state, 'target', ['left_wrist_roll_rubber_hand']))

    def test_partial_or_unknown_joint_cartesian_path_is_rejected(self):
        chain = MagicMock()
        chain.names = ['x', 'y', 'z']
        chain.forward.return_value = np.eye(4)
        planner = GraspPlanner(self.arm, self.collision)
        self.addCleanup(planner.__exit__)
        for partial in (True, False):
            with self.subTest(partial=partial):
                response = GetCartesianPath.Response()
                response.error_code.val = 1
                response.fraction = .5 if partial else 1.
                response.solution.joint_trajectory.joint_names = ['left_wrist_pitch_joint']
                first = JointTrajectoryPoint(positions=[0.])
                last = JointTrajectoryPoint(positions=[.01])
                last.time_from_start.sec = 1
                response.solution.joint_trajectory.points = [first, last]
                planner.call = MagicMock(return_value=response)
                self.assertIsNone(planner.plan_approach(
                    self.state, 'arm_left', 'left', .025, np.array([0., 1., 0.]),
                    chain, {}, 'target', ['left_amazing_hand']))


if __name__ == '__main__':
    unittest.main()
