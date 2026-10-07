# Copyright (c) 2026 Nakatogawa Laboratory. All rights reserved.

"""起動済みモック環境での明示的な把持検証．通常のテスト実行では動かさない．

G1_RUN_ARM_GRASP_SIMULATION=1 を指定し，sim_bringup が起動した Docker 内で実行する．
検証用物体だけを後処理し，開始時の腕・腰の関節角へ戻す．
"""

import copy
import math
import os
import unittest
from unittest.mock import patch

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from sensor_msgs.msg import JointState

from erasers_g1_api.robot_control import (
    ArmControl, ArmGrasp, _object_reference_pose, _pose_matrix,
)


@unittest.skipUnless(os.getenv('G1_RUN_ARM_GRASP_SIMULATION') == '1', '明示的なシミュレーション検証専用')
class TestArmGraspSimulation(unittest.TestCase):
    def setUp(self):
        print(f'python_pid={os.getpid()} stop_command=docker_stop_agy', flush=True)
        rclpy.init()
        self.node = Node('codex_arm_grasp_simulation')
        self.arm = ArmControl(self.node, use_sim_time=True, timeout_sec=10.0)
        self.collision = self.arm.collision
        self.grasp = ArmGrasp(self.arm)
        self.ids = []
        self.initial_joints = {}
        self.initial_acm = None
        self.addCleanup(self.cleanup)
        client = self.node.create_client(GetParameters, '/move_group/get_parameters')
        self.assertTrue(client.wait_for_service(timeout_sec=5.0))
        future = client.call_async(GetParameters.Request(names=['use_sim_time']))
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        self.assertTrue(future.done() and future.result().values[0].bool_value)
        joints = self.arm.get_current_joint_pose()
        controlled_joints = {'waist_yaw_joint'} | {
            f'{side}_{joint}_joint' for side in ('left', 'right')
            for joint in ('shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow', 'wrist_roll')
        }
        self.initial_joints = {
            name: value for name, value in joints.items()
            if name in controlled_joints
        }
        self.assertEqual(len(self.initial_joints), 11)
        print(f'initial_joints={self.initial_joints}', flush=True)
        self.initial_acm = self.collision.get_allowed_collision_matrix()
        self.assertIsNotNone(self.initial_acm)

    def cleanup(self):
        failures = []

        def check(operation, description):
            try:
                if not operation():
                    failures.append(description)
            except Exception as exc:
                failures.append(f'{description}: {exc}')

        try:
            for name in self.ids:
                if self.collision.get_attached_object(name) is not None:
                    check(lambda: self.collision.detach(name), f'{name} のデタッチ')
                if self.collision.get_object(name) is not None:
                    check(lambda: self.collision.remove_collision(name), f'{name} の除去')
            if self.initial_acm is not None:
                check(lambda: self.collision.apply_allowed_collision_matrix(self.initial_acm),
                      '接触許可の復元')
            if self.initial_joints:
                check(lambda: self.arm.joint_control(
                    planning_group='arm_both_with_waist', **self.initial_joints), '開始姿勢への復帰')
            for name in self.ids:
                check(lambda: self.collision.get_object(name) is None, f'{name} の残存確認')
                check(lambda: self.collision.get_attached_object(name) is None,
                      f'{name} の結合残存確認')
            if not failures:
                print('cleanup: 検証用物体の除去と開始時の関節姿勢への復帰を確認', flush=True)
        finally:
            self.node.destroy_node()
            rclpy.shutdown()
        self.assertEqual(failures, [])

    def object_position(self, name, attached=False):
        # 最新の関節メッセージまでコールバックを処理し，移動前の TF を避ける．
        self.assertTrue(self.arm.get_current_joint_pose())
        if attached:
            aco = self.collision.get_attached_object(name)
            self.assertIsNotNone(aco)
            obj = aco.object
        else:
            obj = self.collision.get_object(name)
            self.assertIsNotNone(obj)
        pose = self.arm.transform_pose(_object_reference_pose(obj), obj.header.frame_id, 'pelvis')
        self.assertIsNotNone(pose)
        return np.array([pose.position.x, pose.position.y, pose.position.z])

    def test_both_arms_grasp_lift_and_place(self):
        for side in ('left', 'right'):
            with self.subTest(side=side):
                name = f'codex_grasp_{os.getpid()}_{side}'
                self.assertIsNone(self.collision.get_object(name))
                self.ids.append(name)
                self.assertTrue(self.arm.move_groupstate())
                # 片腕の動作で変わる腰・手先姿勢に依存しない，固定座標の検証対象．
                pose = Pose()
                pose.orientation.w = 1.
                pose.position.x, pose.position.z = 0.35, 0.18
                pose.position.y = 0.18 if side == 'left' else -0.18
                pose = self.arm.transform_pose(pose, 'base_link', 'pelvis')
                self.assertIsNotNone(pose)
                position = _pose_matrix(pose)[:3, 3]
                self.assertTrue(self.collision.add_cylinder(
                    name, ref_frame='pelvis', x=float(position[0]), y=float(position[1]),
                    z=float(position[2]), radius=0.015, height=0.04))
                original_attach = self.collision.attach
                original_execute = self.arm._execute_checked_trajectory
                attachment_position = []
                execution_count = 0

                def checked_execute(trajectory):
                    nonlocal execution_count
                    execution_count += 1
                    if execution_count == 1:
                        return original_execute(trajectory)
                    baseline = self.collision.get_robot_state()
                    start = _pose_matrix(self.arm._fk_pose(baseline, f'{side}_amazing_hand'))
                    recorded = []

                    def record(message):
                        values = dict(zip(message.name, message.position))
                        snapshot = copy.deepcopy(baseline)
                        snapshot.joint_state.position = [
                            values.get(name, value) for name, value in zip(
                                baseline.joint_state.name, baseline.joint_state.position)]
                        recorded.append(snapshot)

                    subscription = self.node.create_subscription(
                        JointState, '/joint_states', record, 10)
                    try:
                        ok = original_execute(trajectory)
                    finally:
                        self.node.destroy_subscription(subscription)
                    self.assertTrue(ok)
                    self.assertGreater(len(recorded), 2)
                    matrices = [_pose_matrix(self.arm._fk_pose(s, f'{side}_amazing_hand'))
                                for s in recorded]
                    axis_angles, motion_angles = [], []
                    for actual in matrices:
                        axis_angles.append(math.acos(float(np.clip(
                            actual[:3, 1] @ start[:3, 1], -1.0, 1.0))))
                    for first, last in zip(matrices, matrices[1:]):
                        step = last[:3, 3] - first[:3, 3]
                        length = np.linalg.norm(step)
                        if length > 1e-5:
                            motion_angles.append(math.acos(float(np.clip(
                                step @ first[:3, 1] / length, -1.0, 1.0))))
                    self.assertTrue(motion_angles)
                    self.assertLessEqual(max(axis_angles), 0.1)
                    self.assertLessEqual(max(motion_angles), 0.1)
                    selected = self.grasp.last_grasp_plan
                    direction = np.array(selected['direction_pelvis'])
                    self.assertTrue(all(float(m[:3, 1] @ direction) >= math.cos(0.1)
                                        for m in matrices))
                    print(f'{side}: +Y 実測軸誤差={math.degrees(max(axis_angles)):.3f}deg '
                          f'進行方向誤差={math.degrees(max(motion_angles)):.3f}deg '
                          f'観測数={len(recorded)}', flush=True)
                    print(f'{side}: selected={selected}', flush=True)
                    return ok

                def checked_attach(*args, **kwargs):
                    before = self.object_position(name)
                    state = self.collision.get_robot_state()
                    ee = _pose_matrix(self.arm._fk_pose(state, f'{side}_amazing_hand'))
                    local = ee[:3, :3].T @ (before - ee[:3, 3])
                    self.assertGreater(local[1], 0.0)
                    self.assertLessEqual(math.atan2(np.linalg.norm(local[[0, 2]]), local[1]), 0.1)
                    ok = original_attach(*args, **kwargs)
                    if ok:
                        after = self.object_position(name, attached=True)
                        attachment_position.append(after)
                        self.assertLess(float(np.linalg.norm(after - before)), 0.002)
                        self.assertIsNone(self.collision.get_object(name))
                    return ok

                with patch.object(self.collision, 'attach', side_effect=checked_attach), \
                        patch.object(self.arm, '_execute_checked_trajectory',
                                     side_effect=checked_execute):
                    self.assertTrue(self.grasp.grasp(
                        name, arm_side=side, pre_offset_dist=0.025, lift_height=0.04))
                self.assertEqual(len(attachment_position), 1)
                after_lift = self.object_position(name, attached=True)
                lift_delta = after_lift - attachment_position[0]
                self.assertGreater(lift_delta[2], 0.02)
                # 位置優先 IK による手先回転を含む物体変位を計測して記録する．
                print(f'{side}: attachment_jump<2mm, '
                      f'object_lift_delta={lift_delta.tolist()}', flush=True)
                self.assertEqual(self.collision.get_allowed_collision_matrix(), self.initial_acm)

                held_target = position + [0.0, 0.0, 0.02]
                self.assertTrue(self.grasp.place(
                    *map(float, held_target), ref='pelvis', arm_side=side,
                    object=name, detach=False))
                self.assertIsNotNone(self.collision.get_attached_object(name))
                held_error = float(np.linalg.norm(
                    self.object_position(name, attached=True) - held_target))
                self.assertLessEqual(held_error, ArmGrasp.POSITION_TOLERANCE)

                self.assertTrue(self.grasp.place(
                    *map(float, position), ref='pelvis', arm_side=side, object=name))
                self.assertIsNone(self.collision.get_attached_object(name))
                placed_error = float(np.linalg.norm(self.object_position(name) - position))
                self.assertLessEqual(placed_error, ArmGrasp.POSITION_TOLERANCE)
                print(f'{side}: held_error={held_error:.6f}m '
                      f'placed_error={placed_error:.6f}m', flush=True)
                self.assertTrue(self.collision.remove_collision(name))


@unittest.skipUnless(os.getenv('G1_RUN_ARM_GRASP_SIMULATION') == '1', '明示的なシミュレーション検証専用')
class TestSampleGraspDirections(unittest.TestCase):
    """サンプルを左右三方向で実行し，実際の関節状態から接近方向を計測する．"""

    def test_auto_with_obstacles(self):
        from samples.manipulation.sample_object_grasp import main

        print(f"obstacles_python_pid={os.getpid()}", flush=True)
        original = ArmGrasp.grasp
        for blocked in ('none', 'side', 'all'):
            with self.subTest(blocked=blocked):
                observed = []

                def checked_grasp(grasp, *args, **kwargs):
                    arm = grasp._ArmGrasp__arm
                    collision = arm.collision
                    before = collision.get_robot_state()
                    acm = collision.get_allowed_collision_matrix()
                    with patch.object(arm, '_execute_checked_trajectory',
                                      wraps=arm._execute_checked_trajectory) as execute:
                        result = original(grasp, *args, **kwargs)
                    if blocked == 'all':
                        self.assertFalse(result)
                        execute.assert_not_called()
                        self.assertEqual(list(before.joint_state.position), list(
                            collision.get_robot_state().joint_state.position))
                        self.assertIsNone(grasp.last_grasp_plan)
                    else:
                        self.assertTrue(result)
                        selected = grasp.last_grasp_plan
                        self.assertNotEqual(selected['approach_type'], blocked)
                        self.assertGreater(selected['offset_dist'], 0.)
                        print(f'auto/{blocked}: selected={selected}', flush=True)
                    self.assertEqual(collision.get_allowed_collision_matrix(), acm)
                    observed.append(result)
                    return result

                args = ['--ros-args', '-p', 'approach_type:=auto',
                        '-p', 'object_size_z:=0.16', '-p', f'block_approach:={blocked}']
                if blocked == 'all':
                    args += ['-p', 'expect_failure:=true']
                with patch.object(ArmGrasp, 'grasp', new=checked_grasp):
                    self.assertEqual(main(args=args), 0)
                self.assertEqual(observed, [blocked != 'all'])

    def test_six_directions(self):
        print(f"six_directions_python_pid={os.getpid()}", flush=True)
        from samples.manipulation.sample_object_grasp import main

        original = ArmControl._execute_checked_trajectory
        for side in ('left', 'right'):
            for mode in ('side', 'front', 'top'):
                with self.subTest(side=side, mode=mode):
                    execution_count = 0
                    checked = []
                    direction = {'side': [0., -1. if side == 'left' else 1., 0.],
                                 'front': [1., 0., 0.], 'top': [0., 0., -1.]}[mode]

                    def execute(arm, trajectory):
                        nonlocal execution_count
                        execution_count += 1
                        if execution_count == 1:
                            return original(arm, trajectory)
                        node = arm._ArmControl__node
                        baseline = arm.collision.get_robot_state()
                        recorded = []

                        def record(message):
                            values = dict(zip(message.name, message.position))
                            snapshot = copy.deepcopy(baseline)
                            snapshot.joint_state.position = [
                                values.get(n, q) for n, q in zip(
                                    baseline.joint_state.name, baseline.joint_state.position)]
                            recorded.append(snapshot)

                        subscription = node.create_subscription(
                            JointState, '/joint_states', record, 10)
                        try:
                            result = original(arm, trajectory)
                        finally:
                            node.destroy_subscription(subscription)
                        self.assertTrue(result)
                        poses = [_pose_matrix(arm._fk_pose(s, f'{side}_amazing_hand'))
                                 for s in recorded]
                        self.assertGreater(len(poses), 2)
                        axis_errors = [math.acos(float(np.clip(
                            p[:3, 1] @ direction, -1., 1.))) for p in poses]
                        motion_errors = []
                        for first, last in zip(poses, poses[1:]):
                            delta = last[:3, 3] - first[:3, 3]
                            if np.linalg.norm(delta) > 1e-5:
                                motion_errors.append(math.acos(float(np.clip(
                                    delta @ direction / np.linalg.norm(delta), -1., 1.))))
                        self.assertTrue(motion_errors)
                        self.assertLessEqual(max(axis_errors), .1)
                        self.assertLessEqual(max(motion_errors), .1)
                        checked.append(True)
                        print(f'{side}/{mode}: actual_axis_error='
                              f'{math.degrees(max(axis_errors)):.3f}deg actual_motion_error='
                              f'{math.degrees(max(motion_errors)):.3f}deg samples={len(poses)}',
                              flush=True)
                        return result

                    args = ['--ros-args', '-p', f'approach_type:={mode}',
                            '-p', f'arm_side:={side}',
                            '-p', 'object_y:=0.18' if side == 'left' else 'object_y:=-0.18',
                            '-p', ('object_size_z:=0.08' if mode == 'top'
                                   else 'object_size_z:=0.16')]
                    with patch.object(ArmControl, '_execute_checked_trajectory', new=execute):
                        self.assertEqual(main(args=args), 0)
                    self.assertEqual(checked, [True])


if __name__ == '__main__':
    unittest.main()
