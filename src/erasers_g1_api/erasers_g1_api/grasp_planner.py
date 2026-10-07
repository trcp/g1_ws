# Copyright (c) 2026 Nakatogawa Laboratory. All rights reserved.

"""把持方向・距離の候補を，実行中の MoveIt シーンで移動前に評価する．"""

import copy
from dataclasses import dataclass
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes, OrientationConstraint
from moveit_msgs.srv import GetCartesianPath, GetMotionPlan, GetStateValidity
from rcl_interfaces.srv import GetParameters

from .grasp_geometry import HandChain, distance_candidates, object_near_extent
from .robot_control import (
    _matrix_pose, _object_reference_pose, _pose_matrix, _positive_y_path_valid,
    _sample_joint_trajectory,
)


@dataclass
class GraspPlan:
    """評価済みの接近前軌道と最終接近軌道．方向は pelvis 基準．"""

    approach_type: str
    offset: float
    pre_offset: float
    direction: np.ndarray
    center: np.ndarray
    initial_state: object
    pre_state: object
    pre_trajectory: object
    approach_trajectory: object
    score: float


class GraspPlanner:
    """方向候補を独立に評価し，成立する候補から関節移動の小さいものを選ぶ．"""

    def __init__(self, arm, collision):
        self.arm, self.collision = arm, collision
        self.node = arm._ArmControl__node
        self.timeout = arm._ArmControl__timeout_sec
        self.logger = self.node.get_logger()
        self.clients = []
        self.parameters = self.client(GetParameters, '/move_group/get_parameters')
        self.validity = self.client(GetStateValidity, '/check_state_validity')
        self.planning = self.client(GetMotionPlan, '/plan_kinematic_path')
        self.cartesian = self.client(GetCartesianPath, '/compute_cartesian_path')
        self.last_rejection = ''
        self.group = ''

    def __enter__(self):
        return self

    def __exit__(self, *args):
        for client in self.clients:
            self.node.destroy_client(client)

    def client(self, message, name):
        client = self.node.create_client(message, name,
                                         callback_group=self.arm._ArmControl__cb_group)
        self.clients.append(client)
        return client

    def call(self, client, request):
        if not client.wait_for_service(timeout_sec=self.timeout):
            raise RuntimeError(f'サービスが利用できません: {client.srv_name}')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=self.timeout)
        if not future.done():
            future.cancel()
            raise RuntimeError(f'サービスがタイムアウトしました: {client.srv_name}')
        result = future.result()
        if result is None:
            raise RuntimeError(f'サービス応答がありません: {client.srv_name}')
        return result

    @staticmethod
    def state_at(state, names, positions):
        result = copy.deepcopy(state)
        values = dict(zip(names, positions))
        if not set(values).issubset(state.joint_state.name):
            raise ValueError('モデル外の関節を含む把持候補です．')
        result.joint_state.position = [values.get(n, q) for n, q in zip(
            state.joint_state.name, state.joint_state.position)]
        result.is_diff = False
        return result

    def valid(self, state, target=None, touch_links=()):
        # シーンを変更せず，対象と指定ハンドの接触だけを候補評価時に許可する．
        response = self.call(self.validity, GetStateValidity.Request(
            robot_state=state, group_name=self.group))
        if response.valid:
            return True
        if (not response.contacts or any(not c.result for c in response.constraint_result)):
            return False
        allowed = {frozenset((target, link)) for link in touch_links}
        return all(frozenset((c.contact_body_1, c.contact_body_2)) in allowed
                   for c in response.contacts)

    def plan(self, obj, mode, side, group, offset, pre_offset, ref):
        """三方向と距離候補を評価する．このメソッドはロボットを動かさない．"""
        self.last_rejection = ''
        self.group = group
        response = self.call(self.parameters, GetParameters.Request(names=['robot_description']))
        urdf = response.values[0].string_value
        state = self.collision.get_robot_state()
        if state is None:
            return None
        fixed = dict(zip(state.joint_state.name, state.joint_state.position))
        group_joints = set()
        for values in self.arm._ArmControl__srdf_states.get(group, {}).values():
            group_joints.update(values)
        names = [n for n in fixed if n in group_joints]
        if not names or set(names) != group_joints:
            raise ValueError('計画グループの関節が MoveIt の状態と一致しません．')
        chain = HandChain(urdf, f'{side}_amazing_hand', names)
        current = np.array([fixed[n] for n in names])
        actual = self.arm._fk_pose(state, f'{side}_amazing_hand')
        if actual is None or not np.allclose(
                chain.forward(current, fixed), _pose_matrix(actual), atol=1e-6):
            raise ValueError('把持計算のモデルと MoveIt の FK が一致しません．')
        if not self.valid(state):
            raise ValueError('開始状態が衝突しています．把持候補は実行しません．')
        reference = Pose()
        reference.orientation.w = 1.0
        reference = self.arm.transform_pose(reference, ref, 'pelvis')
        object_pose = self.arm.transform_pose(obj.pose, obj.header.frame_id, 'pelvis')
        center_pose = self.arm.transform_pose(
            _object_reference_pose(obj), obj.header.frame_id, 'pelvis')
        if reference is None or object_pose is None or center_pose is None:
            return None
        frame = _pose_matrix(reference)
        object_matrix = _pose_matrix(object_pose)
        center = _pose_matrix(center_pose)[:3, 3]
        directions = {'side': np.array([0., -1. if side == 'left' else 1., 0.]),
                      'front': np.array([1., 0., 0.]), 'top': np.array([0., 0., -1.])}
        modes = list(directions) if mode == 'auto' else [mode]
        touch_links = [f'{side}_amazing_hand', f'{side}_wrist_roll_rubber_hand']
        feasible = []
        for approach in modes:
            self.last_rejection = ''
            direction = frame[:3, :3] @ directions[approach]
            extent = (object_near_extent(obj, object_matrix, center, direction, _pose_matrix)
                      if offset is None else 0.)
            proposals = []
            for distance, pre_distance in distance_candidates(
                    extent, chain.hand_reach, offset, pre_offset):
                position = center - direction * (distance + pre_distance)
                solutions = []
                for seed in chain.seeds(current):
                    q = chain.solve(position, direction, seed, fixed)
                    if q is None or any(np.linalg.norm(q - old) < 0.1 for old in solutions):
                        continue
                    solutions.append(q)
                for q in solutions:
                    # 軸方向の余裕を持つ距離を優先し，同等なら移動が小さい姿勢を選ぶ．
                    travel = np.linalg.norm(q - current)
                    margin = np.min(np.minimum(q - chain.lower, chain.upper - q))
                    score = travel + 2. * pre_distance + 0.02 / max(margin, 0.01)
                    proposals.append((score, distance, pre_distance, q))
            proposals.sort(key=lambda item: item[0])
            self.logger.info(f'把持候補 {approach}: 関節限界内の姿勢 {len(proposals)} 件．')
            # 全候補を評価するが，同じ方向で成立した後はほかの方向の評価へ進む．
            for score, distance, pre_distance, q in proposals:
                pre_state = self.state_at(state, names, q)
                if not self.valid(pre_state):
                    self.last_rejection = '接近前姿勢の衝突'
                    continue
                pre_trajectory = self.plan_pre(state, pre_state, names, group)
                if pre_trajectory is None:
                    self.last_rejection = '接近前の衝突回避経路'
                    continue
                points = pre_trajectory.joint_trajectory
                pre_state = self.state_at(state, points.joint_names, points.points[-1].positions)
                approach_trajectory = self.plan_approach(
                    pre_state, group, side, pre_distance, direction, chain, fixed,
                    obj.id, touch_links)
                if approach_trajectory is None:
                    continue
                feasible.append(GraspPlan(
                    approach, distance, pre_distance, direction, center, state, pre_state,
                    pre_trajectory, approach_trajectory, score))
                break
            else:
                self.logger.info(f'把持候補 {approach}: 成立しません（{self.last_rejection or "姿勢解なし"}）．')
        if not feasible:
            self.logger.error('方向・距離・衝突条件を満たす把持候補がありません．')
            return None
        selected = min(feasible, key=lambda p: p.score)
        self.logger.info(
            f'把持選択: approach_type={selected.approach_type}, '
            f'offset_dist={selected.offset:.4f} m, pre_offset_dist={selected.pre_offset:.4f} m')
        return selected

    def plan_pre(self, initial, target, names, group):
        """接触を許可せず，接近前姿勢までの通常の衝突回避軌道を計画する．"""
        request = GetMotionPlan.Request()
        request.motion_plan_request = self.arm._create_move_group_goal(group).request
        request.motion_plan_request.start_state = initial
        request.motion_plan_request.allowed_planning_time = min(2., self.timeout * 0.5)
        request.motion_plan_request.num_planning_attempts = 2
        values = dict(zip(target.joint_state.name, target.joint_state.position))
        constraints = Constraints()
        constraints.joint_constraints = [JointConstraint(
            joint_name=name, position=float(values[name]), tolerance_above=0.0001,
            tolerance_below=0.0001, weight=1.) for name in names]
        request.motion_plan_request.goal_constraints = [constraints]
        response = self.call(self.planning, request).motion_plan_response
        if (response.error_code.val != MoveItErrorCodes.SUCCESS
                or not response.trajectory.joint_trajectory.points):
            return None
        return response.trajectory

    def plan_approach(self, state, group, side, distance, direction, chain, fixed,
                      target, touches):
        """直線経路を計画し，補間後の方向と動作グループの衝突を検査する．"""
        values = dict(zip(state.joint_state.name, state.joint_state.position))
        start = chain.forward([values[n] for n in chain.names], fixed)
        end = start.copy()
        end[:3, 3] += direction * distance
        request = GetCartesianPath.Request()
        request.header.frame_id = 'pelvis'
        request.start_state, request.group_name = state, group
        request.link_name = f'{side}_amazing_hand'
        request.waypoints = [_matrix_pose(end)]
        request.max_step = 0.002
        request.jump_threshold = 0.
        # 接触許可をシーンへ書き込まず，後段で動作グループの全標本を衝突判定する．
        request.avoid_collisions = False
        request.max_velocity_scaling_factor = 0.2
        request.max_acceleration_scaling_factor = 0.2
        constraint = OrientationConstraint()
        constraint.header.frame_id = 'pelvis'
        constraint.link_name = request.link_name
        constraint.orientation = _matrix_pose(start).orientation
        constraint.absolute_x_axis_tolerance = 0.1
        constraint.absolute_y_axis_tolerance = math.pi
        constraint.absolute_z_axis_tolerance = 0.1
        constraint.weight = 1.
        request.path_constraints.orientation_constraints = [constraint]
        response = self.call(self.cartesian, request)
        if (response.error_code.val != MoveItErrorCodes.SUCCESS
                or not math.isfinite(response.fraction) or response.fraction < 1. - 1e-9):
            self.last_rejection = '直線接近の部分経路'
            return None
        trajectory = response.solution.joint_trajectory
        samples = _sample_joint_trajectory(trajectory)
        if (samples is None or set(trajectory.joint_names) != set(chain.names)
                or response.solution.multi_dof_joint_trajectory.points):
            self.last_rejection = '軌道の関節・時間情報'
            return None
        if any(abs(values[n] - q) > 0.001 for n, q in zip(trajectory.joint_names, samples[0])):
            return None
        matrices, states = [], []
        for sample in samples:
            sample_values = dict(zip(trajectory.joint_names, sample))
            q = np.array([sample_values[n] for n in chain.names])
            if np.any(q < chain.lower) or np.any(q > chain.upper):
                self.last_rejection = '接近中の関節限界'
                return None
            matrices.append(chain.forward(q, fixed))
            states.append(self.state_at(state, trajectory.joint_names, sample))
        reference = start.copy()
        reference[:3, 1] = direction
        if not _positive_y_path_valid(matrices, reference, distance):
            self.last_rejection = '接近中の +Y 軸・進行方向'
            return None
        for snapshot in states:
            if not self.valid(snapshot, target, touches):
                self.last_rejection = '接近中の周辺・自己衝突'
                return None
        return response.solution

    def matches_state(self, expected):
        """計画後の開始状態の変化を検出する．"""
        current = self.collision.get_robot_state()
        if current is None:
            return False
        values = dict(zip(current.joint_state.name, current.joint_state.position))
        return all(name in values and abs(values[name] - position) < 0.0261
                   for name, position in zip(expected.joint_state.name,
                                             expected.joint_state.position))
