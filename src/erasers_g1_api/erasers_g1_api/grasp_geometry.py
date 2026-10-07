# Copyright (c) 2026 Nakatogawa Laboratory. All rights reserved.

"""把持候補の幾何計算．ロボットの移動や PlanningScene の変更は行わない．"""

import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np
from ament_index_python.packages import get_package_share_directory


def rotation(axis, angle):
    """単位軸の回転行列を Rodrigues の式で求める．"""
    x, y, z = axis
    skew = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    return np.eye(3) + math.sin(angle) * skew + (1. - math.cos(angle)) * (skew @ skew)


def origin_matrix(element):
    """URDF の origin を同次変換へ変換する．"""
    matrix = np.eye(4)
    if element is not None:
        matrix[:3, 3] = np.fromstring(element.get('xyz', '0 0 0'), sep=' ')
        roll, pitch, yaw = np.fromstring(element.get('rpy', '0 0 0'), sep=' ')
        matrix[:3, :3] = (rotation([0., 0., 1.], yaw)
                          @ rotation([0., 1., 0.], pitch) @ rotation([1., 0., 0.], roll))
    return matrix


class HandChain:
    """実行中の URDF による手先 FK・ヤコビアンと，関節限界付き候補探索．"""

    def __init__(self, urdf, link, joint_names):
        root = ET.fromstring(urdf)
        by_child = {j.find('child').get('link'): j for j in root.findall('joint')}
        joints = []
        current = link
        while current != 'pelvis':
            joint = by_child[current]
            joints.append(joint)
            current = joint.find('parent').get('link')
        self.names = list(joint_names)
        self.lower = np.empty(len(self.names))
        self.upper = np.empty(len(self.names))
        self.chain = []
        found = set()
        for joint in reversed(joints):
            name, kind = joint.get('name'), joint.get('type')
            axis_element = joint.find('axis')
            axis = np.fromstring(axis_element.get('xyz') if axis_element is not None
                                 else '1 0 0', sep=' ')
            axis /= np.linalg.norm(axis)
            index = self.names.index(name) if name in self.names else None
            if index is not None:
                if kind != 'revolute' or joint.find('mimic') is not None:
                    raise ValueError(f'把持計算で未対応の関節です: {name}')
                limits = joint.find('limit')
                self.lower[index] = float(limits.get('lower')) + 1e-5
                self.upper[index] = float(limits.get('upper')) - 1e-5
                found.add(name)
            elif kind not in ('fixed', 'revolute'):
                raise ValueError(f'把持計算で未対応の関節です: {name}')
            self.chain.append((name, kind, origin_matrix(joint.find('origin')), axis, index))
        if found != set(self.names):
            raise ValueError('計画グループと手先の関節列が一致しません．')
        self.hand_reach = self._hand_reach(root, by_child[link])

    @staticmethod
    def _hand_reach(root, tip_joint):
        """手先の +Y 側にあるハンド形状の端を取得する．STL と基本形状に対応する．"""
        parent = tip_joint.find('parent').get('link')
        link = root.find(f"link[@name='{parent}']")
        to_tip = np.linalg.inv(origin_matrix(tip_joint.find('origin')))
        reaches = []
        for collision in link.findall('collision'):
            transform = to_tip @ origin_matrix(collision.find('origin'))
            geometry = collision.find('geometry')
            mesh = geometry.find('mesh')
            if mesh is not None:
                filename = mesh.get('filename')
                if filename.startswith('package://'):
                    package, relative = filename[len('package://'):].split('/', 1)
                    path = Path(get_package_share_directory(package)) / relative
                else:
                    path = Path(filename.removeprefix('file://'))
                data = path.read_bytes()
                count = struct.unpack('<I', data[80:84])[0] if len(data) >= 84 else 0
                if len(data) == 84 + 50 * count:
                    dtype = np.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)),
                                      ('attribute', '<u2')])
                    vertices = np.frombuffer(data[84:], dtype=dtype)['vertices'].reshape(-1, 3)
                else:
                    vertices = np.array([list(map(float, line.split()[1:]))
                                         for line in data.decode('ascii').splitlines()
                                         if line.strip().startswith('vertex ')])
                vertices = vertices * np.fromstring(mesh.get('scale', '1 1 1'), sep=' ')
                reaches.append(float(np.max(vertices @ transform[1, :3]) + transform[1, 3]))
            elif geometry.find('box') is not None:
                size = np.fromstring(geometry.find('box').get('size'), sep=' ')
                reaches.append(float(transform[1, 3] + np.abs(transform[1, :3]) @ size / 2))
            elif geometry.find('sphere') is not None:
                reaches.append(transform[1, 3] + float(geometry.find('sphere').get('radius')))
            elif geometry.find('cylinder') is not None:
                shape = geometry.find('cylinder')
                reaches.append(transform[1, 3]
                               + float(shape.get('radius')) * np.linalg.norm(transform[1, :2])
                               + float(shape.get('length')) / 2 * abs(transform[1, 2]))
            else:
                raise ValueError('ハンドのコリジョン形状を解釈できません．')
        if not reaches or not np.isfinite(reaches).all():
            raise ValueError('ハンドのコリジョン寸法を取得できません．')
        return max(0., max(reaches))

    def forward(self, positions, fixed, jacobian=False):
        """pelvis 基準の姿勢と，位置・+Y 軸のヤコビアンを返す．"""
        matrix = np.eye(4)
        axes = []
        for name, kind, origin, axis, index in self.chain:
            matrix = matrix @ origin
            if kind != 'fixed':
                if index is not None:
                    axes.append((index, matrix[:3, :3] @ axis, matrix[:3, 3].copy()))
                angle = positions[index] if index is not None else fixed[name]
                matrix[:3, :3] = matrix[:3, :3] @ rotation(axis, angle)
        if not jacobian:
            return matrix
        derivative = np.zeros((6, len(self.names)))
        for index, axis, origin in axes:
            derivative[:3, index] = np.cross(axis, matrix[:3, 3] - origin)
            derivative[3:, index] = np.cross(axis, matrix[:3, 1]) * 0.15
        return matrix, derivative

    def solve(self, position, direction, seed, fixed):
        """位置と +Y 軸の方向を同時に解く．軸回りの回転は拘束しない．"""
        q = np.clip(np.array(seed, dtype=float), self.lower, self.upper)
        for _ in range(100):
            matrix, jacobian = self.forward(q, fixed, jacobian=True)
            error = np.r_[position - matrix[:3, 3], (direction - matrix[:3, 1]) * 0.15]
            if np.linalg.norm(error[:3]) < 0.0002 and np.linalg.norm(error[3:]) < 0.0003:
                return q
            step = jacobian.T @ np.linalg.solve(
                jacobian @ jacobian.T + np.eye(6) * 1e-5, error)
            scale = min(1., 0.25 / max(1e-9, np.max(np.abs(step))))
            improved = False
            for factor in (scale, scale / 2, scale / 4, scale / 8):
                trial = np.clip(q + step * factor, self.lower, self.upper)
                pose = self.forward(trial, fixed)
                residual = np.r_[position - pose[:3, 3], (direction - pose[:3, 1]) * 0.15]
                if np.linalg.norm(residual) < np.linalg.norm(error) - 1e-9:
                    q, improved = trial, True
                    break
            if not improved:
                return None
        return None

    def seeds(self, current):
        """現在姿勢，中央姿勢，再現可能な異なる姿勢から候補を探索する．"""
        yield np.array(current)
        yield np.clip(np.zeros(len(self.names)), self.lower, self.upper)
        generator = np.random.default_rng(7)
        for _ in range(8):
            seed = generator.uniform(self.lower * 0.8, self.upper * 0.8)
            yield seed
            # X/Z 軸関節の相補的な姿勢も調べ，左右の探索が一方の手首姿勢へ偏るのを防ぐ．
            reflected = seed.copy()
            for _, _, _, axis, index in self.chain:
                if index is not None and abs(axis[1]) < 0.5:
                    reflected[index] = self.lower[index] + self.upper[index] - seed[index]
            yield reflected


def object_near_extent(obj, object_matrix, center, direction, pose_matrix):
    """物体基準点から接近側の形状端までの距離を求める．複合形状も含める．"""
    extents = []
    if len(obj.primitives) != len(obj.primitive_poses) or len(obj.meshes) != len(obj.mesh_poses):
        raise ValueError('物体形状と姿勢の個数が一致しません．')
    for shape, pose in zip(obj.primitives, obj.primitive_poses):
        matrix = object_matrix @ pose_matrix(pose)
        axis = matrix[:3, :3].T @ direction
        dimensions = np.array(shape.dimensions)
        if not np.isfinite(dimensions).all() or np.any(dimensions <= 0):
            raise ValueError('物体寸法が不正です．')
        if shape.type == shape.BOX and len(dimensions) == 3:
            extent = np.abs(axis) @ dimensions / 2
        elif shape.type == shape.SPHERE and len(dimensions) == 1:
            extent = dimensions[0]
        elif shape.type == shape.CYLINDER and len(dimensions) == 2:
            extent = dimensions[0] / 2 * abs(axis[2]) + dimensions[1] * np.linalg.norm(axis[:2])
        elif shape.type == shape.CONE and len(dimensions) == 2:
            extent = max(-dimensions[0] / 2 * axis[2],
                         dimensions[0] / 2 * axis[2] + dimensions[1] * np.linalg.norm(axis[:2]))
        else:
            raise ValueError('自動距離計算で未対応の形状です．距離を明示してください．')
        extents.append(float((center - matrix[:3, 3]) @ direction + extent))
    for mesh, pose in zip(obj.meshes, obj.mesh_poses):
        matrix = object_matrix @ pose_matrix(pose)
        vertices = np.array([[v.x, v.y, v.z] for v in mesh.vertices])
        if not len(vertices) or not np.isfinite(vertices).all():
            raise ValueError('物体メッシュが不正です．')
        extents.append(float((center - matrix[:3, 3]) @ direction
                             - np.min(vertices @ (matrix[:3, :3].T @ direction))))
    if obj.planes or not extents:
        raise ValueError('有限な物体形状が必要です．距離を明示してください．')
    return max(0., max(extents))


def distance_candidates(extent, hand_reach, offset, pre_offset):
    """形状表面付近の把持距離と，短距離からの接近候補を返す．"""
    offsets = ([float(offset)] if offset is not None else
               [max(0.005, extent + hand_reach - inset) for inset in (0.005, 0.015, 0.025)])
    approaches = [float(pre_offset)] if pre_offset is not None else [0.025, 0.05, 0.10]
    return [(o, p) for o in dict.fromkeys(offsets) for p in approaches]
