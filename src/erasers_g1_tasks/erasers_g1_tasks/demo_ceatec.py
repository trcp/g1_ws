"""CEATEC 第1段階：FSM 確認と上半身・環境コリジョンの初期化．"""

import argparse
import json
import math
from pathlib import Path
import sys
from typing import Any, Mapping, Sequence

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from erasers_g1_api.robot_control import ArmControl, G1Control


class DemoCEATEC:
    """既に FSM_ID=500 の実機だけを初期化する．"""

    def __init__(
        self,
        node: Node,
        obstacles: Sequence[Mapping[str, Any]],
        timeout_sec: float = 10.0,
    ) -> None:
        """環境箱の座標・寸法は呼び出し側が明示する．

        obstacles の各要素は Collision.add_box に渡す辞書とする．
        name，ref_frame，x，y，z，scale_x，scale_y，scale_z を必須とし，
        roll，pitch，yaw は省略時に 0 rad とする．位置・寸法は m．
        このクラスは FSM の遷移要求を送信しない．初期化失敗時も，
        自動復旧・姿勢復帰・登録済みコリジョンの削除は行わない．
        """
        if not math.isfinite(timeout_sec) or timeout_sec <= 0:
            raise ValueError('timeout_sec は正の有限値を指定してください．')
        self.node = node
        self.initialized = False
        self.robot = G1Control(node, timeout_sec=timeout_sec)

        def require_start_state() -> None:
            fsm_id = self.robot.get_current_robot_pose()
            if fsm_id != 500:
                raise RuntimeError(
                    f'FSM_ID={fsm_id} のため初期化を拒否しました．FSM は変更しません．')
            self.node.get_logger().info('FSM_ID=500 を確認しました．')

        require_start_state()
        required = {
            'name', 'ref_frame', 'x', 'y', 'z',
            'scale_x', 'scale_y', 'scale_z',
        }
        optional = {'roll', 'pitch', 'yaw'}
        boxes = [dict(spec) for spec in obstacles]
        if not boxes:
            raise ValueError('obstacles_* の環境コリジョン定義が必要です．')
        names = set()
        for box in boxes:
            if required - box.keys() or box.keys() - (required | optional):
                raise ValueError('環境コリジョンの必須項目または未対応項目を確認してください．')
            name = box['name']
            if (not isinstance(name, str) or not name.startswith('obstacles_')
                    or name == 'obstacles_' or name in names):
                raise ValueError('環境コリジョン名は一意な obstacles_* にしてください．')
            names.add(name)
            if not isinstance(box['ref_frame'], str) or not box['ref_frame'].strip():
                raise ValueError('環境コリジョンの基準座標系を指定してください．')
            for key in ('x', 'y', 'z', 'roll', 'pitch', 'yaw',
                        'scale_x', 'scale_y', 'scale_z'):
                box[key] = float(box.get(key, 0.0))
                if not math.isfinite(box[key]):
                    raise ValueError(f'{name}: {key} は有限値を指定してください．')
            if min(box['scale_x'], box['scale_y'], box['scale_z']) <= 0:
                raise ValueError(f'{name}: 箱の各寸法は正の値を指定してください．')

        self.arm = ArmControl(node, timeout_sec=timeout_sec)
        self.collision = self.arm.collision
        self.obstacle_names = []

        require_start_state()
        if not self.arm.upper_body_control(True, recover_fault=False):
            raise RuntimeError('上半身制御の有効化に失敗しました．')
        self.node.get_logger().info('上半身制御を有効化しました．')

        require_start_state()
        if not self.arm.move_groupstate(
                group_name='arm_both_with_waist', state='walk', wait=True):
            raise RuntimeError('上半身の walk 姿勢への移行に失敗しました．')
        self.node.get_logger().info('arm_both_with_waist の walk 遷移が完了しました．')

        for box in boxes:
            if not self.collision.add_box(**box):
                raise RuntimeError(
                    f"環境コリジョンの登録に失敗しました: {box['name']}")
            self.obstacle_names.append(box['name'])
            self.node.get_logger().info(f"環境コリジョンを登録しました: {box['name']}")

        self.initialized = True
        self.node.get_logger().info(
            f'第1段階の初期化が完了しました: {self.obstacle_names}')


def main(args=None):
    """実測した環境箱の JSON 配列を読み込み，第1段階だけを実行する．"""
    argv = sys.argv if args is None else [sys.argv[0], *args]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--obstacles-file', type=Path, required=True,
        help='実測した環境箱の JSON 配列．各要素の name は obstacles_*')
    parser.add_argument(
        '--timeout', type=float, default=10.0, help='API の応答待機時間（秒）')
    options = parser.parse_args(remove_ros_args(argv)[1:])
    if not math.isfinite(options.timeout) or options.timeout <= 0:
        parser.error('--timeout は正の有限値を指定してください．')
    try:
        with options.obstacles_file.open(encoding='utf-8') as stream:
            obstacles = json.load(stream)
        if (not isinstance(obstacles, list) or not obstacles
                or any(not isinstance(box, dict) for box in obstacles)):
            raise ValueError('環境設定は，空でない辞書の JSON 配列で指定してください．')
    except (OSError, ValueError) as exc:
        parser.error(f'環境設定を読み込めません: {exc}')

    rclpy.init(args=argv)
    node = None
    try:
        node = Node('demo_ceatec')
        DemoCEATEC(node, obstacles, timeout_sec=options.timeout)
        return 0
    except (Exception, KeyboardInterrupt) as exc:
        if node is not None:
            node.get_logger().error(f'第1段階を中断しました: {exc}')
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
