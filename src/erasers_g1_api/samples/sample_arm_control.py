#!/usr/bin/env python3
"""シミュレーションで物体の把持・持ち上げ・元の位置への配置を行う．

use_sim_time の既定値は True．実機では ROS 引数で False を指定する．
PlanningScene の結合を扱うサンプルであり，物理的な保持力の検出は行わない．
"""

from geometry_msgs.msg import Pose
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from erasers_g1_api.robot_control import ArmControl, ArmGrasp


def main():
    """各操作の成否を確認し，作成した物体だけを後処理する．"""
    rclpy.init()
    node = Node('sample_arm_control', parameter_overrides=[
        Parameter('use_sim_time', value=True),
    ])
    collision = None
    owned = []

    def require(ok, message):
        if not ok:
            raise RuntimeError(message)

    try:
        arm = ArmControl(node)
        collision = arm.collision
        grasp = ArmGrasp(arm)
        table_name, object_name = 'sample_arm_table', 'sample_arm_cylinder'
        for name in (table_name, object_name):
            require(collision.get_object(name) is None
                    and collision.get_attached_object(name) is None,
                    f'同名の物体が存在します: {name}')

        require(arm.upper_body_control(True), '上半身制御を有効化できません．')
        require(arm.move_groupstate(state='home'), '開始姿勢へ移動できません．')

        owned.append(table_name)
        require(collision.add_box(
            name=table_name, ref_frame='torso_link',
            x=0.42, y=0.18, z=-0.22, scale_x=0.25, scale_y=0.30, scale_z=0.04,
        ), '作業台を登録できません．')
        owned.append(object_name)
        require(collision.add_cylinder(
            name=object_name, ref_frame='torso_link',
            x=0.42, y=0.18, z=-0.05, radius=0.02, height=0.15,
        ), '把持対象を登録できません．')

        # 腰を動かす前の物体位置を保存し，同じワールド位置へ戻す．
        registered = collision.get_object_pose(object_name)
        require(registered is not None, '物体位置を取得できません．')
        target = Pose()
        target.position.x, target.position.y, target.position.z = registered[:3]
        target.orientation.w = 1.0
        target = arm.transform_pose(target, registered[6], 'pelvis')
        require(target is not None, '配置先の座標変換に失敗しました．')

        # 円柱と手首の接近前の干渉を避け，+Y 軸を保つ短い直線接近を指定する．
        require(grasp.grasp(object_name, offset_dist=0.04, pre_offset_dist=0.025, lift=True),
                '把持または持ち上げに失敗しました．')
        require(grasp.place(
            target.position.x, target.position.y, target.position.z,
            ref='pelvis', object=object_name,
        ), '配置に失敗しました．')
        for name in reversed(owned):
            require(collision.remove_collision(name), f'物体を除去できません: {name}')
        owned.clear()
        require(arm.move_groupstate(state='walk'), '歩行姿勢へ復帰できません．')
        require(arm.upper_body_control(False), '上半身制御を解放できません．')
        node.get_logger().info('把持・持ち上げ・配置が完了しました．')
        return 0
    except Exception as exc:
        node.get_logger().error(str(exc))
        return 1
    finally:
        if collision is not None:
            for name in reversed(owned):
                try:
                    if collision.get_attached_object(name) is not None:
                        if not collision.detach(name):
                            node.get_logger().error(f'デタッチに失敗しました: {name}')
                            continue
                    if collision.get_object(name) is not None:
                        if not collision.remove_collision(name):
                            node.get_logger().error(f'物体の除去に失敗しました: {name}')
                except Exception as exc:
                    node.get_logger().error(f'後処理に失敗しました: {name}: {exc}')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
