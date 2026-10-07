#!/usr/bin/env python3
'''
物体把持のサンプルコード
'''
# ROS2
from rclpy.node import Node
import rclpy
# API
from erasers_g1_api.robot_control import ArmControl, ArmCollision, ArmGrasp


def main():
    # ROS2 初期化
    rclpy.init()
    node = Node('sample_object_grasp')

    # API を初期化
    arm = ArmControl(node, use_sim_time=False)
    collision = ArmCollision(node)
    grasp = ArmGrasp(arm, collision)

    # すべてのコリジョンを削除
    collision.remove_all_collisions()

    # 初期姿勢に遷移
    arm.upper_body_control(True)
    arm.move_groupstate()

    # テーブルを作成
    table_thickness = 0.025
    collision.add_box(
        name='table',
        ref_frame='base_link',
        x=0.4,
        y=0.0,
        z=0.0,
        scale_x=0.5,
        scale_y=1.0,
        scale_z=table_thickness
    )
    # object_0 を作成
    collision.add_box(
        name='object_0',
        ref_frame='base_link',
        x=0.35,
        y=0.2,
        z=table_thickness+0.08/2.0,
        scale_x=0.08,
        scale_y=0.08,
        scale_z=0.08
    )
    # object_1 を作成
    collision.add_sphere(
        name='object_1',
        ref_frame='base_link',
        x=0.35,
        y=0.0,
        z=table_thickness+0.05/2.0,
        radius=0.05
    )
    # object_2 を作成
    collision.add_cylinder(
        name='object_2',
        ref_frame='base_link',
        x=0.35,
        y=-0.2,
        z=table_thickness+0.2/2.0,
        height=0.2,
        radius=0.05
    )

    # 物体把持
    grasp.grasp('object_1')
    # 把持したオブジェクトを削除
    collision.remove_collision('object_1')

    # 初期姿勢に遷移
    arm.upper_body_control(True)
    arm.move_groupstate()

    # ROS2 終了
    node.destroy_node()
    rclpy.shutdown()
