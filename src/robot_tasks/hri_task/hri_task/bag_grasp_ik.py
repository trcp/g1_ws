#!/usr/bin/env python3
import sys, os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import math

try:
    from g1_config import RobotGeometry, CameraParams
except ImportError:
    # 依存関係がない環境でテストできるようにフォールバック
    class RobotGeometry:
        WAIST_HEIGHT_FROM_GROUND = 0.60
        NECK_BASE_X = 0.15
        NECK_BASE_Y = 0.00
        NECK_BASE_Z = 0.55
        SHOULDER_X = 0.004
        SHOULDER_Y = 0.100
        SHOULDER_Z = 0.238
        L_UPPER = 0.192
        L_LOWER = 0.270
    class CameraParams:
        CAM_FROM_NECK_X = 0.05
        CAM_FROM_NECK_Y = 0.00
        CAM_FROM_NECK_Z = 0.05
        FX = 615.0
        FY = 615.0
        CX = 320.0
        CY = 240.0

def safe_acos(x):
    return math.acos(max(-1.0, min(1.0, x)))

def solve_2link_ik(fwd: float, down: float, L1: float, L2: float):
    D = math.sqrt(fwd**2 + down**2)

    if D > L1 + L2:
        ratio = (L1 + L2 - 0.001) / D if D > 0 else 0
        fwd *= ratio
        down *= ratio
        D = L1 + L2 - 0.001
    elif D < abs(L1 - L2):
        ratio = (abs(L1 - L2) + 0.001) / D if D > 0 else 0
        fwd *= ratio
        down *= ratio
        D = abs(L1 - L2) + 0.001

    cos_elbow = (L1**2 + L2**2 - D**2) / (2.0 * L1 * L2)
    elbow_inner = safe_acos(cos_elbow)
    elbow_angle = math.pi - elbow_inner

    angle_to_target = math.atan2(fwd, down + 1e-9)
    cos_beta = (L1**2 + D**2 - L2**2) / (2.0 * L1 * D)
    beta = safe_acos(cos_beta)
    shoulder_angle = angle_to_target - beta

    return shoulder_angle, elbow_angle

def calculate_bag_grasp_joints(bag_cx, bag_cy, distance_z, head_tilt=-0.5):
    """
    YOLOのバウンディングボックス中心座標と距離から、
    右手でバッグを把持するための 腰・右肩・右肘 の目標関節角を計算する。

    Parameters:
    -----------
    bag_cx : float
        YOLOのバウンディングボックス中心 X座標（ピクセル）
    bag_cy : float
        YOLOのバウンディングボックス中心 Y座標（ピクセル）
    distance_z : float
        カメラからバッグまでの距離 (メートル)
    head_tilt : float
        頭部のピッチ角度（ラジアン）。負の値は下を向いていることを示す。

    Returns:
    --------
    dict
        各ジョイントの目標角度
    """
    geo = RobotGeometry()
    cam = CameraParams()

    # 1. カメラ座標系でのバッグ位置
    # Y軸は下向き、X軸は右向き、Z軸は奥。
    cam_x = distance_z * (bag_cx - cam.CX) / cam.FX
    cam_y = distance_z * (bag_cy - cam.CY) / cam.FY

    # ROS / ロボットベースの座標系（Z=上, X=前, Y=左）に変換
    # 頭が head_tilt だけピッチ回転していることを加味する
    cos_t = math.cos(head_tilt)
    sin_t = math.sin(head_tilt)
    
    bag_pos_x = distance_z * cos_t + cam_y * sin_t
    bag_pos_y = -cam_x
    bag_pos_z = distance_z * sin_t - cam_y * cos_t

    # 2. 腰（Waist）を原点としたバッグの座標に変換
    cam_offset_x = geo.NECK_BASE_X + cam.CAM_FROM_NECK_X
    cam_offset_y = geo.NECK_BASE_Y + cam.CAM_FROM_NECK_Y
    cam_offset_z = geo.NECK_BASE_Z + cam.CAM_FROM_NECK_Z

    target_w_x = bag_pos_x + cam_offset_x
    target_w_y = bag_pos_y + cam_offset_y
    target_w_z = bag_pos_z + cam_offset_z

    # 3. 腰の回転角（Waist Yaw）を計算
    # 右肩の正面にバッグが来るように腰を回す
    sy = -geo.SHOULDER_Y  # 右肩のYオフセット
    R = math.hypot(target_w_x, target_w_y)
    asin_val = sy / R if R > abs(sy) else math.copysign(1.0, sy)
    waist_yaw = math.atan2(target_w_y, target_w_x) - math.asin(asin_val)
    
    # 制限 (-1.2 〜 1.2 rad)
    waist_yaw = max(-1.2, min(1.2, waist_yaw))

    # 4. 腰が回った後の右肩から見たターゲット座標
    cos_w = math.cos(waist_yaw)
    sin_w = math.sin(waist_yaw)
    
    # 腰回転後の右肩のワールド座標
    sx = geo.SHOULDER_X * cos_w - sy * sin_w
    s_y = geo.SHOULDER_X * sin_w + sy * cos_w
    
    # 手前方向の距離 (dx) を計算 (ターゲットベクトルを腕の正面方向に射影)
    dx = (target_w_x - sx) * cos_w + (target_w_y - s_y) * sin_w
    dz = target_w_z - geo.SHOULDER_Z

    # 4. Mujoco版フルIKに基づく計算（肩と肘を両方動かして正確に手を伸ばす）
    # 簡略化モデル(肩ピッチ0固定)だと、カメラの高さにあるバッグに物理的に届かず手が動かないため
    fwd = dx
    down = -dz
    
    shoulder_angle, elbow_angle = solve_2link_ik(fwd, down, geo.L_UPPER, geo.L_LOWER)
    
    # 角度をロボットの関節座標系にマッピング
    right_shoulder_pitch = -shoulder_angle
    elbow_pitch = 1.5708 - elbow_angle

    # その他の関節は固定値 (右腕用に左右対称または適切な符号を設定)
    right_shoulder_roll = -0.2
    right_shoulder_yaw = 0.0
    right_wrist_roll = 0.0

    return {
        "waist_yaw_joint": waist_yaw,
        "right_shoulder_pitch_joint": right_shoulder_pitch,
        "right_elbow_joint": elbow_pitch,
        "right_shoulder_roll_joint": right_shoulder_roll,
        "right_shoulder_yaw_joint": right_shoulder_yaw,
        "right_wrist_roll_joint": right_wrist_roll
    }
