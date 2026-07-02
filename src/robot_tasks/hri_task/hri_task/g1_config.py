"""
G1 共有設定ファイル
==================
ロボット寸法・カメラパラメータ・符号規則・移動閾値を一箇所に集約。
実機測定後はこのファイルだけ書き換えれば全モジュールに反映される。

座標系 (原点 = Waist Yaw 関節)
  X : ロボット前方
  Y : ロボット左方向
  Z : 上方向

単位: メートル [m] / ラジアン [rad]
"""
import sys, os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import math
from dataclasses import dataclass, field


# ══════════════════════════════════════════════════════
# 1. ロボット寸法  ← 実機に合わせて編集
# ══════════════════════════════════════════════════════
@dataclass
class MujocoJointConfig:
    TARGET: str = "mujoco"
    MODEL_XML_PATH: str = "/home/roboworks/unitree_mujoco/unitree_robots/g1/g1_23dof.xml"
    REAL_OFFSETS: dict = field(default_factory=dict)
    JOINT_NAMES: list = field(default_factory=lambda: [
        "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
        "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
        "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
        "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
        "waist_yaw_joint",
        "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
        "left_elbow_joint", "left_wrist_roll_joint",
        "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
        "right_elbow_joint", "right_wrist_roll_joint"
    ])
    GRASP_JOINT_MAP_LEFT: dict = field(default_factory=lambda: {
        "waist_yaw": "waist_yaw_joint",
        "shoulder_pitch": "left_shoulder_pitch_joint",
        "elbow_pitch": "left_elbow_joint",
        "wrist_roll": "left_wrist_roll_joint"
    })
    GRASP_JOINT_MAP_RIGHT: dict = field(default_factory=lambda: {
        "waist_yaw": "waist_yaw_joint",
        "shoulder_pitch": "right_shoulder_pitch_joint",
        "elbow_pitch": "right_elbow_joint",
        "wrist_roll": "right_wrist_roll_joint"
    })

@dataclass
class RobotGeometry:
    """
    G1 の骨格寸法と関節リミット。
    デフォルト値は暫定値。実測後に上書きしてください。
    """

    # ── 腰の地面からの高さ ─────────────────────────────
    # 腰関節(Waist Yaw)が地面から何m離れているか
    WAIST_HEIGHT_FROM_GROUND: float = 0.60  # [m] 暫定 60cm 以上

    # ── 首(パンチルト軸)の腰原点からの位置 ──────────────
    NECK_BASE_X: float = 0.15   # [m] 前後 (通常 0 = 腰の真上)
    NECK_BASE_Y: float = 0.00   # [m] 左右 (通常 0 = 中央)
    NECK_BASE_Z: float = 0.55  # [m] 高さ (腰から首までの距離)

    # ── 肩位置 (腰原点からの相対位置, 腰ヨー=0のとき) ───
    SHOULDER_X: float = 0.004    # [m] 前後 (通常 0)
    SHOULDER_Y: float = 0.100    # [m] 左右 (左肩: +Y 方向)
    SHOULDER_Z: float = 0.238    # [m] 高さ

    # ── アームリンク長 ─────────────────────────────────
    L_UPPER: float = 0.192       # [m] 肩→肘
    L_LOWER: float = 0.270       # [m] 肘→手のひら奥(指の付け根) (手首0.220 + 0.05m追加)

    # ── 関節リミット [rad] ─────────────────────────────
    # 腰ヨー (カメラは正面のみ → ±45°で十分)
    WAIST_YAW_MIN:  float = -1.57   # [rad] ≈ -90°
    WAIST_YAW_MAX:  float =  1.57   # [rad] ≈ +90°

    # 肩ピッチ (0=水平前方, 負=下, 正=上)
    SHOULDER_PITCH_MIN: float = -3.14
    SHOULDER_PITCH_MAX: float =  1.57

    # 肘ピッチ (MuJoCo実測: range=[-1.0472, 2.0944])
    ELBOW_PITCH_MIN: float = -1.0472
    ELBOW_PITCH_MAX: float =  2.0944

    # 手首ロール
    WRIST_ROLL_MIN: float = -1.57
    WRIST_ROLL_MAX: float =  1.57

    # 首パン (正=左)
    NECK_PAN_MIN:  float = -1.57
    NECK_PAN_MAX:  float =  1.57

    # 首チルト (正=上, 負=下)
    NECK_TILT_MIN: float = -0.87    # ≈ -50° (下を向く限界)
    NECK_TILT_MAX: float =  0.52    # ≈ +30° (上を見る限界)

    @property
    def max_reach(self) -> float:
        """アーム最大リーチ [m]"""
        return self.L_UPPER + self.L_LOWER

    @property
    def min_reach(self) -> float:
        """アーム最小リーチ [m] (肘を最大限曲げたとき)"""
        return abs(self.L_UPPER - self.L_LOWER)


# ══════════════════════════════════════════════════════
# 2. 掴み動作時のオフセット設定 (実機調整用)
# ══════════════════════════════════════════════════════
@dataclass
class GraspOffsets:
    """
    目標座標（カメラで捉えた物体の中心）から、
    ロボットが実際に手を伸ばす座標までのズレ（オフセット）を定義します。
    実機テスト時に「もう少し手を横にずらしたい」「奥まで差し込みたい」
    という場合は、まずここを調整してください。
    """
    # ── 高さ(Z軸)の補正 ──────────────────────────────────
    # [m] 負の値 = 物体より下を狙う / 正の値 = 物体より上を狙う
    # G1は肩の傾斜構造により計算より約5cm手先が上振れするため、デフォルトで -0.05m 下げています
    Z_OFFSET: float = -0.05

    # ── 奥行き(X軸)の補正 ────────────────────────────────
    # [m] 手のひら（指の付け根）に物体をジャストフィットさせるための設定は
    # RobotGeometry の L_LOWER の延長 (+5cm) によって物理モデル側で解決済みのため、
    # ここは基本的に 0.0 で問題ありません。
    X_OFFSET: float = 0.00

    # ── 横アプローチ時の左右(Y軸)の補正 ─────────────────
    # [m] 横から掴む時、物体の中心ではなく「物体の横」に手を配置するためのズレ。
    # 正の値 = 物体の【外側】へずらす (左手なら左へ、右手なら右へ自動で計算されます)
    # 手の厚みで物体にぶつからないよう、十分な外側(10cm)を狙います。
    SIDE_APPROACH_Y_OFFSET: float = 0.10

    # ── 上アプローチ時の高さ(Z軸)の追加補正 ─────────────
    # [m] 上から掴む時は、物体の中心ではなく「少し上」で手を広げて下ろすため、
    # Z_OFFSET に加えてさらに高さをかさ上げします。(5cm)
    TOP_APPROACH_Z_OFFSET: float = 0.05


# ══════════════════════════════════════════════════════
# 3. カメラパラメータ  ← 実機に合わせて編集
# ══════════════════════════════════════════════════════
@dataclass
class CameraParams:
    """
    RealSense カメラの取り付け位置・姿勢・内部パラメータ。

    カメラは首のパンチルト機構に取り付けられている。
    パンチルト軸からのオフセットをここで定義する。
    """

    # ── パンチルト軸からのカメラ位置 [m] ───────────────
    # (チルト回転後のローカル座標系で定義)
    CAM_FROM_NECK_X: float = 0.05   # [m] 前方 (首軸より前にカメラがある)
    CAM_FROM_NECK_Y: float = 0.00   # [m] 左右 (中央なら 0)
    CAM_FROM_NECK_Z: float = 0.05   # [m] 上方 (首軸より上にカメラがある)

    # ── カメラ固定取り付け姿勢 ─────────────────────────
    # パンチルト機構に対するカメラ自体の姿勢ズレ
    # (通常は 0。カメラが少し下向きに固定されている場合は負の値)
    MOUNT_PITCH: float = 0.00   # [rad] 0=水平, 負=下向き
    MOUNT_YAW:   float = 0.00   # [rad] 0=正面

    # ── RealSense 内部パラメータ (D435i 640×480) ──────
    FX: float = 615.0   # 焦点距離 x [px]
    FY: float = 615.0   # 焦点距離 y [px]
    CX: float = 320.0   # 主点 x [px]
    CY: float = 240.0   # 主点 y [px]

    # ── 画像解像度 ─────────────────────────────────────
    IMG_WIDTH:  int = 640
    IMG_HEIGHT: int = 480


# ══════════════════════════════════════════════════════
# 3. 符号規則  ← 関節が逆方向に動いたら ±1 で調整
# ══════════════════════════════════════════════════════
@dataclass
class SignConv:
    """
    各関節の符号規則。
    実機で方向が逆なら sign を -1 に変更。
    OFFSET はゼロ点ズレの補正 (ロボット固有のゼロ位置がずれている場合)。
    """

    # ── 方向符号 ───────────────────────────────────────
    WAIST_YAW:      int = +1    # +1: 正 = 左回転
    SHOULDER_PITCH: int = -1    # -1: IK正(前方)→MuJoCo負(前方)
    ELBOW_PITCH:    int = +1    # +1: IK正(曲げ)→MuJoCo正(曲げ)
    WRIST_ROLL:     int = +1    # +1: 正 = 時計回り (肩側から見て)
    NECK_PAN:       int = +1    # +1: 正 = 左
    NECK_TILT:      int = +1    # +1: 正 = 上

    # ── ゼロ点オフセット [rad] (符号反転後に加算) ──────
    SHOULDER_PITCH_OFFSET: float = 0.0
    ELBOW_PITCH_OFFSET:    float = 0.0
    WRIST_ROLL_OFFSET:     float = 0.0


# ══════════════════════════════════════════════════════
# 4. 移動閾値  ← 把持可能距離と移動判定
# ══════════════════════════════════════════════════════
@dataclass
class MovementThresholds:
    """
    移動判定のための閾値。
    出力層で「把持可能」「前進必要」「後退必要」「横移動必要」を判定する。
    """

    # ── 到達距離 ───────────────────────────────────────
    # 肩からの距離でリーチ判定する
    REACH_TOO_CLOSE: float = 0.10    # [m] これより近いと腕が畳めず掴めない
    REACH_COMFORTABLE_MIN: float = 0.15  # [m] 快適に掴める最小距離
    REACH_COMFORTABLE_MAX: float = 0.42  # [m] 快適に掴める最大距離 (max_reach - マージン)
    REACH_ABSOLUTE_MAX: float = 0.48     # [m] 物理的最大 (≈ L_UPPER + L_LOWER - 少し)

    # ── 移動マージン ───────────────────────────────────
    APPROACH_TARGET_DIST: float = 0.35   # [m] 移動後に目標とする肩-物体間距離
    FORWARD_MARGIN: float = 0.05         # [m] 前進/後退時の追加マージン

    # ── 高さ補正 ───────────────────────────────────────
    # 物体が肩より大きく低い場合、腕を下に伸ばすと前方リーチが減る
    # → 近づく必要がある
    HEIGHT_CLOSE_APPROACH: float = -0.30  # [m] 肩より30cm以上低いなら接近推奨

    # ── 横方向移動 ──────────────────────────────────────
    # 腰ヨーが限界に達しても物体に正対できない場合
    LATERAL_MOVE_THRESHOLD: float = 0.10  # [m] 横移動の最小単位

    # ── 歩行単位 ───────────────────────────────────────
    STEP_FORWARD: float = 0.10   # [m] 1歩の前進量
    STEP_LATERAL: float = 0.05   # [m] 1歩の横移動量
    STEP_BACKWARD: float = 0.08  # [m] 1歩の後退量


# ══════════════════════════════════════════════════════
# 5. IK ソルバー設定
# ══════════════════════════════════════════════════════
@dataclass
class IKConfig:
    """
    逆運動学ソルバーのパラメータ。
    """

    # ── 腰角度探索 ─────────────────────────────────────
    WAIST_SEARCH_STEP: float = 0.035  # [rad] ≈ 2° 刻み
    # 探索範囲は RobotGeometry の WAIST_YAW_MIN/MAX を使用

    # ── 姿勢評価の重み ─────────────────────────────────
    # スコア = Σ(weight × penalty)。小さいほど良い姿勢。
    WEIGHT_WAIST_ROTATION: float = 1.0    # 腰回転量へのペナルティ
    WEIGHT_ELBOW_COMFORT:  float = 0.5    # 肘角度の自然さ (π/3 からの距離)
    WEIGHT_SHOULDER_RANGE: float = 0.3    # 肩角度の大きさ
    WEIGHT_LATERAL_OFFSET: float = 10.0   # 矢状面からのズレ (大きいと不正確)

    # ── 理想肘角度 ─────────────────────────────────────
    IDEAL_ELBOW_ANGLE: float = 1.05  # [rad] ≈ 60° (自然な曲がり具合)


# ══════════════════════════════════════════════════════
# 6. ユーティリティ関数
# ══════════════════════════════════════════════════════
def clamp(v: float, lo: float, hi: float) -> float:
    """値 v を [lo, hi] の範囲にクランプする"""
    return max(lo, min(hi, v))


def safe_acos(x: float) -> float:
    """数値誤差で |x|>1 になっても安全な acos"""
    return math.acos(max(-1.0, min(1.0, x)))
