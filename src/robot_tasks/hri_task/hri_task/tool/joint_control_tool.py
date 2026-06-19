#!/usr/bin/env python3
"""
カメラのpanと腕の関節に好きな値を投げて動かせるテストツール
（バックグラウンドで10Hzでパブリッシュし続けることで姿勢を維持します）
"""
import rclpy
from rclpy.node import Node
import sys
import threading

# direct_joint_control から DirectJointController をインポートして姿勢維持ループを活用する
from direct_joint_control import DirectJointController

ALIASES = {
    'lsp': 'left_shoulder_pitch_joint',
    'lsr': 'left_shoulder_roll_joint',
    'lsy': 'left_shoulder_yaw_joint',
    'le':  'left_elbow_joint',
    'lwr': 'left_wrist_roll_joint',
    'rsp': 'right_shoulder_pitch_joint',
    'rsr': 'right_shoulder_roll_joint',
    'rsy': 'right_shoulder_yaw_joint',
    're':  'right_elbow_joint',
    'rwr': 'right_wrist_roll_joint',
    'waist': 'waist_yaw_joint',
    'pan': 'pan',
    'tilt': 'tilt'
}

class JointControlTool(Node):
    def __init__(self):
        super().__init__('joint_control_tool')
        
        self.get_logger().info("Initializing DirectJointController (this will hold posture)...")
        self.direct_arm = DirectJointController(self)
        
        # G1Controlのインポート（ヘッド制御用）
        try:
            from erasers_g1_api.robot_control import G1Control
            self.control = G1Control(self)
        except ImportError as e:
            self.get_logger().warn(f"Could not import G1Control: {e}")
            self.control = None

        self.last_target = None
        self.last_val = 0.0

def main():
    rclpy.init()
    tool = JointControlTool()
    
    # バックグラウンドでROSのイベントループを回す
    spin_thread = threading.Thread(target=rclpy.spin, args=(tool,), daemon=True)
    spin_thread.start()
    
    print("\n" + "="*40)
    print("=== Interactive Joint Control Tool ===")
    print("姿勢は自動的に保持（10Hz Publish）されます。")
    print("\n【ショートカット一覧】")
    for short, full in ALIASES.items():
        print(f"  {short:<5} -> {full}")
    print("\n【使い方】")
    print("  lsp -1.5    : 左肩ピッチを -1.5 に設定")
    print("  +0.1        : 最後に操作した関節の値を +0.1 する（微調整に便利！）")
    print("  -0.05       : 最後に操作した関節の値を -0.05 する")
    print("  home        : 初期姿勢に戻る")
    print("  exit / quit : 終了")
    print("="*40 + "\n")
    
    while True:
        try:
            cmd_line = input("> ").strip()
            if not cmd_line:
                continue
                
            parts = cmd_line.split()
            cmd = parts[0].lower()
            
            if cmd in ["exit", "quit"]:
                break
                
            elif cmd == "home":
                tool.direct_arm.go_home(hold_sec=0.0)
                print("Sent HOME pose")
                continue

            # 微調整コマンド（+0.1, -0.1 など）
            if cmd.startswith('+') or cmd.startswith('-'):
                if tool.last_target is None:
                    print("Error: まだ対象の関節が選択されていません。先に 'lsp -1.0' のように指定してください。")
                    continue
                try:
                    delta = float(cmd)
                    new_val = tool.last_val + delta
                    cmd = tool.last_target # cmdをターゲット名に差し替え
                    parts = [cmd, str(new_val)]
                except ValueError:
                    print("Error: 数値の形式が正しくありません。")
                    continue

            # エイリアスの解決
            target = ALIASES.get(cmd, cmd)
            
            if target in ['pan', 'tilt']:
                if len(parts) < 2:
                    print(f"Usage: {cmd} <value>")
                    continue
                val = float(parts[1])
                if tool.control:
                    if target == 'pan':
                        tool.control.move_head(pan=val)
                    else:
                        tool.control.move_head(tilt=val)
                    print(f"Sent head {target}: {val}")
                    tool.last_target = target
                    tool.last_val = val
                else:
                    print("G1Control not available for head movement.")
                    
            else:
                # 腕・腰の関節
                if len(parts) < 2:
                    print(f"Usage: {cmd} <value>")
                    continue
                val = float(parts[1])
                
                # direct_arm に送る（バックグラウンドループがこれを10Hzで送り続ける）
                tool.direct_arm.send_joints({target: val}, hold_sec=0.0)
                print(f"Sent {target}: {val:.3f}")
                
                tool.last_target = cmd # ショートカット名を保存
                tool.last_val = val
                
        except ValueError:
            print("Error: Invalid number format.")
        except KeyboardInterrupt:
            print("\nExiting...")
            break
        except Exception as e:
            print(f"Error: {e}")
            
    rclpy.shutdown()

if __name__ == '__main__':
    main()
