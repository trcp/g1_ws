#!/usr/bin/env python3
"""
hri_task メインステートマシン

フロー:
  1周目: ドアベルskip → 玄関へ移動 → 人を追従 → 名前/飲み物を聞く → リビングへ案内 → 空席を指す
  2周目: 同上 + お互いの紹介
  バッグ受取 → ホスト追従 → バッグを置く
"""
import sys, os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.node import Node
import smach
import json
import time
import threading

# ============================================================
# README / 既知のバグと対策について
# ============================================================
# 【ナビゲーションと関節パブリッシュの干渉について】
# direct_joint_control.py (DirectJointController) は、
# tts_say 等の長時間の動作中にも腕が下がる（力が抜ける）のを防ぎ姿勢を維持するため、
# 常にバックグラウンドで10Hzで関節角をパブリッシュし続けています。
# 
# 過去（50Hzでパブリッシュしていた際）、コントローラーの負荷が高くなり
# ナビゲーションモジュールとの通信干渉（コマンドブロック）が発生したことがありましたが、
# 10Hzに落とした現在の実装であれば、ナビゲーションとパブリッシュを「両立」して動かすことができます。
# 
# 万が一、ナビゲーション移動中にうまく動かない・エラーが出るといった干渉問題が再発した場合、
# `move_to_cb` (ナビゲーション移動関数) の中でコメントアウトされている
# `direct_arm.pause()` および `direct_arm.resume()` のコメントを解除し、
# 移動中のみパブリッシュを止める処置を復活させてください。
# ============================================================

def save_current_guest_info(node, guest_idx, name=None, drink=None, features=None):
    # 保存先はホームディレクトリ
    file_path = os.path.expanduser("~/hri_guest_info.json")
    data = {}
    if os.path.exists(file_path):
        try:
            with open(file_path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception as e:
            if node:
                node.get_logger().warn(f"Failed to load existing json: {e}")
            
    guest_key = f"Guest_{guest_idx}"
    if guest_key not in data:
        data[guest_key] = {}
        
    if name is not None:
        data[guest_key]["name"] = name
    if drink is not None:
        data[guest_key]["drink"] = drink
    if features is not None:
        data[guest_key]["features"] = features
        
    data["last_updated"] = time.strftime("%Y-%m-%d %H:%M:%S")
        
    try:
        with open(file_path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=4, ensure_ascii=False)
        if node:
            node.get_logger().info(f"Updated guest info at {file_path}")
    except Exception as e:
        if node:
            node.get_logger().error(f"Failed to save guest info: {e}")

from std_msgs.msg import String

# ====== g1_ws API ======
from erasers_g1_api.tts import TTS
from erasers_g1_api.robot_control import G1Control
from erasers_g1_api.robot_control import G1Navigation  # デバッグのためコメントアウト

from erasers_g1_api.state_skills.recongnition import SpeechToText

# ====== ローカルモジュール ======
from word_sprit import search_keywords
from yolo_states import (
    YoloTrackingState, YoloEmptyChairState,
    YoloFindBagState, YoloFollowHostState,
    YoloBagGraspInteractionState
)
from direct_joint_control import (
    DirectJointController,
    ARM_POSE_EXTEND_LEFT,
    ARM_POSE_EXTEND_RIGHT
)

# ====== 設定 ======

# 座標リスト [x, y, yaw(rad)]
LOCATIONS = {
    'entrance': [-3.1, 0.82, 3.16],
    'chair_front': [-2.0, 2.12, -1.70],
}

# ドアベルをスキップするかどうか（一旦skip）Trueがスキップfalseが実行
SKIP_DOORBELL = True

# バッグの把持インタラクションをスキップするか
SKIP_BAG_GRASP = False

# ホスト追従をスキップするか
SKIP_FOLLOW_HOST = False

# 特徴抽出のモード ("online" または "offline")
FEATURE_EXTRACTION_MODE = "offline"


# ゲストの名前・飲み物の辞書
TARGET_DICT = {
    "name": [
        # 女性の名前 (25個)
        "mary", "patricia", "jennifer", "linda", "elizabeth", "alice",
        "barbara", "susan", "jessica", "sarah", "karen", 
        "nancy", "lisa", "betty", "margaret", "sandra", 
        "ashley", "kimberly", "emily", "donna", "michelle", 
        "carol", "amanda", "dorothy", "melissa", "deborah",
        # 男性の名前 (25個)
        "james", "john", "robert", "michael", "william", 
        "david", "richard", "joseph", "thomas", "charles", 
        "christopher", "daniel", "matthew", "anthony", "mark", 
        "donald", "steven", "paul", "andrew", "joshua", 
        "kenneth", "kevin", "brian", "george", "timothy","bob","dave","tom"
    ],
    "drink": {
        "Cola": ["coca cola", "coca-cola", "coke", "cola", "pepsi"],
        "Red Bull": ["red bull", "redbull", "red-bull", "energy drink"],
        "Coffee": ["coffee", "cafe", "espresso", "americano", "latte", "cappuccino"],
        "Tea": ["green tea", "black tea", "tea", "ice tea", "iced tea", "earl grey"],
        "Water": ["mineral water", "sparkling water", "water", "still water", "soda water"],
        "Orange Juice": ["orange juice", "orangejuice", "oj"],
        "Apple Juice": ["apple juice", "applejuice"],
        "Milk": ["milk", "whole milk", "low-fat milk"],
        "Beer": ["beer", "ale", "lager", "draft beer"],
        "Wine": ["wine", "red wine", "white wine", "rose wine"],
        "Lemonade": ["lemonade", "lemon juice"],
        "Hot Chocolate": ["hot chocolate", "cocoa", "hot cocoa"],
        "Smoothie": ["smoothie", "fruit smoothie"],
        "Ginger Ale": ["ginger ale", "gingerale"],
        "Tomato Juice": ["tomato juice", "tomatojuice"]
    }
}
# アームの関節値は direct_joint_control.py からインポートするためここからは削除


# ====== コールバック関数 ======

@smach.cb_interface(outcomes=['success', 'failure'], input_keys=[])
def move_to_cb(userdata, node: Node, tts_say, navigation,
               control: G1Control, location_name: str, direct_arm=None,
               message: str = None, arrival_message: str = None, yolo_cmd_pub=None):
    """指定座標に移動する。ナビゲーション中は進行方向を向く。"""
    try:
        # ナビ中はYOLO停止
        if yolo_cmd_pub:
            msg = String()
            msg.data = json.dumps({"command": "stop"})
            yolo_cmd_pub.publish(msg)

        if message:
            tts_say(message)

        # ナビゲーション姿勢: 進行方向を見る

        coords = LOCATIONS[location_name]
        # デバッグ: 移動コマンドの代わりにログ出力
        node.get_logger().info(f"[DEBUG] Simulating movement to {location_name}: {coords}")
        
        # もしナビゲーション移動時に干渉問題が再発した場合は、以下のコメントを外して一時停止してください
        if direct_arm:
            direct_arm.pause()
        
        if navigation:
             navigation.move_abs(x=coords[0], y=coords[1], yaw=coords[2])
        time.sleep(2.0) # 移動にかかる時間の代わり

        # もし上で pause() した場合は、移動後に resume() を呼んで再開してください
        if direct_arm:
            direct_arm.resume()

        if arrival_message:
            tts_say(arrival_message)

        # 到着後: インタラクション姿勢に戻す
        return 'success'
    except Exception as e:
        node.get_logger().error(f"Move Error: {e}")
        # 移動に失敗しても姿勢だけはリセットを試みる
        try:
            node.get_logger().error(f"Move Error: {e}")
        except:
            pass
        return 'failure'


@smach.cb_interface(outcomes=['success', 'failure'], input_keys=[])
def arm_action_cb(userdata, node: Node, tts_say, direct_arm: DirectJointController,
                  action_type: str):
    """アーム操作を一括で行うコールバック。DirectJointController を使用。"""
    try:
        if action_type == 'point_seat':
            # YoloEmptyChairState で既に腰の目標値はセットされている。
            # ここで腕の目標値をセットして2秒間待つことで、腰と腕が同時に動く。
            if direct_arm:
                direct_arm.point_right(hold_sec=2.0)
                
            # 何番目の椅子が空いているかを計算して喋る
            seat_idx = getattr(direct_arm, 'empty_seat_index', 1)
            ordinals = {1: "first", 2: "second", 3: "third", 4: "fourth", 5: "fifth"}
            ordinal_str = ordinals.get(seat_idx, f"{seat_idx}th")
            
            # 姿勢が固定されたまま喋る
            tts_say(f"The {ordinal_str} seat from the left is empty. Please sit there.")
            
            # 喋り終わった後、次のフェーズに進むときにホームポジションへ移行する
            if direct_arm:
                direct_arm.go_home(hold_sec=2.0)

        elif action_type == 'receive_bag':
            # 両腕を前に伸ばしてバッグを受け取る
            tts_say("I am ready to receive your bag. Please hand it to me.")
            # hand_control("open") は現在非対応なので省略
            if direct_arm:
                direct_arm.extend_both_arms(hold_sec=3.0)
            # hand_control(node, "left", "open")
            # hand_control(node, "right", "open")
            time.sleep(1.0)
            tts_say("Please hand me the bag.")
        
            if direct_arm:
                direct_arm.extend_both_arms(hold_sec=3.0)
                # hand_control("close") は現在非対応なので省略
                tts_say("Thank you. I have the bag.")

        elif action_type == 'drop_bag':
            tts_say("I will place your bag here.")
            # hand_control("open") は現在非対応なので省略
            time.sleep(1.0)
            direct_arm.go_home(hold_sec=2.0)

        return 'success'
    except Exception as e:
        node.get_logger().error(f"Arm Error: {e}")
        # エラーでもホームに戻そうとする
        try:
            direct_arm.go_home(hold_sec=2.0)
        except:
            pass
        return 'failure'


@smach.cb_interface(outcomes=['success', 'retry', 'failure'],
                    input_keys=['stt_text', 'guest_name', 'guest_drink', 'num_challenge', 'success_keywards'],
                    output_keys=['guest_name', 'guest_drink', 'num_challenge'])
def parse_guest_info_cb(userdata, node: Node, direct_arm=None, guest_index=1):
    """word_sprit を使って STT テキストから名前と飲み物を抽出する。"""
    text = userdata.stt_text
    found = search_keywords(text, TARGET_DICT)

    name = found['name'][0].capitalize() if found['name'] else ""
    drink = found['drink'][0].capitalize() if found['drink'] else ""

    if name:
        userdata.guest_name = name
    if drink:
        userdata.guest_drink = drink

    current_name = userdata.guest_name
    current_drink = userdata.guest_drink

    # 取得した最新の情報を保存
    save_current_guest_info(node, guest_index, name=current_name if current_name else None, drink=current_drink if current_drink else None)

    if current_name and current_drink:
        node.get_logger().info(f"Parsed -> Name: {current_name}, Drink: {current_drink}")
        
        tts = TTS(node)
        tts.say(f"Hello {current_name}, you like {current_drink}. Welcome to my home.")
        
        time.sleep(0.1)

        # インタラクションが完全に成功して終わったらホームポジションに戻し、追従も終了する
        if direct_arm:
            direct_arm.stop_background_tracking()
            node.get_logger().info("  -> Resetting posture to home after successful interaction")
            direct_arm.go_home(hold_sec=3.0)
            
        # YOLOノードを停止
        cmd_pub = node.create_publisher(String, '/yolo_human/command', 10)
        msg = String()
        msg.data = "stop"
        cmd_pub.publish(msg)
            
        return 'success'
    else:
        # 失敗回数のカウントと制限
        if not hasattr(userdata, 'num_challenge'):
            userdata.num_challenge = 0
        userdata.num_challenge += 1

        if userdata.num_challenge >= 3:
            node.get_logger().warn("Exceeded max retries (2). Reporting missing info instead of using dummy.")
            
            missing_parts = []
            if not current_name:
                missing_parts.append("your name")
            if not current_drink:
                missing_parts.append("your favorite drink")
            
            tts = TTS(node)
            if len(missing_parts) == 2:
                tts.say("I could not catch your name and your favorite drink.")
            else:
                tts.say(f"I could not catch {missing_parts[0]}.")
            
            time.sleep(1.0)
            
            if current_name and not current_drink:
                tts.say(f"Hello {current_name}. Welcome to my home.")
            elif not current_name and current_drink:
                tts.say(f"Hello, I see you like {current_drink}. Welcome to my home.")
            else:
                tts.say("Anyway, welcome to my home.")
            
            # 成功時と同じく追従終了処理を行う
            if direct_arm:
                direct_arm.stop_background_tracking()
                node.get_logger().info("  -> Resetting posture to home after fallback")
                direct_arm.go_home(hold_sec=3.0)
                
            cmd_pub = node.create_publisher(String, '/yolo_human/command', 10)
            msg = String()
            msg.data = "stop"
            cmd_pub.publish(msg)
            
            return 'success'

        node.get_logger().warn(f"Missing info. Current -> Name: {current_name}, Drink: {current_drink}")
        tts = TTS(node)
        
        missing = []
        if not current_name:
            missing.append("your name")
        if not current_drink:
            missing.append("your favorite drink")
            
        if len(missing) == 2:
            tts.say("I could not catch that. Please tell me your name and favorite drink again after pin sound.")
        else:
            tts.say(f"I could not catch {missing[0]}. Please tell me {missing[0]} again after pin sound.")
            
        return 'retry'


@smach.cb_interface(outcomes=['success', 'failure'],
                    input_keys=['g1_name', 'g1_drink', 'g2_name', 'g2_drink'])
def introduce_guests_cb(userdata, node: Node, tts_say, control: G1Control, direct_arm=None):
    """ゲストをお互いに紹介する。"""
    try:
        # ゲスト2の方を向いて、ホスト（ゲスト1）を紹介する。
        # 直前の POINT_SEAT_2 の後にホームポジションに戻っているため、保存しておいたゲスト2の角度へ腰を回す。
        node.get_logger().info("Introducing Guest 1 (Host) to Guest 2...")
        if direct_arm:
            if hasattr(direct_arm, 'guest2_waist_yaw') and hasattr(direct_arm, 'host_waist_yaw'):
                target_waist_2 = max(-1.2, min(1.2, direct_arm.guest2_waist_yaw))
                node.get_logger().info(f"Turning waist to Guest 2 ({target_waist_2}) and pointing to Guest 1...")
                # 腰と腕を同時に動かす
                direct_arm.point_at_guest(target_waist_2, direct_arm.host_waist_yaw, hold_sec=2.0)
                
        g2_greeting = f"Hello {userdata.g2_name}" if userdata.g2_name else "Hello"
        g1_intro = ""
        if userdata.g1_name and userdata.g1_drink:
            g1_intro = f"they are {userdata.g1_name}. Their favorite drink is {userdata.g1_drink}."
        elif userdata.g1_name:
            g1_intro = f"they are {userdata.g1_name}."
        elif userdata.g1_drink:
            g1_intro = f"their favorite drink is {userdata.g1_drink}."
        else:
            g1_intro = "I could not catch their name or favorite drink."

        tts_say(f"{g2_greeting}, {g1_intro}")
        time.sleep(0.5)

        # 指差しの腕を下ろす
        if direct_arm:
            direct_arm.go_home(hold_sec=1.0)

        # 次に、ホスト（ゲスト1）の方を向いて、ゲスト2を紹介する。
        if direct_arm:
            if hasattr(direct_arm, 'host_waist_yaw'):
                # YoloEmptyChairStateで正確に計算されたホストの角度を使用
                target_waist = direct_arm.host_waist_yaw
               
            else:
                # 万が一計算されていなかった場合の推測フォールバック
                current_waist = direct_arm.current_joints.get('waist_yaw_joint', 0.0)
                target_waist = 0.6 if current_waist < 0 else -0.6
                
            # 安全のためリミットをかける
            target_waist = max(-1.2, min(1.2, target_waist))
            
            # 指差し動作 (ゲスト2の方を指す)
            if hasattr(direct_arm, 'guest2_waist_yaw'):
                node.get_logger().info(f"Turning waist to Guest 1 ({target_waist}) and pointing to Guest 2...")
                # 腰と腕を同時に動かす
                direct_arm.point_at_guest(target_waist, direct_arm.guest2_waist_yaw, hold_sec=2.0)
            else:
                node.get_logger().info(f"Turning waist to Guest 1: {target_waist}")
                direct_arm.send_joints({'waist_yaw_joint': target_waist}, hold_sec=2.0)

        node.get_logger().info("Introducing Guest 2 to Guest 1 (Host)...")
        
        g1_greeting = f"Hello {userdata.g1_name}" if userdata.g1_name else "Hello"
        g2_intro = ""
        if userdata.g2_name and userdata.g2_drink:
            g2_intro = f"they are {userdata.g2_name}. Their favorite drink is {userdata.g2_drink}."
        elif userdata.g2_name:
            g2_intro = f"they are {userdata.g2_name}."
        elif userdata.g2_drink:
            g2_intro = f"their favorite drink is {userdata.g2_drink}."
        else:
            g2_intro = "I could not catch their name or favorite drink."
            
        tts_say(f"{g1_greeting}, {g2_intro}")
        time.sleep(0.5)

        # 全て終わったら正面（ホーム）に戻す
        if direct_arm:
            direct_arm.go_home(hold_sec=2.0)
            
        return 'success'
    except Exception as e:
        node.get_logger().error(f"Error in introduce_guests: {e}")
        try:
            control.move_head(pan=0.0)
        except:
            pass
        return 'failure'


@smach.cb_interface(outcomes=['success', 'failure'], input_keys=[])
def skip_doorbell_cb(userdata, node: Node, tts_say):
    """ドアベルは一旦スキップ。将来追加時はここを書き換える。"""
    node.get_logger().info("[DOORBELL] Skipped (not implemented yet)")
    return 'success'

@smach.cb_interface(outcomes=['skip', 'run'], input_keys=[])
def check_skip_bag_grasp_cb(userdata, node: Node):
    if SKIP_BAG_GRASP:
        node.get_logger().info("Skipping Bag Grasp Interaction (SKIP_BAG_GRASP is True).")
        return 'skip'
    return 'run'

@smach.cb_interface(outcomes=['skip', 'run'], input_keys=[])
def check_skip_follow_host_cb(userdata, node: Node):
    if SKIP_FOLLOW_HOST:
        node.get_logger().info("Skipping Follow Host (SKIP_FOLLOW_HOST is True).")
        return 'skip'
    return 'run'



@smach.cb_interface(outcomes=['success', 'failure'],
                    input_keys=['g1_name', 'g1_features'])
def describe_guest_1_cb(userdata, node: Node, tts_say):
    """ゲスト2にゲスト1（ホスト）の特徴を伝える"""
    features = userdata.g1_features if hasattr(userdata, 'g1_features') else {}
    spoken = 0
    
    if isinstance(features, list) and len(features) >= 4:
        tts_say("Let me tell you four features about the first guest.")
        time.sleep(1.0)
        tts_say(f"First, {features[0]}.")
        time.sleep(0.5)
        tts_say(f"Second, {features[1]}.")
        time.sleep(0.5)
        tts_say(f"Third, {features[2]}.")
        time.sleep(0.5)
        tts_say(f"And lastly, {features[3]}.")
        time.sleep(0.5)
        return 'success'
    elif isinstance(features, list):
        tts_say("Let me tell you some features about the first guest.")
        time.sleep(1.0)
        for i, feature in enumerate(features):
            tts_say(f"Feature {i+1}: {feature}.")
            time.sleep(0.5)
        return 'success'

    # 以下はオフライン版のフォールバック
    tts_say("I will tell you the first guest's features.")
    time.sleep(1.0)
    if isinstance(features, dict):
        if features.get('color'):
            name_str = userdata.g1_name if userdata.g1_name else "the first guest"
            tts_say(f"By the way, {name_str} is wearing a {features['color']} shirt.")
            spoken += 1
        if spoken < 4 and features.get('glasses'):
            tts_say(f"They are {features['glasses']}.")
            spoken += 1
        if spoken < 4 and features.get('hat'):
            tts_say(f"They are {features['hat']}.")
            spoken += 1
        if spoken < 4 and features.get('pattern'):
            tts_say(f"Their clothing pattern is {features['pattern']}.")
            spoken += 1
        if spoken < 4 and features.get('hair'):
            tts_say(f"They have {features['hair']}.")
            spoken += 1
        if spoken < 4 and features.get('sleeve'):
            tts_say(f"They are wearing {features['sleeve']}.")
            spoken += 1

    time.sleep(0.5)
    return 'success'


# ====== インタラクション用ステート ======

class YoloSpeechToTextState(smach.State):
    """
    STTで音声を聞きながら、バックグラウンドでYOLOによる腰追従を行うステート。
    一番近い人を追従する。
    """

    def __init__(self, node, tts, start_msg, direct_arm, control, guest_index=1):
        self.speech_to_text = SpeechToText(node=node, tts=tts, start_msg=start_msg)
        super().__init__(
            outcomes=['success', 'timeout', 'failure'],
            input_keys=list(self.speech_to_text.get_registered_input_keys()),
            output_keys=list(self.speech_to_text.get_registered_output_keys()) + ['guest_features']
        )
        self.node = node
        self.direct_arm = direct_arm
        self.control = control
        self.guest_index = guest_index
        self.yolo = YoloTrackingState(
            node=node, target_classes=["person"], timeout=1.0)
            
        self.cmd_pub = self.node.create_publisher(String, '/yolo_human/command', 10)
        self.res_sub = self.node.create_subscription(String, '/yolo_human/result', self._yolo_cb, 10)
        self.latest_features = {}

    def _yolo_cb(self, msg):
        try:
            data = json.loads(msg.data)
            min_z = float('inf')
            closest_features = None
            for det in data:
                if det.get('label') == 'person' and det.get('features'):
                    z = det.get('distance_z', 999.0)
                    if z < min_z:
                        min_z = z
                        closest_features = det['features']
            
            if closest_features is not None:
                self.latest_features = closest_features
        except:
            pass

    def execute(self, userdata):
        self.node.get_logger().info(f"[INTERACTION] Starting STT (YOLO tracking & feature extraction running in background, Mode: {FEATURE_EXTRACTION_MODE})")
        
        self.latest_features = {} # リセット

        cmd_msg = String()
        cmd_msg.data = json.dumps({
            "command": "start", 
            "classes": ["person"], 
            "extract_features": True,
            "feature_mode": FEATURE_EXTRACTION_MODE
        })
        self.cmd_pub.publish(cmd_msg)
        
        # バックグラウンド追従を開始
        if self.direct_arm:
            self.direct_arm.start_background_tracking()

        # STT を実行
        outcome = self.speech_to_text.execute(userdata)

        # STT終了後、特徴抽出の結果が返ってくるまで最大15秒待機
        self.node.get_logger().info("[INTERACTION] Waiting up to 15 seconds for feature extraction result...")
        start_wait = time.time()
        while rclpy.ok() and (time.time() - start_wait < 15.0):
            if self.latest_features:
                self.node.get_logger().info(f"[INTERACTION] Received features: {self.latest_features}")
                break
            time.sleep(0.5)

        if not self.latest_features:
            self.node.get_logger().warn("[INTERACTION] Feature extraction timed out or failed.")

        userdata.guest_features = self.latest_features
        # 特徴量が取れたら保存
        if self.latest_features:
            save_current_guest_info(self.node, self.guest_index, features=self.latest_features)

        # ステート終了時にYOLOと追従を停止し、姿勢を正面に戻す
        reset_msg = String()
        reset_msg.data = json.dumps({"command": "stop"})
        self.cmd_pub.publish(reset_msg)

        if self.direct_arm:
            self.direct_arm.stop_background_tracking()

        return outcome


# ====== サブステートマシン ======

def create_greeting_sm(node, tts_say, tts_obj, direct_arm, control, guest_index=1):
    """挨拶＋名前と飲み物の聞き取りを行うサブステートマシン。"""
    sm = smach.StateMachine(
        outcomes=['success', 'failure'],
        output_keys=['guest_name', 'guest_drink', 'guest_features'])
    sm.userdata.num_challenge = 0
    sm.userdata.stt_text = ""
    sm.userdata.success_keywards = []
    sm.userdata.guest_name = ""
    sm.userdata.guest_drink = ""

    with sm:
        smach.StateMachine.add(
            'ASK_INFO',
            YoloSpeechToTextState(
                node=node, tts=tts_obj,
                start_msg="Hello! I am the host robot. "
                          "What is your name and favorite drink please talk tell me after pin sound",
                direct_arm=direct_arm,
                control=control,
                guest_index=guest_index),
            transitions={
                'success': 'PARSE_INFO',
                'timeout': 'ASK_INFO',
                'failure': 'failure'})

        smach.StateMachine.add(
            'PARSE_INFO',
            smach.CBState(cb=parse_guest_info_cb, cb_kwargs={'node': node, 'direct_arm': direct_arm, 'guest_index': guest_index}),
            transitions={
                'success': 'success',
                'retry': 'ASK_INFO',
                'failure': 'failure'})

    return sm


# ====== メイン ======

def main():
    rclpy.init()
    node = Node('hri_task_main')

    # API 初期化
    tts = TTS(node)
    SAY = tts.say
    CONTROL = G1Control(node)
    NAVIGATION = G1Navigation(node) # デバッグのためコメントアウト
    
    ARM = DirectJointController(node)

    # YOLO コマンド用パブリッシャ
    yolo_cmd_pub = node.create_publisher(String, '/yolo_human/command', 10)

    # トップレベル SMACH
    sm = smach.StateMachine(outcomes=['task_completed', 'task_failed'])

    # Global Userdata
    sm.userdata.guest1_name = ""
    sm.userdata.guest1_drink = ""
    sm.userdata.guest1_features = {}
    sm.userdata.guest2_name = ""
    sm.userdata.guest2_drink = ""
    sm.userdata.guest2_features = {}

    SAY("hri task start!")

    with sm:
        # ====== 1周目: ゲスト1 ======
        smach.StateMachine.add(
            'WAIT_DOOR',
            smach.CBState(cb=skip_doorbell_cb,
                          cb_kwargs={'node': node, 'tts_say': SAY}),
            transitions={'success': 'MOVE_TO_ENTRANCE_1',
                         'failure': 'task_failed'})

        smach.StateMachine.add(
            'MOVE_TO_ENTRANCE_1',
            smach.CBState(cb=move_to_cb, cb_kwargs={
                'node': node, 'tts_say': SAY, 'navigation': NAVIGATION,
                'control': CONTROL, 'location_name': 'entrance',
                'direct_arm': ARM,
                'arrival_message': "I have arrived at the entrance.",
                'yolo_cmd_pub': yolo_cmd_pub}),
            transitions={'success': 'TRACK_PERSON_1',
                         'failure': 'task_failed'})

        smach.StateMachine.add(
            'TRACK_PERSON_1',
            YoloTrackingState(node=node, direct_arm=ARM, use_waist=True),
            transitions={'success': 'GREET_GUEST_1',
                         'failure': 'GREET_GUEST_1',
                         'timeout': 'GREET_GUEST_1'})

        smach.StateMachine.add(
            'GREET_GUEST_1',
            create_greeting_sm(node, SAY, tts, ARM, CONTROL, guest_index=1),
            transitions={'success': 'MOVE_TO_LIVING_1',
                         'failure': 'task_failed'},
            remapping={'guest_name': 'guest1_name',
                       'guest_drink': 'guest1_drink',
                       'guest_features': 'guest1_features'})

        smach.StateMachine.add(
            'MOVE_TO_LIVING_1',
            smach.CBState(cb=move_to_cb, cb_kwargs={
                'node': node, 'tts_say': SAY, 'navigation': NAVIGATION,
                'control': CONTROL, 'location_name': 'chair_front',
                'direct_arm': ARM,
                'message': "Please follow me to the living room.",
                'yolo_cmd_pub': yolo_cmd_pub}),
            transitions={'success': 'FIND_EMPTY_SEAT_1',
                         'failure': 'task_failed'})

        smach.StateMachine.add(
            'FIND_EMPTY_SEAT_1',
            YoloEmptyChairState(node=node, direct_arm=ARM, guest_index=1),
            transitions={'success': 'POINT_SEAT_1',
                         'failure': 'POINT_SEAT_1',
                         'timeout': 'POINT_SEAT_1'})

        smach.StateMachine.add(
            'POINT_SEAT_1',
            smach.CBState(cb=arm_action_cb, cb_kwargs={
                'node': node, 'tts_say': SAY, 'direct_arm': ARM,
                'action_type': 'point_seat'}),
            transitions={'success': 'WAIT_DOOR_2',
                         'failure': 'WAIT_DOOR_2'})

        # ====== 2周目: ゲスト2 ======
        smach.StateMachine.add(
            'WAIT_DOOR_2',
            smach.CBState(cb=skip_doorbell_cb,
                          cb_kwargs={'node': node, 'tts_say': SAY}),
            transitions={'success': 'MOVE_TO_ENTRANCE_2',
                         'failure': 'task_failed'})

        smach.StateMachine.add(
            'MOVE_TO_ENTRANCE_2',
            smach.CBState(cb=move_to_cb, cb_kwargs={
                'node': node, 'tts_say': SAY, 'navigation': NAVIGATION,
                'control': CONTROL, 'location_name': 'entrance',
                'direct_arm': ARM,
                'message': "I will go greet the next guest.",
                'arrival_message': "I have arrived at the entrance.",
                'yolo_cmd_pub': yolo_cmd_pub}),
            transitions={'success': 'TRACK_PERSON_2',
                         'failure': 'task_failed'})

        smach.StateMachine.add(
            'TRACK_PERSON_2',
            YoloTrackingState(node=node, direct_arm=ARM, use_waist=True),
            transitions={'success': 'GREET_GUEST_2',
                         'failure': 'GREET_GUEST_2',
                         'timeout': 'GREET_GUEST_2'})

        smach.StateMachine.add(
            'GREET_GUEST_2',
            create_greeting_sm(node, SAY, tts, ARM, CONTROL, guest_index=2),
            transitions={'success': 'DESCRIBE_GUEST_1',
                         'failure': 'task_failed'},
            remapping={'guest_name': 'guest2_name',
                       'guest_drink': 'guest2_drink',
                       'guest_features': 'guest2_features'})

        smach.StateMachine.add(
            'DESCRIBE_GUEST_1',
            smach.CBState(cb=describe_guest_1_cb, cb_kwargs={'node': node, 'tts_say': SAY}),
            transitions={'success': 'CHECK_BAG_GRASP',
                         'failure': 'CHECK_BAG_GRASP'},
            remapping={'g1_name': 'guest1_name', 'g1_features': 'guest1_features'})

        smach.StateMachine.add(
            'CHECK_BAG_GRASP',
            smach.CBState(cb=check_skip_bag_grasp_cb, cb_kwargs={'node': node}),
            transitions={'skip': 'MOVE_TO_LIVING_2',
                         'run': 'BAG_GRASP_INTERACTION'})

        smach.StateMachine.add(
            'BAG_GRASP_INTERACTION',
            YoloBagGraspInteractionState(node=node, tts_say=SAY, direct_arm=ARM, control=CONTROL, timeout=8.0),
            transitions={'success': 'MOVE_TO_LIVING_2',
                         'failure': 'MOVE_TO_LIVING_2',
                         'timeout': 'MOVE_TO_LIVING_2'})


        smach.StateMachine.add(
            'MOVE_TO_LIVING_2',
            smach.CBState(cb=move_to_cb, cb_kwargs={
                'node': node, 'tts_say': SAY, 'navigation': NAVIGATION,
                'control': CONTROL, 'location_name': 'chair_front',
                'direct_arm': ARM,
                'message': "Please follow me to the living room.",
                'yolo_cmd_pub': yolo_cmd_pub}),
            transitions={'success': 'FIND_EMPTY_SEAT_2',
                         'failure': 'task_failed'})

        smach.StateMachine.add(
            'FIND_EMPTY_SEAT_2',
            YoloEmptyChairState(node=node, direct_arm=ARM, guest_index=2),
            transitions={'success': 'POINT_SEAT_2',
                         'failure': 'POINT_SEAT_2',
                         'timeout': 'POINT_SEAT_2'})

        smach.StateMachine.add(
            'POINT_SEAT_2',
            smach.CBState(cb=arm_action_cb, cb_kwargs={
                'node': node, 'tts_say': SAY, 'direct_arm': ARM,
                'action_type': 'point_seat'}),
            transitions={'success': 'INTRODUCE_GUESTS',
                         'failure': 'INTRODUCE_GUESTS'})

        # ====== 紹介 ======
        smach.StateMachine.add(
            'INTRODUCE_GUESTS',
            smach.CBState(cb=introduce_guests_cb, cb_kwargs={
                'node': node, 'tts_say': SAY, 'control': CONTROL, 'direct_arm': ARM}),
            transitions={'success': 'CHECK_SKIP_FOLLOW',
                         'failure': 'CHECK_SKIP_FOLLOW'},
            remapping={'g1_name': 'guest1_name', 'g1_drink': 'guest1_drink',
                       'g2_name': 'guest2_name', 'g2_drink': 'guest2_drink'})

        # ====== ホスト追従 (止まったら終了) ======
        smach.StateMachine.add(
            'CHECK_SKIP_FOLLOW',
            smach.CBState(cb=check_skip_follow_host_cb, cb_kwargs={'node': node}),
            transitions={'skip': 'DROP_BAG',
                         'run': 'FOLLOW_HOST'})

        smach.StateMachine.add(
            'FOLLOW_HOST',
            YoloFollowHostState(
                node=node, tts_say=SAY, direct_arm=ARM, control=CONTROL,
                max_duration=60.0, stop_threshold=0.05,
                stop_count_required=10, stop_distance=0.8),
            transitions={'success': 'DROP_BAG',
                         'failure': 'DROP_BAG',
                         'timeout': 'DROP_BAG'})

        # ====== バッグを置く ======
        smach.StateMachine.add(
            'DROP_BAG',
            smach.CBState(cb=arm_action_cb, cb_kwargs={
                'node': node, 'tts_say': SAY, 'direct_arm': ARM,
                'action_type': 'drop_bag'}),
            transitions={'success': 'task_completed',
                         'failure': 'task_completed'})

    # 実行
    node.get_logger().info("============ hri Task SMACH Started ============")
    outcome = sm.execute()

    if outcome == 'task_completed':
        SAY("I have successfully completed the hri task.")
    else:
        SAY("Task failed.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
