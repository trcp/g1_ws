#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import time
import sys
import numpy as np

# ROS 1の 'smach' は ROS 2 標準ではないため、
# ここではステートマシンの挙動を模倣するクラスとして定義します
class MockStateMachine:
    def __init__(self, outcomes):
        self.outcomes = outcomes
    def execute(self):
        print("[SMACH] Executing State Machine...")
        # 成功したとして 'success' を返すシミュレーション
        return 'success'

# --- Mock Classes for Robot Hardware (HSRのインターフェースを模倣) ---

class MockWholeBody:
    def move_to_neutral(self):
        print("[HARDWARE] Body: Moving to NEUTRAL pose.")
        time.sleep(1.0)

    def move_to_go(self):
        print("[HARDWARE] Body: Moving to TRAVEL pose.")
        time.sleep(1.0)
    
    def move_to_joint_positions(self, joints):
        print(f"[HARDWARE] Body: Moving joints: {joints}")
        time.sleep(1.0)

    def gaze_point(self, point, ref_frame_id):
        print(f"[HARDWARE] Head: Gazing at point {point} in frame '{ref_frame_id}'")

class MockOmniBase:
    def go_abs(self, x, y, yaw, timeout, frame_id):
        print(f"[NAVIGATION] Moving to ABSOLUTE coords: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f} (frame={frame_id})")
        time.sleep(2.0) # 移動時間のシミュレーション

    def go_rel(self, x, y, yaw, timeout, frame_id):
        print(f"[NAVIGATION] Moving RELATIVE: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")
        time.sleep(1.0)

    def pose(self):
        # 現在位置を返すダミー
        return [0.0, 0.0, 0.0]

class MockTTS:
    def say(self, text):
        print(f"\n[TTS] 🗣️ Robot says: \"{text}\"")
        time.sleep(1.0) # 発話時間の待機

# --- Global Variables & Configuration ---

# Global Node for ROS 2 logging
node = None

# Hardware interfaces
whole_body = MockWholeBody()
omni_base = MockOmniBase()
tts = MockTTS()
SAY = tts.say

# Configuration
HAND_TAPPED_THRESHOLD = 5
start_loc = [0.0, 0.0, 0.0] 
drink_loc = [2.0, 2.0, 1.57]
chair_loc = [3.0, 1.0, 0.0]
number_of_person = 2

host_info = [
    ["hiroya", "basketball"],
    ["art", "soccer"],
]

# ゲストの特徴を保存するリスト
guests_features = []

# --- Functions (Original logic preserved) ---

def wait_starting():
    """
    開始待ちとドアオープン待ちのステートマシンを実行する関数
    """
    print("\n--- FUNCTION: wait_starting ---")
    whole_body.move_to_neutral()
    SAY("I'm ready to start the task receptionist. Please tap my hand to start.")
    
    # wait_hand_pushed の代わり
    print("[SENSOR] Waiting for hand tap... (Press Enter to simulate tap)")
    # input() # 自動実行したい場合はコメントアウト
    print("[SENSOR] Hand tapped detected!")

    # SMACHのシミュレーション
    sm = MockStateMachine(outcomes=["success", "failure"])
    
    # 'WaitingDoorOpen' ステートの代わり
    print("[SMACH] State: WaitingDoorOpen")
    print("[VISION/LIDAR] Checking if door is open...")
    time.sleep(1.0)
    SAY("Could you open the door for me?")
    time.sleep(1.0)
    print("[VISION/LIDAR] Door is OPEN.")
    SAY("Thank you.")
    
    return sm.execute()


def get_start_loc():
    """
    スタート位置へ移動
    """
    print("\n--- FUNCTION: get_start_loc ---")
    global start_loc
    x, y, yaw = start_loc
    omni_base.go_abs(x, y, yaw, 0, "map")


def image_msg2base64(img):
    """
    画像をBase64に変換（OpenAI API用）
    """
    print("[IMG PROC] Converting image to Base64 string...")
    return "dummy_base64_string"


def detect_person():
    """
    YOLOを使って人を探し、近づく処理
    """
    print("\n--- FUNCTION: detect_person ---")
    print("[VISION] Waiting for RGB/Depth images...")
    
    # ループ処理のシミュレーション
    print("[VISION] YOLOv8 Predicting... Searching for class 'person'...")
    time.sleep(0.5)
    
    # 人が見つからなかった場合の首振りや発話のロジックが元コードにはありましたが
    # ここでは「見つかった」として進めます
    distance = 1.5 # meters
    print(f"[VISION] Person found! Distance: {distance}m")
    
    # 位置合わせの移動シミュレーション
    print("[CONTROL] Aligning body/head to the person...")
    
    # 撮影した画像を返す想定
    dummy_img = np.zeros((480, 640, 3), dtype=np.uint8)
    print("[VISION] Captured person image.")
    return dummy_img


def extract_person_feauture(person_image):
    """
    OpenAI GPT-4o で人の特徴を抽出する
    """
    print("\n--- FUNCTION: extract_person_feauture ---")
    print("[AI SERVICE] Sending image to OpenAI (GPT-4o)...")
    
    # 実際のAPIコールの代わりにダミー応答を返す
    print("[AI SERVICE] Analyzing: 'Describe the appearance of the person in the center.'")
    
    simulated_response = """
    * Wearing a blue t-shirt
    * Short brown hair
    * Wearing glasses
    """
    print(f"[AI SERVICE] Response received:\n{simulated_response}")
    return simulated_response


def ask_to_human(message: str) -> str:
    """
    Whisperを使って人間に質問し、回答を得る
    """
    print(f"\n--- FUNCTION: ask_to_human ({message}) ---")
    
    whole_body.gaze_point(point=[1.0, 0.0, 1.7], ref_frame_id="base_link")
    
    SAY(f"{message} Please speak loudly.")
    
    # Whisperサービスの呼び出しシミュレーション
    print("[SPEECH] Listening... (Running Whisper Service)")
    time.sleep(2.0)
    
    # ここでマイク入力をシミュレーション
    # 実際にはここで音声認識結果が入る
    simulated_speech_text = "My name is Tanaka and I like sushi"
    print(f"[SPEECH] Recognized text: '{simulated_speech_text}'")
    
    SAY(f'You said "{simulated_speech_text}". Push my hand if this is correct.')
    print("[SENSOR] Waiting for hand confirmation... (Simulated: CONFIRMED)")
    SAY("Thank you.")
    
    return simulated_speech_text


def get_person_info(message: str) -> str:
    """
    GPTを使って発話内容から名前と興味を抽出する
    """
    print("\n--- FUNCTION: get_person_info ---")
    print(f"[AI SERVICE] Sending text to OpenAI: '{message}'")
    print("[AI SERVICE] Prompt: 'Extract name and interest...'")
    
    simulated_info = "Name: Tanaka, Interest: Sushi"
    print(f"[AI SERVICE] Extracted Info: {simulated_info}")
    
    return simulated_info


def move_to_drink_loc() -> str:
    """
    飲み物エリアへ移動し、好きな飲み物を聞く
    """
    print("\n--- FUNCTION: move_to_drink_loc ---")
    x, y, yaw = drink_loc
    omni_base.go_abs(x, y, yaw, 0, "map")
    
    drink_name = ask_to_human("What is your favorite drink?")
    return drink_name


def get_chair_loc():
    """
    椅子エリアへ移動
    """
    print("\n--- FUNCTION: get_chair_loc ---")
    whole_body.move_to_go()
    x, y, yaw = chair_loc
    omni_base.go_abs(x, y, yaw, 0, "map")


def get_empty_chair():
    """
    空いている椅子を探す（YOLO + ロジック）
    """
    print("\n--- FUNCTION: get_empty_chair ---")
    whole_body.move_to_go()
    
    print("[VISION] Detecting 'chair' and 'person' objects...")
    # 検出結果のシミュレーション
    # chairs = [bbox1, bbox2], persons = [bbox_p1]
    
    print("[LOGIC] Calculating empty chair based on person position...")
    # calc_empty_chair のロジック結果
    # 右側の椅子が空いていると仮定
    
    target_chair_bbox = [200, 300, 100, 100] # x, y, w, h
    print(f"[VISION] Empty chair found at bbox: {target_chair_bbox}")
    
    return target_chair_bbox


def calc_rel_location(xy, point_cloud, tf_buffer):
    """
    2D画像座標とPointCloudから3D座標を計算
    """
    print(f"[MATH] Calculating 3D position for pixel {xy} using PointCloud & TF...")
    # ダミーの3D座標を返す
    class MockPoint:
        x = 3.5
        y = 1.2
        z = 0.5
    return MockPoint()


def point_empty_chair(point):
    """
    空いている椅子を指差す
    """
    print("\n--- FUNCTION: point_empty_chair ---")
    if point is None:
        print("[ERROR] Invalid point")
        return

    whole_body.move_to_neutral()
    whole_body.gaze_point(point=[point.x, point.y, point.z], ref_frame_id="map")
    
    # 回転計算のロジック
    print(f"[NAVIGATION] Rotating towards chair at ({point.x}, {point.y})...")
    omni_base.go_rel(0, 0, 0.5, 0, "base_link") # 0.5rad回転シミュレーション
    
    SAY("Found!!")
    
    # 指差し動作
    print("[HARDWARE] Arm: Moving to POINTING pose.")
    whole_body.move_to_joint_positions({
        "arm_lift_joint": 0.2,
        "arm_flex_joint": -0.628,
        "wrist_flex_joint": -0.932
    })
    
    SAY("please sit on that chair")
    time.sleep(2.0)
    whole_body.move_to_neutral()


def main(args=None):
    # ROS 2 初期化
    rclpy.init(args=args)
    global node
    node = Node('receptionist_task')
    
    print("==========================================")
    print("   ROS 2 RECEPTIONIST TASK (MOCK) START   ")
    print("==========================================")
    
    time.sleep(1)
    whole_body.move_to_neutral()

    try:
        # ドアオープン待ち
        wait_starting()
        
        # ゲスト対応ループ
        for i in range(number_of_person):
            print(f"\n\n>>> Handling GUEST No. {i+1} <<<")
            
            get_start_loc()
            time.sleep(2.0) # 移動時間シミュレーション
            
            SAY("please come in front of me")
            time.sleep(1)
            
            # 人検出処理
            person_img = detect_person()
            base64_img = image_msg2base64(person_img)
            
            # 特徴抽出 (OpenAI)
            current_feature = extract_person_feauture(base64_img)
            guests_features.append(current_feature)
            
            SAY("I found a person")
            time.sleep(1)
            
            # 名前と興味を聞く
            person_messege = ask_to_human(
                "Could you tell us your name and one of your interests? "
            )
            person_info = get_person_info(person_messege)

            SAY("please follow behind me")
            time.sleep(1.0)

            # 飲み物エリアへ案内
            SAY("we go to drink area")
            guest_favorite_drink = move_to_drink_loc()
            print(f"[INFO] Guest's favorite drink is: {guest_favorite_drink}")

            whole_body.move_to_go()
            time.sleep(1.0)
            
            # 飲み物案内
            print("[HARDWARE] Head: Tilting down to look at drinks.")
            time.sleep(2.0)
            SAY("There is coffee, water, and juice on the desk.")
            SAY("Your favorite drinks are here.")

            # リビング（椅子）へ移動
            get_chair_loc()

            current_host_name = host_info[i][0]
            current_host_interest = host_info[i][1]

            # ホスト紹介
            omni_base.go_rel(0, 0, -3.14, 0, "base_link") # 振り返る
            SAY(f"host's name is {current_host_name}")
            SAY(f"interest is {current_host_interest}")
            
            omni_base.go_rel(0, 0, 3.14, 0, "base_link") # 元に戻る
            SAY("Hi, I'll introduce our guests now.")
            
            print(f"[INFO] Introducing guest info: {person_info}")
            SAY(person_info)
            time.sleep(1.0)

            # 空き椅子検出と指差し
            # PointCloud取得のシミュレーション
            print("[SENSOR] Waiting for PointCloud2 message...") 
            current_pointcloud = "dummy_pointcloud_data"
            
            empty_chair_bbox = get_empty_chair()

            if empty_chair_bbox:
                # 座標計算のシミュレーション
                x, y, w, h = empty_chair_bbox
                center_x = int(x + (w / 2))
                center_y = int(y + h * 0.8)
                
                target_point = calc_rel_location((center_x, center_y), current_pointcloud, None)
                point_empty_chair(target_point)
            else:
                print("[WARN] No empty chair found.")
                SAY("Sorry, I could not find a place to sit.")

            # 2人目のゲストの場合、1人目の特徴を紹介する（元のロジック通り）
            if i == 1:
                feature_to_report = guests_features[0] if len(guests_features) > 0 else "No features found"
                print(f"[INFO] Reporting first guest's feature: {feature_to_report}")
                SAY(f"The first guest looks like this: {feature_to_report}")

    except KeyboardInterrupt:
        print("Interrupted by user.")
    except Exception as e:
        print(f"[ERROR] Exception occurred: {e}")
        import traceback
        traceback.print_exc()
        SAY("An error occurred.")
    
    print("\n[MAIN] All guests guided. Returning to start.")
    whole_body.move_to_go()
    get_start_loc()
    
    SAY("Task finished. Thank you.")
    time.sleep(2)

    # ROS 2 シャットダウン
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
