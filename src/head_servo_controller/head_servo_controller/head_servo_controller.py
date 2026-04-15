import sys
import math
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger  # Added for calibration
from dynamixel_sdk import *
from serial import SerialException

# ★ カスタムサービスのインポート
from g1_srvs.srv import MoveServo

# ==========================================
#               ユーザー設定エリア
# ==========================================
DEVICENAME = '/dev/ttyUSB0'
BAUDRATE   = 1000000
PROTOCOL_VERSION = 2.0

ID_PAN  = 1
ID_TILT = 0

JOINT_NAME_PAN  = "xl330_joint"
JOINT_NAME_TILT = "d455_joint"

PAN_HOME_PULSE  = 2500
TILT_HOME_PULSE = 2048

PAN_DIR  = 1
TILT_DIR = 1

#PAN_RAD_OFFSET  = 0.325
PAN_RAD_OFFSET  = -0.1
TILT_RAD_OFFSET = -0.057

# --- キャリブレーション結果に基づくリミット設定 (マージン込み) ---
# Measured: Tilt [1039:2751], Pan [1938:3131]
PAN_MIN_PULSE  = 2000
PAN_MAX_PULSE  = 3100
TILT_MIN_PULSE = 1100
TILT_MAX_PULSE = 2700

# --- 移動速度の設定 (Profile Velocity) ---
MOVING_SPEED = 60 
CALIB_SPEED  = 20   # キャリブレーション時の低速移動速度

# --- 制御パラメータ ---
CONTROL_PERIOD_SEC = 0.05  # 20Hz
VEL_TIMEOUT_SEC    = 0.5
SERVICE_TIMEOUT    = 5.0
GOAL_TOLERANCE     = 0.08
LOAD_THRESHOLD     = 400     # 負荷しきい値 (XL330 Load: -1000~1000)
LOAD_COUNT_LIMIT   = 5       # しきい値超え連続カウント数

# ==========================================

ADDR_OPERATING_MODE   = 11
ADDR_TORQUE_ENABLE    = 64
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION    = 116
ADDR_PRESENT_LOAD     = 126 # ★負荷(Current/Load)
ADDR_PRESENT_POSITION = 132

class PanTiltNode(Node):
    def __init__(self):
        super().__init__('head_servo_controller')
        
        self.target_pan_rad = 0.0
        self.target_tilt_rad = 0.0
        self.vel_cmd_pan = 0.0
        self.vel_cmd_tilt = 0.0
        self.last_vel_time = 0.0
        self.is_connected = False

        self.declare_parameter('dx_path', DEVICENAME)

        self.portHandler = PortHandler(self.get_parameter('dx_path').value)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        self.try_connect()

        self.pub_pan_raw = self.create_publisher(Int32, '/pan/present_position', 10)
        self.pub_tilt_raw = self.create_publisher(Int32, '/tilt/present_position', 10)
        self.pub_joint_state = self.create_publisher(JointState, '/joint_states', 10)

        self.create_subscription(Twist, '/servo_vel', self.vel_callback, 10)
        self.create_subscription(JointState, '/upper_joints_control', self.joint_control_callback, 10)
        self.create_service(MoveServo, '/move_servo', self.move_servo_callback)
        self.create_service(Trigger, '/calibrate_head', self.calibrate_callback)

        self.timer = self.create_timer(CONTROL_PERIOD_SEC, self.timer_callback)

    def try_connect(self):
        self.get_logger().info(f"Connecting to {self.get_parameter('dx_path').value}...")
        try:
            if self.portHandler.is_open:
                self.portHandler.closePort()
            
            if self.portHandler.openPort():
                if self.portHandler.setBaudRate(BAUDRATE):
                    self.get_logger().info(f"Connected. Initializing motors...")
                    # 接続時にPan/Tilt両方の設定を試みる
                    if self.setup_motor(ID_PAN, "Pan") and self.setup_motor(ID_TILT, "Tilt"):
                        self.get_logger().info("Motors Ready!")
                        self.is_connected = True
                        return True
            self.get_logger().error("Failed to open port or set baudrate.")
        except Exception as e:
            self.get_logger().error(f"Connection Exception: {e}")
        
        self.is_connected = False
        return False

    def setup_motor(self, dxl_id, name):
        try:
            # 1. トルクOFF
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, 0)
            time.sleep(0.02)
            
            # 2. モード設定 (Position Control Mode)
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_OPERATING_MODE, 3)
            time.sleep(0.02)
            
            # 3. ★速度制限 (Profile Velocity) の設定
            self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, ADDR_PROFILE_VELOCITY, MOVING_SPEED)
            time.sleep(0.02)
            
            # 4. トルクON
            res, err = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, 1)
            if res == COMM_SUCCESS and err == 0:
                self.get_logger().info(f"[{name}] Velocity Profile set to {MOVING_SPEED}. Torque ON.")
                return True
        except Exception:
            pass
        self.get_logger().warn(f"[{name}] Setup failed. Will retry.")
        return False

    def safe_read_pulse(self, dxl_id):
        if not self.is_connected: return None
        try:
            pos, res, err = self.packetHandler.read4ByteTxRx(self.portHandler, dxl_id, ADDR_PRESENT_POSITION)
            if res == COMM_SUCCESS:
                return pos
        except (SerialException, OSError) as e:
            self.handle_disconnect(e)
        return None

    def safe_read_load(self, dxl_id):
        if not self.is_connected: return 0
        try:
            # Load is 2 bytes (int16)
            load, res, err = self.packetHandler.read2ByteTxRx(self.portHandler, dxl_id, ADDR_PRESENT_LOAD)
            if res == COMM_SUCCESS:
                # Convert to signed 16-bit
                if load > 32767: load -= 65536
                return load
        except (SerialException, OSError) as e:
            self.handle_disconnect(e)
        return 0

    def safe_write_pulse(self, dxl_id, pulse):
        if not self.is_connected: return
        
        # Apply Limits
        pulse = int(pulse)
        if dxl_id == ID_PAN:
            pulse = max(PAN_MIN_PULSE, min(PAN_MAX_PULSE, pulse))
        elif dxl_id == ID_TILT:
            pulse = max(TILT_MIN_PULSE, min(TILT_MAX_PULSE, pulse))
        else:
            pulse = max(0, min(4095, pulse))
            
        try:
            res, err = self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, ADDR_GOAL_POSITION, pulse)
        except (SerialException, OSError) as e:
            self.handle_disconnect(e)
    
    def set_profile_velocity(self, dxl_id, velocity):
        if not self.is_connected: return
        try:
             self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, ADDR_PROFILE_VELOCITY, velocity)
        except:
            pass

    def handle_disconnect(self, e):
        if self.is_connected:
            self.get_logger().error(f"CRITICAL: Communication lost! ({e})")
            self.is_connected = False

    def rad_to_pulse(self, rad, home_pulse, direction, offset_rad):
        return int(home_pulse + ((rad - offset_rad) * (4096.0 / (2 * math.pi)) * direction))

    def pulse_to_rad(self, pulse, home_pulse, direction, offset_rad):
        return (pulse - home_pulse) * (2 * math.pi / 4096.0) * direction + offset_rad

    def write_position_rad(self, dxl_id, rad, home_pulse, direction, offset_rad):
        pulse = self.rad_to_pulse(rad, home_pulse, direction, offset_rad)
        self.safe_write_pulse(dxl_id, pulse)

    # --- 共通処理: 状態の取得と配信 ---
    def publish_current_state(self):
        pan_pulse = self.safe_read_pulse(ID_PAN)
        tilt_pulse = self.safe_read_pulse(ID_TILT)

        if pan_pulse is not None and tilt_pulse is not None:
            self.pub_pan_raw.publish(Int32(data=pan_pulse))
            self.pub_tilt_raw.publish(Int32(data=tilt_pulse))

            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.name = [JOINT_NAME_PAN, JOINT_NAME_TILT]
            
            p_rad = self.pulse_to_rad(pan_pulse, PAN_HOME_PULSE, PAN_DIR, PAN_RAD_OFFSET)
            t_rad = self.pulse_to_rad(tilt_pulse, TILT_HOME_PULSE, TILT_DIR, TILT_RAD_OFFSET)
            
            joint_state.position = [p_rad, t_rad]
            joint_state.velocity = []
            joint_state.effort = []
            self.pub_joint_state.publish(joint_state)
            
            return p_rad, t_rad
        return None, None

    # --- キャリブレーションヘルパー ---
    def execute_calibration_for_joint(self, dxl_id, name):
        self.get_logger().info(f"Calibrating {name} (ID: {dxl_id})...")
        
        # 1. 速度を落とす
        self.set_profile_velocity(dxl_id, CALIB_SPEED)
        
        detected_min_pulse = -1
        detected_max_pulse = -1
        
        # Upper Limit Search (Pulse Increase)
        self.get_logger().info(f"[{name}] Searching Upper Limit...")
        start_pulse = self.safe_read_pulse(dxl_id)
        if start_pulse is None:
             return None, None
             
        # ゆっくり限界まで動かす(最大4000まで)
        self.safe_write_pulse(dxl_id, 4000) 
        
        overload_count = 0
        detected_upper_pulse = 4000 # default fallback
        
        for _ in range(200): # 10秒
            load = abs(self.safe_read_load(dxl_id))
            pos  = self.safe_read_pulse(dxl_id)
            
            if load > LOAD_THRESHOLD:
                overload_count += 1
                # self.get_logger().info(f"[{name}] High Load: {load}")
            else:
                overload_count = 0
            
            if overload_count > LOAD_COUNT_LIMIT:
                detected_upper_pulse = pos
                self.get_logger().info(f"[{name}] Upper Limit Detected at {pos} (Load: {load})")
                self.safe_write_pulse(dxl_id, pos) # Stop
                break
            time.sleep(0.05)
            
        # 少し戻す
        time.sleep(1.0)
        self.safe_write_pulse(dxl_id, detected_upper_pulse - 200)
        time.sleep(2.0)

        # Lower Limit Search (Pulse Decrease)
        self.get_logger().info(f"[{name}] Searching Lower Limit...")
        self.safe_write_pulse(dxl_id, 100) # ほぼ0へ
        
        overload_count = 0
        detected_lower_pulse = 0 # default fallback
        
        for _ in range(200):
            load = abs(self.safe_read_load(dxl_id))
            pos  = self.safe_read_pulse(dxl_id)
            
            if load > LOAD_THRESHOLD:
                overload_count += 1
                # self.get_logger().info(f"[{name}] High Load: {load}")
            else:
                overload_count = 0
                
            if overload_count > LOAD_COUNT_LIMIT:
                detected_lower_pulse = pos
                self.get_logger().info(f"[{name}] Lower Limit Detected at {pos} (Load: {load})")
                self.safe_write_pulse(dxl_id, pos) # Stop
                break
            time.sleep(0.05)
            
        # 速度を戻す
        self.set_profile_velocity(dxl_id, MOVING_SPEED)
        
        # 中央へ戻る
        center = int((detected_upper_pulse + detected_lower_pulse) / 2)
        self.safe_write_pulse(dxl_id, center)
        time.sleep(1.0) # Wait for move
        
        return detected_lower_pulse, detected_upper_pulse

    # --- キャリブレーションコールバック ---
    def calibrate_callback(self, request, response):
        self.get_logger().info("Starting System Calibration...")
        if not self.is_connected:
            response.success = False
            response.message = "Motor not connected"
            return response

        # Calibrate Tilt
        tilt_min, tilt_max = self.execute_calibration_for_joint(ID_TILT, "Tilt")
        if tilt_min is None:
            response.success = False
            response.message = "Tilt Calibration Failed (Read Error)"
            return response

        # Calibrate Pan
        pan_min, pan_max = self.execute_calibration_for_joint(ID_PAN, "Pan")
        if pan_min is None:
            response.success = False
            response.message = "Pan Calibration Failed (Read Error)"
            return response

        response.success = True
        response.message = f"Tilt [{tilt_min}:{tilt_max}], Pan [{pan_min}:{pan_max}]"
        return response

    # --- サービスコールバック ---
    def move_servo_callback(self, request, response):
        target_pan = request.pan
        target_tilt = request.tilt
        
        self.get_logger().info(f"MoveServo: Pan={target_pan:.2f}, Tilt={target_tilt:.2f}")
        
        self.target_pan_rad = target_pan
        self.target_tilt_rad = target_tilt
        
        self.write_position_rad(ID_PAN, target_pan, PAN_HOME_PULSE, PAN_DIR, PAN_RAD_OFFSET)
        self.write_position_rad(ID_TILT, target_tilt, TILT_HOME_PULSE, TILT_DIR, TILT_RAD_OFFSET)
        
        success = self.wait_for_both_arrival(target_pan, target_tilt)
        
        response.success = success
        if success:
            self.get_logger().info(" -> Target Reached.")
        else:
            self.get_logger().warn(" -> Timeout or Stalled.")
            
        return response

    def wait_for_both_arrival(self, target_pan, target_tilt):
        start_time = time.time()
        while (time.time() - start_time) < SERVICE_TIMEOUT:
            if not self.is_connected: return False
            
            curr_pan, curr_tilt = self.publish_current_state()
            
            if curr_pan is not None and curr_tilt is not None:
                pure_curr_pan = curr_pan - PAN_RAD_OFFSET
                pure_curr_tilt = curr_tilt - TILT_RAD_OFFSET
                
                err_pan = abs(target_pan - pure_curr_pan)
                err_tilt = abs(target_tilt - pure_curr_tilt)
                
                if err_pan < GOAL_TOLERANCE and err_tilt < GOAL_TOLERANCE:
                    return True
            time.sleep(0.05)
        return False

    def vel_callback(self, msg):
        self.vel_cmd_pan = msg.angular.z
        self.vel_cmd_tilt = msg.angular.y
        self.last_vel_time = time.time()

    def joint_control_callback(self, msg):
        # メッセージ内の関節名リストから、対象の関節を探して目標値を更新
        for i, name in enumerate(msg.name):
            if name == JOINT_NAME_PAN:
                self.target_pan_rad = msg.position[i]
                # 位置指定が来た場合、速度制御による更新（自動移動）を止めるため
                # last_vel_time をリセットする（あるいはタイムアウトさせる）
                # ここでは、速度指令が来ていないことにするために時間を古くする
                self.last_vel_time = 0.0 
            elif name == JOINT_NAME_TILT:
                self.target_tilt_rad = msg.position[i]
                self.last_vel_time = 0.0

        if self.is_connected:
             self.write_position_rad(ID_PAN, self.target_pan_rad, PAN_HOME_PULSE, PAN_DIR, PAN_RAD_OFFSET)
             self.write_position_rad(ID_TILT, self.target_tilt_rad, TILT_HOME_PULSE, TILT_DIR, TILT_RAD_OFFSET)

    def timer_callback(self):
        if not self.is_connected:
            self.get_logger().warn("Connection lost. Retrying...", throttle_duration_sec=2.0)
            self.try_connect()
            return

        # 速度指令タイムアウト内であれば、速度分を加算処理する
        if (time.time() - self.last_vel_time) < VEL_TIMEOUT_SEC:
            self.target_pan_rad  += self.vel_cmd_pan  * CONTROL_PERIOD_SEC
            self.target_tilt_rad += self.vel_cmd_tilt * CONTROL_PERIOD_SEC
            self.write_position_rad(ID_PAN, self.target_pan_rad, PAN_HOME_PULSE, PAN_DIR, PAN_RAD_OFFSET)
            self.write_position_rad(ID_TILT, self.target_tilt_rad, TILT_HOME_PULSE, TILT_DIR, TILT_RAD_OFFSET)

        curr_pan, curr_tilt = self.publish_current_state()
        
        # 速度制御していないときは、現在位置をターゲットとして同期し続ける（手で動かした場合などの追従）
        if curr_pan is not None and curr_tilt is not None:
            if (time.time() - self.last_vel_time) >= VEL_TIMEOUT_SEC:
                 # ただし、joint_control_callback で位置指定された直後だと
                 # ここで上書きしてしまわないか注意が必要。
                 # 今回は joint_control_callback で書き込みを行っているので、
                 # ループ周期で現在値に上書きされると「保持」動作になる。
                 # 常にターゲット＝現在値 にすると、指令値への移動中に押し戻される可能性があるが、
                 # XL330は位置指令を受ければその位置に行こうとするので、
                 # ここで target を update してしまうと、移動途中の値が target になってしまうかもしれない。
                 
                 # 改良: 速度制御タイムアウト中かつ、明示的な位置指令も来ていない（=手動アイドリング）だけ同期する？
                 # 簡易的には、ターゲットと現在値の差分が大きければ移動中とみなせるが…
                 # いったん元のロジックを踏襲するなら、
                 # 「速度指令が来ていない」ときは「現在値をターゲットにする」というロジックは
                 # 「脱力」に近い挙動（外力で動かせる）を意図している可能性がある。
                 # ですが、Profile Velocity を設定しているので脱力はしていないはず。
                 # 今回の変更で JointControl 指令が来た場合も last_vel_time = 0 にしているので
                 # ここで target が現在値で上書きされてしまう。
                 
                 # 修正: JointControl制御時は、targetを上書きしないようにするフラグが必要かもしれないが、
                 # 既存ロジックは「速度指令タイムアウト外なら現在位置に追従（=ホールド）」に見える。
                 # move_servo_callbackも書き込みっぱなしで、timer_callbackで上書きされる恐れがある？
                 # move_servo_callback はブロッキングで待機しているので、その間 timer_callback は呼ばれない（シングルスレッドなら）。
                 
                 # JointControlCallback は非同期に来るため、その値をセットしても
                 # 直後の TimerCallback で上書きされると動かない。
                 
                 # 対策: 
                 # target_pan_rad / target_tilt_rad は「最終的な目標位置」として保持する。
                 # 「現在位置に追従」するのは、「目標位置が設定されていない」あるいは「外力で動かされた」場合などだが、
                 # ここではシンプルに「速度制御中のみ加算」とし、それ以外は「設定されたターゲット位置を維持」すべきである。
                 # しかし元のコードの意図（249-251行目）は「速度入力が途切れたら、その時点の場所を維持（現在値を読み取ってターゲットにする）」というもの。
                 # これだと、JointControl で目標を与えても、次の瞬間に「まだ動いていない現在値」で上書きされて止まってしまう。
                 
                 # したがって、 249-251行目のロジックは、
                 # 「JointControl での指令が直近になかった場合」に限定すべきである。
                 # あるいは、Targetと現在値の乖離が大きい場合は移動中とみなして更新しない、など。
                 
                 pass

    def __del__(self):
        try:
            if self.is_connected:
                self.packetHandler.write1ByteTxRx(self.portHandler, ID_PAN, ADDR_TORQUE_ENABLE, 0)
                self.packetHandler.write1ByteTxRx(self.portHandler, ID_TILT, ADDR_TORQUE_ENABLE, 0)
            self.portHandler.closePort()
        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = PanTiltNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
