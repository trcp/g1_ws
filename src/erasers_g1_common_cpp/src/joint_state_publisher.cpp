#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <unitree_hg/msg/low_state.hpp>  // G1 (hg) のメッセージタイプに変更
#include <string>
#include <vector>
#include <memory>
#include <map> // マッピングのために追加

using std::placeholders::_1;

class JointStatePublisher : public rclcpp::Node
{
public:
  JointStatePublisher()
  : Node("joint_state_publisher")
  {
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "/joint_states", qos_profile);

    // サブスクライブするメッセージタイプを G1 (hg) 用に変更
    low_state_sub_ = this->create_subscription<unitree_hg::msg::LowState>(
      "/lowstate", qos_profile, std::bind(&JointStatePublisher::lowStateCallback, this, _1));

    // URDF (g1_23dof.urdf) に基づく 23個の関節名を定義
    joint_names_ = {
      // Left Leg (6)
      "left_hip_pitch_joint",
      "left_hip_roll_joint",
      "left_hip_yaw_joint",
      "left_knee_joint",
      "left_ankle_pitch_joint",
      "left_ankle_roll_joint",
      // Right Leg (6)
      "right_hip_pitch_joint",
      "right_hip_roll_joint",
      "right_hip_yaw_joint",
      "right_knee_joint",
      "right_ankle_pitch_joint",
      "right_ankle_roll_joint",
      // Waist (1)
      "waist_yaw_joint",
      // Left Arm (5)
      "left_shoulder_pitch_joint",
      "left_shoulder_roll_joint",
      "left_shoulder_yaw_joint",
      "left_elbow_joint",
      "left_wrist_roll_joint",
      // Right Arm (5)
      "right_shoulder_pitch_joint",
      "right_shoulder_roll_joint",
      "right_shoulder_yaw_joint",
      "right_elbow_joint",
      "right_wrist_roll_joint"
    };

    num_joints_ = joint_names_.size(); // 23 になる
    
    // Pythonコード (G1JointIndex) に基づき、URDFの関節名とmotor_stateのインデックスをマッピング
    // URDFに存在しない関節 (WaistRoll/Pitch, WristPitch/Yaw) はスキップ
    joint_to_index_map_["left_hip_pitch_joint"] = 0;
    joint_to_index_map_["left_hip_roll_joint"] = 1;
    joint_to_index_map_["left_hip_yaw_joint"] = 2;
    joint_to_index_map_["left_knee_joint"] = 3;
    joint_to_index_map_["left_ankle_pitch_joint"] = 4;
    joint_to_index_map_["left_ankle_roll_joint"] = 5;
    joint_to_index_map_["right_hip_pitch_joint"] = 6;
    joint_to_index_map_["right_hip_roll_joint"] = 7;
    joint_to_index_map_["right_hip_yaw_joint"] = 8;
    joint_to_index_map_["right_knee_joint"] = 9;
    joint_to_index_map_["right_ankle_pitch_joint"] = 10;
    joint_to_index_map_["right_ankle_roll_joint"] = 11;
    joint_to_index_map_["waist_yaw_joint"] = 12;
    // index 13 (WaistRoll), 14 (WaistPitch) は 23DOF URDF にないのでスキップ
    joint_to_index_map_["left_shoulder_pitch_joint"] = 15;
    joint_to_index_map_["left_shoulder_roll_joint"] = 16;
    joint_to_index_map_["left_shoulder_yaw_joint"] = 17;
    joint_to_index_map_["left_elbow_joint"] = 18;
    joint_to_index_map_["left_wrist_roll_joint"] = 19;
    // index 20 (LeftWristPitch), 21 (LeftWristYaw) はスキップ
    joint_to_index_map_["right_shoulder_pitch_joint"] = 22;
    joint_to_index_map_["right_shoulder_roll_joint"] = 23;
    joint_to_index_map_["right_shoulder_yaw_joint"] = 24;
    joint_to_index_map_["right_elbow_joint"] = 25;
    joint_to_index_map_["right_wrist_roll_joint"] = 26;
    // index 27 (RightWristPitch), 28 (RightWristYaw) はスキップ

    RCLCPP_INFO(this->get_logger(), "JointStatePublisher (G1 23DOF) started. Publishing %zu joints (BEST_EFFORT QoS).", num_joints_);
  }

private:
  // コールバックのメッセージタイプを G1 (hg) 用に変更
  void lowStateCallback(const unitree_hg::msg::LowState::SharedPtr msg)
  {
    auto joint_state_msg = sensor_msgs::msg::JointState();

    joint_state_msg.header.stamp = this->get_clock()->now();

    // 23関節分の領域を確保
    joint_state_msg.name.resize(num_joints_);
    joint_state_msg.position.resize(num_joints_);
    joint_state_msg.velocity.resize(num_joints_);
    joint_state_msg.effort.resize(num_joints_);

    // 23個の関節名をループ処理
    for (size_t i = 0; i < num_joints_; ++i)
    {
      const std::string& joint_name = joint_names_[i];
      
      // マップから motor_state のインデックスを取得
      // .at() を使い、キーが存在しない場合は例外をスロー（デバッグに役立つ）
      try {
        int motor_index = joint_to_index_map_.at(joint_name);

        // LowStateメッセージは35個のモーターステートを持つ
        if (motor_index < 35) 
        {
          // マッピングに基づき、正しいインデックスからデータを取得
          joint_state_msg.name[i] = joint_name;
          joint_state_msg.position[i] = msg->motor_state[motor_index].q;     // 位置 (rad)
          joint_state_msg.velocity[i] = msg->motor_state[motor_index].dq;    // 速度 (rad/s)
          joint_state_msg.effort[i] = msg->motor_state[motor_index].tau_est; // 推定トルク (N*m)
        }
      } catch (const std::out_of_range& e) {
        RCLCPP_WARN_ONCE(this->get_logger(), "Joint name '%s' not found in joint_to_index_map_.", joint_name.c_str());
      }
    }

    joint_state_pub_->publish(joint_state_msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr low_state_sub_; // G1 (hg) 用
  
  std::vector<std::string> joint_names_;
  std::map<std::string, int> joint_to_index_map_; // 関節名とインデックスのマッピング
  size_t num_joints_; // 23
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointStatePublisher>());
  rclcpp::shutdown();
  return 0;
}
