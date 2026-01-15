#include <memory>
#include <string>
#include <vector>
#include <map>
#include <cmath>
#include <mutex>
#include <urdf/model.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "unitree_hg/msg/low_cmd.hpp"
#include "unitree_hg/msg/low_state.hpp"
#include "g1/g1.hpp"

using LowCmd = unitree_hg::msg::LowCmd;
using LowState = unitree_hg::msg::LowState;

class ArmJointControl : public rclcpp::Node
{
public:
  ArmJointControl() : Node("arm_joint_control")
  {
    // URDF parameter
    this->declare_parameter("urdf", "/home/unitree/colcon_ws/src/erasers_g1/g1_description/urdf/g1_comp.urdf");
    this->declare_parameter("control_frequency", 100.0);
    this->declare_parameter("max_joint_velocity", 2.0); // rad/s

    std::string urdf_file = this->get_parameter("urdf").as_string();
    control_frequency_ = this->get_parameter("control_frequency").as_double();
    max_joint_velocity_ = this->get_parameter("max_joint_velocity").as_double();
    control_dt_ = 1.0 / control_frequency_;

    // Load URDF
    urdf::Model model;
    if (!model.initFile(urdf_file))
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF file: %s", urdf_file.c_str());
      return; 
    }
    RCLCPP_INFO(this->get_logger(), "Successfully parsed URDF file: %s", urdf_file.c_str());

    // Publisher
    pub_ = this->create_publisher<LowCmd>("/arm_sdk", 10);

    // Subscriber for LowState (to get initial positions)
    low_state_sub_ = this->create_subscription<LowState>(
      "/lowstate", 10,
      std::bind(&ArmJointControl::lowStateCallback, this, std::placeholders::_1));

    // Subscriber for JointState (Control target)
    joint_target_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/arm_joint_control", 10,
      std::bind(&ArmJointControl::jointStateCallback, this, std::placeholders::_1));

    // Timer for control loop
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(control_dt_),
      std::bind(&ArmJointControl::controlLoop, this));

    // Initialize joint mapping (URDF joint name -> G1 Motor Index)
    initializeJointMapping();
  }

private:
  void initializeJointMapping()
  {
    joint_map_["left_shoulder_pitch_joint"] = static_cast<int>(G1Arm5JointIndex::LEFT_SHOULDER_PITCH);
    joint_map_["left_shoulder_roll_joint"] = static_cast<int>(G1Arm5JointIndex::LEFT_SHOULDER_ROLL);
    joint_map_["left_shoulder_yaw_joint"] = static_cast<int>(G1Arm5JointIndex::LEFT_SHOULDER_YAW);
    joint_map_["left_elbow_joint"] = static_cast<int>(G1Arm5JointIndex::LEFT_ELBOW_PITCH);
    joint_map_["left_wrist_roll_joint"] = static_cast<int>(G1Arm5JointIndex::LEFT_ELBOW_ROLL);

    joint_map_["right_shoulder_pitch_joint"] = static_cast<int>(G1Arm5JointIndex::RIGHT_SHOULDER_PITCH);
    joint_map_["right_shoulder_roll_joint"] = static_cast<int>(G1Arm5JointIndex::RIGHT_SHOULDER_ROLL);
    joint_map_["right_shoulder_yaw_joint"] = static_cast<int>(G1Arm5JointIndex::RIGHT_SHOULDER_YAW);
    joint_map_["right_elbow_joint"] = static_cast<int>(G1Arm5JointIndex::RIGHT_ELBOW_PITCH);
    joint_map_["right_wrist_roll_joint"] = static_cast<int>(G1Arm5JointIndex::RIGHT_ELBOW_ROLL);

    // Waist mappings
    joint_map_["waist_yaw_joint"] = static_cast<int>(G1Arm5JointIndex::WAIST_YAW);
    joint_map_["waist_roll_joint"] = static_cast<int>(G1Arm5JointIndex::WAIST_ROLL);
    joint_map_["waist_pitch_joint"] = static_cast<int>(G1Arm5JointIndex::WAIST_PITCH);
  }

  void lowStateCallback(const LowState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // Initialize current commands with actual feedback positions only once
    if (!initialized_)
    {
      for (const auto& pair : joint_map_)
      {
        int idx = pair.second;
        if (idx >= 0 && idx < 35) // Bounds check based on G1 motor count
        {
           double current_pos = msg->motor_state[idx].q;
           current_cmd_[idx] = current_pos;
           target_pos_[idx] = current_pos; // Set initial target to current pos
        }
      }
      initialized_ = true;
      RCLCPP_INFO(this->get_logger(), "Initialized joint positions from LowState.");
    }
  }

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!initialized_) return; // Don't accept targets until initialized

    for (size_t i = 0; i < msg->name.size(); ++i)
    {
      if (joint_map_.find(msg->name[i]) != joint_map_.end())
      {
        int motor_idx = joint_map_[msg->name[i]];
        if (motor_idx >= 0 && motor_idx < 35)
        {
            target_pos_[motor_idx] = msg->position[i];
        }
      }
    }
  }

  void controlLoop()
  {
    if (!initialized_) return;

    std::lock_guard<std::mutex> lock(data_mutex_);
    LowCmd cmd;
    bool has_active_joints = false;

    double max_delta = max_joint_velocity_ * control_dt_;

    for (const auto& pair : joint_map_)
    {
        int idx = pair.second;
        if (target_pos_.find(idx) != target_pos_.end())
        {
            double target = target_pos_[idx];
            double current = current_cmd_[idx];

            // Smooth interpolation (Ramping)
            double diff = target - current;
            double step = 0.0;

            if (std::abs(diff) > max_delta)
            {
                step = (diff > 0) ? max_delta : -max_delta;
            }
            else
            {
                step = diff;
            }

            current += step;
            current_cmd_[idx] = current;

            // Populate command
            cmd.motor_cmd[idx].q = current;
            cmd.motor_cmd[idx].dq = 0.0;
            cmd.motor_cmd[idx].kp = 60.0;
            cmd.motor_cmd[idx].kd = 1.5;
            cmd.motor_cmd[idx].tau = 0.0;
            
            has_active_joints = true;
        }
    }

    if (has_active_joints)
    {
        // Set control flag for Arm SDK
        cmd.motor_cmd[static_cast<int>(G1Arm5JointIndex::NOT_USED_JOINT)].q = 1.0F; 
        pub_->publish(cmd);
    }
  }

  rclcpp::Publisher<LowCmd>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_target_sub_;
  rclcpp::Subscription<LowState>::SharedPtr low_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  std::map<std::string, int> joint_map_;
  std::map<int, double> current_cmd_;
  std::map<int, double> target_pos_;
  
  std::mutex data_mutex_;
  bool initialized_ = false;
  double control_frequency_;
  double control_dt_;
  double max_joint_velocity_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmJointControl>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
