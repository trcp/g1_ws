#include "g1_hw_controller/g1_upper_body_hw.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "g1/g1.hpp"

namespace g1_hw_controller
{
hardware_interface::CallbackReturn G1UpperBodyHW::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  // Initialize joint mapping from G1.hpp (same as arm_joint_control.cpp)
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

  joint_map_["waist_yaw_joint"] = static_cast<int>(G1Arm5JointIndex::WAIST_YAW);
  joint_map_["waist_roll_joint"] = static_cast<int>(G1Arm5JointIndex::WAIST_ROLL);
  joint_map_["waist_pitch_joint"] = static_cast<int>(G1Arm5JointIndex::WAIST_PITCH);

  // Initialize ROS 2 Node
  node_ = std::make_shared<rclcpp::Node>("g1_upper_body_hw_node");
  
  enable_service_ = node_->create_service<std_srvs::srv::SetBool>(
    "/enable_upper_body_control",
    std::bind(&G1UpperBodyHW::enableControlCallback, this, std::placeholders::_1, std::placeholders::_2));
    
  joint_command_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("/upper_joints_control", 10);
  
  rclcpp::QoS qos_profile(10);
  qos_profile.best_effort();
  low_state_sub_ = node_->create_subscription<unitree_hg::msg::LowState>(
    "/lowstate", qos_profile, std::bind(&G1UpperBodyHW::lowStateCallback, this, std::placeholders::_1));

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> G1UpperBodyHW::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> G1UpperBodyHW::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn G1UpperBodyHW::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Set current position as initial command
  for (size_t i = 0; i < hw_states_.size(); ++i)
  {
    if (std::isnan(hw_states_[i]))
    {
      hw_states_[i] = 0.0;
    }
    hw_commands_[i] = hw_states_[i];
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn G1UpperBodyHW::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type G1UpperBodyHW::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Spin to get latest subscription data
  rclcpp::spin_some(node_);

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    const auto & joint_name = info_.joints[i].name;
    if (feedback_map_.find(joint_name) != feedback_map_.end())
    {
      hw_states_[i] = feedback_map_[joint_name];
    }
    
    // When disabled, sync commands with current physical states to prevent jumping upon re-enable
    if (!control_enabled_) {
      hw_commands_[i] = hw_states_[i];
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type G1UpperBodyHW::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!control_enabled_) {
    return hardware_interface::return_type::OK; // Silence topic so downstream arm controller handles fading
  }

  auto msg = std::make_unique<sensor_msgs::msg::JointState>();
  msg->header.stamp = node_->now();

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    if (!std::isnan(hw_commands_[i]))
    {
      msg->name.push_back(info_.joints[i].name);
      msg->position.push_back(hw_commands_[i]);
    }
  }

  if (!msg->name.empty())
  {
    joint_command_pub_->publish(std::move(msg));
  }

  return hardware_interface::return_type::OK;
}

void G1UpperBodyHW::lowStateCallback(const unitree_hg::msg::LowState::SharedPtr msg)
{
  for (const auto & pair : joint_map_)
  {
    const std::string & joint_name = pair.first;
    int index = pair.second;
    if (index >= 0 && index < 35)
    {
      feedback_map_[joint_name] = msg->motor_state[index].q;
    }
  }
}

void G1UpperBodyHW::enableControlCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                          std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  control_enabled_ = request->data;
  response->success = true;
  response->message = control_enabled_ ? "Upper body control enabled" : "Upper body control disabled";
  RCLCPP_INFO(node_->get_logger(), "%s", response->message.c_str());
}

}  // namespace g1_hw_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(g1_hw_controller::G1UpperBodyHW, hardware_interface::SystemInterface)
