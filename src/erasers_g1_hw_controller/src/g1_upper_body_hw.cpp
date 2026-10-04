#include "erasers_g1_hw_controller/g1_upper_body_hw.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include "erasers_g1_common/g1_joint_descriptor.hpp"

namespace erasers_g1_hw_controller
{
hardware_interface::CallbackReturn G1UpperBodyHW::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS) {return hardware_interface::CallbackReturn::ERROR;}
  const auto & descriptors = erasers_g1_common::g1JointDescriptors();
  std::vector<erasers_g1_common::G1JointControlLimit> limits;
  std::string error;
  if (!erasers_g1_common::loadG1JointLimits(info.original_xml, limits, error)) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  for (const auto & joint : info_.joints) {
    const auto descriptor = std::find_if(descriptors.begin() + 12, descriptors.end(),
      [&](const auto & d) {return joint.name == d.name;});
    if (descriptor == descriptors.end() || joint_map_.count(joint.name) ||
      !limits.at(descriptor->motor_index).active)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }
    joint_map_[joint.name] = descriptor->motor_index;
  }
  hw_states_.assign(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_ = hw_states_;
  node_ = std::make_shared<rclcpp::Node>("g1_upper_body_hw_node");
  enable_client_ = node_->create_client<Enable>("/robot_controller/upper_body/enable");
  enable_service_ = node_->create_service<Enable>("/enable_upper_body_control",
    [this](std::shared_ptr<rclcpp::Service<Enable>> service,
      std::shared_ptr<rmw_request_id_t> header, std::shared_ptr<Enable::Request> request) {
      enableControlCallback(service, header, request);
    });
  joint_command_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
    "/upper_joints_control", rclcpp::QoS(1).reliable());
  low_state_sub_ = node_->create_subscription<unitree_hg::msg::LowState>(
    "/lowstate", rclcpp::QoS(1).best_effort(),
    std::bind(&G1UpperBodyHW::lowStateCallback, this, std::placeholders::_1));
  emergency_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/emergency_stop/active", rclcpp::QoS(1).reliable().transient_local(),
    [this](std_msgs::msg::Bool::ConstSharedPtr message) {
      if (message->data) {
        emergency_latched_ = true;
        control_enabled_ = false;
        ++generation_;
      }
    });
  emergency_latch_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/emergency_stop/latched", rclcpp::QoS(1).reliable().transient_local(),
    [this](std_msgs::msg::Bool::ConstSharedPtr message) {
      if (message->data) {
        emergency_latched_ = true;
        control_enabled_ = false;
        ++generation_;
      }
    });
  pending_timer_ = node_->create_wall_timer(std::chrono::milliseconds(100), [this]() {
    if (pending_ && Clock::now() >= pending_->deadline) {
      enable_client_->remove_pending_request(pending_->request_id);
      auto response = std::make_shared<Enable::Response>();
      response->success = false;
      response->message = "制御ノードからの許可がタイムアウトしました";
      pending_->service->send_response(*pending_->header, *response);
      pending_.reset();
      control_enabled_ = false;
    }
  });
  return hardware_interface::CallbackReturn::SUCCESS;
}
std::vector<hardware_interface::StateInterface> G1UpperBodyHW::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> result;
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    result.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]);
  }
  return result;
}
std::vector<hardware_interface::CommandInterface> G1UpperBodyHW::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> result;
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    result.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]);
  }
  return result;
}
hardware_interface::CallbackReturn G1UpperBodyHW::on_activate(const rclcpp_lifecycle::State &)
{
  control_enabled_ = false;
  hw_commands_ = hw_states_;
  return hardware_interface::CallbackReturn::SUCCESS;
}
hardware_interface::CallbackReturn G1UpperBodyHW::on_deactivate(const rclcpp_lifecycle::State &)
{
  control_enabled_ = false;
  ++generation_;
  // 下流は入力途絶を検出して制御権を解放する。
  return hardware_interface::CallbackReturn::SUCCESS;
}
hardware_interface::return_type G1UpperBodyHW::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  rclcpp::spin_some(node_);
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    const auto state = feedback_map_.find(info_.joints[i].name);
    if (state != feedback_map_.end()) {hw_states_[i] = state->second;}
    if (!control_enabled_) {hw_commands_[i] = hw_states_[i];}
  }
  return hardware_interface::return_type::OK;
}
hardware_interface::return_type G1UpperBodyHW::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!control_enabled_) {return hardware_interface::return_type::OK;}
  if (emergency_latched_ || Clock::now() - feedback_time_ > std::chrono::milliseconds(200)) {
    control_enabled_ = false;
    return hardware_interface::return_type::ERROR;
  }
  sensor_msgs::msg::JointState message;
  message.header.stamp = node_->now();
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    if (!std::isfinite(hw_commands_[i])) {return hardware_interface::return_type::ERROR;}
    message.name.push_back(info_.joints[i].name);
    message.position.push_back(hw_commands_[i]);
  }
  joint_command_pub_->publish(message);
  return hardware_interface::return_type::OK;
}
void G1UpperBodyHW::lowStateCallback(unitree_hg::msg::LowState::SharedPtr message)
{
  std::map<std::string, double> candidate;
  for (const auto & joint : joint_map_) {
    const double q = message->motor_state.at(joint.second).q;
    if (!std::isfinite(q)) {return;}
    candidate[joint.first] = q;
  }
  feedback_map_ = std::move(candidate);
  feedback_time_ = Clock::now();
}
void G1UpperBodyHW::enableControlCallback(
  std::shared_ptr<rclcpp::Service<Enable>> service,
  std::shared_ptr<rmw_request_id_t> header, std::shared_ptr<Enable::Request> request)
{
  auto response = std::make_shared<Enable::Response>();
  if (pending_ || emergency_latched_ || !enable_client_->service_is_ready()) {
    response->success = false;
    response->message = "制御ノードが利用できないか、別の要求・緊急停止が有効です";
    service->send_response(*header, *response);
    return;
  }
  if (!request->data) {control_enabled_ = false; ++generation_;}
  const auto generation = ++generation_;
  auto future = enable_client_->async_send_request(request,
    [this, generation](rclcpp::Client<Enable>::SharedFuture result) {
      if (!pending_ || pending_->generation != generation) {return;}
      auto response = std::make_shared<Enable::Response>();
      try {*response = *result.get();}
      catch (const std::exception & error) {response->success = false; response->message = error.what();}
      if (pending_->enable) {
        response->success = response->success && generation_ == generation &&
          !emergency_latched_ && feedback_time_ != Clock::time_point{} &&
          Clock::now() - feedback_time_ <= std::chrono::milliseconds(200);
        if (response->success) {
          for (std::size_t i = 0; i < info_.joints.size(); ++i) {
            hw_commands_[i] = feedback_map_.at(info_.joints[i].name);
          }
        }
        control_enabled_ = response->success;
      }
      pending_->service->send_response(*pending_->header, *response);
      pending_.reset();
    });
  pending_ = Pending{service, header, future.request_id, generation, request->data,
    Clock::now() + std::chrono::seconds(8)};
}
}  // namespace erasers_g1_hw_controller

PLUGINLIB_EXPORT_CLASS(erasers_g1_hw_controller::G1UpperBodyHW, hardware_interface::SystemInterface)
