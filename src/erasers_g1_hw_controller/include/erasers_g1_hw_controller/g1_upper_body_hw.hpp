#pragma once

#include <chrono>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <unitree_hg/msg/low_state.hpp>

namespace erasers_g1_hw_controller
{
class G1UpperBodyHW : public hardware_interface::SystemInterface
{
public:
  RCLCPP_UNIQUE_PTR_DEFINITIONS(G1UpperBodyHW)
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;
private:
  using Enable = std_srvs::srv::SetBool;
  using Clock = std::chrono::steady_clock;
  struct Pending
  {
    std::shared_ptr<rclcpp::Service<Enable>> service;
    std::shared_ptr<rmw_request_id_t> header;
    int64_t request_id;
    uint64_t generation;
    bool enable;
    Clock::time_point deadline;
  };
  void lowStateCallback(unitree_hg::msg::LowState::SharedPtr);
  void enableControlCallback(std::shared_ptr<rclcpp::Service<Enable>>,
    std::shared_ptr<rmw_request_id_t>, std::shared_ptr<Enable::Request>);
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_command_pub_;
  rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr low_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_latch_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr transition_sub_;
  rclcpp::Service<Enable>::SharedPtr enable_service_;
  rclcpp::Client<Enable>::SharedPtr enable_client_;
  rclcpp::TimerBase::SharedPtr pending_timer_;
  std::optional<Pending> pending_;
  bool control_enabled_{false};
  bool emergency_latched_{false};
  uint64_t generation_{0};
  Clock::time_point feedback_time_{};
  std::vector<double> hw_commands_, hw_states_;
  std::map<std::string, std::size_t> joint_map_;
  std::map<std::string, double> feedback_map_;
};
}  // namespace erasers_g1_hw_controller
