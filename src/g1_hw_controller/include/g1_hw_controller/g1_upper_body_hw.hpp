#ifndef G1_HW_CONTROLLER__G1_UPPER_BODY_HW_HPP_
#define G1_HW_CONTROLLER__G1_UPPER_BODY_HW_HPP_

#include <memory>
#include <string>
#include <vector>
#include <map>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "unitree_hg/msg/low_state.hpp"

namespace g1_hw_controller
{
class G1UpperBodyHW : public hardware_interface::SystemInterface
{
public:
  RCLCPP_UNIQUE_PTR_DEFINITIONS(G1UpperBodyHW)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // ROS 2 Node for communication
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_command_pub_;
  rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr low_state_sub_;

  // Store joint states and commands
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;
  std::map<std::string, int> joint_map_;
  std::map<int, double> feedback_states_;

  void lowStateCallback(const unitree_hg::msg::LowState::SharedPtr msg);
};

}  // namespace g1_hw_controller

#endif  // G1_HW_CONTROLLER__G1_UPPER_BODY_HW_HPP_
