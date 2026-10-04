#ifndef ERASERS_G1_COMMON__ROBOT_MANAGER_HPP_
#define ERASERS_G1_COMMON__ROBOT_MANAGER_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <unitree_hg/msg/low_state.hpp>
#include <unitree_hg/msg/bms_state.hpp>

#include "erasers_g1_common/lowstate_adapter.hpp"
#include "erasers_g1_common/g1_joint_descriptor.hpp"

namespace erasers_g1_common
{

class RobotManagerNode : public rclcpp::Node
{
public:
  explicit RobotManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~RobotManagerNode() override = default;

private:
  void declare_parameters();
  void load_parameters();
  void lowstate_callback(const unitree_hg::msg::LowState::SharedPtr msg);
  void bms_callback(const unitree_hg::msg::BmsState::SharedPtr msg);
  void diag_timer_callback();
  void joint_timer_callback();

  std::string lowstate_topic_;
  std::string bms_topic_;
  std::string joint_states_topic_;
  std::string imu_topic_;
  std::string battery_topic_;
  std::string diagnostics_topic_;

  bool publish_joint_states_{true};
  bool publish_imu_{true};
  bool publish_battery_{true};
  bool publish_diagnostics_{true};

  double joint_publish_rate_hz_{30.0};
  double imu_publish_rate_hz_{200.0};
  double diagnostics_publish_rate_hz_{1.0};

  AdapterConfig adapter_config_;
  JointMappingConfig mapping_config_;
  bool mapping_valid_{false};
  std::string mapping_error_;

  rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr lowstate_sub_;
  rclcpp::Subscription<unitree_hg::msg::BmsState>::SharedPtr bms_sub_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;

  rclcpp::TimerBase::SharedPtr diag_timer_;
  rclcpp::TimerBase::SharedPtr joint_timer_;

  RateLimiter imu_rate_limiter_;

  std::mutex state_mutex_;
  unitree_hg::msg::LowState last_lowstate_;
  unitree_hg::msg::BmsState last_bms_;
  rclcpp::Time last_lowstate_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_bms_time_{0, 0, RCL_ROS_TIME};
  bool lowstate_received_{false};
  bool bms_received_{false};
  bool bms_was_stale_{false};
  bool last_imu_valid_{true};
  std::string last_imu_warning_;
};

}

#endif  // ERASERS_G1_COMMON__ROBOT_MANAGER_HPP_

