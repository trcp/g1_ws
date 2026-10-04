#ifndef ERASERS_G1_COMMON__LOWSTATE_ADAPTER_HPP_
#define ERASERS_G1_COMMON__LOWSTATE_ADAPTER_HPP_

#include <chrono>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>
#include <set>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <unitree_hg/msg/low_state.hpp>
#include <unitree_hg/msg/bms_state.hpp>

namespace erasers_g1_common
{

struct JointMappingConfig
{
  std::vector<std::string> joint_names;
  std::vector<int> motor_indices;
  std::vector<bool> active;

  bool isValid(std::string & error_msg) const;
};

enum class InvalidSamplePolicy
{
  DROP,
  NAN_FILL
};

InvalidSamplePolicy parseInvalidSamplePolicy(const std::string & policy_str);

struct AdapterConfig
{
  std::string imu_frame_id{"imu_in_torso"};
  std::string joint_state_frame_id{""};
  double angular_velocity_scale{1.0};
  double linear_acceleration_scale{1.0};
  uint32_t expected_tick_step{0};
  InvalidSamplePolicy invalid_sample_policy{InvalidSamplePolicy::DROP};

  double lowstate_stale_timeout_sec{0.2};
  double bms_stale_timeout_sec{2.0};
};

struct RateLimiter
{
  double rate_hz{100.0};
  rclcpp::Time last_pub_time{0, 0, RCL_ROS_TIME};
  uint32_t last_tick{0};
  bool has_tick{false};

  bool shouldPublish(double target_rate_hz, const rclcpp::Time & now, uint32_t current_tick);
};

bool validateQuaternion(double x, double y, double z, double w);

void setNormalizedQuaternion(sensor_msgs::msg::Imu & imu_msg, float w, float x, float y, float z);

bool convertImu(
  const unitree_hg::msg::LowState & lowstate,
  const rclcpp::Time & stamp,
  const AdapterConfig & config,
  sensor_msgs::msg::Imu & imu_msg,
  std::string & diag_warning);

bool convertJointState(
  const unitree_hg::msg::LowState & lowstate,
  const rclcpp::Time & stamp,
  const JointMappingConfig & mapping,
  const AdapterConfig & config,
  sensor_msgs::msg::JointState & joint_msg,
  std::string & diag_error);

void convertBatteryState(
  const unitree_hg::msg::BmsState & bms_state,
  const rclcpp::Time & stamp,
  sensor_msgs::msg::BatteryState & battery_msg);

sensor_msgs::msg::BatteryState createStaleBatteryState(const rclcpp::Time & stamp);

diagnostic_msgs::msg::DiagnosticArray createDiagnostics(
  const rclcpp::Time & stamp,
  const std::string & node_name,
  bool lowstate_received,
  bool lowstate_stale,
  uint32_t lowstate_tick,
  const unitree_hg::msg::LowState & last_lowstate,
  bool mapping_valid,
  const std::string & mapping_error,
  bool imu_valid,
  const std::string & imu_warning,
  bool bms_received,
  bool bms_stale,
  const unitree_hg::msg::BmsState & last_bms,
  const JointMappingConfig & mapping);

}

#endif  // ERASERS_G1_COMMON__LOWSTATE_ADAPTER_HPP_

