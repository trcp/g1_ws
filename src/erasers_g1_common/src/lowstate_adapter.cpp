#include "erasers_g1_common/lowstate_adapter.hpp"

#include <algorithm>
#include <sstream>
#include <limits>
#include <iomanip>

namespace erasers_g1_common
{

bool JointMappingConfig::isValid(std::string & error_msg) const
{
  if (joint_names.size() != motor_indices.size() || active.size() != joint_names.size()) {
    error_msg = "joint_names size (" + std::to_string(joint_names.size()) +
      ") does not match motor_indices size (" + std::to_string(motor_indices.size()) + ")";
    return false;
  }
  if (joint_names.empty()) {
    error_msg = "joint_names mapping is empty";
    return false;
  }

  std::set<std::string> name_set;
  std::set<int> index_set;

  for (size_t i = 0; i < joint_names.size(); ++i) {
    const auto & name = joint_names[i];
    int idx = motor_indices[i];

    if (name.empty()) {
      error_msg = "joint_names element at index " + std::to_string(i) + " is empty";
      return false;
    }
    if (name_set.count(name) > 0) {
      error_msg = "duplicate joint_name found: '" + name + "'";
      return false;
    }
    name_set.insert(name);

    if (idx < 0 || idx >= 35) {
      error_msg = "motor_index out of range [0, 34]: " + std::to_string(idx) + " for joint '" + name + "'";
      return false;
    }
    if (index_set.count(idx) > 0) {
      error_msg = "duplicate motor_index found: " + std::to_string(idx);
      return false;
    }
    index_set.insert(idx);
  }

  error_msg = "";
  return true;
}

InvalidSamplePolicy parseInvalidSamplePolicy(const std::string & policy_str)
{
  if (policy_str == "nan") {
    return InvalidSamplePolicy::NAN_FILL;
  }
  return InvalidSamplePolicy::DROP;
}

bool RateLimiter::shouldPublish(double target_rate_hz, const rclcpp::Time & now, uint32_t current_tick)
{
  if (target_rate_hz <= 0.0) {
    return false;
  }

  if (has_tick && current_tick == last_tick) {
    return false;
  }

  if (last_pub_time.nanoseconds() == 0) {
    last_pub_time = now;
    last_tick = current_tick;
    has_tick = true;
    return true;
  }

  double dt = (now - last_pub_time).seconds();
  double min_dt = 1.0 / target_rate_hz - 1e-6;

  if (dt < 0.0 || dt >= min_dt) {
    last_pub_time = now;
    last_tick = current_tick;
    has_tick = true;
    return true;
  }

  return false;
}

bool validateQuaternion(double x, double y, double z, double w)
{
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || !std::isfinite(w)) {
    return false;
  }
  double norm_sq = x * x + y * y + z * z + w * w;
  if (norm_sq < 1e-6 || !std::isfinite(norm_sq)) {
    return false;
  }
  return true;
}

void setNormalizedQuaternion(sensor_msgs::msg::Imu & imu_msg, float w, float x, float y, float z)
{
  double qx = static_cast<double>(x);
  double qy = static_cast<double>(y);
  double qz = static_cast<double>(z);
  double qw = static_cast<double>(w);

  double norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
  if (norm < 1e-6) {
    imu_msg.orientation.x = 0.0;
    imu_msg.orientation.y = 0.0;
    imu_msg.orientation.z = 0.0;
    imu_msg.orientation.w = 1.0;
    imu_msg.orientation_covariance[0] = -1.0;
  } else {
    imu_msg.orientation.x = qx / norm;
    imu_msg.orientation.y = qy / norm;
    imu_msg.orientation.z = qz / norm;
    imu_msg.orientation.w = qw / norm;
    std::fill(imu_msg.orientation_covariance.begin(), imu_msg.orientation_covariance.end(), 0.0);
  }
}

bool convertImu(
  const unitree_hg::msg::LowState & lowstate,
  const rclcpp::Time & stamp,
  const AdapterConfig & config,
  sensor_msgs::msg::Imu & imu_msg,
  std::string & diag_warning)
{
  diag_warning.clear();
  imu_msg.header.stamp = stamp;
  imu_msg.header.frame_id = config.imu_frame_id;

  const auto & imu_state = lowstate.imu_state;

  bool gyro_valid = std::isfinite(imu_state.gyroscope[0]) &&
                    std::isfinite(imu_state.gyroscope[1]) &&
                    std::isfinite(imu_state.gyroscope[2]);

  bool accel_valid = std::isfinite(imu_state.accelerometer[0]) &&
                     std::isfinite(imu_state.accelerometer[1]) &&
                     std::isfinite(imu_state.accelerometer[2]);

  if (!gyro_valid || !accel_valid) {
    diag_warning = "IMU gyro or acceleration contain non-finite values";
    return false;
  }

  imu_msg.angular_velocity.x = imu_state.gyroscope[0] * config.angular_velocity_scale;
  imu_msg.angular_velocity.y = imu_state.gyroscope[1] * config.angular_velocity_scale;
  imu_msg.angular_velocity.z = imu_state.gyroscope[2] * config.angular_velocity_scale;
  std::fill(imu_msg.angular_velocity_covariance.begin(), imu_msg.angular_velocity_covariance.end(), 0.0);

  imu_msg.linear_acceleration.x = imu_state.accelerometer[0] * config.linear_acceleration_scale;
  imu_msg.linear_acceleration.y = imu_state.accelerometer[1] * config.linear_acceleration_scale;
  imu_msg.linear_acceleration.z = imu_state.accelerometer[2] * config.linear_acceleration_scale;
  std::fill(imu_msg.linear_acceleration_covariance.begin(), imu_msg.linear_acceleration_covariance.end(), 0.0);

  float qw = imu_state.quaternion[0];
  float qx = imu_state.quaternion[1];
  float qy = imu_state.quaternion[2];
  float qz = imu_state.quaternion[3];

  if (validateQuaternion(qx, qy, qz, qw)) {
    setNormalizedQuaternion(imu_msg, qw, qx, qy, qz);
  } else {
    diag_warning = "Invalid IMU quaternion (norm zero or non-finite). Setting covariance[0] = -1.0";
    imu_msg.orientation.x = 0.0;
    imu_msg.orientation.y = 0.0;
    imu_msg.orientation.z = 0.0;
    imu_msg.orientation.w = 1.0;
    std::fill(imu_msg.orientation_covariance.begin(), imu_msg.orientation_covariance.end(), 0.0);
    imu_msg.orientation_covariance[0] = -1.0;
  }

  return true;
}

bool convertJointState(
  const unitree_hg::msg::LowState & lowstate,
  const rclcpp::Time & stamp,
  const JointMappingConfig & mapping,
  const AdapterConfig & config,
  sensor_msgs::msg::JointState & joint_msg,
  std::string & diag_error)
{
  diag_error.clear();
  std::string mapping_err;
  if (!mapping.isValid(mapping_err)) {
    diag_error = "Joint mapping invalid: " + mapping_err;
    return false;
  }

  joint_msg.header.stamp = stamp;
  joint_msg.header.frame_id = config.joint_state_frame_id;

  size_t n = mapping.joint_names.size();
  joint_msg.name = mapping.joint_names;
  joint_msg.position.resize(n);
  joint_msg.velocity.resize(n);
  joint_msg.effort.resize(n);

  bool has_nonfinite = false;

  for (size_t i = 0; i < n; ++i) {
    if (!mapping.active[i]) {
      joint_msg.position[i] = 0.0;
      joint_msg.velocity[i] = 0.0;
      joint_msg.effort[i] = 0.0;
      continue;
    }
    int idx = mapping.motor_indices[i];
    const auto & m = lowstate.motor_state[idx];

    bool sample_finite = std::isfinite(m.q) && std::isfinite(m.dq) && std::isfinite(m.tau_est);

    if (!sample_finite) {
      has_nonfinite = true;
      if (config.invalid_sample_policy == InvalidSamplePolicy::DROP) {
        diag_error = "Non-finite motor sample detected at index " + std::to_string(idx) +
          " for joint '" + mapping.joint_names[i] + "' (dropping JointState)";
        return false;
      } else {
        joint_msg.position[i] = std::numeric_limits<double>::quiet_NaN();
        joint_msg.velocity[i] = std::numeric_limits<double>::quiet_NaN();
        joint_msg.effort[i] = std::numeric_limits<double>::quiet_NaN();
      }
    } else {
      joint_msg.position[i] = static_cast<double>(m.q);
      joint_msg.velocity[i] = static_cast<double>(m.dq);
      joint_msg.effort[i] = static_cast<double>(m.tau_est);
    }
  }

  if (has_nonfinite) {
    diag_error = "Non-finite motor sample encountered, filled with NaNs";
  }

  return true;
}

void convertBatteryState(
  const unitree_hg::msg::BmsState & bms_state,
  const rclcpp::Time & stamp,
  sensor_msgs::msg::BatteryState & battery_msg)
{
  battery_msg.header.stamp = stamp;
  battery_msg.header.frame_id = "";

  battery_msg.voltage = std::numeric_limits<float>::quiet_NaN();
  battery_msg.temperature = std::numeric_limits<float>::quiet_NaN();
  battery_msg.current = std::numeric_limits<float>::quiet_NaN();
  battery_msg.charge = std::numeric_limits<float>::quiet_NaN();
  battery_msg.capacity = std::numeric_limits<float>::quiet_NaN();
  battery_msg.design_capacity = std::numeric_limits<float>::quiet_NaN();

  if (bms_state.soc <= 100) {
    battery_msg.percentage = static_cast<float>(bms_state.soc) / 100.0f;
  } else {
    battery_msg.percentage = std::numeric_limits<float>::quiet_NaN();
  }

  battery_msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
  battery_msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
  battery_msg.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
  battery_msg.present = true;

  battery_msg.cell_voltage.clear();
  battery_msg.cell_temperature.clear();
  battery_msg.location = "main_battery";
  battery_msg.serial_number = "";
}

sensor_msgs::msg::BatteryState createStaleBatteryState(const rclcpp::Time & stamp)
{
  sensor_msgs::msg::BatteryState msg;
  msg.header.stamp = stamp;
  msg.voltage = std::numeric_limits<float>::quiet_NaN();
  msg.percentage = std::numeric_limits<float>::quiet_NaN();
  msg.present = false;
  msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
  msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
  msg.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
  return msg;
}

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
  const JointMappingConfig & mapping)
{
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = stamp;

  {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = node_name + ": LowState";
    status.hardware_id = "Unitree_G1_LowState";

    if (!lowstate_received) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "No LowState message received";
    } else if (lowstate_stale) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "LowState message is stale";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "LowState receiving normally";
    }

    if (lowstate_received) {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = "tick"; kv.value = std::to_string(lowstate_tick); status.values.push_back(kv);
      kv.key = "version_0"; kv.value = std::to_string(last_lowstate.version[0]); status.values.push_back(kv);
      kv.key = "version_1"; kv.value = std::to_string(last_lowstate.version[1]); status.values.push_back(kv);
      kv.key = "mode_pr"; kv.value = std::to_string(last_lowstate.mode_pr); status.values.push_back(kv);
      kv.key = "mode_machine"; kv.value = std::to_string(last_lowstate.mode_machine); status.values.push_back(kv);
      kv.key = "crc"; kv.value = std::to_string(last_lowstate.crc); status.values.push_back(kv);
      kv.key = "wireless_remote_present"; kv.value = "true"; status.values.push_back(kv);
    }
    array.status.push_back(status);
  }

  {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = node_name + ": Joint Mapping";
    status.hardware_id = "Unitree_G1_JointState";

    if (!mapping_valid) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = mapping_error.empty() ? "Invalid joint mapping" : mapping_error;
    } else if (!lowstate_received || lowstate_stale) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "Joint mapping configured, waiting for fresh LowState";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "Joint mapping valid and publishing";
    }
    array.status.push_back(status);
  }

  {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = node_name + ": IMU";
    status.hardware_id = "Unitree_G1_IMU";

    if (!lowstate_received || lowstate_stale) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "IMU state unavailable (no LowState)";
    } else if (!imu_valid) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = imu_warning.empty() ? "IMU data invalid" : imu_warning;
    } else if (!imu_warning.empty()) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = imu_warning;
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "IMU operating normally";
    }
    array.status.push_back(status);
  }

  {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = node_name + ": Battery";
    status.hardware_id = "Unitree_G1_BMS";

    if (!bms_received) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "BMS topic unavailable (no data received)";
    } else if (bms_stale) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "BMS topic stale";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "BMS operating normally";

      diagnostic_msgs::msg::KeyValue kv;
      kv.key = "soc"; kv.value = std::to_string(last_bms.soc); status.values.push_back(kv);
      kv.key = "soh"; kv.value = std::to_string(last_bms.soh); status.values.push_back(kv);
      kv.key = "cycle"; kv.value = std::to_string(last_bms.cycle); status.values.push_back(kv);
      kv.key = "current_raw"; kv.value = std::to_string(last_bms.current); status.values.push_back(kv);
    }
    array.status.push_back(status);
  }

  if (lowstate_received && !lowstate_stale) {
    for (size_t i = 0; i < mapping.joint_names.size(); ++i) {
      if (!mapping.active[i]) {continue;}
      const auto & m = last_lowstate.motor_state[mapping.motor_indices[i]];
      diagnostic_msgs::msg::DiagnosticStatus status;
      status.name = node_name + ": Motor " + std::to_string(i);
      status.hardware_id = "Unitree_G1_Motor_" + std::to_string(i);

      if (m.motorstate != 0) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message = "Motor error code: " + std::to_string(m.motorstate);
      } else {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        status.message = "Motor normal";
      }

      diagnostic_msgs::msg::KeyValue kv;
      kv.key = "motor_index"; kv.value = std::to_string(i); status.values.push_back(kv);
      kv.key = "mode"; kv.value = std::to_string(m.mode); status.values.push_back(kv);
      kv.key = "q"; kv.value = std::to_string(m.q); status.values.push_back(kv);
      kv.key = "dq"; kv.value = std::to_string(m.dq); status.values.push_back(kv);
      kv.key = "ddq"; kv.value = std::to_string(m.ddq); status.values.push_back(kv);
      kv.key = "tau_est"; kv.value = std::to_string(m.tau_est); status.values.push_back(kv);
      kv.key = "temperature_0_raw"; kv.value = std::to_string(m.temperature[0]); status.values.push_back(kv);
      kv.key = "temperature_1_raw"; kv.value = std::to_string(m.temperature[1]); status.values.push_back(kv);
      kv.key = "motor_terminal_voltage"; kv.value = std::to_string(m.vol); status.values.push_back(kv);
      kv.key = "motorstate"; kv.value = std::to_string(m.motorstate); status.values.push_back(kv);

      array.status.push_back(status);
    }
  }

  return array;
}

}

