#include "erasers_g1_common/robot_manager.hpp"
#include <stdexcept>

namespace erasers_g1_common
{

RobotManagerNode::RobotManagerNode(const rclcpp::NodeOptions & options)
: Node("robot_manager", options)
{
  declare_parameters();
  load_parameters();

  if (publish_joint_states_) {
    std::string mapping_err;
    std::vector<G1JointControlLimit> limits;
    const auto description = get_parameter("robot_description").as_string();
    const auto input = description.empty() ? get_parameter("urdf").as_string() : description;
    mapping_valid_ = loadG1JointLimits(input, limits, mapping_err);
    if (mapping_valid_) {
      for (const auto & joint : limits) {
        mapping_config_.joint_names.push_back(joint.name);
        mapping_config_.motor_indices.push_back(static_cast<int>(joint.motor_index));
        mapping_config_.active.push_back(joint.active);
      }
      mapping_valid_ = mapping_config_.isValid(mapping_err);
    }
    if (!mapping_valid_) {
      mapping_error_ = mapping_err;
      RCLCPP_FATAL(this->get_logger(), "G1 Joint Mapping auto-validation failed: %s", mapping_err.c_str());
      throw std::runtime_error("G1 Joint Mapping auto-validation failed: " + mapping_err);
    } else {
      RCLCPP_INFO(this->get_logger(), "G1 Joint Mapping validated successfully from URDF with %zu joints.", mapping_config_.joint_names.size());
    }
  }

  if (publish_joint_states_) {
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      joint_states_topic_, rclcpp::QoS(1).best_effort().durability_volatile());
  }
  if (publish_imu_) {
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::QoS(1).best_effort().durability_volatile());
  }
  if (publish_battery_) {
    battery_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>(
      battery_topic_, rclcpp::QoS(1).reliable().durability_volatile());
  }
  if (publish_diagnostics_) {
    diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic_, rclcpp::QoS(1).reliable().durability_volatile());
  }

  auto lowstate_qos = rclcpp::QoS(1).best_effort().durability_volatile();
  lowstate_sub_ = this->create_subscription<unitree_hg::msg::LowState>(
    lowstate_topic_,
    lowstate_qos,
    std::bind(&RobotManagerNode::lowstate_callback, this, std::placeholders::_1));

  auto bms_qos = rclcpp::QoS(1).best_effort().durability_volatile();
  bms_sub_ = this->create_subscription<unitree_hg::msg::BmsState>(
    bms_topic_,
    bms_qos,
    std::bind(&RobotManagerNode::bms_callback, this, std::placeholders::_1));

  double diag_timer_period = 1.0 / std::max(0.1, diagnostics_publish_rate_hz_);
  diag_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(diag_timer_period),
    std::bind(&RobotManagerNode::diag_timer_callback, this));

  if (publish_joint_states_) {
    if (!std::isfinite(joint_publish_rate_hz_) || joint_publish_rate_hz_ <= 0.0) {
      throw std::invalid_argument("joint_publish_rate_hz は正の有限値にしてください");
    }
    joint_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / joint_publish_rate_hz_)),
      std::bind(&RobotManagerNode::joint_timer_callback, this));
  }
  RCLCPP_INFO(this->get_logger(), "robot_manager を初期化しました。駆動用の通信経路はありません");
}

void RobotManagerNode::declare_parameters()
{
  declare_parameter<std::string>("robot_description", "");
  declare_parameter<std::string>("urdf", "");
  this->declare_parameter<std::string>("lowstate_topic", "/lowstate");
  this->declare_parameter<std::string>("bms_topic", "/lf/bmsstate");
  this->declare_parameter<std::string>("joint_states_topic", "/joint_states");
  this->declare_parameter<std::string>("imu_topic", "/imu");
  this->declare_parameter<std::string>("battery_topic", "/battery_states");
  this->declare_parameter<std::string>("diagnostics_topic", "/diagnostics");

  this->declare_parameter<std::string>("imu_frame_id", "imu_in_torso");
  this->declare_parameter<std::string>("joint_state_frame_id", "");

  this->declare_parameter<bool>("publish_joint_states", true);
  this->declare_parameter<bool>("publish_imu", true);
  this->declare_parameter<bool>("publish_battery", true);
  this->declare_parameter<bool>("publish_diagnostics", true);

  this->declare_parameter<double>("joint_publish_rate_hz", 30.0);
  this->declare_parameter<double>("imu_publish_rate_hz", 200.0);
  this->declare_parameter<double>("diagnostics_publish_rate_hz", 1.0);

  this->declare_parameter<double>("lowstate_stale_timeout_sec", 0.2);
  this->declare_parameter<double>("bms_stale_timeout_sec", 2.0);

  this->declare_parameter<double>("angular_velocity_scale", 1.0);
  this->declare_parameter<double>("linear_acceleration_scale", 1.0);
  this->declare_parameter<int>("expected_tick_step", 0);

  this->declare_parameter<std::string>("invalid_joint_sample_policy", "drop");
}

void RobotManagerNode::load_parameters()
{
  lowstate_topic_ = this->get_parameter("lowstate_topic").as_string();
  bms_topic_ = this->get_parameter("bms_topic").as_string();
  joint_states_topic_ = this->get_parameter("joint_states_topic").as_string();
  imu_topic_ = this->get_parameter("imu_topic").as_string();
  battery_topic_ = this->get_parameter("battery_topic").as_string();
  diagnostics_topic_ = this->get_parameter("diagnostics_topic").as_string();

  adapter_config_.imu_frame_id = this->get_parameter("imu_frame_id").as_string();
  adapter_config_.joint_state_frame_id = this->get_parameter("joint_state_frame_id").as_string();

  publish_joint_states_ = this->get_parameter("publish_joint_states").as_bool();
  publish_imu_ = this->get_parameter("publish_imu").as_bool();
  publish_battery_ = this->get_parameter("publish_battery").as_bool();
  publish_diagnostics_ = this->get_parameter("publish_diagnostics").as_bool();

  joint_publish_rate_hz_ = this->get_parameter("joint_publish_rate_hz").as_double();
  imu_publish_rate_hz_ = this->get_parameter("imu_publish_rate_hz").as_double();
  diagnostics_publish_rate_hz_ = this->get_parameter("diagnostics_publish_rate_hz").as_double();

  adapter_config_.lowstate_stale_timeout_sec = this->get_parameter("lowstate_stale_timeout_sec").as_double();
  adapter_config_.bms_stale_timeout_sec = this->get_parameter("bms_stale_timeout_sec").as_double();

  adapter_config_.angular_velocity_scale = this->get_parameter("angular_velocity_scale").as_double();
  adapter_config_.linear_acceleration_scale = this->get_parameter("linear_acceleration_scale").as_double();
  adapter_config_.expected_tick_step = static_cast<uint32_t>(this->get_parameter("expected_tick_step").as_int());

  std::string policy_str = this->get_parameter("invalid_joint_sample_policy").as_string();
  adapter_config_.invalid_sample_policy = parseInvalidSamplePolicy(policy_str);
}

void RobotManagerNode::lowstate_callback(const unitree_hg::msg::LowState::SharedPtr msg)
{
  rclcpp::Time now = this->now();

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!lowstate_received_ || msg->tick != last_lowstate_.tick) {
      last_lowstate_time_ = now;
    }
    last_lowstate_ = *msg;
    lowstate_received_ = true;
  }

  if (publish_imu_ && imu_pub_) {
    if (imu_rate_limiter_.shouldPublish(imu_publish_rate_hz_, now, msg->tick)) {
      sensor_msgs::msg::Imu imu_msg;
      std::string warning;
      bool valid = convertImu(*msg, now, adapter_config_, imu_msg, warning);
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        last_imu_valid_ = valid;
        last_imu_warning_ = warning;
      }
      if (valid) {
        imu_pub_->publish(imu_msg);
      }
    }
  }

}

void RobotManagerNode::joint_timer_callback()
{
  unitree_hg::msg::LowState state;
  const auto stamp = now();
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!lowstate_received_ || !mapping_valid_ ||
      (stamp - last_lowstate_time_).seconds() < 0.0 ||
      (stamp - last_lowstate_time_).seconds() > adapter_config_.lowstate_stale_timeout_sec)
    {
      return;
    }
    state = last_lowstate_;
  }
  sensor_msgs::msg::JointState message;
  std::string error;
  if (convertJointState(state, stamp, mapping_config_, adapter_config_, message, error)) {
    joint_state_pub_->publish(message);
  } else {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", error.c_str());
  }
}

void RobotManagerNode::bms_callback(const unitree_hg::msg::BmsState::SharedPtr msg)
{
  rclcpp::Time now = this->now();

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_bms_ = *msg;
    last_bms_time_ = now;
    bms_received_ = true;
    bms_was_stale_ = false;
  }

  if (publish_battery_ && battery_pub_) {
    sensor_msgs::msg::BatteryState bstate;
    convertBatteryState(*msg, now, bstate);
    battery_pub_->publish(bstate);
  }
}

void RobotManagerNode::diag_timer_callback()
{
  rclcpp::Time now = this->now();

  unitree_hg::msg::LowState lowstate;
  unitree_hg::msg::BmsState bms;
  rclcpp::Time lowstate_time{0, 0, RCL_ROS_TIME};
  rclcpp::Time bms_time{0, 0, RCL_ROS_TIME};
  bool ls_rec = false, bms_rec = false;
  bool imu_val = true;
  std::string imu_warn;

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    lowstate = last_lowstate_;
    bms = last_bms_;
    lowstate_time = last_lowstate_time_;
    bms_time = last_bms_time_;
    ls_rec = lowstate_received_;
    bms_rec = bms_received_;
    imu_val = last_imu_valid_;
    imu_warn = last_imu_warning_;
  }

  bool ls_stale = false;
  if (ls_rec) {
    double dt = (now - lowstate_time).seconds();
    if (dt > adapter_config_.lowstate_stale_timeout_sec) {
      ls_stale = true;
    }
  }

  bool bms_stale = false;
  if (bms_rec) {
    double dt = (now - bms_time).seconds();
    if (dt > adapter_config_.bms_stale_timeout_sec) {
      bms_stale = true;
    }
  }

  if (publish_battery_ && battery_pub_ && bms_rec && bms_stale) {
    bool publish_stale_once = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!bms_was_stale_) {
        bms_was_stale_ = true;
        publish_stale_once = true;
      }
    }
    if (publish_stale_once) {
      auto stale_bstate = createStaleBatteryState(now);
      battery_pub_->publish(stale_bstate);
    }
  }

  if (publish_diagnostics_ && diag_pub_) {
    auto diag_array = createDiagnostics(
      now,
      this->get_name(),
      ls_rec,
      ls_stale,
      lowstate.tick,
      lowstate,
      mapping_valid_,
      mapping_error_,
      imu_val,
      imu_warn,
      bms_rec,
      bms_stale,
      bms,
      mapping_config_);

    diag_pub_->publish(diag_array);
  }
}

}

