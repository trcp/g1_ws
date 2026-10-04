#include "erasers_g1_common/robot_controller.hpp"
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <limits>
#include <set>
#include <sstream>
#include <utility>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

#include "erasers_g1_common/arm_action_protocol.hpp"
#include "erasers_g1_common/g1_joint_descriptor.hpp"

namespace erasers_g1_common
{

namespace
{

double age_ms(
  const RobotControllerNode::SteadyTime & now,
  const RobotControllerNode::SteadyTime & stamp)
{
  if (stamp == RobotControllerNode::SteadyTime{}) {
    return -1.0;
  }
  return std::chrono::duration<double, std::milli>(now - stamp).count();
}

bool gid_equal(const rmw_gid_t & lhs, const rmw_gid_t & rhs)
{
  return lhs.implementation_identifier != nullptr &&
         rhs.implementation_identifier != nullptr &&
         std::strcmp(lhs.implementation_identifier, rhs.implementation_identifier) == 0 &&
         std::memcmp(lhs.data, rhs.data, RMW_GID_STORAGE_SIZE) == 0;
}

}  // namespace

RobotControllerNode::RobotControllerNode(
  const rclcpp::NodeOptions & options,
  std::shared_ptr<LocoClientInterface> loco_client,
  std::shared_ptr<ArmActionClientInterface> arm_action_client,
  std::shared_ptr<LowCmdSinkInterface> lowcmd_sink,
  SteadyNowFunction steady_now,
  UpperBodyPolicy upper_body_policy,
  std::vector<G1JointControlLimit> upper_body_limits)
: Node("robot_controller", options),
  loco_client_(std::move(loco_client)),
  arm_action_client_(std::move(arm_action_client)),
  lowcmd_sink_(std::move(lowcmd_sink)),
  steady_now_(std::move(steady_now)),
  upper_body_session_(upper_body_policy),
  upper_body_limits_(std::move(upper_body_limits))
{
  if (!steady_now_) {
    steady_now_ = []() {return std::chrono::steady_clock::now();};
  }
  if (!loco_client_) {
    loco_client_ = std::make_shared<G1LocoClient>(
      this, "/api/sport/request", "/api/sport/response");
  }

  declare_parameter<std::vector<int64_t>>("allow_fsm_ids", {500, 801});
  allow_fsm_ids_ = get_parameter("allow_fsm_ids").as_integer_array();

  // 受付後に停止した要求も、ROS 2 publish の直前に遮断する。
  loco_client_->setRequestGuard(
    [this](int64_t api, const std::string & parameter,
      const UnitreeApiClient::PublishOperation & publish, std::string & error) {
      std::lock_guard<std::mutex> gate(send_gate_);
      if (emergency_latched_ && api != kGetFsmIdApiId && api != kGetFsmModeApiId) {
        bool allowed = false;
        try {
          const auto value = nlohmann::json::parse(parameter);
          if (api == kSetFsmIdApiId) {
            const auto fsm = value.at("data").get<int32_t>();
            allowed = fsm == 0 || fsm == 1 || fsm == 3 || fsm == 706;
          } else if (api == kSetVelocityApiId) {
            const auto velocity = value.at("velocity").get<std::vector<double>>();
            allowed = velocity.size() == 3 &&
              std::all_of(velocity.begin(), velocity.end(), [](double v) {return v == 0.0;});
          }
        } catch (const nlohmann::json::exception &) {
          allowed = false;
        }
        if (!allowed) {
          error = "EMERGENCY_STOP_LATCHED";
          return false;
        }
      }
      publish();
      return true;
    });

  if (!arm_action_client_) {
    arm_action_client_ = std::make_shared<ArmActionDirectClient>(
      this, "/api/arm/request", "/api/arm/response");
  }
  arm_action_client_->setExecutePublishGuard(
    [this](const ArmActionClientInterface::PublishOperation & publish, std::string & error) {
      std::lock_guard<std::mutex> gate(send_gate_);
      std::lock_guard<std::mutex> lock(upper_body_mutex_);
      const auto & state = upper_body_arbiter_.state();
      if (state.owner != UpperBodyOwner::ARM_ACTION || state.phase == UpperBodyPhase::FAULT ||
        (upper_transition_active_ || emergency_latched_))
      {
        error = "LOCAL_CANCEL: safety or ownership changed before API 7106 publish";
        return false;
      }
      publish();
      error.clear();
      return true;
    });
  if (!lowcmd_sink_) {
    lowcmd_sink_ = std::make_shared<RosLowCmdSink>(this, "/arm_sdk");
  }

  declare_parameter<std::string>("urdf", "");
  declare_parameter<std::string>("robot_description", "");
  const auto description = get_parameter("robot_description").as_string();
  const auto model_input = description.empty() ? get_parameter("urdf").as_string() : description;
  if (upper_body_limits_.empty()) {
    upper_body_limits_valid_ = loadAndValidateG1UpperBodyLimits(
      model_input, upper_body_limits_, upper_body_configuration_error_);
  } else {
    upper_body_limits_valid_ = upper_body_limits_.size() == kG1UpperBodyJointCount;
    if (!upper_body_limits_valid_) {
      upper_body_configuration_error_ = "Expected 17 G1 upper-body slots";
    }
  }
  if (upper_body_limits_valid_) {
    std::vector<G1JointControlLimit> ordered_limits;
    ordered_limits.reserve(kG1UpperBodyJointCount);
    for (const auto & sdk_joint : g1ArmSdkJoints()) {
      const auto found = std::find_if(
        upper_body_limits_.begin(), upper_body_limits_.end(),
        [&sdk_joint](const G1JointControlLimit & limit) {
          return limit.name == sdk_joint.name && limit.motor_index == sdk_joint.motor_index;
        });
      if (found == upper_body_limits_.end()) {
        upper_body_limits_valid_ = false;
        upper_body_configuration_error_ =
          "Missing or mismatched G1 ArmSdk limit for " + std::string(sdk_joint.name);
        break;
      }
      ordered_limits.push_back(*found);
    }
    if (upper_body_limits_valid_) {
      upper_body_limits_ = std::move(ordered_limits);
    }
  }
  if (upper_body_limits_valid_) {
    upper_body_limits_valid_ = upper_body_session_.configure(
      upper_body_limits_, upper_body_configuration_error_);
  }
  if (!upper_body_limits_valid_) {
    upper_body_arbiter_.fault(
      "UPPER_BODY_CONFIGURATION_FAULT: " + upper_body_configuration_error_);
    upper_abort_requested_ = true;
    RCLCPP_ERROR(
      get_logger(), "Upper-body control disabled: %s",
      upper_body_configuration_error_.c_str());
  }

  service_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  state_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  motion_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  emergency_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  const auto transient_qos = rclcpp::QoS(1).reliable().transient_local();
  fsm_id_pub_ = create_publisher<std_msgs::msg::Int32>(
    "/robot_controller/fsm_id", transient_qos);
  fsm_mode_pub_ = create_publisher<std_msgs::msg::Int32>(
    "/robot_controller/fsm_mode", transient_qos);
  transition_active_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/robot_controller/transition_active", transient_qos);
  diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/robot_controller/diagnostics", rclcpp::QoS(1).reliable());
  publish_transition_active(false);

  pose_service_ = create_service<erasers_g1_interfaces::srv::RobotPose>(
    "/robot_pose",
    std::bind(
      &RobotControllerNode::handle_robot_pose, this,
      std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, service_cb_group_);

  arm_action_service_ = create_service<erasers_g1_interfaces::srv::ArmAction>(
    "/arm_action",
    [this](
      std::shared_ptr<rclcpp::Service<erasers_g1_interfaces::srv::ArmAction>> service,
      std::shared_ptr<rmw_request_id_t> header,
      std::shared_ptr<erasers_g1_interfaces::srv::ArmAction::Request> request)
    {
      handle_arm_action_deferred(service, header, request);
    },
    rmw_qos_profile_services_default, service_cb_group_);

  upper_enable_service_ = create_service<std_srvs::srv::SetBool>(
    "/robot_controller/upper_body/enable",
    [this](std::shared_ptr<rclcpp::Service<std_srvs::srv::SetBool>> service,
      std::shared_ptr<rmw_request_id_t> header,
      std::shared_ptr<std_srvs::srv::SetBool::Request> request) {
      handle_upper_body_enable_deferred(service, header, request);
    }, rmw_qos_profile_services_default, service_cb_group_);
  legacy_pose_service_ = create_service<erasers_g1_interfaces::srv::PosePolicy>(
    "/robot_controller/pose_policy",
    std::bind(&RobotControllerNode::handle_legacy_pose, this,
      std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, service_cb_group_);

  rclcpp::SubscriptionOptions state_options;
  state_options.callback_group = state_cb_group_;
  rclcpp::SubscriptionOptions emergency_options;
  emergency_options.callback_group = emergency_cb_group_;
  rclcpp::SubscriptionOptions motion_options;
  motion_options.callback_group = motion_cb_group_;

  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", rclcpp::QoS(1),
    std::bind(&RobotControllerNode::cmd_vel_callback, this, std::placeholders::_1),
    motion_options);
  emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/emergency_stop/active", transient_qos,
    std::bind(&RobotControllerNode::emergency_stop_callback, this, std::placeholders::_1),
    emergency_options);
  emergency_latch_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/emergency_stop/latched", transient_qos,
    std::bind(&RobotControllerNode::emergency_latch_callback, this, std::placeholders::_1),
    emergency_options);
  arm_action_state_sub_ = create_subscription<std_msgs::msg::String>(
    "/arm/action/state", rclcpp::QoS(1),
    std::bind(&RobotControllerNode::arm_action_state_callback, this, std::placeholders::_1),
    state_options);
  upper_joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "/upper_joints_control",
    rclcpp::QoS(1).best_effort().durability_volatile(),
    std::bind(
      &RobotControllerNode::upper_joint_command_callback, this,
      std::placeholders::_1, std::placeholders::_2),
    state_options);
  upper_lowstate_sub_ = create_subscription<unitree_hg::msg::LowState>(
    "/lowstate", rclcpp::QoS(1).best_effort().durability_volatile(),
    std::bind(&RobotControllerNode::upper_lowstate_callback, this, std::placeholders::_1),
    state_options);
  external_arm_request_sub_ = create_subscription<unitree_api::msg::Request>(
    "/api/arm/request", rclcpp::QoS(10),
    std::bind(
      &RobotControllerNode::external_arm_request_callback, this,
      std::placeholders::_1, std::placeholders::_2),
    state_options);

  fsm_poll_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / std::max(0.1, fsm_state_publish_rate_hz_)),
    std::bind(&RobotControllerNode::poll_fsm_state_timer, this), state_cb_group_);
  control_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / std::max(1.0, control_rate_hz_)),
    std::bind(&RobotControllerNode::control_loop_timer, this), motion_cb_group_);
  diagnostics_timer_ = create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&RobotControllerNode::publish_diagnostics_timer, this), state_cb_group_);

  emergency_running_ = true;
  emergency_thread_ = std::thread(&RobotControllerNode::emergency_worker_loop, this);
  worker_running_ = true;
  worker_thread_ = std::thread(&RobotControllerNode::api_worker_loop, this);
  control_job_running_ = true;
  control_job_thread_ = std::thread(&RobotControllerNode::control_job_worker_loop, this);
  upper_body_running_ = true;
  upper_body_thread_ = std::thread(&RobotControllerNode::upper_body_worker_loop, this);

  RCLCPP_INFO(
    get_logger(),
    "RobotControllerNode initialized; upper-body SDK remains disabled until explicit enable");
}

RobotControllerNode::~RobotControllerNode()
{
  if (rclcpp::ok()) {
    std::string ignored;
    request_upper_body_disable(ignored);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(1200);
    while (std::chrono::steady_clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(upper_body_mutex_);
        if (upper_body_arbiter_.state().phase != UpperBodyPhase::RELEASING) {
          break;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  shutdown_workers();
}

void RobotControllerNode::publish_transition_active(bool active)
{
  {
    std::lock_guard<std::mutex> lock(motion_state_mutex_);
    transition_active_ = active;
  }
  {
    std::lock_guard<std::mutex> lock(upper_body_mutex_);
    upper_transition_active_ = active;
  }
  std_msgs::msg::Bool msg;
  msg.data = active;
  transition_active_pub_->publish(msg);
}

void RobotControllerNode::poll_fsm_state_timer()
{
  if (!transition_mutex_.try_lock()) {
    return;
  }
  const auto id_result = loco_client_->getFsmId(api_timeout_sec_);
  if (id_result.success) {
    std_msgs::msg::Int32 msg;
    msg.data = id_result.fsm_id;
    fsm_id_pub_->publish(msg);
    {
      std::lock_guard<std::mutex> lock(motion_state_mutex_);
      current_fsm_id_ = id_result.fsm_id;
      fsm_state_time_ = steady_now_();
    }
    {
      std::lock_guard<std::mutex> lock(upper_body_mutex_);
      upper_fsm_id_ = id_result.fsm_id;
      upper_fsm_time_ = steady_now_();
    }
  }
  const auto mode_result = loco_client_->getFsmMode(api_timeout_sec_);
  if (mode_result.success) {
    std_msgs::msg::Int32 msg;
    msg.data = mode_result.fsm_mode;
    fsm_mode_pub_->publish(msg);
    std::lock_guard<std::mutex> lock(upper_body_mutex_);
    upper_fsm_mode_ = mode_result.fsm_mode;
    upper_fsm_mode_time_ = steady_now_();
  }
  transition_mutex_.unlock();
}

void RobotControllerNode::handle_robot_pose(
  const std::shared_ptr<erasers_g1_interfaces::srv::RobotPose::Request> request,
  std::shared_ptr<erasers_g1_interfaces::srv::RobotPose::Response> response)
{
  int32_t target_fsm = -1;
  switch (request->mode) {
    case erasers_g1_interfaces::srv::RobotPose::Request::ZERO_TORQUE:
      target_fsm = kFsmZeroTorque;
      break;
    case erasers_g1_interfaces::srv::RobotPose::Request::DAMP:
      target_fsm = kFsmDamp;
      break;
    case erasers_g1_interfaces::srv::RobotPose::Request::LOCKED_STAND:
      target_fsm = kFsmStandUp;
      break;
    case erasers_g1_interfaces::srv::RobotPose::Request::START:
      target_fsm = kFsmStart;
      break;
    case erasers_g1_interfaces::srv::RobotPose::Request::LOCKED_SEAT:
      target_fsm = 3;
      break;
    case erasers_g1_interfaces::srv::RobotPose::Request::RUNNING:
      target_fsm = 801;
      break;
    case erasers_g1_interfaces::srv::RobotPose::Request::SQUAT:
      target_fsm = 706;
      break;
    case erasers_g1_interfaces::srv::RobotPose::Request::LIE:
      response->success = false;
      response->fsm_id = -1;
      response->api_status_code = -1;
      response->message = "Requested pose is not supported by the current official G1 Loco API";
      return;
    default:
      response->success = false;
      response->fsm_id = -1;
      response->api_status_code = -1;
      response->message = "Unknown /robot_pose request mode: " + std::to_string(request->mode);
      return;
  }

  transition_to(target_fsm, response);
}

void RobotControllerNode::transition_to(
  int32_t target_fsm,
  std::shared_ptr<erasers_g1_interfaces::srv::RobotPose::Response> response)
{
  const bool safety_pose = target_fsm == kFsmZeroTorque || target_fsm == kFsmDamp ||
    target_fsm == 706 || target_fsm == 3;
  if (emergency_latched_ && !safety_pose) {
    response->success = false;
    response->message = "EMERGENCY_STOP_LATCHED";
    return;
  }
  bool joint_owned = false;
  bool arm_action_owned = false;
  {
    std::lock_guard<std::mutex> lock(upper_body_mutex_);
    joint_owned = upper_body_arbiter_.jointStateOwnsControl();
    arm_action_owned = upper_body_arbiter_.state().owner == UpperBodyOwner::ARM_ACTION;
  }
  if ((joint_owned || arm_action_owned) &&
    !safety_pose)
  {
    response->success = false;
    response->fsm_id = -1;
    response->api_status_code = 0;
    response->message = joint_owned ?
      "LOCAL_REJECT: UPPER_BODY_JOINT_STATE_OWNS_CONTROL" :
      "LOCAL_REJECT: UPPER_BODY_ARM_ACTION_OWNS_CONTROL";
    return;
  }
  if (joint_owned || arm_action_owned) {
    upper_body_fault("SAFETY_POSE_REQUEST");
  }

  std::unique_lock<std::mutex> transition_lock(transition_mutex_, std::try_to_lock);
  if (!transition_lock.owns_lock()) {
    response->success = false;
    response->message = "POSE_TRANSITION_BUSY";
    return;
  }
  publish_transition_active(true);
  {
    std::lock_guard<std::mutex> motion_lock(motion_state_mutex_);
    enqueue_priority_stop_locked();
  }
  const auto current_result = loco_client_->getFsmId(api_timeout_sec_);
  int32_t current_fsm = current_result.success ? current_result.fsm_id : -1;
  if (current_result.success && current_fsm == target_fsm) {
    response->success = true;
    response->fsm_id = target_fsm;
    response->api_status_code = 0;
    response->message = "Robot is already in target FSM " + std::to_string(target_fsm);
    publish_transition_active(false);
    return;
  }
  if (stop_before_transition_) {
    loco_client_->stopMove(api_timeout_sec_);
  }
  const auto set_result = loco_client_->setFsmId(target_fsm, api_timeout_sec_);
  if (!set_result.success) {
    response->success = false;
    response->fsm_id = current_fsm;
    response->api_status_code = set_result.status_code;
    response->message = "SetFsmId failed: " + set_result.message;
    publish_transition_active(false);
    return;
  }

  const auto deadline = steady_now_() + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(transition_timeout_sec_));
  bool completed = false;
  int32_t final_fsm = -1;
  while (rclcpp::ok() && steady_now_() < deadline && (!emergency_latched_ || safety_pose)) {
    const auto result = loco_client_->getFsmId(api_timeout_sec_);
    if (result.success) {
      final_fsm = result.fsm_id;
      if (final_fsm == target_fsm) {
        completed = true;
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::duration<double>(transition_poll_period_sec_));
  }
  publish_transition_active(false);
  if (completed) {
    const auto observation_time = steady_now_();
    {
      std::lock_guard<std::mutex> lock(motion_state_mutex_);
      current_fsm_id_ = final_fsm;
      fsm_state_time_ = observation_time;
    }
    {
      std::lock_guard<std::mutex> lock(upper_body_mutex_);
      upper_fsm_id_ = final_fsm;
      upper_fsm_time_ = observation_time;
    }
  }
  response->success = completed;
  response->fsm_id = final_fsm;
  response->api_status_code = completed ? 0 : -1;
  response->message = completed ?
    "Successfully transitioned to FSM " + std::to_string(target_fsm) :
    "FSM transition timed out; current FSM " + std::to_string(final_fsm);
}

void RobotControllerNode::arm_action_state_callback(const std_msgs::msg::String::SharedPtr msg)
{
  ArmActionState parsed;
  std::string error;
  if (!parse_arm_action_state(msg->data, parsed, &error)) {
    bool must_fault = false;
    {
      std::lock_guard<std::mutex> arm_lock(arm_state_mutex_);
      arm_state_malformed_ = true;
    }
    {
      std::lock_guard<std::mutex> upper_lock(upper_body_mutex_);
      must_fault = upper_body_arbiter_.jointStateOwnsControl();
    }
    if (must_fault) {
      upper_body_fault("MALFORMED_ARM_ACTION_STATE");
    }
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Failed to parse /arm/action/state: %s", error.c_str());
    return;
  }

  bool external_conflict = false;
  bool no_arm_request_pending = false;
  {
    std::lock_guard<std::mutex> arm_lock(arm_state_mutex_);
    arm_state_valid_ = true;
    arm_state_malformed_ = false;
    arm_action_id_ = parsed.id;
    arm_action_name_ = parsed.name;
    arm_holding_ = parsed.holding;
    arm_state_time_ = steady_now_();
    ++arm_state_seq_;
    if (arm_request_pending_ && arm_state_seq_ > pending_arm_baseline_seq_) {
      if (parsed.id == pending_arm_request_id_) {
        pending_seen_requested_id_ = true;
        pending_seen_holding_ = parsed.holding;
        pending_observed_action_name_ = parsed.name;
        pending_observed_holding_ = parsed.holding;
      }
      if (parsed.id == kArmActionNormalId && !parsed.holding &&
        (pending_arm_request_id_ == kArmActionReleaseId || pending_seen_requested_id_))
      {
        pending_seen_terminal_zero_ = true;
      }
    }
    no_arm_request_pending = !arm_request_pending_;
  }
  arm_state_cv_.notify_all();

  {
    std::lock_guard<std::mutex> upper_lock(upper_body_mutex_);
    if (upper_body_arbiter_.jointStateOwnsControl() &&
      (parsed.id != kArmActionNormalId || parsed.holding))
    {
      external_conflict = true;
    } else if (parsed.id == kArmActionNormalId && !parsed.holding &&
      no_arm_request_pending)
    {
      if (release_observation_pending_) {
        release_observation_pending_ = false;
      }
      upper_body_arbiter_.observeRemoteNormal();
    }
  }
  if (external_conflict) {
    upper_body_fault("EXTERNAL_ARM_ACTION_STATE");
  }
}

void RobotControllerNode::handle_legacy_pose(
  const std::shared_ptr<erasers_g1_interfaces::srv::PosePolicy::Request> request,
  std::shared_ptr<erasers_g1_interfaces::srv::PosePolicy::Response> response)
{
  using Pose = erasers_g1_interfaces::srv::RobotPose;
  const std::map<std::string, int8_t> poses{
    {"zero_torque", Pose::Request::ZERO_TORQUE}, {"damp", Pose::Request::DAMP},
    {"start", Pose::Request::START}, {"running", Pose::Request::RUNNING},
    {"squat", Pose::Request::SQUAT}, {"sit", Pose::Request::LOCKED_SEAT},
    {"stand_up", Pose::Request::LOCKED_STAND}};
  const auto pose = poses.find(request->pose);
  if (pose != poses.end()) {
    auto result = std::make_shared<erasers_g1_interfaces::srv::RobotPose::Response>();
    auto translated = std::make_shared<Pose::Request>();
    translated->mode = pose->second;
    handle_robot_pose(translated, result);
    response->success = result->success;
    return;
  }
  if (request->pose == "stop_move") {
    {
      std::lock_guard<std::mutex> lock(motion_state_mutex_);
      last_raw_cmd_.is_stop = true;
      last_cmd_steady_ = SteadyTime{};
      enqueue_priority_stop_locked();
    }
    response->success = loco_client_->stopMove(api_timeout_sec_).success;
    return;
  }
  std::unique_lock<std::mutex> transition_lock(transition_mutex_, std::try_to_lock);
  if (!transition_lock.owns_lock() || emergency_latched_) {response->success = false; return;}
  {
    std::lock_guard<std::mutex> lock(upper_body_mutex_);
    if (upper_body_arbiter_.state().owner != UpperBodyOwner::NONE ||
      upper_body_arbiter_.state().phase == UpperBodyPhase::FAULT)
    {
      response->success = false;
      return;
    }
    upper_transition_active_ = true;
  }
  int64_t api = 0;
  nlohmann::json parameter;
  if (request->pose == "high_stand" || request->pose == "low_stand") {
    api = 7104;
    parameter["data"] = request->pose == "high_stand" ? 4294967295.0 : 0.0;
  } else if (request->pose == "balance_stand") {
    api = 7102;
    parameter["data"] = 0;
  } else if (request->pose == "wave_hand" || request->pose == "wave_hand_with_turn") {
    api = 7106;
    parameter["data"] = request->pose == "wave_hand" ? 0 : 1;
  } else if (request->pose == "shake_hand") {
    api = 7106;
    parameter["data"] = legacy_shake_holding_ ? 3 : 2;
  }
  const auto result = api == 0 ? LocoResult{} :
    loco_client_->callLegacy(api, parameter.dump(), api_timeout_sec_);
  response->success = result.success;
  {
    std::lock_guard<std::mutex> lock(upper_body_mutex_);
    upper_transition_active_ = false;
    if (api == 7106 && (result.success || result.timed_out)) {
      upper_body_arbiter_.markUnknown("LEGACY_ARM_TASK_PENDING");
    }
  }
  if (request->pose == "shake_hand" && result.success) {
    legacy_shake_holding_ = !legacy_shake_holding_;
  }
}

void RobotControllerNode::handle_arm_action_deferred(
  const std::shared_ptr<rclcpp::Service<erasers_g1_interfaces::srv::ArmAction>> service,
  const std::shared_ptr<rmw_request_id_t> header,
  const std::shared_ptr<erasers_g1_interfaces::srv::ArmAction::Request> request)
{
  auto reject = [&](const std::string & message) {
      auto response = std::make_shared<erasers_g1_interfaces::srv::ArmAction::Response>();
      response->success = false;
      response->action_id = request->mode;
      response->api_status_code = 0;
      response->message = message;
      service->send_response(*header, *response);
  };

  if (emergency_latched_) {
    reject("EMERGENCY_STOP_LATCHED");
    return;
  }
  bool fresh_normal_noop = false;
  bool arm_request_pending = false;
  {
    std::lock_guard<std::mutex> arm_lock(arm_state_mutex_);
    arm_request_pending = arm_request_pending_;
    fresh_normal_noop = request->mode == kArmActionReleaseId && arm_state_valid_ &&
      arm_action_id_ == kArmActionNormalId && !arm_holding_ &&
      steady_now_() - arm_state_time_ <= upper_body_session_.policy().arm_state_timeout &&
      !arm_request_pending_;
  }
  if (arm_request_pending) {
    reject("BUSY: ARM_ACTION request already pending");
    return;
  }
  {
    std::lock_guard<std::mutex> upper_lock(upper_body_mutex_);
    if (upper_body_arbiter_.jointStateOwnsControl()) {
      reject("LOCAL_REJECT: UPPER_BODY_JOINT_STATE_OWNS_CONTROL");
      return;
    }
    if (fresh_normal_noop && upper_body_arbiter_.state().owner == UpperBodyOwner::NONE) {
      auto response = std::make_shared<erasers_g1_interfaces::srv::ArmAction::Response>();
      response->success = true;
      response->action_id = request->mode;
      response->api_status_code = 0;
      response->message = "Arm is already in a freshly observed normal state.";
      service->send_response(*header, *response);
      return;
    }
    if (emergency_latched_) {
      reject("EMERGENCY_STOP_LATCHED");
      return;
    }
    std::string reason;
    if (!upper_body_arbiter_.reserveArmAction(
        request->mode == kArmActionReleaseId, reason))
    {
      reject(reason);
      return;
    }
  }
  {
    std::lock_guard<std::mutex> arm_lock(arm_state_mutex_);
    if (arm_request_pending_) {
      reject("BUSY: ARM_ACTION request already pending");
      return;
    }
    arm_request_pending_ = true;
    pending_arm_request_id_ = request->mode;
    pending_arm_baseline_seq_ = arm_state_seq_;
    pending_seen_requested_id_ = false;
    pending_seen_holding_ = false;
    pending_seen_terminal_zero_ = false;
    pending_observed_action_name_.clear();
    pending_observed_holding_ = false;
  }

  if (!enqueue_control_job([this, service, header, request]() {
      auto response = std::make_shared<erasers_g1_interfaces::srv::ArmAction::Response>();
      process_arm_action(request->mode, *response);
      service->send_response(*header, *response);
    }))
  {
    {
      std::lock_guard<std::mutex> arm_lock(arm_state_mutex_);
      arm_request_pending_ = false;
    }
    {
      std::lock_guard<std::mutex> upper_lock(upper_body_mutex_);
      upper_body_arbiter_.markUnknown("ARM_ACTION_JOB_QUEUE_BUSY");
    }
    reject("BUSY: control job worker unavailable");
  }
}

void RobotControllerNode::process_arm_action(
  int32_t request_mode,
  erasers_g1_interfaces::srv::ArmAction::Response & response)
{
  response.action_id = request_mode;
  auto stopped = [&]() {
      if (!emergency_latched_) {return false;}
      response.success = false;
      response.api_status_code = 0;
      response.message = "EMERGENCY_STOP_LATCHED";
      std::lock_guard<std::mutex> lock(arm_state_mutex_);
      arm_request_pending_ = false;
      return true;
    };
  if (stopped()) {return;}
  {
    std::lock_guard<std::mutex> lock(upper_body_mutex_);
    if (upper_transition_active_ || emergency_latched_) {
      response.success = false;
      response.api_status_code = 0;
      response.message = "LOCAL_REJECT: robot safety state does not allow ArmAction";
      upper_body_arbiter_.completeArmAction(false);
    } else {
      response.message.clear();
    }
  }
  if (!response.message.empty()) {
    std::lock_guard<std::mutex> arm_lock(arm_state_mutex_);
    arm_request_pending_ = false;
    return;
  }

  const auto fsm_result = loco_client_->getFsmId(api_timeout_sec_);
  if (!fsm_result.success ||
    std::find(allow_fsm_ids_.begin(), allow_fsm_ids_.end(), fsm_result.fsm_id) ==
    allow_fsm_ids_.end())
  {
    response.success = false;
    response.api_status_code = fsm_result.status_code;
    response.message = fsm_result.success ?
      "Arm Action rejected locally: FSM is not allowed" :
      "Arm Action rejected locally: fresh FSM read failed";
    {
      std::lock_guard<std::mutex> lock(upper_body_mutex_);
      upper_body_arbiter_.completeArmAction(false);
    }
    {
      std::lock_guard<std::mutex> arm_lock(arm_state_mutex_);
      arm_request_pending_ = false;
    }
    return;
  }

  if (fsm_result.fsm_id == 801) {
    const auto mode = loco_client_->getFsmMode(api_timeout_sec_);
    if (!mode.success || (mode.fsm_mode != 0 && mode.fsm_mode != 3)) {
      response.success = false;
      response.message = "G1_FSM_MODE_NOT_ALLOWED";
      {std::lock_guard<std::mutex> lock(upper_body_mutex_);
        upper_body_arbiter_.completeArmAction(false);}
      {std::lock_guard<std::mutex> lock(arm_state_mutex_); arm_request_pending_ = false;}
      return;
    }
  }
  const auto arm_result = arm_action_client_->executeAction(
    request_mode,
    std::chrono::duration<double>(
      upper_body_session_.policy().arm_action_api_timeout).count());
  if (stopped()) {return;}
  if (!arm_result.success) {
    response.success = false;
    response.api_status_code = arm_result.status_code;
    response.message = arm_result.message;
    {
      std::lock_guard<std::mutex> lock(upper_body_mutex_);
      if (arm_result.cancelled_before_publish) {
        upper_body_arbiter_.completeArmAction(false);
      } else if (arm_result.timed_out || (!arm_result.received && !arm_result.timed_out)) {
        upper_body_arbiter_.markUnknown("ARM_ACTION_RPC_RESULT_UNKNOWN");
      } else {
        upper_body_arbiter_.completeArmAction(false);
      }
    }
    {
      std::lock_guard<std::mutex> arm_lock(arm_state_mutex_);
      arm_request_pending_ = false;
    }
    return;
  }

  bool reached_condition = false;
  bool holding_result = false;
  std::string observed_action_name;
  bool observed_holding = false;
  {
    std::unique_lock<std::mutex> arm_lock(arm_state_mutex_);
    const auto deadline = std::chrono::steady_clock::now() +
      upper_body_session_.policy().arm_action_state_timeout;
    reached_condition = arm_state_cv_.wait_until(
      arm_lock, deadline,
      [this]() {
        return !control_job_running_ || upper_abort_requested_ || emergency_latched_ ||
               pending_seen_holding_ || pending_seen_terminal_zero_;
      });
    reached_condition = reached_condition && control_job_running_ && !upper_abort_requested_ &&
      !emergency_latched_ &&
      (pending_seen_holding_ || pending_seen_terminal_zero_);
    holding_result = pending_seen_holding_ && !pending_seen_terminal_zero_;
    observed_action_name = pending_observed_action_name_;
    observed_holding = pending_observed_holding_;
    arm_request_pending_ = false;
  }

  if (stopped()) {return;}
  if (!reached_condition) {
    response.success = false;
    response.api_status_code = arm_result.status_code;
    response.message = "ArmAction state condition timed out; result is unknown";
    std::lock_guard<std::mutex> lock(upper_body_mutex_);
    upper_body_arbiter_.markUnknown("ARM_ACTION_STATE_RESULT_UNKNOWN");
    return;
  }

  response.success = true;
  response.api_status_code = arm_result.status_code;
  response.message = "ArmAction action_name='" + observed_action_name +
    "', holding=" + (observed_holding ? "true" : "false");
  {
    std::lock_guard<std::mutex> lock(upper_body_mutex_);
    if (emergency_latched_) {
      response.success = false;
      response.message = "EMERGENCY_STOP_LATCHED";
    } else {
      upper_body_arbiter_.completeArmAction(holding_result);
    }
  }
}

void RobotControllerNode::handle_upper_body_enable_deferred(
  const std::shared_ptr<rclcpp::Service<std_srvs::srv::SetBool>> service,
  const std::shared_ptr<rmw_request_id_t> header,
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request)
{
  if (emergency_latched_) {
    auto response = std::make_shared<std_srvs::srv::SetBool::Response>();
    response->success = false;
    response->message = "EMERGENCY_STOP_LATCHED";
    service->send_response(*header, *response);
    return;
  }
  if (!request->data) {
    auto response = std::make_shared<std_srvs::srv::SetBool::Response>();
    request_upper_body_disable(response->message);
    response->success = true;
    service->send_response(*header, *response);
    return;
  }

  if (emergency_latched_) {
    auto response = std::make_shared<std_srvs::srv::SetBool::Response>();
    response->success = false;
    response->message = "EMERGENCY_STOP_LATCHED";
    service->send_response(*header, *response);
    return;
  }
  bool recovery_attempt = false;
  std::string latched_fault_reason;
  {
    std::lock_guard<std::mutex> lock(upper_body_mutex_);
    const auto & state = upper_body_arbiter_.state();
    if (state.owner == UpperBodyOwner::JOINT_STATE &&
      (state.phase == UpperBodyPhase::ARMED ||
      state.phase == UpperBodyPhase::ACQUIRING || state.phase == UpperBodyPhase::ACTIVE))
    {
      auto response = std::make_shared<std_srvs::srv::SetBool::Response>();
      response->success = true;
      response->message = state.phase == UpperBodyPhase::ACTIVE ?
        "ALREADY_ACTIVE" : "ALREADY_ARMED";
      service->send_response(*header, *response);
      return;
    }
    if (state.phase == UpperBodyPhase::RELEASING) {
      auto response = std::make_shared<std_srvs::srv::SetBool::Response>();
      response->success = false;
      response->message = "BUSY: RELEASE_IN_PROGRESS";
      service->send_response(*header, *response);
      return;
    }
    if (emergency_latched_) {
      auto response = std::make_shared<std_srvs::srv::SetBool::Response>();
      response->success = false;
      response->message = "EMERGENCY_STOP_LATCHED";
      service->send_response(*header, *response);
      return;
    }
    recovery_attempt = state.phase == UpperBodyPhase::FAULT;
    latched_fault_reason = state.fault_reason;
    std::string reason;
    if (!upper_body_arbiter_.prepareFaultRecovery(reason) ||
      !upper_body_arbiter_.reserveJointState(reason))
    {
      auto response = std::make_shared<std_srvs::srv::SetBool::Response>();
      response->success = false;
      response->message = reason;
      service->send_response(*header, *response);
      return;
    }
    upper_abort_requested_ = false;
    stop_fence_complete_ = true;
    command_source_gid_.clear();
    last_command_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    session_ros_start_ = now();
    upper_body_session_.arm(upper_body_arbiter_.state().generation, steady_now_());
  }

  if (!enqueue_control_job([this, service, header, recovery_attempt, latched_fault_reason]() {
      auto response = std::make_shared<std_srvs::srv::SetBool::Response>();
      process_upper_body_enable(true, recovery_attempt, latched_fault_reason, *response);
      service->send_response(*header, *response);
    }))
  {
    {
      std::lock_guard<std::mutex> lock(upper_body_mutex_);
      if (recovery_attempt) {
        upper_body_arbiter_.fault(latched_fault_reason);
        upper_body_session_.abort();
        upper_abort_requested_ = true;
      } else {
        upper_body_arbiter_.cancelArmed();
      }
    }
    auto response = std::make_shared<std_srvs::srv::SetBool::Response>();
    response->success = false;
    response->message = "BUSY: control job worker unavailable";
    service->send_response(*header, *response);
  }
}

void RobotControllerNode::process_upper_body_enable(
  bool enable,
  bool recovery_attempt,
  const std::string & latched_fault_reason,
  std_srvs::srv::SetBool::Response & response)
{
  if (!enable) {
    request_upper_body_disable(response.message);
    response.success = true;
    return;
  }
  std::string reason;
  if (!upper_body_preconditions(reason, true)) {
    std::lock_guard<std::mutex> lock(upper_body_mutex_);
    if (recovery_attempt) {
      upper_body_arbiter_.fault(latched_fault_reason);
      upper_body_session_.abort();
      upper_abort_requested_ = true;
    } else {
      upper_body_arbiter_.cancelArmed();
    }
    response.success = false;
    response.message = reason;
    return;
  }
  {
    std::lock_guard<std::mutex> lock(upper_body_mutex_);
    if (emergency_latched_ ||
      upper_body_arbiter_.state().owner != UpperBodyOwner::JOINT_STATE ||
      upper_body_arbiter_.state().phase != UpperBodyPhase::ARMED)
    {
      response.success = false;
      response.message = "ARMED_CANCELLED";
      return;
    }
  }
  response.success = true;
  response.message = "ARMED_WAITING_COMMAND";
}

void RobotControllerNode::request_upper_body_disable(std::string & message)
{
  std::lock_guard<std::mutex> lock(upper_body_mutex_);
  const auto phase = upper_body_arbiter_.state().phase;
  if (phase == UpperBodyPhase::FAULT) {
    upper_body_arbiter_.acknowledgeFaultDisable();
    message = "FAULT_LATCHED: local disable acknowledged";
    return;
  }
  if (phase == UpperBodyPhase::ARMED) {
    upper_body_arbiter_.cancelArmed();
    command_source_gid_.clear();
    message = "DISABLE_ACCEPTED";
    return;
  }
  if (phase == UpperBodyPhase::ACQUIRING || phase == UpperBodyPhase::ACTIVE) {
    if (upper_body_arbiter_.beginRelease()) {
      upper_body_session_.requestRelease(
        upper_body_arbiter_.state().generation, steady_now_());
      message = "DISABLE_ACCEPTED";
      return;
    }
  }
  if (phase == UpperBodyPhase::RELEASING) {
    message = "DISABLE_ACCEPTED: already releasing";
    return;
  }
  message = "ALREADY_DISABLED";
}

bool RobotControllerNode::validate_upper_joint_command(
  const sensor_msgs::msg::JointState & msg,
  const std::string & source_gid,
  std::vector<std::size_t> & indices,
  std::vector<double> & positions,
  std::string & reason)
{
  const auto phase = upper_body_arbiter_.state().phase;
  if (emergency_latched_) {reason = "EMERGENCY_STOP_LATCHED"; return false;}
  if (phase != UpperBodyPhase::ARMED && phase != UpperBodyPhase::ACQUIRING &&
    phase != UpperBodyPhase::ACTIVE)
  {
    reason = "UPPER_BODY_NOT_ARMED";
    return false;
  }
  if (msg.name.empty() || msg.position.size() != msg.name.size() ||
    !msg.velocity.empty() || !msg.effort.empty())
  {
    reason = "INVALID_JOINT_STATE_ARRAYS";
    return false;
  }
  if (msg.header.stamp.sec == 0 && msg.header.stamp.nanosec == 0) {
    reason = "ZERO_COMMAND_STAMP";
    return false;
  }
  if (msg.header.stamp.nanosec >= 1000000000U) {
    reason = "INVALID_COMMAND_STAMP";
    return false;
  }
  const rclcpp::Time stamp(msg.header.stamp, get_clock()->get_clock_type());
  const rclcpp::Time ros_now = now();
  const double stamp_age = (ros_now - stamp).seconds();
  if (stamp_age > upper_body_session_.policy().max_past_stamp_sec ||
    stamp_age < -upper_body_session_.policy().max_future_stamp_sec)
  {
    reason = "STALE_OR_FUTURE_COMMAND_STAMP";
    return false;
  }
  if (stamp < session_ros_start_ ||
    (last_command_stamp_.nanoseconds() != 0 && stamp <= last_command_stamp_))
  {
    reason = "OLD_OR_NON_MONOTONIC_COMMAND_STAMP";
    return false;
  }
  if (last_ros_now_.nanoseconds() != 0 && ros_now < last_ros_now_) {
    reason = "ROS_TIME_ROLLBACK";
    return false;
  }
  if (source_gid.empty()) {
    reason = "COMMAND_SOURCE_GID_UNAVAILABLE";
    return false;
  }
  if (!command_source_gid_.empty() && command_source_gid_ != source_gid) {
    reason = "MULTIPLE_JOINT_STATE_SOURCES";
    return false;
  }

  std::set<std::string> seen;
  indices.clear();
  positions.clear();
  for (std::size_t input_index = 0; input_index < msg.name.size(); ++input_index) {
    if (!seen.insert(msg.name[input_index]).second) {
      reason = "DUPLICATE_JOINT_NAME";
      return false;
    }
    const auto spec = std::find_if(
      g1ArmSdkJoints().begin(), g1ArmSdkJoints().end(),
      [&](const auto & joint) {return msg.name[input_index] == joint.name;});
    if (spec == g1ArmSdkJoints().end()) {
      reason = "UNKNOWN_OR_NON_UPPER_BODY_JOINT";
      return false;
    }
    const std::size_t index = static_cast<std::size_t>(
      std::distance(g1ArmSdkJoints().begin(), spec));
    if (!upper_body_limits_[index].active) {reason = "INACTIVE_G1_JOINT"; return false;}
    const double position = msg.position[input_index];
    const float wire_position = static_cast<float>(position);
    if (!std::isfinite(position) || !std::isfinite(wire_position) ||
      position < upper_body_limits_[index].lower || position > upper_body_limits_[index].upper)
    {
      reason = "NONFINITE_OR_OUT_OF_RANGE_POSITION";
      return false;
    }
    indices.push_back(index);
    positions.push_back(position);
  }
  last_ros_now_ = ros_now;
  return true;
}

void RobotControllerNode::upper_joint_command_callback(
  const sensor_msgs::msg::JointState::SharedPtr msg,
  const rclcpp::MessageInfo & info)
{
  const std::string source_gid = gid_to_string(info.get_rmw_message_info().publisher_gid);
  std::string reason;
  bool fault = false;
  {
    std::lock_guard<std::mutex> lock(upper_body_mutex_);
    std::vector<std::size_t> indices;
    std::vector<double> positions;
    if (!validate_upper_joint_command(*msg, source_gid, indices, positions, reason)) {
      fault = reason == "MULTIPLE_JOINT_STATE_SOURCES" || reason == "ROS_TIME_ROLLBACK";
    } else {
      if (upper_body_arbiter_.state().phase == UpperBodyPhase::ARMED) {
        const auto now_steady = steady_now_();
        bool arm_normal = false;
        {
          std::lock_guard<std::mutex> arm_lock(arm_state_mutex_);
          arm_normal = arm_state_valid_ && !arm_state_malformed_ &&
            arm_action_id_ == kArmActionNormalId && !arm_holding_ &&
            !arm_request_pending_ &&
            now_steady - arm_state_time_ <= upper_body_session_.policy().arm_state_timeout;
        }
        const bool inputs_fresh = lowstate_valid_ &&
          now_steady - lowstate_time_ <= upper_body_session_.policy().lowstate_timeout &&
          std::find(
            upper_body_session_.policy().allowed_fsm_ids.begin(),
            upper_body_session_.policy().allowed_fsm_ids.end(),
            upper_fsm_id_) != upper_body_session_.policy().allowed_fsm_ids.end() &&
          now_steady - upper_fsm_time_ <= upper_body_session_.policy().fsm_timeout &&
          !upper_transition_active_ && arm_normal &&
          upper_joint_sub_->get_publisher_count() == 1U;
        bool feedback_valid_and_stationary = true;
        for (std::size_t i = 0; i < measured_q_.size(); ++i) {
          if (!upper_body_limits_[i].active) {continue;}
          feedback_valid_and_stationary = feedback_valid_and_stationary &&
            std::isfinite(measured_q_[i]) && std::isfinite(measured_dq_[i]) &&
            measured_q_[i] >= upper_body_limits_[i].lower &&
            measured_q_[i] <= upper_body_limits_[i].upper &&
            std::abs(measured_dq_[i]) <=
            upper_body_session_.policy().max_acquire_measured_velocity;
        }
        if (!inputs_fresh || !feedback_valid_and_stationary) {
          reason = "FRESH_SAFETY_STATE_REQUIRED";
        } else {
          std::string session_error;
          if (!upper_body_session_.beginAcquisition(
              measured_q_, upper_body_arbiter_.state().generation,
              now_steady, session_error) ||
            !upper_body_session_.updateTarget(indices, positions, now_steady, session_error) ||
            !upper_body_arbiter_.beginAcquiring())
          {
            reason = session_error.empty() ? "ACQUISITION_STATE_ERROR" : session_error;
            fault = true;
          }
        }
      } else {
        std::string session_error;
        if (!upper_body_session_.updateTarget(
            indices, positions, steady_now_(), session_error))
        {
          reason = session_error;
          fault = true;
        }
      }
      if (reason.empty()) {
        if (command_source_gid_.empty()) {
          command_source_gid_ = source_gid;
        }
        last_command_stamp_ = rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type());
        ++command_rx_seq_;
      }
    }
  }
  if (fault) {
    upper_body_fault(reason);
  } else if (!reason.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Rejected /upper_joints_control: %s", reason.c_str());
  }
}

void RobotControllerNode::upper_lowstate_callback(
  const unitree_hg::msg::LowState::SharedPtr msg)
{
  bool fault = false;
  std::string reason;
  bool tick_rebased = false;
  uint32_t previous_tick = 0;
  {
    std::lock_guard<std::mutex> lock(upper_body_mutex_);
    if (lowstate_valid_) {
      const uint32_t delta = msg->tick - lowstate_tick_;
      if (delta == 0U) {
        ++duplicate_lowstate_ticks_;
        return;
      }
      if (delta > 0x80000000U) {
        // LowState tick is a transport sequence indicator, not an actuator-safety
        // assertion.  DDS may deliver a reordered sample and the robot-side
        // publisher may restart its sequence.  A finite, newly received state is
        // still valid feedback; rebase the sequence and retain the regular
        // lowstate_timeout guard for actual feedback loss.
        previous_tick = lowstate_tick_;
        ++lowstate_tick_rebases_;
        tick_rebased = true;
      }
    }
    std::array<double, kG1UpperBodyJointCount> q{};
    std::array<double, kG1UpperBodyJointCount> dq{};
    if (!fault) {
      for (std::size_t i = 0; i < g1ArmSdkJoints().size(); ++i) {
        if (!upper_body_limits_valid_ || !upper_body_limits_[i].active) {continue;}
        const auto motor_index = g1ArmSdkJoints()[i].motor_index;
        q[i] = msg->motor_state.at(motor_index).q;
        dq[i] = msg->motor_state.at(motor_index).dq;
        if (!std::isfinite(q[i]) || !std::isfinite(dq[i])) {
          fault = upper_body_arbiter_.jointStateOwnsControl();
          reason = "NONFINITE_LOWSTATE";
          break;
        }
      }
    }
    if (!fault) {
      measured_q_ = q;
      measured_dq_ = dq;
      lowstate_tick_ = msg->tick;
      lowstate_time_ = steady_now_();
      lowstate_valid_ = true;
    }
  }
  if (fault) {
    upper_body_fault(reason);
  } else if (tick_rebased) {
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "LowState tick rebased from %u to %u; accepting fresh finite feedback",
      previous_tick, msg->tick);
  }
}

void RobotControllerNode::external_arm_request_callback(
  const unitree_api::msg::Request::SharedPtr msg,
  const rclcpp::MessageInfo & info)
{
  const int64_t api_id = msg->header.identity.api_id;
  if (api_id != kApiIdArmActionExecute) {
    return;
  }
  const auto self_gid = arm_action_client_->requestPublisherGid();
  if (self_gid && gid_equal(*self_gid, info.get_rmw_message_info().publisher_gid)) {
    return;
  }
  bool joint_owned = false;
  {
    std::lock_guard<std::mutex> lock(upper_body_mutex_);
    joint_owned = upper_body_arbiter_.jointStateOwnsControl();
  }
  if (joint_owned) {
    upper_body_fault("EXTERNAL_ARM_ACTION_REQUEST_7106");
  }
}

void RobotControllerNode::upper_body_worker_loop()
{
  auto next = std::chrono::steady_clock::now();
  while (upper_body_running_) {
    next += upper_body_session_.policy().control_period;
    std::this_thread::sleep_until(next);
    const auto wake_time = std::chrono::steady_clock::now();
    if (wake_time - next > upper_body_session_.policy().control_period) {
      next = wake_time;
    }

    UpperBodyStepResult candidate;
    std::string fault_reason;
    {
      std::lock_guard<std::mutex> lock(upper_body_mutex_);
      const auto now_steady = steady_now_();
      const auto phase = upper_body_arbiter_.state().phase;
      const bool acquiring = phase == UpperBodyPhase::ACQUIRING;
      const bool active = phase == UpperBodyPhase::ACTIVE;
      const bool releasing = phase == UpperBodyPhase::RELEASING;
      if (!acquiring && !active && !releasing) {
        continue;
      }

      if (!lowstate_valid_ || now_steady - lowstate_time_ >
        upper_body_session_.policy().lowstate_timeout)
      {
        fault_reason = "LOWSTATE_TIMEOUT";
      } else if (std::find(
          upper_body_session_.policy().allowed_fsm_ids.begin(),
          upper_body_session_.policy().allowed_fsm_ids.end(),
          upper_fsm_id_) == upper_body_session_.policy().allowed_fsm_ids.end() ||
        upper_fsm_time_ == SteadyTime{} || now_steady - upper_fsm_time_ >
        upper_body_session_.policy().fsm_timeout)
      {
        fault_reason = "FSM_UNSUPPORTED_OR_STALE: id=" + std::to_string(upper_fsm_id_) +
          " age_ms=" + std::to_string(age_ms(now_steady, upper_fsm_time_));
      } else if (upper_transition_active_) {
        fault_reason = "ROBOT_SAFETY_STATE_ACTIVE";
      }
      if (fault_reason.empty() && !emergency_hold_ && (active || acquiring) &&
        upper_body_session_.hasValidCommand() &&
        now_steady - upper_body_session_.lastValidCommandTime() >
        upper_body_session_.policy().command_timeout)
      {
        if (upper_body_arbiter_.beginRelease()) {
          upper_body_session_.requestRelease(upper_body_arbiter_.state().generation, now_steady);
        }
      }
      if (fault_reason.empty() && active) {
        bool excessive = false;
        const auto & q_command = upper_body_session_.commandedPosition();
        for (std::size_t i = 0; i < q_command.size(); ++i) {
          if (!upper_body_limits_[i].active) {continue;}
          excessive = excessive ||
            std::abs(q_command[i] - measured_q_[i]) >
            upper_body_session_.policy().tracking_error_limit;
        }
        if (excessive) {
          if (tracking_error_since_ == SteadyTime{}) {
            tracking_error_since_ = now_steady;
          } else if (now_steady - tracking_error_since_ >
            upper_body_session_.policy().tracking_error_duration)
          {
            fault_reason = "UPPER_BODY_TRACKING_ERROR";
          }
        } else {
          tracking_error_since_ = SteadyTime{};
        }
      }

      if (fault_reason.empty()) {
        const auto updated_phase = upper_body_arbiter_.state().phase;
        candidate = upper_body_session_.step(
          updated_phase == UpperBodyPhase::ACQUIRING,
          updated_phase == UpperBodyPhase::ACTIVE,
          updated_phase == UpperBodyPhase::RELEASING,
          now_steady);
        if (candidate.fault) {
          fault_reason = candidate.fault_reason;
        } else if (candidate.became_active) {
          upper_body_arbiter_.markActive();
        }
      }
    }
    if (!fault_reason.empty()) {
      upper_body_fault(fault_reason);
      continue;
    }
    if (!candidate.publish) {
      continue;
    }

    bool publish_failed = false;
    std::string publish_error;
    {
      std::lock_guard<std::mutex> gate(send_gate_);
      std::lock_guard<std::mutex> lock(upper_body_mutex_);
      const auto & state = upper_body_arbiter_.state();
      const bool phase_allows = state.phase == UpperBodyPhase::ACQUIRING ||
        state.phase == UpperBodyPhase::ACTIVE || state.phase == UpperBodyPhase::RELEASING;
      if (upper_abort_requested_ || state.abort_requested || !phase_allows ||
        candidate.generation != state.generation)
      {
        continue;
      }
      const auto send_time = steady_now_();
      if (!lowstate_valid_ || send_time - lowstate_time_ >
        upper_body_session_.policy().lowstate_timeout ||
        std::find(
          upper_body_session_.policy().allowed_fsm_ids.begin(),
          upper_body_session_.policy().allowed_fsm_ids.end(),
          upper_fsm_id_) == upper_body_session_.policy().allowed_fsm_ids.end() ||
        upper_fsm_time_ == SteadyTime{} || send_time - upper_fsm_time_ >
        upper_body_session_.policy().fsm_timeout || upper_transition_active_)
      {
        continue;
      }
      if (!lowcmd_sink_->publish(candidate.command, publish_error)) {
        publish_failed = true;
      } else {
        ++upper_body_published_count_;
        sdk_publish_active_ = true;
        last_send_time_ = steady_now_();
        if (candidate.release_complete) {
          sdk_publish_active_ = false;
          release_observation_pending_ = true;
          command_source_gid_.clear();
          upper_body_arbiter_.completeRelease();
          lowcmd_sink_->close();
        }
      }
    }
    if (publish_failed) {
      upper_body_fault("LOWCMD_PUBLISH_FAILED: " + publish_error);
    }
  }
}

void RobotControllerNode::upper_body_fault(const std::string & reason)
{
  upper_abort_requested_ = true;
  stop_fence_complete_ = false;
  {
    std::lock_guard<std::mutex> gate(send_gate_);
    std::lock_guard<std::mutex> lock(upper_body_mutex_);
    fault_event_time_ = steady_now_();
    ++fault_event_seq_;
    upper_body_arbiter_.fault(reason);
    upper_body_session_.abort();
    sdk_publish_active_ = false;
    stop_fence_complete_ = true;
    command_source_gid_.clear();
    lowcmd_sink_->close();
  }
  arm_state_cv_.notify_all();
  RCLCPP_ERROR(get_logger(), "Upper-body control FAULT: %s", reason.c_str());
}

bool RobotControllerNode::upper_body_preconditions(std::string & reason, bool refresh_fsm)
{
  if (!upper_body_limits_valid_) {
    reason = "UPPER_BODY_CONFIGURATION_FAULT: " + upper_body_configuration_error_;
    return false;
  }
  if (has_parameter("use_sim_time") && get_parameter("use_sim_time").as_bool()) {
    reason = "USE_SIM_TIME_NOT_ALLOWED_FOR_HARDWARE_CONTROL";
    return false;
  }
  if (refresh_fsm) {
    const auto result = loco_client_->getFsmId(api_timeout_sec_);
    if (!result.success) {
      reason = "FRESH_FSM_READ_REQUIRED";
      return false;
    }
    std::lock_guard<std::mutex> lock(upper_body_mutex_);
    upper_fsm_id_ = result.fsm_id;
    upper_fsm_time_ = steady_now_();
  }

  bool arm_normal = false;
  {
    std::lock_guard<std::mutex> arm_lock(arm_state_mutex_);
    arm_normal = arm_state_valid_ && !arm_state_malformed_ &&
      arm_action_id_ == kArmActionNormalId && !arm_holding_ && !arm_request_pending_ &&
      steady_now_() - arm_state_time_ <= upper_body_session_.policy().arm_state_timeout;
  }
  std::lock_guard<std::mutex> lock(upper_body_mutex_);
  const auto now_steady = steady_now_();
  if (std::find(
      upper_body_session_.policy().allowed_fsm_ids.begin(),
      upper_body_session_.policy().allowed_fsm_ids.end(),
      upper_fsm_id_) == upper_body_session_.policy().allowed_fsm_ids.end())
  {
    reason = "G1_ALLOWED_FSM_REQUIRED";
    return false;
  }
  if (upper_transition_active_ || emergency_latched_) {
    reason = "POSE_TRANSITION_ACTIVE";
    return false;
  }
  if (!lowstate_valid_ || now_steady - lowstate_time_ >
    upper_body_session_.policy().lowstate_timeout)
  {
    reason = "FRESH_LOWSTATE_REQUIRED";
    return false;
  }
  for (std::size_t i = 0; i < measured_q_.size(); ++i) {
    if (!upper_body_limits_[i].active) {continue;}
    if (!std::isfinite(measured_q_[i]) || !std::isfinite(measured_dq_[i])) {
      reason = "UPPER_BODY_FEEDBACK_INVALID: " + upper_body_limits_[i].name +
        " has non-finite q or dq";
      return false;
    }
    if (measured_q_[i] < upper_body_limits_[i].lower ||
      measured_q_[i] > upper_body_limits_[i].upper)
    {
      reason = "UPPER_BODY_FEEDBACK_OUT_OF_RANGE: " + upper_body_limits_[i].name +
        " q=" + std::to_string(measured_q_[i]) + " range=[" +
        std::to_string(upper_body_limits_[i].lower) + "," +
        std::to_string(upper_body_limits_[i].upper) + "]";
      return false;
    }
    if (std::abs(measured_dq_[i]) > upper_body_session_.policy().max_acquire_measured_velocity) {
      reason = "UPPER_BODY_NOT_STATIONARY: " + upper_body_limits_[i].name +
        " dq=" + std::to_string(measured_dq_[i]) + " max=" +
        std::to_string(upper_body_session_.policy().max_acquire_measured_velocity);
      return false;
    }
  }
  if (!arm_normal) {
    reason = "FRESH_ARM_STATE_REQUIRED";
    return false;
  }
  return true;
}

void RobotControllerNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  const bool finite = std::isfinite(msg->linear.x) && std::isfinite(msg->linear.y) &&
    std::isfinite(msg->linear.z) && std::isfinite(msg->angular.x) &&
    std::isfinite(msg->angular.y) && std::isfinite(msg->angular.z);
  if (!finite) {
    std::lock_guard<std::mutex> lock(motion_state_mutex_);
    disable_and_priority_stop_locked("Non-finite /cmd_vel values");
    return;
  }
  const float vx = std::clamp(
    static_cast<float>(msg->linear.x), static_cast<float>(-max_linear_x_),
    static_cast<float>(max_linear_x_));
  const float vy = std::clamp(
    static_cast<float>(msg->linear.y), static_cast<float>(-max_linear_y_),
    static_cast<float>(max_linear_y_));
  const float omega = std::clamp(
    static_cast<float>(msg->angular.z), static_cast<float>(-max_angular_z_),
    static_cast<float>(max_angular_z_));
  std::lock_guard<std::mutex> lock(motion_state_mutex_);
  if (emergency_latched_) {return;}
  last_cmd_steady_ = steady_now_();
  last_raw_cmd_.vx = vx;
  last_raw_cmd_.vy = vy;
  last_raw_cmd_.omega = omega;
  last_raw_cmd_.stamp = now();
  last_raw_cmd_.is_stop = false;
  has_cmd_vel_sample_ = true;
}

void RobotControllerNode::emergency_latch_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!msg->data) {return;}
  {
    std::lock_guard<std::mutex> gate(send_gate_);
    std::lock_guard<std::mutex> lock(upper_body_mutex_);
    if (emergency_latched_.exchange(true)) {return;}
    upper_body_arbiter_.invalidatePendingCommand();
  }
  {
    std::lock_guard<std::mutex> lock(motion_state_mutex_);
    has_cmd_vel_sample_ = false;
    last_cmd_steady_ = SteadyTime{};
    enqueue_priority_stop_locked();
  }
  arm_state_cv_.notify_all();
}

void RobotControllerNode::emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  bool changed = false;
  {
    std::lock_guard<std::mutex> lock(motion_state_mutex_);
    changed = msg->data != emergency_stop_active_;
    emergency_stop_active_ = msg->data;
    emergency_stop_received_ = true;
  }
  if (!changed) {return;}

  if (!msg->data) {
    // 物理入力の解除でホールドだけを解放する。受付ラッチは変更しない。
    std::lock_guard<std::mutex> gate(send_gate_);
    std::lock_guard<std::mutex> lock(upper_body_mutex_);
    if (emergency_hold_) {
      emergency_hold_ = false;
      if (upper_body_arbiter_.beginRelease()) {
        upper_body_session_.requestRelease(
          upper_body_arbiter_.state().generation, steady_now_());
      }
    }
    return;
  }

  bool release_arm = false;
  bool arm_normal_fresh = false;
  std::string hold_error;
  {
    std::lock_guard<std::mutex> gate(send_gate_);
    emergency_latched_ = true;
    {
      std::lock_guard<std::mutex> lock(arm_state_mutex_);
      release_arm = arm_request_pending_ ||
        (arm_state_valid_ && (arm_action_id_ != kArmActionNormalId || arm_holding_));
      arm_normal_fresh = arm_state_valid_ && !arm_state_malformed_ && !release_arm &&
        steady_now_() - arm_state_time_ <= upper_body_session_.policy().arm_state_timeout;
    }
    std::lock_guard<std::mutex> lock(upper_body_mutex_);
    release_arm = release_arm ||
      upper_body_arbiter_.state().owner == UpperBodyOwner::ARM_ACTION;
    emergency_hold_ = false;
    if (release_arm) {
      // Arm Action 中は 99 のみ。既存の関節出力もここで閉じる。
      upper_abort_requested_ = true;
      upper_body_session_.abort();
      upper_body_arbiter_.markUnknown("EMERGENCY_ARM_RELEASE_PENDING");
      sdk_publish_active_ = false;
      lowcmd_sink_->close();
      ++emergency_generation_;
    } else {
      const auto stamp = steady_now_();
      const auto & policy = upper_body_session_.policy();
      const bool fresh = arm_normal_fresh && lowstate_valid_ &&
        stamp - lowstate_time_ <= policy.lowstate_timeout &&
        upper_fsm_time_ != SteadyTime{} && stamp - upper_fsm_time_ <= policy.fsm_timeout &&
        std::find(policy.allowed_fsm_ids.begin(), policy.allowed_fsm_ids.end(),
          upper_fsm_id_) != policy.allowed_fsm_ids.end() && !upper_transition_active_;
      if (!fresh || !upper_body_limits_valid_) {
        hold_error = "EMERGENCY_HOLD_STATE_UNAVAILABLE";
      } else if (upper_body_arbiter_.beginEmergencyHold(hold_error)) {
        const auto generation = upper_body_arbiter_.state().generation;
        if (upper_body_session_.beginAcquisition(measured_q_, generation, stamp, hold_error)) {
          upper_body_session_.holdMeasured(measured_q_, generation);
          upper_abort_requested_ = false;
          emergency_hold_ = true;
          tracking_error_since_ = SteadyTime{};
        }
      }
    }
  }
  if (release_arm) {
    {
      std::lock_guard<std::mutex> lock(emergency_mutex_);
      emergency_release_pending_ = true;
    }
    emergency_cv_.notify_one();
  } else if (!hold_error.empty()) {
    upper_body_fault(hold_error);
  }
  {
    std::lock_guard<std::mutex> lock(motion_state_mutex_);
    has_cmd_vel_sample_ = false;
    last_cmd_steady_ = SteadyTime{};
    disable_and_priority_stop_locked("Emergency stop activated");
  }
  arm_state_cv_.notify_all();
}

void RobotControllerNode::emergency_worker_loop()
{
  while (emergency_running_) {
    uint64_t generation;
    {
      std::unique_lock<std::mutex> lock(emergency_mutex_);
      emergency_cv_.wait(lock, [this]() {
        return emergency_release_pending_ || !emergency_running_;
      });
      if (!emergency_running_) {break;}
      emergency_release_pending_ = false;
      generation = emergency_generation_;
    }
    const auto result = arm_action_client_->emergencyRelease(
      2.0, [this, generation](
        const UnitreeApiClient::PublishOperation & publish, std::string & error) {
        std::lock_guard<std::mutex> gate(send_gate_);
        if (!emergency_running_ || !emergency_latched_ ||
          generation != emergency_generation_)
        {
          error = "停止要求の世代が更新されました";
          return false;
        }
        publish();
        return true;
      });
    emergency_release_status_ = result.status_code;
    if (result.success) {
      RCLCPP_WARN(get_logger(), "緊急停止の Arm Action 99 が受理されました。実行状態は別途監視します");
    } else {
      RCLCPP_ERROR(get_logger(), "緊急停止の Arm Action 99: %s", result.message.c_str());
    }
  }
}

void RobotControllerNode::control_loop_timer()
{
  const rclcpp::Time ros_now = now();
  CommandTarget target;
  bool should_enqueue = false;
  {
    std::lock_guard<std::mutex> lock(motion_state_mutex_);
    const bool fsm_allowed = std::find(
      allowed_motion_fsm_ids_.begin(), allowed_motion_fsm_ids_.end(),
      current_fsm_id_) != allowed_motion_fsm_ids_.end();
    if (emergency_latched_ || emergency_stop_active_ || transition_active_ || !fsm_allowed ||
      fsm_state_time_ == SteadyTime{} ||
      steady_now_() - fsm_state_time_ > std::chrono::seconds(2) ||
      consecutive_api_failures_ >= max_consecutive_api_failures_) {
      if (!last_target_was_zero_) {enqueue_priority_stop_locked();}
      return;
    }
    const double command_age = last_cmd_steady_ == SteadyTime{} ?
      999.0 : std::chrono::duration<double>(steady_now_() - last_cmd_steady_).count();
    float target_vx = 0.0F;
    float target_vy = 0.0F;
    float target_omega = 0.0F;
    if (command_age <= cmd_vel_timeout_sec_ && !last_raw_cmd_.is_stop) {
      target_vx = last_raw_cmd_.vx;
      target_vy = last_raw_cmd_.vy;
      target_omega = last_raw_cmd_.omega;
    }
    double dt = last_control_time_.nanoseconds() == 0 ?
      0.05 : (ros_now - last_control_time_).seconds();
    dt = std::clamp(dt, 0.001, 0.2);
    last_control_time_ = ros_now;
    curr_slew_vx_ += std::clamp(
      target_vx - curr_slew_vx_,
      static_cast<float>(-max_linear_accel_ * dt),
      static_cast<float>(max_linear_accel_ * dt));
    curr_slew_vy_ += std::clamp(
      target_vy - curr_slew_vy_,
      static_cast<float>(-max_linear_accel_ * dt),
      static_cast<float>(max_linear_accel_ * dt));
    curr_slew_omega_ += std::clamp(
      target_omega - curr_slew_omega_,
      static_cast<float>(-max_angular_accel_ * dt),
      static_cast<float>(max_angular_accel_ * dt));
    bool is_zero = std::abs(curr_slew_vx_) < command_epsilon_ &&
      std::abs(curr_slew_vy_) < command_epsilon_ &&
      std::abs(curr_slew_omega_) < command_epsilon_;
    if (is_zero) {
      curr_slew_vx_ = curr_slew_vy_ = curr_slew_omega_ = 0.0F;
    }
    const double api_delta = (ros_now - last_api_enqueue_time_).seconds();
    const double minimum = 1.0 / std::max(0.1, api_update_rate_hz_) - 1e-4;
    if ((!is_zero || !last_target_was_zero_) && api_delta >= minimum) {
      target.vx = curr_slew_vx_;
      target.vy = curr_slew_vy_;
      target.omega = curr_slew_omega_;
      target.is_stop = is_zero;
      target.stamp = ros_now;
      should_enqueue = true;
      last_api_enqueue_time_ = ros_now;
      last_target_was_zero_ = is_zero;
    }
  }
  if (should_enqueue) {
    enqueue_command(target);
  }
}

void RobotControllerNode::enqueue_command(const CommandTarget & target)
{
  std::lock_guard<std::mutex> lock(mailbox_mutex_);
  if (target.priority_stop) {
    priority_stop_pending_ = true;
    ++motion_generation_;
  } else if (priority_stop_pending_) {
    return;
  }
  mailbox_target_ = target;
  mailbox_target_.generation = motion_generation_;
  mailbox_has_cmd_ = true;
  mailbox_cv_.notify_one();
}

void RobotControllerNode::disable_and_priority_stop_locked(const std::string & reason)
{
  RCLCPP_WARN(get_logger(), "Motion control output suspended: %s", reason.c_str());
  enqueue_priority_stop_locked();
}

void RobotControllerNode::enqueue_priority_stop_locked()
{
  curr_slew_vx_ = curr_slew_vy_ = curr_slew_omega_ = 0.0F;
  last_target_was_zero_ = true;
  CommandTarget target;
  target.is_stop = true;
  target.priority_stop = true;
  target.stamp = now();
  enqueue_command(target);
}

void RobotControllerNode::api_worker_loop()
{
  while (worker_running_) {
    CommandTarget target;
    {
      std::unique_lock<std::mutex> lock(mailbox_mutex_);
      mailbox_cv_.wait(lock, [this]() {return mailbox_has_cmd_ || !worker_running_;});
      if (!worker_running_) {
        break;
      }
      target = mailbox_target_;
      mailbox_has_cmd_ = false;
    }
    worker_busy_ = true;
    std::lock_guard<std::mutex> send_lock(motion_send_gate_);
    if (!target.is_stop && (emergency_latched_ || target.generation != motion_generation_)) {
      worker_busy_ = false;
      continue;
    }
    if (!target.is_stop) {
      std::lock_guard<std::mutex> lock(motion_state_mutex_);
      const auto send_time = steady_now_();
      const bool fsm_allowed = std::find(
        allowed_motion_fsm_ids_.begin(), allowed_motion_fsm_ids_.end(), current_fsm_id_) !=
        allowed_motion_fsm_ids_.end();
      if (emergency_latched_ || emergency_stop_active_ || transition_active_ || !fsm_allowed ||
        fsm_state_time_ == SteadyTime{} ||
        send_time - fsm_state_time_ > std::chrono::seconds(2) ||
        last_cmd_steady_ == SteadyTime{} ||
        std::chrono::duration<double>(send_time - last_cmd_steady_).count() >
        cmd_vel_timeout_sec_ || consecutive_api_failures_ >= max_consecutive_api_failures_)
      {
        target.is_stop = true;
      }
    }
    const auto result = target.is_stop ?
      loco_client_->stopMove(velocity_duration_sec_) :
      loco_client_->setVelocity(
      target.vx, target.vy, target.omega, static_cast<float>(velocity_duration_sec_));
    worker_busy_ = false;
    if (target.priority_stop) {
      std::lock_guard<std::mutex> mailbox_lock(mailbox_mutex_);
      priority_stop_pending_ = false;
    }
    std::lock_guard<std::mutex> lock(motion_state_mutex_);
    last_api_status_code_ = result.status_code;
    if (result.success) {
      consecutive_api_failures_ = 0;
      has_sent_motion_command_ = true;
    } else {
      ++consecutive_api_failures_;
    }
  }
}

bool RobotControllerNode::enqueue_control_job(std::function<void()> job)
{
  std::lock_guard<std::mutex> lock(control_job_mutex_);
  if (!control_job_running_ || control_job_busy_ || pending_control_job_) {
    return false;
  }
  pending_control_job_ = std::move(job);
  control_job_cv_.notify_one();
  return true;
}

void RobotControllerNode::control_job_worker_loop()
{
  while (control_job_running_) {
    std::function<void()> job;
    {
      std::unique_lock<std::mutex> lock(control_job_mutex_);
      control_job_cv_.wait(
        lock, [this]() {return pending_control_job_ || !control_job_running_;});
      if (!control_job_running_) {
        break;
      }
      job = std::move(pending_control_job_);
      pending_control_job_ = {};
      control_job_busy_ = true;
    }
    job();
    control_job_busy_ = false;
  }
}

void RobotControllerNode::shutdown_workers()
{
  emergency_running_ = false;
  emergency_cv_.notify_all();
  if (emergency_thread_.joinable()) {emergency_thread_.join();}
  upper_body_running_ = false;
  if (upper_body_thread_.joinable()) {
    upper_body_thread_.join();
  }
  lowcmd_sink_->close();

  control_job_running_ = false;
  arm_state_cv_.notify_all();
  control_job_cv_.notify_all();
  if (control_job_thread_.joinable()) {
    control_job_thread_.join();
  }

  if (worker_running_.exchange(false)) {
    bool sent_motion = false;
    {
      std::lock_guard<std::mutex> lock(motion_state_mutex_);
      sent_motion = has_sent_motion_command_;
    }
    mailbox_cv_.notify_all();
    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }
    if (sent_motion) {
      loco_client_->stopMove(1.0);
    }
  }
}

void RobotControllerNode::publish_diagnostics_timer()
{
  publish_diagnostics(now());
}

void RobotControllerNode::publish_diagnostics(const rclcpp::Time & stamp)
{
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = stamp;

  diagnostic_msgs::msg::DiagnosticStatus motion;
  motion.name = "robot_controller: Integrated State & Safety";
  motion.hardware_id = "Unitree_G1_Controller";
  {
    std::lock_guard<std::mutex> lock(motion_state_mutex_);
    motion.level = (emergency_latched_ || emergency_stop_active_ || transition_active_ || consecutive_api_failures_ > 0) ?
      diagnostic_msgs::msg::DiagnosticStatus::WARN :
      diagnostic_msgs::msg::DiagnosticStatus::OK;
    motion.message = emergency_stop_active_ ? "Emergency Stop Active" :
      (emergency_latched_ ? "Emergency Stop Latched: restart required" :
      (transition_active_ ? "Pose Transition Active" : "Ready"));
    auto add = [&](const std::string & key, const std::string & value) {
        diagnostic_msgs::msg::KeyValue item;
        item.key = key;
        item.value = value;
        motion.values.push_back(item);
      };
    add("emergency_stop", emergency_stop_active_ ? "true" : "false");
    add("emergency_latched", emergency_latched_ ? "true" : "false");
    add("emergency_arm_release_status", std::to_string(emergency_release_status_.load()));
    add("transition_active", transition_active_ ? "true" : "false");
    add("current_fsm_id", std::to_string(current_fsm_id_));
    add("consecutive_api_failures", std::to_string(consecutive_api_failures_));
  }
  array.status.push_back(motion);

  diagnostic_msgs::msg::DiagnosticStatus upper;
  upper.name = "robot_controller: Upper Body Ownership";
  upper.hardware_id = "Unitree_G1_ArmSdk";
  {
    std::lock_guard<std::mutex> lock(upper_body_mutex_);
    const auto now_steady = steady_now_();
    const auto & state = upper_body_arbiter_.state();
    upper.level = state.phase == UpperBodyPhase::FAULT ?
      diagnostic_msgs::msg::DiagnosticStatus::ERROR :
      (state.phase == UpperBodyPhase::READY ?
      diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN);
    upper.message = UpperBodyArbiter::phaseName(state.phase);
    auto add = [&](const std::string & key, const std::string & value) {
        diagnostic_msgs::msg::KeyValue item;
        item.key = key;
        item.value = value;
        upper.values.push_back(item);
      };
    add("upper_body.owner", UpperBodyArbiter::ownerName(state.owner));
    add("upper_body.phase", UpperBodyArbiter::phaseName(state.phase));
    add("upper_body.generation", std::to_string(state.generation));
    add("upper_body.enabled_intent", state.enabled_intent ? "true" : "false");
    add("upper_body.canonical_joint_fields", "29");
    add("upper_body.active_joints", std::to_string(std::count_if(
      upper_body_limits_.begin(), upper_body_limits_.end(),
      [](const auto & limit) {return limit.active;})));
    add("upper_body.command_source_gid", command_source_gid_);
    add("upper_body.command_rx_seq", std::to_string(command_rx_seq_));
    add("upper_body.command_age_ms", std::to_string(
        age_ms(now_steady, upper_body_session_.lastValidCommandTime())));
    add("upper_body.lowstate_tick", std::to_string(lowstate_tick_));
    add("upper_body.lowstate_age_ms", std::to_string(age_ms(now_steady, lowstate_time_)));
    add("upper_body.duplicate_ticks", std::to_string(duplicate_lowstate_ticks_));
    add("upper_body.lowstate_tick_rebases", std::to_string(lowstate_tick_rebases_));
    add("upper_body.fsm_id", std::to_string(upper_fsm_id_));
    add("upper_body.fsm_age_ms", std::to_string(age_ms(now_steady, upper_fsm_time_)));
    add("upper_body.sdk_publish_active", sdk_publish_active_ ? "true" : "false");
    add("upper_body.published_count", std::to_string(upper_body_published_count_));
    add("upper_body.last_send_age_ms", std::to_string(age_ms(now_steady, last_send_time_)));
    add("upper_body.weight", std::to_string(upper_body_session_.weight()));
    add("upper_body.emergency_hold", emergency_hold_ ? "true" : "false");
    add("upper_body.release_zero_sent", upper_body_session_.releaseZeroSent() ? "true" : "false");
    add("upper_body.release_observation_pending", release_observation_pending_ ? "true" : "false");
    add("upper_body.fault_reason", state.fault_reason);
    add("upper_body.abort_requested", state.abort_requested ? "true" : "false");
    add("upper_body.stop_fence_complete", stop_fence_complete_ ? "true" : "false");
    add("upper_body.local_arm_action_exclusion", "enforced");
    add("upper_body.external_motion_prevention", "not_guaranteed");
    add("upper_body.external_motion_observation", "enabled");
    add("upper_body.remote_release_ack_available", "false");
  }
  {
    std::lock_guard<std::mutex> arm_lock(arm_state_mutex_);
    auto add = [&](const std::string & key, const std::string & value) {
        diagnostic_msgs::msg::KeyValue item;
        item.key = key;
        item.value = value;
        upper.values.push_back(item);
      };
    add("upper_body.arm_action_id", std::to_string(arm_action_id_));
    add("upper_body.holding", arm_holding_ ? "true" : "false");
    add("upper_body.arm_state_seq", std::to_string(arm_state_seq_));
    add("upper_body.arm_state_age_ms", std::to_string(age_ms(steady_now_(), arm_state_time_)));
  }
  array.status.push_back(upper);
  diag_pub_->publish(array);
}

std::string RobotControllerNode::gid_to_string(const rmw_gid_t & gid)
{
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (std::size_t i = 0; i < RMW_GID_STORAGE_SIZE; ++i) {
    stream << std::setw(2) << static_cast<unsigned int>(gid.data[i]);
  }
  return stream.str();
}

}  // namespace erasers_g1_common
