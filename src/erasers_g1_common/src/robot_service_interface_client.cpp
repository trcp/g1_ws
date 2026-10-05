// Copyright (c) 2026 Nakatogawa Laboratory. All rights reserved.

#include "erasers_g1_common/robot_service_interface_client.hpp"

#include <rmw/qos_profiles.h>

#include <algorithm>
#include <chrono>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>

namespace erasers_g1_common
{
namespace
{

constexpr const char * kDefaultRequestTopic = "/api/robot_state/request";
constexpr const char * kDefaultResponseTopic = "/api/robot_state/response";
constexpr const char * kServiceName = "robot_service_interface";

std::chrono::nanoseconds seconds_to_duration(double seconds)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(seconds));
}

std::string state_label(int32_t status)
{
  if (status == 0) {
    return "enabled";
  }
  if (status == 1) {
    return "disabled";
  }
  return "status=" + std::to_string(status);
}

}  // namespace

RobotServiceInterfaceClientNode::RobotServiceInterfaceClientNode(
  const rclcpp::NodeOptions & options)
: Node("robot_service_interface_client", options)
{
  initialize(nullptr);
}

RobotServiceInterfaceClientNode::RobotServiceInterfaceClientNode(
  std::shared_ptr<RobotStateClientInterface> robot_state_client,
  const rclcpp::NodeOptions & options)
: Node("robot_service_interface_client", options)
{
  initialize(std::move(robot_state_client));
}

void RobotServiceInterfaceClientNode::initialize(
  std::shared_ptr<RobotStateClientInterface> injected_client)
{
  api_timeout_sec_ = declare_parameter<double>("api_timeout_sec", 10.0);
  refresh_period_sec_ = declare_parameter<double>("refresh_period_sec", 30.0);
  verify_after_switch_ = declare_parameter<bool>("verify_after_switch", true);
  switch_settle_delay_sec_ = declare_parameter<double>("switch_settle_delay_sec", 5.0);
  switch_timeout_sec_ = declare_parameter<double>("switch_timeout_sec", 10.0);
  verify_poll_period_sec_ = declare_parameter<double>("verify_poll_period_sec", 0.5);

  if (api_timeout_sec_ <= 0.0 || api_timeout_sec_ > 10.0 ||
    refresh_period_sec_ < 0.0 || switch_settle_delay_sec_ < 0.0 ||
    switch_timeout_sec_ <= 0.0 || switch_timeout_sec_ > 10.0 ||
    verify_poll_period_sec_ <= 0.0 ||
    (verify_after_switch_ && switch_settle_delay_sec_ >= switch_timeout_sec_))
  {
    throw std::invalid_argument(
            "Robot service interface timing parameters are invalid; "
            "api_timeout_sec and switch_timeout_sec must be in (0, 10], and "
            "switch_settle_delay_sec must be smaller than switch_timeout_sec");
  }

  service_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  discovery_callback_group_ = create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  if (injected_client) {
    robot_state_client_ = std::move(injected_client);
  } else {
    robot_state_client_ = std::make_shared<RobotStateDirectClient>(
      this, kDefaultRequestTopic, kDefaultResponseTopic);
  }

  // 初期化シーケンス: 必ず起動時に同期ディスカバリを実施
  perform_initial_discovery();

  // 単一サービスサーバー "robot_service_interface" の生成
  service_server_ = create_service<RobotServiceClient>(
    kServiceName,
    [this](
      const std::shared_ptr<RobotServiceClient::Request> request,
      std::shared_ptr<RobotServiceClient::Response> response)
    {
      handle_service_request(request, response);
    },
    rmw_qos_profile_services_default,
    service_callback_group_);

  if (refresh_period_sec_ > 0.0) {
    refresh_timer_ = create_wall_timer(
      seconds_to_duration(refresh_period_sec_),
      [this]() {
        refresh_services(false);
      },
      discovery_callback_group_);
  }

  RCLCPP_INFO(
    get_logger(),
    "Robot service interface client initialized: service=%s request_topic=%s response_topic=%s",
    service_server_->get_service_name(), kDefaultRequestTopic, kDefaultResponseTopic);
}

void RobotServiceInterfaceClientNode::perform_initial_discovery()
{
  RCLCPP_INFO(get_logger(), "Performing initial discovery of Unitree robot services...");
  const bool success = refresh_services(false);
  if (success) {
    std::size_t count = 0;
    {
      std::lock_guard<std::mutex> lock(registry_mutex_);
      count = service_states_.size();
    }
    RCLCPP_INFO(
      get_logger(),
      "Initial discovery succeeded: %zu Unitree services registered", count);
    initial_discovery_done_ = true;
  } else {
    RCLCPP_INFO(
      get_logger(),
      "Initial discovery deferred until executor starts; scheduling discovery timer (0.5s)");
    initial_discovery_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      [this]() {
        initial_discovery_timer_->cancel();
        if (refresh_services(true)) {
          std::size_t count = 0;
          {
            std::lock_guard<std::mutex> lock(registry_mutex_);
            count = service_states_.size();
          }
          RCLCPP_INFO(
            get_logger(),
            "Deferred initial discovery succeeded: %zu Unitree services registered", count);
            initial_discovery_done_ = true;
        } else {
          RCLCPP_WARN(
            get_logger(),
            "Deferred initial discovery failed. Robot may still be booting; "
            "will retry on-demand or via periodic refresh");
        }
      },
      discovery_callback_group_);
  }
}

bool RobotServiceInterfaceClientNode::refresh_services(bool emit_error_log)
{
  const RobotServiceListResult result = robot_state_client_->list_services(api_timeout_sec_);
  if (!result.success) {
    if (emit_error_log) {
      RCLCPP_ERROR(
        get_logger(), "Failed to discover Unitree services: %s", result.message.c_str());
    }
    return false;
  }

  update_service_states(result.services);
  return true;
}

std::vector<std::string> RobotServiceInterfaceClientNode::get_cached_service_names() const
{
  std::lock_guard<std::mutex> lock(registry_mutex_);
  std::vector<std::string> names;
  names.reserve(service_states_.size());
  for (const auto & entry : service_states_) {
    names.push_back(entry.first);
  }
  return names;
}

void RobotServiceInterfaceClientNode::update_service_states(
  const std::vector<RobotServiceState> & services)
{
  std::map<std::string, RobotServiceState> updated_states;
  for (const auto & state : services) {
    updated_states[state.name] = state;
  }

  std::lock_guard<std::mutex> lock(registry_mutex_);
  service_states_.swap(updated_states);
}

bool RobotServiceInterfaceClientNode::get_service_state(
  const std::string & service_name,
  RobotServiceState & state) const
{
  std::lock_guard<std::mutex> lock(registry_mutex_);
  const auto found = service_states_.find(service_name);
  if (found == service_states_.end()) {
    return false;
  }
  state = found->second;
  return true;
}

void RobotServiceInterfaceClientNode::handle_service_request(
  const std::shared_ptr<RobotServiceClient::Request> request,
  std::shared_ptr<RobotServiceClient::Response> response)
{
  std::lock_guard<std::mutex> operation_lock(operation_mutex_);

  // 要件3-3: Request.name が "NONE" または空文字の場合、一覧取得モード
  if (request->name.empty() || request->name == "NONE") {
    handle_list_services_request(response);
    return;
  }

  // 要件3-2: 通常操作モード
  handle_switch_service_request(request->name, request->enable, response);
}

void RobotServiceInterfaceClientNode::handle_list_services_request(
  std::shared_ptr<RobotServiceClient::Response> response)
{
  // 指摘4: 要求トリガー時にロボットへ最新一覧を問い合わせ・更新（オンデマンド最新化）
  const RobotServiceListResult list_result =
    robot_state_client_->list_services(api_timeout_sec_);

  std::vector<RobotServiceState> current_services;
  if (list_result.success) {
    update_service_states(list_result.services);
    current_services = list_result.services;
  } else {
    // 最新化が失敗した場合、既存キャッシュがあればフォールバックとして使用
    std::lock_guard<std::mutex> lock(registry_mutex_);
    if (!service_states_.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "On-demand ServiceList query failed (%s); falling back to cached service list",
        list_result.message.c_str());
      for (const auto & entry : service_states_) {
        current_services.push_back(entry.second);
      }
    } else {
      response->success = false;
      response->message =
        "Failed to fetch service list from robot: " + list_result.message;
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return;
    }
  }

  // 要件3-3: 各サービス名は必ず改行コード '\n' で区切って出力
  std::ostringstream oss;
  bool first = true;
  for (const auto & svc : current_services) {
    if (!first) {
      oss << "\n";
    }
    first = false;
    oss << svc.name;
  }

  response->success = true;
  response->message = oss.str();
  RCLCPP_INFO(
    get_logger(),
    "Handled list_services request: %zu services returned", current_services.size());
}

void RobotServiceInterfaceClientNode::handle_switch_service_request(
  const std::string & service_name,
  bool enable,
  std::shared_ptr<RobotServiceClient::Response> response)
{
  RobotServiceState state;
  // キャッシュに存在しない場合、オンデマンドで最新化を試みる
  if (!get_service_state(service_name, state)) {
    refresh_services(false);
    if (!get_service_state(service_name, state)) {
      response->success = false;
      response->message =
        "Unitree service '" + service_name + "' was not found in robot ServiceList";
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return;
    }
  }

  // 指摘2: 保護サービスチェックや特定サービス除外は完全排除し、直接切り替えを実行
  RobotServiceState final_state;
  std::string result_message;
  if (!switch_service_locked(service_name, enable, final_state, result_message)) {
    response->success = false;
    response->message = result_message;
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    return;
  }

  response->success = true;
  response->message = result_message;
  RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
}

bool RobotServiceInterfaceClientNode::switch_service_locked(
  const std::string & service_name,
  bool desired_enabled,
  RobotServiceState & final_state,
  std::string & result_message)
{
  RobotServiceState current_state;
  if (!get_service_state(service_name, current_state)) {
    result_message =
      "Unitree service metadata is unavailable for '" + service_name + "'";
    return false;
  }

  const int32_t desired_status = desired_enabled ? 0 : 1;
  if (current_state.status == desired_status) {
    final_state = current_state;
    result_message =
      "Unitree service '" + service_name + "' is already " +
      state_label(desired_status);
    return true;
  }

  const auto operation_start = std::chrono::steady_clock::now();
  const auto deadline = operation_start + seconds_to_duration(switch_timeout_sec_);
  const double initial_remaining_sec =
    std::chrono::duration<double>(deadline - operation_start).count();
  const double switch_call_timeout_sec = std::min(api_timeout_sec_, initial_remaining_sec);

  const RobotServiceSwitchResult switch_result = robot_state_client_->switch_service(
    service_name, desired_enabled, switch_call_timeout_sec);
  if (!switch_result.success) {
    result_message = "ServiceSwitch failed for '" + service_name + "': " +
      switch_result.message;
    return false;
  }

  final_state = current_state;
  final_state.status = switch_result.service_status;

  if (!verify_after_switch_) {
    std::lock_guard<std::mutex> lock(registry_mutex_);
    service_states_[service_name] = final_state;
    result_message = switch_result.message;
    return true;
  }

  const auto settle_deadline = std::min(
    deadline,
    operation_start + seconds_to_duration(switch_settle_delay_sec_));
  if (std::chrono::steady_clock::now() < settle_deadline) {
    std::this_thread::sleep_until(settle_deadline);
  }

  std::string verification_error;
  if (!verify_service_state(
      service_name, desired_enabled, deadline, final_state, verification_error))
  {
    result_message =
      "ServiceSwitch response was accepted, but final state verification failed: " +
      verification_error;
    return false;
  }

  const double elapsed_sec = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - operation_start).count();
  result_message =
    "Unitree service '" + service_name + "' reached " +
    state_label(final_state.status) + " after " + std::to_string(elapsed_sec) + " seconds";
  return true;
}

bool RobotServiceInterfaceClientNode::verify_service_state(
  const std::string & service_name,
  bool desired_enabled,
  const SteadyTimePoint & deadline,
  RobotServiceState & verified_state,
  std::string & error_message)
{
  const int32_t expected_status = desired_enabled ? 0 : 1;
  int32_t last_observed_status = verified_state.status;
  std::string last_verification_error;

  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    const auto now = std::chrono::steady_clock::now();
    const double remaining_sec = std::chrono::duration<double>(deadline - now).count();
    const double call_timeout_sec = std::min(api_timeout_sec_, remaining_sec);
    if (call_timeout_sec <= 0.0) {
      break;
    }

    const RobotServiceListResult list_result =
      robot_state_client_->list_services(call_timeout_sec);
    if (list_result.success) {
      update_service_states(list_result.services);
      const auto found = std::find_if(
        list_result.services.begin(), list_result.services.end(),
        [&service_name](const RobotServiceState & state) {
          return state.name == service_name;
        });

      if (found == list_result.services.end()) {
        last_verification_error =
          "ServiceList verification no longer contains '" + service_name + "'";
      } else {
        verified_state = *found;
        last_observed_status = found->status;
        if (found->status == expected_status) {
          return true;
        }
      }
    } else {
      last_verification_error = "ServiceList verification failed: " + list_result.message;
    }

    const auto after_call = std::chrono::steady_clock::now();
    if (after_call >= deadline) {
      break;
    }
    const auto wake_time = std::min(
      deadline,
      after_call + seconds_to_duration(verify_poll_period_sec_));
    std::this_thread::sleep_until(wake_time);
  }

  error_message =
    "Timed out waiting for Unitree service '" + service_name + "' to become " +
    (desired_enabled ? "enabled" : "disabled") +
    " within " + std::to_string(switch_timeout_sec_) +
    " seconds; last observed status=" + std::to_string(last_observed_status);
  if (!last_verification_error.empty()) {
    error_message += "; " + last_verification_error;
  }
  return false;
}

}  // namespace erasers_g1_common
