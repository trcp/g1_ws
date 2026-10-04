// Copyright (c) 2026 Nakatogawa Laboratory. All rights reserved.

#include "erasers_g1_common/arm_action_direct_client.hpp"

#include <utility>

namespace erasers_g1_common
{

ArmActionDirectClient::ArmActionDirectClient(
  rclcpp::Node * node,
  const std::string & req_topic,
  const std::string & res_topic)
{
  api_client_ = std::make_shared<UnitreeApiClient>(node, req_topic, res_topic);
}

ArmActionDirectClient::ArmActionDirectClient(std::shared_ptr<UnitreeApiClient> api_client)
: api_client_(std::move(api_client))
{
}

ArmActionResult ArmActionDirectClient::executeAction(int32_t action_id, double timeout_sec)
{
  PublishGuard guard;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    guard = execute_publish_guard_;
  }
  return executeGuarded(action_id, timeout_sec, guard);
}

ArmActionResult ArmActionDirectClient::emergencyRelease(
  double timeout_sec, const PublishGuard & guard)
{
  return executeGuarded(kArmActionReleaseId, timeout_sec, guard);
}

ArmActionResult ArmActionDirectClient::executeGuarded(
  int32_t action_id, double timeout_sec, const PublishGuard & guard)
{
  ArmActionResult result;

  if (!api_client_) {
    result.message = "UnitreeApiClient is not initialized";
    return result;
  }

  const std::string param = create_execute_action_parameter(action_id);
  const auto call_res = api_client_->call(
    kApiIdArmActionExecute, param, {}, timeout_sec, guard);

  result.received = call_res.received;
  result.timed_out = call_res.timed_out;
  result.cancelled_before_publish = call_res.cancelled_before_publish;
  result.status_code = call_res.status_code;
  result.elapsed_sec = call_res.elapsed_sec;
  result.data = call_res.data;

  if (call_res.timed_out) {
    result.success = false;
    result.message = "API 7106 request timed out after " + std::to_string(timeout_sec) + "s";
  } else if (call_res.publish_failed) {
    result.success = false;
    result.message = "API 7106 publish failed: " + call_res.error_message;
  } else if (!call_res.received) {
    result.success = false;
    result.message = "API 7106 response not received: " + call_res.error_message;
  } else {
    result.success = (call_res.status_code == 0);
    result.message = format_api_error_message(call_res.status_code);
  }

  return result;
}

void ArmActionDirectClient::setExecutePublishGuard(PublishGuard guard)
{
  std::lock_guard<std::mutex> lock(mutex_);
  execute_publish_guard_ = std::move(guard);
}

ArmActionResult ArmActionDirectClient::getActionList(double timeout_sec)
{
  ArmActionResult result;

  if (!api_client_) {
    result.message = "UnitreeApiClient is not initialized";
    return result;
  }

  const auto call_res = api_client_->call(kApiIdArmActionGetList, "{}", timeout_sec);

  result.received = call_res.received;
  result.timed_out = call_res.timed_out;
  result.status_code = call_res.status_code;
  result.elapsed_sec = call_res.elapsed_sec;
  result.data = call_res.data;

  if (call_res.timed_out) {
    result.success = false;
    result.message = "API 7107 request timed out after " + std::to_string(timeout_sec) + "s";
  } else if (call_res.publish_failed) {
    result.success = false;
    result.message = "API 7107 publish failed: " + call_res.error_message;
  } else if (!call_res.received) {
    result.success = false;
    result.message = "API 7107 response not received: " + call_res.error_message;
  } else {
    result.success = (call_res.status_code == 0);
    result.message = format_api_error_message(call_res.status_code);
  }

  return result;
}

std::optional<rmw_gid_t> ArmActionDirectClient::requestPublisherGid() const
{
  if (!api_client_) {
    return std::nullopt;
  }
  return api_client_->requestPublisherGid();
}

}  // namespace erasers_g1_common
