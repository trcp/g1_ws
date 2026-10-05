// Copyright (c) 2026 Nakatogawa Laboratory. All rights reserved.

#include "erasers_g1_common/robot_state_direct_client.hpp"

#include <algorithm>
#include <cctype>
#include <set>
#include <stdexcept>
#include <utility>

#include <nlohmann/json.hpp>

namespace erasers_g1_common
{
namespace robot_state_protocol
{

std::string make_service_switch_parameter(
  const std::string & service_name,
  bool enable)
{
  if (service_name.empty()) {
    throw std::invalid_argument("Unitree service name must not be empty");
  }

  nlohmann::json parameter;
  parameter["name"] = service_name;
  parameter["switch"] = enable ? 1 : 0;
  return parameter.dump();
}

std::vector<RobotServiceState> parse_service_list_data(
  const std::string & response_data)
{
  const auto json = nlohmann::json::parse(response_data);
  if (!json.is_array()) {
    throw std::invalid_argument("Robot state service list response must be a JSON array");
  }

  std::vector<RobotServiceState> services;
  services.reserve(json.size());
  std::set<std::string> names;

  for (const auto & entry : json) {
    if (!entry.is_object() ||
      !entry.contains("name") || !entry["name"].is_string() ||
      !entry.contains("status") || !entry["status"].is_number_integer() ||
      !entry.contains("protect") || !entry["protect"].is_number_integer())
    {
      throw std::invalid_argument(
              "Robot state service list entry must contain string 'name' and integer "
              "'status'/'protect' fields");
    }

    RobotServiceState state;
    state.name = entry["name"].get<std::string>();
    state.status = entry["status"].get<int32_t>();
    state.protect = entry["protect"].get<int32_t>();

    if (state.name.empty()) {
      throw std::invalid_argument("Robot state service list contains an empty service name");
    }
    if (!names.insert(state.name).second) {
      throw std::invalid_argument(
              "Robot state ServiceList contains duplicate service name: " + state.name);
    }
    services.push_back(std::move(state));
  }

  return services;
}

RobotServiceState parse_service_switch_data(
  const std::string & response_data)
{
  const auto json = nlohmann::json::parse(response_data);
  if (!json.is_object() ||
    !json.contains("name") || !json["name"].is_string() ||
    !json.contains("status") || !json["status"].is_number_integer())
  {
    throw std::invalid_argument(
            "Robot state service switch response must contain string 'name' and integer "
            "'status' fields");
  }

  RobotServiceState state;
  state.name = json["name"].get<std::string>();
  state.status = json["status"].get<int32_t>();
  state.protect = 0;
  if (state.name.empty()) {
    throw std::invalid_argument("Robot state ServiceSwitch response contains an empty name");
  }
  return state;
}

std::string sanitize_ros_service_token(const std::string & service_name)
{
  if (service_name.empty()) {
    throw std::invalid_argument("Unitree service name must not be empty");
  }

  std::string token;
  token.reserve(service_name.size() + 8U);

  for (const unsigned char character : service_name) {
    if (std::isalnum(character) != 0 || character == '_') {
      token.push_back(static_cast<char>(character));
    } else {
      token.push_back('_');
    }
  }

  while (token.find("__") != std::string::npos) {
    token.replace(token.find("__"), 2U, "_");
  }

  if (token.empty()) {
    throw std::invalid_argument("Unitree service name cannot be converted to a ROS service token");
  }

  if (std::isdigit(static_cast<unsigned char>(token.front())) != 0) {
    token.insert(0U, "service_");
  }

  return token;
}

}  // namespace robot_state_protocol

RobotStateDirectClient::RobotStateDirectClient(
  rclcpp::Node * node,
  const std::string & request_topic,
  const std::string & response_topic)
{
  if (node == nullptr) {
    throw std::invalid_argument("RobotStateDirectClient requires a valid ROS node");
  }
  if (request_topic.empty() || response_topic.empty()) {
    throw std::invalid_argument("Robot state request/response topics must not be empty");
  }

  api_client_ = std::make_shared<UnitreeApiClient>(node, request_topic, response_topic);
}

RobotServiceListResult RobotStateDirectClient::list_services(double timeout_sec)
{
  if (timeout_sec <= 0.0) {
    throw std::invalid_argument("Robot state API timeout must be greater than zero");
  }
  std::lock_guard<std::mutex> lock(call_mutex_);
  RobotServiceListResult result;

  const auto api_result = api_client_->call(kRobotStateServiceListApiId, "", timeout_sec);
  result.received = api_result.received;
  result.timed_out = api_result.timed_out;
  result.api_status_code = api_result.status_code;
  result.elapsed_sec = api_result.elapsed_sec;

  if (api_result.timed_out || !api_result.received) {
    result.message = "Robot state ServiceList request timed out or received no response";
    return result;
  }

  if (api_result.status_code != 0) {
    result.message =
      "Robot state ServiceList API returned status code " +
      std::to_string(api_result.status_code);
    return result;
  }

  try {
    result.services = robot_state_protocol::parse_service_list_data(api_result.data);
  } catch (const std::exception & error) {
    result.message = "Robot state ServiceList JSON parse error: " + std::string(error.what());
    return result;
  }

  result.success = true;
  result.message = "OK";
  return result;
}

RobotServiceSwitchResult RobotStateDirectClient::switch_service(
  const std::string & service_name,
  bool enable,
  double timeout_sec)
{
  if (timeout_sec <= 0.0) {
    throw std::invalid_argument("Robot state API timeout must be greater than zero");
  }
  std::lock_guard<std::mutex> lock(call_mutex_);
  RobotServiceSwitchResult result;
  result.service_name = service_name;

  std::string parameter;
  try {
    parameter = robot_state_protocol::make_service_switch_parameter(service_name, enable);
  } catch (const std::exception & error) {
    result.message = error.what();
    return result;
  }

  const auto api_result = api_client_->call(
    kRobotStateServiceSwitchApiId,
    parameter,
    timeout_sec);

  result.received = api_result.received;
  result.timed_out = api_result.timed_out;
  result.api_status_code = api_result.status_code;
  result.elapsed_sec = api_result.elapsed_sec;

  if (api_result.timed_out || !api_result.received) {
    result.message = "Robot state ServiceSwitch request timed out or received no response";
    return result;
  }

  if (api_result.status_code != 0) {
    result.message =
      "Robot state ServiceSwitch API returned status code " +
      std::to_string(api_result.status_code);
    return result;
  }

  RobotServiceState returned_state;
  try {
    returned_state = robot_state_protocol::parse_service_switch_data(api_result.data);
  } catch (const std::exception & error) {
    result.message = "Robot state ServiceSwitch JSON parse error: " + std::string(error.what());
    return result;
  }

  result.service_name = returned_state.name;
  result.service_status = returned_state.status;

  if (returned_state.name != service_name) {
    result.message =
      "Robot state ServiceSwitch response name mismatch: requested '" + service_name +
      "', received '" + returned_state.name + "'";
    return result;
  }

  const int32_t desired_status = enable ? 0 : 1;
  result.success = true;
  if (returned_state.status == desired_status) {
    result.message = "OK";
  } else {
    result.message =
      "Robot state ServiceSwitch request was accepted; the immediate response reported "
      "service status " + std::to_string(returned_state.status) +
      ", while the requested target is " + std::to_string(desired_status) +
      ". The final state must be confirmed with ServiceList.";
  }
  return result;
}

}  // namespace erasers_g1_common
