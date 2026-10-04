#include "erasers_g1_common/g1_loco_client.hpp"

#include <nlohmann/json.hpp>
#include <cmath>

namespace erasers_g1_common
{

G1LocoClient::G1LocoClient(
  rclcpp::Node * node,
  const std::string & req_topic,
  const std::string & res_topic)
{
  api_client_ = std::make_shared<UnitreeApiClient>(node, req_topic, res_topic);
}

void G1LocoClient::setRequestGuard(RequestGuard guard)
{
  std::lock_guard<std::mutex> lock(mutex_);
  request_guard_ = std::move(guard);
}

UnitreeApiClient::CallResult G1LocoClient::guardedCall(
  int64_t api_id, const std::string & parameter, double timeout_sec)
{
  RequestGuard guard;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    guard = request_guard_;
  }
  UnitreeApiClient::PublishGuard publish_guard;
  if (guard) {
    publish_guard = [guard, api_id, parameter](
      const UnitreeApiClient::PublishOperation & publish, std::string & error) {
        return guard(api_id, parameter, publish, error);
      };
  }
  return api_client_->call(api_id, parameter, {}, timeout_sec, publish_guard);
}

std::string G1LocoClient::parseErrorMessage(int32_t status_code)
{
  switch (status_code) {
    case 0:
      return "OK";
    case 7301:
      return "LocoState not available (7301)";
    case 7302:
      return "Invalid fsm id (7302)";
    case 7303:
      return "Invalid task id (7303)";
    case 7304:
      return "FSM ID return denied (7304)";
    default:
      return "Loco API status error code: " + std::to_string(status_code);
  }
}

LocoResult G1LocoClient::getFsmId(double timeout_sec)
{
  LocoResult result;

  auto api_res = guardedCall(kGetFsmIdApiId, "", timeout_sec);
  result.received = api_res.received;
  result.timed_out = api_res.timed_out;
  result.status_code = api_res.status_code;
  result.elapsed_sec = api_res.elapsed_sec;

  if (api_res.timed_out || !api_res.received) {
    result.success = false;
    result.message = "GetFsmId RPC timed out or received no response";
    return result;
  }

  if (api_res.status_code != 0) {
    result.success = false;
    result.message = parseErrorMessage(api_res.status_code);
    return result;
  }

  try {
    auto json = nlohmann::json::parse(api_res.data);
    if (json.contains("data") && json["data"].is_number_integer()) {
      result.fsm_id = json["data"].get<int32_t>();
      result.success = true;
      result.message = "OK";
    } else {
      result.success = false;
      result.message = "GetFsmId response JSON missing integer 'data' field: " + api_res.data;
    }
  } catch (const std::exception & e) {
    result.success = false;
    result.message = "GetFsmId response JSON parse error: " + std::string(e.what());
  }

  return result;
}

LocoResult G1LocoClient::getFsmMode(double timeout_sec)
{
  LocoResult result;

  auto api_res = guardedCall(kGetFsmModeApiId, "", timeout_sec);
  result.received = api_res.received;
  result.timed_out = api_res.timed_out;
  result.status_code = api_res.status_code;
  result.elapsed_sec = api_res.elapsed_sec;

  if (api_res.timed_out || !api_res.received) {
    result.success = false;
    result.message = "GetFsmMode RPC timed out or received no response";
    return result;
  }

  if (api_res.status_code != 0) {
    result.success = false;
    result.message = parseErrorMessage(api_res.status_code);
    return result;
  }

  try {
    auto json = nlohmann::json::parse(api_res.data);
    if (json.contains("data") && json["data"].is_number_integer()) {
      result.fsm_mode = json["data"].get<int32_t>();
      result.success = true;
      result.message = "OK";
    } else {
      result.success = false;
      result.message = "GetFsmMode response JSON missing integer 'data' field: " + api_res.data;
    }
  } catch (const std::exception & e) {
    result.success = false;
    result.message = "GetFsmMode response JSON parse error: " + std::string(e.what());
  }

  return result;
}

LocoResult G1LocoClient::setFsmId(int32_t fsm_id, double timeout_sec)
{
  LocoResult result;

  nlohmann::json param_json;
  param_json["data"] = fsm_id;
  std::string param_str = param_json.dump();

  auto api_res = guardedCall(kSetFsmIdApiId, param_str, timeout_sec);
  result.received = api_res.received;
  result.timed_out = api_res.timed_out;
  result.status_code = api_res.status_code;
  result.elapsed_sec = api_res.elapsed_sec;

  if (api_res.timed_out || !api_res.received) {
    result.success = false;
    result.message = "SetFsmId(" + std::to_string(fsm_id) + ") RPC timed out";
    return result;
  }

  if (api_res.status_code != 0) {
    result.success = false;
    result.message = parseErrorMessage(api_res.status_code);
    return result;
  }

  result.success = true;
  result.fsm_id = fsm_id;
  result.message = "OK";
  return result;
}

LocoResult G1LocoClient::setVelocity(float vx, float vy, float omega, float duration, double timeout_sec)
{
  LocoResult result;

  if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(omega) || !std::isfinite(duration)) {
    result.success = false;
    result.message = "SetVelocity parameters must be finite floats";
    return result;
  }

  nlohmann::json param_json;
  param_json["velocity"] = {vx, vy, omega};
  param_json["duration"] = duration;
  std::string param_str = param_json.dump();

  auto api_res = guardedCall(kSetVelocityApiId, param_str, timeout_sec);
  result.received = api_res.received;
  result.timed_out = api_res.timed_out;
  result.status_code = api_res.status_code;
  result.elapsed_sec = api_res.elapsed_sec;

  if (api_res.timed_out || !api_res.received) {
    result.success = false;
    result.message = "SetVelocity RPC timed out";
    return result;
  }

  if (api_res.status_code != 0) {
    result.success = false;
    result.message = parseErrorMessage(api_res.status_code);
    return result;
  }

  result.success = true;
  result.message = "OK";
  return result;
}

LocoResult G1LocoClient::setSpeedMode(int32_t mode, double timeout_sec)
{
  LocoResult result;

  nlohmann::json param_json;
  param_json["data"] = mode;
  std::string param_str = param_json.dump();

  auto api_res = guardedCall(kSetSpeedModeApiId, param_str, timeout_sec);
  result.received = api_res.received;
  result.timed_out = api_res.timed_out;
  result.status_code = api_res.status_code;
  result.elapsed_sec = api_res.elapsed_sec;

  if (api_res.timed_out || !api_res.received) {
    result.success = false;
    result.message = "SetSpeedMode RPC timed out";
    return result;
  }

  if (api_res.status_code != 0) {
    result.success = false;
    result.message = parseErrorMessage(api_res.status_code);
    return result;
  }

  result.success = true;
  result.message = "OK";
  return result;
}

LocoResult G1LocoClient::stopMove(double timeout_sec)
{
  return setVelocity(0.0f, 0.0f, 0.0f, 0.3f, timeout_sec);
}

LocoResult G1LocoClient::callLegacy(int64_t api_id, const std::string & parameter, double timeout)
{
  const auto api = guardedCall(api_id, parameter, timeout);
  LocoResult result;
  result.received = api.received;
  result.timed_out = api.timed_out;
  result.status_code = api.status_code;
  result.success = api.received && !api.timed_out && api.status_code == 0;
  result.message = result.success ? "OK" : "G1 API の応答が失敗しました";
  return result;
}

}  // namespace erasers_g1_common
