// Copyright (c) 2026 Nakatogawa Laboratory. All rights reserved.

#include "erasers_g1_common/arm_action_protocol.hpp"
#include <nlohmann/json.hpp>

namespace erasers_g1_common
{

std::string create_execute_action_parameter(int32_t action_id)
{
  nlohmann::json js;
  js["data"] = action_id;
  return js.dump();
}

bool parse_arm_action_state(
  const std::string & json_str,
  ArmActionState & state,
  std::string * error_msg)
{
  if (json_str.empty()) {
    if (error_msg) {
      *error_msg = "Empty state string received";
    }
    return false;
  }

  try {
    const auto js = nlohmann::json::parse(json_str);
    if (!js.is_object()) {
      if (error_msg) {
        *error_msg = "Root JSON entity is not an object";
      }
      return false;
    }

    if (!js.contains("id") || !js["id"].is_number_integer()) {
      if (error_msg) {
        *error_msg = "Missing or invalid 'id' field";
      }
      return false;
    }

    if (!js.contains("name") || !js["name"].is_string()) {
      if (error_msg) {
        *error_msg = "Missing or invalid 'name' field";
      }
      return false;
    }

    if (!js.contains("holding") || !js["holding"].is_boolean()) {
      if (error_msg) {
        *error_msg = "Missing or invalid 'holding' field";
      }
      return false;
    }

    state.valid = true;
    state.id = js["id"].get<int32_t>();
    state.name = js["name"].get<std::string>();
    state.holding = js["holding"].get<bool>();
    return true;
  } catch (const nlohmann::json::exception & e) {
    if (error_msg) {
      *error_msg = std::string("JSON parse error: ") + e.what();
    }
    return false;
  } catch (const std::exception & e) {
    if (error_msg) {
      *error_msg = std::string("Standard exception during parse: ") + e.what();
    }
    return false;
  }
}

std::string format_api_error_message(int32_t status_code)
{
  switch (status_code) {
    case 0:
      return "OK";
    case kArmActionErrArmSdk:
      return "The topic rt/armsdk is occupied (code: 7400).";
    case kArmActionErrHolding:
      return "The arm is holding. Expecting release action(99) or the same last action id (code: 7401).";
    case kArmActionErrInvalidActionId:
      return "Invalid action id (code: 7402).";
    case kArmActionErrInvalidFsmId:
      return "Invalid FSM id for arm action (code: 7404).";
    default:
      return "Unitree Arm Action API error with status code: " + std::to_string(status_code);
  }
}

}  // namespace erasers_g1_common
