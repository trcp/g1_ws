// Copyright (c) 2026 Nakatogawa Laboratory. All rights reserved.

#ifndef ERASERS_G1_COMMON__ARM_ACTION_PROTOCOL_HPP_
#define ERASERS_G1_COMMON__ARM_ACTION_PROTOCOL_HPP_

#include <cstdint>
#include <string>

namespace erasers_g1_common
{

// Unitree G1 Arm Action API IDs
constexpr int64_t kApiIdArmActionExecute = 7106;
constexpr int64_t kApiIdArmActionGetList = 7107;

// Special Action IDs
constexpr int32_t kArmActionNormalId = 0;
constexpr int32_t kArmActionReleaseId = 99;

// Unitree G1 Known Arm Action API Status Codes
constexpr int32_t kArmActionErrArmSdk = 7400;
constexpr int32_t kArmActionErrHolding = 7401;
constexpr int32_t kArmActionErrInvalidActionId = 7402;
constexpr int32_t kArmActionErrInvalidFsmId = 7404;

struct ArmActionState
{
  bool valid{false};
  int32_t id{0};
  std::string name;
  bool holding{false};
};

// Pure helper functions for protocol serialization / deserialization
std::string create_execute_action_parameter(int32_t action_id);

bool parse_arm_action_state(
  const std::string & json_str,
  ArmActionState & state,
  std::string * error_msg = nullptr);

std::string format_api_error_message(int32_t status_code);

}  // namespace erasers_g1_common

#endif  // ERASERS_G1_COMMON__ARM_ACTION_PROTOCOL_HPP_
