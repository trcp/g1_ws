// Copyright (c) 2026 Nakatogawa Laboratory. All rights reserved.
 
#ifndef ERASERS_G1_COMMON__MIC_CONTROL_POLICY_HPP_
#define ERASERS_G1_COMMON__MIC_CONTROL_POLICY_HPP_

#include <string>

namespace erasers_g1_common
{

enum class CloseApiPolicy
{
  DISABLED,
  BEST_EFFORT,
  REQUIRED
};

struct CloseDecision
{
  bool success;
  std::string response_message;
};

struct EnableDecision
{
  bool success;
  std::string response_message;
};

// Evaluate the final result of disabling microphone based on policy and local/remote outcomes.
CloseDecision evaluate_close_result(
  CloseApiPolicy policy,
  bool local_stopped,
  bool api_call_needed,
  bool api_success,
  const std::string & api_error_msg);

// Evaluate the final result of enabling microphone based on API call and first packet reception.
EnableDecision evaluate_enable_result(
  CloseApiPolicy policy,
  bool api_success,
  bool timed_out,
  bool received,
  int status_code,
  bool packet_received);

} // namespace erasers_g1_common

#endif // ERASERS_G1_COMMON__MIC_CONTROL_POLICY_HPP_
