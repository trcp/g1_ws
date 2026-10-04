// Copyright (c) 2026 Nakatogawa Laboratory. All rights reserved.
 
#include <erasers_g1_common/mic_control_policy.hpp>

namespace erasers_g1_common
{

CloseDecision evaluate_close_result(
  CloseApiPolicy policy,
  bool local_stopped,
  bool api_call_needed,
  bool api_success,
  const std::string & api_error_msg)
{
  CloseDecision decision;
  if (!local_stopped) {
    decision.success = false;
    decision.response_message = "failed to stop local microphone receiver";
    return decision;
  }

  if (!api_call_needed) {
    decision.success = true;
    decision.response_message = "microphone streaming disabled";
    return decision;
  }

  if (api_success) {
    decision.success = true;
    decision.response_message = "microphone streaming disabled";
  } else {
    if (policy == CloseApiPolicy::REQUIRED) {
      decision.success = false;
      decision.response_message = "microphone capture disabled locally, but " + api_error_msg;
    } else {
      decision.success = true;
      decision.response_message = "microphone capture disabled locally; " + api_error_msg;
    }
  }

  return decision;
}

EnableDecision evaluate_enable_result(
  CloseApiPolicy policy,
  bool api_success,
  bool timed_out,
  bool received,
  int status_code,
  bool packet_received)
{
  EnableDecision decision;
  if (!packet_received) {
    decision.success = false;
    decision.response_message = "no fresh UDP microphone packet was received";
    return decision;
  }

  if (policy == CloseApiPolicy::REQUIRED && !api_success) {
    decision.success = false;
    if (timed_out || !received) {
      decision.response_message =
        "fresh UDP microphone packet was received, but remote microphone API was not confirmed";
    } else {
      decision.response_message =
        "fresh UDP microphone packet was received, but remote microphone API returned status_code=" +
        std::to_string(status_code);
    }
    return decision;
  }

  decision.success = true;
  if (api_success) {
    decision.response_message = "microphone streaming enabled";
  } else if (policy == CloseApiPolicy::DISABLED) {
    decision.response_message = "microphone streaming enabled using local UDP receiver";
  } else if (timed_out || !received) {
    decision.response_message = "microphone streaming enabled; remote microphone API was not confirmed";
  } else {
    decision.response_message =
      "microphone streaming enabled; remote microphone API returned status_code=" +
      std::to_string(status_code);
  }

  return decision;
}

} // namespace erasers_g1_common
