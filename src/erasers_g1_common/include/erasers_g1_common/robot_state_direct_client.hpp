// Copyright (c) 2026 Nakatogawa Laboratory. All rights reserved.

#ifndef ERASERS_G1_COMMON__ROBOT_STATE_DIRECT_CLIENT_HPP_
#define ERASERS_G1_COMMON__ROBOT_STATE_DIRECT_CLIENT_HPP_

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "erasers_g1_common/unitree_api_client.hpp"

namespace erasers_g1_common
{

constexpr int64_t kRobotStateServiceSwitchApiId = 1001;
constexpr int64_t kRobotStateSetReportFreqApiId = 1002;
constexpr int64_t kRobotStateServiceListApiId = 1003;

struct RobotServiceState
{
  std::string name;
  int32_t status{0};
  int32_t protect{0};

  bool enabled() const noexcept
  {
    return status == 0;
  }
};

struct RobotServiceListResult
{
  bool success{false};
  bool received{false};
  bool timed_out{false};
  int32_t api_status_code{-1};
  double elapsed_sec{0.0};
  std::vector<RobotServiceState> services;
  std::string message;
};

struct RobotServiceSwitchResult
{
  bool success{false};
  bool received{false};
  bool timed_out{false};
  int32_t api_status_code{-1};
  // Immediate API 1001 response status. Unitree may report the pre-transition
  // state here; use ServiceList (API 1003) for the final state.
  int32_t service_status{-1};
  double elapsed_sec{0.0};
  std::string service_name;
  std::string message;
};

class RobotStateClientInterface
{
public:
  virtual ~RobotStateClientInterface() = default;

  virtual RobotServiceListResult list_services(double timeout_sec) = 0;

  virtual RobotServiceSwitchResult switch_service(
    const std::string & service_name,
    bool enable,
    double timeout_sec) = 0;
};

namespace robot_state_protocol
{

std::string make_service_switch_parameter(
  const std::string & service_name,
  bool enable);

std::vector<RobotServiceState> parse_service_list_data(
  const std::string & response_data);

RobotServiceState parse_service_switch_data(
  const std::string & response_data);

std::string sanitize_ros_service_token(const std::string & service_name);

}  // namespace robot_state_protocol

class RobotStateDirectClient : public RobotStateClientInterface
{
public:
  RobotStateDirectClient(
    rclcpp::Node * node,
    const std::string & request_topic = "/api/robot_state/request",
    const std::string & response_topic = "/api/robot_state/response");

  RobotServiceListResult list_services(double timeout_sec) override;

  RobotServiceSwitchResult switch_service(
    const std::string & service_name,
    bool enable,
    double timeout_sec) override;

private:
  std::shared_ptr<UnitreeApiClient> api_client_;
  std::mutex call_mutex_;
};

}  // namespace erasers_g1_common

#endif  // ERASERS_G1_COMMON__ROBOT_STATE_DIRECT_CLIENT_HPP_
