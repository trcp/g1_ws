// Copyright (c) 2026 Nakatogawa Laboratory. All rights reserved.

#ifndef ERASERS_G1_COMMON__ARM_ACTION_DIRECT_CLIENT_HPP_
#define ERASERS_G1_COMMON__ARM_ACTION_DIRECT_CLIENT_HPP_

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <optional>
#include <rclcpp/rclcpp.hpp>

#include "erasers_g1_common/arm_action_protocol.hpp"
#include "erasers_g1_common/unitree_api_client.hpp"

namespace erasers_g1_common
{

struct ArmActionResult
{
  bool success{false};
  bool received{false};
  bool timed_out{false};
  bool cancelled_before_publish{false};
  int32_t status_code{-1};
  std::string message;
  std::string data;
  double elapsed_sec{0.0};
};

class ArmActionClientInterface
{
public:
  using PublishOperation = UnitreeApiClient::PublishOperation;
  using PublishGuard = UnitreeApiClient::PublishGuard;

  virtual ~ArmActionClientInterface() = default;

  virtual ArmActionResult executeAction(int32_t action_id, double timeout_sec = 2.0) = 0;
  virtual ArmActionResult getActionList(double timeout_sec = 2.0) = 0;

  // 通常アクションの応答待ちとは独立した、停止専用の解放要求。
  virtual ArmActionResult emergencyRelease(double, const PublishGuard &)
  {
    ArmActionResult result;
    result.message = "停止専用の解放要求に対応していません";
    return result;
  }

  virtual std::optional<rmw_gid_t> requestPublisherGid() const
  {
    return std::nullopt;
  }

  virtual void setExecutePublishGuard(PublishGuard) {}
};

class ArmActionDirectClient : public ArmActionClientInterface
{
public:
  ArmActionDirectClient(
    rclcpp::Node * node,
    const std::string & req_topic = "/api/arm/request",
    const std::string & res_topic = "/api/arm/response");

  explicit ArmActionDirectClient(std::shared_ptr<UnitreeApiClient> api_client);

  ~ArmActionDirectClient() override = default;

  ArmActionResult executeAction(int32_t action_id, double timeout_sec = 2.0) override;
  ArmActionResult getActionList(double timeout_sec = 2.0) override;
  ArmActionResult emergencyRelease(double timeout_sec, const PublishGuard & guard) override;
  std::optional<rmw_gid_t> requestPublisherGid() const override;
  void setExecutePublishGuard(PublishGuard guard) override;

private:
  ArmActionResult executeGuarded(int32_t action_id, double timeout_sec, const PublishGuard & guard);
  std::shared_ptr<UnitreeApiClient> api_client_;
  std::mutex mutex_;
  PublishGuard execute_publish_guard_;
};

}  // namespace erasers_g1_common

#endif  // ERASERS_G1_COMMON__ARM_ACTION_DIRECT_CLIENT_HPP_
