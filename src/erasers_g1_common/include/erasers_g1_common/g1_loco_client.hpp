#ifndef ERASERS_G1_COMMON__G1_LOCO_CLIENT_HPP_
#define ERASERS_G1_COMMON__G1_LOCO_CLIENT_HPP_

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include "erasers_g1_common/unitree_api_client.hpp"

namespace erasers_g1_common
{

// G1 Official Loco API Constants
constexpr int64_t kGetFsmIdApiId = 7001;
constexpr int64_t kGetFsmModeApiId = 7002;
constexpr int64_t kSetFsmIdApiId = 7101;
constexpr int64_t kSetVelocityApiId = 7105;
constexpr int64_t kSetSpeedModeApiId = 7107;

// G1 Official FSM Constants
constexpr int32_t kFsmZeroTorque = 0;
constexpr int32_t kFsmDamp = 1;
constexpr int32_t kFsmStandUp = 4;
constexpr int32_t kFsmStart = 500;

struct LocoResult
{
  bool success{false};
  bool received{false};
  bool timed_out{false};
  int32_t status_code{-1};
  int32_t fsm_id{-1};
  int32_t fsm_mode{-1};
  std::string message;
  double elapsed_sec{0.0};
};

class LocoClientInterface
{
public:
  using RequestGuard = std::function<bool(
    int64_t, const std::string &, const UnitreeApiClient::PublishOperation &, std::string &)>;
  virtual ~LocoClientInterface() = default;
  virtual void setRequestGuard(RequestGuard) {}

  virtual LocoResult getFsmId(double timeout_sec = 2.0) = 0;
  virtual LocoResult getFsmMode(double timeout_sec = 2.0) = 0;
  virtual LocoResult setFsmId(int32_t fsm_id, double timeout_sec = 2.0) = 0;
  virtual LocoResult setVelocity(float vx, float vy, float omega, float duration = 0.3f, double timeout_sec = 2.0) = 0;
  virtual LocoResult setSpeedMode(int32_t mode, double timeout_sec = 2.0) = 0;
  virtual LocoResult stopMove(double timeout_sec = 2.0) = 0;
  virtual LocoResult callLegacy(int64_t, const std::string &, double)
  {return LocoResult{};}
};

class G1LocoClient : public LocoClientInterface
{
public:
  G1LocoClient(
    rclcpp::Node * node,
    const std::string & req_topic = "/api/sport/request",
    const std::string & res_topic = "/api/sport/response");

  ~G1LocoClient() override = default;
  void setRequestGuard(RequestGuard guard) override;

  LocoResult getFsmId(double timeout_sec = 2.0) override;
  LocoResult getFsmMode(double timeout_sec = 2.0) override;
  LocoResult setFsmId(int32_t fsm_id, double timeout_sec = 2.0) override;
  LocoResult setVelocity(float vx, float vy, float omega, float duration = 0.3f, double timeout_sec = 2.0) override;
  LocoResult setSpeedMode(int32_t mode, double timeout_sec = 2.0) override;
  LocoResult stopMove(double timeout_sec = 2.0) override;
  LocoResult callLegacy(int64_t api_id, const std::string & parameter, double timeout) override;

private:
  std::string parseErrorMessage(int32_t status_code);
  UnitreeApiClient::CallResult guardedCall(
    int64_t api_id, const std::string & parameter, double timeout_sec);
  RequestGuard request_guard_;

  std::shared_ptr<UnitreeApiClient> api_client_;
  std::mutex mutex_;
};

}  // namespace erasers_g1_common

#endif  // ERASERS_G1_COMMON__G1_LOCO_CLIENT_HPP_
