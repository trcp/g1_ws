// Copyright (c) 2026 Nakatogawa Laboratory. All rights reserved.

#ifndef ERASERS_G1_COMMON__LED_CLIENT_INTERFACE_HPP_
#define ERASERS_G1_COMMON__LED_CLIENT_INTERFACE_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include "erasers_g1_common/unitree_api_client.hpp"

namespace erasers_g1_common
{

class LedClientInterface
{
public:
  virtual ~LedClientInterface() = default;

  virtual bool set_led_color(
    uint8_t r,
    uint8_t g,
    uint8_t b,
    double timeout_sec = 2.0) = 0;
};

class UnitreeLedClient : public LedClientInterface
{
public:
  // Unitree G1 VuiClient ROBOT_API_ID_AUDIO_SET_RGB_LED = 1010
  static constexpr int64_t kApiIdSetRgbLed = 1010;

  explicit UnitreeLedClient(
    rclcpp::Node * node,
    const std::string & req_topic = "/api/voice/request",
    const std::string & res_topic = "/api/voice/response")
  : node_(node),
    api_client_(std::make_shared<UnitreeApiClient>(node, req_topic, res_topic))
  {
  }

  explicit UnitreeLedClient(
    rclcpp::Node * node,
    std::shared_ptr<UnitreeApiClient> api_client)
  : node_(node),
    api_client_(std::move(api_client))
  {
  }

  bool set_led_color(
    uint8_t r,
    uint8_t g,
    uint8_t b,
    double timeout_sec = 2.0) override
  {
    if (!api_client_) {
      if (node_) {
        RCLCPP_ERROR(node_->get_logger(), "LedClient: api_client_ is null");
      }
      return false;
    }

    nlohmann::json param;
    param["R"] = r;
    param["G"] = g;
    param["B"] = b;

    const std::string param_str = param.dump();
    const auto result = api_client_->call(kApiIdSetRgbLed, param_str, timeout_sec);

    if (result.timed_out) {
      if (node_) {
        RCLCPP_ERROR(
          node_->get_logger(),
          "LedClient: call timed out (api_id=%ld, elapsed=%.3fs)",
          kApiIdSetRgbLed, result.elapsed_sec);
      }
      return false;
    }

    if (result.publish_failed) {
      if (node_) {
        RCLCPP_ERROR(
          node_->get_logger(),
          "LedClient: publish failed: %s", result.error_message.c_str());
      }
      return false;
    }

    if (!result.received || result.status_code != 0) {
      if (node_) {
        RCLCPP_WARN(
          node_->get_logger(),
          "LedClient: request failed with status_code=%d", result.status_code);
      }
      return false;
    }

    return true;
  }

private:
  rclcpp::Node * node_{nullptr};
  std::shared_ptr<UnitreeApiClient> api_client_;
};

}  // namespace erasers_g1_common

#endif  // ERASERS_G1_COMMON__LED_CLIENT_INTERFACE_HPP_
