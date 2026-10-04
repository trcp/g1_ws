// Copyright (c) 2026 Nakatogawa Laboratory. All rights reserved.

#ifndef ERASERS_G1_COMMON__UNITREE_API_CLIENT_HPP_
#define ERASERS_G1_COMMON__UNITREE_API_CLIENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <unitree_api/msg/request.hpp>
#include <unitree_api/msg/response.hpp>
#include <string>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <functional>
#include <map>

namespace erasers_g1_common
{

class UnitreeApiClient
{
public:
  using PublishOperation = std::function<void()>;
  using PublishGuard = std::function<bool(const PublishOperation &, std::string &)>;

  struct CallResult
  {
    bool received{false};
    bool timed_out{false};
    bool publish_failed{false};
    bool cancelled_before_publish{false};
    int32_t status_code{-1};
    int64_t identity_id{0};
    std::string data;
    std::vector<int8_t> binary;
    double elapsed_sec{0.0};
    std::string error_message;
  };

  UnitreeApiClient(
    rclcpp::Node * node,
    const std::string & request_topic,
    const std::string & response_topic);

  virtual ~UnitreeApiClient() = default;

  virtual CallResult call(
    int64_t api_id,
    const std::string & parameter,
    const std::vector<uint8_t> & binary,
    double timeout_sec);

  virtual CallResult call(
    int64_t api_id,
    const std::string & parameter,
    const std::vector<uint8_t> & binary,
    double timeout_sec,
    const PublishGuard & publish_guard);

  virtual CallResult call(
    int64_t api_id,
    const std::string & parameter,
    double timeout_sec);

  // ROS 2 トピックの要求先と応答元が両方接続されているか確認する。
  bool isReady() const;
  const rmw_gid_t & requestPublisherGid() const;

private:
  rclcpp::Node * node_;
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr request_pub_;
  rclcpp::CallbackGroup::SharedPtr response_callback_group_;
  rclcpp::Subscription<unitree_api::msg::Response>::SharedPtr response_sub_;

  std::mutex mutex_;
  std::condition_variable cv_;
  std::map<int64_t, unitree_api::msg::Response> responses_;
  std::atomic<int64_t> next_identity_id_;

  void response_callback(const unitree_api::msg::Response::SharedPtr msg);
};

}  // namespace erasers_g1_common

#endif  // ERASERS_G1_COMMON__UNITREE_API_CLIENT_HPP_
