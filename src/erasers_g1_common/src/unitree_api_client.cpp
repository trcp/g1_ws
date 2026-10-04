// Copyright (c) 2026 Nakatogawa Laboratory. All rights reserved.

#include <erasers_g1_common/unitree_api_client.hpp>
#include <chrono>

namespace erasers_g1_common
{

UnitreeApiClient::UnitreeApiClient(
  rclcpp::Node * node,
  const std::string & request_topic,
  const std::string & response_topic)
: node_(node),
  next_identity_id_(
    std::chrono::steady_clock::now().time_since_epoch().count())
{
  request_pub_ = node_->create_publisher<unitree_api::msg::Request>(request_topic, 1);
  
  response_callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
    
  rclcpp::SubscriptionOptions options;
  options.callback_group = response_callback_group_;
  
  response_sub_ = node_->create_subscription<unitree_api::msg::Response>(
    response_topic,
    rclcpp::QoS(1),
    std::bind(&UnitreeApiClient::response_callback, this, std::placeholders::_1),
    options);
}

UnitreeApiClient::CallResult UnitreeApiClient::call(
  int64_t api_id,
  const std::string & parameter,
  double timeout_sec)
{
  const std::vector<uint8_t> empty_binary;
  return call(api_id, parameter, empty_binary, timeout_sec);
}

UnitreeApiClient::CallResult UnitreeApiClient::call(
  int64_t api_id,
  const std::string & parameter,
  const std::vector<uint8_t> & binary,
  double timeout_sec)
{
  return call(api_id, parameter, binary, timeout_sec, {});
}

UnitreeApiClient::CallResult UnitreeApiClient::call(
  int64_t api_id,
  const std::string & parameter,
  const std::vector<uint8_t> & binary,
  double timeout_sec,
  const PublishGuard & publish_guard)
{
  auto start_time = std::chrono::steady_clock::now();
  const int64_t msg_id = next_identity_id_.fetch_add(1, std::memory_order_relaxed);

  // Create request message
  unitree_api::msg::Request req;
  req.header.identity.id = msg_id;
  req.header.identity.api_id = api_id;
  req.parameter = parameter;
  req.binary = binary;

  CallResult res;
  res.identity_id = msg_id;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    responses_[msg_id] = unitree_api::msg::Response();
  }

  RCLCPP_DEBUG(node_->get_logger(), "Publishing request ID: %ld, API ID: %ld, binary size: %zu", msg_id, api_id, binary.size());
  try {
    if (publish_guard) {
      std::string cancel_reason;
      if (!publish_guard([this, &req]() {request_pub_->publish(req);}, cancel_reason)) {
        res.publish_failed = true;
        res.cancelled_before_publish = true;
        res.error_message = cancel_reason.empty() ?
          "request cancelled by the publish guard" : cancel_reason;
        res.elapsed_sec = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - start_time).count();
        std::lock_guard<std::mutex> lock(mutex_);
        responses_.erase(msg_id);
        return res;
      }
    } else {
      request_pub_->publish(req);
    }
  } catch (const std::exception & error) {
    res.publish_failed = true;
    res.error_message = error.what();
    res.elapsed_sec = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start_time).count();
    std::lock_guard<std::mutex> lock(mutex_);
    responses_.erase(msg_id);
    return res;
  }

  std::unique_lock<std::mutex> lock(mutex_);
  bool cv_timeout = false;
  auto wait_cond = [this, msg_id]() {
    return responses_[msg_id].header.identity.id == msg_id;
  };

  if (timeout_sec > 0.0) {
    auto timeout_duration = std::chrono::duration<double>(timeout_sec);
    cv_timeout = !cv_.wait_for(lock, timeout_duration, wait_cond);
  } else {
    cv_.wait(lock, wait_cond);
  }

  auto end_time = std::chrono::steady_clock::now();
  res.elapsed_sec = std::chrono::duration<double>(end_time - start_time).count();

  if (cv_timeout) {
    res.timed_out = true;
    responses_.erase(msg_id);
    return res;
  }

  // Extract the response
  unitree_api::msg::Response response = responses_[msg_id];
  responses_.erase(msg_id);

  res.received = true;
  res.status_code = response.header.status.code;
  res.data = response.data;
  res.binary = response.binary;

  return res;
}

void UnitreeApiClient::response_callback(const unitree_api::msg::Response::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  int64_t msg_id = msg->header.identity.id;
  
  if (responses_.find(msg_id) != responses_.end()) {
    responses_[msg_id] = *msg;
    cv_.notify_all();
  }
}

bool UnitreeApiClient::isReady() const
{
  return request_pub_->get_subscription_count() > 0 &&
         response_sub_->get_publisher_count() > 0;
}

const rmw_gid_t & UnitreeApiClient::requestPublisherGid() const
{
  return request_pub_->get_gid();
}

}  // namespace erasers_g1_common
