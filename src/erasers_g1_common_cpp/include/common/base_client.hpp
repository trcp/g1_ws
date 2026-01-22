#pragma once

#include <cstdint>
#include <future>
#include <map>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <utility>

#include "nlohmann/json.hpp"
#include "time_tools.hpp"
#include "unitree_api/msg/request.hpp"
#include "unitree_api/msg/response.hpp"
#include "ut_errror.hpp"

class BaseClient {
  using Request = unitree_api::msg::Request;
  using Response = unitree_api::msg::Response;

  rclcpp::Node* node_;
  std::string topic_name_request_;
  std::string topic_name_response_;
  rclcpp::Publisher<Request>::SharedPtr req_puber_;
  rclcpp::Subscription<Response>::SharedPtr req_suber_;

  std::mutex pending_mutex_;
  std::map<int64_t,
           std::shared_ptr<std::promise<std::shared_ptr<const Response>>>>
      pending_requests_;

 public:
  BaseClient(rclcpp::Node* node, const std::string& topic_name_request,
             std::string topic_name_response)
      : node_(node),
        topic_name_request_(topic_name_request),
        topic_name_response_(std::move(topic_name_response)),
        req_puber_(node_->create_publisher<Request>(topic_name_request,
                                                    rclcpp::QoS(1))) {
    req_suber_ = node_->create_subscription<Response>(
        this->topic_name_response_, rclcpp::QoS(1),
        [this](const std::shared_ptr<const Response> data) {
          std::lock_guard<std::mutex> lock(pending_mutex_);
          auto it = pending_requests_.find(data->header.identity.id);
          if (it != pending_requests_.end()) {
            it->second->set_value(data);
            pending_requests_.erase(it);
          }
        });
  }

  int32_t Call(Request req, nlohmann::json& js) {
    auto response_promise =
        std::make_shared<std::promise<std::shared_ptr<const Response>>>();
    auto response_future = response_promise->get_future();

    req.header.identity.id = unitree::common::GetSystemUptimeInNanoseconds();
    const auto identity_id = req.header.identity.id;

    {
      std::lock_guard<std::mutex> lock(pending_mutex_);
      pending_requests_[identity_id] = response_promise;
    }

    req_puber_->publish(req);
    auto status = response_future.wait_for(std::chrono::seconds(5));

    if (status == std::future_status::timeout) {
      // Cleanup on timeout
      std::lock_guard<std::mutex> lock(pending_mutex_);
      pending_requests_.erase(identity_id);
      return UT_ROBOT_TASK_TIMEOUT;
    }

    std::shared_ptr<const Response> response_ptr;
    try {
      response_ptr = response_future.get();
    } catch (...) {
      return UT_ROBOT_TASK_UNKNOWN_ERROR;
    }

    if (response_ptr->header.status.code != 0) {
      std::cout << "error code: " << response_ptr->header.status.code
                << std::endl;
      return response_ptr->header.status.code;
    }
    try {
      js = nlohmann::json::parse(response_ptr->data.data());
    } catch (nlohmann::detail::exception& e) {
    }
    return UT_ROBOT_SUCCESS;
  }

  int32_t Call(Request req) {
    nlohmann::json js;
    return Call(std::move(req), js);
  }
};