#pragma once
#include <chrono>
#include <future>
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace erasers_g1_common
{
// 旧サービスの名前と型を保持し、応答を新ノードから返す。
template<class Service>
class ServiceRelay
{
public:
  ServiceRelay(rclcpp::Node * node, const std::string & old_name,
    const std::string & new_name, std::chrono::seconds timeout)
  : timeout_(timeout)
  {
    client_group_ = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    service_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    client_ = node->create_client<Service>(
      new_name, rmw_qos_profile_services_default, client_group_);
    service_ = node->create_service<Service>(old_name,
      [this](std::shared_ptr<typename Service::Request> request,
        std::shared_ptr<typename Service::Response> response) {
        response->success = false;
        if (!client_->service_is_ready()) {return;}
        auto future = client_->async_send_request(request);
        if (future.wait_for(timeout_) != std::future_status::ready) {
          client_->remove_pending_request(future);
          return;
        }
        try {*response = *future.get();} catch (const std::exception &) {}
      }, rmw_qos_profile_services_default, service_group_);
  }
private:
  std::chrono::seconds timeout_;
  rclcpp::CallbackGroup::SharedPtr client_group_, service_group_;
  typename rclcpp::Client<Service>::SharedPtr client_;
  typename rclcpp::Service<Service>::SharedPtr service_;
};
}  // namespace erasers_g1_common
