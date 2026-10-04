// Copyright (c) 2026 Nakatogawa Laboratory. All rights reserved.

#include <gtest/gtest.h>

#include <chrono>
#include <atomic>
#include <future>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <unitree_api/msg/request.hpp>
#include <unitree_api/msg/response.hpp>

#include "erasers_g1_common/unitree_api_client.hpp"

namespace erasers_g1_common
{
namespace
{

class ExecutorGuard
{
public:
  explicit ExecutorGuard(rclcpp::Executor & executor)
  : executor_(executor), thread_([this]() {executor_.spin();})
  {}

  ~ExecutorGuard()
  {
    executor_.cancel();
    thread_.join();
  }

private:
  rclcpp::Executor & executor_;
  std::thread thread_;
};

class UnitreeApiClientExecutorTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      int argc = 0;
      rclcpp::init(argc, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(UnitreeApiClientExecutorTest, ServiceWaitDoesNotBlockApiResponseCallback)
{
  const std::string request_topic = "/_erasers_g1_test/unitree_api_executor/request";
  const std::string response_topic = "/_erasers_g1_test/unitree_api_executor/response";
  auto server_node = std::make_shared<rclcpp::Node>("blocking_service_node");
  auto simulator_node = std::make_shared<rclcpp::Node>("unitree_api_simulator_node");
  auto client_node = std::make_shared<rclcpp::Node>("blocking_service_client_node");
  auto api_client = std::make_shared<UnitreeApiClient>(
    server_node.get(), request_topic, response_topic);

  auto response_publisher = simulator_node->create_publisher<unitree_api::msg::Response>(
    response_topic, rclcpp::QoS(1));
  auto request_subscription = simulator_node->create_subscription<unitree_api::msg::Request>(
    request_topic, rclcpp::QoS(1),
    [response_publisher](const unitree_api::msg::Request::SharedPtr request) {
      unitree_api::msg::Response response;
      response.header.identity = request->header.identity;
      response.header.status.code = 0;
      response_publisher->publish(response);
    });

  auto service_group = server_node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  auto service = server_node->create_service<std_srvs::srv::SetBool>(
    "/_erasers_g1_test/unitree_api_executor/blocking_service",
    [api_client](
      const std::shared_ptr<std_srvs::srv::SetBool::Request>,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
      const auto result = api_client->call(1002, "{}", 1.0);
      response->success = result.received && !result.timed_out && result.status_code == 0;
      response->message = response->success ? "response callback executed" : "API timeout";
    },
    rmw_qos_profile_services_default,
    service_group);

  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 4U);
  executor.add_node(server_node);
  executor.add_node(simulator_node);
  executor.add_node(client_node);
  ExecutorGuard guard(executor);

  auto client = client_node->create_client<std_srvs::srv::SetBool>(
    "/_erasers_g1_test/unitree_api_executor/blocking_service");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(1)));
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;
  auto future = client->async_send_request(request);
  ASSERT_EQ(future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  EXPECT_TRUE(future.get()->success);

  (void)request_subscription;
  (void)service;
}

TEST_F(UnitreeApiClientExecutorTest, PublishGuardCanCancelBeforeTransportPublish)
{
  const std::string request_topic = "/_erasers_g1_test/unitree_api_guard/request";
  const std::string response_topic = "/_erasers_g1_test/unitree_api_guard/response";
  auto client_node = std::make_shared<rclcpp::Node>("unitree_api_guard_client");
  auto observer_node = std::make_shared<rclcpp::Node>("unitree_api_guard_observer");
  UnitreeApiClient api_client(client_node.get(), request_topic, response_topic);
  std::atomic<int> observed_requests{0};
  auto request_subscription = observer_node->create_subscription<unitree_api::msg::Request>(
    request_topic, rclcpp::QoS(1),
    [&observed_requests](const unitree_api::msg::Request::SharedPtr) {
      ++observed_requests;
    });

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2U);
  executor.add_node(client_node);
  executor.add_node(observer_node);
  ExecutorGuard guard(executor);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  bool guard_called = false;
  const auto result = api_client.call(
    7106, "{}", {}, 10.0,
    [&guard_called](const UnitreeApiClient::PublishOperation &, std::string & reason) {
      guard_called = true;
      reason = "test cancellation";
      return false;
    });

  EXPECT_TRUE(guard_called);
  EXPECT_TRUE(result.publish_failed);
  EXPECT_TRUE(result.cancelled_before_publish);
  EXPECT_FALSE(result.received);
  EXPECT_FALSE(result.timed_out);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(observed_requests.load(), 0);

  (void)request_subscription;
}

}  // namespace
}  // namespace erasers_g1_common
