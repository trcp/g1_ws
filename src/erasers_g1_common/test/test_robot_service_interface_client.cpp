// Copyright (c) 2026 Nakatogawa Laboratory. All rights reserved.

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <erasers_g1_interfaces/srv/robot_service_client.hpp>

#include "erasers_g1_common/robot_service_interface_client.hpp"

namespace erasers_g1_common
{
namespace
{

class FakeRobotStateClient : public RobotStateClientInterface
{
public:
  FakeRobotStateClient()
  {
    states_["video_hub"] = RobotServiceState{"video_hub", 0, 0};
    states_["stereo_patch_pc1"] = RobotServiceState{"stereo_patch_pc1", 1, 0};
    states_["protected_service"] = RobotServiceState{"protected_service", 0, 1};
    states_["robot_state"] = RobotServiceState{"robot_state", 0, 1};
  }

  RobotServiceListResult list_services(double) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++list_calls_;
    for (const auto & pending : pending_states_) {
      states_[pending.first].status = pending.second;
    }
    pending_states_.clear();

    RobotServiceListResult result;
    result.success = true;
    result.received = true;
    result.api_status_code = 0;
    result.message = "OK";
    for (const auto & entry : states_) {
      result.services.push_back(entry.second);
    }
    return result;
  }

  RobotServiceSwitchResult switch_service(
    const std::string & service_name,
    bool enable,
    double) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++switch_calls_;
    switch_events_.emplace_back(service_name, enable);

    RobotServiceSwitchResult result;
    const auto found = states_.find(service_name);
    if (found == states_.end()) {
      result.message = "not found";
      return result;
    }

    pending_states_[service_name] = enable ? 0 : 1;

    result.success = true;
    result.received = true;
    result.api_status_code = 0;
    result.service_name = service_name;
    result.service_status = found->second.status;
    result.message = "accepted with transitional status";
    return result;
  }

  void add_service(const std::string & name, int32_t status = 0, int32_t protect = 0)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    states_[name] = RobotServiceState{name, status, protect};
  }

  int list_calls() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return list_calls_;
  }

  int switch_calls() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return switch_calls_;
  }

  std::vector<std::pair<std::string, bool>> switch_events() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return switch_events_;
  }

private:
  mutable std::mutex mutex_;
  int list_calls_{0};
  int switch_calls_{0};
  std::map<std::string, RobotServiceState> states_;
  std::map<std::string, int32_t> pending_states_;
  std::vector<std::pair<std::string, bool>> switch_events_;
};

class ExecutorSpinGuard
{
public:
  explicit ExecutorSpinGuard(rclcpp::Executor & executor)
  : executor_(executor), spin_thread_([this]() {executor_.spin();})
  {
  }

  ~ExecutorSpinGuard()
  {
    executor_.cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
  }

  ExecutorSpinGuard(const ExecutorSpinGuard &) = delete;
  ExecutorSpinGuard & operator=(const ExecutorSpinGuard &) = delete;

private:
  rclcpp::Executor & executor_;
  std::thread spin_thread_;
};

class RobotServiceInterfaceClientTest : public ::testing::Test
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

rclcpp::NodeOptions fast_test_options(const std::string & node_name)
{
  rclcpp::NodeOptions options;
  options.arguments({
    "--ros-args", "-r", "__node:=" + node_name,
    "-p", "api_timeout_sec:=1.0",
    "-p", "refresh_period_sec:=0.0",
    "-p", "verify_after_switch:=true",
    "-p", "switch_settle_delay_sec:=0.01",
    "-p", "switch_timeout_sec:=1.0",
    "-p", "verify_poll_period_sec:=0.01"});
  return options;
}

std::vector<std::string> split_newlines(const std::string & text)
{
  std::vector<std::string> lines;
  std::istringstream stream(text);
  std::string line;
  while (std::getline(stream, line)) {
    if (!line.empty()) {
      lines.push_back(line);
    }
  }
  return lines;
}

// 1. 初期化時ディスカバリの検証
TEST_F(RobotServiceInterfaceClientTest, InitialDiscoveryHappensDuringNodeCreation)
{
  auto fake = std::make_shared<FakeRobotStateClient>();
  EXPECT_EQ(fake->list_calls(), 0);

  auto server = std::make_shared<RobotServiceInterfaceClientNode>(
    fake, fast_test_options("initial_discovery_node"));

  // ノードコンストラクタ内で同期的に list_services() が呼ばれていること
  EXPECT_GE(fake->list_calls(), 1);

  const auto cached = server->get_cached_service_names();
  EXPECT_EQ(cached.size(), 4U);
}

// 2. 一覧取得モードのオンデマンド最新化検証 (name = "NONE")
TEST_F(RobotServiceInterfaceClientTest, ListServicesOnDemandRefreshesAndReturnsNewlineSeparatedList)
{
  auto fake = std::make_shared<FakeRobotStateClient>();
  auto server = std::make_shared<RobotServiceInterfaceClientNode>(
    fake, fast_test_options("list_services_node"));
  auto client_node = std::make_shared<rclcpp::Node>("test_list_client");

  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 4U);
  executor.add_node(server);
  executor.add_node(client_node);
  ExecutorSpinGuard spin_guard(executor);

  // 初期化時の呼び出し回数を記録
  const int initial_list_calls = fake->list_calls();

  // ロボット側に動的に新しいサービスを追加
  fake->add_service("dynamic_teleop_service", 0, 0);

  auto client = client_node->create_client<erasers_g1_interfaces::srv::RobotServiceClient>(
    "robot_service_interface");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2)));

  auto request = std::make_shared<erasers_g1_interfaces::srv::RobotServiceClient::Request>();
  request->name = "NONE";
  request->enable = false;

  auto future = client->async_send_request(request);
  ASSERT_EQ(future.wait_for(std::chrono::seconds(2)), std::future_status::ready);

  auto response = future.get();
  EXPECT_TRUE(response->success);

  // 要求受信時にオンデマンドで list_services が呼ばれたことを確認
  EXPECT_GT(fake->list_calls(), initial_list_calls);

  // 返却メッセージが改行区切りであることを確認
  const auto lines = split_newlines(response->message);
  EXPECT_EQ(lines.size(), 5U);
  EXPECT_NE(std::find(lines.begin(), lines.end(), "dynamic_teleop_service"), lines.end());
}

// 3. 空文字リクエストでも一覧取得が動作することの検証
TEST_F(RobotServiceInterfaceClientTest, EmptyNameAlsoTriggersListServices)
{
  auto fake = std::make_shared<FakeRobotStateClient>();
  auto server = std::make_shared<RobotServiceInterfaceClientNode>(
    fake, fast_test_options("empty_name_node"));
  auto client_node = std::make_shared<rclcpp::Node>("test_empty_name_client");

  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 4U);
  executor.add_node(server);
  executor.add_node(client_node);
  ExecutorSpinGuard spin_guard(executor);

  auto client = client_node->create_client<erasers_g1_interfaces::srv::RobotServiceClient>(
    "robot_service_interface");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2)));

  auto request = std::make_shared<erasers_g1_interfaces::srv::RobotServiceClient::Request>();
  request->name = "";  // 空文字
  request->enable = true;

  auto future = client->async_send_request(request);
  ASSERT_EQ(future.wait_for(std::chrono::seconds(2)), std::future_status::ready);

  auto response = future.get();
  EXPECT_TRUE(response->success);
  const auto lines = split_newlines(response->message);
  EXPECT_EQ(lines.size(), 4U);
}

// 4. 通常操作モード: 保護チェックなしの直接切り替え (protect != 0 や robot_state も許可)
TEST_F(RobotServiceInterfaceClientTest, DirectServiceSwitchSucceedsWithoutProtectionCheck)
{
  auto fake = std::make_shared<FakeRobotStateClient>();
  auto server = std::make_shared<RobotServiceInterfaceClientNode>(
    fake, fast_test_options("direct_switch_node"));
  auto client_node = std::make_shared<rclcpp::Node>("test_direct_switch_client");

  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 4U);
  executor.add_node(server);
  executor.add_node(client_node);
  ExecutorSpinGuard spin_guard(executor);

  auto client = client_node->create_client<erasers_g1_interfaces::srv::RobotServiceClient>(
    "robot_service_interface");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2)));

  // protect != 0 のサービスに対してもガードされずに直接 switch_service が呼ばれること
  auto request = std::make_shared<erasers_g1_interfaces::srv::RobotServiceClient::Request>();
  request->name = "protected_service";
  request->enable = false;

  auto future = client->async_send_request(request);
  ASSERT_EQ(future.wait_for(std::chrono::seconds(2)), std::future_status::ready);

  auto response = future.get();
  EXPECT_TRUE(response->success);
  EXPECT_EQ(fake->switch_calls(), 1);

  const auto events = fake->switch_events();
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].first, "protected_service");
  EXPECT_FALSE(events[0].second);
}

// 5. 存在しないサービス名へのハンドリング
TEST_F(RobotServiceInterfaceClientTest, UnknownServiceReturnsFailure)
{
  auto fake = std::make_shared<FakeRobotStateClient>();
  auto server = std::make_shared<RobotServiceInterfaceClientNode>(
    fake, fast_test_options("unknown_service_node"));
  auto client_node = std::make_shared<rclcpp::Node>("test_unknown_service_client");

  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 4U);
  executor.add_node(server);
  executor.add_node(client_node);
  ExecutorSpinGuard spin_guard(executor);

  auto client = client_node->create_client<erasers_g1_interfaces::srv::RobotServiceClient>(
    "robot_service_interface");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2)));

  auto request = std::make_shared<erasers_g1_interfaces::srv::RobotServiceClient::Request>();
  request->name = "completely_unknown_service";
  request->enable = true;

  auto future = client->async_send_request(request);
  ASSERT_EQ(future.wait_for(std::chrono::seconds(2)), std::future_status::ready);

  auto response = future.get();
  EXPECT_FALSE(response->success);
  EXPECT_NE(response->message.find("not found"), std::string::npos);
  EXPECT_EQ(fake->switch_calls(), 0);
}

}  // namespace
}  // namespace erasers_g1_common
