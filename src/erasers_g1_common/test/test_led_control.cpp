// Copyright (c) 2026 Nakatogawa Laboratory. All rights reserved.

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include "erasers_g1_common/led_client_interface.hpp"
#include "erasers_g1_common/robot_controller.hpp"

using namespace erasers_g1_common;

class MockLedClient : public LedClientInterface
{
public:
  bool set_led_color(
    uint8_t r,
    uint8_t g,
    uint8_t b,
    double /*timeout_sec*/ = 2.0) override
  {
    last_r = r;
    last_g = g;
    last_b = b;
    call_count++;
    return return_value;
  }

  uint8_t last_r{0};
  uint8_t last_g{0};
  uint8_t last_b{0};
  int call_count{0};
  bool return_value{true};
};

class RobotControllerLedTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};

TEST_F(RobotControllerLedTest, ValidRgbCommandCallsLedClient)
{
  auto mock_client = std::make_shared<MockLedClient>();
  auto node = std::make_shared<RobotControllerNode>(
    rclcpp::NodeOptions(),
    nullptr, nullptr, nullptr, nullptr,
    UpperBodyPolicy{},
    std::vector<G1JointControlLimit>{},
    mock_client);

  auto msg = std::make_shared<std_msgs::msg::Int32MultiArray>();
  msg->data = {255, 128, 64};

  node->handle_led_command(msg);

  EXPECT_EQ(mock_client->call_count, 1);
  EXPECT_EQ(mock_client->last_r, 255);
  EXPECT_EQ(mock_client->last_g, 128);
  EXPECT_EQ(mock_client->last_b, 64);
}

TEST_F(RobotControllerLedTest, RejectNegativeRgbValues)
{
  auto mock_client = std::make_shared<MockLedClient>();
  auto node = std::make_shared<RobotControllerNode>(
    rclcpp::NodeOptions(),
    nullptr, nullptr, nullptr, nullptr,
    UpperBodyPolicy{},
    std::vector<G1JointControlLimit>{},
    mock_client);

  auto msg = std::make_shared<std_msgs::msg::Int32MultiArray>();
  msg->data = {-1, 100, 100};

  node->handle_led_command(msg);

  EXPECT_EQ(mock_client->call_count, 0);
}

TEST_F(RobotControllerLedTest, RejectOverflowRgbValues)
{
  auto mock_client = std::make_shared<MockLedClient>();
  auto node = std::make_shared<RobotControllerNode>(
    rclcpp::NodeOptions(),
    nullptr, nullptr, nullptr, nullptr,
    UpperBodyPolicy{},
    std::vector<G1JointControlLimit>{},
    mock_client);

  auto msg = std::make_shared<std_msgs::msg::Int32MultiArray>();
  msg->data = {100, 256, 100};

  node->handle_led_command(msg);

  EXPECT_EQ(mock_client->call_count, 0);
}

TEST_F(RobotControllerLedTest, RejectInsufficientElements)
{
  auto mock_client = std::make_shared<MockLedClient>();
  auto node = std::make_shared<RobotControllerNode>(
    rclcpp::NodeOptions(),
    nullptr, nullptr, nullptr, nullptr,
    UpperBodyPolicy{},
    std::vector<G1JointControlLimit>{},
    mock_client);

  auto msg = std::make_shared<std_msgs::msg::Int32MultiArray>();
  msg->data = {100, 100};  // only 2 elements

  node->handle_led_command(msg);

  EXPECT_EQ(mock_client->call_count, 0);
}
