// Copyright (c) 2026 Nakatogawa Laboratory. All rights reserved.

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "erasers_g1_common/robot_service_interface_client.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<erasers_g1_common::RobotServiceInterfaceClientNode>();
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 4U);
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
