#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "erasers_g1_common/robot_controller.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<erasers_g1_common::RobotControllerNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
