#include <rclcpp/rclcpp.hpp>
#include "erasers_g1_common/service_relay.hpp"
#include "erasers_g1_interfaces/srv/pose_policy.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("loco_service_client");
  erasers_g1_common::ServiceRelay<erasers_g1_interfaces::srv::PosePolicy> relay(
    node.get(), "/pose_policy", "/robot_controller/pose_policy", std::chrono::seconds(25));
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
