#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include "erasers_g1_common/service_relay.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("mic_server");
  erasers_g1_common::ServiceRelay<std_srvs::srv::SetBool> relay(
    node.get(), "/mic_rec", "/enable_mic", std::chrono::seconds(15));
  auto publisher = node->create_publisher<std_msgs::msg::Int16MultiArray>("/audio/raw", 10);
  auto subscription = node->create_subscription<std_msgs::msg::Int16MultiArray>(
    "/mic_data", 10, [publisher](std_msgs::msg::Int16MultiArray::ConstSharedPtr msg) {
      publisher->publish(*msg);
    });
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
