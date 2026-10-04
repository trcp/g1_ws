#include <rclcpp/rclcpp.hpp>
#include "erasers_g1_common/robot_manager.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<erasers_g1_common::RobotManagerNode>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("robot_manager"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
