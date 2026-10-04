#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <unitree_hg/msg/low_state.hpp>
#include "erasers_g1_common/g1_joint_descriptor.hpp"

class JointStatePublisher : public rclcpp::Node
{
public:
  JointStatePublisher() : Node("joint_state_publisher")
  {
    const auto path = declare_parameter<std::string>("urdf", "");
    const auto description = declare_parameter<std::string>("robot_description", "");
    std::string error;
    if (!erasers_g1_common::loadG1JointLimits(
        description.empty() ? path : description, limits_, error))
    {
      throw std::invalid_argument(error);
    }
    publisher_ = create_publisher<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::QoS(10).best_effort());
    subscription_ = create_subscription<unitree_hg::msg::LowState>(
      "/lowstate", rclcpp::QoS(10).best_effort(),
      [this](unitree_hg::msg::LowState::ConstSharedPtr state) {
        sensor_msgs::msg::JointState msg;
        msg.header.stamp = now();
        for (const auto & joint : limits_) {
          const auto & motor = state->motor_state.at(joint.motor_index);
          msg.name.push_back(joint.name);
          msg.position.push_back(joint.active ? motor.q : 0.0);
          msg.velocity.push_back(joint.active ? motor.dq : 0.0);
          msg.effort.push_back(joint.active ? motor.tau_est : 0.0);
        }
        publisher_->publish(msg);
      });
  }
private:
  std::vector<erasers_g1_common::G1JointControlLimit> limits_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointStatePublisher>());
  rclcpp::shutdown();
  return 0;
}
