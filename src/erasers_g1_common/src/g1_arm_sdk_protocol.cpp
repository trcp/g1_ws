#include "erasers_g1_common/g1_arm_sdk_protocol.hpp"
#include <cmath>
#include <exception>

namespace erasers_g1_common
{
const std::array<G1ArmSdkJoint, kG1UpperBodyJointCount> & g1ArmSdkJoints()
{
  static const auto joints = [] {
    std::array<G1ArmSdkJoint, kG1UpperBodyJointCount> result{};
    for (std::size_t i = 0; i < result.size(); ++i) {
      const auto & joint = g1JointDescriptors()[i + 12];
      result[i] = {joint.name, joint.motor_index, 60.0F, 1.5F};
    }
    return result;
  }();
  return joints;
}

bool makeG1ArmSdkLowCmd(
  const std::array<double, kG1UpperBodyJointCount> & positions, double weight,
  const std::array<G1JointControlLimit, kG1UpperBodyJointCount> & limits,
  unitree_hg::msg::LowCmd & output, std::string & error)
{
  if (!std::isfinite(weight) || weight < 0.0 || weight > 1.0) {
    error = "制御権の重みは有限の 0～1 で指定してください";
    return false;
  }
  unitree_hg::msg::LowCmd command{};
  for (std::size_t i = 0; i < positions.size(); ++i) {
    const auto & spec = g1ArmSdkJoints()[i];
    if (limits[i].name != spec.name || limits[i].motor_index != spec.motor_index) {
      error = "G1 関節表の不一致";
      return false;
    }
    if (!limits[i].active) {continue;}
    if (!std::isfinite(positions[i]) || !std::isfinite(static_cast<float>(positions[i])) ||
      positions[i] < limits[i].lower || positions[i] > limits[i].upper)
    {
      error = "G1 関節指令が制限外です: " + limits[i].name;
      return false;
    }
    auto & motor = command.motor_cmd.at(spec.motor_index);
    motor.q = static_cast<float>(positions[i]);
    motor.kp = spec.kp;
    motor.kd = spec.kd;
  }
  // G1 公式プロトコル。29 番は関節ではなく制御権専用フィールド。
  command.motor_cmd.at(29).q = static_cast<float>(weight);
  output = command;
  error.clear();
  return true;
}

RosLowCmdSink::RosLowCmdSink(rclcpp::Node * node, std::string topic)
: node_(node), topic_(std::move(topic)) {}

bool RosLowCmdSink::publish(const unitree_hg::msg::LowCmd & command, std::string & error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  try {
    if (!publisher_) {
      publisher_ = node_->create_publisher<unitree_hg::msg::LowCmd>(
        topic_, rclcpp::QoS(1).reliable().durability_volatile());
    }
    publisher_->publish(command);
  } catch (const std::exception & e) {error = e.what(); return false;}
  error.clear();
  return true;
}
void RosLowCmdSink::close()
{
  std::lock_guard<std::mutex> lock(mutex_);
  publisher_.reset();
}
}  // namespace erasers_g1_common
