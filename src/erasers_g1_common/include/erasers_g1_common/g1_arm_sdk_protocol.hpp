#pragma once

#include <array>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <unitree_hg/msg/low_cmd.hpp>
#include "erasers_g1_common/g1_joint_descriptor.hpp"

namespace erasers_g1_common
{
struct G1ArmSdkJoint
{
  const char * name;
  std::size_t motor_index;
  float kp;
  float kd;
};
const std::array<G1ArmSdkJoint, kG1UpperBodyJointCount> & g1ArmSdkJoints();
bool makeG1ArmSdkLowCmd(
  const std::array<double, kG1UpperBodyJointCount> & positions, double weight,
  const std::array<G1JointControlLimit, kG1UpperBodyJointCount> & limits,
  unitree_hg::msg::LowCmd & output, std::string & error);

class LowCmdSinkInterface
{
public:
  virtual ~LowCmdSinkInterface() = default;
  virtual bool publish(const unitree_hg::msg::LowCmd &, std::string &) = 0;
  virtual void close() = 0;
};
class RosLowCmdSink : public LowCmdSinkInterface
{
public:
  explicit RosLowCmdSink(rclcpp::Node * node, std::string topic = "/arm_sdk");
  bool publish(const unitree_hg::msg::LowCmd &, std::string &) override;
  void close() override;
private:
  rclcpp::Node * node_;
  std::string topic_;
  std::mutex mutex_;
  rclcpp::Publisher<unitree_hg::msg::LowCmd>::SharedPtr publisher_;
};
}  // namespace erasers_g1_common
