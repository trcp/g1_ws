#include "erasers_g1_common/g1_joint_descriptor.hpp"

#include <cmath>
#include <urdf/model.h>

namespace erasers_g1_common
{
const std::array<G1JointDescriptor, kG1JointCount> & g1JointDescriptors()
{
  static const std::array<G1JointDescriptor, kG1JointCount> joints{{
    {"left_hip_pitch_joint", 0}, {"left_hip_roll_joint", 1},
    {"left_hip_yaw_joint", 2}, {"left_knee_joint", 3},
    {"left_ankle_pitch_joint", 4}, {"left_ankle_roll_joint", 5},
    {"right_hip_pitch_joint", 6}, {"right_hip_roll_joint", 7},
    {"right_hip_yaw_joint", 8}, {"right_knee_joint", 9},
    {"right_ankle_pitch_joint", 10}, {"right_ankle_roll_joint", 11},
    {"waist_yaw_joint", 12}, {"waist_roll_joint", 13}, {"waist_pitch_joint", 14},
    {"left_shoulder_pitch_joint", 15}, {"left_shoulder_roll_joint", 16},
    {"left_shoulder_yaw_joint", 17}, {"left_elbow_joint", 18},
    {"left_wrist_roll_joint", 19}, {"left_wrist_pitch_joint", 20},
    {"left_wrist_yaw_joint", 21}, {"right_shoulder_pitch_joint", 22},
    {"right_shoulder_roll_joint", 23}, {"right_shoulder_yaw_joint", 24},
    {"right_elbow_joint", 25}, {"right_wrist_roll_joint", 26},
    {"right_wrist_pitch_joint", 27}, {"right_wrist_yaw_joint", 28}
  }};
  return joints;
}

bool loadG1JointLimits(
  const std::string & input, std::vector<G1JointControlLimit> & limits, std::string & error)
{
  urdf::Model model;
  const bool xml = input.find('<') != std::string::npos;
  if (input.empty() || !(xml ? model.initString(input) : model.initFile(input))) {
    error = "G1 URDF の読み込みに失敗しました";
    return false;
  }
  std::vector<G1JointControlLimit> candidate;
  std::size_t active = 0;
  for (const auto & desc : g1JointDescriptors()) {
    const auto joint = model.getJoint(desc.name);
    if (!joint || joint->type == urdf::Joint::FIXED) {
      candidate.push_back({desc.name, desc.motor_index, 0.0, 0.0, 0.0, false});
      continue;
    }
    if (joint->type != urdf::Joint::REVOLUTE || joint->mimic || !joint->limits ||
      !std::isfinite(joint->limits->lower) || !std::isfinite(joint->limits->upper) ||
      joint->limits->lower >= joint->limits->upper ||
      !std::isfinite(joint->limits->velocity) || joint->limits->velocity <= 0.0)
    {
      error = std::string("G1 関節制限が不正です: ") + desc.name;
      return false;
    }
    candidate.push_back({desc.name, desc.motor_index, joint->limits->lower,
      joint->limits->upper, joint->limits->velocity, true});
    ++active;
  }
  // G1 本体の構成を検証し、カメラ等の独立アクチュエータを数えない。
  if (active != 23 && active != 29) {
    error = "G1 本体の可動関節数が 23 / 29 ではありません: " + std::to_string(active);
    return false;
  }
  for (const auto & limit : candidate) {
    const auto i = limit.motor_index;
    const bool optional = i == 13 || i == 14 || i == 20 || i == 21 || i == 27 || i == 28;
    if (!optional && !limit.active) {
      error = "G1 必須関節がありません: " + limit.name;
      return false;
    }
  }
  limits = std::move(candidate);
  error.clear();
  return true;
}

bool loadAndValidateG1UpperBodyLimits(
  const std::string & input, std::vector<G1JointControlLimit> & limits, std::string & error)
{
  std::vector<G1JointControlLimit> all;
  if (!loadG1JointLimits(input, all, error)) {return false;}
  limits.assign(all.begin() + 12, all.end());
  return true;
}
}  // namespace erasers_g1_common
