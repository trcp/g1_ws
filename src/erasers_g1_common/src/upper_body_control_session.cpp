#include "erasers_g1_common/upper_body_control_session.hpp"

#include <algorithm>
#include <cmath>
#include <map>

namespace erasers_g1_common
{

UpperBodyControlSession::UpperBodyControlSession(UpperBodyPolicy policy)
: policy_(std::move(policy))
{
}

bool UpperBodyControlSession::configure(
  const std::vector<G1JointControlLimit> & limits,
  std::string & error)
{
  if (limits.size() != kG1UpperBodyJointCount) {
    error = "G1 上半身の 17 スロットが必要です";
    return false;
  }
  std::map<std::string, G1JointControlLimit> by_name;
  for (const auto & limit : limits) {
    by_name.emplace(limit.name, limit);
  }
  for (std::size_t i = 0; i < g1ArmSdkJoints().size(); ++i) {
    const auto found = by_name.find(g1ArmSdkJoints()[i].name);
    if (found == by_name.end() || found->second.motor_index != g1ArmSdkJoints()[i].motor_index) {
      error = "G1 ArmSdk descriptor/URDF mismatch for " +
        std::string(g1ArmSdkJoints()[i].name);
      return false;
    }
    limits_[i] = found->second;
  }
  configured_ = true;
  error.clear();
  return true;
}

void UpperBodyControlSession::arm(uint64_t generation, SteadyTime)
{
  generation_ = generation;
  initialized_ = false;
  has_valid_command_ = false;
  release_zero_sent_ = false;
  weight_ = 0.0;
  v_command_.fill(0.0);
}

bool UpperBodyControlSession::beginAcquisition(
  const std::array<double, kG1UpperBodyJointCount> & measured_q,
  uint64_t generation,
  SteadyTime now,
  std::string & error)
{
  if (!configured_) {
    error = "Upper-body limits are not configured";
    return false;
  }
  for (std::size_t i = 0; i < measured_q.size(); ++i) {
    if (!limits_[i].active) {continue;}
    if (!std::isfinite(measured_q[i]) || measured_q[i] < limits_[i].lower ||
      measured_q[i] > limits_[i].upper)
    {
      error = "Invalid acquisition position for " + limits_[i].name;
      return false;
    }
  }
  generation_ = generation;
  q_command_ = measured_q;
  q_target_ = measured_q;
  v_command_.fill(0.0);
  initialized_ = true;
  acquire_time_ = now;
  last_step_time_ = now;
  weight_ = 0.0;
  release_zero_sent_ = false;
  error.clear();
  return true;
}

bool UpperBodyControlSession::updateTarget(
  const std::vector<std::size_t> & indices,
  const std::vector<double> & positions,
  SteadyTime now,
  std::string & error)
{
  if (!initialized_ || indices.empty() || indices.size() != positions.size()) {
    error = "Invalid upper-body target update";
    return false;
  }
  auto candidate = q_target_;
  for (std::size_t i = 0; i < indices.size(); ++i) {
    const auto index = indices[i];
    if (index >= candidate.size() || !limits_[index].active || !std::isfinite(positions[i]) ||
      positions[i] < limits_[index].lower || positions[i] > limits_[index].upper)
    {
      error = "Upper-body target is outside its validated limit";
      return false;
    }
    candidate[index] = positions[i];
  }
  q_target_ = candidate;
  last_valid_command_time_ = now;
  has_valid_command_ = true;
  error.clear();
  return true;
}

void UpperBodyControlSession::requestRelease(uint64_t generation, SteadyTime now)
{
  generation_ = generation;
  release_time_ = now;
  release_start_weight_ = weight_;
  release_zero_sent_ = false;
  last_step_time_ = now;
}

void UpperBodyControlSession::abort()
{
  generation_++;
  initialized_ = false;
  has_valid_command_ = false;
  weight_ = 0.0;
  v_command_.fill(0.0);
}

double UpperBodyControlSession::approach(double value, double target, double max_delta)
{
  return value + std::clamp(target - value, -max_delta, max_delta);
}

bool UpperBodyControlSession::buildCommand(double weight, UpperBodyStepResult & result)
{
  std::string error;
  if (!makeG1ArmSdkLowCmd(q_command_, weight, limits_, result.command, error)) {
    result.fault = true;
    result.fault_reason = error;
    return false;
  }
  result.publish = true;
  result.generation = generation_;
  ++published_count_;
  return true;
}

UpperBodyStepResult UpperBodyControlSession::step(
  bool acquiring, bool active, bool releasing, SteadyTime now)
{
  UpperBodyStepResult result;
  result.generation = generation_;
  if (!initialized_ || (!acquiring && !active && !releasing)) {
    return result;
  }

  double dt = std::chrono::duration<double>(now - last_step_time_).count();
  if (dt <= 0.0) {
    dt = std::chrono::duration<double>(policy_.control_period).count();
  }
  if (now - last_step_time_ > policy_.max_loop_interval) {
    result.fault = true;
    result.fault_reason = "UPPER_BODY_CONTROL_LOOP_OVERRUN";
    return result;
  }
  last_step_time_ = now;

  if (acquiring) {
    const double elapsed = std::chrono::duration<double>(now - acquire_time_).count();
    const double duration = std::chrono::duration<double>(policy_.acquire_duration).count();
    weight_ = std::clamp(elapsed / duration, 0.0, 1.0);
    result.became_active = weight_ >= 1.0;
    buildCommand(weight_, result);
    return result;
  }

  for (std::size_t i = 0; i < q_command_.size(); ++i) {
    if (!limits_[i].active) {continue;}
    const bool arm_joint = limits_[i].motor_index >= 15;
    const double velocity_limit = std::min(
      limits_[i].velocity,
      arm_joint ? policy_.arm_velocity_limit : policy_.waist_head_velocity_limit);
    const double acceleration_limit = arm_joint ?
      policy_.arm_acceleration_limit : policy_.waist_head_acceleration_limit;

    double desired_velocity = 0.0;
    if (active) {
      const double distance = q_target_[i] - q_command_[i];
      const double braking_velocity = std::sqrt(
        std::max(0.0, 2.0 * acceleration_limit * std::abs(distance)));
      desired_velocity = std::copysign(
        std::min(velocity_limit, braking_velocity), distance);
      if (std::abs(distance) < 1e-9) {
        desired_velocity = 0.0;
      }
    }
    v_command_[i] = approach(v_command_[i], desired_velocity, acceleration_limit * dt);
    const double next = q_command_[i] + v_command_[i] * dt;
    q_command_[i] = std::clamp(next, limits_[i].lower, limits_[i].upper);
    if ((q_command_[i] == limits_[i].lower && v_command_[i] < 0.0) ||
      (q_command_[i] == limits_[i].upper && v_command_[i] > 0.0))
    {
      v_command_[i] = 0.0;
    }
  }

  if (releasing) {
    const double elapsed = std::chrono::duration<double>(now - release_time_).count();
    const double duration = std::chrono::duration<double>(policy_.release_duration).count();
    if (release_start_weight_ <= 0.0 || elapsed >= duration) {
      if (!release_zero_sent_) {
        weight_ = 0.0;
        release_zero_sent_ = true;
        result.release_complete = true;
        buildCommand(0.0, result);
      }
      return result;
    }
    // G1 の浮動小数点 weight を減衰し、最終サンプルだけを明示的なゼロにする。
    weight_ = std::max(
      1e-9, release_start_weight_ * std::max(0.0, 1.0 - elapsed / duration));
  } else {
    weight_ = 1.0;
  }

  buildCommand(weight_, result);
  return result;
}

const UpperBodyPolicy & UpperBodyControlSession::policy() const noexcept {return policy_;}
void UpperBodyControlSession::holdMeasured(
  const std::array<double, kG1UpperBodyJointCount> & measured_q, uint64_t generation)
{
  generation_ = generation;
  q_target_ = measured_q;
  q_command_ = measured_q;
  v_command_.fill(0.0);
}
const std::array<double, kG1UpperBodyJointCount> &
UpperBodyControlSession::commandedPosition() const noexcept {return q_command_;}
double UpperBodyControlSession::weight() const noexcept {return weight_;}
uint64_t UpperBodyControlSession::publishedCount() const noexcept {return published_count_;}
bool UpperBodyControlSession::releaseZeroSent() const noexcept {return release_zero_sent_;}
bool UpperBodyControlSession::hasValidCommand() const noexcept {return has_valid_command_;}
UpperBodyControlSession::SteadyTime
UpperBodyControlSession::lastValidCommandTime() const noexcept {return last_valid_command_time_;}

}  // namespace erasers_g1_common
