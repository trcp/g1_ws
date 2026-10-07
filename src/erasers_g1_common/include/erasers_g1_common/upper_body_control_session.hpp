#ifndef ERASERS_G1_COMMON__UPPER_BODY_CONTROL_SESSION_HPP_
#define ERASERS_G1_COMMON__UPPER_BODY_CONTROL_SESSION_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include <unitree_hg/msg/low_cmd.hpp>

#include "erasers_g1_common/g1_arm_sdk_protocol.hpp"
#include "erasers_g1_common/g1_joint_descriptor.hpp"

namespace erasers_g1_common
{

struct UpperBodyPolicy
{
  std::chrono::milliseconds control_period{10};
  std::chrono::milliseconds lowstate_timeout{200};
  std::chrono::milliseconds fsm_timeout{3000};
  std::chrono::milliseconds arm_state_timeout{2000};
  std::chrono::milliseconds acquire_duration{1000};
  std::chrono::milliseconds release_duration{1000};
  std::chrono::milliseconds max_loop_interval{50};
  double max_past_stamp_sec{0.25};
  double max_future_stamp_sec{0.05};
  // 腕と腰の通常指令の最大回転速度。
  double arm_velocity_limit{2.0};
  double waist_head_velocity_limit{2.0};
  double arm_acceleration_limit{1.50};
  double waist_head_acceleration_limit{0.30};
  // Posture stabilization produces small persistent joint velocities.  This
  // gate rejects deliberate upper-body motion while allowing that normal sway.
  double max_acquire_measured_velocity{0.25};
  double tracking_error_limit{0.20};
  std::chrono::milliseconds tracking_error_duration{200};
  std::chrono::milliseconds arm_action_api_timeout{15000};
  std::chrono::milliseconds arm_action_state_timeout{20000};
  std::chrono::milliseconds command_timeout{500};
  std::vector<int32_t> allowed_fsm_ids{500, 501, 801};
};

struct UpperBodyStepResult
{
  bool publish{false};
  bool became_active{false};
  bool release_complete{false};
  bool fault{false};
  std::string fault_reason;
  uint64_t generation{0};
  unitree_hg::msg::LowCmd command;
};

class UpperBodyControlSession
{
public:
  using SteadyTime = std::chrono::steady_clock::time_point;

  explicit UpperBodyControlSession(UpperBodyPolicy policy = UpperBodyPolicy{});

  bool configure(
    const std::vector<G1JointControlLimit> & limits,
    std::string & error);
  void arm(uint64_t generation, SteadyTime now);
  bool beginAcquisition(
    const std::array<double, kG1UpperBodyJointCount> & measured_q,
    uint64_t generation,
    SteadyTime now,
    std::string & error);
  bool updateTarget(
    const std::vector<std::size_t> & indices,
    const std::vector<double> & positions,
    SteadyTime now,
    std::string & error);
  void requestRelease(uint64_t generation, SteadyTime now);
  void abort();
  void holdMeasured(
    const std::array<double, kG1UpperBodyJointCount> & measured_q, uint64_t generation);

  UpperBodyStepResult step(bool acquiring, bool active, bool releasing, SteadyTime now);

  const UpperBodyPolicy & policy() const noexcept;
  const std::array<double, kG1UpperBodyJointCount> & commandedPosition() const noexcept;
  double weight() const noexcept;
  uint64_t publishedCount() const noexcept;
  bool releaseZeroSent() const noexcept;
  bool hasValidCommand() const noexcept;
  SteadyTime lastValidCommandTime() const noexcept;

private:
  static double approach(double value, double target, double max_delta);
  bool buildCommand(double weight, UpperBodyStepResult & result);

  UpperBodyPolicy policy_;
  std::array<G1JointControlLimit, kG1UpperBodyJointCount> limits_{};
  std::array<double, kG1UpperBodyJointCount> q_command_{};
  std::array<double, kG1UpperBodyJointCount> q_target_{};
  std::array<double, kG1UpperBodyJointCount> v_command_{};
  bool configured_{false};
  bool initialized_{false};
  bool has_valid_command_{false};
  bool release_zero_sent_{false};
  uint64_t generation_{0};
  uint64_t published_count_{0};
  double weight_{0.0};
  double release_start_weight_{0.0};
  SteadyTime acquire_time_{};
  SteadyTime release_time_{};
  SteadyTime last_step_time_{};
  SteadyTime last_valid_command_time_{};
};

}  // namespace erasers_g1_common

#endif  // ERASERS_G1_COMMON__UPPER_BODY_CONTROL_SESSION_HPP_
