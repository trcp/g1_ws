#include <moveit/constraint_samplers/constraint_sampler_manager.h>
#include <moveit/constraint_samplers/union_constraint_sampler.h>
#include <pluginlib/class_list_macros.hpp>

namespace erasers_g1_moveit
{
class DualArmConstraintSampler : public constraint_samplers::UnionConstraintSampler
{
public:
  using UnionConstraintSampler::UnionConstraintSampler;

  bool sample(
    moveit::core::RobotState & state,
    const moveit::core::RobotState & reference_state,
    unsigned int max_attempts) override
  {
    // 共通の腰は最初に参照姿勢を使い、以降の再試行では探索する。
    const moveit::core::RobotState seed(reference_state);
    state = seed;
    if (!first_sample_) {
      state.setToRandomPositions(jmg_);
    }
    first_sample_ = false;
    for (const auto & sampler : getSamplers()) {
      // 出力と参照の別名参照を避け、更新済み FK を各 IK へ渡す。
      state.updateLinkTransforms();
      const moveit::core::RobotState clean_reference(state);
      if (!sampler->sample(state, clean_reference, max_attempts)) {
        return false;
      }
    }
    state.update();
    return true;
  }

private:
  bool first_sample_{true};
};

class DualArmConstraintSamplerAllocator : public constraint_samplers::ConstraintSamplerAllocator
{
public:
  bool canService(
    const planning_scene::PlanningSceneConstPtr &,
    const std::string & group,
    const moveit_msgs::msg::Constraints & constraints) const override
  {
    return (group == "arm_both" || group == "arm_both_with_waist") &&
           (!constraints.position_constraints.empty() ||
           !constraints.orientation_constraints.empty());
  }

  constraint_samplers::ConstraintSamplerPtr alloc(
    const planning_scene::PlanningSceneConstPtr & scene, const std::string & group,
    const moveit_msgs::msg::Constraints & constraints) override
  {
    auto standard = constraint_samplers::ConstraintSamplerManager::selectDefaultSampler(
      scene, group, constraints);
    auto combined =
      std::dynamic_pointer_cast<constraint_samplers::UnionConstraintSampler>(standard);
    if (!combined) {
      return standard;
    }
    RCLCPP_INFO(
      rclcpp::get_logger(
        "dual_arm_constraint_sampler"), "更新済み FK の双腕サンプラ: %s", group.c_str());
    return std::make_shared<DualArmConstraintSampler>(scene, group, combined->getSamplers());
  }
};
}  // namespace erasers_g1_moveit

PLUGINLIB_EXPORT_CLASS(
  erasers_g1_moveit::DualArmConstraintSamplerAllocator,
  constraint_samplers::ConstraintSamplerAllocator)
