#include <moveit/motion_planning_rviz_plugin/motion_planning_display.h>
#include <moveit/constraint_samplers/constraint_sampler_manager.h>
#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/status_property.hpp>
#include <pluginlib/class_loader.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <Eigen/Geometry>
#include <map>
#include <memory>
#include <mutex>

namespace erasers_g1_moveit
{
namespace
{
using State = moveit::core::RobotState;
using Feedback = visualization_msgs::msg::InteractiveMarkerFeedback;
using Marker = visualization_msgs::msg::InteractiveMarker;
using Allocator = constraint_samplers::ConstraintSamplerAllocator;

geometry_msgs::msg::Pose toPose(const Eigen::Isometry3d & transform)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();
  const Eigen::Quaterniond rotation(transform.rotation());
  pose.orientation.x = rotation.x();
  pose.orientation.y = rotation.y();
  pose.orientation.z = rotation.z();
  pose.orientation.w = rotation.w();
  return pose;
}

Eigen::Isometry3d center(State state)
{
  state.update();
  const auto & left = state.getGlobalLinkTransform("left_amazing_hand");
  const auto & right = state.getGlobalLinkTransform("right_amazing_hand");
  Eigen::Isometry3d result = left;
  result.translation() = (left.translation() + right.translation()) * 0.5;
  return result;
}

moveit_msgs::msg::Constraints goals(
  const State & reference, const Eigen::Isometry3d & delta)
{
  moveit_msgs::msg::Constraints result;
  for (const auto * tip : {"left_amazing_hand", "right_amazing_hand"}) {
    const auto target = toPose(delta * reference.getGlobalLinkTransform(tip));
    moveit_msgs::msg::PositionConstraint position;
    position.header.frame_id = reference.getRobotModel()->getModelFrame();
    position.link_name = tip;
    position.weight = 1.0;
    shape_msgs::msg::SolidPrimitive sphere;
    sphere.type = shape_msgs::msg::SolidPrimitive::SPHERE;
    sphere.dimensions = {0.001};
    position.constraint_region.primitives.push_back(sphere);
    geometry_msgs::msg::Pose region;
    region.position = target.position;
    region.orientation.w = 1.0;
    position.constraint_region.primitive_poses.push_back(region);
    moveit_msgs::msg::OrientationConstraint orientation;
    orientation.header = position.header;
    orientation.link_name = tip;
    orientation.orientation = target.orientation;
    orientation.absolute_x_axis_tolerance = 0.1;
    orientation.absolute_y_axis_tolerance = 0.1;
    orientation.absolute_z_axis_tolerance = 0.1;
    orientation.weight = 1.0;
    result.position_constraints.push_back(position);
    result.orientation_constraints.push_back(orientation);
  }
  return result;
}

struct Drag
{
  std::shared_ptr<State> reference;
  Eigen::Isometry3d origin = Eigen::Isometry3d::Identity();
};
}  // namespace

// 標準の Plan/Execute と単腕操作を維持し、双腕目標の操作だけを共通化する。
class CoordinatedMotionPlanningDisplay : public moveit_rviz_plugin::MotionPlanningDisplay
{
public:
  ~CoordinatedMotionPlanningDisplay() override
  {
    clearJobs();
    // コールバックとプラグインローダーの寿命を、基底クラスの破棄前に揃える。
    robot_interaction_.reset();
  }

  void update(float wall_dt, float ros_dt) override
  {
    MotionPlanningDisplay::update(wall_dt, ros_dt);
    if (!robot_interaction_ || !planning_scene_monitor_) {
      return;
    }
    const auto group = getCurrentPlanningGroup();
    const bool dual = group == "arm_both" || group == "arm_both_with_waist";
    if (group == active_group_ &&
      (!dual || robot_interaction_->getActiveEndEffectors().empty()))
    {
      return;
    }
    active_group_ = group;
    robot_interaction_->clear();
    if (!dual) {
      robot_interaction_->decideActiveComponents(group);
      publishInteractiveMarkers(false);
      return;
    }
    installSharedMarker(group);
  }

private:
  void installSharedMarker(const std::string & group)
  {
    if (!loader_) {
      loader_ = std::make_shared<pluginlib::ClassLoader<Allocator>>(
        "moveit_core", "constraint_samplers::ConstraintSamplerAllocator");
      allocator_ = loader_->createSharedInstance("erasers_g1_moveit/DualArmConstraintSampler");
    }
    auto drags = std::make_shared<std::map<std::string, Drag>>();
    // 読み込み元ライブラリは GenericInteraction のコールバックより長く保持する。
    const auto loader = loader_;
    const auto allocator = allocator_;
    robot_interaction_->addActiveComponent(
      [](const State & state, Marker & marker) {
        marker.header.frame_id = state.getRobotModel()->getModelFrame();
        marker.pose = toPose(center(state));
        marker.scale = 0.25;
        marker.description = "双腕共通目標";
        visualization_msgs::msg::InteractiveMarkerControl visible;
        visible.always_visible = true;
        visible.interaction_mode = visible.MOVE_ROTATE_3D;
        visualization_msgs::msg::Marker sphere;
        sphere.type = sphere.SPHERE;
        sphere.pose.orientation.w = 1.0;
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.045;
        sphere.color.r = 0.1;
        sphere.color.g = 0.8;
        sphere.color.b = 0.9;
        sphere.color.a = 1.0;
        visible.markers.push_back(sphere);
        marker.controls.push_back(visible);
        for (int axis = 0; axis < 3; ++axis) {
          visualization_msgs::msg::InteractiveMarkerControl control;
          control.orientation.w = 0.7071067811865476;
          if (axis == 0) {
            control.orientation.x = control.orientation.w;
          } else if (axis == 1) {
            control.orientation.z = control.orientation.w;
          } else {
            control.orientation.y = control.orientation.w;
          }
          control.name = "rotate_" + std::to_string(axis);
          control.interaction_mode = control.ROTATE_AXIS;
          marker.controls.push_back(control);
          control.name = "move_" + std::to_string(axis);
          control.interaction_mode = control.MOVE_AXIS;
          marker.controls.push_back(control);
        }
        return true;
      },
      [this, group, drags, loader, allocator](
        State & state, const Feedback::ConstSharedPtr & feedback)
      {
        auto & drag = (*drags)[feedback->marker_name];
        if (feedback->event_type == Feedback::MOUSE_DOWN || !drag.reference) {
          drag.reference = std::make_shared<State>(state);
          drag.reference->update();
          drag.origin = center(*drag.reference);
        }
        if (feedback->event_type != Feedback::POSE_UPDATE &&
        feedback->event_type != Feedback::MOUSE_UP)
        {
          return true;
        }
        if (feedback->header.frame_id != state.getRobotModel()->getModelFrame()) {
          return false;
        }
        const auto & pose = feedback->pose;
        Eigen::Quaterniond rotation(
          pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
        if (!rotation.coeffs().allFinite() || rotation.norm() < 1e-9) {
          return false;
        }
        Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
        target.linear() = rotation.normalized().toRotationMatrix();
        target.translation() << pose.position.x, pose.position.y, pose.position.z;
        if (!target.matrix().allFinite()) {
          return false;
        }
        const auto constraints = goals(*drag.reference, target * drag.origin.inverse());
        // 二つの手先目標を同じ状態・同じ腰で解き、両方が成立したときだけ反映する。
        const auto scene = planning_scene::PlanningScene::clone(getPlanningSceneRO());
        scene->setCurrentState(state);
        auto sampler = allocator->alloc(scene, group, constraints);
        bool accepted = false;
        if (sampler) {
          State candidate(state);
          for (int attempt = 0; attempt < 4; ++attempt) {
            if (sampler->sample(candidate, state, 10) &&
            candidate.satisfiesBounds(candidate.getJointModelGroup(group)) &&
            scene->isStateValid(candidate, constraints, group))
            {
              state = candidate;
              accepted = true;
              break;
            }
          }
        }
        addMainLoopJob(
          [this, accepted] {
            setStatus(
              accepted ? rviz_common::properties::StatusProperty::Ok :
              rviz_common::properties::StatusProperty::Warn,
              "双腕共通目標", accepted ? "左右の目標を更新しました" :
              "左右の目標を同時に満たせないため、直前の目標を維持します");
          });
        if (feedback->event_type == Feedback::MOUSE_UP) {
          drag.reference.reset();
        }
        return accepted;
      },
      [](const State & state, geometry_msgs::msg::Pose & pose) {
        pose = toPose(center(state));
        return true;
      }, "dual_arm");
    publishInteractiveMarkers(false);
    RCLCPP_INFO(node_->get_logger(), "双腕共通マーカーを有効化: %s", group.c_str());
  }

  std::string active_group_;
  std::shared_ptr<pluginlib::ClassLoader<Allocator>> loader_;
  std::shared_ptr<Allocator> allocator_;
};
}  // namespace erasers_g1_moveit

PLUGINLIB_EXPORT_CLASS(erasers_g1_moveit::CoordinatedMotionPlanningDisplay, rviz_common::Display)
