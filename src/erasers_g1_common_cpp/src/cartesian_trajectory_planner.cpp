#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <chrono>
#include <thread>
#include <mutex>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "g1_srvs/action/cartesian_ee.hpp"
#include <Eigen/Dense>

using namespace std::chrono_literals;

namespace erasers_g1
{

// Utility function for Minimum Jerk Trajectory
double computeMinJerk(double t, double T) {
    if (t <= 0.0) return 0.0;
    if (t >= T) return 1.0;
    double r = t / T;
    return r * r * r * (10.0 - 15.0 * r + 6.0 * r * r);
}

class CartesianTrajectoryPlanner : public rclcpp::Node
{
public:
  using CartesianEE = g1_srvs::action::CartesianEE;
  using GoalHandleCartesianEE = rclcpp_action::ServerGoalHandle<CartesianEE>;

  CartesianTrajectoryPlanner(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("cartesian_trajectory_planner", options)
  {
      // TF2 Listener
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

      // Publishers
      left_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/left_arm/target_pose", 10);
      right_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/right_arm/target_pose", 10);

      // Action Servers
      left_action_server_ = rclcpp_action::create_server<CartesianEE>(
        this,
        "/left_arm/cartesian_planner",
        std::bind(&CartesianTrajectoryPlanner::handle_goal<false>, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&CartesianTrajectoryPlanner::handle_cancel, this, std::placeholders::_1),
        std::bind(&CartesianTrajectoryPlanner::handle_accepted<false>, this, std::placeholders::_1));

      right_action_server_ = rclcpp_action::create_server<CartesianEE>(
        this,
        "/right_arm/cartesian_planner",
        std::bind(&CartesianTrajectoryPlanner::handle_goal<true>, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&CartesianTrajectoryPlanner::handle_cancel, this, std::placeholders::_1),
        std::bind(&CartesianTrajectoryPlanner::handle_accepted<true>, this, std::placeholders::_1));

      RCLCPP_INFO(this->get_logger(), "CartesianTrajectoryPlanner node started.");
  }

private:
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr left_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr right_pose_pub_;

  rclcpp_action::Server<CartesianEE>::SharedPtr left_action_server_;
  rclcpp_action::Server<CartesianEE>::SharedPtr right_action_server_;

  template<bool IsRight>
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const CartesianEE::Goal> goal)
  {
      (void)uuid;
      const char* side = IsRight ? "right" : "left";
      RCLCPP_INFO(this->get_logger(), "Received goal request for %s arm. Duration: %.2f", side, goal->duration);
      if (goal->duration <= 0.0) {
          RCLCPP_WARN(this->get_logger(), "Duration must be positive. Rejecting.");
          return rclcpp_action::GoalResponse::REJECT;
      }
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleCartesianEE> goal_handle)
  {
      RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
      (void)goal_handle;
      return rclcpp_action::CancelResponse::ACCEPT;
  }

  template<bool IsRight>
  void handle_accepted(const std::shared_ptr<GoalHandleCartesianEE> goal_handle)
  {
      // spawn execution thread
      std::string side_str = IsRight ? "right" : "left";
      std::thread{std::bind(&CartesianTrajectoryPlanner::execute_trajectory, this, std::placeholders::_1, side_str), goal_handle}.detach();
  }

  void execute_trajectory(const std::shared_ptr<GoalHandleCartesianEE> goal_handle, const std::string& side)
  {
      RCLCPP_INFO(this->get_logger(), "Executing trajectory for %s arm", side.c_str());
      const auto goal = goal_handle->get_goal();
      auto result = std::make_shared<CartesianEE::Result>();
      auto feedback = std::make_shared<CartesianEE::Feedback>();

      // Get current pose from TF
      std::string ee_frame = (side == "left") ? "left_wrist_roll_rubber_hand" : "right_wrist_roll_rubber_hand";
      geometry_msgs::msg::TransformStamped t;
      try {
          t = tf_buffer_->lookupTransform("base_link", ee_frame, tf2::TimePointZero, tf2::durationFromSec(1.0));
      } catch (const tf2::TransformException & ex) {
          RCLCPP_ERROR(this->get_logger(), "Could not transform base_link to %s: %s", ee_frame.c_str(), ex.what());
          result->success = false;
          result->message = "TF lookup failed.";
          goal_handle->abort(result);
          return;
      }

      // Start Pose
      Eigen::Vector3d p_start(t.transform.translation.x, t.transform.translation.y, t.transform.translation.z);
      Eigen::Quaterniond q_start(t.transform.rotation.w, t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z);

      // Goal Pose
      Eigen::Vector3d p_goal(goal->pose.pose.position.x, goal->pose.pose.position.y, goal->pose.pose.position.z);
      Eigen::Quaterniond q_goal(goal->pose.pose.orientation.w, goal->pose.pose.orientation.x, goal->pose.pose.orientation.y, goal->pose.pose.orientation.z);

      double T = goal->duration;
      double dt = 0.01; // 100 Hz
      rclcpp::Rate rate(1.0 / dt);
      
      auto start_time = this->now();
      
      while (rclcpp::ok()) {
          if (goal_handle->is_canceling()) {
              result->success = false;
              result->message = "Goal canceled.";
              goal_handle->canceled(result);
              RCLCPP_INFO(this->get_logger(), "Goal canceled.");
              return;
          }
          
          double t_elapsed = (this->now() - start_time).seconds();
          bool is_finished = false;
          if (t_elapsed >= T) {
              t_elapsed = T;
              is_finished = true;
          }

          // Interpolation logic
          double s = computeMinJerk(t_elapsed, T);
          
          Eigen::Vector3d p_curr = p_start + s * (p_goal - p_start);
          Eigen::Quaterniond q_curr = q_start.slerp(s, q_goal);

          // Publish current way-point
          geometry_msgs::msg::PoseStamped msg;
          msg.header.stamp = this->now();
          msg.header.frame_id = "base_link";
          msg.pose.position.x = p_curr.x();
          msg.pose.position.y = p_curr.y();
          msg.pose.position.z = p_curr.z();
          msg.pose.orientation.w = q_curr.w();
          msg.pose.orientation.x = q_curr.x();
          msg.pose.orientation.y = q_curr.y();
          msg.pose.orientation.z = q_curr.z();

          if (side == "left") {
              left_pose_pub_->publish(msg);
          } else {
              right_pose_pub_->publish(msg);
          }

          feedback->current_pose = msg;
          feedback->time_elapsed = t_elapsed;
          goal_handle->publish_feedback(feedback);

          if (is_finished) {
              break;
          }

          rate.sleep();
      }

      if (rclcpp::ok()) {
          result->success = true;
          result->message = "Trajectory execution completed successfully.";
          goal_handle->succeed(result);
          RCLCPP_INFO(this->get_logger(), "Goal succeeded.");
      }
  }

};

} // namespace erasers_g1

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<erasers_g1::CartesianTrajectoryPlanner>();
  
  // Multi-threaded executor is often required for Action Servers to process callbacks properly
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}
