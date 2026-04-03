#include <memory>
#include <thread>
#include <algorithm>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFJT = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

class MoveItActionServer : public rclcpp::Node
{
public:
  MoveItActionServer() : Node("moveit_action_server")
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/upper_joints_control", 10);
    
    action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
      this,
      "/upper_body_controller/follow_joint_trajectory",
      std::bind(&MoveItActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MoveItActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&MoveItActionServer::handle_accepted, this, std::placeholders::_1));
      
    // Timer to continuously publish the latest target at 100Hz
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10), // 100 Hz
      std::bind(&MoveItActionServer::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "MoveItActionServer started. Posture maintenance active.");
  }

private:
  void timer_callback()
  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    if (!latest_target_.name.empty()) {
      latest_target_.header.stamp = this->now();
      publisher_->publish(latest_target_);
    }
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FollowJointTrajectory::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with %zu points", goal->trajectory.points.size());
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFJT> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleFJT> goal_handle)
  {
    std::thread{std::bind(&MoveItActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleFJT> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<FollowJointTrajectory::Result>();
    
    if (goal->trajectory.points.empty()) {
      result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
      goal_handle->abort(result);
      return;
    }

    auto start_time = this->now();
    double current_time_offset = 0.0;
    
    rclcpp::Rate loop_rate(100.0); // 100 Hz publication updates
    
    size_t point_index = 0;
    
    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      
      auto now = this->now();
      current_time_offset = (now - start_time).seconds();
      
      // Find the appropriate segment
      while (point_index < goal->trajectory.points.size() && 
             rclcpp::Duration(goal->trajectory.points[point_index].time_from_start).seconds() < current_time_offset) {
        point_index++;
      }
      
      sensor_msgs::msg::JointState js_msg;
      js_msg.name = goal->trajectory.joint_names;
      
      if (point_index >= goal->trajectory.points.size()) {
        // Reached the end
        auto last_point = goal->trajectory.points.back();
        js_msg.position = last_point.positions;
        js_msg.velocity = last_point.velocities;
        
        {
          std::lock_guard<std::mutex> lock(target_mutex_);
          latest_target_ = js_msg;
        }
        break;
      } else if (point_index == 0) {
        // Before the first point
        js_msg.position = goal->trajectory.points[0].positions;
        js_msg.velocity = goal->trajectory.points[0].velocities;
      } else {
        // Interpolate between point_index-1 and point_index
        auto pt0 = goal->trajectory.points[point_index-1];
        auto pt1 = goal->trajectory.points[point_index];
        double t0 = rclcpp::Duration(pt0.time_from_start).seconds();
        double t1 = rclcpp::Duration(pt1.time_from_start).seconds();
        
        double dt = t1 - t0;
        double ratio = (dt > 1e-6) ? (current_time_offset - t0) / dt : 1.0;
        ratio = std::clamp(ratio, 0.0, 1.0);
        
        for (size_t j = 0; j < js_msg.name.size(); ++j) {
           double pos0 = j < pt0.positions.size() ? pt0.positions[j] : 0.0;
           double pos1 = j < pt1.positions.size() ? pt1.positions[j] : 0.0;
           js_msg.position.push_back(pos0 + ratio * (pos1 - pos0));
           
           double vel0 = j < pt0.velocities.size() ? pt0.velocities[j] : 0.0;
           double vel1 = j < pt1.velocities.size() ? pt1.velocities[j] : 0.0;
           js_msg.velocity.push_back(vel0 + ratio * (vel1 - vel0));
        }
      }
      
      {
        std::lock_guard<std::mutex> lock(target_mutex_);
        latest_target_ = js_msg;
      }
      
      loop_rate.sleep();
    }
    
    if (rclcpp::ok()) {
      result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded. Entering posture maintenance.");
    }
  }

  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::JointState latest_target_;
  std::mutex target_mutex_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveItActionServer>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
