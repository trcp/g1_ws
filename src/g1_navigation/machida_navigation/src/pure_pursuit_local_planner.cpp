#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace machida_navigation
{

struct Point2D
{
  double x{0.0};
  double y{0.0};
};

static double yaw_from_quat(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

static double distance(const Point2D & a, const Point2D & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

class PurePursuitLocalPlanner : public rclcpp::Node
{
public:
  explicit PurePursuitLocalPlanner(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("pure_pursuit_local_planner", options)
  {
    declare_parameter("path_topic",          std::string("/execute_path_plan"));
    declare_parameter("cmd_vel_topic",       std::string("/cmd_vel"));
    declare_parameter("execute_topic",       std::string("/execute_local_planner"));
    declare_parameter("map_frame",           std::string("map"));
    declare_parameter("robot_base_frame",    std::string("base_footprint"));
    declare_parameter("control_frequency",   20.0);
    declare_parameter("lookahead_distance",  0.5);
    declare_parameter("linear_velocity",     0.25);
    declare_parameter("max_angular_velocity", 1.0);
    declare_parameter("max_path_deviation",  0.5);
    declare_parameter("goal_tolerance",      0.15);
    declare_parameter("slowdown_distance",   0.6);
    declare_parameter("min_linear_velocity", 0.2);
    declare_parameter("holonomic",           false);
    declare_parameter("max_linear_acceleration",  0.5);
    declare_parameter("max_angular_acceleration", 2.0);

    path_topic_       = get_parameter("path_topic").as_string();
    cmd_vel_topic_    = get_parameter("cmd_vel_topic").as_string();
    map_frame_        = get_parameter("map_frame").as_string();
    robot_base_frame_ = get_parameter("robot_base_frame").as_string();

    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      path_topic_, rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&PurePursuitLocalPlanner::path_callback, this, std::placeholders::_1));

    execute_sub_ = create_subscription<std_msgs::msg::Bool>(
      get_parameter("execute_topic").as_string(), 1,
      std::bind(&PurePursuitLocalPlanner::execute_callback, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

    const double frequency = std::max(1.0, get_parameter("control_frequency").as_double());
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / frequency),
      std::bind(&PurePursuitLocalPlanner::control_loop, this));

    RCLCPP_INFO(get_logger(),
      "PurePursuitLocalPlanner ready: path=%s cmd_vel=%s execute=%s TF %s -> %s mode=%s",
      path_topic_.c_str(), cmd_vel_topic_.c_str(),
      get_parameter("execute_topic").as_string().c_str(),
      map_frame_.c_str(), robot_base_frame_.c_str(),
      get_parameter("holonomic").as_bool() ? "holonomic" : "differential");
  }

private:
  void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    path_.clear();
    path_.reserve(msg->poses.size());
    for (const auto & pose : msg->poses) {
      path_.push_back({pose.pose.position.x, pose.pose.position.y});
    }

    path_frame_id_ = msg->header.frame_id;
    nearest_index_ = 0;
    reached_goal_  = false;

    RCLCPP_INFO(get_logger(), "Path received: %zu poses, frame=%s",
      path_.size(), path_frame_id_.c_str());
  }

  void execute_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(execute_mutex_);
    executing_ = msg->data;
  }

  void control_loop()
  {
    // Stop while execute_local_planner is false
    {
      std::lock_guard<std::mutex> lock(execute_mutex_);
      if (!executing_) {
        publish_stop();
        return;
      }
    }

    Point2D pose;
    double yaw = 0.0;
    bool has_pose = false;
    try {
      const auto tf = tf_buffer_->lookupTransform(
        map_frame_, robot_base_frame_, tf2::TimePointZero);
      pose.x = tf.transform.translation.x;
      pose.y = tf.transform.translation.y;
      yaw    = yaw_from_quat(tf.transform.rotation);
      has_pose = true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Failed to lookup TF %s -> %s: %s",
        map_frame_.c_str(), robot_base_frame_.c_str(), ex.what());
    }

    std::vector<Point2D> path;
    std::string path_frame;
    bool reached_goal;
    size_t nearest_index;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      path          = path_;
      path_frame    = path_frame_id_;
      reached_goal  = reached_goal_;
      nearest_index = nearest_index_;
    }

    if (!has_pose || path.empty()) {
      publish_stop();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Waiting for TF pose and path: pose=%s path_size=%zu",
        has_pose ? "true" : "false", path.size());
      return;
    }

    if (!path_frame.empty() && path_frame != map_frame_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
        "Path frame (%s) and pose frame (%s) differ; this node assumes same-frame coordinates",
        path_frame.c_str(), map_frame_.c_str());
    }

    const Point2D goal = path.back();
    const double dist_to_goal = distance(pose, goal);
    if (reached_goal || dist_to_goal <= get_parameter("goal_tolerance").as_double()) {
      publish_stop();
      std::lock_guard<std::mutex> lock(mutex_);
      if (!reached_goal_) {
        RCLCPP_INFO(get_logger(), "Goal reached (dist=%.3f m)", dist_to_goal);
      }
      reached_goal_ = true;
      return;
    }

    nearest_index = find_nearest_index(path, pose, nearest_index);
    const size_t target_index = find_lookahead_index(
      path, pose, nearest_index, get_parameter("lookahead_distance").as_double());
    const Point2D target = path[target_index];

    const double path_deviation = distance(pose, path[nearest_index]);
    const double max_path_deviation = get_parameter("max_path_deviation").as_double();
    if (max_path_deviation > 0.0 && path_deviation > max_path_deviation) {
      publish_stop();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "Stopping: path deviation %.3f m exceeds %.3f m", path_deviation, max_path_deviation);
      return;
    }

    const double dx = target.x - pose.x;
    const double dy = target.y - pose.y;
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    const double target_x_robot = cos_yaw * dx + sin_yaw * dy;
    const double target_y_robot = -sin_yaw * dx + cos_yaw * dy;
    const double lookahead_sq = std::max(
      target_x_robot * target_x_robot + target_y_robot * target_y_robot, 1e-6);

    double v = desired_linear_velocity(dist_to_goal);
    geometry_msgs::msg::Twist cmd;

    if (get_parameter("holonomic").as_bool()) {
      // Holonomic: translate directly toward target + align heading
      const double dist_to_target = std::hypot(target_x_robot, target_y_robot);
      if (v > 0.0) {
        v = std::max(v, get_parameter("min_linear_velocity").as_double());
      }
      if (dist_to_target > 1e-6) {
        cmd.linear.x = v * target_x_robot / dist_to_target;
        cmd.linear.y = v * target_y_robot / dist_to_target;
      }
      // P-control heading error toward target (clamped to max_angular_velocity)
      const double desired_yaw = std::atan2(target.y - pose.y, target.x - pose.x);
      double heading_error = desired_yaw - yaw;
      while (heading_error >  M_PI) heading_error -= 2.0 * M_PI;
      while (heading_error < -M_PI) heading_error += 2.0 * M_PI;
      const double max_w = get_parameter("max_angular_velocity").as_double();
      cmd.angular.z = std::clamp(heading_error, -max_w, max_w);
    } else {
      // Differential drive: pure pursuit curvature control
      const double curvature = 2.0 * target_y_robot / lookahead_sq;
      const double max_w = get_parameter("max_angular_velocity").as_double();
      double w = 0.0;

      if (target_x_robot < 0.0) {
        v = 0.0;
        w = (target_y_robot >= 0.0) ? max_w : -max_w;
      } else {
        const double abs_curvature = std::abs(curvature);
        if (abs_curvature > 1e-6 && max_w > 0.0) {
          v = std::min(v, max_w / abs_curvature);
        }
        w = std::clamp(v * curvature, -max_w, max_w);
      }

      if (v > 0.0) {
        v = std::max(v, get_parameter("min_linear_velocity").as_double());
      }

      cmd.linear.x = v;
      cmd.angular.z = w;
    }

    const double dt = 1.0 / std::max(1.0, get_parameter("control_frequency").as_double());
    const double max_v_acc = get_parameter("max_linear_acceleration").as_double();
    const double max_w_acc = get_parameter("max_angular_acceleration").as_double();
    cmd.linear.x  = std::clamp(cmd.linear.x,  prev_v_ - max_v_acc * dt, prev_v_ + max_v_acc * dt);
    cmd.linear.y  = std::clamp(cmd.linear.y,  prev_vy_ - max_v_acc * dt, prev_vy_ + max_v_acc * dt);
    cmd.angular.z = std::clamp(cmd.angular.z, prev_w_ - max_w_acc * dt, prev_w_ + max_w_acc * dt);
    prev_v_  = cmd.linear.x;
    prev_vy_ = cmd.linear.y;
    prev_w_  = cmd.angular.z;

    cmd_pub_->publish(cmd);

    {
      std::lock_guard<std::mutex> lock(mutex_);
      nearest_index_ = nearest_index;
    }
  }

  size_t find_nearest_index(
    const std::vector<Point2D> & path,
    const Point2D & pose,
    size_t start_index) const
  {
    const size_t start = std::min(start_index, path.size() - 1);
    size_t best = start;
    double best_dist = distance(pose, path[start]);

    for (size_t i = start + 1; i < path.size(); ++i) {
      const double d = distance(pose, path[i]);
      if (d < best_dist) {
        best = i;
        best_dist = d;
      }
    }
    return best;
  }

  size_t find_lookahead_index(
    const std::vector<Point2D> & path,
    const Point2D & pose,
    size_t start_index,
    double lookahead_distance) const
  {
    const double min_lookahead = std::max(lookahead_distance, 1e-3);
    double path_distance = distance(pose, path[start_index]);
    for (size_t i = start_index + 1; i < path.size(); ++i) {
      path_distance += distance(path[i - 1], path[i]);
      if (path_distance >= min_lookahead) {
        return i;
      }
    }
    return path.size() - 1;
  }

  double desired_linear_velocity(double dist_to_goal) const
  {
    const double v = get_parameter("linear_velocity").as_double();
    const double slowdown_distance = get_parameter("slowdown_distance").as_double();
    if (slowdown_distance <= 1e-6) return v;

    const double scale = std::clamp(dist_to_goal / slowdown_distance, 0.0, 1.0);
    return v * scale;
  }

  void publish_stop()
  {
    cmd_pub_->publish(geometry_msgs::msg::Twist{});
    prev_v_  = 0.0;
    prev_vy_ = 0.0;
    prev_w_  = 0.0;
  }

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr      path_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr      execute_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr   cmd_pub_;
  rclcpp::TimerBase::SharedPtr                              timer_;
  std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::mutex mutex_;
  std::vector<Point2D> path_;
  bool reached_goal_{false};
  size_t nearest_index_{0};
  std::string path_frame_id_;
  std::string path_topic_;
  std::string cmd_vel_topic_;
  std::string map_frame_;
  std::string robot_base_frame_;

  std::mutex execute_mutex_;
  bool executing_{false};

  double prev_v_{0.0};
  double prev_vy_{0.0};
  double prev_w_{0.0};
};

}  // namespace machida_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<machida_navigation::PurePursuitLocalPlanner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
