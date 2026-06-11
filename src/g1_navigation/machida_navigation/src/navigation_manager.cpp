#include <algorithm>
#include <atomic>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/srv/get_plan.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"

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

class NavigationManager : public rclcpp::Node
{
public:
  explicit NavigationManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("navigation_manager", options)
  {
    declare_parameter("map_frame",               std::string("map"));
    declare_parameter("robot_base_frame",        std::string("base_footprint"));
    declare_parameter("local_costmap_topic",     std::string("/local_costmap"));
    declare_parameter("augmented_costmap_topic", std::string("/augmented_local_costmap"));
    declare_parameter("path_obstacle_threshold", 75);
    declare_parameter("path_check_horizon",      1.5);   // how far ahead to check for obstacles [m]
    declare_parameter("path_check_use_memory",   true);  // true=memory grid, false=raw local costmap
    declare_parameter("local_plan_frequency",    5.0);
    declare_parameter("goal_tolerance",          0.15);
    declare_parameter("replan_cooldown",         2.0);
    declare_parameter("obstacle_decay_rate",     5);
    declare_parameter("decay_frequency",         2.0);
    declare_parameter("goal_yaw_tolerance",      0.05);

    map_frame_        = get_parameter("map_frame").as_string();
    robot_base_frame_ = get_parameter("robot_base_frame").as_string();

    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      std::bind(&NavigationManager::goal_callback, this, std::placeholders::_1));

    costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      get_parameter("local_costmap_topic").as_string(), 1,
      std::bind(&NavigationManager::costmap_callback, this, std::placeholders::_1));

    auto latched_qos = rclcpp::QoS(1).transient_local().reliable();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map2d", latched_qos,
      std::bind(&NavigationManager::map_callback, this, std::placeholders::_1));

    current_plan_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/execute_path_plan", latched_qos);
    execute_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/execute_local_planner", rclcpp::QoS(1).reliable());
    augmented_costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      get_parameter("augmented_costmap_topic").as_string(), latched_qos);
    blocked_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/path_blocked_marker", rclcpp::QoS(1).reliable());

    plan_client_ = create_client<nav_msgs::srv::GetPlan>("/compute_global_plan");

    const double local_freq = std::max(0.1, get_parameter("local_plan_frequency").as_double());
    local_plan_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / local_freq),
      std::bind(&NavigationManager::local_plan_callback, this));

    const double decay_freq = std::max(0.1, get_parameter("decay_frequency").as_double());
    decay_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / decay_freq),
      std::bind(&NavigationManager::decay_timer_callback, this));

    action_server_ = rclcpp_action::create_server<nav2_msgs::action::NavigateToPose>(
      this, "navigate_to_pose",
      std::bind(&NavigationManager::handle_goal,     this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&NavigationManager::handle_cancel,   this, std::placeholders::_1),
      std::bind(&NavigationManager::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "NavigationManager ready: monitor=%.1f Hz check_horizon=%.2f m decay=%.1f Hz cooldown=%.2f s",
      local_freq,
      get_parameter("path_check_horizon").as_double(),
      decay_freq,
      get_parameter("replan_cooldown").as_double());
    RCLCPP_INFO(get_logger(), "Action server 'navigate_to_pose' ready");
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const nav2_msgs::action::NavigateToPose::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "Action goal received: (%.3f, %.3f)",
      goal->pose.pose.position.x, goal->pose.pose.position.y);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> /*goal_handle*/)
  {
    RCLCPP_INFO(get_logger(), "Navigation cancel requested via action");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle)
  {
    {
      std::lock_guard<std::mutex> lock(action_mutex_);
      if (active_goal_handle_ && active_goal_handle_->is_active()) {
        active_goal_handle_->abort(std::make_shared<nav2_msgs::action::NavigateToPose::Result>());
      }
      active_goal_handle_ = goal_handle;
      action_start_time_  = now();
    }

    const auto & pose = goal_handle->get_goal()->pose;
    {
      std::lock_guard<std::mutex> lock(path_mutex_);
      stored_goal_      = pose;
      goal_yaw_         = yaw_from_quat(pose.pose.orientation);
      position_reached_ = false;
    }
    request_plan(pose);
  }

  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(memory_mutex_);
    if (memory_initialized_) return;

    memory_info_ = msg->info;
    memory_grid_.assign(
      static_cast<size_t>(msg->info.width) * msg->info.height, 0);
    memory_initialized_ = true;

    RCLCPP_INFO(get_logger(),
      "Obstacle memory grid initialized: %dx%d res=%.3f m/cell",
      msg->info.width, msg->info.height, msg->info.resolution);
  }

  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "Goal received: (%.3f, %.3f)",
      msg->pose.position.x, msg->pose.position.y);
    {
      std::lock_guard<std::mutex> lock(path_mutex_);
      stored_goal_      = *msg;
      goal_yaw_         = yaw_from_quat(msg->pose.orientation);
      position_reached_ = false;
    }
    request_plan(*msg);
  }

  void costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    {
      std::lock_guard<std::mutex> lock(costmap_mutex_);
      local_costmap_ = msg;
    }
    {
      std::lock_guard<std::mutex> lock(memory_mutex_);
      if (memory_initialized_) {
        stamp_to_memory_locked(*msg);
      }
    }
  }

  void stamp_to_memory_locked(const nav_msgs::msg::OccupancyGrid & costmap)
  {
    const auto & linfo = costmap.info;
    const std::string & local_frame = costmap.header.frame_id;
    const int lw = static_cast<int>(linfo.width);
    const int lh = static_cast<int>(linfo.height);
    const int obstacle_thr = get_parameter("path_obstacle_threshold").as_int();

    double tx = 0.0, ty = 0.0;
    double cos_r = 1.0, sin_r = 0.0;
    const bool same_frame = (local_frame == map_frame_ || local_frame.empty());
    if (!same_frame) {
      try {
        const auto tf = tf_buffer_->lookupTransform(
          map_frame_, local_frame, tf2::TimePointZero);
        tx = tf.transform.translation.x;
        ty = tf.transform.translation.y;
        const double yaw = yaw_from_quat(tf.transform.rotation);
        cos_r = std::cos(yaw);
        sin_r = std::sin(yaw);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
          "stamp_to_memory: TF %s -> %s failed: %s",
          local_frame.c_str(), map_frame_.c_str(), ex.what());
        return;
      }
    }

    const double lox  = linfo.origin.position.x;
    const double loy  = linfo.origin.position.y;
    const double lres = linfo.resolution;
    const double gox  = memory_info_.origin.position.x;
    const double goy  = memory_info_.origin.position.y;
    const double gres = memory_info_.resolution;
    const int gw = static_cast<int>(memory_info_.width);
    const int gh = static_cast<int>(memory_info_.height);

    for (int ly = 0; ly < lh; ++ly) {
      for (int lx = 0; lx < lw; ++lx) {
        const int8_t val = costmap.data[static_cast<size_t>(ly * lw + lx)];
        if (val <= 0) continue;

        const double lcx = lox + (lx + 0.5) * lres;
        const double lcy = loy + (ly + 0.5) * lres;

        double gcx, gcy;
        if (same_frame) {
          gcx = lcx;
          gcy = lcy;
        } else {
          gcx = cos_r * lcx - sin_r * lcy + tx;
          gcy = sin_r * lcx + cos_r * lcy + ty;
        }

        const int gx = static_cast<int>((gcx - gox) / gres);
        const int gy = static_cast<int>((gcy - goy) / gres);
        if (gx < 0 || gx >= gw || gy < 0 || gy >= gh) continue;

        auto & cell = memory_grid_[static_cast<size_t>(gy * gw + gx)];
        const int8_t stamp_val = (static_cast<int>(val) >= obstacle_thr) ?
          static_cast<int8_t>(100) : val;
        if (stamp_val > cell) cell = stamp_val;
      }
    }
  }

  void decay_timer_callback()
  {
    nav_msgs::msg::OccupancyGrid::SharedPtr current_costmap;
    {
      std::lock_guard<std::mutex> lock(costmap_mutex_);
      current_costmap = local_costmap_;
    }

    std::lock_guard<std::mutex> lock(memory_mutex_);
    if (!memory_initialized_) return;

    const int decay = get_parameter("obstacle_decay_rate").as_int();
    for (auto & cell : memory_grid_) {
      if (cell > 0) {
        const int next = static_cast<int>(cell) - decay;
        cell = static_cast<int8_t>(next > 0 ? next : 0);
      }
    }

    if (current_costmap && !current_costmap->data.empty()) {
      stamp_to_memory_locked(*current_costmap);
    }

    publish_augmented_costmap_locked();
  }

  void publish_augmented_costmap_locked()
  {
    nav_msgs::msg::OccupancyGrid out;
    out.header.stamp    = now();
    out.header.frame_id = map_frame_;
    out.info            = memory_info_;
    out.data            = memory_grid_;
    augmented_costmap_pub_->publish(out);
  }

  void publish_augmented_costmap_now()
  {
    std::lock_guard<std::mutex> lock(memory_mutex_);
    if (!memory_initialized_) return;
    publish_augmented_costmap_locked();
  }

  void request_plan(const geometry_msgs::msg::PoseStamped & goal)
  {
    if (planning_in_progress_.exchange(true)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "Plan request ignored: previous planning still in progress");
      return;
    }

    if (!plan_client_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "Service /compute_global_plan not available yet");
      planning_in_progress_ = false;
      return;
    }

    publish_augmented_costmap_now();

    auto req = std::make_shared<nav_msgs::srv::GetPlan::Request>();
    req->goal = goal;

    plan_client_->async_send_request(
      req,
      [this](rclcpp::Client<nav_msgs::srv::GetPlan>::SharedFuture future) {
        auto response = future.get();
        planning_in_progress_ = false;

        if (response->plan.poses.empty()) {
          RCLCPP_WARN(get_logger(), "Global planner returned empty path");
          return;
        }

        // Build the global path message to publish to local planners
        nav_msgs::msg::Path global_path;
        global_path.header.stamp    = now();
        global_path.header.frame_id = response->plan.header.frame_id;

        {
          std::lock_guard<std::mutex> lock(path_mutex_);
          path_.clear();
          path_.reserve(response->plan.poses.size());
          for (const auto & pose : response->plan.poses) {
            path_.push_back({pose.pose.position.x, pose.pose.position.y});
            global_path.poses.push_back(pose);
            global_path.poses.back().header = global_path.header;
          }
          path_frame_id_ = response->plan.header.frame_id;
          has_path_      = true;

          // Apply goal orientation to the last pose
          if (!global_path.poses.empty()) {
            global_path.poses.back().pose.orientation = stored_goal_.pose.orientation;
          }
        }

        // Publish global path directly to local planners
        current_plan_pub_->publish(global_path);
        publish_execute(true);

        RCLCPP_INFO(get_logger(), "Global path published: %zu poses",
          global_path.poses.size());
      });
  }

  // Monitor path ahead for obstacles and trigger global replan if blocked
  void local_plan_callback()
  {
    if (planning_in_progress_) return;

    {
      std::lock_guard<std::mutex> alock(action_mutex_);
      if (active_goal_handle_ && active_goal_handle_->is_canceling()) {
        {
          std::lock_guard<std::mutex> plock(path_mutex_);
          has_path_         = false;
          position_reached_ = false;
        }
        publish_execute(false);
        active_goal_handle_->canceled(
          std::make_shared<nav2_msgs::action::NavigateToPose::Result>());
        active_goal_handle_ = nullptr;
        RCLCPP_INFO(get_logger(), "Navigation cancelled");
        return;
      }
    }

    std::vector<Point2D> path;
    geometry_msgs::msg::PoseStamped stored_goal;
    bool has_path;
    bool position_reached;
    double goal_yaw;
    {
      std::lock_guard<std::mutex> lock(path_mutex_);
      path             = path_;
      stored_goal      = stored_goal_;
      has_path         = has_path_;
      position_reached = position_reached_;
      goal_yaw         = goal_yaw_;
    }

    if (!has_path || path.empty()) return;

    Point2D robot;
    double robot_yaw = 0.0;
    if (!get_robot_pose(robot, robot_yaw)) return;

    const double goal_tol     = get_parameter("goal_tolerance").as_double();
    const double goal_yaw_tol = get_parameter("goal_yaw_tolerance").as_double();
    const double dist_to_goal = distance(robot, path.back());

    // Goal reached: yaw alignment phase
    if (position_reached) {
      double yaw_err = goal_yaw - robot_yaw;
      while (yaw_err >  M_PI) yaw_err -= 2.0 * M_PI;
      while (yaw_err < -M_PI) yaw_err += 2.0 * M_PI;
      if (std::abs(yaw_err) <= goal_yaw_tol) {
        {
          std::lock_guard<std::mutex> lock(path_mutex_);
          has_path_         = false;
          position_reached_ = false;
        }
        publish_execute(false);
        RCLCPP_INFO(get_logger(), "Global goal reached (yaw_err=%.3f rad)", yaw_err);
        {
          std::lock_guard<std::mutex> alock(action_mutex_);
          if (active_goal_handle_ && active_goal_handle_->is_active()) {
            active_goal_handle_->succeed(
              std::make_shared<nav2_msgs::action::NavigateToPose::Result>());
            active_goal_handle_ = nullptr;
          }
        }
        return;
      }
      publish_execute(true);
      return;
    }

    // Goal reached: position phase
    if (dist_to_goal <= goal_tol) {
      {
        std::lock_guard<std::mutex> lock(path_mutex_);
        position_reached_ = true;
      }
      publish_execute(true);
      return;
    }

    // Check path points ahead for obstacles
    const size_t nearest_idx   = find_nearest_index(path, robot);
    const double check_horizon = get_parameter("path_check_horizon").as_double();
    const int    obs_threshold = get_parameter("path_obstacle_threshold").as_int();
    const bool   use_memory    = get_parameter("path_check_use_memory").as_bool();

    bool path_blocked = false;
    if (use_memory) {
      std::lock_guard<std::mutex> lock(memory_mutex_);
      if (memory_initialized_) {
        const double mox  = memory_info_.origin.position.x;
        const double moy  = memory_info_.origin.position.y;
        const double mres = memory_info_.resolution;
        const int    mw   = static_cast<int>(memory_info_.width);
        const int    mh   = static_cast<int>(memory_info_.height);

        double acc = 0.0;
        for (size_t i = nearest_idx; i < path.size(); ++i) {
          if (i > nearest_idx) acc += distance(path[i - 1], path[i]);
          if (acc > check_horizon) break;

          const int mx = static_cast<int>((path[i].x - mox) / mres);
          const int my = static_cast<int>((path[i].y - moy) / mres);
          if (mx < 0 || mx >= mw || my < 0 || my >= mh) continue;

          if (static_cast<int>(memory_grid_[static_cast<size_t>(my * mw + mx)]) >= obs_threshold) {
            path_blocked = true;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
              "Path blocked by obstacle at (%.2f, %.2f)", path[i].x, path[i].y);
            publish_blocked_marker(path[i].x, path[i].y);
            break;
          }
        }
      }
    } else {
      nav_msgs::msg::OccupancyGrid::SharedPtr costmap;
      {
        std::lock_guard<std::mutex> lock(costmap_mutex_);
        costmap = local_costmap_;
      }
      if (costmap) {
        const auto & info      = costmap->info;
        const std::string & cm_frame = costmap->header.frame_id;
        const double ox  = info.origin.position.x;
        const double oy  = info.origin.position.y;
        const double res = info.resolution;
        const int    cw  = static_cast<int>(info.width);
        const int    ch  = static_cast<int>(info.height);

        double tx = 0.0, ty = 0.0, cos_r = 1.0, sin_r = 0.0;
        bool tf_ok = true;
        const bool same_frame = (cm_frame.empty() || cm_frame == map_frame_);
        if (!same_frame) {
          try {
            const auto tf = tf_buffer_->lookupTransform(cm_frame, map_frame_, tf2::TimePointZero);
            tx    = tf.transform.translation.x;
            ty    = tf.transform.translation.y;
            const double yaw = yaw_from_quat(tf.transform.rotation);
            cos_r = std::cos(yaw);
            sin_r = std::sin(yaw);
          } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
              "path_check TF %s->%s: %s", map_frame_.c_str(), cm_frame.c_str(), ex.what());
            tf_ok = false;
          }
        }

        if (tf_ok) {
          double acc = 0.0;
          for (size_t i = nearest_idx; i < path.size(); ++i) {
            if (i > nearest_idx) acc += distance(path[i - 1], path[i]);
            if (acc > check_horizon) break;

            const double cx = same_frame ? path[i].x : cos_r * path[i].x - sin_r * path[i].y + tx;
            const double cy = same_frame ? path[i].y : sin_r * path[i].x + cos_r * path[i].y + ty;
            const int gx = static_cast<int>((cx - ox) / res);
            const int gy = static_cast<int>((cy - oy) / res);
            if (gx < 0 || gx >= cw || gy < 0 || gy >= ch) continue;

            if (static_cast<int>(costmap->data[static_cast<size_t>(gy * cw + gx)]) >= obs_threshold) {
              path_blocked = true;
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "Path blocked by obstacle at (%.2f, %.2f)", path[i].x, path[i].y);
              publish_blocked_marker(path[i].x, path[i].y);
              break;
            }
          }
        }
      }
    }

    if (path_blocked) {
      publish_execute(false);
      const rclcpp::Time now_t = get_clock()->now();
      const double cooldown = get_parameter("replan_cooldown").as_double();
      if (!replan_initialized_ ||
          (now_t - last_replan_time_).seconds() >= cooldown)
      {
        last_replan_time_   = now_t;
        replan_initialized_ = true;
        request_plan(stored_goal);
      }
      return;
    }

    publish_execute(true);

    {
      std::lock_guard<std::mutex> alock(action_mutex_);
      if (active_goal_handle_ && active_goal_handle_->is_active()) {
        auto fb = std::make_shared<nav2_msgs::action::NavigateToPose::Feedback>();
        fb->current_pose.header.stamp    = now();
        fb->current_pose.header.frame_id = map_frame_;
        fb->current_pose.pose.position.x = robot.x;
        fb->current_pose.pose.position.y = robot.y;
        fb->distance_remaining = static_cast<float>(distance(robot, path.back()));
        fb->navigation_time    = now() - action_start_time_;
        active_goal_handle_->publish_feedback(fb);
      }
    }
  }

  bool get_robot_pose(Point2D & pose_out, double & yaw_out) const
  {
    try {
      const auto tf = tf_buffer_->lookupTransform(
        map_frame_, robot_base_frame_, tf2::TimePointZero);
      pose_out.x = tf.transform.translation.x;
      pose_out.y = tf.transform.translation.y;
      yaw_out    = yaw_from_quat(tf.transform.rotation);
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), const_cast<rclcpp::Clock&>(*get_clock()), 2000,
        "TF lookup %s -> %s failed: %s",
        map_frame_.c_str(), robot_base_frame_.c_str(), ex.what());
      return false;
    }
  }

  size_t find_nearest_index(
    const std::vector<Point2D> & path,
    const Point2D & pose) const
  {
    size_t best = 0;
    double best_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < path.size(); ++i) {
      const double d = distance(pose, path[i]);
      if (d < best_dist) { best_dist = d; best = i; }
    }
    return best;
  }

  void publish_execute(bool value)
  {
    std_msgs::msg::Bool msg;
    msg.data = value;
    execute_pub_->publish(msg);
  }

  void publish_blocked_marker(double x, double y)
  {
    visualization_msgs::msg::Marker m;
    m.header.stamp    = now();
    m.header.frame_id = map_frame_;
    m.ns              = "path_blocked";
    m.id              = 0;
    m.type            = visualization_msgs::msg::Marker::SPHERE;
    m.action          = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = x;
    m.pose.position.y = y;
    m.pose.position.z = 0.2;
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = 0.3;
    m.color.r = 1.0f; m.color.g = 0.0f; m.color.b = 0.0f; m.color.a = 1.0f;
    m.lifetime = rclcpp::Duration::from_seconds(2.0);
    blocked_marker_pub_->publish(m);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr    costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr    map_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                current_plan_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                execute_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr       augmented_costmap_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr    blocked_marker_pub_;
  rclcpp::Client<nav_msgs::srv::GetPlan>::SharedPtr                plan_client_;
  rclcpp::TimerBase::SharedPtr                                     local_plan_timer_;
  rclcpp::TimerBase::SharedPtr                                     decay_timer_;
  std::unique_ptr<tf2_ros::Buffer>                                 tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener>                      tf_listener_;

  std::mutex path_mutex_;
  std::vector<Point2D>            path_;
  geometry_msgs::msg::PoseStamped stored_goal_;
  std::string                     path_frame_id_;
  bool                            has_path_{false};
  double                          goal_yaw_{0.0};
  bool                            position_reached_{false};

  std::mutex costmap_mutex_;
  nav_msgs::msg::OccupancyGrid::SharedPtr local_costmap_;

  std::mutex memory_mutex_;
  nav_msgs::msg::MapMetaData memory_info_;
  std::vector<int8_t>        memory_grid_;
  bool                       memory_initialized_{false};

  std::atomic<bool> planning_in_progress_{false};
  rclcpp::Time      last_replan_time_;
  bool              replan_initialized_{false};

  std::string map_frame_;
  std::string robot_base_frame_;

  rclcpp_action::Server<nav2_msgs::action::NavigateToPose>::SharedPtr                  action_server_;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> active_goal_handle_;
  std::mutex   action_mutex_;
  rclcpp::Time action_start_time_;
};

}  // namespace machida_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<machida_navigation::NavigationManager>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
