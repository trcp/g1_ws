#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <vector>

#include "machida_navigation/astar.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/srv/get_plan.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace machida_navigation
{

static geometry_msgs::msg::Quaternion yaw_to_quat(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);
  return q;
}

class GlobalPlannerNode : public rclcpp::Node
{
public:
  explicit GlobalPlannerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("global_planner", options)
  {
    declare_parameter("obstacle_threshold", 50);
    declare_parameter("use_smoothing", false);
    declare_parameter("obstacle_cost_weight", 5.0);
    declare_parameter("planner_obstacle_threshold", 99);
    declare_parameter("unknown_cost", 0);
    declare_parameter("log_interval", 500);
    declare_parameter("robot_base_frame", std::string("base_footprint"));
    declare_parameter("local_costmap_topic", std::string("/local_costmap"));
    declare_parameter("goal_snap_to_free", true);
    declare_parameter("goal_snap_max_dist", 1.0);

    robot_base_frame_ = get_parameter("robot_base_frame").as_string();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    auto latched_qos = rclcpp::QoS(1).transient_local().reliable();

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map2d", latched_qos,
      std::bind(&GlobalPlannerNode::map_callback, this, std::placeholders::_1));

    global_costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/global_costmap", latched_qos,
      std::bind(&GlobalPlannerNode::costmap_callback, this, std::placeholders::_1));

    local_costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      get_parameter("local_costmap_topic").as_string(), 1,
      std::bind(&GlobalPlannerNode::local_costmap_callback, this, std::placeholders::_1));

    path_pub_ = create_publisher<nav_msgs::msg::Path>("/global_path", latched_qos);

    plan_srv_ = create_service<nav_msgs::srv::GetPlan>(
      "/compute_global_plan",
      std::bind(&GlobalPlannerNode::handle_plan_request, this,
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(),
      "GlobalPlannerNode ready: service /compute_global_plan; TF map -> %s",
      robot_base_frame_.c_str());
  }

private:
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    current_map_ = msg;
    RCLCPP_INFO(get_logger(), "Map received: %dx%d, res=%.3f m/cell",
      msg->info.width, msg->info.height, msg->info.resolution);
  }

  void costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    current_costmap_ = msg;
    RCLCPP_INFO(get_logger(), "Costmap received: %dx%d",
      msg->info.width, msg->info.height);
  }

  void local_costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(local_costmap_mutex_);
    current_local_costmap_ = msg;
  }

  void handle_plan_request(
    const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request,
    std::shared_ptr<nav_msgs::srv::GetPlan::Response> response)
  {
    RCLCPP_INFO(get_logger(), "Plan requested: goal=(%.3f, %.3f)",
      request->goal.pose.position.x, request->goal.pose.position.y);

    nav_msgs::msg::OccupancyGrid::SharedPtr current_map;
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      if (!current_map_) {
        RCLCPP_ERROR(get_logger(), "No map received yet — cannot plan");
        return;
      }
      current_map = current_map_;
    }

    nav_msgs::msg::OccupancyGrid::SharedPtr current_costmap;
    {
      std::lock_guard<std::mutex> lock(costmap_mutex_);
      if (!current_costmap_) {
        RCLCPP_ERROR(get_logger(), "No costmap received yet — cannot plan");
        return;
      }
      current_costmap = current_costmap_;
    }

    const auto & info = current_map->info;
    const std::string frame_id = map_frame_id(*current_map);

    if (current_costmap->info.width != info.width ||
      current_costmap->info.height != info.height)
    {
      RCLCPP_ERROR(get_logger(),
        "Costmap size (%dx%d) does not match map size (%dx%d) — cannot plan",
        current_costmap->info.width, current_costmap->info.height,
        info.width, info.height);
      return;
    }

    double start_x;
    double start_y;
    if (!get_start_pose(frame_id, start_x, start_y)) {
      RCLCPP_ERROR(get_logger(),
        "No usable start pose — failed to lookup TF %s -> %s",
        frame_id.c_str(), robot_base_frame_.c_str());
      return;
    }

    auto [start_gx, start_gy] = world_to_grid(start_x, start_y, info);
    auto [goal_gx, goal_gy]   = world_to_grid(
      request->goal.pose.position.x, request->goal.pose.position.y, info);

    if (!in_bounds(start_gx, start_gy, info)) {
      RCLCPP_ERROR(get_logger(), "Start (%d,%d) is outside map bounds", start_gx, start_gy);
      return;
    }
    if (!in_bounds(goal_gx, goal_gy, info)) {
      RCLCPP_ERROR(get_logger(), "Goal (%d,%d) is outside map bounds", goal_gx, goal_gy);
      return;
    }

    int obstacle_threshold = get_parameter("obstacle_threshold").as_int();
    int planner_obstacle_threshold = get_parameter("planner_obstacle_threshold").as_int();
    planner_obstacle_threshold = std::clamp(planner_obstacle_threshold, obstacle_threshold + 1, 100);
    double obstacle_cost_weight = get_parameter("obstacle_cost_weight").as_double();
    int unknown_cost = get_parameter("unknown_cost").as_int();
    // unknown cells (-1) are treated as obstacles only when unknown_cost makes them impassable
    const bool unknown_is_obstacle = (unknown_cost >= obstacle_threshold);

    // Check against the original map whether this is an actual wall (do not error on inflation regions)
    const auto & orig = current_map->data;
    int8_t start_val = orig[start_gy * info.width + start_gx];
    if ((unknown_is_obstacle && start_val < 0) || static_cast<int>(start_val) >= obstacle_threshold) {
      RCLCPP_ERROR(get_logger(),
        "Start (%d,%d) is inside an actual obstacle (val=%d)", start_gx, start_gy, start_val);
      return;
    }

    // Build planning grid before goal-snap BFS so BFS can check costmap traversability
    auto grid_data = current_costmap->data;
    overlay_local_costmap(grid_data, info, frame_id);
    grid_data[start_gy * info.width + start_gx] = 0;

    int8_t goal_val = orig[goal_gy * info.width + goal_gx];
    if ((unknown_is_obstacle && goal_val < 0) || static_cast<int>(goal_val) >= obstacle_threshold) {
      const bool snap = get_parameter("goal_snap_to_free").as_bool();
      if (!snap) {
        RCLCPP_ERROR(get_logger(),
          "Goal (%d,%d) is inside an actual obstacle (val=%d). "
          "Set goal_snap_to_free:=true to snap to nearest free cell.",
          goal_gx, goal_gy, goal_val);
        return;
      }

      const double snap_max_dist = get_parameter("goal_snap_max_dist").as_double();
      const int max_cells = static_cast<int>(snap_max_dist / info.resolution);
      const int orig_goal_gx = goal_gx, orig_goal_gy = goal_gy;

      std::queue<std::pair<int, int>> bfs_q;
      std::vector<bool> visited(info.width * info.height, false);
      bfs_q.push({goal_gx, goal_gy});
      visited[static_cast<size_t>(goal_gy * info.width + goal_gx)] = true;

      bool found = false;
      while (!bfs_q.empty()) {
        auto [cx, cy] = bfs_q.front();
        bfs_q.pop();

        if (static_cast<int>(grid_data[static_cast<size_t>(cy * info.width + cx)]) < planner_obstacle_threshold) {
          auto [snapped_wx, snapped_wy] = grid_to_world(
            static_cast<float>(cx), static_cast<float>(cy), info);
          RCLCPP_WARN(get_logger(),
            "Goal (%.3f,%.3f) is inside obstacle (val=%d); "
            "snapped to nearest traversable cell (%.3f,%.3f) [%.2f m away]",
            request->goal.pose.position.x, request->goal.pose.position.y,
            static_cast<int>(goal_val),
            snapped_wx, snapped_wy,
            std::hypot(snapped_wx - request->goal.pose.position.x,
                       snapped_wy - request->goal.pose.position.y));
          goal_gx = cx;
          goal_gy = cy;
          found = true;
          break;
        }

        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            if (dx == 0 && dy == 0) continue;
            int nx = cx + dx, ny = cy + dy;
            if (!in_bounds(nx, ny, info)) continue;
            if (visited[static_cast<size_t>(ny * info.width + nx)]) continue;
            int ddx = nx - orig_goal_gx, ddy = ny - orig_goal_gy;
            if (ddx * ddx + ddy * ddy > max_cells * max_cells) continue;
            visited[static_cast<size_t>(ny * info.width + nx)] = true;
            bfs_q.push({nx, ny});
          }
        }
      }

      if (!found) {
        RCLCPP_ERROR(get_logger(),
          "Goal (%d,%d) is inside obstacle; no traversable cell found within %.2f m",
          orig_goal_gx, orig_goal_gy, snap_max_dist);
        return;
      }
    }

    RCLCPP_INFO(get_logger(),
      "Planning: start=(%.3f,%.3f)->(%d,%d) goal=(%d,%d), obstacle_cost_weight=%.2f",
      start_x, start_y, start_gx, start_gy, goal_gx, goal_gy, obstacle_cost_weight);

    int log_interval = get_parameter("log_interval").as_int();
    auto t_start = now();

    grid_data[goal_gy  * info.width + goal_gx]  = 0;

    ProgressCb progress_cb = nullptr;
    if (log_interval > 0) {
      progress_cb = [this](const std::string & msg) {
          RCLCPP_INFO(get_logger(), "%s", msg.c_str());
        };
    }

    auto result = astar(
      grid_data,
      static_cast<int>(info.width),
      static_cast<int>(info.height),
      start_gx, start_gy,
      goal_gx, goal_gy,
      planner_obstacle_threshold,
      progress_cb,
      log_interval,
      static_cast<float>(obstacle_cost_weight));

    if (!result) {
      RCLCPP_WARN(get_logger(), "A* found no valid path");
      return;
    }

    std::vector<std::pair<float, float>> smoothed;
    if (get_parameter("use_smoothing").as_bool()) {
      smoothed = smooth_path(*result);
    } else {
      for (auto & [x, y] : *result) {
        smoothed.emplace_back(static_cast<float>(x), static_cast<float>(y));
      }
    }

    auto ros_path = build_ros_path(smoothed, info, frame_id);
    response->plan = ros_path;
    path_pub_->publish(ros_path);

    auto elapsed_ns = (now() - t_start).nanoseconds();
    RCLCPP_INFO(get_logger(), "Path found: %zu waypoints in %.1f ms",
      ros_path.poses.size(), elapsed_ns / 1e6);
  }

  void overlay_local_costmap(
    std::vector<int8_t> & grid_data,
    const nav_msgs::msg::MapMetaData & info,
    const std::string & global_frame)
  {
    nav_msgs::msg::OccupancyGrid::SharedPtr local_costmap;
    {
      std::lock_guard<std::mutex> lock(local_costmap_mutex_);
      local_costmap = current_local_costmap_;
    }
    if (!local_costmap || local_costmap->data.empty()) return;

    const auto & linfo = local_costmap->info;
    const std::string & local_frame = local_costmap->header.frame_id;
    const int lw = static_cast<int>(linfo.width);
    const int lh = static_cast<int>(linfo.height);

    double tx = 0.0, ty = 0.0;
    double cos_r = 1.0, sin_r = 0.0;
    const bool same_frame = (local_frame == global_frame || local_frame.empty());
    if (!same_frame) {
      try {
        const auto tf = tf_buffer_->lookupTransform(global_frame, local_frame, tf2::TimePointZero);
        tx = tf.transform.translation.x;
        ty = tf.transform.translation.y;
        const auto & q = tf.transform.rotation;
        const double yaw = std::atan2(
          2.0 * (q.w * q.z + q.x * q.y),
          1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        cos_r = std::cos(yaw);
        sin_r = std::sin(yaw);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
          "overlay_local_costmap: TF %s -> %s failed: %s",
          local_frame.c_str(), global_frame.c_str(), ex.what());
        return;
      }
    }

    const double lox  = linfo.origin.position.x;
    const double loy  = linfo.origin.position.y;
    const double lres = linfo.resolution;
    const double gox  = info.origin.position.x;
    const double goy  = info.origin.position.y;
    const double gres = info.resolution;

    for (int ly = 0; ly < lh; ++ly) {
      for (int lx = 0; lx < lw; ++lx) {
        const int8_t val = local_costmap->data[static_cast<size_t>(ly * lw + lx)];
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
        if (!in_bounds(gx, gy, info)) continue;

        auto & cell = grid_data[static_cast<size_t>(gy * info.width + gx)];
        if (val > cell) cell = val;
      }
    }
  }

  std::string map_frame_id(const nav_msgs::msg::OccupancyGrid & map) const
  {
    if (!map.header.frame_id.empty()) {
      return map.header.frame_id;
    }
    return "map";
  }

  bool get_start_pose(const std::string & frame_id, double & x, double & y)
  {
    try {
      const auto tf = tf_buffer_->lookupTransform(
        frame_id, robot_base_frame_, tf2::TimePointZero);
      x = tf.transform.translation.x;
      y = tf.transform.translation.y;
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
        "Failed to lookup TF %s -> %s: %s",
        frame_id.c_str(), robot_base_frame_.c_str(), ex.what());
      return false;
    }
  }

  std::pair<int, int> world_to_grid(
    double wx, double wy,
    const nav_msgs::msg::MapMetaData & info) const
  {
    double ox = info.origin.position.x;
    double oy = info.origin.position.y;
    double res = info.resolution;
    return {static_cast<int>((wx - ox) / res), static_cast<int>((wy - oy) / res)};
  }

  std::pair<double, double> grid_to_world(
    float gx, float gy,
    const nav_msgs::msg::MapMetaData & info) const
  {
    double ox = info.origin.position.x;
    double oy = info.origin.position.y;
    double res = info.resolution;
    return {ox + (gx + 0.5) * res, oy + (gy + 0.5) * res};
  }

  bool in_bounds(int gx, int gy, const nav_msgs::msg::MapMetaData & info) const
  {
    return gx >= 0 && gx < static_cast<int>(info.width) &&
           gy >= 0 && gy < static_cast<int>(info.height);
  }

  nav_msgs::msg::Path build_ros_path(
    const std::vector<std::pair<float, float>> & smoothed,
    const nav_msgs::msg::MapMetaData & info,
    const std::string & frame_id) const
  {
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp    = now();
    path_msg.header.frame_id = frame_id;

    std::vector<std::pair<double, double>> world_pts;
    for (auto & [gx, gy] : smoothed) {
      world_pts.push_back(grid_to_world(gx, gy, info));
    }

    double prev_yaw = 0.0;
    for (size_t i = 0; i < world_pts.size(); ++i) {
      auto & [wx, wy] = world_pts[i];

      if (i + 1 < world_pts.size()) {
        auto & [nx, ny] = world_pts[i + 1];
        prev_yaw = std::atan2(ny - wy, nx - wx);
      }

      geometry_msgs::msg::PoseStamped pose;
      pose.header            = path_msg.header;
      pose.pose.position.x   = wx;
      pose.pose.position.y   = wy;
      pose.pose.position.z   = 0.0;
      pose.pose.orientation  = yaw_to_quat(prev_yaw);
      path_msg.poses.push_back(pose);
    }

    return path_msg;
  }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr    map_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr    global_costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr    local_costmap_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                path_pub_;
  rclcpp::Service<nav_msgs::srv::GetPlan>::SharedPtr               plan_srv_;
  std::unique_ptr<tf2_ros::Buffer>                                 tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener>                      tf_listener_;

  std::mutex map_mutex_;
  nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;

  std::mutex costmap_mutex_;
  nav_msgs::msg::OccupancyGrid::SharedPtr current_costmap_;

  std::mutex local_costmap_mutex_;
  nav_msgs::msg::OccupancyGrid::SharedPtr current_local_costmap_;

  std::string robot_base_frame_;
};

}  // namespace machida_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<machida_navigation::GlobalPlannerNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
