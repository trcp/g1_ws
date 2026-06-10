#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace machida_navigation
{

struct Point2D { double x{0.0}; double y{0.0}; };

static double yaw_from_quat(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

static double normalize_angle(double a)
{
  while (a >  M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

class DWALocalPlanner : public rclcpp::Node
{
public:
  explicit DWALocalPlanner(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("dwa_local_planner", options)
  {
    // ---- same interface parameters as pure pursuit ----
    declare_parameter("path_topic",             std::string("/execute_path_plan"));
    declare_parameter("cmd_vel_topic",          std::string("/cmd_vel"));
    declare_parameter("execute_topic",          std::string("/execute_local_planner"));
    declare_parameter("map_frame",              std::string("map"));
    declare_parameter("robot_base_frame",       std::string("base_footprint"));
    declare_parameter("control_frequency",      20.0);
    declare_parameter("goal_tolerance",         0.15);
    declare_parameter("goal_yaw_tolerance",     0.05);
    declare_parameter("max_path_deviation",     0.5);
    declare_parameter("min_linear_velocity",    0.2);
    declare_parameter("max_linear_velocity",    0.5);
    declare_parameter("min_angular_velocity",   0.3);
    declare_parameter("max_angular_velocity",   1.0);
    declare_parameter("max_linear_acceleration",  0.5);
    declare_parameter("max_angular_acceleration", 2.0);
    declare_parameter("holonomic",              false);
    declare_parameter("slowdown_distance",      0.6);
    declare_parameter("lookahead_distance",     0.5);
    // ---- DWA-specific parameters ----
    declare_parameter("sim_time",       1.5);    // trajectory simulation horizon [s]
    declare_parameter("sim_granularity", 0.05);  // simulation time step [s]
    declare_parameter("vx_samples",     10);     // forward velocity samples
    declare_parameter("vy_samples",     5);      // lateral velocity samples (holonomic only)
    declare_parameter("vth_samples",    20);     // angular velocity samples
    declare_parameter("heading_bias",   24.0);   // weight for heading score
    declare_parameter("path_bias",      32.0);   // weight for path following score
    declare_parameter("speed_bias",      6.0);   // weight for speed score

    path_topic_       = get_parameter("path_topic").as_string();
    cmd_vel_topic_    = get_parameter("cmd_vel_topic").as_string();
    map_frame_        = get_parameter("map_frame").as_string();
    robot_base_frame_ = get_parameter("robot_base_frame").as_string();

    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      path_topic_, rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&DWALocalPlanner::path_callback, this, std::placeholders::_1));

    execute_sub_ = create_subscription<std_msgs::msg::Bool>(
      get_parameter("execute_topic").as_string(), 1,
      std::bind(&DWALocalPlanner::execute_callback, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

    const double freq = std::max(1.0, get_parameter("control_frequency").as_double());
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / freq),
      std::bind(&DWALocalPlanner::control_loop, this));

    RCLCPP_INFO(get_logger(),
      "DWALocalPlanner ready: path=%s cmd_vel=%s execute=%s TF %s->%s mode=%s",
      path_topic_.c_str(), cmd_vel_topic_.c_str(),
      get_parameter("execute_topic").as_string().c_str(),
      map_frame_.c_str(), robot_base_frame_.c_str(),
      get_parameter("holonomic").as_bool() ? "holonomic" : "differential");
  }

private:
  struct State { double x{0.0}; double y{0.0}; double theta{0.0}; };
  struct Vel   { double vx{0.0}; double vy{0.0}; double w{0.0}; };

  // ---- ROS callbacks ----

  void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    path_.clear();
    path_.reserve(msg->poses.size());
    for (const auto & ps : msg->poses)
      path_.push_back({ps.pose.position.x, ps.pose.position.y});
    if (!msg->poses.empty())
      goal_yaw_ = yaw_from_quat(msg->poses.back().pose.orientation);
    path_frame_id_    = msg->header.frame_id;
    nearest_index_    = 0;
    reached_goal_     = false;
    rotating_to_goal_ = false;
    RCLCPP_INFO(get_logger(), "Path received: %zu poses, goal_yaw=%.3f rad",
      path_.size(), goal_yaw_);
  }

  void execute_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(exec_mutex_);
    executing_ = msg->data;
  }

  // ---- DWA helpers ----

  // Simulate robot kinematics for sim_time, returning final state.
  // traj_pts sampled uniformly across simulation for scoring (keeps cost bounded).
  State simulate(const State & s0, const Vel & vel,
                 double sim_time, double dt, bool holo,
                 std::vector<State> * sampled = nullptr) const
  {
    const int steps = std::max(1, static_cast<int>(sim_time / dt));
    const int sample_stride = std::max(1, steps / 8);  // at most ~8 samples
    State s = s0;
    for (int i = 0; i < steps; ++i) {
      if (holo) {
        s.x += (vel.vx * std::cos(s.theta) - vel.vy * std::sin(s.theta)) * dt;
        s.y += (vel.vx * std::sin(s.theta) + vel.vy * std::cos(s.theta)) * dt;
      } else {
        s.x += vel.vx * std::cos(s.theta) * dt;
        s.y += vel.vx * std::sin(s.theta) * dt;
      }
      s.theta += vel.w * dt;
      if (sampled && (i % sample_stride == 0)) sampled->push_back(s);
    }
    return s;
  }

  size_t find_nearest(const std::vector<Point2D> & path, double x, double y,
                      size_t from) const
  {
    size_t best = std::min(from, path.size() - 1);
    double best_d = std::hypot(path[best].x - x, path[best].y - y);
    for (size_t i = best + 1; i < path.size(); ++i) {
      const double d = std::hypot(path[i].x - x, path[i].y - y);
      if (d < best_d) { best = i; best_d = d; }
    }
    return best;
  }

  // Returns a local target point ahead of near_idx by lookahead along the path.
  Point2D local_target(const std::vector<Point2D> & path, double px, double py,
                       size_t near_idx, double lookahead) const
  {
    double acc = std::hypot(path[near_idx].x - px, path[near_idx].y - py);
    for (size_t i = near_idx + 1; i < path.size(); ++i) {
      acc += std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
      if (acc >= lookahead) return path[i];
    }
    return path.back();
  }

  // Min distance from trajectory sample points to path (search window near near_idx).
  double traj_path_dist(const std::vector<State> & pts, const std::vector<Point2D> & path,
                        size_t near_idx) const
  {
    const size_t wend = std::min(path.size(), near_idx + 60);
    double min_d = std::numeric_limits<double>::max();
    for (const auto & s : pts) {
      for (size_t j = near_idx; j < wend; ++j) {
        const double d = std::hypot(path[j].x - s.x, path[j].y - s.y);
        if (d < min_d) min_d = d;
      }
    }
    return min_d;
  }

  // Heading error of end state toward the target [rad] — lower is better.
  double heading_error(const State & end, const Point2D & target) const
  {
    const double desired = std::atan2(target.y - end.y, target.x - end.x);
    return std::abs(normalize_angle(desired - end.theta));
  }

  // In-place normalize vector to [0,1]; invert=true means lower raw → higher norm.
  static void normalize(std::vector<double> & v, bool invert)
  {
    const double mn = *std::min_element(v.begin(), v.end());
    const double mx = *std::max_element(v.begin(), v.end());
    const double rng = mx - mn;
    for (auto & x : v) {
      x = (rng > 1e-9) ? (x - mn) / rng : 1.0;
      if (invert) x = 1.0 - x;
    }
  }

  static std::vector<double> linspace(int n, double lo, double hi)
  {
    std::vector<double> v(n);
    for (int i = 0; i < n; ++i)
      v[i] = (n > 1) ? lo + (hi - lo) * i / (n - 1) : lo;
    return v;
  }

  // ---- control loop ----

  void control_loop()
  {
    {
      std::lock_guard<std::mutex> lk(mutex_);
      if (reached_goal_) return;
    }
    {
      std::lock_guard<std::mutex> lk(exec_mutex_);
      if (!executing_) {
        if (prev_executing_) { publish_stop(); prev_executing_ = false; }
        return;
      }
      prev_executing_ = true;
    }

    // --- get robot pose from TF ---
    State pose{};
    bool has_pose = false;
    try {
      const auto tf = tf_buffer_->lookupTransform(
        map_frame_, robot_base_frame_, tf2::TimePointZero);
      pose.x     = tf.transform.translation.x;
      pose.y     = tf.transform.translation.y;
      pose.theta = yaw_from_quat(tf.transform.rotation);
      has_pose   = true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "TF %s->%s failed: %s", map_frame_.c_str(), robot_base_frame_.c_str(), ex.what());
    }

    std::vector<Point2D> path;
    double goal_yaw;
    bool rotating;
    size_t near_idx;
    {
      std::lock_guard<std::mutex> lk(mutex_);
      path     = path_;
      goal_yaw = goal_yaw_;
      rotating = rotating_to_goal_;
      near_idx = nearest_index_;
    }

    if (!has_pose || path.empty()) {
      publish_stop();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Waiting: pose=%s path=%zu", has_pose ? "ok" : "missing", path.size());
      return;
    }

    const Point2D & goal     = path.back();
    const double dist_goal   = std::hypot(goal.x - pose.x, goal.y - pose.y);
    const double goal_tol    = get_parameter("goal_tolerance").as_double();
    const double yaw_tol     = get_parameter("goal_yaw_tolerance").as_double();
    const double ctrl_dt     = 1.0 / std::max(1.0, get_parameter("control_frequency").as_double());

    // --- goal rotation phase (same as pure pursuit) ---
    if (dist_goal <= goal_tol || rotating) {
      if (!rotating) prev_vx_ = prev_vy_ = prev_w_ = 0.0;

      double err = normalize_angle(goal_yaw - pose.theta);
      if (std::abs(err) <= yaw_tol) {
        publish_stop();
        std::lock_guard<std::mutex> lk(mutex_);
        if (!reached_goal_) {
          RCLCPP_INFO(get_logger(), "Goal reached (dist=%.3f m, yaw_err=%.3f rad)",
            dist_goal, err);
        }
        reached_goal_ = true; rotating_to_goal_ = false;
        return;
      }

      const double max_w     = get_parameter("max_angular_velocity").as_double();
      const double max_w_acc = get_parameter("max_angular_acceleration").as_double();
      const double min_w     = get_parameter("min_angular_velocity").as_double();
      double w = std::clamp(err, -max_w, max_w);
      w = std::clamp(w, prev_w_ - max_w_acc * ctrl_dt, prev_w_ + max_w_acc * ctrl_dt);
      if (w > 0.0 && w < min_w) w = min_w;
      else if (w < 0.0 && w > -min_w) w = -min_w;

      geometry_msgs::msg::Twist cmd;
      cmd.angular.z = w;
      prev_vx_ = prev_vy_ = 0.0; prev_w_ = w;
      cmd_pub_->publish(cmd);

      std::lock_guard<std::mutex> lk(mutex_);
      rotating_to_goal_ = true;
      return;
    }

    // --- update nearest index and check path deviation ---
    near_idx = find_nearest(path, pose.x, pose.y, near_idx);

    const double dev     = std::hypot(path[near_idx].x - pose.x, path[near_idx].y - pose.y);
    const double max_dev = get_parameter("max_path_deviation").as_double();
    if (max_dev > 0.0 && dev > max_dev) {
      publish_stop();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "Path deviation %.3f m exceeds %.3f m, stopping", dev, max_dev);
      return;
    }

    // --- local target for heading scoring ---
    const Point2D target = local_target(
      path, pose.x, pose.y, near_idx,
      get_parameter("lookahead_distance").as_double());

    // --- DWA parameters ---
    const bool   holo    = get_parameter("holonomic").as_bool();
    const double max_v   = desired_max_v(dist_goal);
    const double min_v   = get_parameter("min_linear_velocity").as_double();
    const double max_w   = get_parameter("max_angular_velocity").as_double();
    const double max_av  = get_parameter("max_linear_acceleration").as_double();
    const double max_aw  = get_parameter("max_angular_acceleration").as_double();
    const double sim_t   = get_parameter("sim_time").as_double();
    const double sim_dt  = get_parameter("sim_granularity").as_double();
    const int    vx_n    = std::max(2, (int)get_parameter("vx_samples").as_int());
    const int    vy_n    = std::max(2, (int)get_parameter("vy_samples").as_int());
    const int    vw_n    = std::max(2, (int)get_parameter("vth_samples").as_int());

    // Dynamic window: velocities reachable within sim_time * 0.3 given acceleration limits.
    // Using a fraction of sim_time (not ctrl_dt) gives a wider window from rest.
    const double win_dt = std::max(ctrl_dt, sim_t * 0.3);
    const double vx_lo  = std::max(0.0,   prev_vx_ - max_av * win_dt);
    const double vx_hi  = std::min(max_v, prev_vx_ + max_av * win_dt);
    const double vy_lo  = holo ? std::max(-max_v, prev_vy_ - max_av * win_dt) : 0.0;
    const double vy_hi  = holo ? std::min( max_v, prev_vy_ + max_av * win_dt) : 0.0;
    const double w_lo   = std::max(-max_w, prev_w_ - max_aw * win_dt);
    const double w_hi   = std::min( max_w, prev_w_ + max_aw * win_dt);

    const auto vx_vals  = linspace(vx_n, vx_lo, vx_hi);
    const auto vy_vals  = holo ? linspace(vy_n, vy_lo, vy_hi) : std::vector<double>{0.0};
    const auto vw_vals  = linspace(vw_n, w_lo, w_hi);

    // --- evaluate all candidates ---
    struct Candidate { Vel vel; double h_err; double p_dist; double spd; };
    std::vector<Candidate> cands;
    cands.reserve(vx_n * (holo ? vy_n : 1) * vw_n);

    for (double vx : vx_vals) {
      for (double vy : vy_vals) {
        for (double w : vw_vals) {
          std::vector<State> pts;
          const State end = simulate(pose, {vx, vy, w}, sim_t, sim_dt, holo, &pts);
          pts.push_back(end);
          cands.push_back({
            {vx, vy, w},
            heading_error(end, target),
            traj_path_dist(pts, path, near_idx),
            std::hypot(vx, vy)
          });
        }
      }
    }

    if (cands.empty()) { publish_stop(); return; }

    // --- normalize scores and pick best ---
    const size_t N = cands.size();
    std::vector<double> h_s(N), p_s(N), sp_s(N);
    for (size_t i = 0; i < N; ++i) {
      h_s[i]  = cands[i].h_err;
      p_s[i]  = cands[i].p_dist;
      sp_s[i] = cands[i].spd;
    }
    normalize(h_s,  true);   // lower heading error → higher score
    normalize(p_s,  true);   // lower path dist    → higher score
    normalize(sp_s, false);  // higher speed        → higher score

    const double alpha = get_parameter("heading_bias").as_double();
    const double beta  = get_parameter("path_bias").as_double();
    const double gamma = get_parameter("speed_bias").as_double();

    size_t best_i = 0;
    double best_score = -std::numeric_limits<double>::max();
    for (size_t i = 0; i < N; ++i) {
      const double score = alpha * h_s[i] + beta * p_s[i] + gamma * sp_s[i];
      if (score > best_score) { best_score = score; best_i = i; }
    }

    Vel best = cands[best_i].vel;

    // Apply min velocity dead-band (robot unresponsive below this threshold)
    const double spd = std::hypot(best.vx, best.vy);
    if (spd > 1e-6 && spd < min_v) {
      const double scale = min_v / spd;
      best.vx *= scale;
      best.vy *= scale;
    }
    if (spd < 1e-3) {
      const double min_w_p = get_parameter("min_angular_velocity").as_double();
      if      (best.w > 0.0 && best.w <  min_w_p) best.w =  min_w_p;
      else if (best.w < 0.0 && best.w > -min_w_p) best.w = -min_w_p;
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x  = best.vx;
    cmd.linear.y  = best.vy;
    cmd.angular.z = best.w;
    prev_vx_ = best.vx; prev_vy_ = best.vy; prev_w_ = best.w;
    cmd_pub_->publish(cmd);

    {
      std::lock_guard<std::mutex> lk(mutex_);
      nearest_index_ = near_idx;
    }
  }

  double desired_max_v(double dist_to_goal) const
  {
    const double mx = get_parameter("max_linear_velocity").as_double();
    const double sd = get_parameter("slowdown_distance").as_double();
    if (sd <= 1e-6) return mx;
    return mx * std::clamp(dist_to_goal / sd, 0.0, 1.0);
  }

  void publish_stop()
  {
    cmd_pub_->publish(geometry_msgs::msg::Twist{});
    prev_vx_ = prev_vy_ = prev_w_ = 0.0;
  }

  // ---- ROS handles ----
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr    path_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr    execute_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr                            timer_;
  std::unique_ptr<tf2_ros::Buffer>                        tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener>             tf_listener_;

  // ---- state ----
  std::mutex mutex_;
  std::vector<Point2D> path_;
  bool   reached_goal_{false};
  bool   rotating_to_goal_{false};
  double goal_yaw_{0.0};
  size_t nearest_index_{0};
  std::string path_frame_id_;
  std::string path_topic_;
  std::string cmd_vel_topic_;
  std::string map_frame_;
  std::string robot_base_frame_;

  std::mutex exec_mutex_;
  bool executing_{false};
  bool prev_executing_{false};

  double prev_vx_{0.0};
  double prev_vy_{0.0};
  double prev_w_{0.0};
};

}  // namespace machida_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<machida_navigation::DWALocalPlanner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
