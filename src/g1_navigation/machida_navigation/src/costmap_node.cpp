#include <cmath>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "machida_navigation/costmap.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace machida_navigation
{

// ============================================================
// GlobalCostmapNode: /map2d -> /global_costmap
// ============================================================
class GlobalCostmapNode : public rclcpp::Node
{
public:
  explicit GlobalCostmapNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("global_costmap_node", options)
  {
    declare_parameter("obstacle_threshold", 50);
    declare_parameter("footprint", std::string("0.4,0.3"));
    declare_parameter("clearance", 0.1);

    auto latched_qos = rclcpp::QoS(1).transient_local().reliable();

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map2d", latched_qos,
      std::bind(&GlobalCostmapNode::map_callback, this, std::placeholders::_1));

    costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/global_costmap", latched_qos);

    RCLCPP_INFO(get_logger(), "CostmapNode ready: /map2d -> /global_costmap");
  }

private:
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    const int obstacle_threshold = get_parameter("obstacle_threshold").as_int();
    const std::string footprint_str = get_parameter("footprint").as_string();
    const double clearance = get_parameter("clearance").as_double();

    const auto & info = msg->info;
    const float body_radius = footprint_radius(footprint_str);
    const int footprint_cells = static_cast<int>(body_radius / info.resolution);
    const int padding_cells   = static_cast<int>(clearance    / info.resolution);

    RCLCPP_INFO(get_logger(),
      "Map received (%dx%d, res=%.3f): body_radius=%.3fm (%d cells), clearance=%.2fm (%d cells)",
      info.width, info.height, info.resolution,
      body_radius, footprint_cells, clearance, padding_cells);

    auto grid_data = distance_transform_grid(
      msg->data,
      static_cast<int>(info.width),
      static_cast<int>(info.height),
      footprint_cells, padding_cells,
      obstacle_threshold);

    nav_msgs::msg::OccupancyGrid costmap_msg;
    costmap_msg.header.stamp    = now();
    costmap_msg.header.frame_id = msg->header.frame_id.empty() ? "map" : msg->header.frame_id;
    costmap_msg.info            = info;
    costmap_msg.data            = std::move(grid_data);
    costmap_pub_->publish(costmap_msg);

    RCLCPP_INFO(get_logger(), "Global costmap published");
  }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr    costmap_pub_;
};

// ============================================================
// LocalCostmapNode: PointCloud2 -> /local_costmap
// ============================================================
class LocalCostmapNode : public rclcpp::Node
{
public:
  explicit LocalCostmapNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("local_costmap_node", options)
  {
    declare_parameter("lidar_topic",          std::string("/points"));
    declare_parameter("local_costmap_frame",  std::string("odom"));
    declare_parameter("robot_base_frame",     std::string("base_footprint"));
    declare_parameter("resolution",           0.05);
    declare_parameter("local_width",          4.0);
    declare_parameter("local_height",         4.0);
    declare_parameter("min_obstacle_height",  0.1);
    declare_parameter("max_obstacle_height",  2.0);
    declare_parameter("min_sensor_range",     0.5);
    declare_parameter("footprint",            std::string("0.4,0.3"));
    declare_parameter("clearance",            0.1);
    declare_parameter("obstacle_threshold",   50);

    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      get_parameter("lidar_topic").as_string(),
      rclcpp::SensorDataQoS(),
      std::bind(&LocalCostmapNode::cloud_callback, this, std::placeholders::_1));

    costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/local_costmap", 1);

    RCLCPP_INFO(get_logger(), "LocalCostmapNode ready: %s -> /local_costmap",
      get_parameter("lidar_topic").as_string().c_str());
  }

private:
  // Rotate vector (lx, ly, lz) by quaternion (qx, qy, qz, qw) and translate by (tx, ty, tz).
  static void transform_point(
    double lx, double ly, double lz,
    double qx, double qy, double qz, double qw,
    double tx, double ty, double tz,
    double & rx, double & ry, double & rz)
  {
    rx = (1.0 - 2.0*(qy*qy + qz*qz))*lx
       + 2.0*(qx*qy - qz*qw)*ly
       + 2.0*(qx*qz + qy*qw)*lz + tx;
    ry = 2.0*(qx*qy + qz*qw)*lx
       + (1.0 - 2.0*(qx*qx + qz*qz))*ly
       + 2.0*(qy*qz - qx*qw)*lz + ty;
    rz = 2.0*(qx*qz - qy*qw)*lx
       + 2.0*(qy*qz + qx*qw)*ly
       + (1.0 - 2.0*(qx*qx + qy*qy))*lz + tz;
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {

    const std::string odom_frame = get_parameter("local_costmap_frame").as_string();
    const std::string base_frame = get_parameter("robot_base_frame").as_string();

    // Lookup robot pose and cloud-to-odom transform
    geometry_msgs::msg::TransformStamped robot_tf;
    geometry_msgs::msg::TransformStamped cloud_tf;
    try {
      robot_tf = tf_buffer_->lookupTransform(odom_frame, base_frame, tf2::TimePointZero);
      cloud_tf = tf_buffer_->lookupTransform(odom_frame, msg->header.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "TF lookup failed: %s", ex.what());
      return;
    }

    const double robot_x = robot_tf.transform.translation.x;
    const double robot_y = robot_tf.transform.translation.y;
    const double robot_z = robot_tf.transform.translation.z;

    const double cqx = cloud_tf.transform.rotation.x;
    const double cqy = cloud_tf.transform.rotation.y;
    const double cqz = cloud_tf.transform.rotation.z;
    const double cqw = cloud_tf.transform.rotation.w;
    const double ctx = cloud_tf.transform.translation.x;
    const double cty = cloud_tf.transform.translation.y;
    const double ctz = cloud_tf.transform.translation.z;

    const float resolution  = static_cast<float>(get_parameter("resolution").as_double());
    const float local_width = static_cast<float>(get_parameter("local_width").as_double());
    const float local_height= static_cast<float>(get_parameter("local_height").as_double());
    const float min_h          = static_cast<float>(get_parameter("min_obstacle_height").as_double());
    const float max_h          = static_cast<float>(get_parameter("max_obstacle_height").as_double());
    const float min_sensor_range_sq = static_cast<float>(
      get_parameter("min_sensor_range").as_double() *
      get_parameter("min_sensor_range").as_double());
    const std::string footprint_str = get_parameter("footprint").as_string();
    const float clearance   = static_cast<float>(get_parameter("clearance").as_double());
    const int obstacle_threshold = get_parameter("obstacle_threshold").as_int();

    const int grid_w = static_cast<int>(local_width  / resolution);
    const int grid_h = static_cast<int>(local_height / resolution);

    // Grid origin (bottom-left corner) centred on robot
    const float origin_x = static_cast<float>(robot_x) - local_width  * 0.5f;
    const float origin_y = static_cast<float>(robot_y) - local_height * 0.5f;

    // Find x/y/z byte offsets in PointCloud2
    int off_x = -1, off_y = -1, off_z = -1;
    for (const auto & field : msg->fields) {
      if (field.name == "x")      off_x = static_cast<int>(field.offset);
      else if (field.name == "y") off_y = static_cast<int>(field.offset);
      else if (field.name == "z") off_z = static_cast<int>(field.offset);
    }
    if (off_x < 0 || off_y < 0 || off_z < 0) {
      RCLCPP_WARN_ONCE(get_logger(), "PointCloud2 missing x/y/z fields");
      return;
    }

    // Project points onto the 2D obstacle list
    std::vector<std::pair<float, float>> obstacle_points;
    const uint8_t * data_ptr  = msg->data.data();
    const uint32_t point_step = msg->point_step;
    const uint32_t num_points = msg->width * msg->height;
    obstacle_points.reserve(num_points / 4);  // rough pre-alloc

    for (uint32_t i = 0; i < num_points; ++i) {
      const uint8_t * p = data_ptr + i * point_step;
      float lx, ly, lz;
      std::memcpy(&lx, p + off_x, sizeof(float));
      std::memcpy(&ly, p + off_y, sizeof(float));
      std::memcpy(&lz, p + off_z, sizeof(float));

      if (!std::isfinite(lx) || !std::isfinite(ly) || !std::isfinite(lz)) continue;

      // Reject points within the sensor dead zone (zero-returns and near-field noise)
      if (lx*lx + ly*ly + lz*lz < min_sensor_range_sq) continue;

      double rx, ry, rz;
      transform_point(lx, ly, lz, cqx, cqy, cqz, cqw, ctx, cty, ctz, rx, ry, rz);

      // Height filter relative to robot base
      const float dz = static_cast<float>(rz - robot_z);
      if (dz < min_h || dz > max_h) continue;

      obstacle_points.emplace_back(static_cast<float>(rx), static_cast<float>(ry));
    }

    // Build raw grid and inflate
    auto raw_grid = build_obstacle_grid(grid_w, grid_h, resolution, origin_x, origin_y,
      obstacle_points);

    const float body_radius   = footprint_radius(footprint_str);
    const int footprint_cells = static_cast<int>(body_radius / resolution);
    const int padding_cells   = static_cast<int>(clearance   / resolution);

    auto inflated = distance_transform_grid(raw_grid, grid_w, grid_h,
      footprint_cells, padding_cells, obstacle_threshold);

    // Publish
    nav_msgs::msg::OccupancyGrid out;
    out.header.stamp              = now();
    out.header.frame_id           = odom_frame;
    out.info.resolution           = resolution;
    out.info.width                = static_cast<uint32_t>(grid_w);
    out.info.height               = static_cast<uint32_t>(grid_h);
    out.info.origin.position.x    = static_cast<double>(origin_x);
    out.info.origin.position.y    = static_cast<double>(origin_y);
    out.info.origin.position.z    = 0.0;
    out.info.origin.orientation.w = 1.0;
    out.data                      = std::move(inflated);
    costmap_pub_->publish(out);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr     costmap_pub_;
  std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace machida_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
#ifdef RUN_LOCAL_COSTMAP
  rclcpp::spin(std::make_shared<machida_navigation::LocalCostmapNode>());
#else
  rclcpp::spin(std::make_shared<machida_navigation::GlobalCostmapNode>());
#endif
  rclcpp::shutdown();
  return 0;
}
