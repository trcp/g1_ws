#include "nakalab_ultralytics_cpp/person_pose_fusion.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nakalab_ultralytics_cpp
{
namespace fusion
{
namespace
{

bool is_zero_stamp(const builtin_interfaces::msg::Time & stamp)
{
  return stamp.sec == 0 && stamp.nanosec == 0;
}

bool is_valid_image_pixel(
  double x,
  double y,
  const sensor_msgs::msg::Image & image)
{
  return std::isfinite(x) && std::isfinite(y) && x >= 0.0 && y >= 0.0 &&
         x < static_cast<double>(image.width) && y < static_cast<double>(image.height);
}

bool is_valid_camera_model(const sensor_msgs::msg::CameraInfo & intrinsics)
{
  return intrinsics.width > 0 && intrinsics.height > 0 &&
         std::isfinite(intrinsics.k[0]) && std::isfinite(intrinsics.k[4]) &&
         intrinsics.k[0] > 0.0 && intrinsics.k[4] > 0.0;
}

std::optional<double> depth_at(
  const sensor_msgs::msg::Image & image,
  double x,
  double y)
{
  const int u = static_cast<int>(std::lround(x));
  const int v = static_cast<int>(std::lround(y));
  if (u < 0 || v < 0 || u >= static_cast<int>(image.width) ||
    v >= static_cast<int>(image.height))
  {
    return std::nullopt;
  }

  if (image.encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
    image.encoding == sensor_msgs::image_encodings::MONO16)
  {
    const auto offset = static_cast<std::size_t>(v) * image.step +
      static_cast<std::size_t>(u) * sizeof(uint16_t);
    if (offset + sizeof(uint16_t) > image.data.size()) {
      return std::nullopt;
    }

    uint16_t raw = 0;
    std::memcpy(&raw, image.data.data() + offset, sizeof(raw));
    if (raw == 0) {
      return std::nullopt;
    }
    return static_cast<double>(raw) * 0.001;
  }

  if (image.encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    const auto offset = static_cast<std::size_t>(v) * image.step +
      static_cast<std::size_t>(u) * sizeof(float);
    if (offset + sizeof(float) > image.data.size()) {
      return std::nullopt;
    }

    float raw = 0.0F;
    std::memcpy(&raw, image.data.data() + offset, sizeof(raw));
    if (!std::isfinite(raw) || raw <= 0.0F) {
      return std::nullopt;
    }
    return static_cast<double>(raw);
  }

  throw std::invalid_argument("Unsupported depth image encoding: " + image.encoding);
}

bool has_float32_xyz_fields(const sensor_msgs::msg::PointCloud2 & pointcloud)
{
  bool has_x = false;
  bool has_y = false;
  bool has_z = false;
  for (const auto & field : pointcloud.fields) {
    if (field.datatype != sensor_msgs::msg::PointField::FLOAT32) {
      continue;
    }
    has_x = has_x || field.name == "x";
    has_y = has_y || field.name == "y";
    has_z = has_z || field.name == "z";
  }
  return has_x && has_y && has_z;
}

}  // namespace

geometry_msgs::msg::Point nan_point()
{
  geometry_msgs::msg::Point point;
  point.x = std::numeric_limits<double>::quiet_NaN();
  point.y = std::numeric_limits<double>::quiet_NaN();
  point.z = std::numeric_limits<double>::quiet_NaN();
  return point;
}

bool is_finite_point(const geometry_msgs::msg::Point & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

bool stamps_within_tolerance(
  const builtin_interfaces::msg::Time & first,
  const builtin_interfaces::msg::Time & second,
  double tolerance_sec)
{
  if (tolerance_sec < 0.0 || is_zero_stamp(first) || is_zero_stamp(second)) {
    return false;
  }

  constexpr int64_t kNanosecondsPerSecond = 1000000000LL;
  const int64_t first_ns =
    static_cast<int64_t>(first.sec) * kNanosecondsPerSecond + first.nanosec;
  const int64_t second_ns =
    static_cast<int64_t>(second.sec) * kNanosecondsPerSecond + second.nanosec;
  const int64_t tolerance_ns =
    static_cast<int64_t>(std::llround(tolerance_sec * kNanosecondsPerSecond));
  return std::abs(first_ns - second_ns) <= tolerance_ns;
}

geometry_msgs::msg::Point project_depth_keypoint(
  const nakalab_ultralytics_interfaces::msg::Point & keypoint,
  const sensor_msgs::msg::Image & depth_image,
  const sensor_msgs::msg::CameraInfo & intrinsics,
  double max_depth_m)
{
  auto point = nan_point();
  if (keypoint.confidence <= 0.0F ||
    !is_valid_image_pixel(keypoint.x, keypoint.y, depth_image) ||
    !is_valid_camera_model(intrinsics))
  {
    return point;
  }

  const auto depth = depth_at(depth_image, keypoint.x, keypoint.y);
  if (!depth || *depth <= 0.0 || *depth > max_depth_m) {
    return point;
  }

  point.z = *depth;
  point.x = (static_cast<double>(keypoint.x) - intrinsics.k[2]) * point.z /
    intrinsics.k[0];
  point.y = (static_cast<double>(keypoint.y) - intrinsics.k[5]) * point.z /
    intrinsics.k[4];
  return point;
}

std::vector<ProjectedPoint> project_pointcloud_to_image(
  const sensor_msgs::msg::PointCloud2 & pointcloud,
  const sensor_msgs::msg::CameraInfo & intrinsics,
  const geometry_msgs::msg::TransformStamped & cloud_to_camera,
  double max_depth_m)
{
  if (!is_valid_camera_model(intrinsics)) {
    throw std::invalid_argument("Color CameraInfo has invalid dimensions or intrinsics");
  }
  if (!has_float32_xyz_fields(pointcloud)) {
    throw std::invalid_argument("PointCloud2 must contain FLOAT32 x/y/z fields");
  }

  std::vector<ProjectedPoint> projected_points;
  projected_points.reserve(
    static_cast<std::size_t>(pointcloud.width) * pointcloud.height);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(pointcloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(pointcloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(pointcloud, "z");
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) {
      continue;
    }

    geometry_msgs::msg::PointStamped source_point;
    source_point.header = pointcloud.header;
    source_point.point.x = *iter_x;
    source_point.point.y = *iter_y;
    source_point.point.z = *iter_z;

    geometry_msgs::msg::PointStamped camera_point;
    tf2::doTransform(source_point, camera_point, cloud_to_camera);
    const auto & point = camera_point.point;
    if (!is_finite_point(point) || point.z <= 0.0 || point.z > max_depth_m) {
      continue;
    }

    const double u = intrinsics.k[0] * point.x / point.z + intrinsics.k[2];
    const double v = intrinsics.k[4] * point.y / point.z + intrinsics.k[5];
    if (!std::isfinite(u) || !std::isfinite(v) ||
      u < 0.0 || v < 0.0 ||
      u >= static_cast<double>(intrinsics.width) ||
      v >= static_cast<double>(intrinsics.height))
    {
      continue;
    }
    projected_points.push_back({u, v, point});
  }

  return projected_points;
}

geometry_msgs::msg::Point fuse_pointcloud_keypoint(
  const nakalab_ultralytics_interfaces::msg::Point & keypoint,
  const nakalab_ultralytics_interfaces::msg::BoundingBox & bounding_box,
  const std::vector<ProjectedPoint> & projected_points)
{
  if (keypoint.confidence <= 0.0F ||
    !std::isfinite(keypoint.x) || !std::isfinite(keypoint.y))
  {
    return nan_point();
  }

  const double min_u = std::min(
    static_cast<double>(bounding_box.top_left.x),
    static_cast<double>(bounding_box.bottom_right.x));
  const double max_u = std::max(
    static_cast<double>(bounding_box.top_left.x),
    static_cast<double>(bounding_box.bottom_right.x));
  const double min_v = std::min(
    static_cast<double>(bounding_box.top_left.y),
    static_cast<double>(bounding_box.bottom_right.y));
  const double max_v = std::max(
    static_cast<double>(bounding_box.top_left.y),
    static_cast<double>(bounding_box.bottom_right.y));
  if (!std::isfinite(min_u) || !std::isfinite(max_u) ||
    !std::isfinite(min_v) || !std::isfinite(max_v))
  {
    return nan_point();
  }

  const ProjectedPoint * nearest_point = nullptr;
  double nearest_distance_sq = std::numeric_limits<double>::infinity();
  for (const auto & projected : projected_points) {
    if (projected.u < min_u || projected.u > max_u ||
      projected.v < min_v || projected.v > max_v ||
      !is_finite_point(projected.point))
    {
      continue;
    }

    const double du = projected.u - keypoint.x;
    const double dv = projected.v - keypoint.y;
    const double distance_sq = du * du + dv * dv;
    if (distance_sq < nearest_distance_sq ||
      (distance_sq == nearest_distance_sq &&
      nearest_point != nullptr && projected.point.z < nearest_point->point.z))
    {
      nearest_point = &projected;
      nearest_distance_sq = distance_sq;
    }
  }
  if (nearest_point == nullptr) {
    return nan_point();
  }
  return nearest_point->point;
}

}  // namespace fusion
}  // namespace nakalab_ultralytics_cpp
