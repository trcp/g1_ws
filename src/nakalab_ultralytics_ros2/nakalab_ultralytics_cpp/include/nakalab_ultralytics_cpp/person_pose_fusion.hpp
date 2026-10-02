#ifndef NAKALAB_ULTRALYTICS_CPP__PERSON_POSE_FUSION_HPP_
#define NAKALAB_ULTRALYTICS_CPP__PERSON_POSE_FUSION_HPP_

#include <cstddef>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nakalab_ultralytics_interfaces/msg/bounding_box.hpp"
#include "nakalab_ultralytics_interfaces/msg/point.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace nakalab_ultralytics_cpp
{
namespace fusion
{

struct ProjectedPoint
{
  double u;
  double v;
  geometry_msgs::msg::Point point;
};

geometry_msgs::msg::Point nan_point();

bool is_finite_point(const geometry_msgs::msg::Point & point);

bool stamps_within_tolerance(
  const builtin_interfaces::msg::Time & first,
  const builtin_interfaces::msg::Time & second,
  double tolerance_sec);

geometry_msgs::msg::Point project_depth_keypoint(
  const nakalab_ultralytics_interfaces::msg::Point & keypoint,
  const sensor_msgs::msg::Image & depth_image,
  const sensor_msgs::msg::CameraInfo & intrinsics,
  double max_depth_m);

std::vector<ProjectedPoint> project_pointcloud_to_image(
  const sensor_msgs::msg::PointCloud2 & pointcloud,
  const sensor_msgs::msg::CameraInfo & intrinsics,
  const geometry_msgs::msg::TransformStamped & cloud_to_camera,
  double max_depth_m);

geometry_msgs::msg::Point fuse_pointcloud_keypoint(
  const nakalab_ultralytics_interfaces::msg::Point & keypoint,
  const nakalab_ultralytics_interfaces::msg::BoundingBox & bounding_box,
  const std::vector<ProjectedPoint> & projected_points);

}  // namespace fusion
}  // namespace nakalab_ultralytics_cpp

#endif  // NAKALAB_ULTRALYTICS_CPP__PERSON_POSE_FUSION_HPP_
