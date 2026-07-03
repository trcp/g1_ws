#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "message_filters/subscriber.h"
#include "nakalab_ultralytics_interfaces/msg/person_pose2_d_array.hpp"
#include "nakalab_ultralytics_interfaces/msg/person_pose3_d.hpp"
#include "nakalab_ultralytics_interfaces/msg/person_pose3_d_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace
{

using nakalab_ultralytics_interfaces::msg::PersonPose2DArray;
using nakalab_ultralytics_interfaces::msg::PersonPose3D;
using nakalab_ultralytics_interfaces::msg::PersonPose3DArray;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseArray;
using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::Image;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

constexpr std::size_t kKeypointCount = 16;
constexpr const char * kInputPosesTopic = "/nu_ros2/person_pose_2d";
constexpr const char * kDepthImageTopic = "/depth_image";
constexpr const char * kColorCameraInfoTopic = "/color_camera_info";
constexpr const char * kDepthCameraInfoTopic = "/depth_camera_info";
constexpr const char * kOutputPosesTopic = "/nu_ros2/person_pose_3d";
constexpr const char * kMarkerTopic = "/nu_ros2/detect_poses";
constexpr const char * kPoseArrayTopic = "/nu_ros2/poses";
constexpr std::array<std::array<std::size_t, 2>, 15> kCocoBonePairs{{
  {{0, 1}},    // nose - left eye
  {{0, 2}},    // nose - right eye
  {{1, 3}},    // left eye - left ear
  {{2, 4}},    // right eye - right ear
  {{5, 6}},    // left shoulder - right shoulder
  {{5, 7}},    // left shoulder - left elbow
  {{7, 9}},    // left elbow - left wrist
  {{6, 8}},    // right shoulder - right elbow
  {{8, 10}},   // right elbow - right wrist
  {{5, 11}},   // left shoulder - left hip
  {{6, 12}},   // right shoulder - right hip
  {{11, 12}},  // left hip - right hip
  {{11, 13}},  // left hip - left knee
  {{13, 15}},  // left knee - left ankle
  {{12, 14}},  // right hip - right knee
}};

double quiet_nan()
{
  return std::numeric_limits<double>::quiet_NaN();
}

bool is_valid_pixel(double x, double y, const Image & image)
{
  return std::isfinite(x) && std::isfinite(y) && x >= 0.0 && y >= 0.0 &&
         x < static_cast<double>(image.width) && y < static_cast<double>(image.height);
}

bool is_finite_point(const geometry_msgs::msg::Point & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

bool is_zero_stamp(const builtin_interfaces::msg::Time & stamp)
{
  return stamp.sec == 0 && stamp.nanosec == 0;
}

double squared_distance(
  const geometry_msgs::msg::Point & first,
  const geometry_msgs::msg::Point & second)
{
  const double dx = first.x - second.x;
  const double dy = first.y - second.y;
  const double dz = first.z - second.z;
  return dx * dx + dy * dy + dz * dz;
}

std::array<float, 3> color_for_pose(std::size_t index)
{
  constexpr std::array<std::array<float, 3>, 10> kPoseColors{{
    {{0.95F, 0.20F, 0.20F}},
    {{0.10F, 0.65F, 1.00F}},
    {{0.20F, 0.85F, 0.35F}},
    {{1.00F, 0.70F, 0.10F}},
    {{0.75F, 0.35F, 1.00F}},
    {{0.00F, 0.85F, 0.80F}},
    {{1.00F, 0.45F, 0.70F}},
    {{0.55F, 0.80F, 0.10F}},
    {{0.25F, 0.35F, 1.00F}},
    {{1.00F, 0.95F, 0.25F}},
  }};
  return kPoseColors[index % kPoseColors.size()];
}

}  // namespace

class PersonPose3DNode : public rclcpp::Node
{
public:
  PersonPose3DNode()
  : Node("person_pose_3d")
  {
    max_depth_m_ = declare_parameter<double>("max_depth_m", 10.0);
    marker_scale_m_ = declare_parameter<double>("marker_scale_m", 0.04);
    dense_cluster_radius_m_ = declare_parameter<double>("dense_cluster_radius_m", 0.35);
    ref_frame_ = declare_parameter<std::string>("ref_frame", "");
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    pose_pub_ = create_publisher<PersonPose3DArray>(kOutputPosesTopic, 10);
    marker_pub_ = create_publisher<MarkerArray>(kMarkerTopic, 10);
    pose_array_pub_ = create_publisher<PoseArray>(kPoseArrayTopic, 10);

    poses_2d_sub_ = create_subscription<PersonPose2DArray>(
      kInputPosesTopic, 10,
      [this](PersonPose2DArray::SharedPtr msg) {on_poses_2d(msg);});

    const auto info_qos = rclcpp::SensorDataQoS();
    color_info_sub_ = create_subscription<CameraInfo>(
      kColorCameraInfoTopic, info_qos,
      [this](CameraInfo::SharedPtr msg) {on_color_camera_info(msg);});
    depth_info_sub_ = create_subscription<CameraInfo>(
      kDepthCameraInfoTopic, info_qos,
      [this](CameraInfo::SharedPtr msg) {on_depth_camera_info(msg);});

    RCLCPP_INFO(
      get_logger(),
      "Waiting for camera info: color='%s', depth='%s'",
      kColorCameraInfoTopic, kDepthCameraInfoTopic);
    RCLCPP_INFO(
      get_logger(),
      "Output reference frame: '%s'",
      ref_frame_.empty() ? "<camera frame>" : ref_frame_.c_str());
  }

private:
  void on_color_camera_info(const CameraInfo::SharedPtr msg)
  {
    if (!color_camera_info_) {
      color_camera_info_ = msg;
      RCLCPP_INFO(get_logger(), "Received color camera info.");
      maybe_start_synchronized_subscribers();
    }
  }

  void on_depth_camera_info(const CameraInfo::SharedPtr msg)
  {
    if (!depth_camera_info_) {
      depth_camera_info_ = msg;
      RCLCPP_INFO(get_logger(), "Received depth camera info.");
      maybe_start_synchronized_subscribers();
    }
  }

  void on_poses_2d(const PersonPose2DArray::SharedPtr msg)
  {
    latest_poses_2d_ = msg;
  }

  void maybe_start_synchronized_subscribers()
  {
    if (!color_camera_info_ || !depth_camera_info_ || depth_sub_) {
      return;
    }

    color_info_sub_.reset();
    depth_info_sub_.reset();

    depth_sub_ = std::make_unique<message_filters::Subscriber<Image>>(
      this, kDepthImageTopic, rmw_qos_profile_sensor_data);
    depth_sub_->registerCallback(
      std::bind(&PersonPose3DNode::on_depth_image, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Started subscriptions: poses='%s', depth='%s'",
      kInputPosesTopic, kDepthImageTopic);
  }

  void on_depth_image(const Image::ConstSharedPtr depth_image)
  {
    if (!latest_poses_2d_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Waiting for PersonPose2DArray on '%s'.", kInputPosesTopic);
      return;
    }

    const CameraInfo & intrinsics = color_camera_info_ ? *color_camera_info_ : *depth_camera_info_;

    PersonPose3DArray poses_3d;
    poses_3d.header = latest_poses_2d_->header;
    if (poses_3d.header.frame_id.empty() && depth_camera_info_) {
      poses_3d.header.frame_id = depth_camera_info_->header.frame_id;
    } else if (depth_camera_info_ && !depth_camera_info_->header.frame_id.empty()) {
      poses_3d.header.frame_id = depth_camera_info_->header.frame_id;
    }
    if (!is_zero_stamp(depth_image->header.stamp)) {
      poses_3d.header.stamp = depth_image->header.stamp;
    } else if (is_zero_stamp(poses_3d.header.stamp)) {
      poses_3d.header.stamp = depth_image->header.stamp;
    }

    poses_3d.poses.reserve(latest_poses_2d_->poses.size());
    for (const auto & pose_2d : latest_poses_2d_->poses) {
      PersonPose3D pose_3d;
      pose_3d.bounding_box = pose_2d.bounding_box;
      pose_3d.confidence = pose_2d.confidence;

      for (std::size_t i = 0; i < kKeypointCount; ++i) {
        pose_3d.keypoints[i] = project_keypoint(pose_2d.keypoints[i], *depth_image, intrinsics);
      }

      poses_3d.poses.push_back(pose_3d);
    }

    auto output_poses = transform_poses_to_ref_frame(poses_3d);
    if (!output_poses) {
      return;
    }

    pose_pub_->publish(*output_poses);
    pose_array_pub_->publish(create_pose_array(*output_poses));
    marker_pub_->publish(create_markers(*output_poses));
  }

  std::optional<PersonPose3DArray> transform_poses_to_ref_frame(
    const PersonPose3DArray & poses_3d)
  {
    const auto target_frame = normalized_ref_frame();
    const auto & source_frame = poses_3d.header.frame_id;
    if (target_frame.empty() || target_frame == source_frame) {
      return poses_3d;
    }

    if (source_frame.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Cannot transform person poses to '%s': source frame is empty.",
        target_frame.c_str());
      return std::nullopt;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(
        target_frame,
        source_frame,
        rclcpp::Time(poses_3d.header.stamp),
        rclcpp::Duration::from_seconds(0.1));
    } catch (const std::exception & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Skipping person pose frame: failed to transform '%s' -> '%s' at stamp %d.%09u: %s",
        source_frame.c_str(), target_frame.c_str(), poses_3d.header.stamp.sec,
        poses_3d.header.stamp.nanosec, ex.what());
      return std::nullopt;
    }

    PersonPose3DArray transformed_poses = poses_3d;
    transformed_poses.header.frame_id = target_frame;
    for (auto & pose_3d : transformed_poses.poses) {
      for (auto & point : pose_3d.keypoints) {
        if (!is_finite_point(point)) {
          continue;
        }

        geometry_msgs::msg::PointStamped source_point;
        source_point.header = poses_3d.header;
        source_point.point = point;

        geometry_msgs::msg::PointStamped transformed_point;
        tf2::doTransform(source_point, transformed_point, transform);
        point = transformed_point.point;
      }
    }
    return transformed_poses;
  }

  std::string normalized_ref_frame() const
  {
    if (ref_frame_.empty() || ref_frame_ == "None" || ref_frame_ == "none") {
      return "";
    }
    return ref_frame_;
  }

  geometry_msgs::msg::Point project_keypoint(
    const nakalab_ultralytics_interfaces::msg::Point & keypoint,
    const Image & depth_image,
    const CameraInfo & intrinsics)
  {
    geometry_msgs::msg::Point point;
    point.x = quiet_nan();
    point.y = quiet_nan();
    point.z = quiet_nan();

    if (keypoint.confidence <= 0.0F || !is_valid_pixel(keypoint.x, keypoint.y, depth_image)) {
      return point;
    }

    const auto depth = depth_at(depth_image, keypoint.x, keypoint.y);
    if (!depth || *depth <= 0.0 || *depth > max_depth_m_) {
      return point;
    }

    const double fx = intrinsics.k[0];
    const double fy = intrinsics.k[4];
    const double cx = intrinsics.k[2];
    const double cy = intrinsics.k[5];
    if (fx == 0.0 || fy == 0.0) {
      return point;
    }

    point.z = *depth;
    point.x = (static_cast<double>(keypoint.x) - cx) * point.z / fx;
    point.y = (static_cast<double>(keypoint.y) - cy) * point.z / fy;
    return point;
  }

  std::optional<double> depth_at(const Image & image, double x, double y)
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
      const auto offset = static_cast<std::size_t>(v) * image.step + static_cast<std::size_t>(u) *
        sizeof(uint16_t);
      if (offset + sizeof(uint16_t) > image.data.size()) {
        return std::nullopt;
      }
      uint16_t raw = 0;
      std::copy_n(image.data.data() + offset, sizeof(uint16_t), reinterpret_cast<uint8_t *>(&raw));
      if (raw == 0) {
        return std::nullopt;
      }
      return static_cast<double>(raw) * 0.001;
    }

    if (image.encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
      const auto offset = static_cast<std::size_t>(v) * image.step + static_cast<std::size_t>(u) *
        sizeof(float);
      if (offset + sizeof(float) > image.data.size()) {
        return std::nullopt;
      }
      float raw = 0.0F;
      std::copy_n(image.data.data() + offset, sizeof(float), reinterpret_cast<uint8_t *>(&raw));
      if (!std::isfinite(raw) || raw <= 0.0F) {
        return std::nullopt;
      }
      return static_cast<double>(raw);
    }

    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Unsupported depth image encoding: '%s'", image.encoding.c_str());
    return std::nullopt;
  }

  PoseArray create_pose_array(const PersonPose3DArray & poses_3d) const
  {
    PoseArray pose_array;
    pose_array.header = poses_3d.header;

    for (const auto & pose_3d : poses_3d.poses) {
      const auto person_position = estimate_dense_position(pose_3d);
      if (!person_position) {
        continue;
      }

      Pose pose;
      pose.position = *person_position;
      pose.orientation.w = 1.0;
      pose_array.poses.push_back(pose);
    }

    return pose_array;
  }

  std::optional<geometry_msgs::msg::Point> estimate_dense_position(
    const PersonPose3D & pose_3d) const
  {
    std::vector<geometry_msgs::msg::Point> valid_points;
    valid_points.reserve(kKeypointCount);
    for (const auto & point : pose_3d.keypoints) {
      if (is_finite_point(point)) {
        valid_points.push_back(point);
      }
    }

    if (valid_points.empty()) {
      return std::nullopt;
    }

    const double radius_sq = dense_cluster_radius_m_ * dense_cluster_radius_m_;
    std::vector<std::size_t> best_cluster;
    best_cluster.reserve(valid_points.size());
    double best_depth_variance = std::numeric_limits<double>::infinity();

    for (std::size_t i = 0; i < valid_points.size(); ++i) {
      std::vector<std::size_t> cluster;
      cluster.reserve(valid_points.size());
      for (std::size_t j = 0; j < valid_points.size(); ++j) {
        if (squared_distance(valid_points[i], valid_points[j]) <= radius_sq) {
          cluster.push_back(j);
        }
      }

      const double depth_variance = z_variance(valid_points, cluster);
      if (cluster.size() > best_cluster.size() ||
        (cluster.size() == best_cluster.size() && depth_variance < best_depth_variance))
      {
        best_cluster = cluster;
        best_depth_variance = depth_variance;
      }
    }

    geometry_msgs::msg::Point center;
    for (const auto point_index : best_cluster) {
      center.x += valid_points[point_index].x;
      center.y += valid_points[point_index].y;
      center.z += valid_points[point_index].z;
    }

    const auto cluster_size = static_cast<double>(best_cluster.size());
    center.x /= cluster_size;
    center.y /= cluster_size;
    center.z /= cluster_size;
    return center;
  }

  double z_variance(
    const std::vector<geometry_msgs::msg::Point> & points,
    const std::vector<std::size_t> & indices) const
  {
    if (indices.empty()) {
      return std::numeric_limits<double>::infinity();
    }

    double mean = 0.0;
    for (const auto index : indices) {
      mean += points[index].z;
    }
    mean /= static_cast<double>(indices.size());

    double variance = 0.0;
    for (const auto index : indices) {
      const double diff = points[index].z - mean;
      variance += diff * diff;
    }
    return variance / static_cast<double>(indices.size());
  }

  MarkerArray create_markers(const PersonPose3DArray & poses_3d) const
  {
    MarkerArray markers;

    Marker clear_marker;
    clear_marker.header = poses_3d.header;
    clear_marker.action = Marker::DELETEALL;
    markers.markers.push_back(clear_marker);

    for (std::size_t i = 0; i < poses_3d.poses.size(); ++i) {
      const auto color = color_for_pose(i);

      Marker marker;
      marker.header = poses_3d.header;
      marker.ns = "person_pose_3d_keypoints";
      marker.id = static_cast<int32_t>(i);
      marker.type = Marker::SPHERE_LIST;
      marker.action = Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = marker_scale_m_;
      marker.scale.y = marker_scale_m_;
      marker.scale.z = marker_scale_m_;
      marker.color.r = color[0];
      marker.color.g = color[1];
      marker.color.b = color[2];
      marker.color.a = 1.0F;

      for (const auto & point : poses_3d.poses[i].keypoints) {
        if (is_finite_point(point)) {
          marker.points.push_back(point);
        }
      }

      markers.markers.push_back(marker);

      Marker bone_marker;
      bone_marker.header = poses_3d.header;
      bone_marker.ns = "person_pose_3d_coco_bones";
      bone_marker.id = static_cast<int32_t>(i);
      bone_marker.type = Marker::LINE_LIST;
      bone_marker.action = Marker::ADD;
      bone_marker.pose.orientation.w = 1.0;
      bone_marker.scale.x = marker_scale_m_ * 0.35;
      bone_marker.color.r = color[0];
      bone_marker.color.g = color[1];
      bone_marker.color.b = color[2];
      bone_marker.color.a = 1.0F;

      for (const auto & bone_pair : kCocoBonePairs) {
        const auto & start = poses_3d.poses[i].keypoints[bone_pair[0]];
        const auto & end = poses_3d.poses[i].keypoints[bone_pair[1]];
        if (is_finite_point(start) && is_finite_point(end)) {
          bone_marker.points.push_back(start);
          bone_marker.points.push_back(end);
        }
      }

      markers.markers.push_back(bone_marker);
    }

    return markers;
  }

  double max_depth_m_{10.0};
  double marker_scale_m_{0.04};
  double dense_cluster_radius_m_{0.35};
  std::string ref_frame_;

  CameraInfo::ConstSharedPtr color_camera_info_;
  CameraInfo::ConstSharedPtr depth_camera_info_;
  PersonPose2DArray::ConstSharedPtr latest_poses_2d_;
  rclcpp::Subscription<CameraInfo>::SharedPtr color_info_sub_;
  rclcpp::Subscription<CameraInfo>::SharedPtr depth_info_sub_;
  rclcpp::Subscription<PersonPose2DArray>::SharedPtr poses_2d_sub_;
  std::unique_ptr<message_filters::Subscriber<Image>> depth_sub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<PersonPose3DArray>::SharedPtr pose_pub_;
  rclcpp::Publisher<PoseArray>::SharedPtr pose_array_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr marker_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PersonPose3DNode>());
  rclcpp::shutdown();
  return 0;
}
