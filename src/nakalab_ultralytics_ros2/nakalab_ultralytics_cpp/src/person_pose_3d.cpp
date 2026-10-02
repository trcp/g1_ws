#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "nakalab_ultralytics_cpp/person_pose_fusion.hpp"
#include "nakalab_ultralytics_interfaces/msg/person_pose2_d_array.hpp"
#include "nakalab_ultralytics_interfaces/msg/person_pose3_d.hpp"
#include "nakalab_ultralytics_interfaces/msg/person_pose3_d_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace
{

using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseArray;
using nakalab_ultralytics_interfaces::msg::PersonPose2DArray;
using nakalab_ultralytics_interfaces::msg::PersonPose3D;
using nakalab_ultralytics_interfaces::msg::PersonPose3DArray;
using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::Image;
using sensor_msgs::msg::PointCloud2;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;
namespace fusion = nakalab_ultralytics_cpp::fusion;

constexpr std::size_t kKeypointCount = 16;
constexpr std::size_t kSyncQueueSize = 60;
constexpr const char * kInputPosesTopic = "/nu_ros2/person_pose_2d";
constexpr const char * kDepthImageTopic = "/depth_image";
constexpr const char * kPointcloudTopic = "/pointcloud";
constexpr const char * kColorCameraInfoTopic = "/color_camera_info";
constexpr const char * kOutputPosesTopic = "/nu_ros2/person_pose_3d";
constexpr const char * kMarkerTopic = "/nu_ros2/detect_poses";
constexpr const char * kPoseArrayTopic = "/nu_ros2/poses";
constexpr std::array<std::array<std::size_t, 2>, 15> kCocoBonePairs{{
  {{0, 1}},
  {{0, 2}},
  {{1, 3}},
  {{2, 4}},
  {{5, 6}},
  {{5, 7}},
  {{7, 9}},
  {{6, 8}},
  {{8, 10}},
  {{5, 11}},
  {{6, 12}},
  {{11, 12}},
  {{11, 13}},
  {{13, 15}},
  {{12, 14}},
}};

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
    sensor_fusion_ = declare_parameter<std::string>("sensor_fusion", "depth");
    fusion_sync_tolerance_sec_ =
      declare_parameter<double>("fusion_sync_tolerance_sec", 0.15);

    validate_parameters();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    pose_pub_ = create_publisher<PersonPose3DArray>(kOutputPosesTopic, 10);
    marker_pub_ = create_publisher<MarkerArray>(kMarkerTopic, 10);
    pose_array_pub_ = create_publisher<PoseArray>(kPoseArrayTopic, 10);

    color_info_sub_ = create_subscription<CameraInfo>(
      kColorCameraInfoTopic, rclcpp::SensorDataQoS(),
      std::bind(&PersonPose3DNode::on_color_camera_info, this, std::placeholders::_1));

    poses_2d_sub_ = std::make_unique<message_filters::Subscriber<PersonPose2DArray>>(
      this, kInputPosesTopic, rmw_qos_profile_default);
    if (sensor_fusion_ == "depth") {
      start_depth_subscriptions();
    } else {
      start_pointcloud_subscriptions();
    }

    RCLCPP_INFO(
      get_logger(),
      "Person pose sensor fusion mode: '%s'", sensor_fusion_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "Output reference frame: '%s'",
      normalized_ref_frame().empty() ? "<camera frame>" : normalized_ref_frame().c_str());
    RCLCPP_INFO(
      get_logger(),
      "Waiting for color CameraInfo on '%s'.", kColorCameraInfoTopic);
  }

private:
  using DepthSyncPolicy =
    message_filters::sync_policies::ApproximateTime<PersonPose2DArray, Image>;
  using PointcloudSyncPolicy =
    message_filters::sync_policies::ApproximateTime<PersonPose2DArray, PointCloud2>;

  void validate_parameters() const
  {
    if (sensor_fusion_ != "depth" && sensor_fusion_ != "pointcloud") {
      throw std::invalid_argument(
              "sensor_fusion must be either 'depth' or 'pointcloud'");
    }
    if (max_depth_m_ <= 0.0 || dense_cluster_radius_m_ < 0.0 ||
      fusion_sync_tolerance_sec_ < 0.0)
    {
      throw std::invalid_argument("Person pose fusion parameters must be positive");
    }
    if (sensor_fusion_ == "pointcloud" && normalized_ref_frame().empty()) {
      throw std::invalid_argument(
              "ref_frame must be set when sensor_fusion is 'pointcloud'");
    }
  }

  void start_depth_subscriptions()
  {
    depth_sub_ = std::make_unique<message_filters::Subscriber<Image>>(
      this, kDepthImageTopic, rmw_qos_profile_sensor_data);
    depth_sync_ = std::make_unique<message_filters::Synchronizer<DepthSyncPolicy>>(
      DepthSyncPolicy(kSyncQueueSize), *poses_2d_sub_, *depth_sub_);
    depth_sync_->setMaxIntervalDuration(
      rclcpp::Duration::from_seconds(fusion_sync_tolerance_sec_));
    depth_sync_->registerCallback(
      std::bind(
        &PersonPose3DNode::on_depth_pair, this,
        std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(),
      "Started synchronized subscriptions: poses='%s', depth='%s'",
      kInputPosesTopic, kDepthImageTopic);
  }

  void start_pointcloud_subscriptions()
  {
    pointcloud_sub_ = std::make_unique<message_filters::Subscriber<PointCloud2>>(
      this, kPointcloudTopic, rmw_qos_profile_sensor_data);
    pointcloud_sync_ =
      std::make_unique<message_filters::Synchronizer<PointcloudSyncPolicy>>(
      PointcloudSyncPolicy(kSyncQueueSize), *poses_2d_sub_, *pointcloud_sub_);
    pointcloud_sync_->setMaxIntervalDuration(
      rclcpp::Duration::from_seconds(fusion_sync_tolerance_sec_));
    pointcloud_sync_->registerCallback(
      std::bind(
        &PersonPose3DNode::on_pointcloud_pair, this,
        std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(),
      "Started synchronized subscriptions: poses='%s', pointcloud='%s'",
      kInputPosesTopic, kPointcloudTopic);
  }

  void on_color_camera_info(const CameraInfo::SharedPtr msg)
  {
    const bool first_message = !color_camera_info_;
    color_camera_info_ = msg;
    if (first_message) {
      RCLCPP_INFO(
        get_logger(), "Received color CameraInfo for frame '%s'.",
        msg->header.frame_id.c_str());
    }
  }

  bool pair_is_usable(
    const PersonPose2DArray & poses_2d,
    const std_msgs::msg::Header & sensor_header,
    const char * sensor_name)
  {
    if (!color_camera_info_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Skipping %s fusion: waiting for CameraInfo on '%s'.",
        sensor_name, kColorCameraInfoTopic);
      clear_markers(poses_2d.header);
      return false;
    }
    if (!fusion::stamps_within_tolerance(
        poses_2d.header.stamp, sensor_header.stamp, fusion_sync_tolerance_sec_))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Skipping %s fusion: RGB and sensor timestamps differ by more than %.3f seconds.",
        sensor_name, fusion_sync_tolerance_sec_);
      clear_markers(poses_2d.header);
      return false;
    }
    if (camera_frame(poses_2d).empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Skipping %s fusion: color camera frame is empty.", sensor_name);
      clear_markers(poses_2d.header);
      return false;
    }
    return true;
  }

  void on_depth_pair(
    const PersonPose2DArray::ConstSharedPtr poses_2d,
    const Image::ConstSharedPtr depth_image)
  {
    if (!pair_is_usable(*poses_2d, depth_image->header, "depth")) {
      return;
    }

    PersonPose3DArray poses_3d;
    poses_3d.header = poses_2d->header;
    poses_3d.header.frame_id = camera_frame(*poses_2d);
    poses_3d.poses.reserve(poses_2d->poses.size());

    try {
      for (const auto & pose_2d : poses_2d->poses) {
        PersonPose3D pose_3d;
        pose_3d.bounding_box = pose_2d.bounding_box;
        pose_3d.confidence = pose_2d.confidence;
        for (std::size_t i = 0; i < kKeypointCount; ++i) {
          pose_3d.keypoints[i] = fusion::project_depth_keypoint(
            pose_2d.keypoints[i], *depth_image, *color_camera_info_, max_depth_m_);
        }
        poses_3d.poses.push_back(pose_3d);
      }
    } catch (const std::exception & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Skipping depth fusion frame: %s", ex.what());
      clear_markers(poses_2d->header);
      return;
    }

    publish_poses(poses_3d);
  }

  void on_pointcloud_pair(
    const PersonPose2DArray::ConstSharedPtr poses_2d,
    const PointCloud2::ConstSharedPtr pointcloud)
  {
    if (!pair_is_usable(*poses_2d, pointcloud->header, "pointcloud")) {
      return;
    }
    if (pointcloud->header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Skipping pointcloud fusion: PointCloud2 frame is empty.");
      clear_markers(poses_2d->header);
      return;
    }

    const auto color_frame = camera_frame(*poses_2d);
    geometry_msgs::msg::TransformStamped cloud_to_camera;
    try {
      cloud_to_camera = tf_buffer_->lookupTransform(
        color_frame,
        rclcpp::Time(poses_2d->header.stamp),
        pointcloud->header.frame_id,
        rclcpp::Time(pointcloud->header.stamp),
        normalized_ref_frame(),
        rclcpp::Duration::from_seconds(0.1));
    } catch (const std::exception & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Skipping pointcloud fusion: failed time-compensated transform '%s' -> '%s' "
        "through '%s': %s",
        pointcloud->header.frame_id.c_str(), color_frame.c_str(),
        normalized_ref_frame().c_str(), ex.what());
      clear_markers(poses_2d->header);
      return;
    }

    std::vector<fusion::ProjectedPoint> projected_points;
    try {
      projected_points = fusion::project_pointcloud_to_image(
        *pointcloud, *color_camera_info_, cloud_to_camera, max_depth_m_);
    } catch (const std::exception & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Skipping pointcloud fusion frame: %s", ex.what());
      clear_markers(poses_2d->header);
      return;
    }

    PersonPose3DArray poses_3d;
    poses_3d.header = poses_2d->header;
    poses_3d.header.frame_id = color_frame;
    poses_3d.poses.reserve(poses_2d->poses.size());
    for (const auto & pose_2d : poses_2d->poses) {
      PersonPose3D pose_3d;
      pose_3d.bounding_box = pose_2d.bounding_box;
      pose_3d.confidence = pose_2d.confidence;
      for (std::size_t i = 0; i < kKeypointCount; ++i) {
        pose_3d.keypoints[i] = fusion::fuse_pointcloud_keypoint(
          pose_2d.keypoints[i],
          pose_2d.bounding_box,
          projected_points);
      }
      poses_3d.poses.push_back(pose_3d);
    }

    publish_poses(poses_3d);
  }

  std::string camera_frame(const PersonPose2DArray & poses_2d) const
  {
    if (color_camera_info_ && !color_camera_info_->header.frame_id.empty()) {
      return color_camera_info_->header.frame_id;
    }
    return poses_2d.header.frame_id;
  }

  std::string normalized_ref_frame() const
  {
    if (ref_frame_.empty() || ref_frame_ == "None" || ref_frame_ == "none") {
      return "";
    }
    return ref_frame_;
  }

  void publish_poses(const PersonPose3DArray & poses_3d)
  {
    auto output_poses = transform_poses_to_ref_frame(poses_3d);
    if (!output_poses) {
      clear_markers(poses_3d.header);
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
        if (!fusion::is_finite_point(point)) {
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

  void clear_markers(const std_msgs::msg::Header & source_header)
  {
    MarkerArray markers;
    Marker clear_marker;
    clear_marker.header = source_header;
    const auto target_frame = normalized_ref_frame();
    if (!target_frame.empty()) {
      clear_marker.header.frame_id = target_frame;
    }
    clear_marker.action = Marker::DELETEALL;
    markers.markers.push_back(clear_marker);
    marker_pub_->publish(markers);
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
      if (fusion::is_finite_point(point)) {
        valid_points.push_back(point);
      }
    }
    if (valid_points.empty()) {
      return std::nullopt;
    }

    const double radius_sq = dense_cluster_radius_m_ * dense_cluster_radius_m_;
    std::vector<std::size_t> best_cluster;
    double best_depth_variance = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < valid_points.size(); ++i) {
      std::vector<std::size_t> cluster;
      for (std::size_t j = 0; j < valid_points.size(); ++j) {
        if (squared_distance(valid_points[i], valid_points[j]) <= radius_sq) {
          cluster.push_back(j);
        }
      }

      const double depth_variance = z_variance(valid_points, cluster);
      if (cluster.size() > best_cluster.size() ||
        (cluster.size() == best_cluster.size() && depth_variance < best_depth_variance))
      {
        best_cluster = std::move(cluster);
        best_depth_variance = depth_variance;
      }
    }

    geometry_msgs::msg::Point center;
    for (const auto index : best_cluster) {
      center.x += valid_points[index].x;
      center.y += valid_points[index].y;
      center.z += valid_points[index].z;
    }
    const auto count = static_cast<double>(best_cluster.size());
    center.x /= count;
    center.y /= count;
    center.z /= count;
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
      const double difference = points[index].z - mean;
      variance += difference * difference;
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

      Marker keypoint_marker;
      keypoint_marker.header = poses_3d.header;
      keypoint_marker.ns = "person_pose_3d_keypoints";
      keypoint_marker.id = static_cast<int32_t>(i);
      keypoint_marker.type = Marker::SPHERE_LIST;
      keypoint_marker.action = Marker::ADD;
      keypoint_marker.pose.orientation.w = 1.0;
      keypoint_marker.scale.x = marker_scale_m_;
      keypoint_marker.scale.y = marker_scale_m_;
      keypoint_marker.scale.z = marker_scale_m_;
      keypoint_marker.color.r = color[0];
      keypoint_marker.color.g = color[1];
      keypoint_marker.color.b = color[2];
      keypoint_marker.color.a = 1.0F;
      for (const auto & point : poses_3d.poses[i].keypoints) {
        if (fusion::is_finite_point(point)) {
          keypoint_marker.points.push_back(point);
        }
      }
      markers.markers.push_back(keypoint_marker);

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
        if (fusion::is_finite_point(start) && fusion::is_finite_point(end)) {
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
  double fusion_sync_tolerance_sec_{0.15};
  std::string ref_frame_;
  std::string sensor_fusion_{"depth"};

  CameraInfo::ConstSharedPtr color_camera_info_;
  rclcpp::Subscription<CameraInfo>::SharedPtr color_info_sub_;
  std::unique_ptr<message_filters::Subscriber<PersonPose2DArray>> poses_2d_sub_;
  std::unique_ptr<message_filters::Subscriber<Image>> depth_sub_;
  std::unique_ptr<message_filters::Subscriber<PointCloud2>> pointcloud_sub_;
  std::unique_ptr<message_filters::Synchronizer<DepthSyncPolicy>> depth_sync_;
  std::unique_ptr<message_filters::Synchronizer<PointcloudSyncPolicy>> pointcloud_sync_;
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
