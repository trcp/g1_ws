#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <vector>

#include "gtest/gtest.h"
#include "nakalab_ultralytics_cpp/person_pose_fusion.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace fusion = nakalab_ultralytics_cpp::fusion;

namespace
{

sensor_msgs::msg::CameraInfo make_camera_info()
{
  sensor_msgs::msg::CameraInfo info;
  info.width = 100;
  info.height = 100;
  info.header.frame_id = "camera";
  info.k[0] = 100.0;
  info.k[2] = 50.0;
  info.k[4] = 100.0;
  info.k[5] = 50.0;
  info.k[8] = 1.0;
  return info;
}

nakalab_ultralytics_interfaces::msg::Point make_keypoint(float x, float y)
{
  nakalab_ultralytics_interfaces::msg::Point keypoint;
  keypoint.x = x;
  keypoint.y = y;
  keypoint.confidence = 1.0F;
  return keypoint;
}

nakalab_ultralytics_interfaces::msg::BoundingBox make_bounding_box()
{
  nakalab_ultralytics_interfaces::msg::BoundingBox bounding_box;
  bounding_box.top_left.x = 30.0F;
  bounding_box.top_left.y = 30.0F;
  bounding_box.bottom_right.x = 70.0F;
  bounding_box.bottom_right.y = 70.0F;
  return bounding_box;
}

sensor_msgs::msg::PointCloud2 make_pointcloud(
  const std::vector<geometry_msgs::msg::Point> & points)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "lidar";
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  for (const auto & point : points) {
    *iter_x = static_cast<float>(point.x);
    *iter_y = static_cast<float>(point.y);
    *iter_z = static_cast<float>(point.z);
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }
  return cloud;
}

geometry_msgs::msg::TransformStamped make_identity_transform()
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "camera";
  transform.child_frame_id = "lidar";
  transform.transform.rotation.w = 1.0;
  return transform;
}

}  // namespace

TEST(PersonPoseFusion, AcceptsStampsAtToleranceBoundary)
{
  builtin_interfaces::msg::Time first;
  first.sec = 10;
  builtin_interfaces::msg::Time second;
  second.sec = 10;
  second.nanosec = 150000000U;

  EXPECT_TRUE(fusion::stamps_within_tolerance(first, second, 0.15));
  second.nanosec += 1U;
  EXPECT_FALSE(fusion::stamps_within_tolerance(first, second, 0.15));
}

TEST(PersonPoseFusion, RejectsZeroStamp)
{
  builtin_interfaces::msg::Time zero;
  builtin_interfaces::msg::Time valid;
  valid.sec = 1;
  EXPECT_FALSE(fusion::stamps_within_tolerance(zero, valid, 0.15));
}

TEST(PersonPoseFusion, Projects16BitDepth)
{
  auto camera_info = make_camera_info();
  sensor_msgs::msg::Image image;
  image.width = camera_info.width;
  image.height = camera_info.height;
  image.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
  image.step = image.width * sizeof(uint16_t);
  image.data.resize(static_cast<std::size_t>(image.step) * image.height, 0);

  const auto keypoint = make_keypoint(60.0F, 50.0F);
  const uint16_t raw_depth = 2000;
  const auto offset = static_cast<std::size_t>(50) * image.step +
    static_cast<std::size_t>(60) * sizeof(raw_depth);
  std::memcpy(image.data.data() + offset, &raw_depth, sizeof(raw_depth));

  const auto point =
    fusion::project_depth_keypoint(keypoint, image, camera_info, 10.0);
  EXPECT_NEAR(point.x, 0.2, 1.0e-6);
  EXPECT_NEAR(point.y, 0.0, 1.0e-6);
  EXPECT_NEAR(point.z, 2.0, 1.0e-6);
}

TEST(PersonPoseFusion, ProjectsFloatDepth)
{
  auto camera_info = make_camera_info();
  sensor_msgs::msg::Image image;
  image.width = camera_info.width;
  image.height = camera_info.height;
  image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  image.step = image.width * sizeof(float);
  image.data.resize(static_cast<std::size_t>(image.step) * image.height, 0);

  const auto keypoint = make_keypoint(50.0F, 60.0F);
  const float raw_depth = 3.0F;
  const auto offset = static_cast<std::size_t>(60) * image.step +
    static_cast<std::size_t>(50) * sizeof(raw_depth);
  std::memcpy(image.data.data() + offset, &raw_depth, sizeof(raw_depth));

  const auto point =
    fusion::project_depth_keypoint(keypoint, image, camera_info, 10.0);
  EXPECT_NEAR(point.x, 0.0, 1.0e-6);
  EXPECT_NEAR(point.y, 0.3, 1.0e-6);
  EXPECT_NEAR(point.z, 3.0, 1.0e-6);
}

TEST(PersonPoseFusion, ProjectsUnorganizedPointcloud)
{
  const auto cloud = make_pointcloud(
  {
    geometry_msgs::msg::Point().set__x(0.0).set__y(0.0).set__z(2.0),
    geometry_msgs::msg::Point().set__x(0.2).set__y(0.0).set__z(2.0),
    geometry_msgs::msg::Point().set__x(0.0).set__y(0.0).set__z(-1.0),
  });

  const auto projected = fusion::project_pointcloud_to_image(
    cloud, make_camera_info(), make_identity_transform(), 10.0);
  ASSERT_EQ(projected.size(), 2U);
  EXPECT_NEAR(projected[0].u, 50.0, 1.0e-6);
  EXPECT_NEAR(projected[0].v, 50.0, 1.0e-6);
  EXPECT_NEAR(projected[1].u, 60.0, 1.0e-5);
}

TEST(PersonPoseFusion, SelectsNearestProjectedPointInsideBoundingBox)
{
  const auto keypoint = make_keypoint(50.0F, 50.0F);
  const auto bounding_box = make_bounding_box();
  const std::vector<fusion::ProjectedPoint> projected_points{
    {49.0, 50.0, geometry_msgs::msg::Point().set__x(-0.02).set__z(4.00)},
    {52.0, 50.0, geometry_msgs::msg::Point().set__x(0.02).set__z(2.00)},
  };

  const auto point = fusion::fuse_pointcloud_keypoint(
    keypoint, bounding_box, projected_points);
  EXPECT_NEAR(point.x, -0.02, 1.0e-6);
  EXPECT_NEAR(point.z, 4.0, 1.0e-6);
}

TEST(PersonPoseFusion, UsesSinglePointInsideBoundingBox)
{
  const auto point = fusion::fuse_pointcloud_keypoint(
    make_keypoint(50.0F, 50.0F),
    make_bounding_box(),
    {{65.0, 65.0, geometry_msgs::msg::Point().set__z(2.0)}});
  EXPECT_TRUE(fusion::is_finite_point(point));
  EXPECT_NEAR(point.z, 2.0, 1.0e-6);
}

TEST(PersonPoseFusion, IgnoresCloserPointOutsideBoundingBox)
{
  const auto point = fusion::fuse_pointcloud_keypoint(
    make_keypoint(31.0F, 50.0F),
    make_bounding_box(),
  {
    {29.0, 50.0, geometry_msgs::msg::Point().set__z(1.0)},
    {40.0, 50.0, geometry_msgs::msg::Point().set__z(3.0)},
  });
  EXPECT_NEAR(point.z, 3.0, 1.0e-6);
}

TEST(PersonPoseFusion, ReturnsNanWhenBoundingBoxContainsNoPointcloud)
{
  const auto point = fusion::fuse_pointcloud_keypoint(
    make_keypoint(50.0F, 50.0F),
    make_bounding_box(),
    {{90.0, 90.0, geometry_msgs::msg::Point().set__z(2.0)}});
  EXPECT_FALSE(fusion::is_finite_point(point));
}

TEST(PersonPoseFusion, RejectsPointcloudWithoutXyzFields)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.width = 1;
  cloud.height = 1;
  EXPECT_THROW(
    fusion::project_pointcloud_to_image(
      cloud, make_camera_info(), make_identity_transform(), 10.0),
    std::invalid_argument);
}
