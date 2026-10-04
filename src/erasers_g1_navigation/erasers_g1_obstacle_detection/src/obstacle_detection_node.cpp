#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <opencv2/opencv.hpp>

class ObstacleDetectionNode : public rclcpp::Node
{
public:
  ObstacleDetectionNode()
  : Node("obstacle_detection_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/head_camera/d455/depth/color/points", 10,
      std::bind(&ObstacleDetectionNode::callback, this, std::placeholders::_1)
    );
  }

private:
  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    sensor_msgs::msg::PointCloud2 cloud_map;

    // --- TF変換 ---
    try {
      auto tf = tf_buffer_.lookupTransform(
        "map", msg->header.frame_id, msg->header.stamp);
      tf2::doTransform(*msg, cloud_map, tf);
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "%s", ex.what());
      return;
    }

    int width = cloud_map.width;
    int height = cloud_map.height;

    cv::Mat image(height, width, CV_8UC3);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_map, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud_map, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud_map, "z");

    sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_r(cloud_map, "r");
    sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_g(cloud_map, "g");
    sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_b(cloud_map, "b");

    for (int v = 0; v < height; ++v)
    {
      for (int u = 0; u < width; ++u,
           ++iter_x, ++iter_y, ++iter_z,
           ++iter_r, ++iter_g, ++iter_b)
      {
        if (!std::isfinite(*iter_z))
        {
          // image.at<cv::Vec3b>(v, u) = cv::Vec3b(0,0,0);
	  image.at<cv::Vec3b>(v, u) = cv::Vec3b(*iter_b, *iter_g, *iter_r);
          continue;
        }

        // --- 高さフィルタ ---
        if (*iter_z > 0.02)  // ←ここ調整（2cm推奨）
        {
          // image.at<cv::Vec3b>(v, u) =
          //   cv::Vec3b(*iter_b, *iter_g, *iter_r);
	  image.at<cv::Vec3b>(v, u) =
	    cv::Vec3b(0,0,0);
        }
        else
        {
          // image.at<cv::Vec3b>(v, u) = cv::Vec3b(0,0,0);
	  image.at<cv::Vec3b>(v, u) = cv::Vec3b(*iter_b, *iter_g, *iter_r);
        }
      }
    }

    cv::imshow("filtered_image", image);
    cv::waitKey(1);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleDetectionNode>());
  rclcpp::shutdown();
  return 0;
}
