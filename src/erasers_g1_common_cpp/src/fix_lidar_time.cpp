#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

/**
 * @brief LiDARデータの未来時刻スタンプを現在のシステム時刻に修正するノード
 */
class PointCloudTimestampFixer : public rclcpp::Node
{
public:
  PointCloudTimestampFixer()
  : Node("pointcloud_timestamp_fixer")
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();

    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/utlidar/cloud_livox_mid360_fixed", qos);

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/utlidar/cloud_livox_mid360", qos,
      std::bind(&PointCloudTimestampFixer::topic_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "C++ PointCloud Timestamp Fixer started.");
    RCLCPP_INFO(this->get_logger(), "Subscribing to: /utlidar/cloud_livox_mid360");
    RCLCPP_INFO(this->get_logger(), "Publishing to: /utlidar/cloud_livox_mid360_fixed");
  }

private:
  void topic_callback(sensor_msgs::msg::PointCloud2::SharedPtr msg) const
  {
    msg->header.stamp = this->now();
    publisher_->publish(*msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudTimestampFixer>());
  rclcpp::shutdown();
  return 0;
}
