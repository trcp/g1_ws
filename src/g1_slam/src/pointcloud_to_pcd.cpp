#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <sstream>
#include <iomanip>

class PointCloudToPCD : public rclcpp::Node
{
public:
  PointCloudToPCD() : Node("pointcloud_to_pcd")
  {
    this->declare_parameter<std::string>("prefix", "map_");
    this->declare_parameter<bool>("binary", false);
    this->declare_parameter<std::string>("input_topic", "/map");

    std::string input_topic;
    this->get_parameter("input_topic", input_topic);

    // サブスクライバの作成
    // system_default QoS を使用 (Best Effortのデータも受け取れるように調整推奨ですが、まずは標準で)
    rclcpp::QoS qos(rclcpp::KeepLast(10)); 
    
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, 
      qos, 
      std::bind(&PointCloudToPCD::callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", input_topic.c_str());
  }

private:
  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromPCLPointCloud2(pcl_pc2, cloud);

    // ファイル名の生成 (タイムスタンプを使用)
    // ROS1のような連番が必要な場合は別途カウンタ変数をメンバに持たせてください
    std::string prefix;
    bool binary_mode;
    this->get_parameter("prefix", prefix);
    this->get_parameter("binary", binary_mode);

    std::stringstream ss;
    ss << prefix << ".pcd";
    std::string filename = ss.str();

    RCLCPP_INFO(this->get_logger(), "Saving to %s (Points: %zu)", filename.c_str(), cloud.size());

    if (cloud.empty()) {
      RCLCPP_WARN(this->get_logger(), "Empty pointcloud received, skipping save.");
      return;
    }

    try {
      if (binary_mode) {
        pcl::io::savePCDFileBinary(filename, cloud);
      } else {
        pcl::io::savePCDFileASCII(filename, cloud);
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to save PCD: %s", e.what());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudToPCD>());
  rclcpp::shutdown();
  return 0;
}
