#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using std::placeholders::_1;

class QosBridgeNode : public rclcpp::Node
{
public:
  QosBridgeNode()
  : Node("qos_bridge_node")
  {
    // --- 受信設定 (パブリッシャに合わせて Volatile) ---
    // Reliability: RELIABLE, Durability: VOLATILE
    auto input_qos = rclcpp::QoS(rclcpp::KeepLast(1))
      .reliability(rclcpp::ReliabilityPolicy::Reliable)
      .durability(rclcpp::DurabilityPolicy::Volatile);

    // --- 送信設定 (サブスクライバに合わせて Transient Local) ---
    // Reliability: RELIABLE, Durability: TRANSIENT_LOCAL
    auto output_qos = rclcpp::QoS(rclcpp::KeepLast(1))
      .reliability(rclcpp::ReliabilityPolicy::Reliable)
      .durability(rclcpp::DurabilityPolicy::TransientLocal);

    // パブリッシャの作成 (/pcd_map_relay)
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/pcd_map_relay", 
      output_qos);

    // サブスクライバの作成 (/pcd_map)
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/pcd_map",
      input_qos,
      std::bind(&QosBridgeNode::topic_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "QoS Bridge Node (C++) has been started.");
    RCLCPP_INFO(this->get_logger(), "Subscribing: /pcd_map (Volatile)");
    RCLCPP_INFO(this->get_logger(), "Publishing:  /pcd_map_relay (Transient Local)");
  }

private:
  void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
  {
    // 受信したメッセージをそのまま再配信
    publisher_->publish(*msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QosBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
