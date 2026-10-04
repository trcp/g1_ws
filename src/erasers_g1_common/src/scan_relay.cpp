#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class ScanRelay : public rclcpp::Node
{
public:
  ScanRelay()
  : Node("scan_relay")
  {
    this->declare_parameter("input_topic", "/scan");
    this->declare_parameter("output_topic", "/scan_reliable");
    // タイムスタンプをどれくらい過去に戻すか (秒)
    this->declare_parameter("time_offset", 0.05); 

    std::string input_topic = this->get_parameter("input_topic").as_string();
    std::string output_topic = this->get_parameter("output_topic").as_string();
    time_offset_ = this->get_parameter("time_offset").as_double();

    // Subscriber: Best Effort
    auto sub_qos = rclcpp::SensorDataQoS();
    
    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      input_topic,
      sub_qos,
      std::bind(&ScanRelay::topic_callback, this, std::placeholders::_1));

    // Publisher: Reliable
    auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(output_topic, pub_qos);

    RCLCPP_INFO(this->get_logger(), "Scan Relay Started with offset -%.3fs", time_offset_);
  }

private:
  void topic_callback(sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // メッセージのタイムスタンプを取得
    rclcpp::Time current_stamp = msg->header.stamp;
    
    // オフセット分だけ時間を戻す (現在の時刻 - time_offset_)
    // これにより、TFバッファ内にデータが存在する時刻に合わせる
    rclcpp::Time new_stamp = current_stamp - rclcpp::Duration::from_seconds(time_offset_);
    msg->header.stamp = new_stamp;

    // 再配信
    pub_->publish(*msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  double time_offset_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanRelay>());
  rclcpp::shutdown();
  return 0;
}
