#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <atomic>

class CameraRelayNode : public rclcpp::Node
{
public:
  CameraRelayNode() : Node("camera_relay_node"), output_state_true_(true)
  {
    image_pub1_ = this->create_publisher<sensor_msgs::msg::Image>("/output1_image", 10);
    camera_info_pub1_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/output1_camera", 10);

    image_pub0_ = this->create_publisher<sensor_msgs::msg::Image>("/output0_image", 10);
    camera_info_pub0_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/output0_camera", 10);

    switch_service_ = this->create_service<std_srvs::srv::SetBool>(
      "switch_camera_output",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
             std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        
        output_state_true_ = request->data; // 状態の更新
        
        response->success = true;
        if (output_state_true_) {
          response->message = "Switched to output1 (/output1_image, /output1_camera)";
        } else {
          response->message = "Switched to output0 (/output0_image, /output0_camera)";
        }
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
      });

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/input_image", 10,
      [this](sensor_msgs::msg::Image::UniquePtr msg) {
        if (output_state_true_) {
          image_pub1_->publish(std::move(msg));
        } else {
          image_pub0_->publish(std::move(msg));
        }
      });

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/input_camera", 10,
      [this](sensor_msgs::msg::CameraInfo::UniquePtr msg) {
        if (output_state_true_) {
          camera_info_pub1_->publish(std::move(msg));
        } else {
          camera_info_pub0_->publish(std::move(msg));
        }
      });

    RCLCPP_INFO(this->get_logger(), "Camera Relay Node has been started.");
    RCLCPP_INFO(this->get_logger(), "Default routing: /input_* -> /output1_*");
    RCLCPP_INFO(this->get_logger(), "Call service '~/switch_camera_output' to change routing.");
  }

private:
  // 出力先の状態を保持（デフォルトはtrue）
  std::atomic<bool> output_state_true_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub1_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub1_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub0_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub0_;
  
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr switch_service_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraRelayNode>());
  rclcpp::shutdown();
  return 0;
}