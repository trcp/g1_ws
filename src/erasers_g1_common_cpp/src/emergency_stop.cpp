#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/wireless_controller.hpp"
#include "g1_srvs/srv/pose_policy.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class EmergencyStopNode : public rclcpp::Node
{
public:
  EmergencyStopNode()
  : Node("emergency_stop_node")
  {
    subscription_ = this->create_subscription<unitree_go::msg::WirelessController>(
      "/wirelesscontroller", 10, std::bind(&EmergencyStopNode::topic_callback, this, _1));
    client_ = this->create_client<g1_srvs::srv::PosePolicy>("/pose_policy");
    if (!client_->wait_for_service(1s)) {
      RCLCPP_WARN(this->get_logger(), "Service /pose_policy not available");
      return;
    }
  }

private:
  void topic_callback(const unitree_go::msg::WirelessController::SharedPtr msg)
  {
    if (msg->keys == 192) {
      if (is_processing_) {
        return;
      }
      is_processing_ = true;
      RCLCPP_ERROR(this->get_logger(), "EMERGENCY STOP BY REMOTE CONTROLLER!!!!!");

      auto request = std::make_shared<g1_srvs::srv::PosePolicy::Request>();
      //request->pose = "damp";
      request->pose = "squat";

      using ServiceResponseFuture = rclcpp::Client<g1_srvs::srv::PosePolicy>::SharedFuture;
      auto response_received_callback = [this](ServiceResponseFuture future) {
        (void)future;
        RCLCPP_INFO(this->get_logger(), "Emergency stop (damp) request completed.");
        is_processing_ = false;
      };

      client_->async_send_request(request, response_received_callback);
    }
  }

  rclcpp::Subscription<unitree_go::msg::WirelessController>::SharedPtr subscription_;
  rclcpp::Client<g1_srvs::srv::PosePolicy>::SharedPtr client_;
  bool is_processing_ = false;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EmergencyStopNode>());
  rclcpp::shutdown();
  return 0;
}
