#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <unitree_go/msg/wireless_controller.hpp>
#include <erasers_g1_interfaces/srv/pose_policy.hpp>

// 入力監視を担当する。駆動の停止・ホールド・99 は robot_controller に集約する。
class EmergencyStopNode : public rclcpp::Node
{
public:
  EmergencyStopNode() : Node("emergency_stop_node")
  {
    declare_parameter<int>("emc_button_index", 0);
    declare_parameter<std::string>("emc_pose", "safety");
    declare_parameter<bool>("enable_zero_torque", true);
    const auto qos = rclcpp::QoS(1).reliable().transient_local();
    active_pub_ = create_publisher<std_msgs::msg::Bool>("/emergency_stop/active", qos);
    latched_pub_ = create_publisher<std_msgs::msg::Bool>("/emergency_stop/latched", qos);
    pose_client_ = create_client<erasers_g1_interfaces::srv::PosePolicy>("/pose_policy");
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/emc/joy", 10, [this](sensor_msgs::msg::Joy::ConstSharedPtr msg) {
        const auto index = get_parameter("emc_button_index").as_int();
        if (index < 0 || static_cast<size_t>(index) >= msg->buttons.size()) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "非常停止ボタンの位置が不正です");
          return;
        }
        joy_active_ = msg->buttons[index] != 0;
        update_state();
      });
    wireless_sub_ = create_subscription<unitree_go::msg::WirelessController>(
      "/wirelesscontroller", 10,
      [this](unitree_go::msg::WirelessController::ConstSharedPtr msg) {
        remote_active_ = (msg->keys & 192) == 192;
        update_state();
      });
  }

private:
  void update_state()
  {
    const bool active = joy_active_ || remote_active_;
    if (received_ && active == active_) {return;}
    const bool entering = active && !active_;
    active_ = active;
    received_ = true;
    latched_ = latched_ || active;
    std_msgs::msg::Bool message;
    message.data = active_;
    active_pub_->publish(message);
    message.data = latched_;
    latched_pub_->publish(message);

    if (entering) {
      RCLCPP_ERROR(get_logger(), "緊急停止を検出しました。通常指令は再起動までロックします");
      const auto pose = get_parameter("emc_pose").as_string();
      // safety が既定。旧パラメータを明示指定した場合だけ姿勢遷移を要求する。
      if (pose != "safety") {request_legacy_pose(pose, true);}
    } else if (latched_) {
      RCLCPP_WARN(get_logger(), "停止入力が解除されました。ホールドを解放し、指令ロックを維持します");
    }
  }

  void request_legacy_pose(const std::string & pose, bool allow_zero_torque)
  {
    if (!pose_client_->service_is_ready()) {
      RCLCPP_ERROR(get_logger(), "旧姿勢サービスが利用できません。停止ラッチは維持します");
      return;
    }
    auto request = std::make_shared<erasers_g1_interfaces::srv::PosePolicy::Request>();
    request->pose = pose;
    pose_client_->async_send_request(request,
      [this, allow_zero_torque](rclcpp::Client<erasers_g1_interfaces::srv::PosePolicy>::SharedFuture result) {
        try {
          if (!result.get()->success) {
            RCLCPP_ERROR(get_logger(), "旧姿勢要求が拒否されました");
          }
          if (allow_zero_torque && get_parameter("enable_zero_torque").as_bool()) {
            request_legacy_pose("zero_torque", false);
          }
        } catch (const std::exception & error) {
          RCLCPP_ERROR(get_logger(), "%s", error.what());
        }
      });
  }

  bool received_{false};
  bool joy_active_{false};
  bool remote_active_{false};
  bool active_{false};
  bool latched_{false};
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr active_pub_, latched_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<unitree_go::msg::WirelessController>::SharedPtr wireless_sub_;
  rclcpp::Client<erasers_g1_interfaces::srv::PosePolicy>::SharedPtr pose_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EmergencyStopNode>());
  rclcpp::shutdown();
  return 0;
}
