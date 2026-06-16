#include <memory>
#include <string>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "unitree_go/msg/wireless_controller.hpp"
#include "g1_srvs/srv/pose_policy.hpp"
#include "g1_srvs/srv/audio_client.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class EmergencyStopNode : public rclcpp::Node
{
public:
  EmergencyStopNode()
  : Node("emergency_stop_node"), prev_button_state_(false), is_processing_(false), 
    is_armed_(false), initial_warning_sent_(false), safety_active_(false),
    restart_warning_active_(false)
  {
    this->declare_parameter<int>("emc_button_index", 0);
    this->declare_parameter<std::string>("emc_pose", "damp");
    this->declare_parameter<bool>("enable_zero_torque", true);

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/emc/joy", 10, std::bind(&EmergencyStopNode::joy_callback, this, _1));

    wireless_sub_ = this->create_subscription<unitree_go::msg::WirelessController>(
      "/wirelesscontroller", 10, std::bind(&EmergencyStopNode::wireless_callback, this, _1));

    pose_policy_client_ = this->create_client<g1_srvs::srv::PosePolicy>("/pose_policy");
    audio_client_ = this->create_client<g1_srvs::srv::AudioClient>("/play_audio");

    auto emergency_stop_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    emergency_active_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "/emergency_stop/active", emergency_stop_qos);
    zero_cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    zero_cmd_timer_ = this->create_wall_timer(
      50ms, std::bind(&EmergencyStopNode::publish_safety_zero_cmd, this));
    restart_warning_timer_ = this->create_wall_timer(
      10s, std::bind(&EmergencyStopNode::publish_restart_warning_tts, this));

    RCLCPP_INFO(this->get_logger(), "Emergency Stop Node has been started.");
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    int button_index = this->get_parameter("emc_button_index").as_int();

    if (button_index < 0 || button_index >= static_cast<int>(msg->buttons.size())) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Button index %d is out of range for the received Joy message.", button_index);
      return;
    }

    bool current_button_state = (msg->buttons[button_index] == 1);

    if (!is_armed_) {
      if (!current_button_state) {
        is_armed_ = true;
        publish_emergency_active(false);
        if (initial_warning_sent_) {
          start_restart_warning_tts();
        }
        RCLCPP_INFO(this->get_logger(), "Emergency button is released. System is now ARMED.");
      } else if (!initial_warning_sent_) {
        send_tts_request("Please release the emergency button");
        initial_warning_sent_ = true;
        RCLCPP_WARN(this->get_logger(), "Emergency button is pressed at startup. Please release it to arm the system.");
      }
      prev_button_state_ = current_button_state;
      return;
    }

    if (current_button_state && !prev_button_state_) {
      execute_emergency_sequence("JOY");
    } else if (!current_button_state && prev_button_state_ && safety_active_) {
      deactivate_safety_stop();
    }

    prev_button_state_ = current_button_state;
  }

  void wireless_callback(const unitree_go::msg::WirelessController::SharedPtr msg)
  {
    if (msg->keys == 192) {
      execute_emergency_sequence("REMOTE CONTROLLER");
    }
  }

  // 緊急停止の一連のシーケンスを管理する共通関数
  void execute_emergency_sequence(const std::string & source)
  {
    if (is_processing_) {
      RCLCPP_WARN(this->get_logger(), "Emergency stop is already in progress. Ignoring %s request.", source.c_str());
      return;
    }

    is_processing_ = true;
    std::string emc_pose = this->get_parameter("emc_pose").as_string();
    RCLCPP_ERROR(this->get_logger(), "EMERGENCY STOP BY %s!!!!! Triggering pose policy: '%s'", source.c_str(), emc_pose.c_str());

    if (emc_pose == "safety") {
      activate_safety_stop();
      send_tts_request("Emergency Stop!");
      RCLCPP_INFO(this->get_logger(), "Safety emergency stop request from %s latched.", source.c_str());
      return;
    }

    send_tts_request("Emergency Stop!");
    
    // 1段階目: 指定されたポーズ (emc_pose) を適用
    call_pose_policy_service(emc_pose, [this, source]() {
      RCLCPP_INFO(this->get_logger(), "Emergency stop request from %s completed.", source.c_str());
      
      // 2段階目: パラメータが有効なら続けて zero_torque に遷移
      bool enable_zero_torque = this->get_parameter("enable_zero_torque").as_bool();
      if (enable_zero_torque) {
        RCLCPP_WARN(this->get_logger(), "enable_zero_torque is true. Transitioning to 'zero_torque'...");
        call_pose_policy_service("zero_torque", [this]() {
          RCLCPP_INFO(this->get_logger(), "Zero torque request completed.");
          is_processing_ = false; // 全てのシーケンスが完了した時点でフラグを下ろす
        });
      } else {
        is_processing_ = false; // zero_torque が無効な場合はここで完了
      }
    });
  }

  void activate_safety_stop()
  {
    safety_active_ = true;

    publish_emergency_active(true);

    publish_safety_zero_cmd();
    RCLCPP_ERROR(this->get_logger(), "Safety emergency stop is active. Publishing zero /cmd_vel continuously.");
  }

  void deactivate_safety_stop()
  {
    safety_active_ = false;
    is_processing_ = false;
    publish_emergency_active(false);
    start_restart_warning_tts();
    RCLCPP_INFO(this->get_logger(), "Safety emergency stop has been released. Restart container is required.");
  }

  void publish_emergency_active(bool active)
  {
    auto active_msg = std_msgs::msg::Bool();
    active_msg.data = active;
    emergency_active_pub_->publish(active_msg);
  }

  void publish_safety_zero_cmd()
  {
    if (!safety_active_) {
      return;
    }

    auto zero_cmd = geometry_msgs::msg::Twist();
    zero_cmd.linear.x = 0.0;
    zero_cmd.linear.y = 0.0;
    zero_cmd.angular.z = 0.0;
    zero_cmd_vel_pub_->publish(zero_cmd);
  }

  void start_restart_warning_tts()
  {
    restart_warning_active_ = true;
    publish_restart_warning_tts();
  }

  void publish_restart_warning_tts()
  {
    if (!restart_warning_active_) {
      return;
    }

    send_tts_request("Emergency button disarmed. Please restart container");
  }

  void send_tts_request(const std::string & text)
  {
    if (!audio_client_->wait_for_service(1s)) {
      RCLCPP_ERROR(this->get_logger(), "Service /play_audio is not available.");
      return;
    }

    auto request = std::make_shared<g1_srvs::srv::AudioClient::Request>();
    request->type = g1_srvs::srv::AudioClient::Request::TYPE_TTS;
    request->text = text;

    audio_client_->async_send_request(
      request,
      [this](rclcpp::Client<g1_srvs::srv::AudioClient>::SharedFuture future) {
        try {
          auto response = future.get();
          if (response->success) {
            RCLCPP_INFO(this->get_logger(), "Successfully sent TTS request.");
          } else {
            RCLCPP_ERROR(this->get_logger(), "TTS request failed: %s", response->message.c_str());
          }
        } catch (const std::exception & e) {
          RCLCPP_ERROR(this->get_logger(), "TTS service call failed: %s", e.what());
        }
      }
    );
  }

  void call_pose_policy_service(const std::string & pose_mode, std::function<void()> on_complete = nullptr)
  {
    if (!pose_policy_client_->wait_for_service(1s)) {
      RCLCPP_ERROR(this->get_logger(), "Service /pose_policy is not available.");
      if (on_complete) on_complete();
      return;
    }

    auto request = std::make_shared<g1_srvs::srv::PosePolicy::Request>();
    request->pose = pose_mode;

    auto result_future = pose_policy_client_->async_send_request(
      request,
      [this, on_complete](rclcpp::Client<g1_srvs::srv::PosePolicy>::SharedFuture future) {
        try {
          auto response = future.get();
          RCLCPP_INFO(this->get_logger(), "Successfully called /pose_policy service.");
        } catch (const std::exception & e) {
          RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
        }
        
        if (on_complete) {
          on_complete();
        }
      }
    );
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<unitree_go::msg::WirelessController>::SharedPtr wireless_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_active_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr zero_cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr zero_cmd_timer_;
  rclcpp::TimerBase::SharedPtr restart_warning_timer_;
  rclcpp::Client<g1_srvs::srv::PosePolicy>::SharedPtr pose_policy_client_;
  rclcpp::Client<g1_srvs::srv::AudioClient>::SharedPtr audio_client_;
  
  bool prev_button_state_;
  bool is_processing_;    
  bool is_armed_;
  bool initial_warning_sent_;
  bool safety_active_;
  bool restart_warning_active_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EmergencyStopNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
