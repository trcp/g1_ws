#include <chrono>
#include <g1/g1_loco_client.hpp>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <algorithm>

#include "common/ut_errror.hpp"
#include "rclcpp/rclcpp.hpp"
#include "g1_srvs/srv/pose_policy.hpp"
#include "g1_srvs/srv/audio_client.hpp"

class LocoServiceClientNode : public rclcpp::Node {
 public:
  using PosePolicy = g1_srvs::srv::PosePolicy;
  using AudioClient = g1_srvs::srv::AudioClient;

  /**
   * @brief コンストラクタ
   */
  explicit LocoServiceClientNode()
      : Node("loco_service_client"), client_(this) {
    service_ = this->create_service<PosePolicy>(
        "pose_policy",
        std::bind(&LocoServiceClientNode::handle_pose_request, this,
                  std::placeholders::_1, std::placeholders::_2));

    audio_client_ptr_ = this->create_client<AudioClient>("/play_audio");

    RCLCPP_INFO(this->get_logger(),
                "Loco Service Client Node started. Ready to receive pose "
                "commands on 'pose_policy' service.");
  }

  bool handleActionError(int32_t error_code) {
    if (error_code == 0) {
      return true;
    }

    if (error_code == -1) { // 一般的なタイムアウト/失敗コード
        RCLCPP_WARN(this->get_logger(), 
            "Action returned code %d (Timeout). Assuming success as robot is moving.", error_code);
        return true; 
    }

    // エラーの場合は詳細をログに出力
    RCLCPP_ERROR(this->get_logger(), "Execute action failed, error code: %d",
                 error_code);
    UT_PRINT_ERR(error_code,
                 unitree::robot::g1::UT_ROBOT_LOCO_ERR_LOCOSTATE_NOT_AVAILABLE);
    UT_PRINT_ERR(error_code,
                 unitree::robot::g1::UT_ROBOT_LOCO_ERR_INVALID_FSM_ID);
    UT_PRINT_ERR(error_code,
                 unitree::robot::g1::UT_ROBOT_LOCO_ERR_INVALID_TASK_ID);
    UT_PRINT_ERR(error_code, UT_ROBOT_TASK_TIMEOUT);
    return false; // 失敗
  }

 private:
  void send_speech_request(const std::string& text) {
      if (!audio_client_ptr_->service_is_ready()) {
          RCLCPP_WARN(this->get_logger(), "Audio service is not available. Skipping speech.");
          return;
      }

      auto request = std::make_shared<AudioClient::Request>();
      request->type = 0; // TTS mode
      request->text = text; 
      request->audio_path = "";

      // 非同期で呼び出す
      audio_client_ptr_->async_send_request(request);
  }
  
  void handle_pose_request(
      const std::shared_ptr<PosePolicy::Request> request,
      std::shared_ptr<PosePolicy::Response> response) {
    RCLCPP_INFO(this->get_logger(), "Received pose request: [%s]",
                request->pose.c_str());

    int32_t ret = -1; 
    const std::string& pose = request->pose;
    bool reset_shake_state = true;

    if (pose == PosePolicy::Request::DAMP) {
      ret = client_.Damp();
    } else if (pose == PosePolicy::Request::START) {
      ret = client_.Start();
    } else if (pose == PosePolicy::Request::SQUAT) {
      ret = client_.Squat();
    } else if (pose == PosePolicy::Request::SIT) {
      ret = client_.Sit();
    } else if (pose == PosePolicy::Request::STAND_UP) {
      ret = client_.StandUp();
    } else if (pose == PosePolicy::Request::ZERO_TORQUE) {
      ret = client_.ZeroTorque();
    } else if (pose == PosePolicy::Request::STOP_MOVE) {
      ret = client_.StopMove();
    } else if (pose == PosePolicy::Request::HIGH_STAND) {
      ret = client_.HighStand();
    } else if (pose == PosePolicy::Request::LOW_STAND) {
      ret = client_.LowStand();
    } else if (pose == PosePolicy::Request::BALANCE_STAND) {
      ret = client_.BalanceStand();
    } else if (pose == PosePolicy::Request::SHAKE_HAND) {
      // ShakeHandのトグル動作は維持
      reset_shake_state = false;
      if (is_shaking_hands_) {
        RCLCPP_INFO(this->get_logger(), "Already shaking hands. Stopping (ShakeHand(1)).");
        ret = client_.ShakeHand(1);
        if (handleActionError(ret)) {
             is_shaking_hands_ = false;
             ret = 0;
        }
      } else {
        RCLCPP_INFO(this->get_logger(), "Starting ShakeHand(0).");
        ret = client_.ShakeHand(0);
        if (handleActionError(ret)) {
             is_shaking_hands_ = true;
             ret = 0;
        }
      }
    } else if (pose == PosePolicy::Request::WAVE_HAND) {
      ret = client_.WaveHand();
    } else if (pose == PosePolicy::Request::WAVE_HAND_WITH_TURN) {
      ret = client_.WaveHand(true);
    } else {
      RCLCPP_WARN(this->get_logger(), "Unknown pose command received: [%s]",
                  pose.c_str());
      response->success = false;
      return;
    }

    if (reset_shake_state) {
        is_shaking_hands_ = false;
    }

    response->success = handleActionError(ret);

    if (response->success) {
      RCLCPP_INFO(this->get_logger(), "Successfully executed pose: [%s]",
                  pose.c_str());
      
      if (pose.find("hand") == std::string::npos) {
          std::string speech_text = pose;
          std::replace(speech_text.begin(), speech_text.end(), '_', ' ');
          speech_text = speech_text + " mode.";
          send_speech_request(speech_text);
      } else {
          RCLCPP_INFO(this->get_logger(), "Skipping speech for pose with 'hand': [%s]", pose.c_str());
      }
      
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to execute pose: [%s]",
                   pose.c_str());
      send_speech_request("Failed to change pose.");
    }
  }

  unitree::robot::g1::LocoClient client_;
  rclcpp::Service<PosePolicy>::SharedPtr service_;
  rclcpp::Client<AudioClient>::SharedPtr audio_client_ptr_;
  
  bool is_shaking_hands_ = false;
};

int main(int argc, char const* argv[]) {
  rclcpp::init(argc, argv);
  auto loco_service_client_node =
      std::make_shared<LocoServiceClientNode>();
  rclcpp::spin(loco_service_client_node);
  rclcpp::shutdown();
  return 0;
}
