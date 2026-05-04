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

class LocoServiceClientNode : public rclcpp::Node {
 public:
  using PosePolicy = g1_srvs::srv::PosePolicy;

  /**
   * @brief コンストラクタ
   */
  explicit LocoServiceClientNode()
      : Node("loco_service_client"), client_(this) {
    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    service_ = this->create_service<PosePolicy>(
        "pose_policy",
        std::bind(&LocoServiceClientNode::handle_pose_request, this,
                  std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        callback_group_);

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
  
  void handle_pose_request(
      const std::shared_ptr<PosePolicy::Request> request,
      std::shared_ptr<PosePolicy::Response> response) {
    RCLCPP_INFO(this->get_logger(), "Received pose request: [%s]",
                request->pose.c_str());

    int32_t ret = -1; 
    const std::string& pose = request->pose;
    bool reset_shake_state = true;
    int target_fsm_id = -1;

    if (pose == PosePolicy::Request::DAMP) {
      ret = client_.Damp();
      target_fsm_id = 1;
    } else if (pose == PosePolicy::Request::START) {
      ret = client_.Start();
      target_fsm_id = 500;
    } else if (pose == PosePolicy::Request::RUNNING) {
      ret = client_.RunningMode();
      target_fsm_id = 801;
    } else if (pose == PosePolicy::Request::SQUAT) {
      ret = client_.SetFsmId(706);
      target_fsm_id = 706;
    } else if (pose == PosePolicy::Request::SIT) {
      ret = client_.Sit();
      target_fsm_id = 3;
    } else if (pose == PosePolicy::Request::STAND_UP) {
      ret = client_.StandUp();
      target_fsm_id = 4;
    } else if (pose == PosePolicy::Request::ZERO_TORQUE) {
      ret = client_.ZeroTorque();
      target_fsm_id = 0;
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

    // 基本命令の成功を確認
    bool success = handleActionError(ret);

    // モード遷移が指定されている場合、遷移完了を待機
    if (success && target_fsm_id != -1) {
      RCLCPP_INFO(this->get_logger(), "Waiting for transition to FSM ID: %d", target_fsm_id);
      ret = client_.WaitFsmId(target_fsm_id);
      if (ret != 0) {
        RCLCPP_ERROR(this->get_logger(), "Transition to FSM ID %d failed or timed out. Current ID might not match.", target_fsm_id);
        success = false;
      } else {
        RCLCPP_INFO(this->get_logger(), "Transition to FSM ID %d completed successfully.", target_fsm_id);
      }
    }

    response->success = success;

    if (response->success) {
      RCLCPP_INFO(this->get_logger(), "Successfully executed pose: [%s]",
                  pose.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to execute pose: [%s]",
                   pose.c_str());
    }
  }

  unitree::robot::g1::LocoClient client_;
  rclcpp::Service<PosePolicy>::SharedPtr service_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  
  bool is_shaking_hands_ = false;
};

int main(int argc, char const* argv[]) {
  rclcpp::init(argc, argv);
  auto loco_service_client_node =
      std::make_shared<LocoServiceClientNode>();
  
  // Use MultiThreadedExecutor to allow concurrent execution of service and topic callbacks
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(loco_service_client_node);
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}
