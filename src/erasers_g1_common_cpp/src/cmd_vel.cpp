#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "g1/g1_loco_client.hpp"
#include "common/ut_errror.hpp"

#include <memory>
#include <functional>
#include <cmath>
#include <mutex>
#include <chrono>
#include <atomic>

using namespace std::chrono_literals;

class CmdVelSubscriberNode : public rclcpp::Node {
 public:
  CmdVelSubscriberNode()
      : Node("g1_cmd_vel_subscriber_node"),
        client_(this), 
        last_cmd_vel_received_time_(this->now()),
        watchdog_timeout_(500ms) // 0.5秒間 cmd_vel が来なければ停止
  {
    // --- 重要: コールバックグループの設定 ---
    
    // 1. cmd_vel 受信用 (並列実行許可: Reentrant)
    callback_group_subscriber_ = this->create_callback_group(
        rclcpp::CallbackGroupType::Reentrant);

    // 2. 制御ループタイマー用 (排他制御: MutuallyExclusive)
    // これを独立させることで、API呼び出し中(Wait中)でも、
    // Nodeのデフォルトグループにある UnitreeClient の受信処理が動けるようにする
    callback_group_timer_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    // --- サブスクライバーの設定 ---
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = callback_group_subscriber_;

    cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel",
        10,
        std::bind(&CmdVelSubscriberNode::cmdVelCallback, this, std::placeholders::_1),
        sub_opt);

    // --- タイマーの設定 ---
    // 第3引数に callback_group_timer_ を指定するのが重要
    control_timer_ = this->create_wall_timer(
        50ms, 
        std::bind(&CmdVelSubscriberNode::controlLoopCallback, this),
        callback_group_timer_);

    // --- 初期化 ---
    // 通信確立待ちのため少しスリープ
    std::this_thread::sleep_for(std::chrono::seconds(1)); 
    
    // 連続移動モードに設定
    int32_t ret = client_.SwitchMoveMode(true);
    if (ret == 0) {
      RCLCPP_INFO(this->get_logger(), "SwitchMoveMode set to true.");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to set SwitchMoveMode. Ret: %d", ret);
    }

    RCLCPP_INFO(this->get_logger(), "G1 cmd_vel node ready. Control Loop: 20Hz");
  }

 private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    target_vx_ = msg->linear.x;
    target_vy_ = msg->linear.y;
    target_omega_ = msg->angular.z;
    last_cmd_vel_received_time_ = this->now();
  }

  void controlLoopCallback() {
    float vx, vy, omega;
    bool is_timeout = false;

    // 現在時刻と最終受信時刻を比較
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      auto elapsed = this->now() - last_cmd_vel_received_time_;
      if (elapsed > watchdog_timeout_) {
        is_timeout = true;
        vx = 0.0f; vy = 0.0f; omega = 0.0f;
      } else {
        vx = target_vx_;
        vy = target_vy_;
        omega = target_omega_;
      }
    }

    int32_t ret = 0;

    if (is_timeout) {
      // タイムアウト時：まだ停止処理が成功していない場合のみ StopMove を送る
      if (!is_stopped_) {
         RCLCPP_WARN(this->get_logger(), "Cmd_vel timeout. Stopping robot.");
         ret = client_.StopMove();
         
         if(ret == 0) {
             is_stopped_ = true; // 停止成功
             RCLCPP_INFO(this->get_logger(), "Robot stopped successfully.");
         } else {
             // 失敗時はログを出すが、次のループでも再試行される
             RCLCPP_ERROR(this->get_logger(), "StopMove failed: %d", ret);
         }
      }
    } else {
      // 通常走行時
      // 速度がほぼゼロの場合も Move(0,0,0) を送り続ける（SwitchMoveMode=true なので安全）
      if (std::abs(vx) < 0.01 && std::abs(vy) < 0.01 && std::abs(omega) < 0.01) {
         ret = client_.Move(0.0, 0.0, 0.0);
         // Move(0,0,0) が成功すれば停止状態とみなす
         if (ret == 0) is_stopped_ = true; 
      } else {
         ret = client_.Move(vx, vy, omega);
         is_stopped_ = false;
      }

      if (ret != 0) {
        RCLCPP_WARN(this->get_logger(), "Move API Call failed: %d", ret);
      }
    }
  }

  // ROS メンバ
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
  rclcpp::CallbackGroup::SharedPtr callback_group_subscriber_;
  rclcpp::CallbackGroup::SharedPtr callback_group_timer_; // ★追加
  rclcpp::TimerBase::SharedPtr control_timer_;
  
  // Unitree Client
  unitree::robot::g1::LocoClient client_;

  // 内部変数
  std::mutex cmd_mutex_;
  float target_vx_{0.0f};
  float target_vy_{0.0f};
  float target_omega_{0.0f};
  rclcpp::Time last_cmd_vel_received_time_;
  rclcpp::Duration watchdog_timeout_;
  bool is_stopped_{true};
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CmdVelSubscriberNode>();

  // MultiThreadedExecutor が必要
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
