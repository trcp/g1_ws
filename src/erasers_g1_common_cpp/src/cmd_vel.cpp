#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp" // Twist メッセージ (cmd_vel)
#include "g1/g1_loco_client.hpp"       // 提供された G1 Loco Client ヘッダ
#include "common/ut_errror.hpp"      // エラーハンドリング用

#include <memory>
#include <functional> // std::bind と std::placeholders
#include <cmath>      // std::abs
#include <mutex>      // ウォッチドッグタイマーのためのMutex
#include <chrono>     // std::chrono::milliseconds

/**
 * @brief /g1/cmd_vel を購読し、LocoClient を介して
 * 内部的に /api/sport/request へ歩行指令を送信するノード
 *
 * ★ cmd_vel が途切れた場合に自動停止するウォッチドッグ機能付き
 */
class CmdVelSubscriberNode : public rclcpp::Node {
 public:
  CmdVelSubscriberNode()
      : Node("g1_cmd_vel_subscriber_node"),
        client_(this),
        // ★ 修正1: タイムアウトを 500ms (0.5秒) に設定
        watchdog_timeout_(std::chrono::milliseconds(500)),
        is_stopped_by_watchdog_(true) // 初期状態は停止
  {
    // /g1/cmd_vel トピックのサブスクライバ (移動コマンド)
    // ★ 修正2: トピック名を "/g1/cmd_vel" に統一
    cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel",
        10, // QoS
        std::bind(&CmdVelSubscriberNode::cmdVelCallback, this,
                  std::placeholders::_1));

    // ★重要: LocoClient を "連続移動モード" に設定
    int32_t ret = client_.SwitchMoveMode(true);
    if (!handleActionError(ret)) {
      RCLCPP_ERROR(
          this->get_logger(),
          "Failed to set SwitchMoveMode(true). Move commands may not work.");
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "SwitchMoveMode set to true (continuous move).");
    }

    // --- ウォッチドッグタイマーの初期化 ---
    last_cmd_vel_received_time_ = this->now();
    // 100ms ごとにウォッチドッグコールバックを呼び出す
    watchdog_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&CmdVelSubscriberNode::watchdogCallback, this));
    // ---

    RCLCPP_INFO(this->get_logger(), "G1 cmd_vel subscriber node started.");
    // ★ 修正2: ログのトピック名とサブスクライブ名を一致
    RCLCPP_INFO(this->get_logger(), "Subscribing to /g1/cmd_vel (Twist)");
    RCLCPP_INFO(this->get_logger(), "Watchdog enabled: Timeout = %.2f s",
                watchdog_timeout_.seconds());
    RCLCPP_WARN(
        this->get_logger(),
        "Node is in simple forwarding mode. Ensure robot is in a "
        "walkable state (e.g., via 'start' service) before publishing cmd_vel.");
  }

 private:
  /**
   * @brief /g1/cmd_vel (Twist) 受信コールバック
   */
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // --- ウォッチドッグタイマーの更新 ---
    {
      // Mutex で排他制御し、時刻とフラグを更新
      std::lock_guard<std::mutex> lock(time_mutex_);
      last_cmd_vel_received_time_ = this->now();
      is_stopped_by_watchdog_ = false; // コマンド受信中はウォッチドッグ停止フラグを解除
    }
    // ---

    float vx = msg->linear.x;
    float vy = msg->linear.y;
    float omega = msg->angular.z;

    RCLCPP_DEBUG(this->get_logger(),
                 "Received cmd_vel: [vx: %.2f, vy: %.2f, omega: %.2f]", vx, vy,
                 omega);

    int32_t ret;
    // 速度がゼロ近辺なら StopMove() を呼ぶ
    if (std::abs(vx) < 0.01 && std::abs(vy) < 0.01 && std::abs(omega) < 0.01) {
      ret = client_.StopMove();
    } else {
      // ゼロでなければ Move() を呼ぶ
      ret = client_.Move(vx, vy, omega);
    }

    if (!handleActionError(ret)) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to send Move/StopMove command via /api/sport/request.");
    }
  }

  /**
   * @brief ウォッチドッグタイマーのコールバック (100ms ごとに実行)
   */
  void watchdogCallback() {
    std::lock_guard<std::mutex> lock(time_mutex_);

    // 既にウォッチドッグによって停止されている場合は、何もしない
    if (is_stopped_by_watchdog_) {
      return;
    }

    // 最後に cmd_vel を受信してからの経過時間を計算
    auto elapsed = this->now() - last_cmd_vel_received_time_;

    // 経過時間がタイムアウト値 (500ms) を超えた場合
    if (elapsed > watchdog_timeout_) {
      RCLCPP_WARN(this->get_logger(),
                  "cmd_vel timeout (%.2f s > %.2f s). Stopping robot.",
                  elapsed.seconds(), watchdog_timeout_.seconds());

      // ロボットを停止させる
      int32_t ret = client_.StopMove();
      if (!handleActionError(ret)) {
        RCLCPP_ERROR(this->get_logger(),
                     "Watchdog failed to send StopMove command.");
      }

      // ウォッチドッグによって停止したフラグを立てる
      // (StopMove を連続で送り続けないようにするため)
      is_stopped_by_watchdog_ = true;
    }
  }

  /**
   * @brief サンプルコード (loco_client_example.cpp)
   * からコピーしたエラーハンドリング関数
   */
  bool handleActionError(int32_t error_code) {
    if (error_code == 0) {
      return true; // 成功
    }
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

  // --- メンバ変数 ---
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
  unitree::robot::g1::LocoClient client_; // Unitree G1 ロコモーションクライアント

  // --- ウォッチドッグ用メンバ変数 ---
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::Time last_cmd_vel_received_time_;
  rclcpp::Duration watchdog_timeout_;
  bool is_stopped_by_watchdog_;
  std::mutex time_mutex_; // last_cmd_vel_received_time_ と
                          // is_stopped_by_watchdog_ を保護
};

// --- main 関数 ---
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CmdVelSubscriberNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
