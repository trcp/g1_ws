#include <chrono>
#include <g1/g1_loco_client.hpp>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "common/ut_errror.hpp"
#include "rclcpp/rclcpp.hpp"
// サービスヘッダファイルをインクルード
#include "g1_srvs/srv/pose_policy.hpp"

/**
 * @brief Unitree G1ロボットの姿勢制御サービスを提供するROS 2ノード
 *
 * "pose_policy" サービス (g1_srvs/srv/PosePolicy) を提供します。
 * サービスリクエストで指定された姿勢名に基づき、g1::LocoClient を介して
 * ロボットにコマンドを送信します。
 */
class LocoServiceClientNode : public rclcpp::Node {
 public:
  // サービス型のエイリアス
  using PosePolicy = g1_srvs::srv::PosePolicy;

  /**
   * @brief コンストラクタ
   */
  explicit LocoServiceClientNode()
      : Node("loco_service_client"), client_(this) {
    // "pose_policy" という名前でサービスサーバーを作成
    service_ = this->create_service<PosePolicy>(
        "pose_policy",
        std::bind(&LocoServiceClientNode::handle_pose_request, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(),
                "Loco Service Client Node started. Ready to receive pose "
                "commands on 'pose_policy' service.");
  }

  /**
   * @brief g1::LocoClient が返すエラーコードを処理します。
   *
   * @param error_code LocoClientから返されたエラーコード
   * @return true 成功 (error_code == 0) の場合
   * @return false 失敗 (error_code != 0) の場合
   */
  bool handleActionError(int32_t error_code) {
    if (error_code == 0) {
      return true; // 成功
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
  /**
   * @brief "pose_policy" サービスのリクエストを処理するコールバック関数
   *
   * @param request サービスリクエスト (実行したい姿勢名を含む)
   * @param response サービスレスポンス (成功/失敗フラグ)
   */
  void handle_pose_request(
      const std::shared_ptr<PosePolicy::Request> request,
      std::shared_ptr<PosePolicy::Response> response) {
    RCLCPP_INFO(this->get_logger(), "Received pose request: [%s]",
                request->pose.c_str());

    int32_t ret = -1; // LocoClientの戻り値 (エラーコード) を格納
    const std::string& pose = request->pose;

    // リクエストされた姿勢名に基づいて、対応するクライアントメソッドを呼び出す
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
    } else if (pose == PosePolicy::Request::ZERO_RORQUE) {
      // サービス定義ファイル (.srv) のタイポ "ZERO_RORQUE" に合わせる
      ret = client_.ZeroTorque();
    } else {
      // サービスで定義されていない姿勢名が来た場合
      RCLCPP_WARN(this->get_logger(), "Unknown pose command received: [%s]",
                  pose.c_str());
      response->success = false;
      return;
    }

    // LocoClientからの戻り値をチェックし、サービスの成功/失敗を設定
    response->success = handleActionError(ret);

    if (response->success) {
      RCLCPP_INFO(this->get_logger(), "Successfully executed pose: [%s]",
                  pose.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to execute pose: [%s]",
                   pose.c_str());
    }
  }

  // Unitree G1 ロコモーションクライアント
  unitree::robot::g1::LocoClient client_;
  // ROS 2 サービスサーバー
  rclcpp::Service<PosePolicy>::SharedPtr service_;
};

/**
 * @brief main関数
 */
int main(int argc, char const* argv[]) {
  // ROS 2 の初期化
  rclcpp::init(argc, argv);

  // LocoServiceClientNode のインスタンスを作成
  auto loco_service_client_node =
      std::make_shared<LocoServiceClientNode>();

  // ノードをスピンさせ、サービスリクエストを待機
  rclcpp::spin(loco_service_client_node);

  // シャットダウン
  rclcpp::shutdown();
  return 0;
}
