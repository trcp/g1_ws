#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>

#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace
{
using Clock = std::chrono::steady_clock;
using Enable = std_srvs::srv::SetBool;
using ListControllers = controller_manager_msgs::srv::ListControllers;

int request_control(const std::shared_ptr<rclcpp::Node> & node)
{
  const double wait_sec = node->declare_parameter<double>("wait_timeout_sec", 30.0);
  const double response_sec = node->declare_parameter<double>("response_timeout_sec", 10.0);
  if (!std::isfinite(wait_sec) || wait_sec <= 0.0 ||
    !std::isfinite(response_sec) || response_sec <= 0.0)
  {
    throw std::invalid_argument("待機時間は正の有限値にしてください");
  }
  if (node->has_parameter("use_sim_time") &&
    node->get_parameter("use_sim_time").as_bool())
  {
    RCLCPP_ERROR(node->get_logger(), "ROS シミュレーション時刻では実機制御権を取得しません");
    return 1;
  }

  // HW を経由することで、取得成功時にだけ関節指令の送信を有効にする。
  const auto enable = node->create_client<Enable>("/enable_upper_body_control");
  const auto authority = node->create_client<Enable>("/robot_controller/upper_body/enable");
  const auto controllers = node->create_client<ListControllers>(
    "/controller_manager/list_controllers");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  const auto deadline = Clock::now() +
    std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(wait_sec));
  bool active = false;
  RCLCPP_INFO(node->get_logger(), "上半身コントローラと取得サービスの起動を待ちます");
  while (rclcpp::ok() && Clock::now() < deadline) {
    if (enable->service_is_ready() && authority->service_is_ready() &&
      controllers->service_is_ready())
    {
      auto pending = controllers->async_send_request(std::make_shared<ListControllers::Request>());
      const auto remaining = deadline - Clock::now();
      const auto timeout = std::max(Clock::duration::zero(),
        std::min(remaining, std::chrono::duration_cast<Clock::duration>(std::chrono::seconds(2))));
      const auto result = executor.spin_until_future_complete(pending, timeout);
      if (result == rclcpp::FutureReturnCode::SUCCESS) {
        const auto response = pending.get();
        active = std::any_of(response->controller.begin(), response->controller.end(),
          [](const auto & controller) {
            return controller.name == "upper_body_controller" && controller.state == "active";
          });
        if (active) {break;}
      } else {
        controllers->remove_pending_request(pending.request_id);
        if (result == rclcpp::FutureReturnCode::INTERRUPTED) {return 1;}
      }
    }
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  if (!active || !rclcpp::ok()) {
    RCLCPP_ERROR(node->get_logger(),
      "起動待ちが終了しました。取得要求は送信していません。"
      "robot_controller の起動設定と upper_body_controller の状態を確認してください");
    return 1;
  }

  // 取得は一度だけ。Arm Action の解放や E-Stop の解除を自動要求しない。
  auto request = std::make_shared<Enable::Request>();
  request->data = true;
  auto pending = enable->async_send_request(request);
  const auto result = executor.spin_until_future_complete(
    pending, std::chrono::duration<double>(response_sec));
  if (result != rclcpp::FutureReturnCode::SUCCESS) {
    enable->remove_pending_request(pending.request_id);
    RCLCPP_ERROR(node->get_logger(),
      "取得要求の結果を確認できませんでした。再送せず終了します。制御状態を確認してください");
    return 1;
  }
  const auto response = pending.get();
  if (!response->success) {
    RCLCPP_ERROR(node->get_logger(), "上半身制御権の取得が拒否されました: %s",
      response->message.c_str());
    return 1;
  }
  RCLCPP_INFO(node->get_logger(),
    "HW の上半身制御が許可されました: %s。関節出力による取得処理へ進みます",
    response->message.c_str());
  return 0;
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 1;
  try {
    result = request_control(std::make_shared<rclcpp::Node>("enable_upper_body_control"));
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("enable_upper_body_control"), "%s", error.what());
  }
  rclcpp::shutdown();
  return result;
}
