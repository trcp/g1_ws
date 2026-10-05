// Copyright (c) 2026 Nakatogawa Laboratory. All rights reserved.

#ifndef ERASERS_G1_COMMON__ROBOT_SERVICE_INTERFACE_CLIENT_HPP_
#define ERASERS_G1_COMMON__ROBOT_SERVICE_INTERFACE_CLIENT_HPP_

#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <erasers_g1_interfaces/srv/robot_service_client.hpp>

#include "erasers_g1_common/robot_state_direct_client.hpp"

namespace erasers_g1_common
{

class RobotServiceInterfaceClientNode : public rclcpp::Node
{
public:
  explicit RobotServiceInterfaceClientNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  RobotServiceInterfaceClientNode(
    std::shared_ptr<RobotStateClientInterface> robot_state_client,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // 同期的なサービス一覧更新（初期化時およびオンデマンド更新で使用）
  bool refresh_services(bool emit_error_log = true);

  // 内部キャッシュからサービス名一覧を取得
  std::vector<std::string> get_cached_service_names() const;

private:
  using RobotServiceClient = erasers_g1_interfaces::srv::RobotServiceClient;
  using SteadyTimePoint = std::chrono::steady_clock::time_point;

  void initialize(std::shared_ptr<RobotStateClientInterface> injected_client);

  // 初期化シーケンス（初期ディスカバリの義務付け）
  void perform_initial_discovery();

  // 単一サービスサーバーのコールバック
  void handle_service_request(
    const std::shared_ptr<RobotServiceClient::Request> request,
    std::shared_ptr<RobotServiceClient::Response> response);

  // 一覧取得モード処理（オンデマンド最新化 + \n 区切りフォーマット）
  void handle_list_services_request(
    std::shared_ptr<RobotServiceClient::Response> response);

  // 通常操作モード処理（シンプル切り替え）
  void handle_switch_service_request(
    const std::string & service_name,
    bool enable,
    std::shared_ptr<RobotServiceClient::Response> response);

  bool switch_service_locked(
    const std::string & service_name,
    bool desired_enabled,
    RobotServiceState & final_state,
    std::string & result_message);

  bool verify_service_state(
    const std::string & service_name,
    bool desired_enabled,
    const SteadyTimePoint & deadline,
    RobotServiceState & verified_state,
    std::string & error_message);

  void update_service_states(const std::vector<RobotServiceState> & services);
  bool get_service_state(const std::string & service_name, RobotServiceState & state) const;

  std::shared_ptr<RobotStateClientInterface> robot_state_client_;

  // 動作調整パラメータ
  double api_timeout_sec_{10.0};
  double refresh_period_sec_{30.0};
  bool verify_after_switch_{true};
  double switch_settle_delay_sec_{5.0};
  double switch_timeout_sec_{10.0};
  double verify_poll_period_sec_{0.5};

  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  rclcpp::CallbackGroup::SharedPtr discovery_callback_group_;
  rclcpp::TimerBase::SharedPtr initial_discovery_timer_;
  rclcpp::TimerBase::SharedPtr refresh_timer_;

  // 固定サービス名 "robot_service_interface" のサーバー
  rclcpp::Service<RobotServiceClient>::SharedPtr service_server_;

  mutable std::mutex registry_mutex_;
  std::mutex operation_mutex_;
  std::map<std::string, RobotServiceState> service_states_;
  bool initial_discovery_done_{false};
};

}  // namespace erasers_g1_common

#endif  // ERASERS_G1_COMMON__ROBOT_SERVICE_INTERFACE_CLIENT_HPP_
