#ifndef ERASERS_G1_COMMON__ROBOT_CONTROLLER_HPP_
#define ERASERS_G1_COMMON__ROBOT_CONTROLLER_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <erasers_g1_interfaces/srv/arm_action.hpp>
#include <erasers_g1_interfaces/srv/robot_pose.hpp>
#include <erasers_g1_interfaces/srv/pose_policy.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <unitree_api/msg/request.hpp>
#include <unitree_hg/msg/low_state.hpp>

#include "erasers_g1_common/arm_action_direct_client.hpp"
#include "erasers_g1_common/g1_arm_sdk_protocol.hpp"
#include "erasers_g1_common/g1_loco_client.hpp"
#include "erasers_g1_common/led_client_interface.hpp"
#include "erasers_g1_common/upper_body_arbiter.hpp"
#include "erasers_g1_common/upper_body_control_session.hpp"

namespace erasers_g1_common
{

struct CommandTarget
{
  float vx{0.0F};
  float vy{0.0F};
  float omega{0.0F};
  bool is_stop{true};
  bool priority_stop{false};
  uint64_t generation{0};
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
};

class RobotControllerNode : public rclcpp::Node
{
public:
  using SteadyTime = std::chrono::steady_clock::time_point;
  using SteadyNowFunction = std::function<SteadyTime()>;

  explicit RobotControllerNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
    std::shared_ptr<LocoClientInterface> loco_client = nullptr,
    std::shared_ptr<ArmActionClientInterface> arm_action_client = nullptr,
    std::shared_ptr<LowCmdSinkInterface> lowcmd_sink = nullptr,
    SteadyNowFunction steady_now = {},
    UpperBodyPolicy upper_body_policy = UpperBodyPolicy{},
    std::vector<G1JointControlLimit> upper_body_limits = {},
    std::shared_ptr<LedClientInterface> led_client = nullptr);

  ~RobotControllerNode() override;

  void handle_led_command(const std_msgs::msg::Int32MultiArray::SharedPtr msg);

private:
  void publish_transition_active(bool active);
  void poll_fsm_state_timer();

  void handle_robot_pose(
    const std::shared_ptr<erasers_g1_interfaces::srv::RobotPose::Request> request,
    std::shared_ptr<erasers_g1_interfaces::srv::RobotPose::Response> response);

  void transition_to(int32_t target_fsm,
    std::shared_ptr<erasers_g1_interfaces::srv::RobotPose::Response> response);
  void handle_legacy_pose(
    const std::shared_ptr<erasers_g1_interfaces::srv::PosePolicy::Request> request,
    std::shared_ptr<erasers_g1_interfaces::srv::PosePolicy::Response> response);
  void handle_arm_action_deferred(
    const std::shared_ptr<rclcpp::Service<erasers_g1_interfaces::srv::ArmAction>> service,
    const std::shared_ptr<rmw_request_id_t> header,
    const std::shared_ptr<erasers_g1_interfaces::srv::ArmAction::Request> request);
  void process_arm_action(
    int32_t request_mode,
    erasers_g1_interfaces::srv::ArmAction::Response & response);
  void arm_action_state_callback(const std_msgs::msg::String::SharedPtr msg);

  void handle_upper_body_enable_deferred(
    const std::shared_ptr<rclcpp::Service<std_srvs::srv::SetBool>> service,
    const std::shared_ptr<rmw_request_id_t> header,
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request);
  void process_upper_body_enable(
    bool enable,
    bool recovery_attempt,
    const std::string & latched_fault_reason,
    std_srvs::srv::SetBool::Response & response);
  void request_upper_body_disable(std::string & message);
  void upper_joint_command_callback(
    const sensor_msgs::msg::JointState::SharedPtr msg,
    const rclcpp::MessageInfo & info);
  void upper_lowstate_callback(const unitree_hg::msg::LowState::SharedPtr msg);
  void external_arm_request_callback(
    const unitree_api::msg::Request::SharedPtr msg,
    const rclcpp::MessageInfo & info);
  void upper_body_worker_loop();
  void upper_body_fault(const std::string & reason);
  bool validate_upper_joint_command(
    const sensor_msgs::msg::JointState & msg,
    const std::string & source_gid,
    std::vector<std::size_t> & indices,
    std::vector<double> & positions,
    std::string & reason);
  bool upper_body_preconditions(std::string & reason, bool refresh_fsm);
  bool is_arm_normal_locked(SteadyTime now_steady) const;
  bool try_initialize_upper_body_ownership();

  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void emergency_latch_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void emergency_worker_loop();
  void control_loop_timer();
  void enqueue_command(const CommandTarget & target);
  void disable_and_priority_stop_locked(const std::string & reason);
  void enqueue_priority_stop_locked();
  void api_worker_loop();

  bool enqueue_control_job(std::function<void()> job);
  void control_job_worker_loop();
  void shutdown_workers();
  void publish_diagnostics_timer();
  void publish_diagnostics(const rclcpp::Time & stamp);
  static std::string gid_to_string(const rmw_gid_t & gid);

  const double api_timeout_sec_{3.0};
  const double transition_timeout_sec_{15.0};
  const double transition_poll_period_sec_{0.1};
  const double fsm_state_publish_rate_hz_{5.0};
  const bool stop_before_transition_{true};
  const double control_rate_hz_{20.0};
  const double api_update_rate_hz_{10.0};
  const double cmd_vel_timeout_sec_{0.25};
  const double velocity_duration_sec_{0.30};
  const double max_linear_x_{1.00};
  const double max_linear_y_{0.50};
  const double max_angular_z_{1.57};
  const double max_linear_accel_{0.50};
  const double max_angular_accel_{1.00};
  const double command_epsilon_{0.001};
  const std::vector<int32_t> allowed_motion_fsm_ids_{500, 501, 801};
  const int max_consecutive_api_failures_{3};

  std::shared_ptr<LocoClientInterface> loco_client_;
  std::shared_ptr<ArmActionClientInterface> arm_action_client_;
  std::shared_ptr<LowCmdSinkInterface> lowcmd_sink_;
  SteadyNowFunction steady_now_;
  UpperBodyControlSession upper_body_session_;
  UpperBodyArbiter upper_body_arbiter_;
  std::vector<G1JointControlLimit> upper_body_limits_;
  bool upper_body_limits_valid_{false};
  std::string upper_body_configuration_error_;

  // Lock order: send_gate_ -> upper_body_mutex_. No other mutex may be held
  // across a transport publish, RPC wait, condition wait, sleep, or join.
  std::mutex send_gate_;
  std::mutex upper_body_mutex_;
  std::atomic<bool> upper_abort_requested_{false};
  std::atomic<bool> stop_fence_complete_{true};
  bool sdk_publish_active_{false};
  uint64_t upper_body_published_count_{0};
  bool release_observation_pending_{false};
  SteadyTime last_send_time_{};
  SteadyTime fault_event_time_{};
  uint64_t fault_event_seq_{0};

  bool lowstate_valid_{false};
  uint32_t lowstate_tick_{0};
  uint64_t duplicate_lowstate_ticks_{0};
  uint64_t lowstate_tick_rebases_{0};
  SteadyTime lowstate_time_{};
  std::array<double, kG1UpperBodyJointCount> measured_q_{};
  std::array<double, kG1UpperBodyJointCount> measured_dq_{};
  SteadyTime tracking_error_since_{};

  uint64_t command_rx_seq_{0};
  std::string command_source_gid_;
  rclcpp::Time last_command_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time session_ros_start_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_ros_now_{0, 0, RCL_ROS_TIME};
  bool upper_transition_active_{false};
  int32_t upper_fsm_id_{-1};
  SteadyTime upper_fsm_time_{};

  std::mutex transition_mutex_;
  std::mutex motion_state_mutex_;
  bool transition_active_{false};
  bool emergency_stop_active_{false};
  bool emergency_stop_received_{false};
  int32_t current_fsm_id_{-1};
  SteadyTime fsm_state_time_{};
  int consecutive_api_failures_{0};
  int last_api_status_code_{0};
  CommandTarget last_raw_cmd_;
  bool has_cmd_vel_sample_{false};
  bool has_sent_motion_command_{false};
  bool last_target_was_zero_{true};
  float curr_slew_vx_{0.0F};
  float curr_slew_vy_{0.0F};
  float curr_slew_omega_{0.0F};
  rclcpp::Time last_control_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_api_enqueue_time_{0, 0, RCL_ROS_TIME};

  std::thread worker_thread_;
  std::atomic<bool> worker_running_{false};
  std::atomic<bool> worker_busy_{false};
  std::mutex mailbox_mutex_;
  std::condition_variable mailbox_cv_;
  CommandTarget mailbox_target_;
  bool mailbox_has_cmd_{false};
  bool priority_stop_pending_{false};
  std::atomic<uint64_t> motion_generation_{0};
  std::mutex motion_send_gate_;
  std::atomic<bool> emergency_latched_{false};
  bool emergency_hold_{false};
  std::thread emergency_thread_;
  std::atomic<bool> emergency_running_{false};
  std::mutex emergency_mutex_;
  std::condition_variable emergency_cv_;
  bool emergency_release_pending_{false};
  std::atomic<uint64_t> emergency_generation_{0};
  std::atomic<int32_t> emergency_release_status_{0};
  bool legacy_shake_holding_{false};
  SteadyTime last_cmd_steady_{};
  int32_t upper_fsm_mode_{-1};
  SteadyTime upper_fsm_mode_time_{};

  std::thread control_job_thread_;
  std::atomic<bool> control_job_running_{false};
  std::atomic<bool> control_job_busy_{false};
  std::mutex control_job_mutex_;
  std::condition_variable control_job_cv_;
  std::function<void()> pending_control_job_;

  std::thread upper_body_thread_;
  std::atomic<bool> upper_body_running_{false};

  std::mutex arm_state_mutex_;
  std::condition_variable arm_state_cv_;
  bool arm_state_valid_{false};
  bool arm_holding_{false};
  int32_t arm_action_id_{0};
  std::string arm_action_name_;
  uint64_t arm_state_seq_{0};
  SteadyTime arm_state_time_{};
  bool arm_state_malformed_{false};
  bool arm_request_pending_{false};
  int32_t pending_arm_request_id_{0};
  uint64_t pending_arm_baseline_seq_{0};
  bool pending_seen_requested_id_{false};
  bool pending_seen_holding_{false};
  bool pending_seen_terminal_zero_{false};
  std::string pending_observed_action_name_;
  bool pending_observed_holding_{false};
  std::vector<int64_t> allow_fsm_ids_{500, 801};

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr fsm_id_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr fsm_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr transition_active_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::CallbackGroup::SharedPtr service_cb_group_;
  rclcpp::CallbackGroup::SharedPtr state_cb_group_;
  rclcpp::CallbackGroup::SharedPtr motion_cb_group_;
  rclcpp::CallbackGroup::SharedPtr emergency_cb_group_;
  rclcpp::Service<erasers_g1_interfaces::srv::RobotPose>::SharedPtr pose_service_;
  rclcpp::Service<erasers_g1_interfaces::srv::PosePolicy>::SharedPtr legacy_pose_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr upper_enable_service_;
  rclcpp::Service<erasers_g1_interfaces::srv::ArmAction>::SharedPtr arm_action_service_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_latch_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr arm_action_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr upper_joint_sub_;
  rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr upper_lowstate_sub_;
  rclcpp::Subscription<unitree_api::msg::Request>::SharedPtr external_arm_request_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr led_sub_;
  std::shared_ptr<LedClientInterface> led_client_;
  rclcpp::TimerBase::SharedPtr fsm_poll_timer_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};

}  // namespace erasers_g1_common

#endif  // ERASERS_G1_COMMON__ROBOT_CONTROLLER_HPP_
