// Copyright (c) 2026 Nakatogawa Laboratory. All rights reserved.

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <erasers_g1_interfaces/action/vui_tts.hpp>
#include <erasers_g1_interfaces/action/vui_audio.hpp>
#include <erasers_g1_common/unitree_api_client.hpp>
#include <erasers_g1_common/audio_playback_monitor.hpp>
#include <erasers_g1_common/mic_udp_receiver.hpp>
#include <erasers_g1_common/mic_control_policy.hpp>
#include <erasers_g1_common/fan_noise_reducer.hpp>
#include <erasers_g1_common/wav_audio_converter.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <nlohmann/json.hpp>
#include <mutex>
#include <atomic>
#include <thread>
#include <chrono>
#include <sys/stat.h>
#include <fstream>
#include <memory>
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <arpa/inet.h>

namespace
{

bool is_absolute_path(const std::string & path)
{
  return !path.empty() && path[0] == '/';
}

bool file_exists_and_is_regular(const std::string & path)
{
  struct stat st;
  if (stat(path.c_str(), &st) != 0) {
    return false;
  }
  return S_ISREG(st.st_mode);
}

} // namespace

namespace erasers_g1_common
{

enum class MicState
{
  DISABLED,
  ENABLING,
  ENABLED,
  DISABLING,
  ERROR
};

const char * mic_state_label(MicState state)
{
  switch (state) {
    case MicState::DISABLED:
      return "DISABLED";
    case MicState::ENABLING:
      return "WAITING_FOR_FRESH_PACKET";
    case MicState::ENABLED:
      return "ENABLED";
    case MicState::DISABLING:
      return "STOPPING";
    case MicState::ERROR:
      return "ERROR";
  }
  return "UNKNOWN";
}

class VuiClientNode : public rclcpp::Node
{
public:
  using VuiTTS = erasers_g1_interfaces::action::VuiTTS;
  using GoalHandleVuiTTS = rclcpp_action::ServerGoalHandle<VuiTTS>;

  using VuiAudio = erasers_g1_interfaces::action::VuiAudio;
  using GoalHandleVuiAudio = rclcpp_action::ServerGoalHandle<VuiAudio>;

  explicit VuiClientNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("vui_client", options),
    tts_index_(0),
    audio_index_(0),
    is_executing_(false),
    mic_state_(MicState::DISABLED)
  {
    // Declare general parameters
    this->declare_parameter<std::string>("request_topic", "/api/voice/request");
    this->declare_parameter<std::string>("response_topic", "/api/voice/response");
    this->declare_parameter<std::string>("audio_msg_topic", "/audio_msg");
    this->declare_parameter<double>("timeout_sec", 10.0);

    // Audio/TTS specific parameters
    this->declare_parameter<std::string>("audio_app_name", "erasers_g1_audio");
    this->declare_parameter<int64_t>("audio_chunk_size", 96000);
    this->declare_parameter<double>("audio_chunk_interval_sec", 1.0);
    this->declare_parameter<int64_t>("audio_max_input_bytes", 10485760); // 10MB
    this->declare_parameter<std::string>("audio_play_stop_key_mode", "stream_id");
    this->declare_parameter<double>("playback_start_timeout_sec", 5.0);
    this->declare_parameter<double>("tts_playback_finish_timeout_sec", 30.0);
    this->declare_parameter<double>("audio_playback_finish_grace_sec", 10.0);
    this->declare_parameter<double>("audio_stop_timeout_sec", 3.0);
    this->declare_parameter<double>("audio_stop_state_timeout_sec", 3.0);
    this->declare_parameter<double>("playback_feedback_period_sec", 0.25);
    this->declare_parameter<double>("playback_completion_quiet_sec", 2.0);

    // Microphone specific parameters
    this->declare_parameter<std::string>("mic_service_name", "/enable_mic");
    this->declare_parameter<std::string>("mic_topic_name", "/mic_data");
    this->declare_parameter<std::string>("mic_network_interface", "");
    this->declare_parameter<std::string>("mic_multicast_group", "239.168.123.161");
    this->declare_parameter<int64_t>("mic_multicast_port", 5555);
    this->declare_parameter<double>("mic_start_timeout_sec", 5.0);
    this->declare_parameter<int64_t>("mic_receive_timeout_ms", 500);
    this->declare_parameter<int64_t>("mic_receive_buffer_bytes", 65536);
    this->declare_parameter<double>("mic_no_data_warn_sec", 3.0);
    this->declare_parameter<int64_t>("mic_qos_depth", 10);
    this->declare_parameter<std::string>("mic_remote_api_policy", "disabled");
    this->declare_parameter<bool>("noise_canceling", false);
    this->declare_parameter<int64_t>("fan_noise_sample_rate", 16000);
    this->declare_parameter<int64_t>("fan_noise_fft_size", 512);
    this->declare_parameter<int64_t>("fan_noise_hop_length", 256);
    this->declare_parameter<double>("noise_reduction_alpha", 1.5);
    this->declare_parameter<double>("noise_reduction_min_gain", 0.20);
    this->declare_parameter<double>("fan_noise_rms_dbfs", -120.0);
    this->declare_parameter<std::vector<double>>("fan_noise_psd", std::vector<double>{});

    // Validate playback_completion_quiet_sec immediately
    double quiet_sec = this->get_parameter("playback_completion_quiet_sec").as_double();
    if (!std::isfinite(quiet_sec) || quiet_sec <= 0.0) {
      RCLCPP_ERROR(this->get_logger(), "invalid playback_completion_quiet_sec parameter: %f", quiet_sec);
      throw std::invalid_argument("invalid playback_completion_quiet_sec");
    }

    // Validate microphone parameters
    std::string mcast_group = this->get_parameter("mic_multicast_group").as_string();
    int64_t mcast_port = this->get_parameter("mic_multicast_port").as_int();
    int64_t recv_timeout = this->get_parameter("mic_receive_timeout_ms").as_int();
    int64_t recv_buffer = this->get_parameter("mic_receive_buffer_bytes").as_int();
    double start_timeout = this->get_parameter("mic_start_timeout_sec").as_double();
    std::string policy_str = this->get_parameter("mic_remote_api_policy").as_string();
    const bool noise_canceling = this->get_parameter("noise_canceling").as_bool();

    struct in_addr addr;
    if (inet_pton(AF_INET, mcast_group.c_str(), &addr) != 1) {
      RCLCPP_ERROR(this->get_logger(), "invalid mic_multicast_group: %s", mcast_group.c_str());
      throw std::invalid_argument("invalid mic_multicast_group");
    }
    if (mcast_port <= 0 || mcast_port > 65535) {
      RCLCPP_ERROR(this->get_logger(), "invalid mic_multicast_port: %ld", mcast_port);
      throw std::invalid_argument("invalid mic_multicast_port");
    }
    if (recv_timeout <= 0) {
      RCLCPP_ERROR(this->get_logger(), "invalid mic_receive_timeout_ms: %ld", recv_timeout);
      throw std::invalid_argument("invalid mic_receive_timeout_ms");
    }
    if (recv_buffer < 2 || recv_buffer > 65536) {
      RCLCPP_ERROR(this->get_logger(), "invalid mic_receive_buffer_bytes: %ld", recv_buffer);
      throw std::invalid_argument("invalid mic_receive_buffer_bytes");
    }
    if (start_timeout <= 0.0) {
      RCLCPP_ERROR(this->get_logger(), "invalid mic_start_timeout_sec: %f", start_timeout);
      throw std::invalid_argument("invalid mic_start_timeout_sec");
    }
    if (policy_str != "disabled" && policy_str != "best_effort" && policy_str != "required") {
      RCLCPP_ERROR(this->get_logger(), "invalid mic_remote_api_policy parameter: %s", policy_str.c_str());
      throw std::invalid_argument("invalid mic_remote_api_policy");
    }
    if (noise_canceling) {
      FanNoiseProfile profile;
      profile.sample_rate = static_cast<int>(this->get_parameter("fan_noise_sample_rate").as_int());
      profile.fft_size = static_cast<int>(this->get_parameter("fan_noise_fft_size").as_int());
      profile.hop_length = static_cast<int>(this->get_parameter("fan_noise_hop_length").as_int());
      profile.alpha = this->get_parameter("noise_reduction_alpha").as_double();
      profile.min_gain = this->get_parameter("noise_reduction_min_gain").as_double();
      profile.psd = this->get_parameter("fan_noise_psd").as_double_array();
      mic_noise_reducer_ = std::make_unique<FanNoiseReducer>(std::move(profile));
    }

    if (policy_str == "disabled") {
      remote_api_policy_ = CloseApiPolicy::DISABLED;
    } else if (policy_str == "best_effort") {
      remote_api_policy_ = CloseApiPolicy::BEST_EFFORT;
    } else {
      remote_api_policy_ = CloseApiPolicy::REQUIRED;
    }

    std::string req_topic = this->get_parameter("request_topic").as_string();
    std::string res_topic = this->get_parameter("response_topic").as_string();
    std::string audio_msg_topic = this->get_parameter("audio_msg_topic").as_string();

    // Initialize UnitreeApiClient
    api_client_ = std::make_shared<UnitreeApiClient>(this, req_topic, res_topic);

    // Initialize AudioPlaybackMonitor
    monitor_ = std::make_shared<AudioPlaybackMonitor>(this, audio_msg_topic);

    // Create Action Servers
    tts_action_server_ = rclcpp_action::create_server<VuiTTS>(
      this,
      "/vui_tts",
      std::bind(&VuiClientNode::handle_tts_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&VuiClientNode::handle_tts_cancel, this, std::placeholders::_1),
      std::bind(&VuiClientNode::handle_tts_accepted, this, std::placeholders::_1)
    );

    audio_action_server_ = rclcpp_action::create_server<VuiAudio>(
      this,
      "/vui_audio",
      std::bind(&VuiClientNode::handle_audio_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&VuiClientNode::handle_audio_cancel, this, std::placeholders::_1),
      std::bind(&VuiClientNode::handle_audio_accepted, this, std::placeholders::_1)
    );

    // Create Mic Publisher
    int64_t mic_depth = this->get_parameter("mic_qos_depth").as_int();
    std::string mic_topic = this->get_parameter("mic_topic_name").as_string();
    mic_pub_ = this->create_publisher<std_msgs::msg::Int16MultiArray>(
      mic_topic,
      rclcpp::QoS(mic_depth)
    );

    // Create Mic Service Server
    std::string mic_service = this->get_parameter("mic_service_name").as_string();
    mic_service_callback_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    mic_service_server_ = this->create_service<std_srvs::srv::SetBool>(
      mic_service,
      std::bind(&VuiClientNode::handle_mic_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default,
      mic_service_callback_group_
    );

    RCLCPP_INFO(this->get_logger(), "vui_client node initialized");
  }

  ~VuiClientNode() override
  {
    RCLCPP_INFO(this->get_logger(), "vui_client node shutting down");
    shutting_down_ = true;
    if (worker_.joinable()) {worker_.join();}
    mic_publish_enabled_.store(false);
    if (mic_receiver_) {
      mic_receiver_->stop();
    }
  }

  // Inject mocks for unit testing
  void set_unitree_api_client(std::shared_ptr<UnitreeApiClient> client)
  {
    api_client_ = client;
  }

  void set_audio_playback_monitor(std::shared_ptr<AudioPlaybackMonitor> monitor)
  {
    monitor_ = monitor;
  }

  void set_mic_udp_receiver(std::shared_ptr<MicUdpReceiver> receiver)
  {
    mic_receiver_ = receiver;
  }

  uint32_t get_tts_index() const
  {
    return tts_index_.load(std::memory_order_relaxed);
  }

private:
  std::shared_ptr<UnitreeApiClient> api_client_;
  std::shared_ptr<AudioPlaybackMonitor> monitor_;
  std::shared_ptr<MicUdpReceiver> mic_receiver_;
  std::unique_ptr<FanNoiseReducer> mic_noise_reducer_;

  rclcpp_action::Server<VuiTTS>::SharedPtr tts_action_server_;
  rclcpp_action::Server<VuiAudio>::SharedPtr audio_action_server_;

  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr mic_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr mic_service_server_;
  rclcpp::CallbackGroup::SharedPtr mic_service_callback_group_;

  std::atomic<uint32_t> tts_index_;
  std::atomic<uint32_t> audio_index_;

  std::mutex execution_mutex_;
  bool is_executing_;
  std::thread worker_;
  std::atomic<bool> worker_busy_{false};
  std::atomic<bool> shutting_down_{false};

  template<class Action, class Handle, class Execute>
  void start_worker(const std::shared_ptr<Handle> & handle, Execute execute)
  {
    if (worker_.joinable()) {worker_.join();}
    worker_ = std::thread([this, handle, execute]() {
      try {
        (this->*execute)(handle);
      } catch (const std::exception & error) {
        RCLCPP_ERROR(get_logger(), "音声処理が失敗しました: %s", error.what());
        // 終了時もローカル状態を終端化し、GoalHandle の暗黙 abort を残さない。
        try {
          if (handle->is_active()) {
            auto result = std::make_shared<typename Action::Result>();
            result->success = false;
            result->message = error.what();
            handle->abort(result);
          }
        } catch (const std::exception &) {}
      }
      worker_busy_ = false;
    });
  }

  std::mutex mic_mutex_;
  MicState mic_state_;
  CloseApiPolicy remote_api_policy_;
  std::atomic<bool> mic_publish_enabled_{false};

  // TTS Server Callbacks
  rclcpp_action::GoalResponse handle_tts_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const VuiTTS::Goal> goal)
  {
    (void)uuid;
    RCLCPP_DEBUG(this->get_logger(), "Received TTS goal request: text='%s', speaker_id=%d",
      goal->text.c_str(), goal->speaker_id);
    if (shutting_down_ || worker_busy_.exchange(true)) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_tts_cancel(
    const std::shared_ptr<GoalHandleVuiTTS> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Received TTS cancel request");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_tts_accepted(const std::shared_ptr<GoalHandleVuiTTS> goal_handle)
  {
    start_worker<VuiTTS>(goal_handle, &VuiClientNode::execute_tts);
  }

  // Audio Server Callbacks
  rclcpp_action::GoalResponse handle_audio_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const VuiAudio::Goal> goal)
  {
    (void)uuid;
    RCLCPP_DEBUG(this->get_logger(), "Received Audio goal request with source_type: %u, file_path: %s",
      goal->source_type, goal->file_path.c_str());
    if (shutting_down_ || worker_busy_.exchange(true)) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_audio_cancel(
    const std::shared_ptr<GoalHandleVuiAudio> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Received Audio cancel request");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_audio_accepted(const std::shared_ptr<GoalHandleVuiAudio> goal_handle)
  {
    start_worker<VuiAudio>(goal_handle, &VuiClientNode::execute_audio);
  }

  // TTS Execution
  // 起動直後の未発見エンドポイントへの送信を防ぐ。再送による二重再生は行わない。
  template<typename GoalHandle, typename Result>
  bool wait_for_voice_endpoints(
    const std::shared_ptr<GoalHandle> & goal_handle,
    const std::shared_ptr<Result> & result)
  {
    const double timeout = this->get_parameter("timeout_sec").as_double();
    const auto started = std::chrono::steady_clock::now();
    bool waiting_logged = false;
    while (rclcpp::ok() && !shutting_down_ && !goal_handle->is_canceling() &&
      std::isfinite(timeout) && timeout > 0.0)
    {
      if (api_client_->isReady()) {
        if (waiting_logged) {
          RCLCPP_INFO(this->get_logger(), "Voice ROS 2 endpoints connected after %.3f seconds",
            std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count());
        }
        return true;
      }
      if (std::chrono::duration<double>(
          std::chrono::steady_clock::now() - started).count() >= timeout)
      {
        break;
      }
      if (!waiting_logged) {
        RCLCPP_INFO(this->get_logger(), "Waiting for voice ROS 2 request/response endpoints");
        waiting_logged = true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    result->success = false;
    result->message = goal_handle->is_canceling() || shutting_down_ || !rclcpp::ok() ?
      "canceled before voice request publish" : "voice ROS 2 endpoints are not ready";
    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
    } else {
      goal_handle->abort(result);
    }
    return false;
  }

  void execute_tts(const std::shared_ptr<GoalHandleVuiTTS> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing TTS goal");
    
    auto feedback = std::make_shared<VuiTTS::Feedback>();
    auto result = std::make_shared<VuiTTS::Result>();
    
    const auto goal = goal_handle->get_goal();

    // Check busy / concurrent execution
    {
      std::lock_guard<std::mutex> lock(execution_mutex_);
      if (is_executing_) {
        RCLCPP_ERROR(this->get_logger(), "busy: another voice request is active");
        result->success = false;
        result->message = "busy: another voice request is active";
        goal_handle->abort(result);
        return;
      }

      // Check if external playback is already active
      auto initial_snap = monitor_->snapshot();
      if (initial_snap.state_received && initial_snap.last_state == PlayState::PLAYING) {
        RCLCPP_ERROR(this->get_logger(), "existing external playback is active");
        result->success = false;
        result->message = "busy: audio playback is already active";
        goal_handle->abort(result);
        return;
      }

      is_executing_ = true;
    }

    struct ExecutionGuard {
      VuiClientNode * node;
      explicit ExecutionGuard(VuiClientNode * n) : node(n) {}
      ~ExecutionGuard() {
        std::lock_guard<std::mutex> lock(node->execution_mutex_);
        node->is_executing_ = false;
      }
    } exec_guard(this);

    // Initial feedback before action publish
    feedback->time = 0.0f;
    feedback->done = false;
    goal_handle->publish_feedback(feedback);

    // Goal validation
    if (goal->text.empty()) {
      RCLCPP_ERROR(this->get_logger(), "invalid goal: empty text");
      result->success = false;
      result->message = "invalid goal: empty text";
      goal_handle->abort(result);
      return;
    }

    if (goal->speaker_id != VuiTTS::Goal::ENGLISH && goal->speaker_id != VuiTTS::Goal::CHINESE) {
      RCLCPP_ERROR(this->get_logger(), "invalid goal: unsupported speaker_id %d", goal->speaker_id);
      result->success = false;
      result->message = "invalid goal: unsupported speaker_id";
      goal_handle->abort(result);
      return;
    }

    // Check if goal was cancelled before publish
    if (shutting_down_ || goal_handle->is_canceling()) {
      result->success = false;
      result->message = "canceled before request publish";
      if (goal_handle->is_canceling()) {goal_handle->canceled(result);} else {goal_handle->abort(result);}
      return;
    }

    // Retrieve and validate timeout parameters
    double start_timeout = this->get_parameter("playback_start_timeout_sec").as_double();
    double finish_timeout = this->get_parameter("tts_playback_finish_timeout_sec").as_double();
    double fb_period = this->get_parameter("playback_feedback_period_sec").as_double();
    double quiet_sec = this->get_parameter("playback_completion_quiet_sec").as_double();

    if (start_timeout <= 0.0 || finish_timeout <= 0.0 || fb_period <= 0.0 || quiet_sec <= 0.0) {
      RCLCPP_ERROR(this->get_logger(), "invalid timeout parameter");
      result->success = false;
      result->message = "invalid timeout parameter";
      goal_handle->abort(result);
      return;
    }

    auto quiet_duration = std::chrono::milliseconds(static_cast<int64_t>(quiet_sec * 1000.0));

    if (!wait_for_voice_endpoints(goal_handle, result)) {
      return;
    }

    // Record baseline monitor sequence and request start time
    const auto baseline = monitor_->snapshot();
    const auto request_started_at = std::chrono::steady_clock::now();

    // Build parameter JSON with official SDK post-increment index (starting from 0)
    const uint32_t current_index = tts_index_.fetch_add(1, std::memory_order_relaxed);
    nlohmann::json parameter;
    parameter["index"] = current_index;
    parameter["text"] = goal->text;
    parameter["speaker_id"] = goal->speaker_id;
    std::string parameter_str = parameter.dump();

    RCLCPP_INFO(this->get_logger(), "Calling Unitree TTS API (1001) with index: %u, speaker_id: %d, text: '%s'",
      current_index, goal->speaker_id, goal->text.c_str());

    double api_timeout_sec = this->get_parameter("timeout_sec").as_double();
    double start_time = this->now().seconds();
    
    // Perform call (API ID 1001 is called exactly once)
    UnitreeApiClient::CallResult call_res = api_client_->call(1001, parameter_str, api_timeout_sec);

    // Check cancel request during wait
    if (shutting_down_ || goal_handle->is_canceling()) {
      RCLCPP_WARN(this->get_logger(), "TTS cancel after publish; physical stop is not guaranteed");
      feedback->time = static_cast<float>(this->now().seconds() - start_time);
      feedback->done = false;
      goal_handle->publish_feedback(feedback);
      result->success = false;
      result->message = "canceled after publish";
      if (goal_handle->is_canceling()) {goal_handle->canceled(result);} else {goal_handle->abort(result);}
      return;
    }

    if (call_res.timed_out) {
      RCLCPP_ERROR(this->get_logger(), "API timeout/status error: Unitree request timeout after %.2f seconds", api_timeout_sec);
      result->success = false;
      result->message = "timeout waiting for Unitree voice response";
      goal_handle->abort(result);
      return;
    }

    if (!call_res.received) {
      RCLCPP_ERROR(this->get_logger(), "API timeout/status error: Failed to receive voice response");
      result->success = false;
      result->message = "failed to receive voice response";
      goal_handle->abort(result);
      return;
    }

    if (call_res.status_code != 0) {
      RCLCPP_ERROR(this->get_logger(), "API timeout/status error: Unitree API status error: status_code=%d, data=%s",
        call_res.status_code, call_res.data.c_str());
      result->success = false;
      result->message = "Unitree API status error: " + std::to_string(call_res.status_code);
      goal_handle->abort(result);
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Unitree API request accepted (identity_id=%ld)", call_res.identity_id);

    // Wait for play_state=1 (PLAYING)
    auto wait_playing_deadline = request_started_at + std::chrono::milliseconds(static_cast<int64_t>(start_timeout * 1000.0));
    auto play_status = monitor_->wait_for_playing_after(
      baseline.transition_sequence,
      wait_playing_deadline,
      [this, &goal_handle]() { return shutting_down_ || goal_handle->is_canceling(); }
    );

    if (play_status == WaitStatus::TIMEOUT) {
      RCLCPP_ERROR(this->get_logger(), "playback start timeout");
      result->success = false;
      result->message = "playback start timeout";
      goal_handle->abort(result);
      return;
    }
    else if (play_status == WaitStatus::CANCELLED || goal_handle->is_canceling()) {
      RCLCPP_WARN(this->get_logger(), "TTS cancel after publish; physical stop is not guaranteed");
      double stop_state_timeout = this->get_parameter("audio_stop_state_timeout_sec").as_double();
      if (stop_state_timeout > 0.0) {
        auto current_snap = monitor_->snapshot();
        monitor_->wait_for_stopped_after(
          current_snap.transition_sequence,
          std::chrono::steady_clock::now() + std::chrono::milliseconds(static_cast<int64_t>(stop_state_timeout * 1000.0)),
          []() { return false; }
        );
      }
      feedback->time = static_cast<float>(this->now().seconds() - start_time);
      feedback->done = false;
      goal_handle->publish_feedback(feedback);
      result->success = false;
      result->message = "canceled after publish";
      if (goal_handle->is_canceling()) {goal_handle->canceled(result);} else {goal_handle->abort(result);}
      return;
    }
    else if (play_status == WaitStatus::SHUTDOWN) {
      result->success = false;
      result->message = "node shutdown";
      goal_handle->abort(result);
      return;
    }

    RCLCPP_INFO(this->get_logger(), "playback state changed to PLAYING");

    // Wait for stable play_state=0 (STOPPED)
    uint64_t playing_sequence = monitor_->snapshot().last_playing_sequence;
    auto overall_finish_deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(static_cast<int64_t>(finish_timeout * 1000.0));
    WaitStatus stop_status = WaitStatus::TIMEOUT;

    PlayState internal_state = PlayState::PLAYING;

    while (rclcpp::ok()) {
      if (shutting_down_ || goal_handle->is_canceling()) {
        stop_status = WaitStatus::CANCELLED;
        break;
      }
      auto now = std::chrono::steady_clock::now();
      if (now >= overall_finish_deadline) {
        stop_status = WaitStatus::TIMEOUT;
        break;
      }

      auto chunk_deadline = std::min(overall_finish_deadline, now + std::chrono::milliseconds(static_cast<int64_t>(fb_period * 1000.0)));
      stop_status = monitor_->wait_for_stable_stopped_after(
        playing_sequence,
        chunk_deadline,
        quiet_duration,
        [this, &goal_handle]() { return shutting_down_ || goal_handle->is_canceling(); }
      );

      // Check transition changes for detailed log output
      auto snap = monitor_->snapshot();
      if (internal_state == PlayState::PLAYING && snap.last_state == PlayState::STOPPED && snap.last_stopped_sequence > playing_sequence) {
        internal_state = PlayState::STOPPED;
        RCLCPP_INFO(this->get_logger(), "playback state changed to STOPPED");
        RCLCPP_INFO(this->get_logger(), "TTS STOPPED candidate: waiting %.3f sec for stable completion", quiet_sec);
      }
      else if (internal_state == PlayState::STOPPED && snap.last_state == PlayState::PLAYING) {
        internal_state = PlayState::PLAYING;
        playing_sequence = snap.last_playing_sequence;
        RCLCPP_INFO(this->get_logger(), "playback state changed to PLAYING");
        RCLCPP_INFO(this->get_logger(), "TTS playback resumed before completion quiet period");
      }

      if (stop_status == WaitStatus::SUCCESS) {
        break;
      }
      if (stop_status == WaitStatus::CANCELLED || stop_status == WaitStatus::SHUTDOWN) {
        break;
      }

      // Publish feedback periodically
      feedback->time = static_cast<float>(this->now().seconds() - start_time);
      feedback->done = false;
      goal_handle->publish_feedback(feedback);
    }

    if (stop_status == WaitStatus::TIMEOUT) {
      RCLCPP_ERROR(this->get_logger(), "playback finish timeout");
      result->success = false;
      result->message = "playback finish timeout";
      goal_handle->abort(result);
      return;
    }
    else if (stop_status == WaitStatus::CANCELLED || goal_handle->is_canceling()) {
      RCLCPP_WARN(this->get_logger(), "TTS cancel after publish; physical stop is not guaranteed");
      double stop_state_timeout = this->get_parameter("audio_stop_state_timeout_sec").as_double();
      if (stop_state_timeout > 0.0) {
        auto current_snap = monitor_->snapshot();
        monitor_->wait_for_stopped_after(
          current_snap.transition_sequence,
          std::chrono::steady_clock::now() + std::chrono::milliseconds(static_cast<int64_t>(stop_state_timeout * 1000.0)),
          []() { return false; }
        );
      }
      feedback->time = static_cast<float>(this->now().seconds() - start_time);
      feedback->done = false;
      goal_handle->publish_feedback(feedback);
      result->success = false;
      result->message = "canceled after publish";
      if (goal_handle->is_canceling()) {goal_handle->canceled(result);} else {goal_handle->abort(result);}
      return;
    }
    else if (stop_status == WaitStatus::SHUTDOWN) {
      result->success = false;
      result->message = "node shutdown";
      goal_handle->abort(result);
      return;
    }

    RCLCPP_INFO(this->get_logger(), "TTS playback stable in STOPPED state");

    // TTS playback completed successfully
    RCLCPP_INFO(this->get_logger(), "TTS playback completed");
    feedback->time = static_cast<float>(this->now().seconds() - start_time);
    feedback->done = true;
    goal_handle->publish_feedback(feedback);

    result->success = true;
    result->message = "playback completed successfully";
    goal_handle->succeed(result);
  }

  void perform_stop_play(
    const std::string & stream_id,
    const std::string & app_name,
    const std::string & reason = "")
  {
    std::string stop_key_mode = this->get_parameter("audio_play_stop_key_mode").as_string();
    double stop_timeout = this->get_parameter("audio_stop_timeout_sec").as_double();

    // Section 6.2: JSON key MUST ALWAYS be "app_name".
    // audio_play_stop_key_mode selects whether the value is app_name or stream_id.
    const std::string stop_target = (stop_key_mode == "app_name") ? app_name : stream_id;

    nlohmann::json stop_param;
    stop_param["app_name"] = stop_target;
    std::string stop_param_str = stop_param.dump();

    if (!reason.empty()) {
      RCLCPP_INFO(this->get_logger(),
        "Sending STOP_PLAY (1004): reason='%s', target='%s', param=%s",
        reason.c_str(), stop_target.c_str(), stop_param_str.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "Calling Unitree STOP_PLAY API (1004) with param: %s", stop_param_str.c_str());
    }

    UnitreeApiClient::CallResult stop_res = api_client_->call(1004, stop_param_str, stop_timeout);

    if (stop_res.timed_out) {
      RCLCPP_WARN(this->get_logger(), "STOP_PLAY API timed out");
    } else if (!stop_res.received) {
      RCLCPP_WARN(this->get_logger(), "STOP_PLAY API failed to receive response");
    } else if (stop_res.status_code != 0) {
      RCLCPP_WARN(this->get_logger(), "STOP_PLAY API status error: status_code=%d", stop_res.status_code);
    } else {
      RCLCPP_INFO(this->get_logger(), "STOP_PLAY API accepted");
    }
  }

  // Audio Execution State Machine
  void execute_audio(const std::shared_ptr<GoalHandleVuiAudio> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing audio goal");
    auto feedback = std::make_shared<VuiAudio::Feedback>();
    auto result = std::make_shared<VuiAudio::Result>();
    const auto goal = goal_handle->get_goal();

    // Check busy / concurrent execution
    {
      std::lock_guard<std::mutex> lock(execution_mutex_);
      if (is_executing_) {
        RCLCPP_ERROR(this->get_logger(), "busy: another voice request is active");
        result->success = false;
        result->message = "busy: another voice request is active";
        result->bytes_sent = 0;
        result->total_bytes = 0;
        result->chunks_sent = 0;
        goal_handle->abort(result);
        return;
      }

      // Check if external playback is already active
      auto initial_snap = monitor_->snapshot();
      if (initial_snap.state_received && initial_snap.last_state == PlayState::PLAYING) {
        RCLCPP_ERROR(this->get_logger(), "existing external playback is active");
        result->success = false;
        result->message = "busy: audio playback is already active";
        result->bytes_sent = 0;
        result->total_bytes = 0;
        result->chunks_sent = 0;
        goal_handle->abort(result);
        return;
      }

      is_executing_ = true;
    }

    struct ExecutionGuard {
      VuiClientNode * node;
      explicit ExecutionGuard(VuiClientNode * n) : node(n) {}
      ~ExecutionGuard() {
        std::lock_guard<std::mutex> lock(node->execution_mutex_);
        node->is_executing_ = false;
      }
    } exec_guard(this);

    // Initial feedback
    feedback->time = 0.0f;
    feedback->done = false;
    feedback->bytes_sent = 0;
    feedback->total_bytes = 0;
    feedback->chunks_sent = 0;
    goal_handle->publish_feedback(feedback);

    // Retrieve and validate timeout parameters
    double start_timeout = this->get_parameter("playback_start_timeout_sec").as_double();
    double finish_grace = this->get_parameter("audio_playback_finish_grace_sec").as_double();
    double stop_state_timeout = this->get_parameter("audio_stop_state_timeout_sec").as_double();
    double fb_period = this->get_parameter("playback_feedback_period_sec").as_double();
    double quiet_sec = this->get_parameter("playback_completion_quiet_sec").as_double();

    if (start_timeout <= 0.0 || finish_grace <= 0.0 || stop_state_timeout <= 0.0 || fb_period <= 0.0 || quiet_sec <= 0.0) {
      RCLCPP_ERROR(this->get_logger(), "invalid timeout parameter");
      result->success = false;
      result->message = "invalid timeout parameter";
      result->bytes_sent = 0;
      result->total_bytes = 0;
      result->chunks_sent = 0;
      goal_handle->abort(result);
      return;
    }

    auto quiet_duration = std::chrono::milliseconds(static_cast<int64_t>(quiet_sec * 1000.0));

    // Validate parameters and modes
    int64_t max_input_bytes = this->get_parameter("audio_max_input_bytes").as_int();
    std::string stop_key_mode = this->get_parameter("audio_play_stop_key_mode").as_string();
    if (max_input_bytes <= 0) {
      RCLCPP_ERROR(this->get_logger(), "invalid audio_max_input_bytes: %ld", max_input_bytes);
      result->success = false;
      result->message = "invalid audio_max_input_bytes";
      goal_handle->abort(result);
      return;
    }
    if (stop_key_mode != "stream_id" && stop_key_mode != "app_name") {
      RCLCPP_ERROR(this->get_logger(), "invalid audio_play_stop_key_mode: %s", stop_key_mode.c_str());
      result->success = false;
      result->message = "invalid audio_play_stop_key_mode";
      goal_handle->abort(result);
      return;
    }

    std::vector<uint8_t> pcm_data;
    std::string err_msg;

    // --- State: VALIDATING ---
    if (goal->source_type == VuiAudio::Goal::SOURCE_PCM16_MONO_16K) {
      if (goal->audio_data.empty()) {
        RCLCPP_ERROR(this->get_logger(), "invalid goal: empty audio_data for PCM");
        result->success = false;
        result->message = "invalid goal: empty audio_data";
        goal_handle->abort(result);
        return;
      }
      if (goal->sample_rate != 16000 || goal->channels != 1 || goal->sample_width_bytes != 2) {
        RCLCPP_ERROR(this->get_logger(), "PCM metadata mismatch: rate=%u, channels=%u, width=%u",
          goal->sample_rate, goal->channels, goal->sample_width_bytes);
        result->success = false;
        result->message = "PCM metadata mismatch (must be 16000Hz, 1ch, 2bytes)";
        goal_handle->abort(result);
        return;
      }
      if (goal->audio_data.size() % 2 != 0) {
        RCLCPP_ERROR(this->get_logger(), "PCM byte size %zu is not aligned to sample width 2", goal->audio_data.size());
        result->success = false;
        result->message = "PCM byte size is not aligned to sample width";
        goal_handle->abort(result);
        return;
      }
      if (static_cast<int64_t>(goal->audio_data.size()) > max_input_bytes) {
        RCLCPP_ERROR(this->get_logger(), "input too large: %zu bytes", goal->audio_data.size());
        result->success = false;
        result->message = "input too large";
        goal_handle->abort(result);
        return;
      }
      pcm_data = goal->audio_data;
    }
    else if (goal->source_type == VuiAudio::Goal::SOURCE_WAV_BYTES) {
      if (goal->audio_data.empty()) {
        RCLCPP_ERROR(this->get_logger(), "invalid goal: empty audio_data for WAV_BYTES");
        result->success = false;
        result->message = "invalid goal: empty audio_data";
        goal_handle->abort(result);
        return;
      }
      if (static_cast<int64_t>(goal->audio_data.size()) > max_input_bytes) {
        RCLCPP_ERROR(this->get_logger(), "input too large: %zu bytes", goal->audio_data.size());
        result->success = false;
        result->message = "input too large";
        goal_handle->abort(result);
        return;
      }
      ConvertedAudio converted;
      if (!convert_pcm_wav_to_robot_audio(
          goal->audio_data, converted, err_msg, static_cast<std::size_t>(max_input_bytes)))
      {
        RCLCPP_ERROR(this->get_logger(), "invalid WAV header or format: %s", err_msg.c_str());
        result->success = false;
        result->message = "invalid WAV: " + err_msg;
        goal_handle->abort(result);
        return;
      }
      RCLCPP_INFO(
        this->get_logger(),
        "Converted WAV bytes: source=%uHz/%uch/%ubit duration=%.3fs, output=16000Hz/1ch/16bit duration=%.3fs",
        converted.source_sample_rate, converted.source_channels,
        converted.source_bits_per_sample, converted.source_duration_sec,
        converted.output_duration_sec);
      pcm_data = std::move(converted.pcm_s16le);
    }
    else if (goal->source_type == VuiAudio::Goal::SOURCE_WAV_FILE) {
      if (goal->file_path.empty()) {
        RCLCPP_ERROR(this->get_logger(), "invalid goal: empty file_path");
        result->success = false;
        result->message = "invalid goal: empty file_path";
        goal_handle->abort(result);
        return;
      }
      if (!is_absolute_path(goal->file_path)) {
        RCLCPP_ERROR(this->get_logger(), "file_path is not absolute: %s", goal->file_path.c_str());
        result->success = false;
        result->message = "file_path is not absolute";
        goal_handle->abort(result);
        return;
      }
      if (!file_exists_and_is_regular(goal->file_path)) {
        RCLCPP_ERROR(this->get_logger(), "file not found or is not a regular file: %s", goal->file_path.c_str());
        result->success = false;
        result->message = "file not found or not regular";
        goal_handle->abort(result);
        return;
      }
      
      // Read file
      std::ifstream ifs(goal->file_path, std::ios::binary | std::ios::ate);
      if (!ifs) {
        RCLCPP_ERROR(this->get_logger(), "failed to open file: %s", goal->file_path.c_str());
        result->success = false;
        result->message = "failed to open file";
        goal_handle->abort(result);
        return;
      }
      std::streamsize file_size = ifs.tellg();
      if (file_size < 0) {
        RCLCPP_ERROR(this->get_logger(), "failed to determine file size: %s", goal->file_path.c_str());
        result->success = false;
        result->message = "failed to determine file size";
        goal_handle->abort(result);
        return;
      }
      if (static_cast<int64_t>(file_size) > max_input_bytes) {
        RCLCPP_ERROR(this->get_logger(), "file too large: %ld bytes", file_size);
        result->success = false;
        result->message = "file too large";
        goal_handle->abort(result);
        return;
      }
      ifs.seekg(0, std::ios::beg);
      std::vector<uint8_t> wav_bytes(file_size);
      if (!ifs.read(reinterpret_cast<char*>(wav_bytes.data()), file_size)) {
        RCLCPP_ERROR(this->get_logger(), "failed to read file: %s", goal->file_path.c_str());
        result->success = false;
        result->message = "failed to read file";
        goal_handle->abort(result);
        return;
      }
      ifs.close();

      ConvertedAudio converted;
      if (!convert_pcm_wav_to_robot_audio(
          wav_bytes, converted, err_msg, static_cast<std::size_t>(max_input_bytes)))
      {
        RCLCPP_ERROR(this->get_logger(), "invalid WAV header or format in file: %s", err_msg.c_str());
        result->success = false;
        result->message = "invalid WAV: " + err_msg;
        goal_handle->abort(result);
        return;
      }
      RCLCPP_INFO(
        this->get_logger(),
        "Converted WAV file: source=%uHz/%uch/%ubit duration=%.3fs, output=16000Hz/1ch/16bit duration=%.3fs",
        converted.source_sample_rate, converted.source_channels,
        converted.source_bits_per_sample, converted.source_duration_sec,
        converted.output_duration_sec);
      pcm_data = std::move(converted.pcm_s16le);
    }
    else {
      RCLCPP_ERROR(this->get_logger(), "unsupported source_type: %u", goal->source_type);
      result->success = false;
      result->message = "unsupported source_type";
      goal_handle->abort(result);
      return;
    }

    // Resolve app_name
    std::string app_name = goal->app_name;
    if (app_name.empty()) {
      app_name = this->get_parameter("audio_app_name").as_string();
    }

    // Resolve stream_id
    std::string stream_id = goal->stream_id;
    if (stream_id.empty()) {
      uint32_t current_audio_index = ++audio_index_;
      auto now_ns = std::chrono::steady_clock::now().time_since_epoch().count();
      stream_id = "ngs_g1_audio_" + std::to_string(now_ns) + "_" + std::to_string(current_audio_index);
    }

    RCLCPP_INFO(this->get_logger(), "Streaming audio: app_name=%s, stream_id=%s, total_bytes=%zu",
      app_name.c_str(), stream_id.c_str(), pcm_data.size());

    // Divide into chunks and stream
    int64_t chunk_size_param = this->get_parameter("audio_chunk_size").as_int();
    if (chunk_size_param <= 0) {
      RCLCPP_ERROR(this->get_logger(), "invalid audio_chunk_size: %ld", chunk_size_param);
      result->success = false;
      result->message = "invalid audio_chunk_size";
      goal_handle->abort(result);
      return;
    }
    size_t chunk_size = static_cast<size_t>(chunk_size_param);

    size_t total_bytes = pcm_data.size();
    size_t bytes_sent = 0;
    uint32_t chunks_sent = 0;
    double start_time = this->now().seconds();
    double timeout_sec = this->get_parameter("timeout_sec").as_double();

    if (!wait_for_voice_endpoints(goal_handle, result)) {
      return;
    }

    // Record baseline sequence right before sending the first chunk
    const auto request_baseline = monitor_->snapshot();
    const auto stream_started_at = std::chrono::steady_clock::now();

    // --- State: STREAMING ---
    while (bytes_sent < total_bytes && rclcpp::ok() && !shutting_down_) {
      // Check cancel before sending
      if (shutting_down_ || goal_handle->is_canceling()) {
        RCLCPP_WARN(this->get_logger(), "Audio cancel requested during streaming; sending stop request");
        perform_stop_play(stream_id, app_name);
        
        if (stop_state_timeout > 0.0) {
          auto current_snap = monitor_->snapshot();
          monitor_->wait_for_stopped_after(
            current_snap.transition_sequence,
            std::chrono::steady_clock::now() + std::chrono::milliseconds(static_cast<int64_t>(stop_state_timeout * 1000.0)),
            []() { return false; }
          );
        }

        result->success = false;
        result->message = "canceled";
        result->bytes_sent = bytes_sent;
        result->total_bytes = total_bytes;
        result->chunks_sent = chunks_sent;
        if (goal_handle->is_canceling()) {goal_handle->canceled(result);} else {goal_handle->abort(result);}
        return;
      }

      size_t current_chunk_size = std::min(chunk_size, total_bytes - bytes_sent);
      std::vector<uint8_t> chunk(pcm_data.begin() + bytes_sent, pcm_data.begin() + bytes_sent + current_chunk_size);

      // Build JSON parameter
      nlohmann::json param;
      param["app_name"] = app_name;
      param["stream_id"] = stream_id;
      std::string param_str = param.dump();

      // Call API (START_PLAY 1003)
      UnitreeApiClient::CallResult call_res = api_client_->call(1003, param_str, chunk, timeout_sec);

      if (call_res.timed_out) {
        RCLCPP_ERROR(this->get_logger(), "API timeout/status error: Unitree START_PLAY timeout after %.2f seconds", timeout_sec);
        perform_stop_play(stream_id, app_name, "START_PLAY timeout");
        result->success = false;
        result->message = "START_PLAY timeout";
        result->bytes_sent = bytes_sent;
        result->total_bytes = total_bytes;
        result->chunks_sent = chunks_sent;
        goal_handle->abort(result);
        return;
      }

      if (!call_res.received) {
        RCLCPP_ERROR(this->get_logger(), "API timeout/status error: Failed to receive START_PLAY response");
        perform_stop_play(stream_id, app_name, "failed to receive START_PLAY response");
        result->success = false;
        result->message = "failed to receive START_PLAY response";
        result->bytes_sent = bytes_sent;
        result->total_bytes = total_bytes;
        result->chunks_sent = chunks_sent;
        goal_handle->abort(result);
        return;
      }

      if (call_res.status_code != 0) {
        RCLCPP_ERROR(this->get_logger(), "API timeout/status error: Unitree START_PLAY API status error: status_code=%d", call_res.status_code);
        perform_stop_play(stream_id, app_name, "START_PLAY status error: " + std::to_string(call_res.status_code));
        result->success = false;
        result->message = "START_PLAY status error: " + std::to_string(call_res.status_code);
        result->bytes_sent = bytes_sent;
        result->total_bytes = total_bytes;
        result->chunks_sent = chunks_sent;
        goal_handle->abort(result);
        return;
      }

      bytes_sent += current_chunk_size;
      chunks_sent++;

      // Publish feedback
      feedback->time = static_cast<float>(this->now().seconds() - start_time);
      feedback->done = false;
      feedback->bytes_sent = bytes_sent;
      feedback->total_bytes = total_bytes;
      feedback->chunks_sent = chunks_sent;
      goal_handle->publish_feedback(feedback);

      // Sleep between chunks if not the last chunk
      if (bytes_sent < total_bytes) {
        double interval = this->get_parameter("audio_chunk_interval_sec").as_double();
        auto sleep_start = std::chrono::steady_clock::now();
        while (std::chrono::duration<double>(std::chrono::steady_clock::now() - sleep_start).count() < interval && rclcpp::ok() && !shutting_down_) {
          if (shutting_down_ || goal_handle->is_canceling()) {
            break;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
      }
    }

    RCLCPP_INFO(this->get_logger(),
      "Audio all %u chunks sent (%zu bytes); waiting for natural playback completion",
      chunks_sent, bytes_sent);

    // --- State: WAITING_FOR_PLAYING_OR_COMPLETION ---
    auto wait_playing_deadline = stream_started_at + std::chrono::milliseconds(static_cast<int64_t>(start_timeout * 1000.0));
    auto play_status = monitor_->wait_for_playing_after(
      request_baseline.transition_sequence,
      wait_playing_deadline,
      [this, &goal_handle]() { return shutting_down_ || goal_handle->is_canceling(); }
    );

    std::chrono::steady_clock::time_point first_playing_time;

    if (play_status == WaitStatus::TIMEOUT) {
      // PLAYING を確認できなければ、短音でも完了として扱わない。
      perform_stop_play(stream_id, app_name, "playback start timeout");
      result->success = false;
      result->message = "playback start timeout";
      result->bytes_sent = bytes_sent;
      result->total_bytes = total_bytes;
      result->chunks_sent = chunks_sent;
      goal_handle->abort(result);
      return;
    }
    else if (play_status == WaitStatus::CANCELLED || goal_handle->is_canceling()) {
      RCLCPP_WARN(this->get_logger(), "Audio cancel requested during wait for PLAYING; sending stop request");
      perform_stop_play(stream_id, app_name, "Audio cancel requested during wait for PLAYING");
      if (stop_state_timeout > 0.0) {
        auto current_snap = monitor_->snapshot();
        monitor_->wait_for_stopped_after(
          current_snap.transition_sequence,
          std::chrono::steady_clock::now() + std::chrono::milliseconds(static_cast<int64_t>(stop_state_timeout * 1000.0)),
          []() { return false; }
        );
      }
      feedback->time = static_cast<float>(this->now().seconds() - start_time);
      feedback->done = false;
      feedback->bytes_sent = bytes_sent;
      feedback->total_bytes = total_bytes;
      feedback->chunks_sent = chunks_sent;
      goal_handle->publish_feedback(feedback);

      result->success = false;
      result->message = "canceled";
      result->bytes_sent = bytes_sent;
      result->total_bytes = total_bytes;
      result->chunks_sent = chunks_sent;
      if (goal_handle->is_canceling()) {goal_handle->canceled(result);} else {goal_handle->abort(result);}
      return;
    }
    else if (play_status == WaitStatus::SHUTDOWN) {
      result->success = false;
      result->message = "node shutdown";
      goal_handle->abort(result);
      return;
    }
    else {
      first_playing_time = std::chrono::steady_clock::now();
      RCLCPP_INFO(this->get_logger(), "playback state changed to PLAYING");
    }

    // --- State: WAITING_FOR_STABLE_STOPPED ---
    double expected_duration_sec = static_cast<double>(total_bytes) / 32000.0;
    double finish_timeout = expected_duration_sec + finish_grace;

    uint64_t playing_sequence = monitor_->snapshot().last_playing_sequence;
    uint64_t minimum_stop_sequence = std::max(playing_sequence, request_baseline.transition_sequence);

    auto overall_finish_deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(static_cast<int64_t>(finish_timeout * 1000.0));
    WaitStatus stop_status = WaitStatus::TIMEOUT;
    PlayState internal_state = PlayState::PLAYING;

    while (rclcpp::ok()) {
      if (shutting_down_ || goal_handle->is_canceling()) {
        stop_status = WaitStatus::CANCELLED;
        break;
      }
      auto now = std::chrono::steady_clock::now();
      if (now >= overall_finish_deadline) {
        stop_status = WaitStatus::TIMEOUT;
        break;
      }

      auto chunk_deadline = std::min(overall_finish_deadline, now + std::chrono::milliseconds(static_cast<int64_t>(fb_period * 1000.0)));
      stop_status = monitor_->wait_for_stable_stopped_after(
        minimum_stop_sequence,
        chunk_deadline,
        quiet_duration,
        [this, &goal_handle]() { return shutting_down_ || goal_handle->is_canceling(); }
      );

      auto snap = monitor_->snapshot();
      if (internal_state == PlayState::PLAYING && snap.last_state == PlayState::STOPPED && snap.last_stopped_sequence > minimum_stop_sequence) {
        internal_state = PlayState::STOPPED;
        RCLCPP_INFO(this->get_logger(), "playback state changed to STOPPED");
        RCLCPP_INFO(this->get_logger(), "Audio STOPPED candidate: waiting %.3f sec for stable completion", quiet_sec);
      }
      else if (internal_state == PlayState::STOPPED && snap.last_state == PlayState::PLAYING) {
        internal_state = PlayState::PLAYING;
        minimum_stop_sequence = std::max(snap.last_playing_sequence, request_baseline.transition_sequence);
        RCLCPP_INFO(this->get_logger(), "playback state changed to PLAYING");
        RCLCPP_INFO(this->get_logger(), "Audio playback resumed before completion quiet period");
      }

      if (stop_status == WaitStatus::SUCCESS) {
        auto elapsed_playback = std::chrono::duration<double>(std::chrono::steady_clock::now() - first_playing_time).count();
        // Guard against premature completion caused by intermediate speech pauses
        if (elapsed_playback < expected_duration_sec - quiet_sec) {
          RCLCPP_DEBUG(this->get_logger(),
            "Audio STOPPED detected early (elapsed=%.2fs < expected=%.2fs); continuing playback monitoring",
            elapsed_playback, expected_duration_sec);
          stop_status = WaitStatus::TIMEOUT;
          continue;
        }
        break;
      }
      if (stop_status == WaitStatus::CANCELLED || stop_status == WaitStatus::SHUTDOWN) {
        break;
      }

      // Publish feedback periodically
      feedback->time = static_cast<float>(this->now().seconds() - start_time);
      feedback->done = false;
      feedback->bytes_sent = bytes_sent;
      feedback->total_bytes = total_bytes;
      feedback->chunks_sent = chunks_sent;
      goal_handle->publish_feedback(feedback);
    }

    if (stop_status == WaitStatus::TIMEOUT) {
      auto elapsed_from_play = std::chrono::duration<double>(std::chrono::steady_clock::now() - first_playing_time).count();
      RCLCPP_ERROR(this->get_logger(),
        "Playback finish timeout: elapsed_from_play=%.2fs, expected=%.2fs; sending STOP_PLAY",
        elapsed_from_play, expected_duration_sec);
      perform_stop_play(stream_id, app_name, "Playback finish timeout");
      result->success = false;
      result->message = "playback finish timeout";
      result->bytes_sent = bytes_sent;
      result->total_bytes = total_bytes;
      result->chunks_sent = chunks_sent;
      goal_handle->abort(result);
      return;
    }
    else if (stop_status == WaitStatus::CANCELLED || goal_handle->is_canceling()) {
      RCLCPP_WARN(this->get_logger(), "Audio cancel requested during playback; sending STOP_PLAY");
      perform_stop_play(stream_id, app_name, "Audio cancel requested during playback");
      if (stop_state_timeout > 0.0) {
        auto current_snap = monitor_->snapshot();
        monitor_->wait_for_stopped_after(
          current_snap.transition_sequence,
          std::chrono::steady_clock::now() + std::chrono::milliseconds(static_cast<int64_t>(stop_state_timeout * 1000.0)),
          []() { return false; }
        );
      }
      feedback->time = static_cast<float>(this->now().seconds() - start_time);
      feedback->done = false;
      feedback->bytes_sent = bytes_sent;
      feedback->total_bytes = total_bytes;
      feedback->chunks_sent = chunks_sent;
      goal_handle->publish_feedback(feedback);

      result->success = false;
      result->message = "canceled";
      result->bytes_sent = bytes_sent;
      result->total_bytes = total_bytes;
      result->chunks_sent = chunks_sent;
      if (goal_handle->is_canceling()) {goal_handle->canceled(result);} else {goal_handle->abort(result);}
      return;
    }
    else if (stop_status == WaitStatus::SHUTDOWN) {
      result->success = false;
      result->message = "node shutdown";
      goal_handle->abort(result);
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Audio playback stable in STOPPED state");

    // Natural playback completion confirmed; perform optional cleanup if stop_after_play was requested
    if (goal->stop_after_play) {
      perform_stop_play(
        stream_id,
        app_name,
        "Natural playback completion confirmed; sending optional STOP_PLAY cleanup"
      );
    }

    // --- State: SUCCEEDED ---
    RCLCPP_INFO(this->get_logger(), "Audio playback completed successfully");
    feedback->time = static_cast<float>(this->now().seconds() - start_time);
    feedback->done = true;
    feedback->bytes_sent = bytes_sent;
    feedback->total_bytes = total_bytes;
    feedback->chunks_sent = chunks_sent;
    goal_handle->publish_feedback(feedback);

    result->success = true;
    result->message = "audio playback completed successfully";
    result->bytes_sent = bytes_sent;
    result->total_bytes = total_bytes;
    result->chunks_sent = chunks_sent;
    goal_handle->succeed(result);
  }

  // Mic Service Callback
  void log_mic_api_result(
    const char * operation,
    const UnitreeApiClient::CallResult & result) const
  {
    if (result.publish_failed) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Mic remote API %s request publish failed: api_id=1002 identity_id=%ld elapsed=%.3fs error='%s'",
        operation, result.identity_id, result.elapsed_sec, result.error_message.c_str());
    } else if (result.timed_out) {
      RCLCPP_WARN(
        this->get_logger(),
        "Mic remote API %s timed out: api_id=1002 identity_id=%ld received=%s status=%d elapsed=%.3fs",
        operation, result.identity_id, result.received ? "true" : "false",
        result.status_code, result.elapsed_sec);
    } else if (!result.received) {
      RCLCPP_WARN(
        this->get_logger(),
        "Mic remote API %s response callback was not observed: api_id=1002 identity_id=%ld status=%d elapsed=%.3fs",
        operation, result.identity_id, result.status_code, result.elapsed_sec);
    } else if (result.status_code != 0) {
      RCLCPP_WARN(
        this->get_logger(),
        "Mic remote API %s response received with failure: api_id=1002 identity_id=%ld status=%d elapsed=%.3fs",
        operation, result.identity_id, result.status_code, result.elapsed_sec);
    } else {
      RCLCPP_INFO(
        this->get_logger(),
        "Mic remote API %s succeeded: api_id=1002 identity_id=%ld status=0 elapsed=%.3fs",
        operation, result.identity_id, result.elapsed_sec);
    }
  }

  void handle_mic_service(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    if (request->data) {
      std::shared_ptr<MicUdpReceiver> receiver;
      uint64_t baseline = 0U;
      {
        std::lock_guard<std::mutex> lock(mic_mutex_);
        RCLCPP_INFO(
          this->get_logger(), "Microphone enable requested: current_state=%s",
          mic_state_label(mic_state_));
        if (mic_state_ == MicState::ENABLED) {
          response->success = true;
          response->message = "microphone already enabled";
          return;
        }
        if (mic_state_ == MicState::ENABLING || mic_state_ == MicState::DISABLING) {
          response->success = false;
          response->message = "microphone transition is already in progress";
          return;
        }

        if (!mic_receiver_) {
          const std::string nic =
            this->get_parameter("mic_network_interface").as_string();
          const std::string mcast_group =
            this->get_parameter("mic_multicast_group").as_string();
          const int64_t mcast_port =
            this->get_parameter("mic_multicast_port").as_int();
          const int64_t recv_timeout =
            this->get_parameter("mic_receive_timeout_ms").as_int();
          const int64_t recv_buffer =
            this->get_parameter("mic_receive_buffer_bytes").as_int();

          mic_receiver_ = std::make_shared<MicUdpReceiver>(
            nic, mcast_group, static_cast<int>(mcast_port),
            static_cast<int>(recv_timeout), static_cast<size_t>(recv_buffer));
          mic_receiver_->set_data_callback(
            [this](const std::vector<int16_t> & pcm_samples) {
              if (!mic_publish_enabled_.load()) {
                RCLCPP_DEBUG_THROTTLE(
                  this->get_logger(), *this->get_clock(), 3000,
                  "Dropping microphone packet because publish gate is closed");
                return;
              }
              std::vector<int16_t> output = pcm_samples;
              if (mic_noise_reducer_) {
                output = mic_noise_reducer_->process(pcm_samples);
              }
              if (output.empty()) {
                return;
              }
              auto msg = std_msgs::msg::Int16MultiArray();
              msg.data = output;
              mic_pub_->publish(msg);
            });
          mic_receiver_->set_error_callback(
            [this](const std::string & message, bool fatal) {
              if (!fatal) {
                RCLCPP_DEBUG_THROTTLE(
                  this->get_logger(), *this->get_clock(), 3000, "%s", message.c_str());
                return;
              }
              RCLCPP_ERROR(this->get_logger(), "Microphone UDP receiver failed: %s", message.c_str());
              mic_publish_enabled_.store(false);
              std::lock_guard<std::mutex> lock(mic_mutex_);
              if (mic_state_ != MicState::DISABLING && mic_state_ != MicState::DISABLED) {
                mic_state_ = MicState::ERROR;
              }
            });
        }
        receiver = mic_receiver_;
        if (mic_noise_reducer_) {
          mic_noise_reducer_->reset();
        }
        baseline = receiver->packet_sequence();
        mic_state_ = MicState::ENABLING;
        mic_publish_enabled_.store(true);
      }

      RCLCPP_INFO(
        this->get_logger(), "Starting microphone UDP receiver: packet_baseline=%lu",
        baseline);
      std::string start_err;
      const bool receiver_ok = receiver->start(start_err);
      RCLCPP_INFO(
        this->get_logger(), "Microphone receiver start result: success=%s detail='%s'",
        receiver_ok ? "true" : "false", start_err.c_str());
      if (!receiver_ok) {
        mic_publish_enabled_.store(false);
        std::lock_guard<std::mutex> lock(mic_mutex_);
        mic_state_ = MicState::DISABLED;
        response->success = false;
        response->message = "failed to start UDP receiver: " + start_err;
        RCLCPP_ERROR(
          this->get_logger(), "Microphone enable failed: final_state=%s",
          mic_state_label(mic_state_));
        return;
      }

      UnitreeApiClient::CallResult call_res{};
      call_res.status_code = -1;
      const bool api_call_needed = remote_api_policy_ != CloseApiPolicy::DISABLED;
      bool api_success = false;
      if (api_call_needed) {
        const double timeout_sec = this->get_parameter("timeout_sec").as_double();
        nlohmann::json parameter;
        parameter["open"] = 1;
        RCLCPP_INFO(
          this->get_logger(),
          "Calling unverified remote microphone API: api_id=1002 parameter=%s policy=%s",
          parameter.dump().c_str(),
          remote_api_policy_ == CloseApiPolicy::REQUIRED ? "required" : "best_effort");
        call_res = api_client_->call(1002, parameter.dump(), timeout_sec);
        api_success = call_res.received && !call_res.timed_out && call_res.status_code == 0;
        log_mic_api_result("open", call_res);
      } else {
        RCLCPP_INFO(
          this->get_logger(),
          "Remote microphone API is disabled; local UDP packet reception is the success criterion");
      }

      const double start_timeout =
        this->get_parameter("mic_start_timeout_sec").as_double();
      const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(start_timeout));
      bool fresh_packet_received = false;
      while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
        const auto slice_deadline = std::min(
          deadline, std::chrono::steady_clock::now() + std::chrono::milliseconds(100));
        if (receiver->wait_for_packet_after(baseline, slice_deadline)) {
          fresh_packet_received = true;
          break;
        }
      }
      const uint64_t fresh_sequence = receiver->packet_sequence();
      RCLCPP_INFO(
        this->get_logger(),
        "Microphone fresh packet result: received=%s baseline=%lu sequence=%lu",
        fresh_packet_received ? "true" : "false", baseline, fresh_sequence);

      const EnableDecision decision = evaluate_enable_result(
        remote_api_policy_, api_success, call_res.timed_out, call_res.received,
        call_res.status_code, fresh_packet_received);

      if (!decision.success) {
        mic_publish_enabled_.store(false);
        receiver->stop();
      }

      {
        std::lock_guard<std::mutex> lock(mic_mutex_);
        mic_state_ = decision.success ? MicState::ENABLED : MicState::DISABLED;
        response->success = decision.success;
        response->message = decision.response_message;
        RCLCPP_INFO(
          this->get_logger(), "Microphone enable completed: success=%s final_state=%s",
          decision.success ? "true" : "false", mic_state_label(mic_state_));
      }
      return;
    }

    std::shared_ptr<MicUdpReceiver> receiver;
    {
      std::lock_guard<std::mutex> lock(mic_mutex_);
      RCLCPP_INFO(
        this->get_logger(), "Microphone disable requested: current_state=%s",
        mic_state_label(mic_state_));
      mic_publish_enabled_.store(false);
      if (mic_state_ == MicState::DISABLED) {
        response->success = true;
        response->message = "microphone already disabled";
        return;
      }
      mic_state_ = MicState::DISABLING;
      receiver = mic_receiver_;
    }

    if (receiver) {
      receiver->stop();
    }
    if (mic_noise_reducer_) {
      mic_noise_reducer_->reset();
    }
    const bool local_stopped = !receiver || !receiver->is_running();

    const bool api_call_needed = remote_api_policy_ != CloseApiPolicy::DISABLED;
    bool api_success = !api_call_needed;
    std::string api_error;
    if (api_call_needed) {
      const double timeout_sec = this->get_parameter("timeout_sec").as_double();
      nlohmann::json parameter;
      parameter["open"] = 0;
      RCLCPP_INFO(
        this->get_logger(),
        "Calling unverified remote microphone API: api_id=1002 parameter=%s policy=%s",
        parameter.dump().c_str(),
        remote_api_policy_ == CloseApiPolicy::REQUIRED ? "required" : "best_effort");
      const UnitreeApiClient::CallResult call_res =
        api_client_->call(1002, parameter.dump(), timeout_sec);
      log_mic_api_result("close", call_res);
      api_success = call_res.received && !call_res.timed_out && call_res.status_code == 0;
      if (call_res.timed_out) {
        api_error = "remote microphone API close request timed out";
      } else if (!call_res.received) {
        api_error = "remote microphone API close response callback was not observed";
      } else if (call_res.status_code != 0) {
        api_error = "remote microphone API close returned status_code=" +
          std::to_string(call_res.status_code);
      }
    }

    const CloseDecision decision = evaluate_close_result(
      remote_api_policy_, local_stopped, api_call_needed, api_success, api_error);
    {
      std::lock_guard<std::mutex> lock(mic_mutex_);
      mic_state_ = local_stopped ? MicState::DISABLED : MicState::ERROR;
      response->success = decision.success;
      response->message = decision.response_message;
      RCLCPP_INFO(
        this->get_logger(), "Microphone disable completed: success=%s final_state=%s",
        decision.success ? "true" : "false", mic_state_label(mic_state_));
    }
  }
};

} // namespace erasers_g1_common

#ifndef ERASERS_G1_VUI_NO_MAIN
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<erasers_g1_common::VuiClientNode>();
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 4U);
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}

#endif
