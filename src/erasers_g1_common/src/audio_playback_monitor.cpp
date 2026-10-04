// Copyright (c) 2026 Nakatogawa Laboratory. All rights reserved.

#include <erasers_g1_common/audio_playback_monitor.hpp>
#include <nlohmann/json.hpp>
#include <algorithm>

namespace erasers_g1_common
{

AudioPlaybackMonitor::AudioPlaybackMonitor(
  rclcpp::Node * node,
  const std::string & topic_name)
: node_(node),
  last_state_(PlayState::UNKNOWN),
  transition_sequence_(0),
  last_playing_sequence_(0),
  last_stopped_sequence_(0),
  state_received_(false),
  last_transition_time_(std::chrono::steady_clock::time_point::min()),
  last_warn_time_(std::chrono::steady_clock::time_point::min())
{
  rclcpp::QoS qos(1);
  qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
  qos.durability(rclcpp::DurabilityPolicy::Volatile);

  subscription_ = node_->create_subscription<std_msgs::msg::String>(
    topic_name,
    qos,
    std::bind(&AudioPlaybackMonitor::topic_callback, this, std::placeholders::_1)
  );
}

Snapshot AudioPlaybackMonitor::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  Snapshot s;
  s.transition_sequence = transition_sequence_;
  s.last_playing_sequence = last_playing_sequence_;
  s.last_stopped_sequence = last_stopped_sequence_;
  s.last_state = last_state_;
  s.state_received = state_received_;
  s.last_transition_time = last_transition_time_;
  return s;
}

void AudioPlaybackMonitor::ingest_payload(const std::string & payload)
{
  try {
    auto json_data = nlohmann::json::parse(payload);
    if (!json_data.contains("play_state")) {
      // play_state field がない場合は静かに無視する（ASR 等の別メッセージ）
      return;
    }

    auto val = json_data["play_state"];
    if (!val.is_number_integer()) {
      log_warn_throttled("play_state is not an integer in /audio_msg");
      return;
    }

    int state_val = val.get<int>();
    if (state_val != 0 && state_val != 1) {
      log_warn_throttled("play_state is not 0 or 1 in /audio_msg: " + std::to_string(state_val));
      return;
    }

    PlayState next_state = (state_val == 1) ? PlayState::PLAYING : PlayState::STOPPED;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      bool transition = false;
      if (!state_received_) {
        state_received_ = true;
        transition = true;
      } else if (last_state_ != next_state) {
        transition = true;
      }

      if (transition) {
        last_state_ = next_state;
        transition_sequence_++;
        last_transition_time_ = std::chrono::steady_clock::now();
        if (next_state == PlayState::PLAYING) {
          last_playing_sequence_ = transition_sequence_;
        } else {
          last_stopped_sequence_ = transition_sequence_;
        }
        cv_.notify_all();
      }
    }
  } catch (const nlohmann::json::parse_error & e) {
    log_warn_throttled("invalid JSON on /audio_msg: " + std::string(e.what()));
  }
}

void AudioPlaybackMonitor::topic_callback(const std_msgs::msg::String::SharedPtr msg)
{
  ingest_payload(msg->data);
}

WaitStatus AudioPlaybackMonitor::wait_for_playing_after(
  uint64_t baseline_sequence,
  std::chrono::steady_clock::time_point deadline,
  const std::function<bool()> & cancel_requested)
{
  std::unique_lock<std::mutex> lock(mutex_);

  auto check_condition = [this, baseline_sequence]() {
    return state_received_ &&
           last_playing_sequence_ > baseline_sequence;
  };

  while (!check_condition()) {
    if (cancel_requested()) {
      return WaitStatus::CANCELLED;
    }
    if (!rclcpp::ok()) {
      return WaitStatus::SHUTDOWN;
    }
    auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      return WaitStatus::TIMEOUT;
    }

    auto wait_duration = std::min(std::chrono::milliseconds(100),
      std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now));
    if (wait_duration <= std::chrono::milliseconds(0)) {
      if (check_condition()) {
        return WaitStatus::SUCCESS;
      }
      return WaitStatus::TIMEOUT;
    }
    cv_.wait_for(lock, wait_duration);
  }

  return WaitStatus::SUCCESS;
}

WaitStatus AudioPlaybackMonitor::wait_for_stopped_after(
  uint64_t minimum_sequence,
  std::chrono::steady_clock::time_point deadline,
  const std::function<bool()> & cancel_requested)
{
  std::unique_lock<std::mutex> lock(mutex_);

  auto check_condition = [this, minimum_sequence]() {
    return state_received_ &&
           last_state_ == PlayState::STOPPED &&
           last_stopped_sequence_ > minimum_sequence;
  };

  while (!check_condition()) {
    if (cancel_requested()) {
      return WaitStatus::CANCELLED;
    }
    if (!rclcpp::ok()) {
      return WaitStatus::SHUTDOWN;
    }
    auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      return WaitStatus::TIMEOUT;
    }

    auto wait_duration = std::min(std::chrono::milliseconds(100),
      std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now));
    if (wait_duration <= std::chrono::milliseconds(0)) {
      if (check_condition()) {
        return WaitStatus::SUCCESS;
      }
      return WaitStatus::TIMEOUT;
    }
    cv_.wait_for(lock, wait_duration);
  }

  return WaitStatus::SUCCESS;
}

WaitStatus AudioPlaybackMonitor::wait_for_stable_stopped_after(
  uint64_t minimum_sequence,
  std::chrono::steady_clock::time_point deadline,
  std::chrono::steady_clock::duration quiet_period,
  const std::function<bool()> & cancel_requested)
{
  std::unique_lock<std::mutex> lock(mutex_);

  auto check_condition = [this, minimum_sequence, quiet_period]() {
    if (!state_received_ || last_state_ != PlayState::STOPPED) {
      return false;
    }
    if (last_stopped_sequence_ <= minimum_sequence) {
      return false;
    }
    auto elapsed = std::chrono::steady_clock::now() - last_transition_time_;
    return elapsed >= quiet_period;
  };

  while (!check_condition()) {
    if (cancel_requested()) {
      return WaitStatus::CANCELLED;
    }
    if (!rclcpp::ok()) {
      return WaitStatus::SHUTDOWN;
    }
    auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      return WaitStatus::TIMEOUT;
    }

    auto next_check_time = now + std::chrono::milliseconds(100);
    if (state_received_ && last_state_ == PlayState::STOPPED && last_stopped_sequence_ > minimum_sequence) {
      next_check_time = last_transition_time_ + quiet_period;
    }
    auto wait_until_time = std::min(deadline, next_check_time);
    wait_until_time = std::min(wait_until_time, now + std::chrono::milliseconds(100));

    if (wait_until_time <= now) {
      if (check_condition()) {
        return WaitStatus::SUCCESS;
      }
      return WaitStatus::TIMEOUT;
    }
    cv_.wait_until(lock, wait_until_time);
  }

  return WaitStatus::SUCCESS;
}

void AudioPlaybackMonitor::log_warn_throttled(const std::string & message)
{
  auto now = std::chrono::steady_clock::now();
  if (std::chrono::duration_cast<std::chrono::seconds>(now - last_warn_time_).count() >= 1) {
    RCLCPP_WARN(node_->get_logger(), "%s", message.c_str());
    last_warn_time_ = now;
  }
}

}  // namespace erasers_g1_common
