// Copyright (c) 2026 Nakatogawa Laboratory. All rights reserved.

#ifndef ERASERS_G1_COMMON__AUDIO_PLAYBACK_MONITOR_HPP_
#define ERASERS_G1_COMMON__AUDIO_PLAYBACK_MONITOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <mutex>
#include <condition_variable>
#include <string>
#include <chrono>
#include <functional>

namespace erasers_g1_common
{

enum class PlayState
{
  UNKNOWN = -1,
  STOPPED = 0,
  PLAYING = 1,
};

struct Snapshot
{
  uint64_t transition_sequence;
  uint64_t last_playing_sequence;
  uint64_t last_stopped_sequence;
  PlayState last_state;
  bool state_received;
  std::chrono::steady_clock::time_point last_transition_time;
};

enum class WaitStatus
{
  SUCCESS,
  TIMEOUT,
  CANCELLED,
  SHUTDOWN
};

class AudioPlaybackMonitor
{
public:
  AudioPlaybackMonitor(
    rclcpp::Node * node,
    const std::string & topic_name);

  virtual ~AudioPlaybackMonitor() = default;

  Snapshot snapshot() const;

  WaitStatus wait_for_playing_after(
    uint64_t baseline_sequence,
    std::chrono::steady_clock::time_point deadline,
    const std::function<bool()> & cancel_requested);

  WaitStatus wait_for_stopped_after(
    uint64_t minimum_sequence,
    std::chrono::steady_clock::time_point deadline,
    const std::function<bool()> & cancel_requested);

  WaitStatus wait_for_stable_stopped_after(
    uint64_t minimum_sequence,
    std::chrono::steady_clock::time_point deadline,
    std::chrono::steady_clock::duration quiet_period,
    const std::function<bool()> & cancel_requested);

  void ingest_payload(const std::string & payload);

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg);

  rclcpp::Node * node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

  mutable std::mutex mutex_;
  std::condition_variable cv_;

  PlayState last_state_;
  uint64_t transition_sequence_;
  uint64_t last_playing_sequence_;
  uint64_t last_stopped_sequence_;
  bool state_received_;
  std::chrono::steady_clock::time_point last_transition_time_;

  // Throttled warning helpers
  std::chrono::steady_clock::time_point last_warn_time_;
  void log_warn_throttled(const std::string & message);
};

}  // namespace erasers_g1_common

#endif  // ERASERS_G1_COMMON__AUDIO_PLAYBACK_MONITOR_HPP_
