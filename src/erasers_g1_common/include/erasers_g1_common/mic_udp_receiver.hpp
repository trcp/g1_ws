// Copyright (c) 2026 Nakatogawa Laboratory. All rights reserved.

#ifndef ERASERS_G1_COMMON__MIC_UDP_RECEIVER_HPP_
#define ERASERS_G1_COMMON__MIC_UDP_RECEIVER_HPP_

#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>
#include <functional>
#include <condition_variable>
#include <chrono>

namespace erasers_g1_common
{

class MicUdpReceiver
{
public:
  using DataCallback = std::function<void(const std::vector<int16_t> &)>;
  using ErrorCallback = std::function<void(const std::string &, bool is_fatal)>;

  MicUdpReceiver(
    const std::string & interface_name,
    const std::string & multicast_group,
    int port,
    int receive_timeout_ms,
    size_t receive_buffer_bytes);

  virtual ~MicUdpReceiver();

  // Start the background receiving thread. Returns true on success.
  virtual bool start(std::string & err_msg);

  // Stop the receiving thread and release sockets.
  virtual void stop();

  // Queries
  virtual uint64_t packet_sequence() const;
  virtual bool is_running() const;
  virtual bool wait_for_packet_after(
    uint64_t baseline,
    std::chrono::steady_clock::time_point deadline);
  std::string resolved_interface() const;
  std::string resolved_ip() const;

  // Callbacks registration
  void set_data_callback(DataCallback cb);
  void set_error_callback(ErrorCallback cb);

private:
  void run();
  bool setup_socket(std::string & err_msg);
  void cleanup_socket();
  std::string resolve_interface_ip(const std::string & target_iface, std::string & err_msg);

  // Configuration
  std::string interface_name_;
  std::string multicast_group_;
  int port_;
  int receive_timeout_ms_;
  size_t receive_buffer_bytes_;

  // Resolved state
  std::string resolved_interface_;
  std::string resolved_ip_;

  // Network objects
  int sock_{-1};

  // Lifecycle and threads
  std::thread thread_;
  std::atomic<bool> is_running_{false};
  std::atomic<bool> stop_requested_{false};

  // Sequence and diagnostics mutex
  mutable std::mutex mutex_;
  std::condition_variable packet_cv_;
  uint64_t packet_sequence_{0};

  // Callbacks
  DataCallback data_cb_;
  ErrorCallback error_cb_;
};

} // namespace erasers_g1_common

#endif // ERASERS_G1_COMMON__MIC_UDP_RECEIVER_HPP_
