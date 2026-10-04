// Copyright (c) 2026 Nakatogawa Laboratory. All rights reserved.

#include <erasers_g1_common/mic_udp_receiver.hpp>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <ifaddrs.h>
#include <netdb.h>
#include <unistd.h>
#include <net/if.h>
#include <cstring>
#include <cerrno>
#include <iostream>
#include <sstream>

namespace erasers_g1_common
{

MicUdpReceiver::MicUdpReceiver(
  const std::string & interface_name,
  const std::string & multicast_group,
  int port,
  int receive_timeout_ms,
  size_t receive_buffer_bytes)
: interface_name_(interface_name),
  multicast_group_(multicast_group),
  port_(port),
  receive_timeout_ms_(receive_timeout_ms),
  receive_buffer_bytes_(receive_buffer_bytes)
{
}

MicUdpReceiver::~MicUdpReceiver()
{
  stop();
}

bool MicUdpReceiver::start(std::string & err_msg)
{
  if (is_running_) {
    return true;
  }

  // Parameter validation
  if (port_ <= 0 || port_ > 65535) {
    err_msg = "Invalid port number: " + std::to_string(port_);
    return false;
  }
  if (receive_timeout_ms_ <= 0) {
    err_msg = "Invalid receive timeout: " + std::to_string(receive_timeout_ms_);
    return false;
  }
  if (receive_buffer_bytes_ < 2 || receive_buffer_bytes_ > 65536) {
    err_msg = "Invalid receive buffer size: " + std::to_string(receive_buffer_bytes_);
    return false;
  }

  struct in_addr mcast_addr;
  if (inet_pton(AF_INET, multicast_group_.c_str(), &mcast_addr) != 1) {
    err_msg = "Invalid multicast group address: " + multicast_group_;
    return false;
  }

  // Resolve IP of the interface
  resolved_ip_ = resolve_interface_ip(interface_name_, err_msg);
  if (resolved_ip_.empty()) {
    return false;
  }

  // Setup UDP socket and join multicast group
  if (!setup_socket(err_msg)) {
    cleanup_socket();
    return false;
  }

  stop_requested_ = false;
  // Start receiver thread
  thread_ = std::thread(&MicUdpReceiver::run, this);
  is_running_ = true;

  return true;
}

void MicUdpReceiver::stop()
{
  if (!is_running_) {
    return;
  }

  stop_requested_ = true;
  packet_cv_.notify_all();

  // Cleanup socket to unblock recvfrom if it is waiting
  cleanup_socket();

  if (thread_.joinable()) {
    thread_.join();
  }

  is_running_ = false;
}

uint64_t MicUdpReceiver::packet_sequence() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return packet_sequence_;
}

bool MicUdpReceiver::is_running() const
{
  return is_running_;
}

bool MicUdpReceiver::wait_for_packet_after(
  uint64_t baseline,
  std::chrono::steady_clock::time_point deadline)
{
  std::unique_lock<std::mutex> lock(mutex_);
  return packet_cv_.wait_until(lock, deadline, [this, baseline]() {
    return packet_sequence_ > baseline || stop_requested_.load();
  }) && packet_sequence_ > baseline;
}

std::string MicUdpReceiver::resolved_interface() const
{
  return resolved_interface_;
}

std::string MicUdpReceiver::resolved_ip() const
{
  return resolved_ip_;
}

void MicUdpReceiver::set_data_callback(DataCallback cb)
{
  data_cb_ = cb;
}

void MicUdpReceiver::set_error_callback(ErrorCallback cb)
{
  error_cb_ = cb;
}

std::string MicUdpReceiver::resolve_interface_ip(const std::string & target_iface, std::string & err_msg)
{
  struct ifaddrs *ifaddr = nullptr;
  if (getifaddrs(&ifaddr) == -1) {
    err_msg = "getifaddrs failed: " + std::string(strerror(errno));
    return "";
  }

  std::string found_ip = "";
  std::string found_name = "";

  if (!target_iface.empty()) {
    // Search for explicitly named interface
    for (struct ifaddrs *ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
      if (ifa->ifa_addr == nullptr || ifa->ifa_addr->sa_family != AF_INET) {
        continue;
      }
      std::string ifa_name(ifa->ifa_name);
      if (ifa_name == target_iface) {
        char host[NI_MAXHOST];
        if (getnameinfo(ifa->ifa_addr, sizeof(struct sockaddr_in), host, NI_MAXHOST, nullptr, 0, NI_NUMERICHOST) == 0) {
          found_ip = host;
          found_name = ifa_name;
          break;
        }
      }
    }
    if (found_ip.empty()) {
      err_msg = "Interface '" + target_iface + "' not found or has no IPv4 address";
    }
  } else {
    // Auto-detect: exclude loopback and look for 192.168.123.0/24 subnet
    std::vector<std::pair<std::string, std::string>> candidates;

    for (struct ifaddrs *ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
      if (ifa->ifa_addr == nullptr || ifa->ifa_addr->sa_family != AF_INET) {
        continue;
      }
      std::string ifa_name(ifa->ifa_name);
      if (ifa_name == "lo" || (ifa->ifa_flags & IFF_LOOPBACK)) {
        continue;
      }
      char host[NI_MAXHOST];
      if (getnameinfo(ifa->ifa_addr, sizeof(struct sockaddr_in), host, NI_MAXHOST, nullptr, 0, NI_NUMERICHOST) == 0) {
        std::string ip(host);
        if (ip.find("192.168.123.") == 0) {
          candidates.push_back({ifa_name, ip});
        }
      }
    }

    if (candidates.size() == 1) {
      found_name = candidates[0].first;
      found_ip = candidates[0].second;
    } else if (candidates.empty()) {
      err_msg = "No suitable interface found on subnet 192.168.123.0/24";
    } else {
      std::ostringstream oss;
      oss << "Multiple interface candidates found on subnet 192.168.123.0/24: ";
      for (size_t i = 0; i < candidates.size(); ++i) {
        oss << candidates[i].first << "(" << candidates[i].second << ")" << (i + 1 < candidates.size() ? ", " : "");
      }
      err_msg = oss.str();
    }
  }

  freeifaddrs(ifaddr);

  if (!found_ip.empty()) {
    resolved_interface_ = found_name;
    return found_ip;
  }
  return "";
}

bool MicUdpReceiver::setup_socket(std::string & err_msg)
{
  sock_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (sock_ < 0) {
    err_msg = "Socket creation failed: " + std::string(strerror(errno));
    return false;
  }

  // Enable SO_REUSEADDR
  int opt = 1;
  if (setsockopt(sock_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
    err_msg = "setsockopt(SO_REUSEADDR) failed: " + std::string(strerror(errno));
    return false;
  }

  // Best-effort SO_REUSEPORT (guard failure)
#ifdef SO_REUSEPORT
  setsockopt(sock_, SOL_SOCKET, SO_REUSEPORT, &opt, sizeof(opt));
#endif

  // Bind to INADDR_ANY
  struct sockaddr_in local_addr{};
  local_addr.sin_family = AF_INET;
  local_addr.sin_port = htons(port_);
  local_addr.sin_addr.s_addr = INADDR_ANY;

  if (bind(sock_, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0) {
    err_msg = "Socket bind to port " + std::to_string(port_) + " failed: " + std::string(strerror(errno));
    return false;
  }

  // Set receive timeout
  struct timeval tv;
  tv.tv_sec = receive_timeout_ms_ / 1000;
  tv.tv_usec = (receive_timeout_ms_ % 1000) * 1000;
  if (setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
    err_msg = "setsockopt(SO_RCVTIMEO) failed: " + std::string(strerror(errno));
    return false;
  }

  // Join multicast group on resolved IP interface (No silent fallback to INADDR_ANY!)
  struct ip_mreq mreq{};
  if (inet_pton(AF_INET, multicast_group_.c_str(), &mreq.imr_multiaddr) != 1) {
    err_msg = "Invalid multicast group: " + multicast_group_;
    return false;
  }
  mreq.imr_interface.s_addr = inet_addr(resolved_ip_.c_str());

  if (setsockopt(sock_, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0) {
    err_msg = "setsockopt(IP_ADD_MEMBERSHIP) on interface " + resolved_interface_ + " (" + resolved_ip_ + ") failed: " + std::string(strerror(errno));
    return false;
  }

  return true;
}

void MicUdpReceiver::cleanup_socket()
{
  if (sock_ >= 0) {
    // Best-effort drop multicast membership before closing
    struct ip_mreq mreq{};
    if (inet_pton(AF_INET, multicast_group_.c_str(), &mreq.imr_multiaddr) == 1 && !resolved_ip_.empty()) {
      mreq.imr_interface.s_addr = inet_addr(resolved_ip_.c_str());
      setsockopt(sock_, IPPROTO_IP, IP_DROP_MEMBERSHIP, &mreq, sizeof(mreq));
    }

    close(sock_);
    sock_ = -1;
  }
}

void MicUdpReceiver::run()
{
  std::vector<char> buffer(receive_buffer_bytes_);

  while (!stop_requested_) {
    ssize_t received = recvfrom(sock_, buffer.data(), buffer.size(), 0, nullptr, nullptr);

    if (stop_requested_) {
      break;
    }

    if (received < 0) {
      if (stop_requested_) {
        break;
      }
      if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
        // Timeout or signal interruption, check loop condition
        continue;
      }
      // Fatal socket error or shutdown
      if (error_cb_) {
        error_cb_("Fatal socket receive error: " + std::string(strerror(errno)), true);
      }
      break;
    }

    if (received == 0) {
      // Empty packet
      continue;
    }

    // Packet byte validation (12.2)
    if (received % 2 != 0) {
      if (error_cb_) {
        error_cb_("Odd byte count (" + std::to_string(received) + ") in UDP microphone packet, dropping whole packet", false);
      }
      continue;
    }

    // PCM S16LE Decoding with explicit little-endian conversion (12.1)
    size_t sample_count = received / 2;
    std::vector<int16_t> samples(sample_count);
    const uint8_t * ubytes = reinterpret_cast<const uint8_t *>(buffer.data());
    for (size_t i = 0; i < sample_count; ++i) {
      uint16_t raw = static_cast<uint16_t>(ubytes[i * 2]) | (static_cast<uint16_t>(ubytes[i * 2 + 1]) << 8);
      samples[i] = static_cast<int16_t>(raw);
    }

    // Update sequence number
    {
      std::lock_guard<std::mutex> lock(mutex_);
      packet_sequence_++;
    }
    packet_cv_.notify_all();

    // Invoke data callback
    if (data_cb_) {
      data_cb_(samples);
    }
  }
}

} // namespace erasers_g1_common
