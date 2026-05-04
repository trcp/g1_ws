#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <netdb.h>

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "g1/g1_audio_client.hpp"

class G1MicServer : public rclcpp::Node {
 public:
  G1MicServer() : Node("mic_server") {
    // Parameters
    this->declare_parameter<std::string>("nic", "");
    this->get_parameter("nic", nic_);

    // Publisher & Service
    audio_pub_ = this->create_publisher<std_msgs::msg::Int16MultiArray>("/audio/raw", 10);
    
    // Create dedicated callback group for the service to prevent deadlock
    service_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    srv_server_ = this->create_service<std_srvs::srv::SetBool>(
        "mic_rec",
        std::bind(&G1MicServer::handle_mic_rec, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        service_cb_group_);

    // VUI Client initialization (using /api/voice since /api/vui returned ERROR 100)
    vui_client_ = std::make_unique<BaseClient>(this, "/api/voice/request", "/api/voice/response");

    is_recording_ = false;

    // Initialization
    if (!init_socket()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize UDP socket.");
    } else {
      RCLCPP_INFO(this->get_logger(), "Mic Server initialized and waiting for service /mic_rec.");
      // Start receiver thread
      receive_thread_ = std::thread(&G1MicServer::receive_loop, this);
    }
  }

  ~G1MicServer() {
    stop_flag_ = true;
    if (receive_thread_.joinable()) {
      receive_thread_.join();
    }
    if (sock_ >= 0) {
      close(sock_);
    }
  }

 private:
  std::string get_local_ip(const std::string &interface_name) {
    struct ifaddrs *ifaddr, *ifa;
    std::string ip = "0.0.0.0";

    if (getifaddrs(&ifaddr) == -1) {
      return ip;
    }

    for (ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next) {
      if (ifa->ifa_addr == NULL || ifa->ifa_addr->sa_family != AF_INET) continue;

      std::string current_ifa_name(ifa->ifa_name);
      char host[NI_MAXHOST];
      getnameinfo(ifa->ifa_addr, sizeof(struct sockaddr_in), host, NI_MAXHOST, NULL, 0, NI_NUMERICHOST);
      std::string current_ip(host);

      // interface_name が指定されている場合
      if (!interface_name.empty()) {
        if (current_ifa_name == interface_name) {
          ip = current_ip;
          break;
        }
      } else {
        // 自動選択：192.168.123.x サブネットを優先
        if (current_ip.find("192.168.123.") == 0) {
          ip = current_ip;
          RCLCPP_INFO(this->get_logger(), "Auto-selected interface: %s (IP: %s)", current_ifa_name.c_str(), ip.c_str());
          break;
        }
      }
    }

    freeifaddrs(ifaddr);
    return ip;
  }

  bool init_socket() {
    sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_ < 0) return false;

    int reuse = 1;
    setsockopt(sock_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(5555);

    if (bind(sock_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
      close(sock_);
      return false;
    }

    // Multicast membership
    struct ip_mreq mreq;
    mreq.imr_multiaddr.s_addr = inet_addr("239.168.123.161");
    
    std::string local_ip = get_local_ip(nic_);
    mreq.imr_interface.s_addr = inet_addr(local_ip.c_str());

    if (setsockopt(sock_, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0) {
      RCLCPP_WARN(this->get_logger(), "Failed to join multicast group on %s. Trying default.", local_ip.c_str());
      mreq.imr_interface.s_addr = htonl(INADDR_ANY);
      if (setsockopt(sock_, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0) {
        close(sock_);
        return false;
      }
    }

    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    return true;
  }

  void handle_mic_rec(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                      std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    bool target_state = request->data;
    
    unitree_api::msg::Request vreq;
    int32_t ret = -1;

    if (target_state) {
      RCLCPP_INFO(this->get_logger(), "Activating G1 Microphone (API 1007)...");
      vreq.header.identity.api_id = 1007; // Vui_Api_Open_Mic
      vreq.parameter = "{}";
      ret = vui_client_->Call(vreq);
    } else {
      RCLCPP_INFO(this->get_logger(), "Deactivating G1 Microphone (API 1008)...");
      vreq.header.identity.api_id = 1008; // Vui_Api_Close_Mic
      vreq.parameter = "{}";
      ret = vui_client_->Call(vreq);
    }

    if (ret == 0) {
      is_recording_ = target_state;
      response->success = true;
      response->message = is_recording_ ? "Recording enabled and VUI opened" : "Recording disabled and VUI closed";
      RCLCPP_INFO(this->get_logger(), "VUI RPC Success: %s", response->message.c_str());
    } else {
      response->success = false;
      response->message = "VUI RPC Failed with code: " + std::to_string(ret) + ". Check if VUI service is running.";
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
      // Even if RPC fails, we might still want to toggle local recording? 
      // No, safety first. If we can't open mic, don't pretend we are recording.
    }
  }

  void receive_loop() {
    uint8_t buffer[8192];
    auto message = std_msgs::msg::Int16MultiArray();

    while (rclcpp::ok() && !stop_flag_) {
      ssize_t n = recvfrom(sock_, buffer, sizeof(buffer), 0, NULL, NULL);
      if (n > 0 && is_recording_) {
        // Convert byte buffer to int16_t vector
        size_t samples_count = n / 2;
        message.data.resize(samples_count);
        memcpy(message.data.data(), buffer, samples_count * 2);
        
        audio_pub_->publish(message);
      }
      // Small sleep to avoid CPU spinning if recvfrom is non-blocking (though we set timeout)
    }
  }

  int sock_ = -1;
  std::string nic_;
  std::atomic<bool> is_recording_{false};
  std::atomic<bool> stop_flag_{false};
  std::thread receive_thread_;
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr audio_pub_;
  rclcpp::CallbackGroup::SharedPtr service_cb_group_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_server_;
  std::unique_ptr<BaseClient> vui_client_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<G1MicServer>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
