#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "g1/g1_audio_client.hpp"

using namespace unitree::ros2::g1;

class VuiDiagnosticNode : public AudioClient {
public:
  VuiDiagnosticNode() : AudioClient() {
    RCLCPP_INFO(this->get_logger(), "VUI Diagnostic Node Started.");
  }

  void run_diagnosis() {
    RCLCPP_INFO(this->get_logger(), "--- Starting System-Wide Service Diagnosis ---");

    std::vector<std::string> namespaces = {
      "voice", "vui", "audiohub", "videohub", 
      "sport", "loco", "robot_state", "gpt", "motion_switcher"
    };

    for (const auto& ns : namespaces) {
      std::string req_topic = "/api/" + ns + "/request";
      std::string res_topic = "/api/" + ns + "/response";
      RCLCPP_INFO(this->get_logger(), "[Step] Testing Namespace: %s", ns.c_str());
      
      BaseClient client(this, req_topic, res_topic);
      
      // Wait for DDS discovery
      rclcpp::sleep_for(std::chrono::milliseconds(1500));
      
      // Sending API 1001 (often GetServerApiVersion) to test if service responds
      unitree_api::msg::Request req;
      req.header.identity.api_id = 1001; 
      int32_t ret = client.Call(req);
      report_status(ns + " (API 1001)", ret);

      if (ret != -1 && ret != -100) {
        RCLCPP_INFO(this->get_logger(), ">> SERVICE IS ALIVE on namespace: %s", ns.c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(), ">> SERVICE DEAD/UNREACHABLE on namespace: %s", ns.c_str());
      }
    }

    RCLCPP_INFO(this->get_logger(), "--- Diagnosis Finished ---");
  }

private:
  int32_t call_api(int32_t api_id, const std::string& parameter) {
    unitree_api::msg::Request req;
    req.header.identity.api_id = api_id;
    req.parameter = parameter;
    
    // We use the internal base_client_ from AudioClient
    // Note: base_client_ is private in AudioClient? Let me check g1_audio_client.hpp again.
    // Ah, base_client_ is private. I might need to use a custom BaseClient or modify AudioClient.
    // Wait, AudioClient in g1_audio_client.hpp has base_client_ as private.
    
    // Let's implement a local BaseClient for diagnostic if needed, 
    // or just use AudioClient's existing methods if I can.
    // Since I can't modify the header easily without a plan, 
    // I'll create a local BaseClient in this node.
    return diagnostic_base_client_.Call(req);
  }

  void report_status(const std::string& action, int32_t code) {
    std::string msg;
    switch (code) {
      case 0: msg = "OK (Success)"; break;
      case 3103: msg = "ERROR 3103: API Not Registered (Service alive but API missing)"; break;
      case 3104: msg = "ERROR 3104: Request Timeout (Robot not responding)"; break;
      case 3202: msg = "ERROR 3202: Internal Server Error/Crash"; break;
      case 3205: msg = "ERROR 3205: Request Denied (Service busy or locked)"; break;
      case -100: msg = "ERROR -100: Local Timeout (DDS failure)"; break;
      case -1: msg = "ERROR -1: Error / Local Timeout"; break;
      default: msg = "ERROR " + std::to_string(code) + ": Unknown error / Alive"; break;
    }
    
    if (code == 0) {
      RCLCPP_INFO(this->get_logger(), "Result for %s: %s", action.c_str(), msg.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Result for %s: %s", action.c_str(), msg.c_str());
    }
  }

  // Define a local BaseClient since AudioClient's is private
  // Topic names from g1_audio_client.hpp: "/api/voice/request", "/api/voice/response"
  BaseClient diagnostic_base_client_{this, "/api/voice/request", "/api/voice/response"};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VuiDiagnosticNode>();
  
  std::thread diag_thread([node]() {
    // wait a bit for node initialization
    rclcpp::sleep_for(std::chrono::milliseconds(1000));
    node->run_diagnosis();
    rclcpp::shutdown();
  });
  
  // Spin is REQUIRED for BaseClient's subscriber callback to be invoked
  rclcpp::spin(node);
  
  if (diag_thread.joinable()) {
    diag_thread.join();
  }
  
  return 0;
}
