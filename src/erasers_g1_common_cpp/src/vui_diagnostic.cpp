#include <iostream>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include "g1/g1_audio_client.hpp"

using namespace unitree::ros2::g1;

class VuiDiagnosticNode : public AudioClient {
public:
  VuiDiagnosticNode() : AudioClient() {
    RCLCPP_INFO(this->get_logger(), "VUI Diagnostic Node Started.");
  }

  void run_diagnosis() {
    RCLCPP_INFO(this->get_logger(), "--- Starting VUI Service Diagnosis (Aggressive) ---");

    std::vector<std::string> namespaces = {"voice", "vui", "audiohub"};
    for (const auto& ns : namespaces) {
      std::string req_topic = "/api/" + ns + "/request";
      std::string res_topic = "/api/" + ns + "/response";
      RCLCPP_INFO(this->get_logger(), "[Step] Testing Namespace: %s", ns.c_str());
      
      BaseClient client(this, req_topic, res_topic);
      
      // 1. Get Volume Check
      unitree_api::msg::Request req;
      req.header.identity.api_id = 1005; // Voice/VUI Get Volume
      int32_t ret = client.Call(req);
      report_status(ns + " (API 1005)", ret);

      // 2. Open Mic Check (Partial)
      if (ret == 0) {
        RCLCPP_INFO(this->get_logger(), ">> SUCCESS on namespace: %s", ns.c_str());
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
      case 3103: msg = "ERROR 3103: API Not Registered (Service might be old or disabled)"; break;
      case 3104: msg = "ERROR 3104: Request Timeout (Robot not responding)"; break;
      case 3202: msg = "ERROR 3202: Internal Server Error (VUI module crashed?)"; break;
      case 3205: msg = "ERROR 3205: Request Denied (Service busy or locked)"; break;
      case -100: msg = "ERROR: Local Timeout (DDS level failure)"; break;
      default: msg = "ERROR " + std::to_string(code) + ": Unknown error"; break;
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
  
  // Run once and exit
  node->run_diagnosis();
  
  rclcpp::shutdown();
  return 0;
}
