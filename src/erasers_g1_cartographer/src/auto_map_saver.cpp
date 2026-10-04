#include <rclcpp/rclcpp.hpp>
#include <cartographer_ros_msgs/srv/write_state.hpp>
#include <cstdlib>
#include <chrono>
#include <thread>
#include <sstream>
#include <string>
#include <wordexp.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;

class AutoMapSaver : public rclcpp::Node
{
public:
  AutoMapSaver() : Node("auto_map_saver")
  {
    this->declare_parameter("map_path", std::string("~/map"));
    this->declare_parameter("map_name", std::string("map"));
    this->declare_parameter("save_late", 5000); // ms

    map_path_ = this->get_parameter("map_path").as_string();
    map_name_ = this->get_parameter("map_name").as_string();
    save_late_ = this->get_parameter("save_late").as_int();

    // Expand path (~ etc.)
    map_path_ = expand_path(map_path_);

    // Create directory
    std::string mkdir_cmd = "mkdir -p " + map_path_;
    int ret = std::system(mkdir_cmd.c_str());
    if (ret != 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create directory: %s", map_path_.c_str());
    }

    // Cartographer service client
    client_write_state_ = this->create_client<cartographer_ros_msgs::srv::WriteState>("/write_state");

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(save_late_), // save_late is in ms
      std::bind(&AutoMapSaver::save_map_callback, this)
    );

    RCLCPP_INFO(this->get_logger(), "Auto Map Saver initialized. Saving every %d ms to %s", save_late_, map_path_.c_str());
  }

private:
  void save_map_callback()
  {
    save_pgm();
    save_pbstream();
  }

  void save_pgm()
  {
    // Use map_saver_cli from nav2_map_server
    // ros2 run nav2_map_server map_saver_cli -f <map_path>/<map_name> --ros-args -p map_subscribe_transient_local:=true -r map:=/map
    // Note: map_subscribe_transient_local might be needed depending on QoS. Cartographer usually publishes transient local map? 
    // Actually cartographer_occupancy_grid_node publishes /map. QoS is usually Transient Local for maps.
    
    std::string map_file_path = map_path_ + "/" + map_name_;
    std::ostringstream oss;
    oss << "ros2 run nav2_map_server map_saver_cli "
        << "-f " << map_file_path << " "
        << "--ros-args -p map_subscribe_transient_local:=true "
        << "-p save_map_timeout:=5.0 ";
        // << "> /dev/null 2>&1"; // Enable output for debugging
    
    // Asynchronous execution via system() blocks, but map saving might take time. 
    // Ideally we should use a separate thread or non-blocking call, but for simplicity we use system().
    // Running in background via & might be better but checking status is harder.
    // For now, let's just run it. The timer callback will block, which is fine as long as we don't block main thread forever?
    // Wait, timer callback runs in the executor. Blocking it blocks other callbacks in this node.
    // Since this node only does saving, it's acceptable.

    // Force overwrite? map_saver_cli usually overwrites.
    
    int ret = std::system(oss.str().c_str());
    if (ret == 0) {
      RCLCPP_INFO(this->get_logger(), "Saved PGM map to %s", map_file_path.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "Failed to save PGM map (maybe map_server not ready or busy)");
    }
  }

  void save_pbstream()
  {
    if (!client_write_state_->service_is_ready()) {
      RCLCPP_WARN(this->get_logger(), "Cartographer /write_state service not ready");
      return;
    }

    auto request = std::make_shared<cartographer_ros_msgs::srv::WriteState::Request>();
    request->filename = map_path_ + "/" + map_name_ + ".pbstream";
    request->include_unfinished_submaps = true;

    auto result_future = client_write_state_->async_send_request(request, 
      [this](rclcpp::Client<cartographer_ros_msgs::srv::WriteState>::SharedFuture future) {
        try {
            auto response = future.get();
            if (response->status.code == 0) {
            RCLCPP_INFO(this->get_logger(), "Saved PBSTREAM to %s", (map_path_ + "/" + map_name_ + ".pbstream").c_str());
            } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to save PBSTREAM: %s", response->status.message.c_str());
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
        }
      });
  }

  std::string expand_path(std::string path) {
    wordexp_t exp_result;
    if (wordexp(path.c_str(), &exp_result, 0) == 0) {
      if(exp_result.we_wordc > 0) {
          std::string expanded = exp_result.we_wordv[0];
          wordfree(&exp_result);
          return expanded;
      }
      wordfree(&exp_result);
    }
    return path;
  }

  std::string map_path_;
  std::string map_name_;
  int save_late_;
  rclcpp::Client<cartographer_ros_msgs::srv::WriteState>::SharedPtr client_write_state_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutoMapSaver>());
  rclcpp::shutdown();
  return 0;
}
