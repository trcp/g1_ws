#include <rclcpp/rclcpp.hpp>
#include <cstdlib>
#include <chrono>
#include <thread>
#include <sstream>
#include <string>
#include <cstdlib>

int main(int argc, char ** argv)
{
  // ROS2 の初期化
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("auto_map_saver");

  // パラメータの宣言（map_pathの初期値を空に設定）
  node->declare_parameter("map_path", std::string("/home/unitree/colcon_ws/map"));
  node->declare_parameter("map_name", std::string("test_map"));
  node->declare_parameter("save_late", 5);

  // パラメータの取得
  std::string map_path = node->get_parameter("map_path").as_string();
  
  // map_pathが空の場合はエラーログを出力して終了
  if (map_path.empty()) {
    RCLCPP_ERROR(node->get_logger(), "map_path parameter must be set!");
    rclcpp::shutdown();
    return 1;
  }

  std::string map_name = node->get_parameter("map_name").as_string();
  int save_late = node->get_parameter("save_late").as_int();

  // 1. ノードの名前空間を取得
  std::string ns = node->get_namespace();

  // 2. 名前空間に基づいてマップのトピック名を決定
  std::string map_topic = "/map";
  // 名前空間がグローバル('/')でない場合、先頭に付与する
  if (ns != "/") {
    map_topic = ns + map_topic;
  }
  RCLCPP_INFO(node->get_logger(), "Target map topic is: %s", map_topic.c_str());

  // 3. トピック名をリマッピングするコマンド文字列を作成
  std::ostringstream oss;
  oss << "ros2 run nav2_map_server map_saver_cli "
      << "-f " << map_path << "/" << map_name << " "
      << "--ros-args -r map:=" << map_topic;
  std::string cmd = oss.str();
  RCLCPP_INFO(node->get_logger(), "Executing command: %s", cmd.c_str());

  // 定期的にマップ保存コマンドを実行するループ
  while (rclcpp::ok())
  {
    // コマンド実行（戻り値が 0 なら成功）
    int ret = std::system(cmd.c_str());
    if (ret == 0)
    {
      RCLCPP_INFO(node->get_logger(), "saved %s", map_path.c_str());
    }
    else
    {
      RCLCPP_ERROR(node->get_logger(), "map does not exist!");
    }
    // 指定秒数待機
    std::this_thread::sleep_for(std::chrono::seconds(save_late));
  }

  // SIGINT (Ctrl+C) により shutdown された場合、最終保存処理を実施
  int ret = std::system(cmd.c_str());
  if (ret == 0)
  {
    RCLCPP_INFO(node->get_logger(), "saved %s", map_path.c_str());
  }
  RCLCPP_INFO(node->get_logger(), "Done ...");

  rclcpp::shutdown();
  return 0;
}
