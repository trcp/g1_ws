#include <chrono>
#include <fstream>
#include <future>
#include <iterator>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "erasers_g1_interfaces/action/vui_audio.hpp"
#include "erasers_g1_interfaces/action/vui_tts.hpp"
#include "erasers_g1_interfaces/srv/audio_client.hpp"
#include "erasers_g1_common/wav_audio_converter.hpp"

class AudioClientNode : public rclcpp::Node
{
  using Audio = erasers_g1_interfaces::action::VuiAudio;
  using TTS = erasers_g1_interfaces::action::VuiTTS;
  using Service = erasers_g1_interfaces::srv::AudioClient;
public:
  AudioClientNode() : Node("audio_client")
  {
    client_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    tts_ = rclcpp_action::create_client<TTS>(this, "/vui_tts", client_group_);
    audio_ = rclcpp_action::create_client<Audio>(this, "/vui_audio", client_group_);
    service_ = create_service<Service>("/play_audio",
      [this](std::shared_ptr<Service::Request> request, std::shared_ptr<Service::Response> response) {
        response->success = false;
        try {
          if (request->type == Service::Request::TYPE_TTS) {
            if (request->text.empty()) {response->message = "テキストが空です"; return;}
            TTS::Goal goal;
            goal.text = request->text;
            response->success = send<TTS>(tts_, goal);
          } else if (request->type == Service::Request::TYPE_WAV) {
            std::ifstream input(request->audio_path, std::ios::binary | std::ios::ate);
            if (!input || input.tellg() <= 0 || input.tellg() > 10485760) {
              response->message = "WAV が読めないか、容量上限を超えています";
              return;
            }
            input.seekg(0);
            Audio::Goal goal;
            goal.source_type = Audio::Goal::SOURCE_WAV_BYTES;
            goal.audio_data.assign(std::istreambuf_iterator<char>(input), {});
            response->success = send<Audio>(audio_, goal);
          } else {
            response->message = "未対応の音声種別です";
            return;
          }
          // 既存サービスは要求受付を返し、再生完了は Action の結果で通知する。
          response->message = response->success ? "音声要求を受け付けました" : "音声要求を受け付けられません";
        } catch (const std::exception & error) {response->message = error.what();}
      });
  }
private:
  template<class Action>
  bool send(const typename rclcpp_action::Client<Action>::SharedPtr & client,
    const typename Action::Goal & goal)
  {
    if (!client->action_server_is_ready()) {return false;}
    auto future = client->async_send_goal(goal);
    if (future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {return false;}
    const auto handle = future.get();
    return handle && handle->get_status() != action_msgs::msg::GoalStatus::STATUS_UNKNOWN;
  }
  rclcpp::CallbackGroup::SharedPtr client_group_;
  rclcpp_action::Client<TTS>::SharedPtr tts_;
  rclcpp_action::Client<Audio>::SharedPtr audio_;
  rclcpp::Service<Service>::SharedPtr service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AudioClientNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
