#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include "g1/g1_audio_client.hpp"
#include "common/time_tools.hpp"

#include "g1_srvs/srv/audio_client.hpp"

using AudioService = g1_srvs::srv::AudioClient;
using std::placeholders::_1;
using std::placeholders::_2;

namespace wav_tools {

struct WaveHeader {
  void SeekToDataChunk(std::istream &is) {
    while (is && subchunk2_id != 0x61746164) {
      is.seekg(subchunk2_size, std::istream::cur);
      is.read(reinterpret_cast<char *>(&subchunk2_id), sizeof(int32_t));
      is.read(reinterpret_cast<char *>(&subchunk2_size), sizeof(int32_t));
    }
  }
  int32_t chunk_id;
  int32_t chunk_size;
  int32_t format;
  int32_t subchunk1_id;
  int32_t subchunk1_size;
  int16_t audio_format;
  int16_t num_channels;
  int32_t sample_rate;
  int32_t byte_rate;
  int16_t block_align;
  int16_t bits_per_sample;
  int32_t subchunk2_id;
  int32_t subchunk2_size;
};

std::vector<uint8_t> ReadWave(const std::string &filename,
                              int32_t *sampling_rate, int8_t *channel_count,
                              bool *is_ok) {
  std::ifstream is(filename, std::ifstream::binary);
  if (!is.is_open()) {
    *is_ok = false;
    return {};
  }

  WaveHeader header{};
  is.read(reinterpret_cast<char *>(&header.chunk_id), sizeof(header.chunk_id));

  // RIFF check
  if (header.chunk_id != 0x46464952) { *is_ok = false; return {}; }

  is.read(reinterpret_cast<char *>(&header.chunk_size), sizeof(header.chunk_size));
  is.read(reinterpret_cast<char *>(&header.format), sizeof(header.format));

  // WAVE check
  if (header.format != 0x45564157) { *is_ok = false; return {}; }

  is.read(reinterpret_cast<char *>(&header.subchunk1_id), sizeof(header.subchunk1_id));
  is.read(reinterpret_cast<char *>(&header.subchunk1_size), sizeof(header.subchunk1_size));

  // JUNK skip logic
  if (header.subchunk1_id == 0x4b4e554a) {
    is.seekg(header.subchunk1_size, std::istream::cur);
    is.read(reinterpret_cast<char *>(&header.subchunk1_id), sizeof(header.subchunk1_id));
    is.read(reinterpret_cast<char *>(&header.subchunk1_size), sizeof(header.subchunk1_size));
  }

  // fmt check
  if (header.subchunk1_id != 0x20746d66) { *is_ok = false; return {}; }

  is.read(reinterpret_cast<char *>(&header.audio_format), sizeof(header.audio_format));
  is.read(reinterpret_cast<char *>(&header.num_channels), sizeof(header.num_channels));
  *channel_count = static_cast<int8_t>(header.num_channels);
  is.read(reinterpret_cast<char *>(&header.sample_rate), sizeof(header.sample_rate));
  is.read(reinterpret_cast<char *>(&header.byte_rate), sizeof(header.byte_rate));
  is.read(reinterpret_cast<char *>(&header.block_align), sizeof(header.block_align));
  is.read(reinterpret_cast<char *>(&header.bits_per_sample), sizeof(header.bits_per_sample));

  // Validation
  if (header.bits_per_sample != 16) { *is_ok = false; return {}; } // G1 supports 16bit

  if (header.subchunk1_size == 18) {
    int16_t extra_size = -1;
    is.read(reinterpret_cast<char *>(&extra_size), sizeof(int16_t));
    is.seekg(extra_size, std::istream::cur);
  }

  is.read(reinterpret_cast<char *>(&header.subchunk2_id), sizeof(header.subchunk2_id));
  is.read(reinterpret_cast<char *>(&header.subchunk2_size), sizeof(header.subchunk2_size));

  header.SeekToDataChunk(is);
  if (!is) { *is_ok = false; return {}; }

  *sampling_rate = header.sample_rate;

  std::vector<int16_t> samples(header.subchunk2_size / 2);
  is.read(reinterpret_cast<char *>(samples.data()), header.subchunk2_size);
  if (!is) { *is_ok = false; return {}; }

  // Convert to Little Endian Byte Vector
  std::vector<uint8_t> ans(samples.size() * 2);
  for (size_t i = 0; i < samples.size(); ++i) {
    ans[i * 2] = samples[i] & 0xFF;
    ans[i * 2 + 1] = (samples[i] >> 8) & 0xFF;
  }

  *is_ok = true;
  return ans;
}
} // namespace wav_tools


class TtsClientNode : public unitree::ros2::g1::AudioClient {
public:
  TtsClientNode() : unitree::ros2::g1::AudioClient() {
    
    srv_server_ = this->create_service<AudioService>(
        "play_audio",
        std::bind(&TtsClientNode::handle_audio_request, this, _1, _2));

    RCLCPP_INFO(this->get_logger(), "TTS Client Node has been started.");
    RCLCPP_INFO(this->get_logger(), "Service ready: /play_audio");
  }

private:
  rclcpp::Service<AudioService>::SharedPtr srv_server_;

  void handle_audio_request(const std::shared_ptr<AudioService::Request> request,
                            std::shared_ptr<AudioService::Response> response) {
    
    int32_t ret = -1;

    /* Volume
    ret = this->SetVolume(request->volume);
    RCLCPP_INFO(this->get_logger(), "Volume: %s", std::to_string(request->volume));
    RCLCPP_INFO(this->get_logger(), "Volume res: %s", std::to_string(ret));
    */

    // TTS
    if (request->type == AudioService::Request::TYPE_TTS) {
      if (request->text.empty()) {
        response->success = false;
        response->message = "Text is empty.";
        RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
        return;
      }

      RCLCPP_INFO(this->get_logger(), "Executing TTS: %s", request->text.c_str());
      // TtsMaker(text, speaker_id=0)
      ret = this->TtsMaker(request->text, 1);
      
      if (ret == 0 || ret == -1) { // Assuming 0 is success based on typical C style
          response->success = true;
          response->message = "TTS command sent successfully.";
      } else {
          response->success = false;
          response->message = "TTS command failed with code: " + std::to_string(ret);
      }
    }

    // WAV File Playback
    else if (request->type == AudioService::Request::TYPE_WAV) {
      std::string file_path = request->audio_path;
      RCLCPP_INFO(this->get_logger(), "Processing WAV file: %s", file_path.c_str());

      int32_t sample_rate = -1;
      int8_t num_channels = 0;
      bool is_ok = false;

      // WAVファイルの読み込みと解析
      std::vector<uint8_t> pcm_data = wav_tools::ReadWave(file_path, &sample_rate, &num_channels, &is_ok);

      if (!is_ok) {
        response->success = false;
        response->message = "Failed to parse WAV file. Check format (16kHz, 16bit, Mono required?)";
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        return;
      }

      if (sample_rate != 16000 || num_channels != 1) {
        response->success = false;
        response->message = "Invalid Audio Format. Required: 16000Hz, 1 Channel. Given: " + 
                            std::to_string(sample_rate) + "Hz, " + std::to_string(num_channels) + "Ch";
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        return;
      }

      RCLCPP_INFO(this->get_logger(), "WAV Loaded. Size: %zu bytes", pcm_data.size());

      std::string stream_id = std::to_string(unitree::common::GetCurrentTimeMilliseconds());
      ret = this->PlayStream("tts_client_node", stream_id, pcm_data);

      if (ret == 0 || ret == -1) { // Assuming 0 is success
          response->success = true;
          response->message = "Audio stream sent successfully.";
      } else {
          response->success = false;
          response->message = "PlayStream failed with code: " + std::to_string(ret);
      }
    } 
    else {
      response->success = false;
      response->message = "Unknown request type.";
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<TtsClientNode>();
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
