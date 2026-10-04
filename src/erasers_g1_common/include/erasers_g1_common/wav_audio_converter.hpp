// Copyright (c) 2026 Nakatogawa Laboratory. All rights reserved.

#ifndef ERASERS_G1_COMMON__WAV_AUDIO_CONVERTER_HPP_
#define ERASERS_G1_COMMON__WAV_AUDIO_CONVERTER_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace erasers_g1_common
{

struct ConvertedAudio
{
  std::vector<uint8_t> pcm_s16le;
  uint32_t source_sample_rate{0};
  uint16_t source_channels{0};
  uint16_t source_bits_per_sample{0};
  double source_duration_sec{0.0};
  double output_duration_sec{0.0};
};

bool convert_pcm_wav_to_robot_audio(
  const std::vector<uint8_t> & wav_bytes,
  ConvertedAudio & converted,
  std::string & error_message,
  std::size_t max_output_bytes = 32U * 1024U * 1024U);

}  // namespace erasers_g1_common

#endif  // ERASERS_G1_COMMON__WAV_AUDIO_CONVERTER_HPP_
