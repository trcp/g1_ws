// Copyright (c) 2026 Nakatogawa Laboratory. All rights reserved.

#include <gtest/gtest.h>
#include <openssl/sha.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "erasers_g1_common/wav_audio_converter.hpp"

namespace erasers_g1_common
{
namespace
{

void append_u16(std::vector<uint8_t> & bytes, uint16_t value)
{
  bytes.push_back(static_cast<uint8_t>(value & 0xFFU));
  bytes.push_back(static_cast<uint8_t>((value >> 8U) & 0xFFU));
}

void append_u32(std::vector<uint8_t> & bytes, uint32_t value)
{
  for (unsigned int shift = 0U; shift < 32U; shift += 8U) {
    bytes.push_back(static_cast<uint8_t>((value >> shift) & 0xFFU));
  }
}

void overwrite_u32(std::vector<uint8_t> & bytes, std::size_t offset, uint32_t value)
{
  for (unsigned int index = 0U; index < 4U; ++index) {
    bytes[offset + index] = static_cast<uint8_t>((value >> (index * 8U)) & 0xFFU);
  }
}

std::vector<uint8_t> make_wav(
  uint32_t sample_rate,
  uint16_t channels,
  uint16_t bits_per_sample,
  const std::vector<int32_t> & interleaved_samples,
  uint16_t format_tag = 1U,
  bool add_odd_unknown_chunk = false)
{
  const uint16_t bytes_per_sample = bits_per_sample / 8U;
  const uint16_t block_align = channels * bytes_per_sample;
  std::vector<uint8_t> bytes{'R', 'I', 'F', 'F', 0, 0, 0, 0, 'W', 'A', 'V', 'E'};

  if (add_odd_unknown_chunk) {
    bytes.insert(bytes.end(), {'J', 'U', 'N', 'K'});
    append_u32(bytes, 3U);
    bytes.insert(bytes.end(), {1U, 2U, 3U, 0U});
  }

  bytes.insert(bytes.end(), {'f', 'm', 't', ' '});
  append_u32(bytes, 16U);
  append_u16(bytes, format_tag);
  append_u16(bytes, channels);
  append_u32(bytes, sample_rate);
  append_u32(bytes, sample_rate * block_align);
  append_u16(bytes, block_align);
  append_u16(bytes, bits_per_sample);

  bytes.insert(bytes.end(), {'d', 'a', 't', 'a'});
  const std::size_t data_size_offset = bytes.size();
  append_u32(bytes, 0U);
  const std::size_t data_start = bytes.size();

  for (const int32_t sample : interleaved_samples) {
    if (bits_per_sample == 8U) {
      bytes.push_back(static_cast<uint8_t>(std::max(0, std::min(255, sample))));
    } else {
      const uint32_t raw = static_cast<uint32_t>(sample);
      for (uint16_t index = 0U; index < bytes_per_sample; ++index) {
        bytes.push_back(static_cast<uint8_t>((raw >> (index * 8U)) & 0xFFU));
      }
    }
  }

  overwrite_u32(bytes, data_size_offset, static_cast<uint32_t>(bytes.size() - data_start));
  if (((bytes.size() - data_start) & 1U) != 0U) {
    bytes.push_back(0U);
  }
  overwrite_u32(bytes, 4U, static_cast<uint32_t>(bytes.size() - 8U));
  return bytes;
}

std::string sha256_hex(const std::vector<uint8_t> & bytes)
{
  unsigned char digest[SHA256_DIGEST_LENGTH];
  SHA256(bytes.data(), bytes.size(), digest);
  std::ostringstream output;
  for (const unsigned char byte : digest) {
    output << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte);
  }
  return output.str();
}

double segment_rms(
  const std::vector<uint8_t> & pcm, std::size_t begin_frame, std::size_t end_frame)
{
  double sum = 0.0;
  for (std::size_t frame = begin_frame; frame < end_frame; ++frame) {
    const uint16_t raw = static_cast<uint16_t>(pcm[frame * 2U]) |
      static_cast<uint16_t>(static_cast<uint16_t>(pcm[frame * 2U + 1U]) << 8U);
    const double sample = static_cast<int16_t>(raw);
    sum += sample * sample;
  }
  return std::sqrt(sum / static_cast<double>(end_frame - begin_frame));
}

TEST(WavAudioConverterTest, Mono16kS16IsByteExactPassthrough)
{
  const std::vector<int32_t> samples{-32768, -1234, 0, 1234, 32767};
  const auto wav = make_wav(16000U, 1U, 16U, samples);
  ConvertedAudio converted;
  std::string error;
  ASSERT_TRUE(convert_pcm_wav_to_robot_audio(wav, converted, error)) << error;

  std::vector<uint8_t> expected;
  for (const int32_t sample : samples) {
    const uint16_t raw = static_cast<uint16_t>(static_cast<int16_t>(sample));
    expected.push_back(static_cast<uint8_t>(raw & 0xFFU));
    expected.push_back(static_cast<uint8_t>((raw >> 8U) & 0xFFU));
  }
  EXPECT_EQ(converted.pcm_s16le, expected);
  EXPECT_EQ(converted.source_sample_rate, 16000U);
  EXPECT_EQ(converted.source_channels, 1U);
  EXPECT_EQ(converted.source_bits_per_sample, 16U);
}

TEST(WavAudioConverterTest, SupportsRatesWidthsStereoSilenceAndOneSample)
{
  for (const uint32_t rate : {24000U, 44100U, 48000U}) {
    for (const uint16_t bits : {8U, 16U, 24U, 32U}) {
      const int32_t zero = bits == 8U ? 128 : 0;
      const auto wav = make_wav(rate, 2U, bits, {zero, zero, zero, zero});
      ConvertedAudio converted;
      std::string error;
      ASSERT_TRUE(convert_pcm_wav_to_robot_audio(wav, converted, error))
        << "rate=" << rate << " bits=" << bits << " error=" << error;
      EXPECT_FALSE(converted.pcm_s16le.empty());
      EXPECT_EQ(converted.pcm_s16le.size() % 2U, 0U);
      EXPECT_EQ(converted.source_channels, 2U);
      EXPECT_EQ(converted.source_bits_per_sample, bits);
    }
  }

  const auto one_sample_wav = make_wav(44100U, 1U, 16U, {1234});
  ConvertedAudio one_sample;
  std::string error;
  ASSERT_TRUE(convert_pcm_wav_to_robot_audio(one_sample_wav, one_sample, error)) << error;
  EXPECT_EQ(one_sample.pcm_s16le.size(), 2U);
}

TEST(WavAudioConverterTest, StereoDownmixUsesEqualWeightsAndClamps)
{
  const auto cancellation_wav = make_wav(
    16000U, 2U, 16U, {32767, -32768, 32767, -32768});
  ConvertedAudio cancellation;
  std::string error;
  ASSERT_TRUE(convert_pcm_wav_to_robot_audio(cancellation_wav, cancellation, error)) << error;
  ASSERT_EQ(cancellation.pcm_s16le.size(), 4U);
  EXPECT_LE(std::abs(static_cast<int>(static_cast<int16_t>(
      static_cast<uint16_t>(cancellation.pcm_s16le[0]) |
      (static_cast<uint16_t>(cancellation.pcm_s16le[1]) << 8U)))), 1);

  const auto extrema_wav = make_wav(48000U, 1U, 32U, {
    std::numeric_limits<int32_t>::min(), std::numeric_limits<int32_t>::max()});
  ConvertedAudio extrema;
  ASSERT_TRUE(convert_pcm_wav_to_robot_audio(extrema_wav, extrema, error)) << error;
  EXPECT_FALSE(extrema.pcm_s16le.empty());
}

TEST(WavAudioConverterTest, PreservesThreeToneSectionsWhenResampling24k)
{
  constexpr uint32_t rate = 24000U;
  constexpr std::size_t section_frames = 2400U;
  constexpr double pi = 3.14159265358979323846;
  std::vector<int32_t> samples;
  samples.reserve(section_frames * 3U);
  for (const double frequency : {440.0, 880.0, 1320.0}) {
    for (std::size_t index = 0U; index < section_frames; ++index) {
      samples.push_back(static_cast<int32_t>(
        std::round(12000.0 * std::sin(2.0 * pi * frequency * index / rate))));
    }
  }

  const auto wav = make_wav(rate, 1U, 16U, samples, 1U, true);
  ConvertedAudio from_bytes;
  ConvertedAudio from_file_bytes;
  std::string error;
  ASSERT_TRUE(convert_pcm_wav_to_robot_audio(wav, from_bytes, error)) << error;
  ASSERT_TRUE(convert_pcm_wav_to_robot_audio(wav, from_file_bytes, error)) << error;
  EXPECT_EQ(sha256_hex(from_bytes.pcm_s16le), sha256_hex(from_file_bytes.pcm_s16le));

  std::vector<uint8_t> reconstructed;
  constexpr std::size_t chunk_size = 1024U;
  for (std::size_t offset = 0U; offset < from_bytes.pcm_s16le.size(); offset += chunk_size) {
    const std::size_t count = std::min(chunk_size, from_bytes.pcm_s16le.size() - offset);
    reconstructed.insert(
      reconstructed.end(), from_bytes.pcm_s16le.begin() + offset,
      from_bytes.pcm_s16le.begin() + offset + count);
  }
  EXPECT_EQ(sha256_hex(reconstructed), sha256_hex(from_bytes.pcm_s16le));

  const std::size_t frames = from_bytes.pcm_s16le.size() / 2U;
  ASSERT_GE(frames, 3U);
  EXPECT_GT(segment_rms(from_bytes.pcm_s16le, 0U, frames / 3U), 0.0);
  EXPECT_GT(segment_rms(from_bytes.pcm_s16le, frames / 3U, 2U * frames / 3U), 0.0);
  EXPECT_GT(segment_rms(from_bytes.pcm_s16le, 2U * frames / 3U, frames), 0.0);
}

TEST(WavAudioConverterTest, RejectsMalformedUnsupportedAndOversizedInputs)
{
  ConvertedAudio converted;
  std::string error;
  EXPECT_FALSE(convert_pcm_wav_to_robot_audio({}, converted, error));
  EXPECT_FALSE(convert_pcm_wav_to_robot_audio(
      {'R', 'I', 'F', 'F', 4, 0, 0, 0, 'N', 'O', 'P', 'E'}, converted, error));

  auto compressed = make_wav(16000U, 1U, 16U, {0}, 3U);
  EXPECT_FALSE(convert_pcm_wav_to_robot_audio(compressed, converted, error));
  auto unsupported_channels = make_wav(16000U, 3U, 16U, {0, 0, 0});
  EXPECT_FALSE(convert_pcm_wav_to_robot_audio(unsupported_channels, converted, error));
  auto zero_rate = make_wav(1U, 1U, 16U, {0});
  overwrite_u32(zero_rate, 24U, 0U);
  overwrite_u32(zero_rate, 28U, 0U);
  EXPECT_FALSE(convert_pcm_wav_to_robot_audio(zero_rate, converted, error));
  auto empty = make_wav(16000U, 1U, 16U, {});
  EXPECT_FALSE(convert_pcm_wav_to_robot_audio(empty, converted, error));

  auto truncated_fmt = make_wav(16000U, 1U, 16U, {0});
  truncated_fmt.resize(24U);
  EXPECT_FALSE(convert_pcm_wav_to_robot_audio(truncated_fmt, converted, error));
  auto truncated_data = make_wav(16000U, 1U, 16U, {0, 1});
  truncated_data.pop_back();
  EXPECT_FALSE(convert_pcm_wav_to_robot_audio(truncated_data, converted, error));

  const auto expands = make_wav(8000U, 1U, 8U, {128, 129, 127, 128});
  EXPECT_FALSE(convert_pcm_wav_to_robot_audio(expands, converted, error, 2U));
}

}  // namespace
}  // namespace erasers_g1_common
