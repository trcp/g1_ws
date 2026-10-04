// Copyright (c) 2026 Nakatogawa Laboratory. All rights reserved.

#include "erasers_g1_common/wav_audio_converter.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace erasers_g1_common
{
namespace
{

constexpr uint32_t kOutputSampleRate = 16000U;

bool read_u16_le(
  const std::vector<uint8_t> & bytes, std::size_t offset, uint16_t & value)
{
  if (offset > bytes.size() || bytes.size() - offset < 2U) {
    return false;
  }
  value = static_cast<uint16_t>(bytes[offset]) |
    static_cast<uint16_t>(static_cast<uint16_t>(bytes[offset + 1U]) << 8U);
  return true;
}

bool read_u32_le(
  const std::vector<uint8_t> & bytes, std::size_t offset, uint32_t & value)
{
  if (offset > bytes.size() || bytes.size() - offset < 4U) {
    return false;
  }
  value = static_cast<uint32_t>(bytes[offset]) |
    (static_cast<uint32_t>(bytes[offset + 1U]) << 8U) |
    (static_cast<uint32_t>(bytes[offset + 2U]) << 16U) |
    (static_cast<uint32_t>(bytes[offset + 3U]) << 24U);
  return true;
}

bool chunk_id_equals(
  const std::vector<uint8_t> & bytes, std::size_t offset, const char id[5])
{
  return offset <= bytes.size() && bytes.size() - offset >= 4U &&
         std::memcmp(bytes.data() + offset, id, 4U) == 0;
}

double decode_sample(
  const uint8_t * sample, uint16_t bits_per_sample)
{
  if (bits_per_sample == 8U) {
    return static_cast<double>(static_cast<int32_t>(sample[0]) - 128) / 128.0;
  }
  if (bits_per_sample == 16U) {
    const uint16_t raw = static_cast<uint16_t>(sample[0]) |
      static_cast<uint16_t>(static_cast<uint16_t>(sample[1]) << 8U);
    return static_cast<double>(static_cast<int16_t>(raw)) / 32768.0;
  }
  if (bits_per_sample == 24U) {
    int32_t raw = static_cast<int32_t>(sample[0]) |
      (static_cast<int32_t>(sample[1]) << 8) |
      (static_cast<int32_t>(sample[2]) << 16);
    if ((raw & 0x00800000) != 0) {
      raw |= static_cast<int32_t>(0xFF000000U);
    }
    return static_cast<double>(raw) / 8388608.0;
  }

  const uint32_t raw = static_cast<uint32_t>(sample[0]) |
    (static_cast<uint32_t>(sample[1]) << 8U) |
    (static_cast<uint32_t>(sample[2]) << 16U) |
    (static_cast<uint32_t>(sample[3]) << 24U);
  return static_cast<double>(static_cast<int32_t>(raw)) / 2147483648.0;
}

int16_t normalized_to_s16(double sample)
{
  const double scaled = std::round(sample * 32768.0);
  const double clamped = std::max(-32768.0, std::min(32767.0, scaled));
  return static_cast<int16_t>(clamped);
}

}  // namespace

bool convert_pcm_wav_to_robot_audio(
  const std::vector<uint8_t> & wav_bytes,
  ConvertedAudio & converted,
  std::string & error_message,
  std::size_t max_output_bytes)
{
  converted = ConvertedAudio{};
  error_message.clear();

  if (wav_bytes.size() < 12U) {
    error_message = "WAV data is too short";
    return false;
  }
  if (!chunk_id_equals(wav_bytes, 0U, "RIFF") ||
    !chunk_id_equals(wav_bytes, 8U, "WAVE"))
  {
    error_message = "invalid RIFF/WAVE header";
    return false;
  }

  uint32_t riff_size = 0U;
  if (!read_u32_le(wav_bytes, 4U, riff_size)) {
    error_message = "truncated RIFF size";
    return false;
  }
  const uint64_t riff_end_u64 = 8ULL + static_cast<uint64_t>(riff_size);
  if (riff_end_u64 < 12ULL || riff_end_u64 > wav_bytes.size()) {
    error_message = "truncated RIFF container";
    return false;
  }
  const std::size_t riff_end = static_cast<std::size_t>(riff_end_u64);

  bool found_fmt = false;
  bool found_data = false;
  uint16_t format_tag = 0U;
  uint16_t channels = 0U;
  uint16_t block_align = 0U;
  uint16_t bits_per_sample = 0U;
  uint32_t sample_rate = 0U;
  uint32_t byte_rate = 0U;
  std::size_t data_offset = 0U;
  std::size_t data_size = 0U;

  std::size_t offset = 12U;
  while (offset < riff_end) {
    if (riff_end - offset < 8U) {
      error_message = "truncated WAV chunk header";
      return false;
    }

    uint32_t chunk_size_u32 = 0U;
    if (!read_u32_le(wav_bytes, offset + 4U, chunk_size_u32)) {
      error_message = "truncated WAV chunk size";
      return false;
    }
    const std::size_t chunk_data_offset = offset + 8U;
    const uint64_t chunk_end_u64 = static_cast<uint64_t>(chunk_data_offset) +
      static_cast<uint64_t>(chunk_size_u32);
    if (chunk_end_u64 > riff_end) {
      error_message = "truncated WAV chunk payload";
      return false;
    }
    if (chunk_id_equals(wav_bytes, offset, "fmt ")) {
      if (found_fmt) {
        error_message = "multiple fmt chunks are not supported";
        return false;
      }
      if (chunk_size_u32 < 16U) {
        error_message = "truncated fmt chunk";
        return false;
      }
      if (!read_u16_le(wav_bytes, chunk_data_offset, format_tag) ||
        !read_u16_le(wav_bytes, chunk_data_offset + 2U, channels) ||
        !read_u32_le(wav_bytes, chunk_data_offset + 4U, sample_rate) ||
        !read_u32_le(wav_bytes, chunk_data_offset + 8U, byte_rate) ||
        !read_u16_le(wav_bytes, chunk_data_offset + 12U, block_align) ||
        !read_u16_le(wav_bytes, chunk_data_offset + 14U, bits_per_sample))
      {
        error_message = "truncated fmt fields";
        return false;
      }
      found_fmt = true;
    } else if (chunk_id_equals(wav_bytes, offset, "data")) {
      if (found_data) {
        error_message = "multiple data chunks are not supported";
        return false;
      }
      data_offset = chunk_data_offset;
      data_size = static_cast<std::size_t>(chunk_size_u32);
      found_data = true;
    }

    const uint64_t next_offset_u64 = chunk_end_u64 + (chunk_size_u32 & 1U);
    if (next_offset_u64 > riff_end) {
      error_message = "missing padding byte after odd-sized WAV chunk";
      return false;
    }
    offset = static_cast<std::size_t>(next_offset_u64);
  }

  if (!found_fmt) {
    error_message = "missing fmt chunk";
    return false;
  }
  if (!found_data) {
    error_message = "missing data chunk";
    return false;
  }
  if (format_tag != 1U) {
    error_message = "unsupported compressed WAV format tag " + std::to_string(format_tag);
    return false;
  }
  if (channels == 0U || channels > 2U) {
    error_message = "unsupported channel count " + std::to_string(channels);
    return false;
  }
  if (sample_rate == 0U) {
    error_message = "sample rate must be greater than zero";
    return false;
  }
  if (bits_per_sample != 8U && bits_per_sample != 16U &&
    bits_per_sample != 24U && bits_per_sample != 32U)
  {
    error_message = "unsupported PCM bits per sample " + std::to_string(bits_per_sample);
    return false;
  }

  const uint32_t bytes_per_sample = bits_per_sample / 8U;
  const uint64_t expected_block_align =
    static_cast<uint64_t>(channels) * bytes_per_sample;
  const uint64_t expected_byte_rate =
    static_cast<uint64_t>(sample_rate) * expected_block_align;
  if (expected_block_align > std::numeric_limits<uint16_t>::max() ||
    block_align != expected_block_align)
  {
    error_message = "invalid block_align";
    return false;
  }
  if (expected_byte_rate > std::numeric_limits<uint32_t>::max() ||
    byte_rate != expected_byte_rate)
  {
    error_message = "invalid byte_rate";
    return false;
  }
  if (data_size == 0U) {
    error_message = "WAV contains no audio frames";
    return false;
  }
  if (data_size % block_align != 0U) {
    error_message = "WAV data size is not aligned to complete frames";
    return false;
  }

  const std::size_t source_frames = data_size / block_align;
  const long double output_frames_exact =
    static_cast<long double>(source_frames) * kOutputSampleRate /
    static_cast<long double>(sample_rate);
  if (!std::isfinite(static_cast<double>(output_frames_exact)) ||
    output_frames_exact > static_cast<long double>(std::numeric_limits<std::size_t>::max()))
  {
    error_message = "converted audio frame count overflow";
    return false;
  }
  const std::size_t output_frames = std::max<std::size_t>(
    1U, static_cast<std::size_t>(std::llround(output_frames_exact)));
  if (output_frames > max_output_bytes / 2U) {
    error_message = "converted audio exceeds output size limit";
    return false;
  }

  converted.source_sample_rate = sample_rate;
  converted.source_channels = channels;
  converted.source_bits_per_sample = bits_per_sample;
  converted.source_duration_sec =
    static_cast<double>(source_frames) / static_cast<double>(sample_rate);
  converted.output_duration_sec =
    static_cast<double>(output_frames) / static_cast<double>(kOutputSampleRate);

  if (sample_rate == kOutputSampleRate && channels == 1U && bits_per_sample == 16U) {
    converted.pcm_s16le.assign(
      wav_bytes.begin() + static_cast<std::ptrdiff_t>(data_offset),
      wav_bytes.begin() + static_cast<std::ptrdiff_t>(data_offset + data_size));
    return true;
  }

  std::vector<double> mono_samples(source_frames);
  for (std::size_t frame = 0U; frame < source_frames; ++frame) {
    const std::size_t frame_offset = data_offset + frame * block_align;
    double mixed = 0.0;
    for (uint16_t channel = 0U; channel < channels; ++channel) {
      const std::size_t sample_offset = frame_offset + channel * bytes_per_sample;
      mixed += decode_sample(wav_bytes.data() + sample_offset, bits_per_sample);
    }
    mono_samples[frame] = mixed / static_cast<double>(channels);
  }

  converted.pcm_s16le.resize(output_frames * 2U);
  for (std::size_t output_index = 0U; output_index < output_frames; ++output_index) {
    const long double source_position =
      static_cast<long double>(output_index) * sample_rate / kOutputSampleRate;
    const std::size_t left = std::min(
      source_frames - 1U, static_cast<std::size_t>(source_position));
    const std::size_t right = std::min(source_frames - 1U, left + 1U);
    const double fraction = static_cast<double>(source_position - left);
    const double interpolated =
      mono_samples[left] + (mono_samples[right] - mono_samples[left]) * fraction;
    const uint16_t encoded = static_cast<uint16_t>(normalized_to_s16(interpolated));
    converted.pcm_s16le[output_index * 2U] = static_cast<uint8_t>(encoded & 0xFFU);
    converted.pcm_s16le[output_index * 2U + 1U] =
      static_cast<uint8_t>((encoded >> 8U) & 0xFFU);
  }

  return true;
}

}  // namespace erasers_g1_common
