// Copyright (c) 2026 Nakatogawa Laboratory. All rights reserved.

#include "erasers_g1_common/fan_noise_reducer.hpp"

#include <opencv2/core.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace erasers_g1_common
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

std::vector<double> make_hann_window(int size)
{
  std::vector<double> window(static_cast<std::size_t>(size));
  for (int index = 0; index < size; ++index) {
    window[static_cast<std::size_t>(index)] =
      0.5 - 0.5 * std::cos((2.0 * kPi * index) / (size - 1));
  }
  return window;
}

void validate_profile(const FanNoiseProfile & profile)
{
  if (profile.sample_rate != 16000 || profile.fft_size != 512 || profile.hop_length != 256) {
    throw std::invalid_argument("fan noise profile must use 16kHz, FFT 512, hop 256");
  }
  if (!std::isfinite(profile.alpha) || profile.alpha <= 0.0 ||
    !std::isfinite(profile.min_gain) || profile.min_gain <= 0.0 || profile.min_gain > 1.0)
  {
    throw std::invalid_argument("fan noise profile has invalid filter coefficients");
  }
  if (profile.psd.size() != static_cast<std::size_t>(profile.fft_size / 2 + 1)) {
    throw std::invalid_argument("fan noise PSD must contain 257 bins");
  }
  for (const double value : profile.psd) {
    if (!std::isfinite(value) || value < 0.0) {
      throw std::invalid_argument("fan noise PSD contains an invalid value");
    }
  }
}

cv::Mat dft_frame(const std::vector<double> & frame, const std::vector<double> & window)
{
  cv::Mat complex(static_cast<int>(frame.size()), 1, CV_64FC2, cv::Scalar(0.0, 0.0));
  for (int index = 0; index < complex.rows; ++index) {
    complex.at<cv::Vec2d>(index, 0)[0] =
      frame[static_cast<std::size_t>(index)] * window[static_cast<std::size_t>(index)];
  }
  cv::dft(complex, complex);
  return complex;
}

}  // namespace

FanNoiseReducer::FanNoiseReducer(FanNoiseProfile profile)
: profile_(std::move(profile))
{
  validate_profile(profile_);
  window_ = make_hann_window(profile_.fft_size);
  reset();
}

void FanNoiseReducer::reset()
{
  input_.clear();
  overlap_signal_.assign(static_cast<std::size_t>(profile_.hop_length), 0.0);
  overlap_weight_.assign(static_cast<std::size_t>(profile_.hop_length), 0.0);
}

std::vector<double> FanNoiseReducer::estimate_psd(
  const std::vector<int16_t> & samples, int fft_size, int hop_length)
{
  if (fft_size != 512 || hop_length != 256 || samples.size() < static_cast<std::size_t>(fft_size)) {
    return {};
  }
  const auto window = make_hann_window(fft_size);
  const std::size_t bins = static_cast<std::size_t>(fft_size / 2 + 1);
  std::vector<std::vector<double>> powers(bins);
  for (std::size_t offset = 0; offset + static_cast<std::size_t>(fft_size) <= samples.size();
    offset += static_cast<std::size_t>(hop_length))
  {
    std::vector<double> frame(static_cast<std::size_t>(fft_size));
    for (int index = 0; index < fft_size; ++index) {
      frame[static_cast<std::size_t>(index)] = samples[offset + static_cast<std::size_t>(index)];
    }
    const cv::Mat spectrum = dft_frame(frame, window);
    for (std::size_t bin = 0; bin < bins; ++bin) {
      const cv::Vec2d value = spectrum.at<cv::Vec2d>(static_cast<int>(bin), 0);
      powers[bin].push_back(value[0] * value[0] + value[1] * value[1]);
    }
  }
  std::vector<double> result(bins, 0.0);
  for (std::size_t bin = 0; bin < bins; ++bin) {
    auto & values = powers[bin];
    if (values.empty()) {
      continue;
    }
    const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
    std::nth_element(values.begin(), middle, values.end());
    result[bin] = std::max(*middle, 1e-12);
  }
  return result;
}

std::vector<int16_t> FanNoiseReducer::process(const std::vector<int16_t> & samples)
{
  input_.reserve(input_.size() + samples.size());
  for (const int16_t value : samples) {
    input_.push_back(static_cast<double>(value));
  }

  std::vector<int16_t> output;
  const std::size_t frame_size = static_cast<std::size_t>(profile_.fft_size);
  const std::size_t hop = static_cast<std::size_t>(profile_.hop_length);
  while (input_.size() >= frame_size) {
    std::vector<double> frame(input_.begin(), input_.begin() + static_cast<std::ptrdiff_t>(frame_size));
    cv::Mat spectrum = dft_frame(frame, window_);
    for (std::size_t bin = 0; bin < profile_.psd.size(); ++bin) {
      const cv::Vec2d value = spectrum.at<cv::Vec2d>(static_cast<int>(bin), 0);
      const double power = value[0] * value[0] + value[1] * value[1];
      const double speech_power = std::max(power - profile_.alpha * profile_.psd[bin], 0.0);
      const double gain = std::clamp(
        speech_power / (speech_power + profile_.psd[bin] + 1e-12), profile_.min_gain, 1.0);
      spectrum.at<cv::Vec2d>(static_cast<int>(bin), 0) *= gain;
      if (bin > 0 && bin + 1 < profile_.psd.size()) {
        spectrum.at<cv::Vec2d>(profile_.fft_size - static_cast<int>(bin), 0) *= gain;
      }
    }

    cv::Mat time_domain;
    cv::idft(spectrum, time_domain, cv::DFT_SCALE | cv::DFT_REAL_OUTPUT);
    for (std::size_t index = 0; index < hop; ++index) {
      const double weighted = time_domain.at<double>(static_cast<int>(index), 0) * window_[index];
      const double weight = window_[index] * window_[index];
      const double value = (weighted + overlap_signal_[index]) /
        std::max(weight + overlap_weight_[index], 1e-12);
      output.push_back(static_cast<int16_t>(std::lround(std::clamp(value, -32767.0, 32767.0))));
    }
    for (std::size_t index = 0; index < hop; ++index) {
      const std::size_t source = index + hop;
      overlap_signal_[index] = time_domain.at<double>(static_cast<int>(source), 0) * window_[source];
      overlap_weight_[index] = window_[source] * window_[source];
    }
    input_.erase(input_.begin(), input_.begin() + static_cast<std::ptrdiff_t>(hop));
  }
  return output;
}

}  // namespace erasers_g1_common
