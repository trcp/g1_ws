// Copyright (c) 2026 Nakatogawa Laboratory. All rights reserved.

#ifndef ERASERS_G1_COMMON__FAN_NOISE_REDUCER_HPP_
#define ERASERS_G1_COMMON__FAN_NOISE_REDUCER_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

namespace erasers_g1_common
{

struct FanNoiseProfile
{
  int sample_rate{16000};
  int fft_size{512};
  int hop_length{256};
  double alpha{1.5};
  double min_gain{0.20};
  std::vector<double> psd;
};

class FanNoiseReducer
{
public:
  explicit FanNoiseReducer(FanNoiseProfile profile);

  std::vector<int16_t> process(const std::vector<int16_t> & samples);
  void reset();

  static std::vector<double> estimate_psd(
    const std::vector<int16_t> & samples, int fft_size = 512, int hop_length = 256);

private:
  FanNoiseProfile profile_;
  std::vector<double> window_;
  std::vector<double> input_;
  std::vector<double> overlap_signal_;
  std::vector<double> overlap_weight_;
};

}  // namespace erasers_g1_common

#endif  // ERASERS_G1_COMMON__FAN_NOISE_REDUCER_HPP_
