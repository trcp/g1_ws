// Copyright (c) 2026 Nakatogawa Laboratory. All rights reserved.

#include <gtest/gtest.h>

#include <erasers_g1_common/fan_noise_reducer.hpp>

#include <cmath>
#include <vector>

namespace
{

erasers_g1_common::FanNoiseProfile test_profile()
{
  erasers_g1_common::FanNoiseProfile profile;
  profile.psd.assign(257U, 100.0);
  return profile;
}

TEST(FanNoiseReducerTest, EstimatesExpectedProfileShape)
{
  std::vector<int16_t> samples(16000U, 0);
  for (std::size_t index = 0; index < samples.size(); ++index) {
    samples[index] = static_cast<int16_t>(1000.0 * std::sin(index * 0.1));
  }
  const auto psd = erasers_g1_common::FanNoiseReducer::estimate_psd(samples);
  ASSERT_EQ(psd.size(), 257U);
  for (const double value : psd) {
    EXPECT_TRUE(std::isfinite(value));
    EXPECT_GE(value, 0.0);
  }
}

TEST(FanNoiseReducerTest, BuffersArbitraryInputChunks)
{
  erasers_g1_common::FanNoiseReducer reducer(test_profile());
  std::vector<int16_t> input(1024U, 1000);
  const auto first = reducer.process(
    std::vector<int16_t>(input.begin(), input.begin() + 300));
  EXPECT_TRUE(first.empty());
  const auto second = reducer.process(
    std::vector<int16_t>(input.begin() + 300, input.end()));
  EXPECT_EQ(second.size(), 768U);
  for (const int16_t value : second) {
    EXPECT_LE(value, 32767);
    EXPECT_GE(value, -32767);
  }
}

TEST(FanNoiseReducerTest, RejectsMalformedProfile)
{
  auto profile = test_profile();
  profile.psd.pop_back();
  EXPECT_THROW(erasers_g1_common::FanNoiseReducer reducer(profile), std::invalid_argument);
}

}  // namespace
