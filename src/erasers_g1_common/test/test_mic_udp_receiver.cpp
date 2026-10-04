// Copyright (c) 2026 Nakatogawa Laboratory. All rights reserved.

#include <gtest/gtest.h>
#include <erasers_g1_common/mic_udp_receiver.hpp>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <vector>

using namespace erasers_g1_common;

namespace
{
void send_udp_packet(int port, const std::vector<uint8_t> & data)
{
  int sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) {
    return;
  }

  // Enable loopback multicast
  int loop = 1;
  setsockopt(sock, IPPROTO_IP, IP_MULTICAST_LOOP, &loop, sizeof(loop));

  struct sockaddr_in dest{};
  dest.sin_family = AF_INET;
  dest.sin_port = htons(port);
  inet_pton(AF_INET, "127.0.0.1", &dest.sin_addr);

  sendto(sock, data.data(), data.size(), 0, (struct sockaddr *)&dest, sizeof(dest));
  close(sock);
}
} // namespace

TEST(MicUdpReceiverTest, ConfigValidation)
{
  std::string err;

  // Invalid multicast IP
  {
    MicUdpReceiver r("lo", "invalid_ip", 5555, 100, 1024);
    EXPECT_FALSE(r.start(err));
  }

  // Port 0
  {
    MicUdpReceiver r("lo", "239.168.123.161", 0, 100, 1024);
    EXPECT_FALSE(r.start(err));
  }

  // Port > 65535
  {
    MicUdpReceiver r("lo", "239.168.123.161", 70000, 100, 1024);
    EXPECT_FALSE(r.start(err));
  }

  // Buffer < 2
  {
    MicUdpReceiver r("lo", "239.168.123.161", 5555, 100, 1);
    EXPECT_FALSE(r.start(err));
  }

  // Buffer > 65536
  {
    MicUdpReceiver r("lo", "239.168.123.161", 5555, 100, 70000);
    EXPECT_FALSE(r.start(err));
  }

  // Timeout <= 0
  {
    MicUdpReceiver r("lo", "239.168.123.161", 5555, 0, 1024);
    EXPECT_FALSE(r.start(err));
  }

  // Invalid interface
  {
    MicUdpReceiver r("invalid_iface_name", "239.168.123.161", 5555, 100, 1024);
    EXPECT_FALSE(r.start(err));
  }
}

TEST(MicUdpReceiverTest, RepeatedLifecycle)
{
  std::string err;
  MicUdpReceiver r("lo", "239.168.123.161", 15554, 100, 1024);

  EXPECT_TRUE(r.start(err));
  EXPECT_TRUE(r.start(err)); // Double start is fine
  r.stop();
  r.stop(); // Double stop is fine
  EXPECT_TRUE(r.start(err));
  r.stop();
}

TEST(MicUdpReceiverTest, PcmDecodeAndSequence)
{
  std::string err;
  int test_port = 15555;
  MicUdpReceiver r("lo", "239.168.123.161", test_port, 100, 1024);

  std::vector<int16_t> received_samples;
  std::mutex mtx;
  std::condition_variable cv;
  bool called = false;

  r.set_data_callback([&](const std::vector<int16_t> & samples) {
    std::lock_guard<std::mutex> lock(mtx);
    received_samples = samples;
    called = true;
    cv.notify_all();
  });

  ASSERT_TRUE(r.start(err));

  // Input PCM byte sequence: 00 00, FF 7F, 00 80, FF FF
  std::vector<uint8_t> test_packet = {0x00, 0x00, 0xFF, 0x7F, 0x00, 0x80, 0xFF, 0xFF};
  
  // Wait a moment for socket to bind
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  send_udp_packet(test_port, test_packet);

  std::unique_lock<std::mutex> lock(mtx);
  cv.wait_for(lock, std::chrono::seconds(1), [&]() { return called; });

  r.stop();

  ASSERT_TRUE(called);
  ASSERT_EQ(received_samples.size(), 4);
  EXPECT_EQ(received_samples[0], 0);
  EXPECT_EQ(received_samples[1], 32767);
  EXPECT_EQ(received_samples[2], -32768);
  EXPECT_EQ(received_samples[3], -1);
  EXPECT_EQ(r.packet_sequence(), 1);
}

TEST(MicUdpReceiverTest, OddBytesAndEmpty)
{
  std::string err;
  int test_port = 15556;
  MicUdpReceiver r("lo", "239.168.123.161", test_port, 100, 1024);

  bool data_called = false;
  bool error_called = false;
  r.set_data_callback([&](const std::vector<int16_t> &) {
    data_called = true;
  });
  r.set_error_callback([&](const std::string &, bool) {
    error_called = true;
  });

  ASSERT_TRUE(r.start(err));
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // Odd bytes (3 bytes)
  send_udp_packet(test_port, {0x01, 0x02, 0x03});
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  EXPECT_FALSE(data_called);
  EXPECT_TRUE(error_called);

  // Reset
  data_called = false;
  error_called = false;

  // Empty packet
  send_udp_packet(test_port, {});
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  EXPECT_FALSE(data_called);
  EXPECT_FALSE(error_called);

  r.stop();
}

TEST(MicUdpReceiverTest, StopCallbackSuppression)
{
  std::string err;
  int test_port = 15557;
  MicUdpReceiver r("lo", "239.168.123.161", test_port, 100, 1024);

  int callback_count = 0;
  r.set_data_callback([&](const std::vector<int16_t> &) {
    callback_count++;
  });

  ASSERT_TRUE(r.start(err));
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // Send packet 1 -> callback count should increase
  send_udp_packet(test_port, {0x01, 0x02});
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  int count_before = callback_count;
  EXPECT_GT(count_before, 0);

  // Stop receiver
  r.stop();

  // Send packet 2 -> callback count should NOT increase
  send_udp_packet(test_port, {0x03, 0x04});
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  EXPECT_EQ(callback_count, count_before);
}

TEST(MicUdpReceiverTest, WaitsForPacketStrictlyAfterBaselineAcrossRestart)
{
  std::string error;
  const int test_port = 15558;
  MicUdpReceiver receiver("lo", "239.168.123.161", test_port, 100, 1024);
  ASSERT_TRUE(receiver.start(error)) << error;
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  send_udp_packet(test_port, {0x01, 0x00});
  ASSERT_TRUE(receiver.wait_for_packet_after(
      0U, std::chrono::steady_clock::now() + std::chrono::seconds(1)));
  receiver.stop();

  const uint64_t baseline = receiver.packet_sequence();
  ASSERT_GT(baseline, 0U);
  ASSERT_TRUE(receiver.start(error)) << error;
  EXPECT_FALSE(receiver.wait_for_packet_after(
      baseline, std::chrono::steady_clock::now() + std::chrono::milliseconds(100)));
  send_udp_packet(test_port, {0x02, 0x00});
  EXPECT_TRUE(receiver.wait_for_packet_after(
      baseline, std::chrono::steady_clock::now() + std::chrono::seconds(1)));
  EXPECT_GT(receiver.packet_sequence(), baseline);
  receiver.stop();
}

// 14. 15.4-15.11 Policy evaluation tests
#include <erasers_g1_common/mic_control_policy.hpp>

TEST(MicControlPolicyTest, ClosePolicyBestEffortStatus100)
{
  // 15.4 Close policy: best_effort + status 100
  CloseDecision dec = evaluate_close_result(
    CloseApiPolicy::BEST_EFFORT,
    true, // local stop success
    true, // api call needed
    false, // api success (status 100)
    "Close Mic API returned status_code=100"
  );
  EXPECT_TRUE(dec.success);
  EXPECT_TRUE(dec.response_message.find("status_code=100") != std::string::npos);
}

TEST(MicControlPolicyTest, ClosePolicyTimeout)
{
  // 15.5 Close policy: timeout
  CloseDecision dec = evaluate_close_result(
    CloseApiPolicy::BEST_EFFORT,
    true, // local stop success
    true, // api call needed
    false, // api success (timeout)
    "Close Mic API timed out"
  );
  EXPECT_TRUE(dec.success);
  EXPECT_TRUE(dec.response_message.find("timed out") != std::string::npos);
}

TEST(MicControlPolicyTest, ClosePolicyRequired)
{
  // 15.6 Close policy: required + status 100
  CloseDecision dec = evaluate_close_result(
    CloseApiPolicy::REQUIRED,
    true, // local stop success
    true, // api call needed
    false, // api success
    "Close Mic API returned status_code=100"
  );
  EXPECT_FALSE(dec.success);
  EXPECT_TRUE(dec.response_message.find("status_code=100") != std::string::npos);
}

TEST(MicControlPolicyTest, LocalStopFailure)
{
  // 15.7 Local stop failure
  CloseDecision dec = evaluate_close_result(
    CloseApiPolicy::BEST_EFFORT,
    false, // local stop failed
    true,
    true,
    ""
  );
  EXPECT_FALSE(dec.success);
  EXPECT_EQ(dec.response_message, "failed to stop local microphone receiver");
}

TEST(MicControlPolicyTest, OpenApiErrorWithPacket)
{
  // 15.8 Open API error + packet
  EnableDecision dec = evaluate_enable_result(
    CloseApiPolicy::BEST_EFFORT,
    false, // api success
    false, // timed out
    true, // received
    100, // status_code 100
    true // packet received
  );
  EXPECT_TRUE(dec.success);
  EXPECT_TRUE(dec.response_message.find("status_code=100") != std::string::npos);
}

TEST(MicControlPolicyTest, OpenApiTimeoutWithPacket)
{
  // 15.9 Open API timeout + packet
  EnableDecision dec = evaluate_enable_result(
    CloseApiPolicy::BEST_EFFORT,
    false, // api success
    true, // timed out
    false, // received
    0,
    true // packet received
  );
  EXPECT_TRUE(dec.success);
  EXPECT_TRUE(dec.response_message.find("not confirmed") != std::string::npos);
}

TEST(MicControlPolicyTest, OpenApiSuccessNoPacket)
{
  // 15.10 Open API success + no packet
  EnableDecision dec = evaluate_enable_result(
    CloseApiPolicy::BEST_EFFORT,
    true, // api success
    false,
    true,
    0,
    false // packet received
  );
  EXPECT_FALSE(dec.success);
  EXPECT_EQ(dec.response_message, "no fresh UDP microphone packet was received");
}

TEST(MicControlPolicyTest, DisabledRemoteApiWithFreshPacketSucceeds)
{
  const EnableDecision decision = evaluate_enable_result(
    CloseApiPolicy::DISABLED, false, false, false, -1, true);
  EXPECT_TRUE(decision.success);
  EXPECT_NE(decision.response_message.find("local UDP"), std::string::npos);
}

TEST(MicControlPolicyTest, RequiredRemoteApiFailureWithFreshPacketFails)
{
  const EnableDecision decision = evaluate_enable_result(
    CloseApiPolicy::REQUIRED, false, false, true, -1, true);
  EXPECT_FALSE(decision.success);
  EXPECT_NE(decision.response_message.find("status_code=-1"), std::string::npos);
}
