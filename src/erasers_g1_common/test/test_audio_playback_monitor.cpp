// Copyright (c) 2026 Nakatogawa Laboratory. All rights reserved.

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <erasers_g1_common/audio_playback_monitor.hpp>
#include <thread>
#include <chrono>
#include <atomic>

using namespace erasers_g1_common;

class AudioPlaybackMonitorTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("test_audio_playback_node");
    monitor_ = std::make_shared<AudioPlaybackMonitor>(node_.get(), "/audio_msg");
  }

  void TearDown() override
  {
    monitor_.reset();
    node_.reset();
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<AudioPlaybackMonitor> monitor_;
};

// 1. 基本的な UNKNOWN -> PLAYING -> STOPPED 遷移
TEST_F(AudioPlaybackMonitorTest, BasicTransitions)
{
  auto snap1 = monitor_->snapshot();
  EXPECT_FALSE(snap1.state_received);
  EXPECT_EQ(snap1.last_state, PlayState::UNKNOWN);

  monitor_->ingest_payload("{\"play_state\":1}");
  auto snap2 = monitor_->snapshot();
  EXPECT_TRUE(snap2.state_received);
  EXPECT_EQ(snap2.last_state, PlayState::PLAYING);
  EXPECT_EQ(snap2.transition_sequence, 1u);
  EXPECT_EQ(snap2.last_playing_sequence, 1u);

  monitor_->ingest_payload("{\"play_state\":0}");
  auto snap3 = monitor_->snapshot();
  EXPECT_EQ(snap3.last_state, PlayState::STOPPED);
  EXPECT_EQ(snap3.transition_sequence, 2u);
  EXPECT_EQ(snap3.last_stopped_sequence, 2u);
}

// 2. 無効なペイロードや無視されるべき payload
TEST_F(AudioPlaybackMonitorTest, InvalidOrIgnoredPayloads)
{
  // ASRなどの別JSON
  monitor_->ingest_payload("{\"asr_result\":\"hello\"}");
  EXPECT_FALSE(monitor_->snapshot().state_received);

  // 不正なJSONフォーマット
  monitor_->ingest_payload("{\"play_state\":");
  EXPECT_FALSE(monitor_->snapshot().state_received);

  // 整数でない play_state
  monitor_->ingest_payload("{\"play_state\":\"playing\"}");
  EXPECT_FALSE(monitor_->snapshot().state_received);

  // 0/1 以外の play_state
  monitor_->ingest_payload("{\"play_state\":2}");
  EXPECT_FALSE(monitor_->snapshot().state_received);
}

// 3. シーケンス検証
TEST_F(AudioPlaybackMonitorTest, SequenceValidation)
{
  auto baseline = monitor_->snapshot().transition_sequence;

  monitor_->ingest_payload("{\"play_state\":1}");
  EXPECT_GT(monitor_->snapshot().last_playing_sequence, baseline);

  monitor_->ingest_payload("{\"play_state\":0}");
  EXPECT_GT(monitor_->snapshot().last_stopped_sequence, monitor_->snapshot().last_playing_sequence);
}

// 4. 重複ステートによる transition_sequence および transition_time の不変性
TEST_F(AudioPlaybackMonitorTest, DuplicatePayloadsNoTransition)
{
  monitor_->ingest_payload("{\"play_state\":1}");
  auto snap1 = monitor_->snapshot();
  
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  monitor_->ingest_payload("{\"play_state\":1}");
  auto snap2 = monitor_->snapshot();

  EXPECT_EQ(snap1.transition_sequence, snap2.transition_sequence);
  EXPECT_EQ(snap1.last_transition_time, snap2.last_transition_time);

  monitor_->ingest_payload("{\"play_state\":0}");
  auto snap3 = monitor_->snapshot();

  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  monitor_->ingest_payload("{\"play_state\":0}");
  auto snap4 = monitor_->snapshot();

  EXPECT_EQ(snap3.transition_sequence, snap4.transition_sequence);
  EXPECT_EQ(snap3.last_transition_time, snap4.last_transition_time);
}

// 5. 待機キャンセル判定の検証
TEST_F(AudioPlaybackMonitorTest, WaitCancelled)
{
  auto baseline = monitor_->snapshot();
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);

  std::atomic<bool> cancel_flag(false);
  auto cancel_pred = [&cancel_flag]() { return cancel_flag.load(); };

  std::thread t([this, baseline, deadline, cancel_pred]() {
    auto status = monitor_->wait_for_playing_after(baseline.transition_sequence, deadline, cancel_pred);
    EXPECT_EQ(status, WaitStatus::CANCELLED);
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  cancel_flag.store(true);
  t.join();
}

// 6. 単一バーストの stable stopped 待機 (13.1)
TEST_F(AudioPlaybackMonitorTest, SingleBurstStable)
{
  auto baseline = monitor_->snapshot();
  auto quiet = std::chrono::milliseconds(50);
  auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);

  // 1 -> 0 を投入
  monitor_->ingest_payload("{\"play_state\":1}");
  auto playing_seq = monitor_->snapshot().last_playing_sequence;
  monitor_->ingest_payload("{\"play_state\":0}");

  // すぐに wait すると最初は成功しないが、少し待つと成功する
  auto t_start = std::chrono::steady_clock::now();
  auto status = monitor_->wait_for_stable_stopped_after(
    playing_seq,
    deadline,
    quiet,
    []() { return false; }
  );

  auto t_end = std::chrono::steady_clock::now();
  EXPECT_EQ(status, WaitStatus::SUCCESS);
  EXPECT_GE(std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count(), 40);
}

// 7. 実機挙動の複数バーストシミュレーション (13.2)
TEST_F(AudioPlaybackMonitorTest, RealWorldMultipleBursts)
{
  auto quiet = std::chrono::milliseconds(60);
  auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(300);

  monitor_->ingest_payload("{\"play_state\":1}");
  auto playing_seq = monitor_->snapshot().last_playing_sequence;

  // 最初の STOPPED 投入
  monitor_->ingest_payload("{\"play_state\":0}");

  std::atomic<bool> check_done(false);
  WaitStatus status = WaitStatus::TIMEOUT;

  std::thread t([&]() {
    status = monitor_->wait_for_stable_stopped_after(
      playing_seq,
      deadline,
      quiet,
      []() { return false; }
    );
    check_done.store(true);
  });

  // quiet period の半分経過した時点で PLAYING に再開
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  EXPECT_FALSE(check_done.load());
  monitor_->ingest_payload("{\"play_state\":1}"); // これにより Candidate が取り消される

  // さらに待っても最初の thread は完了しないはず (再開したため)
  std::this_thread::sleep_for(std::chrono::milliseconds(40));
  EXPECT_FALSE(check_done.load());

  // 2回目の STOPPED を投入
  monitor_->ingest_payload("{\"play_state\":0}");

  // 2回目の STOPPED 後は quiet period 経過して正常終了する
  t.join();
  EXPECT_TRUE(check_done.load());
  EXPECT_EQ(status, WaitStatus::SUCCESS);
}

// 8. 任意回数のバースト (13.3)
TEST_F(AudioPlaybackMonitorTest, ArbitraryBursts)
{
  auto quiet = std::chrono::milliseconds(40);
  
  monitor_->ingest_payload("{\"play_state\":1}");
  auto playing_seq = monitor_->snapshot().last_playing_sequence;

  for (int i = 0; i < 5; ++i) {
    monitor_->ingest_payload("{\"play_state\":0}");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    monitor_->ingest_payload("{\"play_state\":1}");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // 最後の STOPPED
  monitor_->ingest_payload("{\"play_state\":0}");
  auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(150);
  
  auto status = monitor_->wait_for_stable_stopped_after(
    playing_seq,
    deadline,
    quiet,
    []() { return false; }
  );

  EXPECT_EQ(status, WaitStatus::SUCCESS);
}

// 9. 重複 STOPPED が quiet period をリセットしないこと (13.4)
TEST_F(AudioPlaybackMonitorTest, DuplicateStoppedDoesNotResetQuiet)
{
  auto quiet = std::chrono::milliseconds(50);
  auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);

  monitor_->ingest_payload("{\"play_state\":1}");
  auto playing_seq = monitor_->snapshot().last_playing_sequence;
  
  monitor_->ingest_payload("{\"play_state\":0}"); // 遷移時刻はここ

  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  monitor_->ingest_payload("{\"play_state\":0}"); // 重複値。遷移時刻はリセットされないはず

  // 最初の STOPPED 投入時刻から 50ms 後に即 SUCCESS する。
  // もし重複がリセットしていれば、2回目の STOPPED から 50ms (合計 80ms) 待つことになる。
  auto t_start = std::chrono::steady_clock::now();
  auto status = monitor_->wait_for_stable_stopped_after(
    playing_seq,
    deadline,
    quiet,
    []() { return false; }
  );

  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t_start).count();
  EXPECT_EQ(status, WaitStatus::SUCCESS);
  EXPECT_LT(elapsed, 40); // 既に30ms経過しているので残り20ms。余裕を見て40ms未満とする
}

// 10. リクエスト前の STOPPED 状態を完了判定に使わない (13.5)
TEST_F(AudioPlaybackMonitorTest, IgnoreStoppedBeforeRequest)
{
  monitor_->ingest_payload("{\"play_state\":0}");
  auto baseline = monitor_->snapshot();

  // リクエスト発生し PLAYING (1) -> STOPPED (0)
  monitor_->ingest_payload("{\"play_state\":1}");
  auto playing_seq = monitor_->snapshot().last_playing_sequence;
  monitor_->ingest_payload("{\"play_state\":0}");

  auto quiet = std::chrono::milliseconds(20);
  auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);

  auto status = monitor_->wait_for_stable_stopped_after(
    playing_seq,
    deadline,
    quiet,
    []() { return false; }
  );
  EXPECT_EQ(status, WaitStatus::SUCCESS);
}

// 11. STOPPED のみ（PLAYINGなし）の場合のタイムアウト (13.6)
TEST_F(AudioPlaybackMonitorTest, StoppedOnlyFailure)
{
  auto baseline = monitor_->snapshot();
  auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);

  // PLAYING を観測しないまま STOPPED のみ投入
  monitor_->ingest_payload("{\"play_state\":0}");

  auto status = monitor_->wait_for_playing_after(
    baseline.transition_sequence,
    deadline,
    []() { return false; }
  );
  EXPECT_EQ(status, WaitStatus::TIMEOUT);
}

// 12. PLAYING のみの場合のタイムアウト (13.7)
TEST_F(AudioPlaybackMonitorTest, PlayingOnlyFailure)
{
  monitor_->ingest_payload("{\"play_state\":1}");
  auto playing_seq = monitor_->snapshot().last_playing_sequence;

  auto quiet = std::chrono::milliseconds(50);
  auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);

  auto status = monitor_->wait_for_stable_stopped_after(
    playing_seq,
    deadline,
    quiet,
    []() { return false; }
  );
  EXPECT_EQ(status, WaitStatus::TIMEOUT);
}

// 13. quiet period 中のキャンセル (13.9)
TEST_F(AudioPlaybackMonitorTest, WaitStableCancelled)
{
  monitor_->ingest_payload("{\"play_state\":1}");
  auto playing_seq = monitor_->snapshot().last_playing_sequence;

  auto quiet = std::chrono::milliseconds(200);
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);

  std::atomic<bool> cancel_flag(false);
  auto status = WaitStatus::SUCCESS;

  std::thread t([&]() {
    status = monitor_->wait_for_stable_stopped_after(
      playing_seq,
      deadline,
      quiet,
      [&]() { return cancel_flag.load(); }
    );
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  monitor_->ingest_payload("{\"play_state\":0}");

  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  cancel_flag.store(true);
  t.join();

  EXPECT_EQ(status, WaitStatus::CANCELLED);
}

// 14. quiet period 完了前の deadline 到達 (13.10)
TEST_F(AudioPlaybackMonitorTest, WaitStableDeadline)
{
  monitor_->ingest_payload("{\"play_state\":1}");
  auto playing_seq = monitor_->snapshot().last_playing_sequence;

  auto quiet = std::chrono::milliseconds(200); // 200ms

  auto status = WaitStatus::SUCCESS;
  std::thread t([&]() {
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(50); // 先に deadline に達する
    status = monitor_->wait_for_stable_stopped_after(
      playing_seq,
      deadline,
      quiet,
      []() { return false; }
    );
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  monitor_->ingest_payload("{\"play_state\":0}");

  t.join();

  EXPECT_EQ(status, WaitStatus::TIMEOUT);
}
