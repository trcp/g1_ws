#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <sstream>
#include <string>
#include "erasers_g1_common/g1_joint_descriptor.hpp"
#include "erasers_g1_common/g1_arm_sdk_protocol.hpp"
#include "erasers_g1_common/upper_body_control_session.hpp"
#include "erasers_g1_common/upper_body_arbiter.hpp"

using namespace erasers_g1_common;

namespace
{
std::string model(bool full, const std::string & omitted = "")
{
  std::ostringstream xml;
  xml << "<robot name='test_g1'><link name='root'/>";
  for (const auto & joint : g1JointDescriptors()) {
    const auto i = joint.motor_index;
    if (omitted == joint.name || (!full &&
      (i == 13 || i == 14 || i == 20 || i == 21 || i == 27 || i == 28))) {continue;}
    xml << "<link name='link" << i << "'/><joint name='" << joint.name
        << "' type='revolute'><parent link='root'/><child link='link" << i
        << "'/><axis xyz='0 0 1'/><limit lower='-2' upper='2' velocity='1' effort='10'/></joint>";
  }
  xml << "</robot>";
  return xml.str();
}
std::vector<G1JointControlLimit> upper(bool full)
{
  std::vector<G1JointControlLimit> result;
  std::string error;
  if (!loadAndValidateG1UpperBodyLimits(model(full), result, error)) {
    throw std::runtime_error(error);
  }
  return result;
}
}

TEST(G1Description, KeepsCanonical29FieldsFor23And29Hardware)
{
  for (const bool full : {false, true}) {
    std::vector<G1JointControlLimit> limits;
    std::string error;
    ASSERT_TRUE(loadG1JointLimits(model(full), limits, error)) << error;
    ASSERT_EQ(limits.size(), 29U);
    EXPECT_EQ(std::count_if(limits.begin(), limits.end(),
      [](const auto & joint) {return joint.active;}), full ? 29 : 23);
    EXPECT_EQ(limits[12].name, "waist_yaw_joint");
    EXPECT_EQ(limits[22].name, "right_shoulder_pitch_joint");
    EXPECT_EQ(upper(full).size(), 17U);
  }
}

TEST(G1Description, RejectsMissingRequiredJointAndInvalidLimits)
{
  std::vector<G1JointControlLimit> limits;
  std::string error;
  EXPECT_FALSE(loadG1JointLimits(model(false, "waist_yaw_joint"), limits, error));
  auto xml = model(false);
  xml.replace(xml.find("velocity='1'"), 12, "velocity='0'");
  EXPECT_FALSE(loadG1JointLimits(xml, limits, error));
}

TEST(G1Protocol, OnlyCommandsPresentUpperJointsAndUsesWeightSlot29)
{
  for (const bool full : {false, true}) {
    const auto dynamic = upper(full);
    std::array<G1JointControlLimit, kG1UpperBodyJointCount> limits;
    std::copy(dynamic.begin(), dynamic.end(), limits.begin());
    std::array<double, kG1UpperBodyJointCount> q{};
    q.fill(0.1);
    unitree_hg::msg::LowCmd command;
    std::string error;
    ASSERT_TRUE(makeG1ArmSdkLowCmd(q, 0.739, limits, command, error));
    ASSERT_EQ(command.motor_cmd.size(), 35U);
    EXPECT_EQ(command.mode_pr, 0U);
    EXPECT_FLOAT_EQ(command.motor_cmd[29].q, 0.739F);
    for (std::size_t i = 0; i < 35; ++i) {
      if (i >= 12 && i < 29 && limits[i - 12].active) {
        EXPECT_FLOAT_EQ(command.motor_cmd[i].q, 0.1F);
        EXPECT_GT(command.motor_cmd[i].kp, 0.0F);
      } else if (i != 29) {
        EXPECT_FLOAT_EQ(command.motor_cmd[i].q, 0.0F);
        EXPECT_FLOAT_EQ(command.motor_cmd[i].kp, 0.0F);
      }
    }
    EXPECT_FALSE(makeG1ArmSdkLowCmd(q, -0.01, limits, command, error));
    q[0] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(makeG1ArmSdkLowCmd(q, 1.0, limits, command, error));
  }
}

TEST(G1Session, RejectsInactiveTargetsAndReleasesExactlyOnce)
{
  UpperBodyControlSession session;
  std::string error;
  ASSERT_TRUE(session.configure(upper(false), error));
  const auto start = UpperBodyControlSession::SteadyTime{} + std::chrono::seconds(10);
  std::array<double, kG1UpperBodyJointCount> q{};
  session.arm(1, start);
  ASSERT_TRUE(session.beginAcquisition(q, 1, start, error));
  EXPECT_FALSE(session.updateTarget({1}, {0.1}, start, error));
  ASSERT_TRUE(session.updateTarget({0, 3}, {0.1, 0.2}, start, error));
  UpperBodyStepResult step;
  for (int i = 0; i <= 100; ++i) {
    step = session.step(true, false, false, start + std::chrono::milliseconds(i * 10));
    ASSERT_FALSE(step.fault) << step.fault_reason;
  }
  EXPECT_TRUE(step.became_active);
  EXPECT_FLOAT_EQ(step.command.motor_cmd[29].q, 1.0F);
  const auto release = start + std::chrono::seconds(1);
  session.requestRelease(2, release);
  int zeros = 0;
  for (int i = 0; i <= 110; ++i) {
    step = session.step(false, false, true, release + std::chrono::milliseconds(i * 10));
    ASSERT_FALSE(step.fault) << step.fault_reason;
    if (step.publish && step.command.motor_cmd[29].q == 0.0F) {++zeros;}
  }
  EXPECT_EQ(zeros, 1);
}

TEST(G1Session, EnforcesJointVelocityAccelerationAndLoopDeadline)
{
  UpperBodyPolicy policy;
  policy.acquire_duration = std::chrono::milliseconds(10);
  UpperBodyControlSession session(policy);
  std::string error;
  ASSERT_TRUE(session.configure(upper(false), error));
  const auto start = UpperBodyControlSession::SteadyTime{} + std::chrono::seconds(10);
  std::array<double, kG1UpperBodyJointCount> q{};
  ASSERT_TRUE(session.beginAcquisition(q, 1, start, error));
  ASSERT_TRUE(session.updateTarget({3}, {1.0}, start, error));
  ASSERT_TRUE(session.step(true, false, false, start + std::chrono::milliseconds(10)).became_active);
  double last_q = 0.0, last_v = 0.0;
  for (int i = 2; i <= 200; ++i) {
    if (i == 100) {ASSERT_TRUE(session.updateTarget({3}, {-1.0}, start, error));}
    ASSERT_FALSE(session.step(false, true, false, start + std::chrono::milliseconds(i * 10)).fault);
    const double q_now = session.commandedPosition()[3];
    const double v = (q_now - last_q) / 0.01;
    EXPECT_LE(std::abs(v), policy.arm_velocity_limit + 1e-8);
    EXPECT_LE(std::abs(v - last_v), policy.arm_acceleration_limit * 0.01 + 1e-8);
    last_q = q_now;
    last_v = v;
  }
  EXPECT_TRUE(session.step(false, true, false, start + std::chrono::seconds(3)).fault);
}

TEST(G1Ownership, ExcludesArmActionsAndRequiresExplicitFaultRecovery)
{
  UpperBodyArbiter arbiter;
  arbiter.observeRemoteNormal();
  std::string error;
  ASSERT_TRUE(arbiter.reserveJointState(error));
  EXPECT_FALSE(arbiter.reserveArmAction(false, error));
  ASSERT_TRUE(arbiter.beginAcquiring());
  EXPECT_FALSE(arbiter.reserveArmAction(true, error));
  ASSERT_TRUE(arbiter.markActive());
  ASSERT_TRUE(arbiter.beginRelease());
  EXPECT_FALSE(arbiter.reserveArmAction(false, error));
  arbiter.fault("test");
  EXPECT_FALSE(arbiter.prepareFaultRecovery(error));
  arbiter.acknowledgeFaultDisable();
  EXPECT_TRUE(arbiter.prepareFaultRecovery(error));
}
