#ifndef ERASERS_G1_COMMON__UPPER_BODY_ARBITER_HPP_
#define ERASERS_G1_COMMON__UPPER_BODY_ARBITER_HPP_

#include <cstdint>
#include <string>

namespace erasers_g1_common
{

enum class UpperBodyOwner
{
  NONE,
  JOINT_STATE,
  ARM_ACTION,
  UNKNOWN
};

enum class UpperBodyPhase
{
  READY,
  ARMED,
  ACQUIRING,
  ACTIVE,
  RELEASING,
  FAULT
};

struct UpperBodyArbiterState
{
  UpperBodyOwner owner{UpperBodyOwner::UNKNOWN};
  UpperBodyPhase phase{UpperBodyPhase::READY};
  uint64_t generation{0};
  bool enabled_intent{false};
  bool abort_requested{false};
  bool fault_disable_acknowledged{false};
  std::string fault_reason;
};

class UpperBodyArbiter
{
public:
  const UpperBodyArbiterState & state() const noexcept;

  bool jointStateOwnsControl() const noexcept;
  bool reserveJointState(std::string & reason);
  bool beginAcquiring();
  bool markActive();
  bool beginRelease();
  bool beginEmergencyHold(std::string & reason);
  void completeRelease();
  void cancelArmed();
  uint64_t invalidatePendingCommand();

  bool reserveArmAction(bool is_release, std::string & reason);
  void completeArmAction(bool holding);
  void observeRemoteNormal();
  void markUnknown(const std::string & reason);

  void fault(const std::string & reason);
  void acknowledgeFaultDisable();
  bool prepareFaultRecovery(std::string & reason);

  static const char * ownerName(UpperBodyOwner owner) noexcept;
  static const char * phaseName(UpperBodyPhase phase) noexcept;

private:
  UpperBodyArbiterState state_;
};

}  // namespace erasers_g1_common

#endif  // ERASERS_G1_COMMON__UPPER_BODY_ARBITER_HPP_
