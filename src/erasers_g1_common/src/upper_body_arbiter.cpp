#include "erasers_g1_common/upper_body_arbiter.hpp"

namespace erasers_g1_common
{

uint64_t UpperBodyArbiter::invalidatePendingCommand()
{
  return ++state_.generation;
}

const UpperBodyArbiterState & UpperBodyArbiter::state() const noexcept
{
  return state_;
}

bool UpperBodyArbiter::jointStateOwnsControl() const noexcept
{
  return state_.owner == UpperBodyOwner::JOINT_STATE &&
         (state_.phase == UpperBodyPhase::ARMED ||
         state_.phase == UpperBodyPhase::ACQUIRING ||
         state_.phase == UpperBodyPhase::ACTIVE ||
         state_.phase == UpperBodyPhase::RELEASING);
}

bool UpperBodyArbiter::reserveJointState(std::string & reason)
{
  if (state_.phase == UpperBodyPhase::FAULT) {
    if (!state_.fault_disable_acknowledged) {
      reason = "FAULT_LATCHED: disable or release acknowledgement required";
      return false;
    }
  } else {
    if (state_.owner == UpperBodyOwner::UNKNOWN) {
      tryAssumeIdleFromUnknown();
    }
    if (state_.owner != UpperBodyOwner::NONE || state_.phase != UpperBodyPhase::READY) {
      reason = "BUSY: upper body ownership is unavailable";
      return false;
    }
  }

  state_.owner = UpperBodyOwner::JOINT_STATE;
  state_.phase = UpperBodyPhase::ARMED;
  state_.enabled_intent = true;
  state_.abort_requested = false;
  state_.fault_disable_acknowledged = false;
  state_.fault_reason.clear();
  ++state_.generation;
  return true;
}

bool UpperBodyArbiter::beginAcquiring()
{
  if (state_.owner != UpperBodyOwner::JOINT_STATE || state_.phase != UpperBodyPhase::ARMED) {
    return false;
  }
  state_.phase = UpperBodyPhase::ACQUIRING;
  return true;
}

bool UpperBodyArbiter::markActive()
{
  if (state_.owner != UpperBodyOwner::JOINT_STATE ||
    state_.phase != UpperBodyPhase::ACQUIRING)
  {
    return false;
  }
  state_.phase = UpperBodyPhase::ACTIVE;
  return true;
}

bool UpperBodyArbiter::beginEmergencyHold(std::string & reason)
{
  if (state_.phase == UpperBodyPhase::FAULT ||
    state_.owner == UpperBodyOwner::UNKNOWN || state_.owner == UpperBodyOwner::ARM_ACTION)
  {
    reason = "EMERGENCY_HOLD_OWNERSHIP_UNAVAILABLE";
    return false;
  }
  state_.owner = UpperBodyOwner::JOINT_STATE;
  state_.phase = UpperBodyPhase::ACTIVE;
  state_.enabled_intent = false;
  state_.abort_requested = false;
  ++state_.generation;
  return true;
}

bool UpperBodyArbiter::beginRelease()
{
  if (!jointStateOwnsControl()) {
    return false;
  }
  state_.enabled_intent = false;
  state_.abort_requested = false;
  state_.phase = UpperBodyPhase::RELEASING;
  ++state_.generation;
  return true;
}

void UpperBodyArbiter::completeRelease()
{
  state_.owner = UpperBodyOwner::UNKNOWN;
  state_.phase = UpperBodyPhase::READY;
  state_.enabled_intent = false;
  state_.abort_requested = false;
}

void UpperBodyArbiter::cancelArmed()
{
  if (state_.owner == UpperBodyOwner::JOINT_STATE && state_.phase == UpperBodyPhase::ARMED) {
    state_.owner = UpperBodyOwner::NONE;
    state_.phase = UpperBodyPhase::READY;
    state_.enabled_intent = false;
    state_.abort_requested = false;
    ++state_.generation;
  }
}

bool UpperBodyArbiter::reserveArmAction(bool is_release, std::string & reason)
{
  if (jointStateOwnsControl()) {
    reason = "LOCAL_REJECT: UPPER_BODY_JOINT_STATE_OWNS_CONTROL";
    return false;
  }
  if (state_.owner == UpperBodyOwner::UNKNOWN) {
    tryAssumeIdleFromUnknown();
  }
  if (state_.phase == UpperBodyPhase::FAULT || state_.owner == UpperBodyOwner::UNKNOWN) {
    reason = "LOCAL_REJECT: UPPER_BODY_OWNERSHIP_UNKNOWN";
    return false;
  }
  if (state_.owner == UpperBodyOwner::ARM_ACTION && !is_release) {
    reason = "BUSY: ARM_ACTION already owns the upper body";
    return false;
  }
  state_.owner = UpperBodyOwner::ARM_ACTION;
  state_.phase = UpperBodyPhase::READY;
  ++state_.generation;
  return true;
}

void UpperBodyArbiter::completeArmAction(bool holding)
{
  if (state_.phase == UpperBodyPhase::FAULT) {
    return;
  }
  state_.owner = holding ? UpperBodyOwner::ARM_ACTION : UpperBodyOwner::NONE;
  state_.phase = UpperBodyPhase::READY;
}

void UpperBodyArbiter::observeRemoteNormal()
{
  if (state_.phase == UpperBodyPhase::READY && !state_.enabled_intent) {
    state_.owner = UpperBodyOwner::NONE;
  }
}

bool UpperBodyArbiter::tryAssumeIdleFromUnknown()
{
  if (state_.phase == UpperBodyPhase::READY &&
    state_.owner == UpperBodyOwner::UNKNOWN &&
    !state_.enabled_intent &&
    !state_.abort_requested)
  {
    state_.owner = UpperBodyOwner::NONE;
    return true;
  }
  return false;
}

bool UpperBodyArbiter::prepareInitialOwnership(std::string & reason)
{
  if (state_.phase == UpperBodyPhase::FAULT) {
    reason = "FAULT_LATCHED: disable or release acknowledgement required";
    return false;
  }
  if (state_.owner == UpperBodyOwner::NONE) {
    return true;
  }
  if (state_.owner == UpperBodyOwner::UNKNOWN && tryAssumeIdleFromUnknown()) {
    return true;
  }
  reason = "BUSY: upper body ownership is unavailable";
  return false;
}

void UpperBodyArbiter::markUnknown(const std::string & reason)
{
  if (state_.phase == UpperBodyPhase::FAULT) {
    return;
  }
  state_.owner = UpperBodyOwner::UNKNOWN;
  state_.phase = UpperBodyPhase::READY;
  state_.enabled_intent = false;
  state_.fault_reason = reason;
  ++state_.generation;
}

void UpperBodyArbiter::fault(const std::string & reason)
{
  state_.abort_requested = true;
  state_.enabled_intent = false;
  state_.owner = UpperBodyOwner::UNKNOWN;
  state_.phase = UpperBodyPhase::FAULT;
  state_.fault_disable_acknowledged = false;
  state_.fault_reason = reason;
  ++state_.generation;
}

void UpperBodyArbiter::acknowledgeFaultDisable()
{
  if (state_.phase == UpperBodyPhase::FAULT) {
    state_.fault_disable_acknowledged = true;
  }
}

bool UpperBodyArbiter::prepareFaultRecovery(std::string & reason)
{
  if (state_.phase != UpperBodyPhase::FAULT) {
    return true;
  }
  if (!state_.fault_disable_acknowledged) {
    reason = "FAULT_LATCHED: disable or release acknowledgement required";
    return false;
  }
  state_.owner = UpperBodyOwner::NONE;
  state_.phase = UpperBodyPhase::READY;
  state_.abort_requested = false;
  return true;
}

const char * UpperBodyArbiter::ownerName(UpperBodyOwner owner) noexcept
{
  switch (owner) {
    case UpperBodyOwner::NONE: return "NONE";
    case UpperBodyOwner::JOINT_STATE: return "JOINT_STATE";
    case UpperBodyOwner::ARM_ACTION: return "ARM_ACTION";
    case UpperBodyOwner::UNKNOWN: return "UNKNOWN";
  }
  return "UNKNOWN";
}

const char * UpperBodyArbiter::phaseName(UpperBodyPhase phase) noexcept
{
  switch (phase) {
    case UpperBodyPhase::READY: return "READY";
    case UpperBodyPhase::ARMED: return "ARMED";
    case UpperBodyPhase::ACQUIRING: return "ACQUIRING";
    case UpperBodyPhase::ACTIVE: return "ACTIVE";
    case UpperBodyPhase::RELEASING: return "RELEASING";
    case UpperBodyPhase::FAULT: return "FAULT";
  }
  return "FAULT";
}

}  // namespace erasers_g1_common
