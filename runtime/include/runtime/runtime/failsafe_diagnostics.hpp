#pragma once

#include "runtime/common/types.hpp"

#include <cstdint>

namespace runtime {

enum class ActuatorFailsafeCause : uint8_t {
  None = 0,
  CommandStale = 1,
  LinkDown = 2,
  ImuStale = 3,
  KillSwitch = 4,
  ActuatorIoLatch = 5,
};

struct LinkDownHysteresisState {
  bool active = false;
  uint64_t down_since_ns = 0;
  uint64_t up_since_ns = 0;
};

bool update_link_down_hysteresis(bool raw_link_down, uint64_t now_ns, uint64_t down_hold_ns,
                                 uint64_t up_hold_ns, LinkDownHysteresisState& state);

ActuatorFailsafeCause select_actuator_failsafe_cause(bool failsafe_latched, bool kill_active, bool imu_stale,
                                                     bool link_down, bool cmd_timed_out);

FailsafeReason to_failsafe_reason(ActuatorFailsafeCause cause);

void account_failsafe_transition(bool prev_failsafe_active, bool failsafe_active, ActuatorFailsafeCause cause,
                                 RuntimeStats& stats);

}  // namespace runtime
