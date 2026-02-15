#include "runtime/runtime/failsafe_diagnostics.hpp"

namespace runtime {

bool update_link_down_hysteresis(bool raw_link_down, uint64_t now_ns, uint64_t down_hold_ns,
                                 uint64_t up_hold_ns, LinkDownHysteresisState& state) {
  if (raw_link_down) {
    state.up_since_ns = 0;
    if (!state.active) {
      if (state.down_since_ns == 0) {
        state.down_since_ns = now_ns;
      }
      const uint64_t elapsed_ns = (now_ns >= state.down_since_ns) ? (now_ns - state.down_since_ns) : 0;
      if (elapsed_ns >= down_hold_ns) {
        state.active = true;
      }
    }
  } else {
    state.down_since_ns = 0;
    if (state.active) {
      if (state.up_since_ns == 0) {
        state.up_since_ns = now_ns;
      }
      const uint64_t elapsed_ns = (now_ns >= state.up_since_ns) ? (now_ns - state.up_since_ns) : 0;
      if (elapsed_ns >= up_hold_ns) {
        state.active = false;
      }
    } else {
      state.up_since_ns = 0;
    }
  }
  return state.active;
}

ActuatorFailsafeCause select_actuator_failsafe_cause(bool failsafe_latched, bool kill_active, bool imu_stale,
                                                     bool link_down, bool cmd_timed_out) {
  if (failsafe_latched) {
    return ActuatorFailsafeCause::ActuatorIoLatch;
  }
  if (kill_active) {
    return ActuatorFailsafeCause::KillSwitch;
  }
  if (imu_stale) {
    return ActuatorFailsafeCause::ImuStale;
  }
  if (link_down) {
    return ActuatorFailsafeCause::LinkDown;
  }
  if (cmd_timed_out) {
    return ActuatorFailsafeCause::CommandStale;
  }
  return ActuatorFailsafeCause::None;
}

FailsafeReason to_failsafe_reason(ActuatorFailsafeCause cause) {
  switch (cause) {
    case ActuatorFailsafeCause::None:
      return FailsafeReason::None;
    case ActuatorFailsafeCause::CommandStale:
      return FailsafeReason::CommandTimeout;
    case ActuatorFailsafeCause::LinkDown:
      return FailsafeReason::LinkDown;
    case ActuatorFailsafeCause::ImuStale:
      return FailsafeReason::ImuStale;
    case ActuatorFailsafeCause::KillSwitch:
      return FailsafeReason::KillSwitch;
    case ActuatorFailsafeCause::ActuatorIoLatch:
      return FailsafeReason::IoErrorLatch;
  }
  return FailsafeReason::None;
}

void account_failsafe_transition(bool prev_failsafe_active, bool failsafe_active, ActuatorFailsafeCause cause,
                                 RuntimeStats& stats) {
  if (failsafe_active) {
    ++stats.failsafe_activation_count;
    stats.last_failsafe_reason = static_cast<uint32_t>(to_failsafe_reason(cause));
    if (!prev_failsafe_active) {
      ++stats.failsafe_enter_count;
      switch (cause) {
        case ActuatorFailsafeCause::CommandStale:
          ++stats.failsafe_cause_cmd_stale_count;
          break;
        case ActuatorFailsafeCause::LinkDown:
          ++stats.failsafe_cause_link_down_count;
          break;
        case ActuatorFailsafeCause::ImuStale:
          ++stats.failsafe_cause_imu_stale_count;
          break;
        case ActuatorFailsafeCause::KillSwitch:
          ++stats.failsafe_cause_killswitch_count;
          break;
        case ActuatorFailsafeCause::ActuatorIoLatch:
          ++stats.failsafe_cause_actuator_io_latch_count;
          break;
        case ActuatorFailsafeCause::None:
          break;
      }
    }
    return;
  }

  if (prev_failsafe_active) {
    ++stats.failsafe_exit_count;
  }
}

}  // namespace runtime
