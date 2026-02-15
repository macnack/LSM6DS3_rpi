#include "runtime/runtime/failsafe_diagnostics.hpp"

#include <cstdlib>
#include <iostream>

namespace {

#define REQUIRE(cond, msg)      \
  do {                          \
    if (!(cond)) {              \
      std::cerr << msg << "\n"; \
      return false;             \
    }                           \
  } while (0)

using namespace runtime;

bool test_failsafe_transition_accounting() {
  RuntimeStats stats{};

  account_failsafe_transition(false, true, ActuatorFailsafeCause::CommandStale, stats);
  REQUIRE(stats.failsafe_activation_count == 1, "First active tick must increment activation counter");
  REQUIRE(stats.failsafe_enter_count == 1, "First active tick must register one enter event");
  REQUIRE(stats.failsafe_cause_cmd_stale_count == 1, "Command-stale cause must increment on enter");
  REQUIRE(stats.last_failsafe_reason == static_cast<uint32_t>(FailsafeReason::CommandTimeout),
          "Last failsafe reason must map to command timeout");

  account_failsafe_transition(true, true, ActuatorFailsafeCause::CommandStale, stats);
  REQUIRE(stats.failsafe_activation_count == 2, "Active ticks must continue incrementing activation counter");
  REQUIRE(stats.failsafe_enter_count == 1, "Continuous active state must not create extra enter events");
  REQUIRE(stats.failsafe_cause_cmd_stale_count == 1, "Cause counter must only increment on enter edge");

  account_failsafe_transition(true, false, ActuatorFailsafeCause::None, stats);
  REQUIRE(stats.failsafe_exit_count == 1, "Transition to inactive must increment exit counter");

  account_failsafe_transition(false, false, ActuatorFailsafeCause::None, stats);
  REQUIRE(stats.failsafe_exit_count == 1, "Inactive steady state must not increment exit counter");
  return true;
}

bool test_primary_cause_priority() {
  REQUIRE(select_actuator_failsafe_cause(true, true, true, true, true) ==
              ActuatorFailsafeCause::ActuatorIoLatch,
          "I/O latch must be highest-priority actuator failsafe cause");
  REQUIRE(select_actuator_failsafe_cause(false, true, true, true, true) ==
              ActuatorFailsafeCause::KillSwitch,
          "Kill switch must outrank IMU, link-down, and command stale causes");
  REQUIRE(select_actuator_failsafe_cause(false, false, true, true, true) ==
              ActuatorFailsafeCause::ImuStale,
          "IMU stale must outrank link-down and command stale causes");
  REQUIRE(select_actuator_failsafe_cause(false, false, false, true, true) ==
              ActuatorFailsafeCause::LinkDown,
          "Link down must outrank command stale when both are present");
  REQUIRE(select_actuator_failsafe_cause(false, false, false, false, true) ==
              ActuatorFailsafeCause::CommandStale,
          "Command stale must be selected when it is the only cause");
  REQUIRE(select_actuator_failsafe_cause(false, false, false, false, false) == ActuatorFailsafeCause::None,
          "No cause should be selected when all triggers are clear");
  return true;
}

bool test_link_down_hysteresis() {
  constexpr uint64_t kDownHoldNs = 200'000'000ULL;
  constexpr uint64_t kUpHoldNs = 500'000'000ULL;
  LinkDownHysteresisState state{};
  const uint64_t t0 = 1'000'000'000ULL;

  REQUIRE(!update_link_down_hysteresis(true, t0, kDownHoldNs, kUpHoldNs, state),
          "Initial down sample should not pass debounce");
  REQUIRE(!update_link_down_hysteresis(true, t0 + 199'000'000ULL, kDownHoldNs, kUpHoldNs, state),
          "Down debounce should remain inactive before 200ms");
  REQUIRE(update_link_down_hysteresis(true, t0 + 200'000'000ULL, kDownHoldNs, kUpHoldNs, state),
          "Down debounce should activate at 200ms");

  REQUIRE(update_link_down_hysteresis(false, t0 + 350'000'000ULL, kDownHoldNs, kUpHoldNs, state),
          "Up debounce should keep link-down active before 500ms");
  REQUIRE(update_link_down_hysteresis(false, t0 + 849'000'000ULL, kDownHoldNs, kUpHoldNs, state),
          "Up debounce should remain active at 499ms");
  REQUIRE(!update_link_down_hysteresis(false, t0 + 850'000'000ULL, kDownHoldNs, kUpHoldNs, state),
          "Up debounce should clear link-down at 500ms");

  REQUIRE(!update_link_down_hysteresis(true, t0 + 900'000'000ULL, kDownHoldNs, kUpHoldNs, state),
          "A fresh down period should re-arm debounce");
  REQUIRE(!update_link_down_hysteresis(false, t0 + 950'000'000ULL, kDownHoldNs, kUpHoldNs, state),
          "A brief reconnect should cancel the pending down timer");
  REQUIRE(!update_link_down_hysteresis(true, t0 + 1'100'000'000ULL, kDownHoldNs, kUpHoldNs, state),
          "Debounce should restart after reconnect");
  REQUIRE(update_link_down_hysteresis(true, t0 + 1'300'000'000ULL, kDownHoldNs, kUpHoldNs, state),
          "Debounce should activate again once down hold is met");
  return true;
}

}  // namespace

int main() {
  if (!test_failsafe_transition_accounting()) {
    return EXIT_FAILURE;
  }
  if (!test_primary_cause_priority()) {
    return EXIT_FAILURE;
  }
  if (!test_link_down_hysteresis()) {
    return EXIT_FAILURE;
  }
  std::cout << "runtime_unit_runtime_failsafe_causes: ok\n";
  return EXIT_SUCCESS;
}
