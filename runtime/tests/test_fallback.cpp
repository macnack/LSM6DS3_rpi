#include "runtime/runtime/fallback.hpp"

#include <cmath>
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

bool test_estimator_fresh_then_hold_then_cpp() {
  EstimatorSelectionState state;

  EstimatorState py{};
  py.valid = true;
  py.t_ns = 1'000'000'000ULL;
  py.q_body_to_ned = {1.0F, 0.0F, 0.0F, 0.0F};

  EstimatorState cpp{};
  cpp.valid = true;
  cpp.t_ns = 1'000'000'000ULL;
  cpp.q_body_to_ned = {1.0F, 0.0F, 0.0F, 0.0F};

  EstimatorSelectionInput in{};
  in.policy.fresh_timeout_ns = 20'000'000ULL;
  in.policy.hold_timeout_ns = 100'000'000ULL;
  in.has_cpp_candidate = true;
  in.cpp_candidate = cpp;

  in.now_ns = 1'010'000'000ULL;
  in.has_python_candidate = true;
  in.python_candidate = py;
  auto r1 = select_estimator_state(in, state);
  REQUIRE(r1.source == EstimatorSelectionSource::PythonFresh, "Expected PythonFresh estimator selection");

  in.now_ns = 1'050'000'000ULL;
  in.has_python_candidate = false;
  auto r2 = select_estimator_state(in, state);
  REQUIRE(r2.source == EstimatorSelectionSource::PythonHold, "Expected PythonHold estimator selection");

  in.now_ns = 1'250'000'000ULL;
  in.has_python_candidate = false;
  auto r3 = select_estimator_state(in, state);
  REQUIRE(r3.source == EstimatorSelectionSource::CppFallback, "Expected CppFallback estimator selection");

  return true;
}

bool test_controller_fresh_hold_failsafe_and_sanitize() {
  ControllerSelectionState state;

  ControlCommand py{};
  py.valid = true;
  py.armed = true;
  py.t_ns = 2'000'000'000ULL;
  py.servo_norm = {0.1, -0.2, 0.3, -0.4};

  ControlCommand failsafe{};
  failsafe.valid = true;
  failsafe.armed = true;
  failsafe.servo_norm = {0.0, 0.0, 0.0, 0.0};

  ControllerSelectionInput in{};
  in.policy.fresh_timeout_ns = 20'000'000ULL;
  in.policy.hold_timeout_ns = 100'000'000ULL;
  in.failsafe_command = failsafe;

  in.now_ns = 2'005'000'000ULL;
  in.has_python_candidate = true;
  in.python_candidate = py;
  auto c1 = select_controller_command(in, state);
  REQUIRE(c1.source == ControllerSelectionSource::PythonFresh, "Expected PythonFresh controller selection");

  in.now_ns = 2'060'000'000ULL;
  in.has_python_candidate = false;
  auto c2 = select_controller_command(in, state);
  REQUIRE(c2.source == ControllerSelectionSource::PythonHold, "Expected PythonHold controller selection");

  in.now_ns = 2'250'000'000ULL;
  in.has_python_candidate = false;
  auto c3 = select_controller_command(in, state);
  REQUIRE(c3.source == ControllerSelectionSource::Failsafe, "Expected Failsafe controller selection");

  ControlCommand dirty = c3.command;
  dirty.valid = true;
  dirty.servo_norm = {2.0, -2.0, std::nan(""), 0.25};
  const auto clean = sanitize_actuator_command(dirty, -1.0, 1.0);
  REQUIRE(clean.servo_norm[0] == 1.0, "sanitize_actuator_command should clamp high values");
  REQUIRE(clean.servo_norm[1] == -1.0, "sanitize_actuator_command should clamp low values");
  REQUIRE(clean.servo_norm[2] == 0.0, "sanitize_actuator_command should replace NaN with 0");
  REQUIRE(clean.valid == false, "sanitize_actuator_command should mark output invalid when NaN is seen");

  return true;
}

}  // namespace

int main() {
  if (!test_estimator_fresh_then_hold_then_cpp()) {
    return EXIT_FAILURE;
  }
  if (!test_controller_fresh_hold_failsafe_and_sanitize()) {
    return EXIT_FAILURE;
  }
  std::cout << "runtime_unit_fallback: ok\n";
  return EXIT_SUCCESS;
}
