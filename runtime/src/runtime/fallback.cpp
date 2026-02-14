#include "runtime/runtime/fallback.hpp"

#include <algorithm>
#include <cmath>

namespace runtime {

namespace {

bool is_finite(const std::array<float, 4>& a) {
  for (float v : a) {
    if (!std::isfinite(v)) {
      return false;
    }
  }
  return true;
}

bool is_finite(const std::array<float, 3>& a) {
  for (float v : a) {
    if (!std::isfinite(v)) {
      return false;
    }
  }
  return true;
}

bool is_finite(const std::array<double, kServoCount>& a) {
  for (double v : a) {
    if (!std::isfinite(v)) {
      return false;
    }
  }
  return true;
}

bool is_fresh(uint64_t now_ns, uint64_t t_ns, uint64_t timeout_ns) {
  if (t_ns == 0 || now_ns < t_ns) {
    return false;
  }
  return (now_ns - t_ns) <= timeout_ns;
}

bool is_holdable(uint64_t now_ns, uint64_t t_ns, uint64_t hold_ns) {
  if (t_ns == 0 || now_ns < t_ns) {
    return false;
  }
  return (now_ns - t_ns) <= hold_ns;
}

bool is_estimator_valid(const EstimatorState& s) {
  if (!s.valid) {
    return false;
  }
  if (!is_finite(s.q_body_to_ned) || !is_finite(s.vel_ned_mps) || !is_finite(s.pos_ned_m)) {
    return false;
  }

  const double q_norm_sq = static_cast<double>(s.q_body_to_ned[0]) * s.q_body_to_ned[0] +
                           static_cast<double>(s.q_body_to_ned[1]) * s.q_body_to_ned[1] +
                           static_cast<double>(s.q_body_to_ned[2]) * s.q_body_to_ned[2] +
                           static_cast<double>(s.q_body_to_ned[3]) * s.q_body_to_ned[3];
  const double q_norm = std::sqrt(q_norm_sq);
  return q_norm >= 0.85 && q_norm <= 1.15;
}

bool is_command_valid(const ControlCommand& c) {
  if (!c.valid || !is_finite(c.servo_norm)) {
    return false;
  }
  for (double x : c.servo_norm) {
    if (x < -1.0 || x > 1.0) {
      return false;
    }
  }
  return true;
}

}  // namespace

EstimatorSelectionResult select_estimator_state(const EstimatorSelectionInput& input,
                                                EstimatorSelectionState& state) {
  EstimatorSelectionResult result;

  if (input.has_python_candidate && is_estimator_valid(input.python_candidate) &&
      is_fresh(input.now_ns, input.python_candidate.t_ns, input.policy.fresh_timeout_ns)) {
    state.has_last_valid_python = true;
    state.last_valid_python = input.python_candidate;
    result.source = EstimatorSelectionSource::PythonFresh;
    result.state = input.python_candidate;
    return result;
  }

  if (state.has_last_valid_python &&
      is_holdable(input.now_ns, state.last_valid_python.t_ns, input.policy.hold_timeout_ns)) {
    result.source = EstimatorSelectionSource::PythonHold;
    result.state = state.last_valid_python;
    return result;
  }

  if (input.has_cpp_candidate && is_estimator_valid(input.cpp_candidate)) {
    result.source = EstimatorSelectionSource::CppFallback;
    result.state = input.cpp_candidate;
    return result;
  }

  result.source = EstimatorSelectionSource::Failsafe;
  result.state = EstimatorState{};
  result.state.t_ns = input.now_ns;
  result.state.q_body_to_ned = {1.0F, 0.0F, 0.0F, 0.0F};
  result.state.valid = false;
  return result;
}

ControllerSelectionResult select_controller_command(const ControllerSelectionInput& input,
                                                    ControllerSelectionState& state) {
  ControllerSelectionResult result;

  if (input.has_python_candidate && is_command_valid(input.python_candidate) &&
      is_fresh(input.now_ns, input.python_candidate.t_ns, input.policy.fresh_timeout_ns)) {
    state.has_last_valid_python = true;
    state.last_valid_python = input.python_candidate;
    result.source = ControllerSelectionSource::PythonFresh;
    result.command = input.python_candidate;
    return result;
  }

  if (state.has_last_valid_python &&
      is_holdable(input.now_ns, state.last_valid_python.t_ns, input.policy.hold_timeout_ns)) {
    result.source = ControllerSelectionSource::PythonHold;
    result.command = state.last_valid_python;
    return result;
  }

  result.source = ControllerSelectionSource::Failsafe;
  result.command = input.failsafe_command;
  result.command.t_ns = input.now_ns;
  result.command.valid = true;
  return result;
}

ControlCommand sanitize_actuator_command(const ControlCommand& in, double min_norm, double max_norm) {
  ControlCommand out = in;
  out.valid = in.valid;

  if (min_norm > max_norm) {
    std::swap(min_norm, max_norm);
  }

  for (double& x : out.servo_norm) {
    if (!std::isfinite(x)) {
      x = 0.0;
      out.valid = false;
      continue;
    }
    x = std::clamp(x, min_norm, max_norm);
  }

  return out;
}

}  // namespace runtime
