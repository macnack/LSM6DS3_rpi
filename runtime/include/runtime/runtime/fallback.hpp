#pragma once

#include "runtime/common/constants.hpp"
#include "runtime/common/types.hpp"

#include <array>
#include <cstdint>

namespace runtime {

struct FreshnessPolicy {
  uint64_t fresh_timeout_ns = 0;
  uint64_t hold_timeout_ns = 0;
};

enum class EstimatorSelectionSource {
  PythonFresh,
  PythonHold,
  CppFallback,
  Failsafe,
};

enum class ControllerSelectionSource {
  PythonFresh,
  PythonHold,
  Failsafe,
};

struct EstimatorSelectionState {
  bool has_last_valid_python = false;
  EstimatorState last_valid_python{};
};

struct ControllerSelectionState {
  bool has_last_valid_python = false;
  ControlCommand last_valid_python{};
};

struct EstimatorSelectionInput {
  uint64_t now_ns = 0;
  FreshnessPolicy policy{};
  bool has_python_candidate = false;
  EstimatorState python_candidate{};
  bool has_cpp_candidate = false;
  EstimatorState cpp_candidate{};
};

struct EstimatorSelectionResult {
  EstimatorSelectionSource source = EstimatorSelectionSource::Failsafe;
  EstimatorState state{};
};

struct ControllerSelectionInput {
  uint64_t now_ns = 0;
  FreshnessPolicy policy{};
  bool has_python_candidate = false;
  ControlCommand python_candidate{};
  ControlCommand failsafe_command{};
};

struct ControllerSelectionResult {
  ControllerSelectionSource source = ControllerSelectionSource::Failsafe;
  ControlCommand command{};
};

EstimatorSelectionResult select_estimator_state(const EstimatorSelectionInput& input,
                                                EstimatorSelectionState& state);

ControllerSelectionResult select_controller_command(const ControllerSelectionInput& input,
                                                    ControllerSelectionState& state);

ControlCommand sanitize_actuator_command(const ControlCommand& in, double min_norm, double max_norm);

}  // namespace runtime
