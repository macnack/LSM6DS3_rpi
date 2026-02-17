#pragma once

#include "runtime/common/types.hpp"
#include "runtime/config/config.hpp"

#include <cstdint>

namespace runtime {

enum class ImuFaultReason : uint32_t {
  None = 0,
  NanInf = 1,
  ZeroVector = 2,
  Flatline = 3,
  DegeneratePattern = 4,
};

enum class ImuWatchdogState : uint32_t {
  Healthy = 0,
  Suspect = 1,
  Faulted = 2,
  Recovering = 3,
};

struct ImuWatchdogStepResult {
  bool force_invalid = false;
  bool should_attempt_reinit = false;
  bool entered_fault = false;
  ImuFaultReason reason = ImuFaultReason::None;
  ImuWatchdogState state = ImuWatchdogState::Healthy;
};

class ImuWatchdog {
 public:
  explicit ImuWatchdog(ImuWatchdogSection cfg);

  ImuWatchdogStepResult on_sample(const ImuSample& sample, uint64_t now_ns);
  void on_reinit_result(bool success, uint64_t now_ns);

  [[nodiscard]] ImuWatchdogState state() const noexcept { return state_; }
  [[nodiscard]] ImuFaultReason last_fault_reason() const noexcept { return last_fault_reason_; }

  [[nodiscard]] uint64_t fault_count() const noexcept { return fault_count_; }
  [[nodiscard]] uint64_t zero_vector_count() const noexcept { return zero_vector_count_; }
  [[nodiscard]] uint64_t flatline_count() const noexcept { return flatline_count_; }
  [[nodiscard]] uint64_t degenerate_pattern_count() const noexcept { return degenerate_pattern_count_; }

 private:
  [[nodiscard]] ImuFaultReason evaluate_fault_reason(const ImuSample& sample) const;
  [[nodiscard]] bool is_flatline(const ImuSample& sample) const;
  [[nodiscard]] bool is_degenerate_pattern(const ImuSample& sample) const;
  [[nodiscard]] bool can_attempt_reinit() const;

  void enter_fault(ImuFaultReason reason, uint64_t now_ns);

  ImuWatchdogSection cfg_{};
  ImuWatchdogState state_ = ImuWatchdogState::Healthy;
  ImuFaultReason last_fault_reason_ = ImuFaultReason::None;

  bool have_prev_ = false;
  ImuSample prev_{};

  uint32_t zero_streak_ = 0;
  uint32_t flatline_streak_ = 0;
  uint32_t degenerate_streak_ = 0;
  uint32_t healthy_streak_ = 0;

  uint64_t next_reinit_ns_ = 0;
  uint32_t reinit_attempts_ = 0;

  uint64_t fault_count_ = 0;
  uint64_t zero_vector_count_ = 0;
  uint64_t flatline_count_ = 0;
  uint64_t degenerate_pattern_count_ = 0;
};

const char* imu_watchdog_state_name(ImuWatchdogState state) noexcept;
const char* imu_fault_reason_name(ImuFaultReason reason) noexcept;

}  // namespace runtime
