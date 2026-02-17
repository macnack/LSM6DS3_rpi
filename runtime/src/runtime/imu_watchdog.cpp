#include "runtime/runtime/imu_watchdog.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace runtime {

namespace {

bool eq_eps(double a, double b, double eps) {
  return std::abs(a - b) <= eps;
}

bool is_finite(double x) {
  return std::isfinite(x);
}

}  // namespace

ImuWatchdog::ImuWatchdog(ImuWatchdogSection cfg) : cfg_(std::move(cfg)) {}

ImuWatchdogStepResult ImuWatchdog::on_sample(const ImuSample& sample, uint64_t now_ns) {
  ImuWatchdogStepResult out;
  out.state = state_;

  if (!cfg_.enabled) {
    have_prev_ = true;
    prev_ = sample;
    return out;
  }

  const ImuFaultReason reason = evaluate_fault_reason(sample);
  out.reason = reason;

  if (reason == ImuFaultReason::None) {
    zero_streak_ = 0;
    flatline_streak_ = 0;
    degenerate_streak_ = 0;

    if (state_ == ImuWatchdogState::Faulted || state_ == ImuWatchdogState::Recovering) {
      ++healthy_streak_;
      out.force_invalid = true;
      if (healthy_streak_ >= cfg_.healthy_recovery_samples) {
        state_ = ImuWatchdogState::Healthy;
        healthy_streak_ = 0;
        reinit_attempts_ = 0;
        out.force_invalid = false;
      } else {
        state_ = ImuWatchdogState::Recovering;
      }
    } else {
      state_ = ImuWatchdogState::Healthy;
      healthy_streak_ = 0;
    }
  } else {
    healthy_streak_ = 0;
    if (reason == ImuFaultReason::NanInf) {
      enter_fault(reason, now_ns);
      out.entered_fault = true;
    } else if (reason == ImuFaultReason::ZeroVector) {
      ++zero_streak_;
      ++zero_vector_count_;
      if (zero_streak_ >= cfg_.zero_vector_consecutive) {
        enter_fault(reason, now_ns);
        out.entered_fault = true;
      } else if (state_ == ImuWatchdogState::Healthy) {
        state_ = ImuWatchdogState::Suspect;
      }
    } else if (reason == ImuFaultReason::Flatline) {
      ++flatline_streak_;
      ++flatline_count_;
      if (flatline_streak_ >= cfg_.flatline_consecutive) {
        enter_fault(reason, now_ns);
        out.entered_fault = true;
      } else if (state_ == ImuWatchdogState::Healthy) {
        state_ = ImuWatchdogState::Suspect;
      }
    } else if (reason == ImuFaultReason::DegeneratePattern) {
      ++degenerate_streak_;
      ++degenerate_pattern_count_;
      if (degenerate_streak_ >= cfg_.degenerate_pattern_consecutive) {
        enter_fault(reason, now_ns);
        out.entered_fault = true;
      } else if (state_ == ImuWatchdogState::Healthy) {
        state_ = ImuWatchdogState::Suspect;
      }
    }

    if (state_ == ImuWatchdogState::Faulted || state_ == ImuWatchdogState::Recovering) {
      out.force_invalid = true;
    }
  }

  if ((state_ == ImuWatchdogState::Faulted || state_ == ImuWatchdogState::Recovering) &&
      can_attempt_reinit() && now_ns >= next_reinit_ns_) {
    out.should_attempt_reinit = true;
  }

  out.state = state_;
  have_prev_ = true;
  prev_ = sample;
  return out;
}

void ImuWatchdog::on_reinit_result(bool success, uint64_t now_ns) {
  if (!cfg_.enabled) {
    return;
  }
  ++reinit_attempts_;
  next_reinit_ns_ = now_ns + static_cast<uint64_t>(cfg_.recovery_backoff_ms) * 1'000'000ULL;
  state_ = success ? ImuWatchdogState::Recovering : ImuWatchdogState::Faulted;
}

ImuFaultReason ImuWatchdog::evaluate_fault_reason(const ImuSample& sample) const {
  const std::array<double, 6> values = {
      sample.ax_mps2,
      sample.ay_mps2,
      sample.az_mps2,
      sample.gx_rads,
      sample.gy_rads,
      sample.gz_rads,
  };

  for (double v : values) {
    if (!is_finite(v)) {
      return ImuFaultReason::NanInf;
    }
  }

  if (sample.ax_mps2 == 0.0 && sample.ay_mps2 == 0.0 && sample.az_mps2 == 0.0 && sample.gx_rads == 0.0 &&
      sample.gy_rads == 0.0 && sample.gz_rads == 0.0) {
    return ImuFaultReason::ZeroVector;
  }

  if (is_degenerate_pattern(sample)) {
    return ImuFaultReason::DegeneratePattern;
  }

  if (is_flatline(sample)) {
    return ImuFaultReason::Flatline;
  }

  return ImuFaultReason::None;
}

bool ImuWatchdog::is_flatline(const ImuSample& sample) const {
  if (!have_prev_) {
    return false;
  }
  const double eps = cfg_.flatline_epsilon;
  return eq_eps(sample.ax_mps2, prev_.ax_mps2, eps) && eq_eps(sample.ay_mps2, prev_.ay_mps2, eps) &&
         eq_eps(sample.az_mps2, prev_.az_mps2, eps) && eq_eps(sample.gx_rads, prev_.gx_rads, eps) &&
         eq_eps(sample.gy_rads, prev_.gy_rads, eps) && eq_eps(sample.gz_rads, prev_.gz_rads, eps);
}

bool ImuWatchdog::is_degenerate_pattern(const ImuSample& sample) const {
  const double eps = cfg_.flatline_epsilon;
  const bool accel_equal =
      eq_eps(sample.ax_mps2, sample.ay_mps2, eps) && eq_eps(sample.ay_mps2, sample.az_mps2, eps);
  const bool gyro_equal =
      eq_eps(sample.gx_rads, sample.gy_rads, eps) && eq_eps(sample.gy_rads, sample.gz_rads, eps);
  return accel_equal && gyro_equal;
}

bool ImuWatchdog::can_attempt_reinit() const {
  return cfg_.max_reinit_attempts == 0 || reinit_attempts_ < cfg_.max_reinit_attempts;
}

void ImuWatchdog::enter_fault(ImuFaultReason reason, uint64_t now_ns) {
  state_ = ImuWatchdogState::Faulted;
  last_fault_reason_ = reason;
  next_reinit_ns_ = now_ns;
  ++fault_count_;
  zero_streak_ = 0;
  flatline_streak_ = 0;
  degenerate_streak_ = 0;
  healthy_streak_ = 0;
}

const char* imu_watchdog_state_name(ImuWatchdogState state) noexcept {
  switch (state) {
    case ImuWatchdogState::Healthy:
      return "healthy";
    case ImuWatchdogState::Suspect:
      return "suspect";
    case ImuWatchdogState::Faulted:
      return "faulted";
    case ImuWatchdogState::Recovering:
      return "recovering";
  }
  return "unknown";
}

const char* imu_fault_reason_name(ImuFaultReason reason) noexcept {
  switch (reason) {
    case ImuFaultReason::None:
      return "none";
    case ImuFaultReason::NanInf:
      return "nan_inf";
    case ImuFaultReason::ZeroVector:
      return "zero_vector";
    case ImuFaultReason::Flatline:
      return "flatline";
    case ImuFaultReason::DegeneratePattern:
      return "degenerate_pattern";
  }
  return "unknown";
}

}  // namespace runtime
