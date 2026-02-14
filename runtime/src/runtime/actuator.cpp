#include "runtime/runtime/actuator.hpp"

#include "servo/hardware_pwm.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

namespace runtime {

struct HardwareActuatorBackend::Impl {
  std::vector<std::unique_ptr<servo::HardwarePwm>> channels;
};

HardwareActuatorBackend::HardwareActuatorBackend() : impl_(std::make_unique<Impl>()) {}

HardwareActuatorBackend::~HardwareActuatorBackend() { stop(); }

void HardwareActuatorBackend::start(const ActuatorSection& cfg,
                                    const std::array<ServoSection, kServoCount>& servos) {
  impl_->channels.clear();
  impl_->channels.reserve(kServoCount);

  for (std::size_t i = 0; i < kServoCount; ++i) {
    servo::HardwarePwmConfig pwm_cfg;
    pwm_cfg.chip = servos[i].chip;
    pwm_cfg.channel = servos[i].channel;
    pwm_cfg.period_ns = 1'000'000'000ULL / static_cast<uint64_t>(cfg.frequency_hz);
    pwm_cfg.duty_cycle_ns = static_cast<uint64_t>(servos[i].neutral_pwm_us) * 1000ULL;
    pwm_cfg.enabled_on_begin = true;
    pwm_cfg.disable_on_close = true;
    pwm_cfg.unexport_on_close = false;
    pwm_cfg.use_channel_lock = true;

    auto pwm = std::make_unique<servo::HardwarePwm>(pwm_cfg);
    pwm->begin();
    impl_->channels.push_back(std::move(pwm));
  }
}

void HardwareActuatorBackend::stop() noexcept {
  for (auto& ch : impl_->channels) {
    if (ch) {
      ch->close();
    }
  }
  impl_->channels.clear();
}

void HardwareActuatorBackend::write_us(const std::array<uint32_t, kServoCount>& pulses_us) {
  if (impl_->channels.size() != kServoCount) {
    throw std::runtime_error("HardwareActuatorBackend not started");
  }

  for (std::size_t i = 0; i < kServoCount; ++i) {
    impl_->channels[i]->set_duty_cycle_ns(static_cast<uint64_t>(pulses_us[i]) * 1000ULL);
  }
}

void SimActuatorBackend::start(const ActuatorSection&, const std::array<ServoSection, kServoCount>& servos) {
  for (std::size_t i = 0; i < kServoCount; ++i) {
    last_pulses_[i] = servos[i].neutral_pwm_us;
  }
}

void SimActuatorBackend::stop() noexcept {}

void SimActuatorBackend::write_us(const std::array<uint32_t, kServoCount>& pulses_us) { last_pulses_ = pulses_us; }

const std::array<uint32_t, kServoCount>& SimActuatorBackend::last_pulses() const noexcept {
  return last_pulses_;
}

ActuatorMapper::ActuatorMapper(const ActuatorSection& actuator_cfg,
                               const std::array<ServoSection, kServoCount>& servo_cfg)
    : actuator_cfg_(actuator_cfg), servo_cfg_(servo_cfg) {
  for (std::size_t i = 0; i < kServoCount; ++i) {
    last_norm_[i] = neutral_norm_for(servo_cfg_[i]);
  }
}

double ActuatorMapper::clamp(double value, double lo, double hi) {
  if (lo > hi) {
    std::swap(lo, hi);
  }
  return std::max(lo, std::min(hi, value));
}

double ActuatorMapper::neutral_norm_for(const ServoSection& s) const {
  if (s.neutral_pwm_us >= s.max_pwm_us) {
    return 1.0;
  }
  if (s.neutral_pwm_us <= s.min_pwm_us) {
    return -1.0;
  }
  const double upper = static_cast<double>(s.max_pwm_us - s.neutral_pwm_us);
  const double lower = static_cast<double>(s.neutral_pwm_us - s.min_pwm_us);
  if (upper <= 0.0 || lower <= 0.0) {
    return 0.0;
  }
  return 0.0;
}

uint32_t ActuatorMapper::norm_to_pwm_us(double norm, const ServoSection& s) const {
  const double clamped_norm = clamp(norm, s.min_norm, s.max_norm);
  double pulse = static_cast<double>(s.neutral_pwm_us);
  if (clamped_norm >= 0.0) {
    pulse += clamped_norm * static_cast<double>(s.max_pwm_us - s.neutral_pwm_us);
  } else {
    pulse += clamped_norm * static_cast<double>(s.neutral_pwm_us - s.min_pwm_us);
  }

  pulse = clamp(pulse, static_cast<double>(s.min_pwm_us), static_cast<double>(s.max_pwm_us));
  return static_cast<uint32_t>(std::llround(pulse));
}

std::array<uint32_t, kServoCount> ActuatorMapper::map(const ControlCommand& cmd, uint64_t now_ns,
                                                       bool failsafe_override) {
  std::array<uint32_t, kServoCount> out{};

  double dt = 0.0;
  if (last_t_ns_ != 0 && now_ns > last_t_ns_) {
    dt = static_cast<double>(now_ns - last_t_ns_) * 1e-9;
  }
  last_t_ns_ = now_ns;

  for (std::size_t i = 0; i < kServoCount; ++i) {
    const ServoSection& s = servo_cfg_[i];

    double norm = 0.0;
    if (failsafe_override) {
      norm = s.failsafe_norm;
    } else if (!cmd.armed) {
      norm = neutral_norm_for(s);
    } else {
      norm = cmd.servo_norm[i];
    }

    if (!std::isfinite(norm)) {
      norm = 0.0;
    }

    norm = s.direction * (norm + s.trim_norm);
    norm = clamp(norm, s.min_norm, s.max_norm);

    const double slew = actuator_cfg_.slew_limit_norm_per_sec;
    if (dt > 0.0 && slew > 0.0) {
      const double max_delta = slew * dt;
      const double lo = last_norm_[i] - max_delta;
      const double hi = last_norm_[i] + max_delta;
      norm = clamp(norm, lo, hi);
    }

    last_norm_[i] = norm;
    out[i] = norm_to_pwm_us(norm, s);
  }

  return out;
}

std::array<uint32_t, kServoCount> ActuatorMapper::failsafe_pulses() const {
  std::array<uint32_t, kServoCount> out{};
  for (std::size_t i = 0; i < kServoCount; ++i) {
    out[i] = norm_to_pwm_us(servo_cfg_[i].failsafe_norm, servo_cfg_[i]);
  }
  return out;
}

}  // namespace runtime
