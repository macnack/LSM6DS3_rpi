#pragma once

#include "runtime/common/constants.hpp"
#include "runtime/common/types.hpp"
#include "runtime/config/config.hpp"

#include <array>
#include <cstdint>
#include <memory>

namespace runtime {

class ActuatorBackend {
 public:
  virtual ~ActuatorBackend() = default;
  virtual void start(const ActuatorSection& cfg, const std::array<ServoSection, kServoCount>& servos) = 0;
  virtual void stop() noexcept = 0;
  virtual void write_us(const std::array<uint32_t, kServoCount>& pulses_us) = 0;
};

class HardwareActuatorBackend final : public ActuatorBackend {
 public:
  HardwareActuatorBackend();
  ~HardwareActuatorBackend() override;

  void start(const ActuatorSection& cfg, const std::array<ServoSection, kServoCount>& servos) override;
  void stop() noexcept override;
  void write_us(const std::array<uint32_t, kServoCount>& pulses_us) override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

class SimActuatorBackend final : public ActuatorBackend {
 public:
  void start(const ActuatorSection& cfg, const std::array<ServoSection, kServoCount>& servos) override;
  void stop() noexcept override;
  void write_us(const std::array<uint32_t, kServoCount>& pulses_us) override;

  [[nodiscard]] const std::array<uint32_t, kServoCount>& last_pulses() const noexcept;

 private:
  std::array<uint32_t, kServoCount> last_pulses_{1500, 1500, 1500, 1500};
};

class ActuatorMapper {
 public:
  ActuatorMapper(const ActuatorSection& actuator_cfg,
                 const std::array<ServoSection, kServoCount>& servo_cfg);

  std::array<uint32_t, kServoCount> map(const ControlCommand& cmd, uint64_t now_ns,
                                        bool failsafe_override);

  [[nodiscard]] std::array<uint32_t, kServoCount> failsafe_pulses() const;

 private:
  static double clamp(double value, double lo, double hi);
  [[nodiscard]] double neutral_norm_for(const ServoSection& s) const;
  [[nodiscard]] uint32_t norm_to_pwm_us(double norm, const ServoSection& s) const;

  ActuatorSection actuator_cfg_;
  std::array<ServoSection, kServoCount> servo_cfg_;
  std::array<double, kServoCount> last_norm_{0.0, 0.0, 0.0, 0.0};
  uint64_t last_t_ns_ = 0;
};

}  // namespace runtime
