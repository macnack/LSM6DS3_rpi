#pragma once

#include <cstdint>
#include <mutex>
#include <stdexcept>
#include <string>

namespace servo {

class PwmError : public std::runtime_error {
 public:
  explicit PwmError(const std::string& message, int error_number = 0);
  [[nodiscard]] int error_number() const noexcept;

 private:
  int error_number_;
};

struct HardwarePwmConfig {
  uint32_t chip = 0;
  uint32_t channel = 0;
  uint64_t period_ns = 20'000'000ULL;         // 50 Hz default
  uint64_t duty_cycle_ns = 1'500'000ULL;      // 1.5 ms neutral
  bool invert_polarity = false;
  bool enabled_on_begin = true;
  bool disable_on_close = true;
  bool unexport_on_close = false;
  bool use_channel_lock = true;
  std::string lock_dir;  // Empty means auto-select: /run/lock, /var/lock, /dev/shm, /tmp
  unsigned int retries = 3;
};

class HardwarePwm {
 public:
  explicit HardwarePwm(HardwarePwmConfig config = {});
  ~HardwarePwm();

  HardwarePwm(const HardwarePwm&) = delete;
  HardwarePwm& operator=(const HardwarePwm&) = delete;
  HardwarePwm(HardwarePwm&&) = delete;
  HardwarePwm& operator=(HardwarePwm&&) = delete;

  void begin();
  void close() noexcept;

  [[nodiscard]] bool is_open() const noexcept;

  void set_enabled(bool enabled);
  void set_period_ns(uint64_t period_ns);
  void set_duty_cycle_ns(uint64_t duty_cycle_ns);

  [[nodiscard]] uint64_t period_ns() const;
  [[nodiscard]] uint64_t duty_cycle_ns() const;
  [[nodiscard]] bool enabled() const;

  [[nodiscard]] uint32_t chip() const noexcept;
  [[nodiscard]] uint32_t channel() const noexcept;
  [[nodiscard]] std::string channel_path() const;

 private:
  class ChannelLock;

  void ensure_open_locked() const;
  void ensure_valid_timing_locked(uint64_t period_ns, uint64_t duty_cycle_ns) const;

  void export_channel_locked();
  void unexport_channel_locked() noexcept;
  void ensure_channel_path_locked();
  void ensure_channel_writable_locked();

  void write_period_locked(uint64_t period_ns);
  void write_duty_locked(uint64_t duty_cycle_ns);
  void write_enable_locked(bool enabled);
  void write_polarity_locked(bool invert);

  [[nodiscard]] std::string chip_path_locked() const;
  [[nodiscard]] std::string channel_path_locked() const;

  HardwarePwmConfig config_;
  bool open_;
  bool exported_by_us_;
  mutable std::mutex mutex_;
};

struct ServoConfig {
  double min_angle_deg = -90.0;
  double max_angle_deg = 90.0;
  uint32_t frequency_hz = 50;
  uint64_t min_pulse_width_us = 1000;
  uint64_t max_pulse_width_us = 2000;
  uint64_t neutral_pulse_width_us = 1500;
  bool invert_polarity = false;
  bool enabled_on_begin = true;
  bool unexport_on_close = false;
  bool use_channel_lock = true;
  unsigned int retries = 3;
};

class Servo {
 public:
  Servo(uint32_t chip, uint32_t channel, ServoConfig config = {});

  void begin();
  void close() noexcept;
  [[nodiscard]] bool is_open() const noexcept;

  void set_angle_deg(double angle_deg);
  void set_pulse_width_us(uint64_t pulse_width_us);
  void set_enabled(bool enabled);

  [[nodiscard]] uint64_t pulse_width_us() const noexcept;
  [[nodiscard]] double angle_deg() const noexcept;
  [[nodiscard]] uint32_t chip() const noexcept;
  [[nodiscard]] uint32_t channel() const noexcept;

 private:
  [[nodiscard]] uint64_t angle_to_pulse_width_us(double angle_deg) const;
  [[nodiscard]] double pulse_width_to_angle_deg(uint64_t pulse_width_us) const;

  ServoConfig config_;
  HardwarePwm pwm_;
  uint64_t pulse_width_us_;
  double angle_deg_;
  mutable std::mutex mutex_;
};

}  // namespace servo
