#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

struct gpiod_chip;
struct gpiod_line;

namespace buzzer {

class GpioError : public std::runtime_error {
 public:
  explicit GpioError(const std::string& message, int error_number = 0);
  [[nodiscard]] int error_number() const noexcept;

 private:
  int error_number_;
};

struct HardwareBuzzerConfig {
  std::string chip_path = "/dev/gpiochip0";
  uint32_t line = 18;
  bool active_high = true;
  uint32_t max_on_time_ms = 0;
  bool force_off_on_begin = true;
  bool force_off_on_close = true;
  bool simulate = false;
};

class BuzzerOutput {
 public:
  virtual ~BuzzerOutput() = default;
  virtual void begin() = 0;
  virtual void close() noexcept = 0;
  virtual void on() = 0;
  virtual void off() = 0;
  [[nodiscard]] virtual bool is_on() const noexcept = 0;
  [[nodiscard]] virtual bool is_open() const noexcept = 0;
};

class HardwareBuzzer final : public BuzzerOutput {
 public:
  explicit HardwareBuzzer(HardwareBuzzerConfig config = {});
  ~HardwareBuzzer() override;

  HardwareBuzzer(const HardwareBuzzer&) = delete;
  HardwareBuzzer& operator=(const HardwareBuzzer&) = delete;

  void begin() override;
  void close() noexcept override;

  void on() override;
  void off() override;

  [[nodiscard]] bool is_on() const noexcept override;
  [[nodiscard]] bool is_open() const noexcept override;
  [[nodiscard]] int raw_level() const noexcept;

 private:
  void ensure_open() const;
  void apply_state(bool on);
  void apply_state_locked(bool on);
  void enforce_watchdog_locked();
  [[nodiscard]] int desired_raw_level(bool on) const noexcept;

  HardwareBuzzerConfig config_{};
  gpiod_chip* chip_ = nullptr;
  gpiod_line* line_ = nullptr;
  bool open_ = false;
  bool on_ = false;
  int raw_level_ = 0;
  std::chrono::steady_clock::time_point on_since_{};
  mutable std::mutex mutex_;
};

}  // namespace buzzer
