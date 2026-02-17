#include "buzzer/hardware_buzzer.hpp"

#include <cerrno>
#include <cstring>
#include <gpiod.h>

namespace buzzer {

GpioError::GpioError(const std::string& message, int error_number)
    : std::runtime_error(message), error_number_(error_number) {}

int GpioError::error_number() const noexcept { return error_number_; }

HardwareBuzzer::HardwareBuzzer(HardwareBuzzerConfig config) : config_(std::move(config)) {}

HardwareBuzzer::~HardwareBuzzer() { close(); }

void HardwareBuzzer::begin() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (open_) {
    return;
  }

  if (!config_.simulate) {
    chip_ = ::gpiod_chip_open(config_.chip_path.c_str());
    if (chip_ == nullptr) {
      const int err = errno;
      throw GpioError("gpiod_chip_open failed for '" + config_.chip_path + "'", err);
    }

    line_ = ::gpiod_chip_get_line(chip_, static_cast<unsigned int>(config_.line));
    if (line_ == nullptr) {
      const int err = errno;
      ::gpiod_chip_close(chip_);
      chip_ = nullptr;
      throw GpioError("gpiod_chip_get_line failed for line " + std::to_string(config_.line), err);
    }

    const int initial_level = desired_raw_level(false);
    if (::gpiod_line_request_output(line_, "buzzer-rpi", initial_level) < 0) {
      const int err = errno;
      line_ = nullptr;
      ::gpiod_chip_close(chip_);
      chip_ = nullptr;
      throw GpioError("gpiod_line_request_output failed", err);
    }
  }

  open_ = true;
  on_ = false;
  raw_level_ = desired_raw_level(false);
  on_since_ = std::chrono::steady_clock::time_point{};

  if (config_.force_off_on_begin) {
    apply_state_locked(false);
  }
}

void HardwareBuzzer::close() noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!open_) {
    return;
  }

  if (config_.force_off_on_close) {
    try {
      apply_state_locked(false);
    } catch (...) {
    }
  }

  if (!config_.simulate) {
    if (line_ != nullptr) {
      ::gpiod_line_release(line_);
      line_ = nullptr;
    }
    if (chip_ != nullptr) {
      ::gpiod_chip_close(chip_);
      chip_ = nullptr;
    }
  }

  open_ = false;
  on_ = false;
  raw_level_ = desired_raw_level(false);
  on_since_ = std::chrono::steady_clock::time_point{};
}

void HardwareBuzzer::ensure_open() const {
  if (!open_) {
    throw GpioError("HardwareBuzzer is not open; call begin() first");
  }
}

void HardwareBuzzer::apply_state(bool on) {
  std::lock_guard<std::mutex> lock(mutex_);
  ensure_open();
  apply_state_locked(on);
}

void HardwareBuzzer::apply_state_locked(bool on) {
  const int raw = desired_raw_level(on);

  if (!config_.simulate) {
    if (line_ == nullptr) {
      throw GpioError("GPIO line is not requested");
    }
    if (::gpiod_line_set_value(line_, raw) < 0) {
      const int err = errno;
      throw GpioError("gpiod_line_set_value failed (errno=" + std::to_string(err) + ": " +
                          std::strerror(err) + ")",
                      err);
    }
  }

  on_ = on;
  raw_level_ = raw;
  if (on_) {
    on_since_ = std::chrono::steady_clock::now();
  } else {
    on_since_ = std::chrono::steady_clock::time_point{};
  }
}

void HardwareBuzzer::enforce_watchdog_locked() {
  if (!open_ || !on_ || config_.max_on_time_ms == 0) {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  const auto elapsed =
      std::chrono::duration_cast<std::chrono::milliseconds>(now - on_since_).count();
  if (elapsed >= static_cast<long long>(config_.max_on_time_ms)) {
    apply_state_locked(false);
  }
}

int HardwareBuzzer::desired_raw_level(bool on) const noexcept {
  const bool raw = config_.active_high ? on : !on;
  return raw ? 1 : 0;
}

void HardwareBuzzer::on() { apply_state(true); }

void HardwareBuzzer::off() { apply_state(false); }

bool HardwareBuzzer::is_on() const noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!open_) {
    return false;
  }
  const_cast<HardwareBuzzer*>(this)->enforce_watchdog_locked();
  return on_;
}

bool HardwareBuzzer::is_open() const noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  return open_;
}

int HardwareBuzzer::raw_level() const noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  return raw_level_;
}

}  // namespace buzzer
