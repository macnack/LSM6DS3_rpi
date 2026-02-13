#include "servo/hardware_pwm.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <filesystem>
#include <sstream>
#include <string>
#include <sys/file.h>
#include <thread>
#include <unistd.h>

namespace servo {

namespace {

constexpr const char* kPwmRoot = "/sys/class/pwm";
constexpr uint64_t kNsPerUs = 1000ULL;

HardwarePwmConfig make_pwm_config(uint32_t chip, uint32_t channel, const ServoConfig& config) {
  if (config.frequency_hz == 0) {
    throw std::invalid_argument("Servo frequency_hz must be > 0");
  }

  HardwarePwmConfig pwm_config;
  pwm_config.chip = chip;
  pwm_config.channel = channel;
  pwm_config.period_ns = 1'000'000'000ULL / static_cast<uint64_t>(config.frequency_hz);
  pwm_config.duty_cycle_ns = config.neutral_pulse_width_us * kNsPerUs;
  pwm_config.invert_polarity = config.invert_polarity;
  pwm_config.enabled_on_begin = config.enabled_on_begin;
  pwm_config.unexport_on_close = config.unexport_on_close;
  pwm_config.use_channel_lock = config.use_channel_lock;
  pwm_config.retries = config.retries;
  return pwm_config;
}

std::string format_errno(const std::string& message, int err) {
  std::ostringstream oss;
  oss << message;
  if (err != 0) {
    oss << " (errno=" << err << ": " << std::strerror(err) << ")";
  }
  return oss.str();
}

void write_text_file_retry(const std::string& path, const std::string& value, unsigned int retries,
                           bool allow_busy) {
  const unsigned int attempts = retries + 1;
  for (unsigned int attempt = 0; attempt < attempts; ++attempt) {
    const int fd = ::open(path.c_str(), O_WRONLY | O_CLOEXEC);
    if (fd < 0) {
      const int err = errno;
      const bool transient = (err == EINTR || err == EAGAIN || err == EBUSY || err == ETIMEDOUT);
      if (transient && attempt + 1 < attempts) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2U * (attempt + 1)));
        continue;
      }
      throw PwmError("Failed to open '" + path + "' for write", err);
    }

    const ssize_t written = ::write(fd, value.data(), value.size());
    const int write_err = (written < 0) ? errno : EIO;
    (void)::close(fd);

    if (written == static_cast<ssize_t>(value.size())) {
      return;
    }

    const bool busy = (write_err == EBUSY || write_err == EINVAL);
    const bool transient = (write_err == EINTR || write_err == EAGAIN || write_err == ETIMEDOUT ||
                            (allow_busy && busy));
    if (transient && attempt + 1 < attempts) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2U * (attempt + 1)));
      continue;
    }

    throw PwmError("Failed to write '" + path + "'", write_err);
  }

  throw PwmError("Unexpected write retry loop termination for '" + path + "'");
}

uint64_t read_u64_file(const std::string& path) {
  const int fd = ::open(path.c_str(), O_RDONLY | O_CLOEXEC);
  if (fd < 0) {
    throw PwmError("Failed to open '" + path + "' for read", errno);
  }

  char buffer[64] = {};
  const ssize_t n = ::read(fd, buffer, sizeof(buffer) - 1);
  const int read_err = errno;
  (void)::close(fd);

  if (n <= 0) {
    throw PwmError("Failed to read from '" + path + "'", read_err);
  }

  std::string text(buffer, static_cast<std::size_t>(n));
  try {
    return std::stoull(text);
  } catch (const std::exception&) {
    throw PwmError("Unexpected non-numeric content in '" + path + "'");
  }
}

}  // namespace

class HardwarePwm::ChannelLock {
 public:
  ChannelLock(uint32_t chip, uint32_t channel, bool enabled) : fd_(-1) {
    if (!enabled) {
      return;
    }

    const std::string path = "/tmp/servo_pwmchip" + std::to_string(chip) + "_pwm" +
                             std::to_string(channel) + ".lock";

    fd_ = ::open(path.c_str(), O_RDWR | O_CREAT | O_CLOEXEC, 0644);
    if (fd_ < 0) {
      throw PwmError("Failed to open PWM lock file '" + path + "'", errno);
    }

    if (::flock(fd_, LOCK_EX | LOCK_NB) < 0) {
      const int err = errno;
      (void)::close(fd_);
      fd_ = -1;
      throw PwmError("PWM channel lock already held for '" + path + "'", err);
    }
  }

  ~ChannelLock() {
    if (fd_ >= 0) {
      (void)::flock(fd_, LOCK_UN);
      (void)::close(fd_);
      fd_ = -1;
    }
  }

  ChannelLock(const ChannelLock&) = delete;
  ChannelLock& operator=(const ChannelLock&) = delete;
  ChannelLock(ChannelLock&&) = delete;
  ChannelLock& operator=(ChannelLock&&) = delete;

 private:
  int fd_;
};

PwmError::PwmError(const std::string& message, int error_number)
    : std::runtime_error(format_errno(message, error_number)), error_number_(error_number) {}

int PwmError::error_number() const noexcept { return error_number_; }

HardwarePwm::HardwarePwm(HardwarePwmConfig config)
    : config_(config), open_(false), exported_by_us_(false) {
  if (config_.period_ns == 0) {
    throw std::invalid_argument("PWM period_ns must be > 0");
  }
  if (config_.duty_cycle_ns > config_.period_ns) {
    throw std::invalid_argument("PWM duty_cycle_ns must be <= period_ns");
  }
}

HardwarePwm::~HardwarePwm() { close(); }

void HardwarePwm::begin() {
  std::scoped_lock lock(mutex_);
  if (open_) {
    return;
  }

  ChannelLock channel_lock(config_.chip, config_.channel, config_.use_channel_lock);

  export_channel_locked();
  ensure_channel_path_locked();

  write_enable_locked(false);
  write_polarity_locked(config_.invert_polarity);
  write_period_locked(config_.period_ns);
  write_duty_locked(config_.duty_cycle_ns);

  if (config_.enabled_on_begin) {
    write_enable_locked(true);
  }

  open_ = true;
}

void HardwarePwm::close() noexcept {
  std::scoped_lock lock(mutex_);
  if (!open_ && !config_.unexport_on_close) {
    return;
  }

  try {
    ChannelLock channel_lock(config_.chip, config_.channel, config_.use_channel_lock);

    if (open_) {
      try {
        write_enable_locked(false);
      } catch (...) {
        // close must not throw
      }
    }

    if (config_.unexport_on_close && exported_by_us_) {
      unexport_channel_locked();
    }
  } catch (...) {
    // close must not throw
  }

  open_ = false;
  exported_by_us_ = false;
}

bool HardwarePwm::is_open() const noexcept {
  std::scoped_lock lock(mutex_);
  return open_;
}

void HardwarePwm::set_enabled(bool enabled) {
  std::scoped_lock lock(mutex_);
  ensure_open_locked();
  ChannelLock channel_lock(config_.chip, config_.channel, config_.use_channel_lock);
  write_enable_locked(enabled);
}

void HardwarePwm::set_period_ns(uint64_t period_ns) {
  std::scoped_lock lock(mutex_);
  ensure_open_locked();
  ensure_valid_timing_locked(period_ns, config_.duty_cycle_ns);

  ChannelLock channel_lock(config_.chip, config_.channel, config_.use_channel_lock);
  const bool was_enabled = (read_u64_file(channel_path_locked() + "/enable") != 0ULL);
  write_enable_locked(false);
  write_period_locked(period_ns);
  write_duty_locked(config_.duty_cycle_ns);
  write_enable_locked(was_enabled);
}

void HardwarePwm::set_duty_cycle_ns(uint64_t duty_cycle_ns) {
  std::scoped_lock lock(mutex_);
  ensure_open_locked();
  ensure_valid_timing_locked(config_.period_ns, duty_cycle_ns);

  ChannelLock channel_lock(config_.chip, config_.channel, config_.use_channel_lock);
  write_duty_locked(duty_cycle_ns);
}

uint64_t HardwarePwm::period_ns() const {
  std::scoped_lock lock(mutex_);
  if (!open_) {
    return config_.period_ns;
  }
  return read_u64_file(channel_path_locked() + "/period");
}

uint64_t HardwarePwm::duty_cycle_ns() const {
  std::scoped_lock lock(mutex_);
  if (!open_) {
    return config_.duty_cycle_ns;
  }
  return read_u64_file(channel_path_locked() + "/duty_cycle");
}

bool HardwarePwm::enabled() const {
  std::scoped_lock lock(mutex_);
  if (!open_) {
    return false;
  }
  return (read_u64_file(channel_path_locked() + "/enable") != 0ULL);
}

uint32_t HardwarePwm::chip() const noexcept { return config_.chip; }

uint32_t HardwarePwm::channel() const noexcept { return config_.channel; }

std::string HardwarePwm::channel_path() const {
  std::scoped_lock lock(mutex_);
  return channel_path_locked();
}

void HardwarePwm::ensure_open_locked() const {
  if (!open_) {
    throw PwmError("PWM channel is not open; call begin() first");
  }
}

void HardwarePwm::ensure_valid_timing_locked(uint64_t period_ns, uint64_t duty_cycle_ns) const {
  if (period_ns == 0) {
    throw std::invalid_argument("PWM period_ns must be > 0");
  }
  if (duty_cycle_ns > period_ns) {
    throw std::invalid_argument("PWM duty_cycle_ns must be <= period_ns");
  }
}

void HardwarePwm::export_channel_locked() {
  const std::string path = chip_path_locked() + "/export";
  try {
    write_text_file_retry(path, std::to_string(config_.channel), config_.retries, true);
    exported_by_us_ = true;
  } catch (const PwmError& ex) {
    if (ex.error_number() == EBUSY || ex.error_number() == EINVAL) {
      exported_by_us_ = false;
      return;
    }
    throw;
  }
}

void HardwarePwm::unexport_channel_locked() noexcept {
  try {
    write_text_file_retry(chip_path_locked() + "/unexport", std::to_string(config_.channel),
                          config_.retries, true);
  } catch (...) {
    // no-throw
  }
}

void HardwarePwm::ensure_channel_path_locked() {
  const std::filesystem::path path(channel_path_locked());
  const unsigned int attempts = config_.retries + 1;

  for (unsigned int attempt = 0; attempt < attempts; ++attempt) {
    if (std::filesystem::exists(path)) {
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5U * (attempt + 1)));
  }

  throw PwmError("PWM channel path did not appear: '" + channel_path_locked() + "'");
}

void HardwarePwm::write_period_locked(uint64_t period_ns) {
  ensure_valid_timing_locked(period_ns, config_.duty_cycle_ns);
  write_text_file_retry(channel_path_locked() + "/period", std::to_string(period_ns),
                        config_.retries, true);
  config_.period_ns = period_ns;
}

void HardwarePwm::write_duty_locked(uint64_t duty_cycle_ns) {
  ensure_valid_timing_locked(config_.period_ns, duty_cycle_ns);
  write_text_file_retry(channel_path_locked() + "/duty_cycle", std::to_string(duty_cycle_ns),
                        config_.retries, true);
  config_.duty_cycle_ns = duty_cycle_ns;
}

void HardwarePwm::write_enable_locked(bool enabled) {
  write_text_file_retry(channel_path_locked() + "/enable", enabled ? "1" : "0", config_.retries,
                        true);
}

void HardwarePwm::write_polarity_locked(bool invert) {
  write_text_file_retry(channel_path_locked() + "/polarity", invert ? "inversed" : "normal",
                        config_.retries, true);
}

std::string HardwarePwm::chip_path_locked() const {
  return std::string(kPwmRoot) + "/pwmchip" + std::to_string(config_.chip);
}

std::string HardwarePwm::channel_path_locked() const {
  return chip_path_locked() + "/pwm" + std::to_string(config_.channel);
}

Servo::Servo(uint32_t chip, uint32_t channel, ServoConfig config)
    : config_(config),
      pwm_(make_pwm_config(chip, channel, config)),
      pulse_width_us_(config.neutral_pulse_width_us),
      angle_deg_(pulse_width_to_angle_deg(config.neutral_pulse_width_us)) {
  if (config_.frequency_hz == 0) {
    throw std::invalid_argument("Servo frequency_hz must be > 0");
  }
  if (config_.max_angle_deg <= config_.min_angle_deg) {
    throw std::invalid_argument("Servo max_angle_deg must be greater than min_angle_deg");
  }
  if (config_.min_pulse_width_us >= config_.max_pulse_width_us) {
    throw std::invalid_argument(
        "Servo max_pulse_width_us must be greater than min_pulse_width_us");
  }
  if (config_.neutral_pulse_width_us < config_.min_pulse_width_us ||
      config_.neutral_pulse_width_us > config_.max_pulse_width_us) {
    throw std::invalid_argument("Servo neutral_pulse_width_us must be inside min/max pulse range");
  }
}

void Servo::begin() {
  std::scoped_lock lock(mutex_);
  pwm_.begin();
  pwm_.set_duty_cycle_ns(pulse_width_us_ * kNsPerUs);
}

void Servo::close() noexcept {
  std::scoped_lock lock(mutex_);
  pwm_.close();
}

bool Servo::is_open() const noexcept {
  std::scoped_lock lock(mutex_);
  return pwm_.is_open();
}

void Servo::set_angle_deg(double angle_deg) {
  std::scoped_lock lock(mutex_);
  const uint64_t pulse_us = angle_to_pulse_width_us(angle_deg);
  pwm_.set_duty_cycle_ns(pulse_us * kNsPerUs);
  pulse_width_us_ = pulse_us;
  angle_deg_ = pulse_width_to_angle_deg(pulse_us);
}

void Servo::set_pulse_width_us(uint64_t pulse_width_us) {
  std::scoped_lock lock(mutex_);
  if (pulse_width_us < config_.min_pulse_width_us || pulse_width_us > config_.max_pulse_width_us) {
    throw std::invalid_argument("Servo pulse_width_us is outside configured min/max limits");
  }

  pwm_.set_duty_cycle_ns(pulse_width_us * kNsPerUs);
  pulse_width_us_ = pulse_width_us;
  angle_deg_ = pulse_width_to_angle_deg(pulse_width_us);
}

void Servo::set_enabled(bool enabled) {
  std::scoped_lock lock(mutex_);
  pwm_.set_enabled(enabled);
}

uint64_t Servo::pulse_width_us() const noexcept {
  std::scoped_lock lock(mutex_);
  return pulse_width_us_;
}

double Servo::angle_deg() const noexcept {
  std::scoped_lock lock(mutex_);
  return angle_deg_;
}

uint32_t Servo::chip() const noexcept { return pwm_.chip(); }

uint32_t Servo::channel() const noexcept { return pwm_.channel(); }

uint64_t Servo::angle_to_pulse_width_us(double angle_deg) const {
  const double clamped_angle =
      std::max(config_.min_angle_deg, std::min(config_.max_angle_deg, angle_deg));
  const double span_angle = config_.max_angle_deg - config_.min_angle_deg;
  const double ratio = (clamped_angle - config_.min_angle_deg) / span_angle;
  const double span_pulse =
      static_cast<double>(config_.max_pulse_width_us - config_.min_pulse_width_us);
  const double pulse = static_cast<double>(config_.min_pulse_width_us) + (ratio * span_pulse);
  return static_cast<uint64_t>(std::llround(pulse));
}

double Servo::pulse_width_to_angle_deg(uint64_t pulse_width_us) const {
  const uint64_t clamped =
      std::max(config_.min_pulse_width_us, std::min(config_.max_pulse_width_us, pulse_width_us));
  const double span_pulse =
      static_cast<double>(config_.max_pulse_width_us - config_.min_pulse_width_us);
  const double ratio = static_cast<double>(clamped - config_.min_pulse_width_us) / span_pulse;
  return config_.min_angle_deg + ratio * (config_.max_angle_deg - config_.min_angle_deg);
}

}  // namespace servo
