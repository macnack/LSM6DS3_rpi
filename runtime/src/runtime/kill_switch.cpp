#include "runtime/runtime/kill_switch.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <unistd.h>

namespace runtime {

namespace {

std::string gpio_base(uint32_t gpio) {
  return "/sys/class/gpio/gpio" + std::to_string(gpio);
}

void write_text(const std::string& path, const std::string& value) {
  const int fd = ::open(path.c_str(), O_WRONLY | O_CLOEXEC);
  if (fd < 0) {
    throw std::runtime_error("Failed to open '" + path + "' for write (errno=" + std::to_string(errno) +
                             ": " + std::strerror(errno) + ")");
  }
  const ssize_t n = ::write(fd, value.data(), value.size());
  const int write_errno = errno;
  (void)::close(fd);
  if (n != static_cast<ssize_t>(value.size())) {
    throw std::runtime_error("Failed to write '" + path + "' (errno=" + std::to_string(write_errno) +
                             ": " + std::strerror(write_errno) + ")");
  }
}

}  // namespace

KillSwitchMonitor::KillSwitchMonitor(const KillSwitchSection& cfg) : cfg_(cfg) {}

KillSwitchMonitor::~KillSwitchMonitor() { stop(); }

void KillSwitchMonitor::start() {
  if (!cfg_.enabled || value_fd_ >= 0) {
    return;
  }

#if !defined(__linux__)
  throw std::runtime_error("Kill switch requires Linux sysfs GPIO support");
#else
  ensure_exported();
  configure_input();

  const std::string value_path = gpio_base(cfg_.gpio) + "/value";
  value_fd_ = ::open(value_path.c_str(), O_RDONLY | O_CLOEXEC);
  if (value_fd_ < 0) {
    throw std::runtime_error("Failed to open kill switch value pin at '" + value_path +
                             "' (errno=" + std::to_string(errno) + ": " + std::strerror(errno) + ")");
  }

  // Startup self-check: fail early if the GPIO cannot be read.
  try {
    (void)read_value();
    tripped_ = false;
    open_count_ = 0;
  } catch (...) {
    stop();
    throw;
  }
#endif
}

void KillSwitchMonitor::stop() noexcept {
  if (value_fd_ >= 0) {
    (void)::close(value_fd_);
    value_fd_ = -1;
  }

#if defined(__linux__)
  if (cfg_.enabled && exported_by_us_) {
    try {
      write_text("/sys/class/gpio/unexport", std::to_string(cfg_.gpio));
    } catch (...) {
    }
  }
#endif

  exported_by_us_ = false;
  open_count_ = 0;
}

bool KillSwitchMonitor::enabled() const noexcept { return cfg_.enabled; }

bool KillSwitchMonitor::tripped() const noexcept { return tripped_; }

bool KillSwitchMonitor::poll() {
  if (!cfg_.enabled) {
    return false;
  }
  if (value_fd_ < 0) {
    throw std::runtime_error("Kill switch is enabled but not started");
  }

  const int value = read_value();
  const bool nc_closed = (value == static_cast<int>(cfg_.nc_closed_value));
  const bool open_detected = !nc_closed;

  if (open_detected) {
    ++open_count_;
  } else {
    open_count_ = 0;
    if (!cfg_.latch_on_trip) {
      tripped_ = false;
    }
  }

  if (open_count_ >= cfg_.debounce_samples) {
    tripped_ = true;
  }

  return tripped_;
}

void KillSwitchMonitor::ensure_exported() {
  const std::string base = gpio_base(cfg_.gpio);
  if (std::filesystem::exists(base)) {
    exported_by_us_ = false;
    return;
  }

  write_text("/sys/class/gpio/export", std::to_string(cfg_.gpio));
  exported_by_us_ = true;

  for (int i = 0; i < 20; ++i) {
    if (std::filesystem::exists(base)) {
      return;
    }
    ::usleep(5'000);
  }

  throw std::runtime_error("Timed out waiting for GPIO export on kill switch pin " +
                           std::to_string(cfg_.gpio));
}

void KillSwitchMonitor::configure_input() {
  write_text(gpio_base(cfg_.gpio) + "/direction", "in");
}

int KillSwitchMonitor::read_value() {
  char ch = 0;
  if (::lseek(value_fd_, 0, SEEK_SET) < 0) {
    throw std::runtime_error("Kill switch lseek failed (errno=" + std::to_string(errno) + ": " +
                             std::strerror(errno) + ")");
  }
  if (::read(value_fd_, &ch, 1) != 1) {
    throw std::runtime_error("Kill switch read failed (errno=" + std::to_string(errno) + ": " +
                             std::strerror(errno) + ")");
  }
  if (ch == '0') {
    return 0;
  }
  if (ch == '1') {
    return 1;
  }
  throw std::runtime_error("Kill switch value file returned invalid character");
}

}  // namespace runtime
