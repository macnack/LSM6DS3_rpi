#include "ads1115/linux_i2c.hpp"

#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sstream>
#include <string>
#include <sys/ioctl.h>
#include <thread>
#include <unistd.h>

namespace ads1115 {

namespace {

std::string format_errno(const std::string& prefix, int err) {
  std::ostringstream oss;
  oss << prefix;
  if (err != 0) {
    oss << " (errno=" << err << ": " << std::strerror(err) << ")";
  }
  return oss.str();
}

bool is_retryable_errno(int err) {
  return err == EAGAIN || err == EINTR || err == ETIMEDOUT || err == EBUSY;
}

}  // namespace

I2cError::I2cError(const std::string& message, int error_number)
    : std::runtime_error(format_errno(message, error_number)), error_number_(error_number) {}

int I2cError::error_number() const noexcept { return error_number_; }

LinuxI2c::LinuxI2c(std::string bus_path, uint8_t address, unsigned int retries)
    : bus_path_(std::move(bus_path)), address_(address), retries_(retries), fd_(-1) {
  if (bus_path_.empty()) {
    throw std::invalid_argument("I2C bus path must not be empty");
  }
  if (address_ > 0x7F) {
    throw std::invalid_argument("I2C address must be 7-bit (0x00..0x7F)");
  }
}

LinuxI2c::~LinuxI2c() { close(); }

void LinuxI2c::open() {
  std::scoped_lock lock(mutex_);
  if (fd_ >= 0) {
    return;
  }

  const int fd = ::open(bus_path_.c_str(), O_RDWR | O_CLOEXEC);
  if (fd < 0) {
    throw_errno("Failed to open I2C bus '" + bus_path_ + "'");
  }
  fd_ = fd;
}

void LinuxI2c::close() noexcept {
  std::scoped_lock lock(mutex_);
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool LinuxI2c::is_open() const noexcept {
  std::scoped_lock lock(mutex_);
  return fd_ >= 0;
}

void LinuxI2c::write_register(uint8_t reg, const uint8_t* data, std::size_t length) {
  if ((data == nullptr) && (length > 0)) {
    throw std::invalid_argument("write_register data pointer is null while length > 0");
  }
  if (length > 255) {
    throw std::invalid_argument("write_register payload too large for one I2C message");
  }

  std::vector<uint8_t> tx;
  tx.reserve(length + 1);
  tx.push_back(reg);
  tx.insert(tx.end(), data, data + length);

  i2c_msg msg{};
  msg.addr = address_;
  msg.flags = 0;
  msg.len = static_cast<__u16>(tx.size());
  msg.buf = tx.data();

  run_rdwr(&msg, 1);
}

void LinuxI2c::write_register(uint8_t reg, uint8_t value) { write_register(reg, &value, 1); }

std::vector<uint8_t> LinuxI2c::read_registers(uint8_t reg, std::size_t length) {
  if (length == 0) {
    return {};
  }
  if (length > 65535) {
    throw std::invalid_argument("read_registers length too large");
  }

  std::vector<uint8_t> rx(length);
  uint8_t reg_addr = reg;

  i2c_msg msgs[2]{};
  msgs[0].addr = address_;
  msgs[0].flags = 0;
  msgs[0].len = 1;  // register address write
  msgs[0].buf = &reg_addr;

  msgs[1].addr = address_;
  msgs[1].flags = I2C_M_RD;  // repeated-start read
  msgs[1].len = static_cast<__u16>(length);
  msgs[1].buf = rx.data();

  run_rdwr(msgs, 2);
  return rx;
}

void LinuxI2c::ensure_open() const {
  if (fd_ < 0) {
    throw I2cError("I2C bus is not open");
  }
}

void LinuxI2c::run_rdwr(struct i2c_msg* messages, std::size_t message_count) {
  std::scoped_lock lock(mutex_);
  ensure_open();

  i2c_rdwr_ioctl_data rdwr{};
  rdwr.msgs = messages;
  rdwr.nmsgs = static_cast<__u32>(message_count);

  for (unsigned int attempt = 0; attempt <= retries_; ++attempt) {
    if (::ioctl(fd_, I2C_RDWR, &rdwr) >= 0) {
      return;
    }

    const int err = errno;
    if (attempt >= retries_ || !is_retryable_errno(err)) {
      throw I2cError("I2C_RDWR transaction failed", err);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

[[noreturn]] void LinuxI2c::throw_errno(const std::string& context) const {
  throw I2cError(context, errno);
}

}  // namespace ads1115
