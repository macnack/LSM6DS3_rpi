#include "lsm6ds3/linux_spi.hpp"

#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sstream>
#include <string>
#include <sys/ioctl.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace lsm6ds3 {

namespace {

std::string format_errno(const std::string& prefix, int err) {
  std::ostringstream oss;
  oss << prefix;
  if (err != 0) {
    oss << " (errno=" << err << ": " << std::strerror(err) << ")";
  }
  return oss.str();
}

}  // namespace

SpiError::SpiError(const std::string& message, int error_number)
    : std::runtime_error(format_errno(message, error_number)),
      error_number_(error_number) {}

int SpiError::error_number() const noexcept { return error_number_; }

LinuxSpi::LinuxSpi(std::string device_path, uint32_t speed_hz, uint8_t mode,
                   uint8_t bits_per_word, unsigned int retries)
    : device_path_(std::move(device_path)),
      speed_hz_(speed_hz),
      mode_(mode),
      bits_per_word_(bits_per_word),
      retries_(retries),
      fd_(-1) {
  if (device_path_.empty()) {
    throw std::invalid_argument("SPI device path must not be empty");
  }
  if (speed_hz_ == 0) {
    throw std::invalid_argument("SPI speed must be > 0");
  }
  if (mode_ > 3) {
    throw std::invalid_argument("SPI mode must be in range 0..3");
  }
  if (bits_per_word_ == 0) {
    throw std::invalid_argument("SPI bits_per_word must be > 0");
  }
}

LinuxSpi::~LinuxSpi() { close(); }

void LinuxSpi::open() {
  std::scoped_lock lock(mutex_);
  if (fd_ >= 0) {
    return;
  }

  const int fd = ::open(device_path_.c_str(), O_RDWR | O_CLOEXEC);
  if (fd < 0) {
    throw_errno("Failed to open SPI device '" + device_path_ + "'");
  }
  fd_ = fd;

  try {
    configure_device_locked();
  } catch (...) {
    ::close(fd_);
    fd_ = -1;
    throw;
  }
}

void LinuxSpi::close() noexcept {
  std::scoped_lock lock(mutex_);
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool LinuxSpi::is_open() const noexcept {
  std::scoped_lock lock(mutex_);
  return fd_ >= 0;
}

void LinuxSpi::write_register(uint8_t reg, const uint8_t* data, std::size_t length) {
  if ((data == nullptr) && (length > 0)) {
    throw std::invalid_argument("write_register data pointer is null while length > 0");
  }
  if (length > 4096) {
    throw std::invalid_argument("write_register length exceeds sane limit (4096)");
  }

  std::vector<uint8_t> tx;
  tx.reserve(length + 1);
  tx.push_back(static_cast<uint8_t>(reg & 0x7F));  // MSB=0 for write.
  if (length > 0) {
    tx.insert(tx.end(), data, data + length);
  }

  std::scoped_lock lock(mutex_);
  ensure_open();
  transfer_locked(tx.data(), nullptr, tx.size());
}

void LinuxSpi::write_register(uint8_t reg, uint8_t value) { write_register(reg, &value, 1); }

std::vector<uint8_t> LinuxSpi::read_registers(uint8_t reg, std::size_t length) {
  if (length == 0) {
    return {};
  }
  if (length > 4096) {
    throw std::invalid_argument("read_registers length exceeds sane limit (4096)");
  }

  std::vector<uint8_t> tx(length + 1, 0x00);
  std::vector<uint8_t> rx(length + 1, 0x00);

  tx[0] = static_cast<uint8_t>(reg | 0x80);  // MSB=1 for read.

  std::scoped_lock lock(mutex_);
  ensure_open();
  transfer_locked(tx.data(), rx.data(), tx.size());

  return {rx.begin() + 1, rx.end()};
}

uint32_t LinuxSpi::speed_hz() const noexcept { return speed_hz_; }

uint8_t LinuxSpi::mode() const noexcept { return mode_; }

uint8_t LinuxSpi::bits_per_word() const noexcept { return bits_per_word_; }

void LinuxSpi::ensure_open() const {
  if (fd_ < 0) {
    throw SpiError("SPI device is not open");
  }
}

void LinuxSpi::configure_device_locked() {
  ensure_open();

  uint8_t mode = mode_;
  if (::ioctl(fd_, SPI_IOC_WR_MODE, &mode) < 0) {
    throw_errno("Failed to set SPI mode");
  }
  if (::ioctl(fd_, SPI_IOC_RD_MODE, &mode) < 0) {
    throw_errno("Failed to read SPI mode");
  }
  mode_ = mode;

  uint8_t bits = bits_per_word_;
  if (::ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) {
    throw_errno("Failed to set SPI bits-per-word");
  }
  if (::ioctl(fd_, SPI_IOC_RD_BITS_PER_WORD, &bits) < 0) {
    throw_errno("Failed to read SPI bits-per-word");
  }
  bits_per_word_ = bits;

  uint32_t speed = speed_hz_;
  if (::ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
    throw_errno("Failed to set SPI max speed");
  }
  if (::ioctl(fd_, SPI_IOC_RD_MAX_SPEED_HZ, &speed) < 0) {
    throw_errno("Failed to read SPI max speed");
  }
  speed_hz_ = speed;
}

void LinuxSpi::transfer_locked(const uint8_t* tx, uint8_t* rx, std::size_t length) {
  ensure_open();
  if (length == 0) {
    return;
  }
  if (length > 0xFFFFU) {
    throw std::invalid_argument("SPI transfer length exceeds kernel transfer limit");
  }

  spi_ioc_transfer transfer{};
  transfer.tx_buf = reinterpret_cast<uintptr_t>(tx);
  transfer.rx_buf = reinterpret_cast<uintptr_t>(rx);
  transfer.len = static_cast<__u32>(length);
  transfer.speed_hz = speed_hz_;
  transfer.bits_per_word = bits_per_word_;

  const unsigned int attempts = retries_ + 1;
  for (unsigned int attempt = 0; attempt < attempts; ++attempt) {
    if (::ioctl(fd_, SPI_IOC_MESSAGE(1), &transfer) >= 0) {
      return;
    }

    const int err = errno;
    const bool transient = (err == EAGAIN || err == EINTR || err == ETIMEDOUT || err == EBUSY);
    if (!transient || attempt + 1 >= attempts) {
      throw SpiError("SPI transfer failed", err);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(2 * (attempt + 1)));
  }

  throw SpiError("Unexpected SPI retry loop termination");
}

[[noreturn]] void LinuxSpi::throw_errno(const std::string& context) const {
  throw SpiError(context, errno);
}

}  // namespace lsm6ds3
