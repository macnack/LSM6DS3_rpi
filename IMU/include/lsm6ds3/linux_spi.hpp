#pragma once

#include <cstdint>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

namespace lsm6ds3 {

class SpiError : public std::runtime_error {
 public:
  explicit SpiError(const std::string& message, int error_number = 0);
  [[nodiscard]] int error_number() const noexcept;

 private:
  int error_number_;
};

class LinuxSpi {
 public:
  LinuxSpi(std::string device_path, uint32_t speed_hz = 5000000, uint8_t mode = 3,
           uint8_t bits_per_word = 8, unsigned int retries = 2);
  ~LinuxSpi();

  LinuxSpi(const LinuxSpi&) = delete;
  LinuxSpi& operator=(const LinuxSpi&) = delete;
  LinuxSpi(LinuxSpi&&) = delete;
  LinuxSpi& operator=(LinuxSpi&&) = delete;

  void open();
  void close() noexcept;
  [[nodiscard]] bool is_open() const noexcept;

  void write_register(uint8_t reg, const uint8_t* data, std::size_t length);
  void write_register(uint8_t reg, uint8_t value);
  [[nodiscard]] std::vector<uint8_t> read_registers(uint8_t reg, std::size_t length);

  [[nodiscard]] uint32_t speed_hz() const noexcept;
  [[nodiscard]] uint8_t mode() const noexcept;
  [[nodiscard]] uint8_t bits_per_word() const noexcept;

 private:
  void ensure_open() const;
  void configure_device_locked();
  void transfer_locked(const uint8_t* tx, uint8_t* rx, std::size_t length);
  [[noreturn]] void throw_errno(const std::string& context) const;

  std::string device_path_;
  uint32_t speed_hz_;
  uint8_t mode_;
  uint8_t bits_per_word_;
  unsigned int retries_;
  int fd_;
  mutable std::mutex mutex_;
};

}  // namespace lsm6ds3
