#pragma once

#include <cstdint>
#include <linux/i2c.h>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

namespace lsm6ds3 {


class I2cError : public std::runtime_error {
 public:
  explicit I2cError(const std::string& message, int error_number = 0);
  [[nodiscard]] int error_number() const noexcept;

 private:
  int error_number_;
};

class LinuxI2c {
 public:
  LinuxI2c(std::string bus_path, uint8_t address, unsigned int retries = 2);
  ~LinuxI2c();

  LinuxI2c(const LinuxI2c&) = delete;
  LinuxI2c& operator=(const LinuxI2c&) = delete;
  LinuxI2c(LinuxI2c&&) = delete;
  LinuxI2c& operator=(LinuxI2c&&) = delete;

  void open();
  void close() noexcept;
  [[nodiscard]] bool is_open() const noexcept;

  void write_register(uint8_t reg, const uint8_t* data, std::size_t length);
  void write_register(uint8_t reg, uint8_t value);
  [[nodiscard]] std::vector<uint8_t> read_registers(uint8_t reg, std::size_t length);

 private:
  void ensure_open() const;
  void run_rdwr(struct i2c_msg* messages, std::size_t message_count);
  [[noreturn]] void throw_errno(const std::string& context) const;

  std::string bus_path_;
  uint8_t address_;
  unsigned int retries_;
  int fd_;
  mutable std::mutex mutex_;
};

}  // namespace lsm6ds3
