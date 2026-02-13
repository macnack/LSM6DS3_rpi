#pragma once

#include "bmp390/linux_i2c.hpp"

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

struct bmp3_dev;

namespace bmp390 {

class Bmp390 {
 public:
  struct Reading {
    double temperature_c;
    double pressure_pa;
  };

  explicit Bmp390(std::string bus_path = "/dev/i2c-1", uint8_t address = 0x77,
                  unsigned int retries = 2);
  ~Bmp390();

  Bmp390(const Bmp390&) = delete;
  Bmp390& operator=(const Bmp390&) = delete;

  void begin();
  void close(bool power_down = true) noexcept;

  void configure_default();
  void set_sampling(uint8_t pressure_oversampling, uint8_t temperature_oversampling,
                    uint8_t odr, uint8_t iir_filter);

  [[nodiscard]] Reading read();
  [[nodiscard]] double read_temperature_c();
  [[nodiscard]] double read_pressure_pa();
  [[nodiscard]] double read_pressure_hpa();

 private:
  static int8_t i2c_read(uint8_t reg_addr, uint8_t* read_data, uint32_t len, void* intf_ptr);
  static int8_t i2c_write(uint8_t reg_addr, uint8_t* write_data, uint32_t len, void* intf_ptr);
  static void delay_us(uint32_t period, void* intf_ptr);

  void ensure_ready() const;
  void set_power_mode(uint8_t mode);
  void check_bmp_result(int8_t result, const char* operation) const;

  std::unique_ptr<LinuxI2c> i2c_;
  std::unique_ptr<bmp3_dev> dev_;
  bool initialized_;
  mutable std::mutex mutex_;
};

}  // namespace bmp390
