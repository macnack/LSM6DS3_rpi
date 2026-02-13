#pragma once

#include "lsm6ds3/linux_spi.hpp"
#include "lsm6ds3/lsm6ds3.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

namespace lsm6ds3 {

class Lsm6ds3Spi {
 public:
  using AccelOdr = Lsm6ds3::AccelOdr;
  using GyroOdr = Lsm6ds3::GyroOdr;
  using AccelScale = Lsm6ds3::AccelScale;
  using GyroScale = Lsm6ds3::GyroScale;

  explicit Lsm6ds3Spi(std::string device_path = "/dev/spidev0.0", uint32_t speed_hz = 5000000,
                      uint8_t mode = 3, unsigned int retries = 2);
  ~Lsm6ds3Spi();

  Lsm6ds3Spi(const Lsm6ds3Spi&) = delete;
  Lsm6ds3Spi& operator=(const Lsm6ds3Spi&) = delete;

  void begin();
  void close(bool power_down = true) noexcept;

  void enable_accel();
  void enable_gyro();

  void set_accel_odr(AccelOdr odr);
  void set_gyro_odr(GyroOdr odr);
  void set_accel_scale(AccelScale scale);
  void set_gyro_scale(GyroScale scale);

  [[nodiscard]] std::array<int16_t, 3> read_accel_raw();
  [[nodiscard]] std::array<int16_t, 3> read_gyro_raw();

  [[nodiscard]] std::array<double, 3> read_accel_si();
  [[nodiscard]] std::array<double, 3> read_gyro_si();

 private:
  void ensure_ready_locked() const;
  void configure_defaults_locked();
  void write_reg_locked(uint8_t reg, uint8_t value);
  [[nodiscard]] uint8_t read_reg_locked(uint8_t reg);
  [[nodiscard]] std::array<int16_t, 3> read_3_axis_raw_locked(uint8_t start_reg);

  void apply_accel_settings_locked();
  void apply_gyro_settings_locked();

  [[nodiscard]] double accel_lsb_to_mps2_locked() const;
  [[nodiscard]] double gyro_lsb_to_rads_locked() const;

  std::unique_ptr<LinuxSpi> spi_;
  bool initialized_;

  AccelOdr accel_odr_;
  GyroOdr gyro_odr_;
  AccelScale accel_scale_;
  GyroScale gyro_scale_;
  mutable std::mutex mutex_;
};

}  // namespace lsm6ds3
