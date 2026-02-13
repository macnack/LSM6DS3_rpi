#pragma once

#include "lsm6ds3/linux_i2c.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

namespace lsm6ds3 {

class Lsm6ds3 {
 public:
  enum class AccelOdr : uint8_t {
    PowerDown = 0x00,
    Hz12_5 = 0x10,
    Hz26 = 0x20,
    Hz52 = 0x30,
    Hz104 = 0x40,
    Hz208 = 0x50,
    Hz416 = 0x60,
    Hz833 = 0x70,
  };

  enum class GyroOdr : uint8_t {
    PowerDown = 0x00,
    Hz12_5 = 0x10,
    Hz26 = 0x20,
    Hz52 = 0x30,
    Hz104 = 0x40,
    Hz208 = 0x50,
    Hz416 = 0x60,
    Hz833 = 0x70,
  };

  enum class AccelScale : uint8_t {
    G2 = 0x00,
    G16 = 0x04,
    G4 = 0x08,
    G8 = 0x0C,
  };

  enum class GyroScale : uint8_t {
    Dps245 = 0x00,  // 250 dps nominal in many APIs.
    Dps500 = 0x04,
    Dps1000 = 0x08,
    Dps2000 = 0x0C,
  };

  explicit Lsm6ds3(std::string bus_path = "/dev/i2c-1", uint8_t address = 0x6A,
                   unsigned int retries = 2);
  ~Lsm6ds3();

  Lsm6ds3(const Lsm6ds3&) = delete;
  Lsm6ds3& operator=(const Lsm6ds3&) = delete;

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

  std::unique_ptr<LinuxI2c> i2c_;
  bool initialized_;

  AccelOdr accel_odr_;
  GyroOdr gyro_odr_;
  AccelScale accel_scale_;
  GyroScale gyro_scale_;
  mutable std::mutex mutex_;
};

}  // namespace lsm6ds3
