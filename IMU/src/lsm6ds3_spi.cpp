#include "lsm6ds3/lsm6ds3_spi.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace lsm6ds3 {

namespace {

constexpr uint8_t REG_WHO_AM_I = 0x0F;
constexpr uint8_t WHO_AM_I_LSM6DS3 = 0x69;
constexpr uint8_t WHO_AM_I_ISM330DLC = 0x6A;
constexpr uint8_t WHO_AM_I_LSM6DSR = 0x6B;

constexpr uint8_t REG_CTRL1_XL = 0x10;
constexpr uint8_t REG_CTRL2_G = 0x11;
constexpr uint8_t REG_CTRL3_C = 0x12;

constexpr uint8_t REG_OUTX_L_G = 0x22;
constexpr uint8_t REG_OUTX_L_XL = 0x28;

constexpr uint8_t CTRL3_C_IF_INC = 0x04;  // CTRL3_C.IF_INC for auto-increment.

constexpr double GRAVITY = 9.80665;
constexpr double DEG_TO_RAD = 3.14159265358979323846 / 180.0;

int16_t combine_le_u8(uint8_t lsb, uint8_t msb) {
  return static_cast<int16_t>(static_cast<uint16_t>(lsb) |
                              (static_cast<uint16_t>(msb) << 8));
}

}  // namespace

Lsm6ds3Spi::Lsm6ds3Spi(std::string device_path, uint32_t speed_hz, uint8_t mode,
                       unsigned int retries)
    : spi_(std::make_unique<LinuxSpi>(std::move(device_path), speed_hz, mode, 8, retries)),
      initialized_(false),
      accel_odr_(AccelOdr::Hz104),
      gyro_odr_(GyroOdr::Hz104),
      accel_scale_(AccelScale::G2),
      gyro_scale_(GyroScale::Dps245) {}

Lsm6ds3Spi::~Lsm6ds3Spi() { close(true); }

void Lsm6ds3Spi::begin() {
  std::scoped_lock lock(mutex_);
  spi_->open();

  const uint8_t who_am_i = read_reg_locked(REG_WHO_AM_I);
  if (who_am_i != WHO_AM_I_LSM6DS3 && who_am_i != WHO_AM_I_ISM330DLC &&
      who_am_i != WHO_AM_I_LSM6DSR) {
    initialized_ = false;
    spi_->close();
    std::ostringstream oss;
    oss << "Unexpected WHO_AM_I for LSM6DS3 family over SPI: got 0x" << std::hex
        << std::setw(2) << std::setfill('0') << static_cast<int>(who_am_i)
        << ", expected 0x69 (LSM6DS3), 0x6a (ISM330DLC), or 0x6b (LSM6DSR)";
    throw std::runtime_error(oss.str());
  }

  configure_defaults_locked();
  initialized_ = true;
}

void Lsm6ds3Spi::close(bool power_down) noexcept {
  std::scoped_lock lock(mutex_);

  if (power_down && initialized_ && spi_->is_open()) {
    try {
      write_reg_locked(REG_CTRL1_XL, 0x00);  // CTRL1_XL ODR_XL=power-down.
      write_reg_locked(REG_CTRL2_G, 0x00);   // CTRL2_G ODR_G=power-down.
    } catch (...) {
      // no-throw close path
    }
  }

  initialized_ = false;
  spi_->close();
}

void Lsm6ds3Spi::enable_accel() {
  std::scoped_lock lock(mutex_);
  if (spi_->is_open()) {
    apply_accel_settings_locked();
  }
}

void Lsm6ds3Spi::enable_gyro() {
  std::scoped_lock lock(mutex_);
  if (spi_->is_open()) {
    apply_gyro_settings_locked();
  }
}

void Lsm6ds3Spi::set_accel_odr(AccelOdr odr) {
  std::scoped_lock lock(mutex_);
  accel_odr_ = odr;
  if (!spi_->is_open()) {
    return;
  }

  apply_accel_settings_locked();
}

void Lsm6ds3Spi::set_gyro_odr(GyroOdr odr) {
  std::scoped_lock lock(mutex_);
  gyro_odr_ = odr;
  if (!spi_->is_open()) {
    return;
  }

  apply_gyro_settings_locked();
}

void Lsm6ds3Spi::set_accel_scale(AccelScale scale) {
  std::scoped_lock lock(mutex_);
  accel_scale_ = scale;
  if (spi_->is_open()) {
    apply_accel_settings_locked();
  }
}

void Lsm6ds3Spi::set_gyro_scale(GyroScale scale) {
  std::scoped_lock lock(mutex_);
  gyro_scale_ = scale;
  if (spi_->is_open()) {
    apply_gyro_settings_locked();
  }
}

std::array<int16_t, 3> Lsm6ds3Spi::read_accel_raw() {
  std::scoped_lock lock(mutex_);
  ensure_ready_locked();
  return read_3_axis_raw_locked(REG_OUTX_L_XL);
}

std::array<int16_t, 3> Lsm6ds3Spi::read_gyro_raw() {
  std::scoped_lock lock(mutex_);
  ensure_ready_locked();
  return read_3_axis_raw_locked(REG_OUTX_L_G);
}

std::array<double, 3> Lsm6ds3Spi::read_accel_si() {
  std::scoped_lock lock(mutex_);
  ensure_ready_locked();
  const auto raw = read_3_axis_raw_locked(REG_OUTX_L_XL);
  const double scale = accel_lsb_to_mps2_locked();
  return {raw[0] * scale, raw[1] * scale, raw[2] * scale};
}

std::array<double, 3> Lsm6ds3Spi::read_gyro_si() {
  std::scoped_lock lock(mutex_);
  ensure_ready_locked();
  const auto raw = read_3_axis_raw_locked(REG_OUTX_L_G);
  const double scale = gyro_lsb_to_rads_locked();
  return {raw[0] * scale, raw[1] * scale, raw[2] * scale};
}

void Lsm6ds3Spi::ensure_ready_locked() const {
  if (!initialized_) {
    throw std::runtime_error("LSM6DS3 SPI sensor not initialized; call begin() first");
  }
}

void Lsm6ds3Spi::configure_defaults_locked() {
  write_reg_locked(REG_CTRL3_C, CTRL3_C_IF_INC);  // Enable register auto-increment.
  accel_scale_ = AccelScale::G2;
  gyro_scale_ = GyroScale::Dps245;
  accel_odr_ = AccelOdr::Hz104;
  gyro_odr_ = GyroOdr::Hz104;
  apply_accel_settings_locked();
  apply_gyro_settings_locked();
}

void Lsm6ds3Spi::write_reg_locked(uint8_t reg, uint8_t value) { spi_->write_register(reg, value); }

uint8_t Lsm6ds3Spi::read_reg_locked(uint8_t reg) {
  const auto data = spi_->read_registers(reg, 1);
  return data[0];
}

std::array<int16_t, 3> Lsm6ds3Spi::read_3_axis_raw_locked(uint8_t start_reg) {
  const auto data = spi_->read_registers(start_reg, 6);
  return {
      combine_le_u8(data[0], data[1]),
      combine_le_u8(data[2], data[3]),
      combine_le_u8(data[4], data[5]),
  };
}

void Lsm6ds3Spi::apply_accel_settings_locked() {
  const uint8_t reg = static_cast<uint8_t>(accel_odr_) | static_cast<uint8_t>(accel_scale_);
  write_reg_locked(REG_CTRL1_XL, reg);
}

void Lsm6ds3Spi::apply_gyro_settings_locked() {
  const uint8_t reg = static_cast<uint8_t>(gyro_odr_) | static_cast<uint8_t>(gyro_scale_);
  write_reg_locked(REG_CTRL2_G, reg);
}

double Lsm6ds3Spi::accel_lsb_to_mps2_locked() const {
  switch (accel_scale_) {
    case AccelScale::G2:
      return (0.061e-3 * GRAVITY);
    case AccelScale::G4:
      return (0.122e-3 * GRAVITY);
    case AccelScale::G8:
      return (0.244e-3 * GRAVITY);
    case AccelScale::G16:
      return (0.488e-3 * GRAVITY);
  }
  throw std::runtime_error("Unsupported accel scale");
}

double Lsm6ds3Spi::gyro_lsb_to_rads_locked() const {
  switch (gyro_scale_) {
    case GyroScale::Dps245:
      return (8.75e-3 * DEG_TO_RAD);
    case GyroScale::Dps500:
      return (17.50e-3 * DEG_TO_RAD);
    case GyroScale::Dps1000:
      return (35.0e-3 * DEG_TO_RAD);
    case GyroScale::Dps2000:
      return (70.0e-3 * DEG_TO_RAD);
  }
  throw std::runtime_error("Unsupported gyro scale");
}

}  // namespace lsm6ds3
