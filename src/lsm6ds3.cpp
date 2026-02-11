#include "lsm6ds3/lsm6ds3.hpp"

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
constexpr uint8_t WHO_AM_I_LSM6DSR = 0x6B;

constexpr uint8_t REG_CTRL1_XL = 0x10;
constexpr uint8_t REG_CTRL2_G = 0x11;
constexpr uint8_t REG_CTRL3_C = 0x12;

constexpr uint8_t REG_OUTX_L_G = 0x22;
constexpr uint8_t REG_OUTX_L_XL = 0x28;

constexpr uint8_t CTRL3_C_IF_INC = 0x04;  // CTRL3_C.IF_INC for auto-increment.

constexpr double GRAVITY = 9.80665;
constexpr double DEG_TO_RAD = M_PI / 180.0;

int16_t combine_le_u8(uint8_t lsb, uint8_t msb) {
  return static_cast<int16_t>(static_cast<uint16_t>(lsb) |
                              (static_cast<uint16_t>(msb) << 8));
}

}  // namespace

Lsm6ds3::Lsm6ds3(std::string bus_path, uint8_t address, unsigned int retries)
    : i2c_(std::make_unique<LinuxI2c>(std::move(bus_path), address, retries)),
      initialized_(false),
      accel_odr_(AccelOdr::Hz104),
      gyro_odr_(GyroOdr::Hz104),
      accel_scale_(AccelScale::G2),
      gyro_scale_(GyroScale::Dps245) {
  if (address != 0x6A && address != 0x6B) {
    throw std::invalid_argument("LSM6DS3 I2C address must be 0x6A or 0x6B");
  }
}

Lsm6ds3::~Lsm6ds3() { close(true); }

void Lsm6ds3::begin() {
  i2c_->open();

  const uint8_t who_am_i = read_reg(REG_WHO_AM_I);
  if (who_am_i != WHO_AM_I_LSM6DS3 && who_am_i != WHO_AM_I_LSM6DSR) {
    close(false);
    std::ostringstream oss;
    oss << "Unexpected WHO_AM_I for LSM6DS3: got 0x" << std::hex
        << std::setw(2) << std::setfill('0') << static_cast<int>(who_am_i)
        << ", expected 0x69 (LSM6DS3) or 0x6b (LSM6DSR)";
    throw std::runtime_error(oss.str());
  }

  configure_defaults();
  initialized_ = true;
}

void Lsm6ds3::close(bool power_down) noexcept {
  if (power_down && i2c_->is_open()) {
    try {
      write_reg(REG_CTRL1_XL, 0x00);  // CTRL1_XL ODR_XL=power-down.
      write_reg(REG_CTRL2_G, 0x00);   // CTRL2_G ODR_G=power-down.
    } catch (...) {
      // no-throw close path
    }
  }

  initialized_ = false;
  i2c_->close();
}

void Lsm6ds3::enable_accel() { set_accel_odr(accel_odr_); }

void Lsm6ds3::enable_gyro() { set_gyro_odr(gyro_odr_); }

void Lsm6ds3::set_accel_odr(AccelOdr odr) {
  accel_odr_ = odr;
  if (!i2c_->is_open()) {
    return;
  }

  const uint8_t reg = static_cast<uint8_t>(odr) | static_cast<uint8_t>(accel_scale_);
  write_reg(REG_CTRL1_XL, reg);
}

void Lsm6ds3::set_gyro_odr(GyroOdr odr) {
  gyro_odr_ = odr;
  if (!i2c_->is_open()) {
    return;
  }

  const uint8_t reg = static_cast<uint8_t>(odr) | static_cast<uint8_t>(gyro_scale_);
  write_reg(REG_CTRL2_G, reg);
}

void Lsm6ds3::set_accel_scale(AccelScale scale) {
  accel_scale_ = scale;
  if (i2c_->is_open()) {
    set_accel_odr(accel_odr_);
  }
}

void Lsm6ds3::set_gyro_scale(GyroScale scale) {
  gyro_scale_ = scale;
  if (i2c_->is_open()) {
    set_gyro_odr(gyro_odr_);
  }
}

std::array<int16_t, 3> Lsm6ds3::read_accel_raw() {
  ensure_ready();
  return read_3_axis_raw(REG_OUTX_L_XL);
}

std::array<int16_t, 3> Lsm6ds3::read_gyro_raw() {
  ensure_ready();
  return read_3_axis_raw(REG_OUTX_L_G);
}

std::array<double, 3> Lsm6ds3::read_accel_si() {
  const auto raw = read_accel_raw();
  const double scale = accel_lsb_to_mps2();
  return {raw[0] * scale, raw[1] * scale, raw[2] * scale};
}

std::array<double, 3> Lsm6ds3::read_gyro_si() {
  const auto raw = read_gyro_raw();
  const double scale = gyro_lsb_to_rads();
  return {raw[0] * scale, raw[1] * scale, raw[2] * scale};
}

void Lsm6ds3::ensure_ready() const {
  if (!initialized_) {
    throw std::runtime_error("LSM6DS3 not initialized; call begin() first");
  }
}

void Lsm6ds3::configure_defaults() {
  write_reg(REG_CTRL3_C, CTRL3_C_IF_INC);  // Enable register auto-increment.
  set_accel_scale(AccelScale::G2);
  set_gyro_scale(GyroScale::Dps245);
  set_accel_odr(AccelOdr::Hz104);
  set_gyro_odr(GyroOdr::Hz104);
}

void Lsm6ds3::write_reg(uint8_t reg, uint8_t value) { i2c_->write_register(reg, value); }

uint8_t Lsm6ds3::read_reg(uint8_t reg) {
  const auto data = i2c_->read_registers(reg, 1);
  return data[0];
}

std::array<int16_t, 3> Lsm6ds3::read_3_axis_raw(uint8_t start_reg) {
  const auto data = i2c_->read_registers(start_reg, 6);
  return {
      combine_le_u8(data[0], data[1]),
      combine_le_u8(data[2], data[3]),
      combine_le_u8(data[4], data[5]),
  };
}

double Lsm6ds3::accel_lsb_to_mps2() const {
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

double Lsm6ds3::gyro_lsb_to_rads() const {
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
