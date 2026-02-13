#include "bmp390/bmp390.hpp"

#include "../I2C/bmp3.h"
#include "../I2C/bmp3_defs.h"

#include <chrono>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace bmp390 {

namespace {

void validate_oversampling(uint8_t value, const char* name) {
  if (value > BMP3_OVERSAMPLING_32X) {
    std::ostringstream oss;
    oss << name << " must be one of BMP3_NO_OVERSAMPLING..BMP3_OVERSAMPLING_32X";
    throw std::invalid_argument(oss.str());
  }
}

void validate_odr(uint8_t value) {
  if (value > BMP3_ODR_0_001_HZ) {
    throw std::invalid_argument("ODR must be one of BMP3_ODR_* constants");
  }
}

void validate_iir(uint8_t value) {
  if (value > BMP3_IIR_FILTER_COEFF_127) {
    throw std::invalid_argument("IIR filter must be one of BMP3_IIR_FILTER_* constants");
  }
}

}  // namespace

Bmp390::Bmp390(std::string bus_path, uint8_t address, unsigned int retries)
    : i2c_(std::make_unique<LinuxI2c>(std::move(bus_path), address, retries)),
      dev_(std::make_unique<bmp3_dev>()),
      initialized_(false) {
  std::memset(dev_.get(), 0, sizeof(bmp3_dev));
  dev_->intf = BMP3_I2C_INTF;
  dev_->intf_ptr = i2c_.get();
  dev_->read = &Bmp390::i2c_read;
  dev_->write = &Bmp390::i2c_write;
  dev_->delay_us = &Bmp390::delay_us;
}

Bmp390::~Bmp390() { close(true); }

void Bmp390::begin() {
  i2c_->open();
  check_bmp_result(bmp3_init(dev_.get()), "bmp3_init");

  if (dev_->chip_id != BMP390_CHIP_ID) {
    std::ostringstream oss;
    oss << "Unexpected BMP390 chip id: 0x" << std::hex << static_cast<int>(dev_->chip_id)
        << ", expected 0x" << static_cast<int>(BMP390_CHIP_ID);
    throw std::runtime_error(oss.str());
  }

  {
    std::scoped_lock lock(mutex_);
    initialized_ = true;
  }

  configure_default();
}

void Bmp390::close(bool power_down) noexcept {
  std::scoped_lock lock(mutex_);

  if (power_down && initialized_ && i2c_->is_open()) {
    try {
      set_power_mode(BMP3_MODE_SLEEP);
    } catch (...) {
      // no-throw close path
    }
  }

  initialized_ = false;
  i2c_->close();
}

void Bmp390::configure_default() {
  ensure_ready();
  set_sampling(
      BMP3_OVERSAMPLING_8X,    // osr_p in OSR register (0x1C)
      BMP3_OVERSAMPLING_2X,    // osr_t in OSR register (0x1C)
      BMP3_ODR_25_HZ,          // odr in ODR register (0x1D)
      BMP3_IIR_FILTER_COEFF_3  // filter in CONFIG register (0x1F)
  );
}

void Bmp390::set_sampling(uint8_t pressure_oversampling, uint8_t temperature_oversampling,
                          uint8_t odr, uint8_t iir_filter) {
  std::scoped_lock lock(mutex_);
  ensure_ready();

  validate_oversampling(pressure_oversampling, "pressure_oversampling");
  validate_oversampling(temperature_oversampling, "temperature_oversampling");
  validate_odr(odr);
  validate_iir(iir_filter);

  bmp3_settings settings{};
  settings.press_en = BMP3_ENABLE;  // PWR_CTRL.press_en
  settings.temp_en = BMP3_ENABLE;   // PWR_CTRL.temp_en
  settings.odr_filter.press_os = pressure_oversampling;
  settings.odr_filter.temp_os = temperature_oversampling;
  settings.odr_filter.odr = odr;
  settings.odr_filter.iir_filter = iir_filter;

  constexpr uint32_t kDesiredSettings =
      BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN | BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS |
      BMP3_SEL_ODR | BMP3_SEL_IIR_FILTER;

  check_bmp_result(bmp3_set_sensor_settings(kDesiredSettings, &settings, dev_.get()),
                   "bmp3_set_sensor_settings");
  set_power_mode(BMP3_MODE_NORMAL);
}

Bmp390::Reading Bmp390::read() {
  std::scoped_lock lock(mutex_);
  ensure_ready();

  bmp3_data data{};
  check_bmp_result(bmp3_get_sensor_data(BMP3_PRESS_TEMP, &data, dev_.get()), "bmp3_get_sensor_data");

#ifdef BMP3_FLOAT_COMPENSATION
  return {data.temperature, data.pressure};
#else
  // TODO: Verify fixed-point scaling for integer-compensation builds if Bosch updates API.
  return {static_cast<double>(data.temperature) / 100.0, static_cast<double>(data.pressure) / 100.0};
#endif
}

double Bmp390::read_temperature_c() { return read().temperature_c; }

double Bmp390::read_pressure_pa() { return read().pressure_pa; }

double Bmp390::read_pressure_hpa() { return read_pressure_pa() / 100.0; }

int8_t Bmp390::i2c_read(uint8_t reg_addr, uint8_t* read_data, uint32_t len, void* intf_ptr) {
  if (read_data == nullptr || intf_ptr == nullptr) {
    return BMP3_E_NULL_PTR;
  }

  try {
    auto* i2c = static_cast<LinuxI2c*>(intf_ptr);
    const auto bytes = i2c->read_registers(reg_addr, len);
    std::memcpy(read_data, bytes.data(), len);
    return BMP3_OK;
  } catch (...) {
    return BMP3_E_COMM_FAIL;
  }
}

int8_t Bmp390::i2c_write(uint8_t reg_addr, uint8_t* write_data, uint32_t len, void* intf_ptr) {
  if ((write_data == nullptr && len > 0) || intf_ptr == nullptr) {
    return BMP3_E_NULL_PTR;
  }

  try {
    auto* i2c = static_cast<LinuxI2c*>(intf_ptr);
    i2c->write_register(reg_addr, write_data, len);
    return BMP3_OK;
  } catch (...) {
    return BMP3_E_COMM_FAIL;
  }
}

void Bmp390::delay_us(uint32_t period, void* /*intf_ptr*/) {
  std::this_thread::sleep_for(std::chrono::microseconds(period));
}

void Bmp390::ensure_ready() const {
  if (!initialized_) {
    throw std::runtime_error("BMP390 not initialized; call begin() first");
  }
}

void Bmp390::set_power_mode(uint8_t mode) {
  bmp3_settings settings{};
  settings.op_mode = mode;
  check_bmp_result(bmp3_set_op_mode(&settings, dev_.get()), "bmp3_set_op_mode");
}

void Bmp390::check_bmp_result(int8_t result, const char* operation) const {
  if (result == BMP3_OK) {
    return;
  }

  std::ostringstream oss;
  oss << operation << " failed with code " << static_cast<int>(result);

  switch (result) {
    case BMP3_E_COMM_FAIL:
      oss << " (communication failure)";
      break;
    case BMP3_E_DEV_NOT_FOUND:
      oss << " (device not found)";
      break;
    case BMP3_E_INVALID_ODR_OSR_SETTINGS:
      oss << " (invalid ODR/oversampling combination)";
      break;
    default:
      break;
  }

  throw std::runtime_error(oss.str());
}

}  // namespace bmp390
