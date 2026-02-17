#include "runtime/runtime/sensors.hpp"

#include <cmath>
#include <cctype>
#include <memory>
#include <string>
#include <stdexcept>

#if defined(__linux__) && (RUNTIME_HAVE_IMU == 1)
#include "lsm6ds3/lsm6ds3_spi.hpp"
#define RUNTIME_ENABLE_IMU_HW 1
#else
#define RUNTIME_ENABLE_IMU_HW 0
#endif

#if defined(__linux__) && (RUNTIME_HAVE_BARO == 1)
#include "bmp390/bmp390.hpp"
#define RUNTIME_ENABLE_BARO_HW 1
#else
#define RUNTIME_ENABLE_BARO_HW 0
#endif

namespace runtime {

namespace {

#if RUNTIME_ENABLE_IMU_HW
std::string normalize_token(std::string value) {
  std::string out;
  out.reserve(value.size());
  for (const char ch : value) {
    if (ch == '_' || ch == '-' || std::isspace(static_cast<unsigned char>(ch))) {
      continue;
    }
    out.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(ch))));
  }
  return out;
}
#endif

#if RUNTIME_ENABLE_IMU_HW
class HardwareImuBackend final : public ImuBackend {
 public:
  explicit HardwareImuBackend(const RuntimeConfig& cfg)
      : sensor_(cfg.imu.spi_device, cfg.imu.spi_speed_hz, static_cast<uint8_t>(cfg.imu.spi_mode)),
        cfg_(cfg) {}

  void start() override {
    sensor_.begin();
    sensor_.set_accel_odr(parse_accel_odr(cfg_.imu.accel_odr));
    sensor_.set_gyro_odr(parse_gyro_odr(cfg_.imu.gyro_odr));
    sensor_.set_accel_scale(parse_accel_scale(cfg_.imu.accel_scale));
    sensor_.set_gyro_scale(parse_gyro_scale(cfg_.imu.gyro_scale));
  }

  void stop() noexcept override { sensor_.close(true); }

  ImuSample read_once(uint64_t now_ns) override {
    const auto accel = sensor_.read_accel_si();
    const auto gyro = sensor_.read_gyro_si();

    ImuSample out;
    out.t_ns = now_ns;
    out.ax_mps2 = accel[0];
    out.ay_mps2 = accel[1];
    out.az_mps2 = accel[2];
    out.gx_rads = gyro[0];
    out.gy_rads = gyro[1];
    out.gz_rads = gyro[2];
    out.valid = true;
    return out;
  }

 private:
  static lsm6ds3::Lsm6ds3Spi::AccelOdr parse_accel_odr(const std::string& raw) {
    const std::string v = normalize_token(raw);
    if (v == "powerdown" || v == "off" || v == "0" || v == "0hz") {
      return lsm6ds3::Lsm6ds3Spi::AccelOdr::PowerDown;
    }
    if (v == "12.5" || v == "12.5hz" || v == "12p5hz" || v == "125hz") {
      return lsm6ds3::Lsm6ds3Spi::AccelOdr::Hz12_5;
    }
    if (v == "26" || v == "26hz") {
      return lsm6ds3::Lsm6ds3Spi::AccelOdr::Hz26;
    }
    if (v == "52" || v == "52hz") {
      return lsm6ds3::Lsm6ds3Spi::AccelOdr::Hz52;
    }
    if (v == "104" || v == "104hz") {
      return lsm6ds3::Lsm6ds3Spi::AccelOdr::Hz104;
    }
    if (v == "208" || v == "208hz") {
      return lsm6ds3::Lsm6ds3Spi::AccelOdr::Hz208;
    }
    if (v == "416" || v == "416hz") {
      return lsm6ds3::Lsm6ds3Spi::AccelOdr::Hz416;
    }
    if (v == "833" || v == "833hz") {
      return lsm6ds3::Lsm6ds3Spi::AccelOdr::Hz833;
    }
    throw std::runtime_error("Invalid imu.accel_odr: '" + raw +
                             "' (expected power_down|12.5hz|26hz|52hz|104hz|208hz|416hz|833hz)");
  }

  static lsm6ds3::Lsm6ds3Spi::GyroOdr parse_gyro_odr(const std::string& raw) {
    const std::string v = normalize_token(raw);
    if (v == "powerdown" || v == "off" || v == "0" || v == "0hz") {
      return lsm6ds3::Lsm6ds3Spi::GyroOdr::PowerDown;
    }
    if (v == "12.5" || v == "12.5hz" || v == "12p5hz" || v == "125hz") {
      return lsm6ds3::Lsm6ds3Spi::GyroOdr::Hz12_5;
    }
    if (v == "26" || v == "26hz") {
      return lsm6ds3::Lsm6ds3Spi::GyroOdr::Hz26;
    }
    if (v == "52" || v == "52hz") {
      return lsm6ds3::Lsm6ds3Spi::GyroOdr::Hz52;
    }
    if (v == "104" || v == "104hz") {
      return lsm6ds3::Lsm6ds3Spi::GyroOdr::Hz104;
    }
    if (v == "208" || v == "208hz") {
      return lsm6ds3::Lsm6ds3Spi::GyroOdr::Hz208;
    }
    if (v == "416" || v == "416hz") {
      return lsm6ds3::Lsm6ds3Spi::GyroOdr::Hz416;
    }
    if (v == "833" || v == "833hz") {
      return lsm6ds3::Lsm6ds3Spi::GyroOdr::Hz833;
    }
    throw std::runtime_error("Invalid imu.gyro_odr: '" + raw +
                             "' (expected power_down|12.5hz|26hz|52hz|104hz|208hz|416hz|833hz)");
  }

  static lsm6ds3::Lsm6ds3Spi::AccelScale parse_accel_scale(const std::string& raw) {
    const std::string v = normalize_token(raw);
    if (v == "2g" || v == "2") {
      return lsm6ds3::Lsm6ds3Spi::AccelScale::G2;
    }
    if (v == "4g" || v == "4") {
      return lsm6ds3::Lsm6ds3Spi::AccelScale::G4;
    }
    if (v == "8g" || v == "8") {
      return lsm6ds3::Lsm6ds3Spi::AccelScale::G8;
    }
    if (v == "16g" || v == "16") {
      return lsm6ds3::Lsm6ds3Spi::AccelScale::G16;
    }
    throw std::runtime_error("Invalid imu.accel_scale: '" + raw + "' (expected 2g|4g|8g|16g)");
  }

  static lsm6ds3::Lsm6ds3Spi::GyroScale parse_gyro_scale(const std::string& raw) {
    const std::string v = normalize_token(raw);
    if (v == "245dps" || v == "245") {
      return lsm6ds3::Lsm6ds3Spi::GyroScale::Dps245;
    }
    if (v == "500dps" || v == "500") {
      return lsm6ds3::Lsm6ds3Spi::GyroScale::Dps500;
    }
    if (v == "1000dps" || v == "1000") {
      return lsm6ds3::Lsm6ds3Spi::GyroScale::Dps1000;
    }
    if (v == "2000dps" || v == "2000") {
      return lsm6ds3::Lsm6ds3Spi::GyroScale::Dps2000;
    }
    throw std::runtime_error("Invalid imu.gyro_scale: '" + raw +
                             "' (expected 245dps|500dps|1000dps|2000dps)");
  }

  lsm6ds3::Lsm6ds3Spi sensor_;
  RuntimeConfig cfg_;
};
#else
class HardwareImuBackend final : public ImuBackend {
 public:
  explicit HardwareImuBackend(const RuntimeConfig&) {}
  void start() override { throw std::runtime_error("Hardware IMU backend unavailable on this platform/build"); }
  void stop() noexcept override {}
  ImuSample read_once(uint64_t) override { return ImuSample{}; }
};
#endif

class SimImuBackend final : public ImuBackend {
 public:
  void start() override {}
  void stop() noexcept override {}

  ImuSample read_once(uint64_t now_ns) override {
    const double t = static_cast<double>(now_ns) * 1e-9;
    ImuSample out;
    out.t_ns = now_ns;
    out.ax_mps2 = 0.6 * std::sin(0.8 * t);
    out.ay_mps2 = 0.5 * std::sin(1.1 * t + 0.3);
    out.az_mps2 = 9.80665 + 0.15 * std::cos(0.4 * t);
    out.gx_rads = 0.05 * std::sin(1.3 * t);
    out.gy_rads = 0.06 * std::cos(1.7 * t);
    out.gz_rads = 0.04 * std::sin(0.5 * t);
    out.valid = true;
    return out;
  }
};

#if RUNTIME_ENABLE_BARO_HW
class HardwareBaroBackend final : public BaroBackend {
 public:
  explicit HardwareBaroBackend(const RuntimeConfig& cfg)
      : sensor_(cfg.baro.i2c_bus, static_cast<uint8_t>(cfg.baro.i2c_address)) {}

  void start() override { sensor_.begin(); }

  void stop() noexcept override { sensor_.close(true); }

  BaroSample read_once(uint64_t now_ns) override {
    const auto reading = sensor_.read();
    BaroSample out;
    out.t_ns = now_ns;
    out.pressure_pa = reading.pressure_pa;
    out.temperature_c = reading.temperature_c;
    out.valid = true;
    return out;
  }

 private:
  bmp390::Bmp390 sensor_;
};
#else
class HardwareBaroBackend final : public BaroBackend {
 public:
  explicit HardwareBaroBackend(const RuntimeConfig&) {}
  void start() override { throw std::runtime_error("Hardware BARO backend unavailable on this platform/build"); }
  void stop() noexcept override {}
  BaroSample read_once(uint64_t) override { return BaroSample{}; }
};
#endif

class SimBaroBackend final : public BaroBackend {
 public:
  void start() override {}
  void stop() noexcept override {}

  BaroSample read_once(uint64_t now_ns) override {
    const double t = static_cast<double>(now_ns) * 1e-9;
    BaroSample out;
    out.t_ns = now_ns;
    out.pressure_pa = 101325.0 + 120.0 * std::sin(0.08 * t);
    out.temperature_c = 25.0 + 0.8 * std::sin(0.02 * t + 0.2);
    out.valid = true;
    return out;
  }
};

}  // namespace

std::unique_ptr<ImuBackend> make_imu_backend(const RuntimeConfig& cfg, bool sim_mode) {
  if (sim_mode) {
    return std::make_unique<SimImuBackend>();
  }
  return std::make_unique<HardwareImuBackend>(cfg);
}

std::unique_ptr<BaroBackend> make_baro_backend(const RuntimeConfig& cfg, bool sim_mode) {
  if (sim_mode) {
    return std::make_unique<SimBaroBackend>();
  }
  return std::make_unique<HardwareBaroBackend>(cfg);
}

}  // namespace runtime
