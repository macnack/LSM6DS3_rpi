#include "runtime/runtime/sensors.hpp"

#include <cmath>
#include <memory>
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
class HardwareImuBackend final : public ImuBackend {
 public:
  explicit HardwareImuBackend(const RuntimeConfig& cfg)
      : sensor_(cfg.imu.spi_device, cfg.imu.spi_speed_hz, static_cast<uint8_t>(cfg.imu.spi_mode)) {}

  void start() override { sensor_.begin(); }

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
  lsm6ds3::Lsm6ds3Spi sensor_;
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
