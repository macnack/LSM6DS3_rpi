#include "runtime/runtime/estimator.hpp"

#include <algorithm>
#include <cmath>

namespace runtime {

namespace {

std::array<float, 4> euler_to_quat(double roll, double pitch, double yaw) {
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);

  const double w = cr * cp * cy + sr * sp * sy;
  const double x = sr * cp * cy - cr * sp * sy;
  const double y = cr * sp * cy + sr * cp * sy;
  const double z = cr * cp * sy - sr * sp * cy;

  return {static_cast<float>(w), static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)};
}

double pressure_to_altitude_m(double pressure_pa) {
  if (pressure_pa <= 0.0) {
    return 0.0;
  }
  return 44330.0 * (1.0 - std::pow(pressure_pa / 101325.0, 0.1903));
}

}  // namespace

CppEstimator::CppEstimator(const EstimatorCppSection& cfg) : cfg_(cfg) {}

EstimatorState CppEstimator::step(const ImuSample& imu, const BaroSample& baro) {
  EstimatorState out{};

  if (!imu.valid) {
    out.t_ns = imu.t_ns;
    out.valid = false;
    return out;
  }

  double dt = 0.0;
  if (!initialized_) {
    initialized_ = true;
    last_t_ns_ = imu.t_ns;
  } else if (imu.t_ns > last_t_ns_) {
    dt = static_cast<double>(imu.t_ns - last_t_ns_) * 1e-9;
    last_t_ns_ = imu.t_ns;
  }

  roll_rad_ += imu.gx_rads * dt;
  pitch_rad_ += imu.gy_rads * dt;
  yaw_rad_ += imu.gz_rads * dt;

  const double accel_roll = std::atan2(imu.ay_mps2, imu.az_mps2);
  const double accel_pitch = std::atan2(-imu.ax_mps2,
                                        std::sqrt(imu.ay_mps2 * imu.ay_mps2 + imu.az_mps2 * imu.az_mps2));

  const double a = std::clamp(cfg_.accel_complementary_gain, 0.0, 1.0);
  roll_rad_ = (1.0 - a) * roll_rad_ + a * accel_roll;
  pitch_rad_ = (1.0 - a) * pitch_rad_ + a * accel_pitch;

  double alt_m = last_alt_m_;
  if (baro.valid) {
    const double measured_alt_m = pressure_to_altitude_m(baro.pressure_pa);
    if (!baro_alt_initialized_) {
      last_alt_m_ = measured_alt_m;
      baro_alt_initialized_ = true;
    }
    const double gain = std::clamp(cfg_.baro_alt_gain, 0.0, 1.0);
    last_alt_m_ = (1.0 - gain) * last_alt_m_ + gain * measured_alt_m;
    alt_m = last_alt_m_;
  }

  out.t_ns = imu.t_ns;
  out.q_body_to_ned = euler_to_quat(roll_rad_, pitch_rad_, yaw_rad_);
  out.vel_ned_mps = {0.0F, 0.0F, 0.0F};
  out.pos_ned_m = {0.0F, 0.0F, static_cast<float>(-alt_m)};
  out.valid = true;
  return out;
}

}  // namespace runtime
