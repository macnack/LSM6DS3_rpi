#include "runtime/runtime/controller.hpp"

#include <algorithm>
#include <cmath>

namespace runtime {

namespace {

constexpr double kPi = 3.14159265358979323846;

void quat_to_roll_pitch(const std::array<float, 4>& q, double& roll, double& pitch) {
  const double w = q[0];
  const double x = q[1];
  const double y = q[2];
  const double z = q[3];

  const double sinr_cosp = 2.0 * (w * x + y * z);
  const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
  roll = std::atan2(sinr_cosp, cosr_cosp);

  const double sinp = 2.0 * (w * y - z * x);
  if (std::abs(sinp) >= 1.0) {
    pitch = std::copysign(kPi / 2.0, sinp);
  } else {
    pitch = std::asin(sinp);
  }
}

}  // namespace

CppController::CppController(const ControllerCppSection& cfg) : cfg_(cfg) {}

ControlCommand CppController::step(const ImuSample& imu, const EstimatorState& est, bool armed,
                                   uint64_t now_ns) {
  ControlCommand out{};
  out.t_ns = now_ns;
  out.armed = armed;
  out.valid = true;

  double roll = 0.0;
  double pitch = 0.0;
  if (est.valid) {
    quat_to_roll_pitch(est.q_body_to_ned, roll, pitch);
  }

  const double roll_cmd = -(cfg_.kp_roll * roll) - (cfg_.kd_p * imu.gx_rads);
  const double pitch_cmd = -(cfg_.kp_pitch * pitch) - (cfg_.kd_p * imu.gy_rads);

  const double limit = std::max(0.0, cfg_.max_deflection_norm);
  const double mix0 = std::clamp(roll_cmd - pitch_cmd, -limit, limit);
  const double mix1 = std::clamp(-roll_cmd - pitch_cmd, -limit, limit);
  const double mix2 = std::clamp(roll_cmd + pitch_cmd, -limit, limit);
  const double mix3 = std::clamp(-roll_cmd + pitch_cmd, -limit, limit);

  out.servo_norm = {mix0, mix1, mix2, mix3};
  return out;
}

}  // namespace runtime
