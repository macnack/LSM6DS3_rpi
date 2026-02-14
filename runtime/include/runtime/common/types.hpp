#pragma once

#include "runtime/common/constants.hpp"

#include <array>
#include <cstdint>

namespace runtime {

enum class EstimatorMode {
  CppNative,
  PythonDev,
  CppDev,
};

enum class ControllerMode {
  CppNative,
  PythonDev,
  CppDev,
};

struct ImuSample {
  uint64_t t_ns = 0;
  double ax_mps2 = 0.0;
  double ay_mps2 = 0.0;
  double az_mps2 = 9.80665;
  double gx_rads = 0.0;
  double gy_rads = 0.0;
  double gz_rads = 0.0;
  bool valid = false;
};

struct BaroSample {
  uint64_t t_ns = 0;
  double pressure_pa = 101325.0;
  double temperature_c = 20.0;
  bool valid = false;
};

struct SensorHealth {
  uint64_t imu_read_errors = 0;
  uint64_t i2c_read_errors = 0;
  uint64_t queue_drops = 0;
  bool imu_stale = false;
  bool baro_stale = false;
};

struct EstimatorState {
  uint64_t t_ns = 0;
  std::array<float, 4> q_body_to_ned{1.0F, 0.0F, 0.0F, 0.0F};
  std::array<float, 3> vel_ned_mps{0.0F, 0.0F, 0.0F};
  std::array<float, 3> pos_ned_m{0.0F, 0.0F, 0.0F};
  bool valid = false;
};

struct ControlCommand {
  uint64_t t_ns = 0;
  bool armed = false;
  std::array<double, kServoCount> servo_norm{0.0, 0.0, 0.0, 0.0};
  bool valid = false;
};

struct ActuatorHealth {
  uint64_t cmd_missed_deadlines = 0;
  uint64_t io_errors = 0;
  bool failsafe_active = false;
};

struct RuntimeStats {
  ActuatorHealth actuator_health{};
  uint64_t control_ticks = 0;
  uint64_t actuator_ticks = 0;
  uint64_t imu_ticks = 0;
  uint64_t i2c_ticks = 0;
  uint64_t estimator_ticks = 0;

  uint64_t python_estimator_accept_count = 0;
  uint64_t python_estimator_reject_count = 0;
  uint64_t python_controller_accept_count = 0;
  uint64_t python_controller_reject_count = 0;

  uint64_t failsafe_activation_count = 0;

  uint64_t control_deadline_miss_count = 0;
  uint64_t actuator_deadline_miss_count = 0;
  uint64_t imu_deadline_miss_count = 0;
  uint64_t baro_deadline_miss_count = 0;

  uint64_t control_jitter_p50_ns = 0;
  uint64_t control_jitter_p95_ns = 0;
  uint64_t control_jitter_p99_ns = 0;
  uint64_t control_jitter_max_ns = 0;

  uint64_t actuator_jitter_p50_ns = 0;
  uint64_t actuator_jitter_p95_ns = 0;
  uint64_t actuator_jitter_p99_ns = 0;
  uint64_t actuator_jitter_max_ns = 0;

  uint64_t imu_jitter_p50_ns = 0;
  uint64_t imu_jitter_p95_ns = 0;
  uint64_t imu_jitter_p99_ns = 0;
  uint64_t imu_jitter_max_ns = 0;

  uint64_t baro_jitter_p50_ns = 0;
  uint64_t baro_jitter_p95_ns = 0;
  uint64_t baro_jitter_p99_ns = 0;
  uint64_t baro_jitter_max_ns = 0;

  uint64_t actuator_cmd_age_last_ns = 0;
  uint64_t actuator_cmd_age_max_ns = 0;

  uint64_t i2c_recovery_count = 0;

  bool killswitch_active = false;
  uint64_t killswitch_trip_count = 0;
};

}  // namespace runtime
