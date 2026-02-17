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

enum class FailsafeReason : uint32_t {
  None = 0,
  CommandTimeout = 1,
  ImuStale = 2,
  IoErrorLatch = 3,
  KillSwitch = 4,
  InvalidEstimator = 5,
  LinkDown = 6,
};

struct RuntimeStats {
  ActuatorHealth actuator_health{};
  uint64_t control_ticks = 0;
  uint64_t actuator_ticks = 0;
  uint64_t igniter_ticks = 0;
  uint64_t imu_ticks = 0;
  uint64_t i2c_ticks = 0;
  uint64_t estimator_ticks = 0;

  uint64_t external_estimator_accept_count = 0;
  uint64_t external_estimator_reject_count = 0;
  uint64_t external_controller_accept_count = 0;
  uint64_t external_controller_reject_count = 0;
  uint64_t igniter_command_accept_count = 0;
  uint64_t igniter_command_reject_count = 0;

  uint64_t failsafe_activation_count = 0;
  uint64_t failsafe_enter_count = 0;
  uint64_t failsafe_exit_count = 0;
  uint64_t failsafe_cause_cmd_stale_count = 0;
  uint64_t failsafe_cause_link_down_count = 0;
  uint64_t failsafe_cause_imu_stale_count = 0;
  uint64_t failsafe_cause_killswitch_count = 0;
  uint64_t failsafe_cause_actuator_io_latch_count = 0;

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
  uint64_t imu_watchdog_fault_count = 0;
  uint64_t imu_watchdog_zero_vector_count = 0;
  uint64_t imu_watchdog_flatline_count = 0;
  uint64_t imu_watchdog_degenerate_pattern_count = 0;
  uint64_t imu_reinit_attempt_count = 0;
  uint64_t imu_reinit_success_count = 0;
  uint64_t imu_reinit_failure_count = 0;
  uint32_t imu_last_fault_reason = 0;
  uint32_t imu_watchdog_state = 0;

  bool killswitch_active = false;
  uint64_t killswitch_trip_count = 0;
  bool igniter_armed = false;
  bool igniter_global_fault_latched = false;
  uint8_t igniter_active_mask = 0;

  // sim_net bridge
  uint64_t sim_net_sensor_frames = 0;
  uint64_t sim_net_sensor_crc_fail = 0;
  uint64_t sim_net_sensor_disconnects = 0;
  uint64_t sim_net_actuator_frames = 0;
  uint64_t sim_net_actuator_send_errors = 0;
  uint64_t sim_net_actuator_clients = 0;
  uint64_t sim_net_actuator_disconnects = 0;
  bool sim_net_actuator_client_connected = false;

  bool degraded_mode_active = false;
  uint32_t last_failsafe_reason = static_cast<uint32_t>(FailsafeReason::None);
  uint64_t imu_consecutive_failures = 0;
  uint64_t baro_consecutive_failures = 0;
  uint64_t ipc_permission_reject_count = 0;
  uint64_t external_msg_stale_reject_count = 0;
  uint64_t external_msg_schema_reject_count = 0;
  uint64_t external_msg_crc_reject_count = 0;
  uint64_t external_msg_time_regression_reject_count = 0;
};

}  // namespace runtime
