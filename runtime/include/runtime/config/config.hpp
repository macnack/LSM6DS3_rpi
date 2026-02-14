#pragma once

#include "runtime/common/types.hpp"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace runtime {

struct RuntimeSection {
  bool sim_mode = true;
  bool auto_sim_on_hw_error = true;
  double run_duration_s = 0.0;
  uint32_t log_period_ms = 1000;
};

struct ModesSection {
  EstimatorMode estimator_mode = EstimatorMode::CppNative;
  ControllerMode controller_mode = ControllerMode::CppNative;
};

struct ThreadsSection {
  uint32_t control_hz = 250;
  uint32_t actuator_hz = 50;
  uint32_t imu_hz = 500;
  uint32_t estimator_hz = 250;
  uint32_t baro_hz = 20;

  int control_priority = 90;
  int actuator_priority = 85;
  int imu_priority = 80;
  int estimator_priority = 60;
  int i2c_priority = 50;

  int control_cpu = -1;
  int imu_cpu = -1;
};

struct TimeoutsSection {
  uint32_t controller_fresh_ms = 16;
  uint32_t controller_hold_ms = 100;
  uint32_t estimator_fresh_ms = 16;
  uint32_t estimator_hold_ms = 120;
  uint32_t actuator_cmd_timeout_ms = 60;
  uint32_t imu_stale_ms = 20;
  uint32_t baro_stale_ms = 300;
};

struct IpcSection {
  std::string sensor_snapshot_shm = "/rt_sensor_snapshot_v1";
  std::string estimator_state_shm = "/rt_estimator_state_v1";
  std::string controller_command_shm = "/rt_controller_command_v1";
  uint32_t open_retry_ms = 50;
  uint32_t open_retry_count = 200;
};

struct ImuSection {
  std::string spi_device = "/dev/spidev0.0";
  uint32_t spi_speed_hz = 5'000'000;
  uint32_t spi_mode = 3;
};

struct BaroSection {
  std::string i2c_bus = "/dev/i2c-1";
  uint32_t i2c_address = 0x77;
  uint32_t recovery_error_threshold = 5;
  uint32_t recovery_backoff_ms = 100;
};

struct KillSwitchSection {
  bool enabled = false;
  uint32_t gpio = 17;
  uint32_t nc_closed_value = 0;
  uint32_t debounce_samples = 2;
  bool latch_on_trip = true;
};

struct ServoSection {
  uint32_t chip = 0;
  uint32_t channel = 0;
  uint32_t min_pwm_us = 1000;
  uint32_t max_pwm_us = 2000;
  uint32_t neutral_pwm_us = 1500;
  double direction = 1.0;
  double trim_norm = 0.0;
  double min_norm = -1.0;
  double max_norm = 1.0;
  double failsafe_norm = 0.0;
};

struct ActuatorSection {
  bool use_hardware = false;
  bool start_armed = false;
  bool latch_failsafe_on_io_errors = true;
  uint32_t max_io_errors_before_latch = 3;
  uint32_t frequency_hz = 50;
  double slew_limit_norm_per_sec = 4.0;
};

struct EstimatorCppSection {
  double accel_complementary_gain = 0.02;
  double baro_alt_gain = 0.1;
};

struct ControllerCppSection {
  double kp_roll = 1.2;
  double kp_pitch = 1.2;
  double kd_p = 0.15;
  double max_deflection_norm = 0.8;
};

struct RuntimeConfig {
  RuntimeSection runtime;
  ModesSection modes;
  ThreadsSection threads;
  TimeoutsSection timeouts;
  IpcSection ipc;
  ImuSection imu;
  BaroSection baro;
  KillSwitchSection killswitch;
  ActuatorSection actuator;
  std::array<ServoSection, kServoCount> servos{};
  EstimatorCppSection estimator_cpp;
  ControllerCppSection controller_cpp;
};

RuntimeConfig load_runtime_config(const std::string& path);

std::string estimator_mode_to_string(EstimatorMode mode);
std::string controller_mode_to_string(ControllerMode mode);
EstimatorMode parse_estimator_mode(const std::string& value);
ControllerMode parse_controller_mode(const std::string& value);

std::string runtime_config_to_string(const RuntimeConfig& cfg);

}  // namespace runtime
