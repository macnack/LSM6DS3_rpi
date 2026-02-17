#include "runtime/config/config.hpp"

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <unistd.h>

namespace {

#define REQUIRE(cond, msg)      \
  do {                          \
    if (!(cond)) {              \
      std::cerr << msg << "\n"; \
      return false;             \
    }                           \
  } while (0)

std::string write_config_file(const std::string& suffix, const std::string& chip0,
                              const std::string& chip1, const std::string& chip2,
                              const std::string& chip3) {
  const std::string path = "/tmp/rt_igniter_cfg_" + std::to_string(::getpid()) + "_" + suffix + ".toml";
  std::ofstream out(path);
  out << "[runtime]\n"
      << "sim_mode = true\n"
      << "fail_fast_on_hw_error = true\n"
      << "allow_auto_sim_fallback = false\n"
      << "run_duration_s = 0\n"
      << "log_period_ms = 1000\n"
      << "\n[modes]\n"
      << "estimator_mode = \"cpp_native\"\n"
      << "controller_mode = \"cpp_native\"\n"
      << "\n[threads]\n"
      << "control_hz = 250\n"
      << "actuator_hz = 50\n"
      << "igniter_hz = 250\n"
      << "imu_hz = 500\n"
      << "estimator_hz = 250\n"
      << "baro_hz = 20\n"
      << "control_priority = 90\n"
      << "actuator_priority = 85\n"
      << "igniter_priority = 82\n"
      << "imu_priority = 80\n"
      << "estimator_priority = 60\n"
      << "i2c_priority = 50\n"
      << "control_cpu = -1\n"
      << "imu_cpu = -1\n"
      << "\n[timeouts]\n"
      << "controller_fresh_ms = 16\n"
      << "controller_hold_ms = 100\n"
      << "estimator_fresh_ms = 16\n"
      << "estimator_hold_ms = 120\n"
      << "actuator_cmd_timeout_ms = 60\n"
      << "imu_stale_ms = 20\n"
      << "baro_stale_ms = 300\n"
      << "max_consecutive_imu_failures = 5\n"
      << "max_consecutive_baro_failures = 5\n"
      << "\n[security]\n"
      << "require_local_ipc_permissions = false\n"
      << "require_loopback_sim_net = true\n"
      << "\n[ipc]\n"
      << "sensor_snapshot_shm = \"/rt_sensor_snapshot_v1\"\n"
      << "estimator_state_shm = \"/rt_estimator_state_v1\"\n"
      << "controller_command_shm = \"/rt_controller_command_v1\"\n"
      << "open_retry_ms = 50\n"
      << "open_retry_count = 200\n"
      << "\n[imu]\n"
      << "spi_device = \"/dev/spidev0.0\"\n"
      << "spi_speed_hz = 5000000\n"
      << "spi_mode = 3\n"
      << "accel_odr = \"104hz\"\n"
      << "gyro_odr = \"104hz\"\n"
      << "accel_scale = \"2g\"\n"
      << "gyro_scale = \"245dps\"\n"
      << "\n[baro]\n"
      << "i2c_bus = \"/dev/i2c-1\"\n"
      << "i2c_address = 0x77\n"
      << "recovery_error_threshold = 5\n"
      << "recovery_backoff_ms = 100\n"
      << "\n[actuator]\n"
      << "use_hardware = false\n"
      << "start_armed = false\n"
      << "latch_failsafe_on_io_errors = true\n"
      << "max_io_errors_before_latch = 3\n"
      << "frequency_hz = 50\n"
      << "slew_limit_norm_per_sec = 4.0\n"
      << "\n[igniter]\n"
      << "enabled = true\n"
      << "use_hardware = true\n"
      << "fault_policy = \"global\"\n"
      << "settle_ms = 5\n"
      << "latch_faults = true\n"
      << "default_fire_ms = 200\n"
      << "max_fire_ms = 2000\n"
      << "command_shm = \"/rt_igniter_command_v1\"\n"
      << "status_shm = \"/rt_igniter_status_v1\"\n"
      << "\n[igniter0]\n"
      << "enabled = true\n"
      << "input_chip = \"" << chip0 << "\"\n"
      << "input_line = 4\n"
      << "status_chip = \"" << chip0 << "\"\n"
      << "status_line = 17\n"
      << "\n[igniter1]\n"
      << "enabled = true\n"
      << "input_chip = \"" << chip1 << "\"\n"
      << "input_line = 5\n"
      << "status_chip = \"" << chip1 << "\"\n"
      << "status_line = 18\n"
      << "\n[igniter2]\n"
      << "enabled = true\n"
      << "input_chip = \"" << chip2 << "\"\n"
      << "input_line = 6\n"
      << "status_chip = \"" << chip2 << "\"\n"
      << "status_line = 19\n"
      << "\n[igniter3]\n"
      << "enabled = true\n"
      << "input_chip = \"" << chip3 << "\"\n"
      << "input_line = 7\n"
      << "status_chip = \"" << chip3 << "\"\n"
      << "status_line = 20\n"
      << "\n[servo0]\n"
      << "chip = 0\n"
      << "channel = 0\n"
      << "min_pwm_us = 1000\n"
      << "max_pwm_us = 2000\n"
      << "neutral_pwm_us = 1500\n"
      << "direction = 1.0\n"
      << "trim_norm = 0.0\n"
      << "min_norm = -1.0\n"
      << "max_norm = 1.0\n"
      << "failsafe_norm = 0.0\n"
      << "\n[servo1]\n"
      << "chip = 0\n"
      << "channel = 1\n"
      << "min_pwm_us = 1000\n"
      << "max_pwm_us = 2000\n"
      << "neutral_pwm_us = 1500\n"
      << "direction = 1.0\n"
      << "trim_norm = 0.0\n"
      << "min_norm = -1.0\n"
      << "max_norm = 1.0\n"
      << "failsafe_norm = 0.0\n"
      << "\n[servo2]\n"
      << "chip = 0\n"
      << "channel = 2\n"
      << "min_pwm_us = 1000\n"
      << "max_pwm_us = 2000\n"
      << "neutral_pwm_us = 1500\n"
      << "direction = 1.0\n"
      << "trim_norm = 0.0\n"
      << "min_norm = -1.0\n"
      << "max_norm = 1.0\n"
      << "failsafe_norm = 0.0\n"
      << "\n[servo3]\n"
      << "chip = 0\n"
      << "channel = 3\n"
      << "min_pwm_us = 1000\n"
      << "max_pwm_us = 2000\n"
      << "neutral_pwm_us = 1500\n"
      << "direction = 1.0\n"
      << "trim_norm = 0.0\n"
      << "min_norm = -1.0\n"
      << "max_norm = 1.0\n"
      << "failsafe_norm = 0.0\n"
      << "\n[estimator_cpp]\n"
      << "accel_complementary_gain = 0.02\n"
      << "baro_alt_gain = 0.1\n"
      << "\n[controller_cpp]\n"
      << "kp_roll = 1.2\n"
      << "kp_pitch = 1.2\n"
      << "kd_p = 0.15\n"
      << "max_deflection_norm = 0.8\n";
  return path;
}

bool test_same_chip_required() {
  const std::string path = write_config_file("bad", "/dev/gpiochip0", "/dev/gpiochip1",
                                             "/dev/gpiochip0", "/dev/gpiochip0");
  bool threw = false;
  try {
    (void)runtime::load_runtime_config(path);
  } catch (const std::exception&) {
    threw = true;
  }
  (void)::unlink(path.c_str());
  REQUIRE(threw, "Config must fail when igniter input chips differ");
  return true;
}

bool test_same_chip_ok() {
  const std::string path =
      write_config_file("ok", "/dev/gpiochip0", "/dev/gpiochip0", "/dev/gpiochip0", "/dev/gpiochip0");
  bool threw = false;
  try {
    (void)runtime::load_runtime_config(path);
  } catch (const std::exception&) {
    threw = true;
  }
  (void)::unlink(path.c_str());
  REQUIRE(!threw, "Config should accept igniter channels on one gpiochip");
  return true;
}

}  // namespace

int main() {
  if (!test_same_chip_required()) {
    return EXIT_FAILURE;
  }
  if (!test_same_chip_ok()) {
    return EXIT_FAILURE;
  }
  std::cout << "runtime_unit_igniter_config_validation: ok\n";
  return EXIT_SUCCESS;
}
