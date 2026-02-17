#include "runtime/config/config.hpp"

#include "runtime/config/toml_parser.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

namespace runtime {

namespace {

uint32_t as_u32(const TomlValue& value, const std::string& key_name) {
  if (value.is_int()) {
    const int64_t x = value.as_int();
    if (x < 0 || x > static_cast<int64_t>(UINT32_MAX)) {
      throw std::runtime_error("Config key '" + key_name + "' is out of uint32 range");
    }
    return static_cast<uint32_t>(x);
  }
  if (value.is_double()) {
    const double x = value.as_double();
    if (!std::isfinite(x) || x < 0.0 || x > static_cast<double>(UINT32_MAX) || std::floor(x) != x) {
      throw std::runtime_error("Config key '" + key_name + "' must be an integer-compatible number");
    }
    return static_cast<uint32_t>(x);
  }
  throw std::runtime_error("Config key '" + key_name + "' must be integer");
}

int as_i32(const TomlValue& value, const std::string& key_name) {
  if (value.is_int()) {
    const int64_t x = value.as_int();
    if (x < static_cast<int64_t>(INT32_MIN) || x > static_cast<int64_t>(INT32_MAX)) {
      throw std::runtime_error("Config key '" + key_name + "' is out of int32 range");
    }
    return static_cast<int>(x);
  }
  if (value.is_double()) {
    const double x = value.as_double();
    if (!std::isfinite(x) || std::floor(x) != x || x < static_cast<double>(INT32_MIN) ||
        x > static_cast<double>(INT32_MAX)) {
      throw std::runtime_error("Config key '" + key_name + "' must be integer-compatible number");
    }
    return static_cast<int>(x);
  }
  throw std::runtime_error("Config key '" + key_name + "' must be integer");
}

double as_double(const TomlValue& value, const std::string& key_name) {
  if (value.is_double() || value.is_int()) {
    return value.as_double();
  }
  throw std::runtime_error("Config key '" + key_name + "' must be numeric");
}

bool as_bool(const TomlValue& value, const std::string& key_name) {
  if (!value.is_bool()) {
    throw std::runtime_error("Config key '" + key_name + "' must be boolean");
  }
  return value.as_bool();
}

std::string as_string(const TomlValue& value, const std::string& key_name) {
  if (!value.is_string()) {
    throw std::runtime_error("Config key '" + key_name + "' must be string");
  }
  return value.as_string();
}

const TomlSection& require_section(const TomlDocument& doc, const std::string& name) {
  const auto it = doc.find(name);
  if (it == doc.end()) {
    throw std::runtime_error("Missing required section [" + name + "]");
  }
  return it->second;
}

void validate_allowed_keys(const TomlSection& section, const std::unordered_set<std::string>& allowed,
                           const std::string& section_name) {
  for (const auto& [key, _] : section) {
    if (allowed.find(key) == allowed.end()) {
      throw std::runtime_error("Unknown key '" + key + "' in section [" + section_name + "]");
    }
  }
}

void validate_allowed_sections(const TomlDocument& doc,
                               const std::unordered_set<std::string>& allowed_sections) {
  for (const auto& [section, _] : doc) {
    if (allowed_sections.find(section) == allowed_sections.end()) {
      throw std::runtime_error("Unknown section [" + section + "]");
    }
  }
}

template <typename Fn>
void maybe_apply(const TomlSection& section, const std::string& key, Fn&& fn) {
  const auto it = section.find(key);
  if (it != section.end()) {
    fn(it->second);
  }
}

}  // namespace

std::string estimator_mode_to_string(EstimatorMode mode) {
  switch (mode) {
    case EstimatorMode::CppNative:
      return "cpp_native";
    case EstimatorMode::PythonDev:
      return "python_dev";
    case EstimatorMode::CppDev:
      return "cpp_dev";
  }
  return "cpp_native";
}

std::string controller_mode_to_string(ControllerMode mode) {
  switch (mode) {
    case ControllerMode::CppNative:
      return "cpp_native";
    case ControllerMode::PythonDev:
      return "python_dev";
    case ControllerMode::CppDev:
      return "cpp_dev";
  }
  return "cpp_native";
}

EstimatorMode parse_estimator_mode(const std::string& value) {
  if (value == "cpp_native") {
    return EstimatorMode::CppNative;
  }
  if (value == "python_dev") {
    return EstimatorMode::PythonDev;
  }
  if (value == "cpp_dev") {
    return EstimatorMode::CppDev;
  }
  throw std::runtime_error("Invalid estimator_mode: " + value + " (expected cpp_native|python_dev|cpp_dev)");
}

ControllerMode parse_controller_mode(const std::string& value) {
  if (value == "cpp_native") {
    return ControllerMode::CppNative;
  }
  if (value == "python_dev") {
    return ControllerMode::PythonDev;
  }
  if (value == "cpp_dev") {
    return ControllerMode::CppDev;
  }
  throw std::runtime_error("Invalid controller_mode: " + value + " (expected cpp_native|python_dev|cpp_dev)");
}

RuntimeConfig load_runtime_config(const std::string& path) {
  const TomlDocument doc = parse_toml_file(path);

  const std::unordered_set<std::string> required_sections = {
      "runtime",      "modes",     "threads",       "timeouts",   "security",
      "ipc",
      "imu",          "baro",      "actuator",      "servo0",     "servo1",
      "servo2",       "servo3",    "estimator_cpp", "controller_cpp",
  };

  std::unordered_set<std::string> allowed_sections = required_sections;
  allowed_sections.insert("killswitch");
  allowed_sections.insert("sim_net");
  allowed_sections.insert("imu_watchdog");
  allowed_sections.insert("igniter");
  allowed_sections.insert("igniter0");
  allowed_sections.insert("igniter1");
  allowed_sections.insert("igniter2");
  allowed_sections.insert("igniter3");

  validate_allowed_sections(doc, allowed_sections);

  for (const auto& section : required_sections) {
    (void)require_section(doc, section);
  }

  RuntimeConfig cfg;

  const auto& runtime = require_section(doc, "runtime");
  validate_allowed_keys(runtime, {"sim_mode", "fail_fast_on_hw_error", "allow_auto_sim_fallback",
                                  "run_duration_s", "log_period_ms"},
                        "runtime");
  maybe_apply(runtime, "sim_mode", [&](const TomlValue& v) { cfg.runtime.sim_mode = as_bool(v, "runtime.sim_mode"); });
  maybe_apply(runtime, "fail_fast_on_hw_error",
              [&](const TomlValue& v) { cfg.runtime.fail_fast_on_hw_error = as_bool(v, "runtime.fail_fast_on_hw_error"); });
  maybe_apply(runtime, "allow_auto_sim_fallback", [&](const TomlValue& v) {
    cfg.runtime.allow_auto_sim_fallback = as_bool(v, "runtime.allow_auto_sim_fallback");
  });
  maybe_apply(runtime, "run_duration_s",
              [&](const TomlValue& v) { cfg.runtime.run_duration_s = as_double(v, "runtime.run_duration_s"); });
  maybe_apply(runtime, "log_period_ms",
              [&](const TomlValue& v) { cfg.runtime.log_period_ms = as_u32(v, "runtime.log_period_ms"); });

  const auto& modes = require_section(doc, "modes");
  validate_allowed_keys(modes, {"estimator_mode", "controller_mode"}, "modes");
  maybe_apply(modes, "estimator_mode",
              [&](const TomlValue& v) { cfg.modes.estimator_mode = parse_estimator_mode(as_string(v, "modes.estimator_mode")); });
  maybe_apply(modes, "controller_mode",
              [&](const TomlValue& v) { cfg.modes.controller_mode = parse_controller_mode(as_string(v, "modes.controller_mode")); });

  const auto& threads = require_section(doc, "threads");
  validate_allowed_keys(threads,
                        {"control_hz", "actuator_hz", "igniter_hz", "imu_hz", "estimator_hz", "baro_hz",
                         "control_priority", "actuator_priority", "igniter_priority", "imu_priority",
                         "estimator_priority", "i2c_priority", "control_cpu", "imu_cpu"},
                        "threads");
  maybe_apply(threads, "control_hz", [&](const TomlValue& v) { cfg.threads.control_hz = as_u32(v, "threads.control_hz"); });
  maybe_apply(threads, "actuator_hz", [&](const TomlValue& v) { cfg.threads.actuator_hz = as_u32(v, "threads.actuator_hz"); });
  maybe_apply(threads, "igniter_hz", [&](const TomlValue& v) { cfg.threads.igniter_hz = as_u32(v, "threads.igniter_hz"); });
  maybe_apply(threads, "imu_hz", [&](const TomlValue& v) { cfg.threads.imu_hz = as_u32(v, "threads.imu_hz"); });
  maybe_apply(threads, "estimator_hz", [&](const TomlValue& v) { cfg.threads.estimator_hz = as_u32(v, "threads.estimator_hz"); });
  maybe_apply(threads, "baro_hz", [&](const TomlValue& v) { cfg.threads.baro_hz = as_u32(v, "threads.baro_hz"); });
  maybe_apply(threads, "control_priority", [&](const TomlValue& v) { cfg.threads.control_priority = as_i32(v, "threads.control_priority"); });
  maybe_apply(threads, "actuator_priority", [&](const TomlValue& v) { cfg.threads.actuator_priority = as_i32(v, "threads.actuator_priority"); });
  maybe_apply(threads, "igniter_priority", [&](const TomlValue& v) {
    cfg.threads.igniter_priority = as_i32(v, "threads.igniter_priority");
  });
  maybe_apply(threads, "imu_priority", [&](const TomlValue& v) { cfg.threads.imu_priority = as_i32(v, "threads.imu_priority"); });
  maybe_apply(threads, "estimator_priority", [&](const TomlValue& v) { cfg.threads.estimator_priority = as_i32(v, "threads.estimator_priority"); });
  maybe_apply(threads, "i2c_priority", [&](const TomlValue& v) { cfg.threads.i2c_priority = as_i32(v, "threads.i2c_priority"); });
  maybe_apply(threads, "control_cpu", [&](const TomlValue& v) { cfg.threads.control_cpu = as_i32(v, "threads.control_cpu"); });
  maybe_apply(threads, "imu_cpu", [&](const TomlValue& v) { cfg.threads.imu_cpu = as_i32(v, "threads.imu_cpu"); });

  const auto& timeouts = require_section(doc, "timeouts");
  validate_allowed_keys(timeouts,
                        {"controller_fresh_ms", "controller_hold_ms", "estimator_fresh_ms", "estimator_hold_ms",
                         "actuator_cmd_timeout_ms", "imu_stale_ms", "baro_stale_ms",
                         "max_consecutive_imu_failures", "max_consecutive_baro_failures"},
                        "timeouts");
  maybe_apply(timeouts, "controller_fresh_ms",
              [&](const TomlValue& v) { cfg.timeouts.controller_fresh_ms = as_u32(v, "timeouts.controller_fresh_ms"); });
  maybe_apply(timeouts, "controller_hold_ms",
              [&](const TomlValue& v) { cfg.timeouts.controller_hold_ms = as_u32(v, "timeouts.controller_hold_ms"); });
  maybe_apply(timeouts, "estimator_fresh_ms",
              [&](const TomlValue& v) { cfg.timeouts.estimator_fresh_ms = as_u32(v, "timeouts.estimator_fresh_ms"); });
  maybe_apply(timeouts, "estimator_hold_ms",
              [&](const TomlValue& v) { cfg.timeouts.estimator_hold_ms = as_u32(v, "timeouts.estimator_hold_ms"); });
  maybe_apply(timeouts, "actuator_cmd_timeout_ms", [&](const TomlValue& v) {
    cfg.timeouts.actuator_cmd_timeout_ms = as_u32(v, "timeouts.actuator_cmd_timeout_ms");
  });
  maybe_apply(timeouts, "imu_stale_ms", [&](const TomlValue& v) { cfg.timeouts.imu_stale_ms = as_u32(v, "timeouts.imu_stale_ms"); });
  maybe_apply(timeouts, "baro_stale_ms", [&](const TomlValue& v) { cfg.timeouts.baro_stale_ms = as_u32(v, "timeouts.baro_stale_ms"); });
  maybe_apply(timeouts, "max_consecutive_imu_failures", [&](const TomlValue& v) {
    cfg.timeouts.max_consecutive_imu_failures = as_u32(v, "timeouts.max_consecutive_imu_failures");
  });
  maybe_apply(timeouts, "max_consecutive_baro_failures", [&](const TomlValue& v) {
    cfg.timeouts.max_consecutive_baro_failures = as_u32(v, "timeouts.max_consecutive_baro_failures");
  });

  const auto& security = require_section(doc, "security");
  validate_allowed_keys(security, {"require_local_ipc_permissions", "require_loopback_sim_net"},
                        "security");
  maybe_apply(security, "require_local_ipc_permissions", [&](const TomlValue& v) {
    cfg.security.require_local_ipc_permissions = as_bool(v, "security.require_local_ipc_permissions");
  });
  maybe_apply(security, "require_loopback_sim_net", [&](const TomlValue& v) {
    cfg.security.require_loopback_sim_net = as_bool(v, "security.require_loopback_sim_net");
  });

  const auto& ipc = require_section(doc, "ipc");
  validate_allowed_keys(ipc,
                        {"sensor_snapshot_shm", "estimator_state_shm", "controller_command_shm", "open_retry_ms",
                         "open_retry_count"},
                        "ipc");
  maybe_apply(ipc, "sensor_snapshot_shm",
              [&](const TomlValue& v) { cfg.ipc.sensor_snapshot_shm = as_string(v, "ipc.sensor_snapshot_shm"); });
  maybe_apply(ipc, "estimator_state_shm",
              [&](const TomlValue& v) { cfg.ipc.estimator_state_shm = as_string(v, "ipc.estimator_state_shm"); });
  maybe_apply(ipc, "controller_command_shm", [&](const TomlValue& v) {
    cfg.ipc.controller_command_shm = as_string(v, "ipc.controller_command_shm");
  });
  maybe_apply(ipc, "open_retry_ms", [&](const TomlValue& v) { cfg.ipc.open_retry_ms = as_u32(v, "ipc.open_retry_ms"); });
  maybe_apply(ipc, "open_retry_count", [&](const TomlValue& v) {
    cfg.ipc.open_retry_count = as_u32(v, "ipc.open_retry_count");
  });

  const auto& imu = require_section(doc, "imu");
  validate_allowed_keys(imu, {"spi_device", "spi_speed_hz", "spi_mode", "accel_odr", "gyro_odr",
                              "accel_scale", "gyro_scale"},
                        "imu");
  maybe_apply(imu, "spi_device", [&](const TomlValue& v) { cfg.imu.spi_device = as_string(v, "imu.spi_device"); });
  maybe_apply(imu, "spi_speed_hz", [&](const TomlValue& v) { cfg.imu.spi_speed_hz = as_u32(v, "imu.spi_speed_hz"); });
  maybe_apply(imu, "spi_mode", [&](const TomlValue& v) { cfg.imu.spi_mode = as_u32(v, "imu.spi_mode"); });
  maybe_apply(imu, "accel_odr", [&](const TomlValue& v) { cfg.imu.accel_odr = as_string(v, "imu.accel_odr"); });
  maybe_apply(imu, "gyro_odr", [&](const TomlValue& v) { cfg.imu.gyro_odr = as_string(v, "imu.gyro_odr"); });
  maybe_apply(imu, "accel_scale",
              [&](const TomlValue& v) { cfg.imu.accel_scale = as_string(v, "imu.accel_scale"); });
  maybe_apply(imu, "gyro_scale", [&](const TomlValue& v) { cfg.imu.gyro_scale = as_string(v, "imu.gyro_scale"); });

  const auto imu_watchdog_it = doc.find("imu_watchdog");
  if (imu_watchdog_it != doc.end()) {
    const auto& imu_watchdog = imu_watchdog_it->second;
    validate_allowed_keys(imu_watchdog,
                          {"enabled", "zero_vector_consecutive", "flatline_consecutive",
                           "degenerate_pattern_consecutive", "flatline_epsilon",
                           "healthy_recovery_samples", "recovery_backoff_ms", "max_reinit_attempts"},
                          "imu_watchdog");
    maybe_apply(imu_watchdog, "enabled",
                [&](const TomlValue& v) { cfg.imu_watchdog.enabled = as_bool(v, "imu_watchdog.enabled"); });
    maybe_apply(imu_watchdog, "zero_vector_consecutive", [&](const TomlValue& v) {
      cfg.imu_watchdog.zero_vector_consecutive = as_u32(v, "imu_watchdog.zero_vector_consecutive");
    });
    maybe_apply(imu_watchdog, "flatline_consecutive", [&](const TomlValue& v) {
      cfg.imu_watchdog.flatline_consecutive = as_u32(v, "imu_watchdog.flatline_consecutive");
    });
    maybe_apply(imu_watchdog, "degenerate_pattern_consecutive", [&](const TomlValue& v) {
      cfg.imu_watchdog.degenerate_pattern_consecutive =
          as_u32(v, "imu_watchdog.degenerate_pattern_consecutive");
    });
    maybe_apply(imu_watchdog, "flatline_epsilon", [&](const TomlValue& v) {
      cfg.imu_watchdog.flatline_epsilon = as_double(v, "imu_watchdog.flatline_epsilon");
    });
    maybe_apply(imu_watchdog, "healthy_recovery_samples", [&](const TomlValue& v) {
      cfg.imu_watchdog.healthy_recovery_samples = as_u32(v, "imu_watchdog.healthy_recovery_samples");
    });
    maybe_apply(imu_watchdog, "recovery_backoff_ms", [&](const TomlValue& v) {
      cfg.imu_watchdog.recovery_backoff_ms = as_u32(v, "imu_watchdog.recovery_backoff_ms");
    });
    maybe_apply(imu_watchdog, "max_reinit_attempts", [&](const TomlValue& v) {
      cfg.imu_watchdog.max_reinit_attempts = as_u32(v, "imu_watchdog.max_reinit_attempts");
    });
  }

  const auto& baro = require_section(doc, "baro");
  validate_allowed_keys(baro, {"i2c_bus", "i2c_address", "recovery_error_threshold", "recovery_backoff_ms"},
                        "baro");
  maybe_apply(baro, "i2c_bus", [&](const TomlValue& v) { cfg.baro.i2c_bus = as_string(v, "baro.i2c_bus"); });
  maybe_apply(baro, "i2c_address", [&](const TomlValue& v) { cfg.baro.i2c_address = as_u32(v, "baro.i2c_address"); });
  maybe_apply(baro, "recovery_error_threshold", [&](const TomlValue& v) {
    cfg.baro.recovery_error_threshold = as_u32(v, "baro.recovery_error_threshold");
  });
  maybe_apply(baro, "recovery_backoff_ms", [&](const TomlValue& v) {
    cfg.baro.recovery_backoff_ms = as_u32(v, "baro.recovery_backoff_ms");
  });

  const auto ks_it = doc.find("killswitch");
  if (ks_it != doc.end()) {
    const auto& ks = ks_it->second;
    validate_allowed_keys(ks, {"enabled", "gpio", "nc_closed_value", "debounce_samples", "latch_on_trip"},
                          "killswitch");
    maybe_apply(ks, "enabled",
                [&](const TomlValue& v) { cfg.killswitch.enabled = as_bool(v, "killswitch.enabled"); });
    maybe_apply(ks, "gpio", [&](const TomlValue& v) { cfg.killswitch.gpio = as_u32(v, "killswitch.gpio"); });
    maybe_apply(ks, "nc_closed_value", [&](const TomlValue& v) {
      cfg.killswitch.nc_closed_value = as_u32(v, "killswitch.nc_closed_value");
    });
    maybe_apply(ks, "debounce_samples", [&](const TomlValue& v) {
      cfg.killswitch.debounce_samples = as_u32(v, "killswitch.debounce_samples");
    });
    maybe_apply(ks, "latch_on_trip", [&](const TomlValue& v) {
      cfg.killswitch.latch_on_trip = as_bool(v, "killswitch.latch_on_trip");
    });
  }

  const auto& actuator = require_section(doc, "actuator");
  validate_allowed_keys(actuator,
                        {"use_hardware", "start_armed", "latch_failsafe_on_io_errors", "max_io_errors_before_latch",
                         "frequency_hz", "slew_limit_norm_per_sec"},
                        "actuator");
  maybe_apply(actuator, "use_hardware", [&](const TomlValue& v) { cfg.actuator.use_hardware = as_bool(v, "actuator.use_hardware"); });
  maybe_apply(actuator, "start_armed", [&](const TomlValue& v) { cfg.actuator.start_armed = as_bool(v, "actuator.start_armed"); });
  maybe_apply(actuator, "latch_failsafe_on_io_errors", [&](const TomlValue& v) {
    cfg.actuator.latch_failsafe_on_io_errors = as_bool(v, "actuator.latch_failsafe_on_io_errors");
  });
  maybe_apply(actuator, "max_io_errors_before_latch", [&](const TomlValue& v) {
    cfg.actuator.max_io_errors_before_latch = as_u32(v, "actuator.max_io_errors_before_latch");
  });
  maybe_apply(actuator, "frequency_hz", [&](const TomlValue& v) { cfg.actuator.frequency_hz = as_u32(v, "actuator.frequency_hz"); });
  maybe_apply(actuator, "slew_limit_norm_per_sec", [&](const TomlValue& v) {
    cfg.actuator.slew_limit_norm_per_sec = as_double(v, "actuator.slew_limit_norm_per_sec");
  });

  const auto igniter_it = doc.find("igniter");
  if (igniter_it != doc.end()) {
    const auto& igniter = igniter_it->second;
    validate_allowed_keys(igniter,
                          {"enabled", "use_hardware", "fault_policy", "settle_ms", "latch_faults",
                           "default_fire_ms", "max_fire_ms", "command_shm", "status_shm"},
                          "igniter");
    maybe_apply(igniter, "enabled", [&](const TomlValue& v) { cfg.igniter.enabled = as_bool(v, "igniter.enabled"); });
    maybe_apply(igniter, "use_hardware",
                [&](const TomlValue& v) { cfg.igniter.use_hardware = as_bool(v, "igniter.use_hardware"); });
    maybe_apply(igniter, "fault_policy",
                [&](const TomlValue& v) { cfg.igniter.fault_policy = as_string(v, "igniter.fault_policy"); });
    maybe_apply(igniter, "settle_ms", [&](const TomlValue& v) { cfg.igniter.settle_ms = as_u32(v, "igniter.settle_ms"); });
    maybe_apply(igniter, "latch_faults",
                [&](const TomlValue& v) { cfg.igniter.latch_faults = as_bool(v, "igniter.latch_faults"); });
    maybe_apply(igniter, "default_fire_ms", [&](const TomlValue& v) {
      cfg.igniter.default_fire_ms = as_u32(v, "igniter.default_fire_ms");
    });
    maybe_apply(igniter, "max_fire_ms",
                [&](const TomlValue& v) { cfg.igniter.max_fire_ms = as_u32(v, "igniter.max_fire_ms"); });
    maybe_apply(igniter, "command_shm",
                [&](const TomlValue& v) { cfg.igniter.command_shm = as_string(v, "igniter.command_shm"); });
    maybe_apply(igniter, "status_shm",
                [&](const TomlValue& v) { cfg.igniter.status_shm = as_string(v, "igniter.status_shm"); });
  }

  for (std::size_t i = 0; i < cfg.igniter_channels.size(); ++i) {
    const std::string sec_name = "igniter" + std::to_string(i);
    const auto sec_it = doc.find(sec_name);
    if (sec_it == doc.end()) {
      continue;
    }
    const auto& sec = sec_it->second;
    validate_allowed_keys(sec, {"enabled", "input_chip", "input_line", "status_chip", "status_line"}, sec_name);
    auto& out = cfg.igniter_channels[i];
    maybe_apply(sec, "enabled", [&](const TomlValue& v) { out.enabled = as_bool(v, sec_name + ".enabled"); });
    maybe_apply(sec, "input_chip", [&](const TomlValue& v) { out.input_chip = as_string(v, sec_name + ".input_chip"); });
    maybe_apply(sec, "input_line", [&](const TomlValue& v) { out.input_line = as_u32(v, sec_name + ".input_line"); });
    maybe_apply(sec, "status_chip",
                [&](const TomlValue& v) { out.status_chip = as_string(v, sec_name + ".status_chip"); });
    maybe_apply(sec, "status_line", [&](const TomlValue& v) { out.status_line = as_u32(v, sec_name + ".status_line"); });
  }

  for (std::size_t i = 0; i < cfg.servos.size(); ++i) {
    const std::string sec_name = "servo" + std::to_string(i);
    const auto& sec = require_section(doc, sec_name);
    validate_allowed_keys(sec,
                          {"chip", "channel", "min_pwm_us", "max_pwm_us", "neutral_pwm_us", "direction", "trim_norm",
                           "min_norm", "max_norm", "failsafe_norm"},
                          sec_name);
    auto& out = cfg.servos[i];
    maybe_apply(sec, "chip", [&](const TomlValue& v) { out.chip = as_u32(v, sec_name + ".chip"); });
    maybe_apply(sec, "channel", [&](const TomlValue& v) { out.channel = as_u32(v, sec_name + ".channel"); });
    maybe_apply(sec, "min_pwm_us", [&](const TomlValue& v) { out.min_pwm_us = as_u32(v, sec_name + ".min_pwm_us"); });
    maybe_apply(sec, "max_pwm_us", [&](const TomlValue& v) { out.max_pwm_us = as_u32(v, sec_name + ".max_pwm_us"); });
    maybe_apply(sec, "neutral_pwm_us", [&](const TomlValue& v) {
      out.neutral_pwm_us = as_u32(v, sec_name + ".neutral_pwm_us");
    });
    maybe_apply(sec, "direction", [&](const TomlValue& v) { out.direction = as_double(v, sec_name + ".direction"); });
    maybe_apply(sec, "trim_norm", [&](const TomlValue& v) { out.trim_norm = as_double(v, sec_name + ".trim_norm"); });
    maybe_apply(sec, "min_norm", [&](const TomlValue& v) { out.min_norm = as_double(v, sec_name + ".min_norm"); });
    maybe_apply(sec, "max_norm", [&](const TomlValue& v) { out.max_norm = as_double(v, sec_name + ".max_norm"); });
    maybe_apply(sec, "failsafe_norm",
                [&](const TomlValue& v) { out.failsafe_norm = as_double(v, sec_name + ".failsafe_norm"); });

    if (out.min_pwm_us >= out.max_pwm_us) {
      throw std::runtime_error(sec_name + ": min_pwm_us must be < max_pwm_us");
    }
    if (out.neutral_pwm_us < out.min_pwm_us || out.neutral_pwm_us > out.max_pwm_us) {
      throw std::runtime_error(sec_name + ": neutral_pwm_us must be within [min_pwm_us, max_pwm_us]");
    }
    if (out.min_norm >= out.max_norm) {
      throw std::runtime_error(sec_name + ": min_norm must be < max_norm");
    }
  }

  const auto& estimator_cpp = require_section(doc, "estimator_cpp");
  validate_allowed_keys(estimator_cpp, {"accel_complementary_gain", "baro_alt_gain"}, "estimator_cpp");
  maybe_apply(estimator_cpp, "accel_complementary_gain", [&](const TomlValue& v) {
    cfg.estimator_cpp.accel_complementary_gain = as_double(v, "estimator_cpp.accel_complementary_gain");
  });
  maybe_apply(estimator_cpp, "baro_alt_gain", [&](const TomlValue& v) {
    cfg.estimator_cpp.baro_alt_gain = as_double(v, "estimator_cpp.baro_alt_gain");
  });

  const auto& controller_cpp = require_section(doc, "controller_cpp");
  validate_allowed_keys(controller_cpp, {"kp_roll", "kp_pitch", "kd_p", "max_deflection_norm"},
                        "controller_cpp");
  maybe_apply(controller_cpp, "kp_roll", [&](const TomlValue& v) {
    cfg.controller_cpp.kp_roll = as_double(v, "controller_cpp.kp_roll");
  });
  maybe_apply(controller_cpp, "kp_pitch", [&](const TomlValue& v) {
    cfg.controller_cpp.kp_pitch = as_double(v, "controller_cpp.kp_pitch");
  });
  maybe_apply(controller_cpp, "kd_p", [&](const TomlValue& v) {
    cfg.controller_cpp.kd_p = as_double(v, "controller_cpp.kd_p");
  });
  maybe_apply(controller_cpp, "max_deflection_norm", [&](const TomlValue& v) {
    cfg.controller_cpp.max_deflection_norm = as_double(v, "controller_cpp.max_deflection_norm");
  });

  const auto sim_net_it = doc.find("sim_net");
  if (sim_net_it != doc.end()) {
    const auto& sim_net = sim_net_it->second;
    validate_allowed_keys(sim_net, {"enabled", "sensor_host", "sensor_port", "actuator_bind_host", "actuator_port",
                                    "connect_timeout_ms"},
                          "sim_net");
    maybe_apply(sim_net, "enabled", [&](const TomlValue& v) { cfg.sim_net.enabled = as_bool(v, "sim_net.enabled"); });
    maybe_apply(sim_net, "sensor_host",
                [&](const TomlValue& v) { cfg.sim_net.sensor_host = as_string(v, "sim_net.sensor_host"); });
    maybe_apply(sim_net, "sensor_port",
                [&](const TomlValue& v) { cfg.sim_net.sensor_port = static_cast<uint16_t>(as_u32(v, "sim_net.sensor_port")); });
    maybe_apply(sim_net, "actuator_bind_host", [&](const TomlValue& v) {
      cfg.sim_net.actuator_bind_host = as_string(v, "sim_net.actuator_bind_host");
    });
    maybe_apply(sim_net, "actuator_port",
                [&](const TomlValue& v) { cfg.sim_net.actuator_port = static_cast<uint16_t>(as_u32(v, "sim_net.actuator_port")); });
    maybe_apply(sim_net, "connect_timeout_ms", [&](const TomlValue& v) {
      cfg.sim_net.connect_timeout_ms = as_u32(v, "sim_net.connect_timeout_ms");
    });
  }

  if (cfg.killswitch.nc_closed_value > 1) {
    throw std::runtime_error("killswitch.nc_closed_value must be 0 or 1");
  }
  if (cfg.killswitch.debounce_samples == 0) {
    throw std::runtime_error("killswitch.debounce_samples must be >= 1");
  }
  if (cfg.baro.recovery_error_threshold == 0) {
    throw std::runtime_error("baro.recovery_error_threshold must be >= 1");
  }
  if (cfg.timeouts.max_consecutive_imu_failures == 0) {
    throw std::runtime_error("timeouts.max_consecutive_imu_failures must be >= 1");
  }
  if (cfg.timeouts.max_consecutive_baro_failures == 0) {
    throw std::runtime_error("timeouts.max_consecutive_baro_failures must be >= 1");
  }
  if (cfg.threads.igniter_hz == 0) {
    throw std::runtime_error("threads.igniter_hz must be >= 1");
  }
  if (cfg.imu_watchdog.zero_vector_consecutive == 0) {
    throw std::runtime_error("imu_watchdog.zero_vector_consecutive must be >= 1");
  }
  if (cfg.imu_watchdog.flatline_consecutive == 0) {
    throw std::runtime_error("imu_watchdog.flatline_consecutive must be >= 1");
  }
  if (cfg.imu_watchdog.degenerate_pattern_consecutive == 0) {
    throw std::runtime_error("imu_watchdog.degenerate_pattern_consecutive must be >= 1");
  }
  if (!std::isfinite(cfg.imu_watchdog.flatline_epsilon) || cfg.imu_watchdog.flatline_epsilon <= 0.0) {
    throw std::runtime_error("imu_watchdog.flatline_epsilon must be > 0");
  }
  if (cfg.imu_watchdog.healthy_recovery_samples == 0) {
    throw std::runtime_error("imu_watchdog.healthy_recovery_samples must be >= 1");
  }
  if (cfg.imu_watchdog.recovery_backoff_ms == 0) {
    throw std::runtime_error("imu_watchdog.recovery_backoff_ms must be >= 1");
  }
  if (cfg.igniter.settle_ms == 0) {
    throw std::runtime_error("igniter.settle_ms must be >= 1");
  }
  if (cfg.igniter.default_fire_ms == 0) {
    throw std::runtime_error("igniter.default_fire_ms must be >= 1");
  }
  if (cfg.igniter.max_fire_ms == 0) {
    throw std::runtime_error("igniter.max_fire_ms must be >= 1");
  }
  if (cfg.igniter.default_fire_ms > cfg.igniter.max_fire_ms) {
    throw std::runtime_error("igniter.default_fire_ms must be <= igniter.max_fire_ms");
  }
  if (cfg.igniter.fault_policy != "global" && cfg.igniter.fault_policy != "isolated") {
    throw std::runtime_error("igniter.fault_policy must be one of: global | isolated");
  }
  if (cfg.igniter.enabled) {
    std::string input_chip;
    std::string status_chip;
    std::unordered_set<uint32_t> input_lines;
    std::unordered_set<uint32_t> status_lines;
    for (std::size_t i = 0; i < cfg.igniter_channels.size(); ++i) {
      const auto& ch = cfg.igniter_channels[i];
      if (!ch.enabled) {
        throw std::runtime_error("igniter.enabled=true requires igniter" + std::to_string(i) +
                                 ".enabled=true for all channels");
      }
      if (ch.input_chip.empty()) {
        throw std::runtime_error("igniter" + std::to_string(i) + ".input_chip must not be empty");
      }
      if (ch.status_chip.empty()) {
        throw std::runtime_error("igniter" + std::to_string(i) + ".status_chip must not be empty");
      }
      if (input_chip.empty()) {
        input_chip = ch.input_chip;
      } else if (input_chip != ch.input_chip) {
        throw std::runtime_error("All igniter input lines must be on the same gpiochip");
      }
      if (status_chip.empty()) {
        status_chip = ch.status_chip;
      } else if (status_chip != ch.status_chip) {
        throw std::runtime_error("All igniter status lines must be on the same gpiochip");
      }
      if (!input_lines.insert(ch.input_line).second) {
        throw std::runtime_error("Igniter input lines must be unique across channels");
      }
      if (!status_lines.insert(ch.status_line).second) {
        throw std::runtime_error("Igniter status lines must be unique across channels");
      }
    }
  }
  if (cfg.security.require_loopback_sim_net && cfg.sim_net.enabled) {
    const bool sensor_loopback = (cfg.sim_net.sensor_host == "127.0.0.1");
    const bool actuator_loopback = (cfg.sim_net.actuator_bind_host == "127.0.0.1");
    if (!sensor_loopback || !actuator_loopback) {
      throw std::runtime_error(
          "sim_net endpoints must be 127.0.0.1 when security.require_loopback_sim_net=true");
    }
  }
  if (!cfg.runtime.allow_auto_sim_fallback && !cfg.runtime.fail_fast_on_hw_error) {
    throw std::runtime_error(
        "runtime.fail_fast_on_hw_error must be true when runtime.allow_auto_sim_fallback is false");
  }

  return cfg;
}

std::string runtime_config_to_string(const RuntimeConfig& cfg) {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(3);
  oss << "[runtime]\n";
  oss << "sim_mode=" << (cfg.runtime.sim_mode ? "true" : "false") << "\n";
  oss << "fail_fast_on_hw_error=" << (cfg.runtime.fail_fast_on_hw_error ? "true" : "false") << "\n";
  oss << "allow_auto_sim_fallback=" << (cfg.runtime.allow_auto_sim_fallback ? "true" : "false") << "\n";
  oss << "run_duration_s=" << cfg.runtime.run_duration_s << "\n";
  oss << "log_period_ms=" << cfg.runtime.log_period_ms << "\n";

  oss << "[modes]\n";
  oss << "estimator_mode=" << estimator_mode_to_string(cfg.modes.estimator_mode) << "\n";
  oss << "controller_mode=" << controller_mode_to_string(cfg.modes.controller_mode) << "\n";

  oss << "[threads]\n";
  oss << "control_hz=" << cfg.threads.control_hz << ", actuator_hz=" << cfg.threads.actuator_hz
      << ", igniter_hz=" << cfg.threads.igniter_hz << ", imu_hz=" << cfg.threads.imu_hz
      << ", estimator_hz=" << cfg.threads.estimator_hz << ", baro_hz=" << cfg.threads.baro_hz
      << "\n";

  oss << "[ipc]\n";
  oss << "sensor_snapshot_shm=" << cfg.ipc.sensor_snapshot_shm << "\n";
  oss << "estimator_state_shm=" << cfg.ipc.estimator_state_shm << "\n";
  oss << "controller_command_shm=" << cfg.ipc.controller_command_shm << "\n";

  oss << "[security]\n";
  oss << "require_local_ipc_permissions=" << (cfg.security.require_local_ipc_permissions ? "true" : "false") << "\n";
  oss << "require_loopback_sim_net=" << (cfg.security.require_loopback_sim_net ? "true" : "false") << "\n";

  oss << "[imu]\n";
  oss << "spi_device=" << cfg.imu.spi_device << "\n";
  oss << "spi_speed_hz=" << cfg.imu.spi_speed_hz << "\n";
  oss << "spi_mode=" << cfg.imu.spi_mode << "\n";
  oss << "accel_odr=" << cfg.imu.accel_odr << "\n";
  oss << "gyro_odr=" << cfg.imu.gyro_odr << "\n";
  oss << "accel_scale=" << cfg.imu.accel_scale << "\n";
  oss << "gyro_scale=" << cfg.imu.gyro_scale << "\n";

  oss << "[imu_watchdog]\n";
  oss << "enabled=" << (cfg.imu_watchdog.enabled ? "true" : "false") << "\n";
  oss << "zero_vector_consecutive=" << cfg.imu_watchdog.zero_vector_consecutive << "\n";
  oss << "flatline_consecutive=" << cfg.imu_watchdog.flatline_consecutive << "\n";
  oss << "degenerate_pattern_consecutive=" << cfg.imu_watchdog.degenerate_pattern_consecutive << "\n";
  oss << "flatline_epsilon=" << cfg.imu_watchdog.flatline_epsilon << "\n";
  oss << "healthy_recovery_samples=" << cfg.imu_watchdog.healthy_recovery_samples << "\n";
  oss << "recovery_backoff_ms=" << cfg.imu_watchdog.recovery_backoff_ms << "\n";
  oss << "max_reinit_attempts=" << cfg.imu_watchdog.max_reinit_attempts << "\n";

  oss << "[baro]\n";
  oss << "recovery_error_threshold=" << cfg.baro.recovery_error_threshold << "\n";
  oss << "recovery_backoff_ms=" << cfg.baro.recovery_backoff_ms << "\n";

  oss << "[killswitch]\n";
  oss << "enabled=" << (cfg.killswitch.enabled ? "true" : "false") << "\n";
  oss << "gpio=" << cfg.killswitch.gpio << "\n";
  oss << "nc_closed_value=" << cfg.killswitch.nc_closed_value << "\n";
  oss << "debounce_samples=" << cfg.killswitch.debounce_samples << "\n";
  oss << "latch_on_trip=" << (cfg.killswitch.latch_on_trip ? "true" : "false") << "\n";

  oss << "[igniter]\n";
  oss << "enabled=" << (cfg.igniter.enabled ? "true" : "false") << "\n";
  oss << "use_hardware=" << (cfg.igniter.use_hardware ? "true" : "false") << "\n";
  oss << "fault_policy=" << cfg.igniter.fault_policy << "\n";
  oss << "settle_ms=" << cfg.igniter.settle_ms << "\n";
  oss << "latch_faults=" << (cfg.igniter.latch_faults ? "true" : "false") << "\n";
  oss << "default_fire_ms=" << cfg.igniter.default_fire_ms << "\n";
  oss << "max_fire_ms=" << cfg.igniter.max_fire_ms << "\n";
  oss << "command_shm=" << cfg.igniter.command_shm << "\n";
  oss << "status_shm=" << cfg.igniter.status_shm << "\n";

  for (std::size_t i = 0; i < cfg.igniter_channels.size(); ++i) {
    const auto& ch = cfg.igniter_channels[i];
    oss << "[igniter" << i << "] enabled=" << (ch.enabled ? "true" : "false")
        << " input_chip=" << ch.input_chip << " input_line=" << ch.input_line
        << " status_chip=" << ch.status_chip << " status_line=" << ch.status_line << "\n";
  }

  for (std::size_t i = 0; i < cfg.servos.size(); ++i) {
    const auto& s = cfg.servos[i];
    oss << "[servo" << i << "] chip=" << s.chip << " channel=" << s.channel << " min_pwm_us="
        << s.min_pwm_us << " max_pwm_us=" << s.max_pwm_us << " neutral_pwm_us=" << s.neutral_pwm_us
        << " direction=" << s.direction << " trim_norm=" << s.trim_norm << " failsafe_norm="
        << s.failsafe_norm << "\n";
  }

  oss << "[sim_net]\n";
  oss << "enabled=" << (cfg.sim_net.enabled ? "true" : "false") << "\n";
  oss << "sensor_host=" << cfg.sim_net.sensor_host << "\n";
  oss << "sensor_port=" << cfg.sim_net.sensor_port << "\n";
  oss << "actuator_bind_host=" << cfg.sim_net.actuator_bind_host << "\n";
  oss << "actuator_port=" << cfg.sim_net.actuator_port << "\n";
  oss << "connect_timeout_ms=" << cfg.sim_net.connect_timeout_ms << "\n";

  return oss.str();
}

}  // namespace runtime
