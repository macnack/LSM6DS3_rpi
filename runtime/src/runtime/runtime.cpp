#include "runtime/runtime/runtime.hpp"

#include "runtime/common/time.hpp"
#include "runtime/ipc/messages.hpp"
#include "runtime/ipc/shm_mailbox.hpp"
#include "runtime/runtime/sim_net.hpp"
#include "runtime/runtime/actuator.hpp"
#include "runtime/runtime/controller.hpp"
#include "runtime/runtime/estimator.hpp"
#include "runtime/runtime/external_time_validation.hpp"
#include "runtime/runtime/failsafe_diagnostics.hpp"
#include "runtime/runtime/fallback.hpp"
#include "runtime/runtime/kill_switch.hpp"
#include "runtime/runtime/imu_watchdog.hpp"
#include "runtime/runtime/rt_thread.hpp"
#include "runtime/runtime/sensors.hpp"

#if defined(RUNTIME_HAVE_IGNITER) && (RUNTIME_HAVE_IGNITER == 1)
#include "igniter/gpio_batch.hpp"
#include "igniter/igniter_bank.hpp"
#endif

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace runtime {

namespace {

struct SharedState {
  std::mutex mutex;
  ImuSample imu{};
  BaroSample baro{};
  EstimatorState cpp_estimator{};
  ControlCommand latest_command{};
  bool has_command = false;
};

struct I2cDeviceJob {
  std::string name;
  uint32_t period_ms = 0;
  uint8_t priority = 0;
  uint64_t next_due_ns = 0;
  std::function<void(uint64_t)> poll_once;
};

struct JitterSummary {
  uint64_t p50_ns = 0;
  uint64_t p95_ns = 0;
  uint64_t p99_ns = 0;
  uint64_t max_ns = 0;
};

class JitterTracker {
 public:
  void add(uint64_t jitter_ns) {
    std::lock_guard<std::mutex> lock(mutex_);
    samples_[head_] = jitter_ns;
    head_ = (head_ + 1U) % samples_.size();
    if (count_ < samples_.size()) {
      ++count_;
    }
  }

  [[nodiscard]] JitterSummary summary() const {
    std::vector<uint64_t> sorted;
    sorted.reserve(count_);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (std::size_t i = 0; i < count_; ++i) {
        sorted.push_back(samples_[i]);
      }
    }
    if (sorted.empty()) {
      return {};
    }
    std::sort(sorted.begin(), sorted.end());
    const auto at_percentile = [&](std::size_t pct) -> uint64_t {
      const std::size_t idx =
          ((sorted.size() - 1U) * pct) / 100U;
      return sorted[idx];
    };

    JitterSummary out;
    out.p50_ns = at_percentile(50);
    out.p95_ns = at_percentile(95);
    out.p99_ns = at_percentile(99);
    out.max_ns = sorted.back();
    return out;
  }

 private:
  static constexpr std::size_t kCapacity = 4096;
  mutable std::mutex mutex_;
  std::array<uint64_t, kCapacity> samples_{};
  std::size_t head_ = 0;
  std::size_t count_ = 0;
};

bool is_finite_array(const std::array<float, 4>& a) {
  for (float v : a) {
    if (!std::isfinite(v)) {
      return false;
    }
  }
  return true;
}

bool is_finite_array(const std::array<float, 3>& a) {
  for (float v : a) {
    if (!std::isfinite(v)) {
      return false;
    }
  }
  return true;
}

bool validate_header(uint32_t msg_magic, uint16_t version, uint16_t payload_bytes, uint16_t expected_payload) {
  return msg_magic == kMessageMagic && version == kMessageVersion && payload_bytes == expected_payload;
}

enum class ExternalMsgValidationError : uint8_t {
  None = 0,
  Schema = 1,
  Crc = 2,
  Payload = 3,
  Stale = 4,
  TimeRegression = 5,
};

bool validate_external_estimator_msg(const ExternalEstimatorStateMsg& msg, EstimatorState& out,
                                     ExternalMsgValidationError* error_out = nullptr) {
  if (error_out != nullptr) {
    *error_out = ExternalMsgValidationError::None;
  }
  if (!validate_header(msg.msg_magic, msg.msg_version, msg.payload_bytes,
                       payload_size_bytes<ExternalEstimatorStateMsg>())) {
    if (error_out != nullptr) {
      *error_out = ExternalMsgValidationError::Schema;
    }
    return false;
  }
  if (!validate_message_crc(msg)) {
    if (error_out != nullptr) {
      *error_out = ExternalMsgValidationError::Crc;
    }
    return false;
  }
  if (msg.valid == 0) {
    if (error_out != nullptr) {
      *error_out = ExternalMsgValidationError::Payload;
    }
    return false;
  }
  if (!is_finite_array(msg.q_body_to_ned) || !is_finite_array(msg.vel_ned_mps) ||
      !is_finite_array(msg.pos_ned_m)) {
    if (error_out != nullptr) {
      *error_out = ExternalMsgValidationError::Payload;
    }
    return false;
  }

  const double q_norm = std::sqrt(static_cast<double>(msg.q_body_to_ned[0]) * msg.q_body_to_ned[0] +
                                  static_cast<double>(msg.q_body_to_ned[1]) * msg.q_body_to_ned[1] +
                                  static_cast<double>(msg.q_body_to_ned[2]) * msg.q_body_to_ned[2] +
                                  static_cast<double>(msg.q_body_to_ned[3]) * msg.q_body_to_ned[3]);
  if (q_norm < 0.85 || q_norm > 1.15) {
    if (error_out != nullptr) {
      *error_out = ExternalMsgValidationError::Payload;
    }
    return false;
  }

  out.t_ns = msg.t_ns;
  out.q_body_to_ned = msg.q_body_to_ned;
  out.vel_ned_mps = msg.vel_ned_mps;
  out.pos_ned_m = msg.pos_ned_m;
  out.valid = true;
  return true;
}

bool validate_external_controller_msg(const ExternalControllerCommandMsg& msg, ControlCommand& out,
                                      ExternalMsgValidationError* error_out = nullptr) {
  if (error_out != nullptr) {
    *error_out = ExternalMsgValidationError::None;
  }
  if (!validate_header(msg.msg_magic, msg.msg_version, msg.payload_bytes,
                       payload_size_bytes<ExternalControllerCommandMsg>())) {
    if (error_out != nullptr) {
      *error_out = ExternalMsgValidationError::Schema;
    }
    return false;
  }
  if (!validate_message_crc(msg)) {
    if (error_out != nullptr) {
      *error_out = ExternalMsgValidationError::Crc;
    }
    return false;
  }

  out = ControlCommand{};
  out.t_ns = msg.t_ns;
  out.armed = (msg.armed != 0);
  out.valid = true;

  for (std::size_t i = 0; i < kServoCount; ++i) {
    const float value = msg.servo_norm[i];
    if (!std::isfinite(value)) {
      if (error_out != nullptr) {
        *error_out = ExternalMsgValidationError::Payload;
      }
      return false;
    }
    if (value < -1.0F || value > 1.0F) {
      if (error_out != nullptr) {
        *error_out = ExternalMsgValidationError::Payload;
      }
      return false;
    }
    out.servo_norm[i] = value;
  }

  return true;
}

#if defined(RUNTIME_HAVE_IGNITER) && (RUNTIME_HAVE_IGNITER == 1)
bool validate_igniter_command_msg(const IgniterCommandMsg& msg) {
  if (!validate_header(msg.msg_magic, msg.msg_version, msg.payload_bytes,
                       payload_size_bytes<IgniterCommandMsg>())) {
    return false;
  }
  if (!validate_message_crc(msg)) {
    return false;
  }
  if (msg.fire_mask > 0x0F) {
    return false;
  }
  const auto action = static_cast<IgniterCommandAction>(msg.action);
  switch (action) {
    case IgniterCommandAction::None:
    case IgniterCommandAction::Arm:
    case IgniterCommandAction::Disarm:
    case IgniterCommandAction::FireMask:
    case IgniterCommandAction::ClearFault:
      return true;
  }
  return false;
}
#endif

ControlCommand make_failsafe_command(const std::array<ServoSection, kServoCount>& servos, uint64_t now_ns) {
  ControlCommand cmd{};
  cmd.t_ns = now_ns;
  cmd.armed = true;
  cmd.valid = true;
  for (std::size_t i = 0; i < kServoCount; ++i) {
    cmd.servo_norm[i] = servos[i].failsafe_norm;
  }
  return cmd;
}

const char* failsafe_reason_name(FailsafeReason reason) {
  switch (reason) {
    case FailsafeReason::None:
      return "none";
    case FailsafeReason::CommandTimeout:
      return "command_timeout";
    case FailsafeReason::ImuStale:
      return "imu_stale";
    case FailsafeReason::IoErrorLatch:
      return "io_error_latch";
    case FailsafeReason::KillSwitch:
      return "kill_switch";
    case FailsafeReason::InvalidEstimator:
      return "invalid_estimator";
    case FailsafeReason::LinkDown:
      return "link_down";
  }
  return "unknown";
}

bool is_loopback_host(const std::string& host) {
  return host == "127.0.0.1";
}

void record_external_time_reject(ExternalTimeValidationResult result, RuntimeStats& stats) {
  if (result == ExternalTimeValidationResult::RejectRegression) {
    ++stats.external_msg_time_regression_reject_count;
  } else if (result == ExternalTimeValidationResult::RejectStale ||
             result == ExternalTimeValidationResult::RejectFuture) {
    ++stats.external_msg_stale_reject_count;
  }
}

uint64_t ns_to_ms(uint64_t t_ns) {
  return t_ns / 1'000'000ULL;
}

}  // namespace

class Runtime::Impl {
 public:
  Impl(RuntimeConfig cfg, RuntimeCliOverrides overrides)
      : cfg_(std::move(cfg)),
        overrides_(std::move(overrides)),
        sim_mode_(cfg_.runtime.sim_mode),
        sensor_snapshot_mailbox_(cfg_.ipc.sensor_snapshot_shm, true),
        estimator_state_mailbox_(cfg_.ipc.estimator_state_shm, true),
        controller_cmd_mailbox_(cfg_.ipc.controller_command_shm, true),
        igniter_cmd_mailbox_(cfg_.igniter.command_shm, true),
        igniter_status_mailbox_(cfg_.igniter.status_shm, true),
        cpp_estimator_(cfg_.estimator_cpp),
        cpp_controller_(cfg_.controller_cpp),
        actuator_mapper_(cfg_.actuator, cfg_.servos),
        failsafe_cmd_(make_failsafe_command(cfg_.servos, monotonic_time_ns())),
        kill_switch_(cfg_.killswitch) {}

  void run() {
    bump_stat([this](RuntimeStats& s) { s.degraded_mode_active = degraded_mode_active_.load(std::memory_order_relaxed); });
    setup_backends();
    setup_mailboxes();

    start_workers();

    uint64_t status_period_ns = 0;
    uint64_t next_status_write_ns = 0;
    if (!overrides_.status_file.empty()) {
      const uint64_t period_ms =
          (cfg_.runtime.log_period_ms > 0) ? cfg_.runtime.log_period_ms : 1000ULL;
      status_period_ns = ns_from_ms(period_ms);
      next_status_write_ns = monotonic_time_ns() + status_period_ns;
    }

    auto maybe_write_periodic_status = [this, status_period_ns, &next_status_write_ns]() mutable {
      if (status_period_ns == 0) {
        return;
      }
      const uint64_t now_ns = monotonic_time_ns();
      if (now_ns >= next_status_write_ns) {
        maybe_write_status_file();
        next_status_write_ns = now_ns + status_period_ns;
      }
    };

    const double runtime_sec = effective_run_duration_s();
    if (runtime_sec > 0.0) {
      const uint64_t deadline_ns = monotonic_time_ns() + static_cast<uint64_t>(runtime_sec * 1e9);
      while (!stop_requested_.load(std::memory_order_relaxed) && monotonic_time_ns() < deadline_ns) {
        maybe_write_periodic_status();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
      stop_requested_.store(true, std::memory_order_relaxed);
    } else {
      while (!stop_requested_.load(std::memory_order_relaxed)) {
        maybe_write_periodic_status();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
    }

    join_workers();
    shutdown_backends();
    maybe_write_status_file();
  }

  void request_stop() { stop_requested_.store(true, std::memory_order_relaxed); }

  RuntimeStats stats_snapshot() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    return stats_;
  }

 private:
  void setup_backends() {
    auto setup = [this](bool sim_mode) {
      if (cfg_.security.require_loopback_sim_net && cfg_.sim_net.enabled) {
        if (!is_loopback_host(cfg_.sim_net.sensor_host) ||
            !is_loopback_host(cfg_.sim_net.actuator_bind_host)) {
          throw std::runtime_error(
              "sim_net endpoints must be 127.0.0.1 when security.require_loopback_sim_net=true");
        }
      }

      if (sim_mode && cfg_.sim_net.enabled) {
        sim_net_link_ = std::make_unique<SimNetLink>(cfg_.sim_net);
        sim_net_link_->start();
        imu_backend_ = std::make_unique<NetImuBackend>(*sim_net_link_, cfg_.timeouts);
        baro_backend_ = std::make_unique<NetBaroBackend>(*sim_net_link_, cfg_.timeouts);
      } else {
        imu_backend_ = make_imu_backend(cfg_, sim_mode);
        baro_backend_ = make_baro_backend(cfg_, sim_mode);
      }
      if (!sim_mode && cfg_.actuator.use_hardware) {
        actuator_backend_ = std::make_unique<HardwareActuatorBackend>();
      } else {
        actuator_backend_ = std::make_unique<SimActuatorBackend>();
      }
#if defined(RUNTIME_HAVE_IGNITER) && (RUNTIME_HAVE_IGNITER == 1)
      if (cfg_.igniter.enabled) {
        igniter::IgniterBank::Config ign_cfg;
        ign_cfg.faultPolicy = (cfg_.igniter.fault_policy == "isolated") ? igniter::FaultPolicy::Isolated
                                                                         : igniter::FaultPolicy::Global;
        for (std::size_t i = 0; i < kIgniterCount; ++i) {
          ign_cfg.channels[i].enabled = cfg_.igniter_channels[i].enabled;
          ign_cfg.channels[i].driver.settleMs = cfg_.igniter.settle_ms;
          ign_cfg.channels[i].driver.latchFaults = cfg_.igniter.latch_faults;
          ign_cfg.channels[i].igniter.defaultFireMs = cfg_.igniter.default_fire_ms;
          ign_cfg.channels[i].igniter.maxFireMs = cfg_.igniter.max_fire_ms;
        }

        std::unique_ptr<igniter::GpioBatchOut> out;
        std::unique_ptr<igniter::GpioBatchIn> in;
        if (!sim_mode && cfg_.igniter.use_hardware) {
          out = igniter::make_hardware_batch_out();
          in = igniter::make_hardware_batch_in();
        } else {
          out = std::make_unique<igniter::SimGpioBatchOut>();
          in = std::make_unique<igniter::SimGpioBatchIn>();
        }

        igniter_bank_ = std::make_unique<igniter::IgniterBank>(ign_cfg, std::move(out), std::move(in));

        std::array<uint32_t, kIgniterCount> output_lines{0, 0, 0, 0};
        std::array<uint32_t, kIgniterCount> status_lines{0, 0, 0, 0};
        for (std::size_t i = 0; i < kIgniterCount; ++i) {
          output_lines[i] = cfg_.igniter_channels[i].input_line;
          status_lines[i] = cfg_.igniter_channels[i].status_line;
        }
        igniter_bank_->bind_hardware(cfg_.igniter_channels[0].input_chip, cfg_.igniter_channels[0].status_chip,
                                     output_lines, status_lines);
        igniter_bank_->init(ns_to_ms(monotonic_time_ns()));
      } else {
        igniter_bank_.reset();
      }
#else
      if (cfg_.igniter.enabled) {
        throw std::runtime_error(
            "Igniter service enabled in config but runtime was built without IGNITER module support");
      }
#endif

      imu_backend_->start();
      baro_backend_->start();
      actuator_backend_->start(cfg_.actuator, cfg_.servos);
      kill_switch_.start();
    };

    try {
      setup(sim_mode_);
    } catch (const std::exception& ex) {
      // Production-safe startup policy:
      // - allow_auto_sim_fallback=false => fail fast on hardware startup errors.
      // - allow_auto_sim_fallback=true  => explicit degraded sim fallback path.
      if (!sim_mode_ && cfg_.runtime.allow_auto_sim_fallback) {
        std::cerr << "RT_CORE_EVENT severity=ERROR event=hardware_startup_failed action=sim_fallback reason=\""
                  << ex.what() << "\"\n";
        sim_mode_ = true;
        degraded_mode_active_.store(true, std::memory_order_relaxed);
        bump_stat([](RuntimeStats& s) { s.degraded_mode_active = true; });
        setup(true);
      } else if (!sim_mode_ && cfg_.runtime.fail_fast_on_hw_error) {
        throw;
      } else {
        throw;
      }
    }
  }

  void setup_mailboxes() {
    set_shm_security_policy(cfg_.security.require_local_ipc_permissions);
    try {
      sensor_snapshot_mailbox_.open(true);
      estimator_state_mailbox_.open(true);
      controller_cmd_mailbox_.open(true);
      if (cfg_.igniter.enabled) {
        igniter_cmd_mailbox_.open(true);
        igniter_status_mailbox_.open(true);
      }
    } catch (const std::exception&) {
      bump_stat([](RuntimeStats& s) {
        ++s.ipc_permission_reject_count;
        s.degraded_mode_active = true;
      });
      degraded_mode_active_.store(true, std::memory_order_relaxed);
      throw;
    }
  }

  void shutdown_backends() noexcept {
#if defined(RUNTIME_HAVE_IGNITER) && (RUNTIME_HAVE_IGNITER == 1)
    if (igniter_bank_) {
      try {
        igniter_bank_->disarm(ns_to_ms(monotonic_time_ns()));
      } catch (...) {
      }
    }
#endif
    if (actuator_backend_) {
      actuator_backend_->stop();
    }
    if (baro_backend_) {
      baro_backend_->stop();
    }
    if (imu_backend_) {
      imu_backend_->stop();
    }
    if (sim_net_link_) {
      sim_net_link_->stop();
      sim_net_link_.reset();
    }
    kill_switch_.stop();
  }

  void start_workers() {
    workers_.push_back(std::thread([this] { control_worker(); }));
    workers_.push_back(std::thread([this] { actuator_worker(); }));
    workers_.push_back(std::thread([this] { igniter_worker(); }));
    workers_.push_back(std::thread([this] { imu_worker(); }));
    workers_.push_back(std::thread([this] { i2c_hub_worker(); }));
    workers_.push_back(std::thread([this] { estimator_worker(); }));
  }

  void join_workers() {
    for (auto& t : workers_) {
      if (t.joinable()) {
        t.join();
      }
    }
    workers_.clear();
  }

  void control_worker() {
    std::string rt_err;
    if (!configure_current_thread_rt("control", cfg_.threads.control_priority, cfg_.threads.control_cpu,
                                     &rt_err)) {
      std::cerr << "control_thread RT warning: " << rt_err << "\n";
    }

    const uint64_t period_ns = hz_to_period_ns(cfg_.threads.control_hz);
    const uint64_t miss_tolerance_ns = period_ns / 2U;
    uint64_t scheduled_tick_ns = monotonic_time_ns();

    uint64_t external_estimator_seq_seen = 0;
    uint64_t external_controller_seq_seen = 0;
    uint64_t external_estimator_t_ns_seen = 0;
    uint64_t external_controller_t_ns_seen = 0;
    uint64_t sensor_seq = 0;
    uint64_t local_tick = 0;
    bool prev_kill_active = false;

    uint64_t last_log_ns = monotonic_time_ns();

    while (!stop_requested_.load(std::memory_order_relaxed)) {
      const uint64_t now_ns = monotonic_time_ns();
      const uint64_t jitter_ns = (now_ns >= scheduled_tick_ns) ? (now_ns - scheduled_tick_ns)
                                                                : (scheduled_tick_ns - now_ns);
      control_jitter_tracker_.add(jitter_ns);
      if (now_ns > scheduled_tick_ns + miss_tolerance_ns) {
        bump_stat([](RuntimeStats& s) { ++s.control_deadline_miss_count; });
      }

      ImuSample imu;
      BaroSample baro;
      EstimatorState cpp_est;
      {
        std::lock_guard<std::mutex> lock(shared_.mutex);
        imu = shared_.imu;
        baro = shared_.baro;
        cpp_est = shared_.cpp_estimator;
      }

      const bool imu_stale =
          (!imu.valid) ||
          (now_ns > imu.t_ns && (now_ns - imu.t_ns) > ns_from_ms(cfg_.timeouts.imu_stale_ms));

      bool kill_active = false;
      if (kill_switch_.enabled()) {
        try {
          kill_active = kill_switch_.poll();
        } catch (const std::exception& ex) {
          kill_active = true;
          if (!kill_switch_read_error_logged_.exchange(true, std::memory_order_relaxed)) {
            std::cerr << "Kill switch read error, forcing failsafe: " << ex.what() << "\n";
          }
        }
      }
      kill_switch_active_.store(kill_active, std::memory_order_relaxed);
      if (kill_active && !prev_kill_active) {
        bump_stat([](RuntimeStats& s) { ++s.killswitch_trip_count; });
      }
      prev_kill_active = kill_active;
      bump_stat([kill_active](RuntimeStats& s) { s.killswitch_active = kill_active; });

      SensorSnapshotMsg sensor_msg{};
      fill_message_header(sensor_msg, ++sensor_seq, now_ns);
      sensor_msg.imu_valid = imu.valid ? 1U : 0U;
      sensor_msg.baro_valid = baro.valid ? 1U : 0U;
      sensor_msg.ax_mps2 = imu.ax_mps2;
      sensor_msg.ay_mps2 = imu.ay_mps2;
      sensor_msg.az_mps2 = imu.az_mps2;
      sensor_msg.gx_rads = imu.gx_rads;
      sensor_msg.gy_rads = imu.gy_rads;
      sensor_msg.gz_rads = imu.gz_rads;
      sensor_msg.pressure_pa = baro.pressure_pa;
      sensor_msg.temperature_c = baro.temperature_c;
      finalize_message_crc(sensor_msg);
      sensor_snapshot_mailbox_.write(sensor_msg);

      EstimatorState selected_est{};
      if (cfg_.modes.estimator_mode == EstimatorMode::CppNative) {
        selected_est = cpp_est;
      } else {
        EstimatorState py_candidate{};
        bool has_py_candidate = false;

        ExternalEstimatorStateMsg py_msg{};
        uint64_t stable_seq = 0;
        if (estimator_state_mailbox_.try_read(py_msg, &stable_seq) && stable_seq != external_estimator_seq_seen) {
          external_estimator_seq_seen = stable_seq;
          ExternalMsgValidationError err = ExternalMsgValidationError::None;
          if (validate_external_estimator_msg(py_msg, py_candidate, &err)) {
            const auto time_result =
                validate_external_timestamp(monotonic_time_ns(), py_candidate.t_ns, external_estimator_t_ns_seen,
                                            ns_from_ms(cfg_.timeouts.estimator_fresh_ms));
            if (time_result == ExternalTimeValidationResult::Accept) {
              external_estimator_t_ns_seen = py_candidate.t_ns;
              has_py_candidate = true;
            } else {
              bump_stat([time_result](RuntimeStats& s) {
                ++s.external_estimator_reject_count;
                record_external_time_reject(time_result, s);
              });
            }
          } else {
            bump_stat([err](RuntimeStats& s) {
              ++s.external_estimator_reject_count;
              if (err == ExternalMsgValidationError::Schema) {
                ++s.external_msg_schema_reject_count;
              } else if (err == ExternalMsgValidationError::Crc) {
                ++s.external_msg_crc_reject_count;
              }
            });
          }
        }

        EstimatorSelectionInput est_input;
        est_input.now_ns = now_ns;
        est_input.policy.fresh_timeout_ns = ns_from_ms(cfg_.timeouts.estimator_fresh_ms);
        est_input.policy.hold_timeout_ns = ns_from_ms(cfg_.timeouts.estimator_hold_ms);
        est_input.has_python_candidate = has_py_candidate;
        est_input.python_candidate = py_candidate;
        est_input.has_cpp_candidate = cpp_est.valid;
        est_input.cpp_candidate = cpp_est;

        const auto est_result = select_estimator_state(est_input, estimator_selection_state_);
        selected_est = est_result.state;
        if (est_result.source == EstimatorSelectionSource::PythonFresh) {
          bump_stat([](RuntimeStats& s) { ++s.external_estimator_accept_count; });
        }
        if (est_result.source == EstimatorSelectionSource::Failsafe) {
          selected_est.valid = false;
        }
      }

      ControlCommand cmd{};
      if (cfg_.modes.controller_mode == ControllerMode::CppNative) {
        const bool armed = cfg_.actuator.start_armed && !kill_active;
        cmd = cpp_controller_.step(imu, selected_est, armed, now_ns);
      } else {
        ControlCommand py_candidate{};
        bool has_py_candidate = false;

        ExternalControllerCommandMsg py_msg{};
        uint64_t stable_seq = 0;
        if (controller_cmd_mailbox_.try_read(py_msg, &stable_seq) && stable_seq != external_controller_seq_seen) {
          external_controller_seq_seen = stable_seq;
          ExternalMsgValidationError err = ExternalMsgValidationError::None;
          if (validate_external_controller_msg(py_msg, py_candidate, &err)) {
            const auto time_result =
                validate_external_timestamp(monotonic_time_ns(), py_candidate.t_ns, external_controller_t_ns_seen,
                                            ns_from_ms(cfg_.timeouts.controller_fresh_ms));
            if (time_result == ExternalTimeValidationResult::Accept) {
              external_controller_t_ns_seen = py_candidate.t_ns;
              has_py_candidate = true;
            } else {
              bump_stat([time_result](RuntimeStats& s) {
                ++s.external_controller_reject_count;
                record_external_time_reject(time_result, s);
              });
            }
          } else {
            bump_stat([err](RuntimeStats& s) {
              ++s.external_controller_reject_count;
              if (err == ExternalMsgValidationError::Schema) {
                ++s.external_msg_schema_reject_count;
              } else if (err == ExternalMsgValidationError::Crc) {
                ++s.external_msg_crc_reject_count;
              }
            });
          }
        }

        ControllerSelectionInput ctrl_input;
        ctrl_input.now_ns = now_ns;
        ctrl_input.policy.fresh_timeout_ns = ns_from_ms(cfg_.timeouts.controller_fresh_ms);
        ctrl_input.policy.hold_timeout_ns = ns_from_ms(cfg_.timeouts.controller_hold_ms);
        ctrl_input.has_python_candidate = has_py_candidate;
        ctrl_input.python_candidate = py_candidate;
        ctrl_input.failsafe_command = failsafe_cmd_;

        const auto ctrl_result = select_controller_command(ctrl_input, controller_selection_state_);
        cmd = ctrl_result.command;
        if (ctrl_result.source == ControllerSelectionSource::PythonFresh) {
          bump_stat([](RuntimeStats& s) { ++s.external_controller_accept_count; });
        }
      }

      FailsafeReason control_failsafe_reason = FailsafeReason::None;
      if (kill_active) {
        control_failsafe_reason = FailsafeReason::KillSwitch;
      } else if (imu_stale) {
        control_failsafe_reason = FailsafeReason::ImuStale;
      } else if (!selected_est.valid) {
        control_failsafe_reason = FailsafeReason::InvalidEstimator;
      }

      if (control_failsafe_reason != FailsafeReason::None) {
        cmd = failsafe_cmd_;
        cmd.t_ns = now_ns;
        cmd.armed = false;
        const auto failsafe_reason_u32 = static_cast<uint32_t>(control_failsafe_reason);
        bump_stat([failsafe_reason_u32](RuntimeStats& s) { s.last_failsafe_reason = failsafe_reason_u32; });
      }

      cmd = sanitize_actuator_command(cmd, -1.0, 1.0);
      cmd.t_ns = now_ns;

      {
        std::lock_guard<std::mutex> lock(shared_.mutex);
        shared_.latest_command = cmd;
        shared_.has_command = true;
      }

      ++local_tick;
      bump_stat([](RuntimeStats& s) { ++s.control_ticks; });
      if ((local_tick % 64ULL) == 0ULL) {
        const JitterSummary js = control_jitter_tracker_.summary();
        bump_stat([&](RuntimeStats& s) {
          s.control_jitter_p50_ns = js.p50_ns;
          s.control_jitter_p95_ns = js.p95_ns;
          s.control_jitter_p99_ns = js.p99_ns;
          s.control_jitter_max_ns = js.max_ns;
        });
        if (sim_net_link_) {
          const SimNetStats sn = sim_net_link_->stats_snapshot();
          bump_stat([&](RuntimeStats& s) {
            s.sim_net_sensor_frames = sn.sensor_frames;
            s.sim_net_sensor_crc_fail = sn.sensor_crc_fail;
            s.sim_net_sensor_disconnects = sn.sensor_disconnects;
            s.sim_net_actuator_frames = sn.actuator_frames;
            s.sim_net_actuator_send_errors = sn.actuator_send_errors;
            s.sim_net_actuator_clients = sn.actuator_clients;
            s.sim_net_actuator_disconnects = sn.actuator_disconnects;
            s.sim_net_actuator_client_connected = sn.actuator_client_connected;
          });
        }
      }

      if (cfg_.runtime.log_period_ms > 0) {
        const uint64_t now_for_log = monotonic_time_ns();
        const uint64_t log_period_ns = ns_from_ms(cfg_.runtime.log_period_ms);
        if (now_for_log - last_log_ns >= log_period_ns) {
          last_log_ns = now_for_log;
          const RuntimeStats snapshot = stats_snapshot();
          std::cout << "[" << now_for_log << "] rt_core: control=" << snapshot.control_ticks
                    << " actuator=" << snapshot.actuator_ticks
                    << " imu=" << snapshot.imu_ticks
                    << " ext_est_accept=" << snapshot.external_estimator_accept_count
                    << " ext_ctrl_accept=" << snapshot.external_controller_accept_count
                    << " failsafe=" << snapshot.failsafe_activation_count
                    << " failsafe_reason=" << failsafe_reason_name(
                           static_cast<FailsafeReason>(snapshot.last_failsafe_reason))
                    << " ks=" << (snapshot.killswitch_active ? "1" : "0")
                    << " ctrl_j99_ns=" << snapshot.control_jitter_p99_ns
                    << " i2c_recovery=" << snapshot.i2c_recovery_count
                    << " degraded=" << (snapshot.degraded_mode_active ? "1" : "0") << "\n";
        }
      }

      scheduled_tick_ns += period_ns;
      sleep_until_ns(scheduled_tick_ns);
    }
  }

  void actuator_worker() {
    std::string rt_err;
    if (!configure_current_thread_rt("actuator", cfg_.threads.actuator_priority, -1, &rt_err)) {
      std::cerr << "actuator_worker RT warning: " << rt_err << "\n";
    }

    const uint64_t period_ns = hz_to_period_ns(cfg_.threads.actuator_hz);
    const uint64_t miss_tolerance_ns = period_ns / 2U;
    uint64_t scheduled_tick_ns = monotonic_time_ns();
    uint64_t local_tick = 0;

    uint32_t consecutive_io_errors = 0;
    bool failsafe_latched = false;
    bool prev_failsafe_override = false;
    LinkDownHysteresisState link_down_state{};
    const uint64_t link_down_hold_ns = ns_from_ms(200);
    const uint64_t link_up_hold_ns = ns_from_ms(500);

    while (!stop_requested_.load(std::memory_order_relaxed)) {
      const uint64_t now_ns = monotonic_time_ns();
      const uint64_t jitter_ns = (now_ns >= scheduled_tick_ns) ? (now_ns - scheduled_tick_ns)
                                                                : (scheduled_tick_ns - now_ns);
      actuator_jitter_tracker_.add(jitter_ns);
      if (now_ns > scheduled_tick_ns + miss_tolerance_ns) {
        bump_stat([](RuntimeStats& s) { ++s.actuator_deadline_miss_count; });
      }

      ControlCommand cmd{};
      ImuSample imu{};
      bool has_cmd = false;
      {
        std::lock_guard<std::mutex> lock(shared_.mutex);
        has_cmd = shared_.has_command;
        cmd = shared_.latest_command;
        imu = shared_.imu;
      }

      bool cmd_timed_out = true;
      uint64_t cmd_age_ns = 0;
      if (has_cmd && cmd.t_ns > 0 && now_ns >= cmd.t_ns) {
        cmd_age_ns = now_ns - cmd.t_ns;
        cmd_timed_out = cmd_age_ns > ns_from_ms(cfg_.timeouts.actuator_cmd_timeout_ms);
      }

      const bool imu_stale =
          (!imu.valid) ||
          (now_ns > imu.t_ns && (now_ns - imu.t_ns) > ns_from_ms(cfg_.timeouts.imu_stale_ms));
      const bool kill_active = kill_switch_active_.load(std::memory_order_relaxed);
      bool raw_link_down = false;
      if (cfg_.sim_net.enabled && sim_net_link_) {
        raw_link_down = !sim_net_link_->actuator_client_connected();
      }
      const bool link_down = cfg_.sim_net.enabled && sim_net_link_ &&
                             update_link_down_hysteresis(raw_link_down, now_ns, link_down_hold_ns,
                                                         link_up_hold_ns, link_down_state);
      const bool failsafe_override = failsafe_latched || cmd_timed_out || imu_stale || kill_active || link_down;
      const ActuatorFailsafeCause failsafe_cause =
          select_actuator_failsafe_cause(failsafe_latched, kill_active, imu_stale, link_down, cmd_timed_out);
      bump_stat([prev_failsafe_override, failsafe_override, failsafe_cause](RuntimeStats& s) {
        account_failsafe_transition(prev_failsafe_override, failsafe_override, failsafe_cause, s);
      });
      prev_failsafe_override = failsafe_override;

      if (!has_cmd || cmd_timed_out) {
        bump_stat([](RuntimeStats& s) {
          ++s.actuator_health.cmd_missed_deadlines;
          ++s.actuator_deadline_miss_count;
        });
      }

      if (sim_net_link_) {
        sim_net_link_->publish_actuator(cmd, now_ns);
      }

      const auto pulses = actuator_mapper_.map(has_cmd ? cmd : failsafe_cmd_, now_ns, failsafe_override);

      try {
        actuator_backend_->write_us(pulses);
        consecutive_io_errors = 0;
      } catch (const std::exception& ex) {
        (void)ex;
        ++consecutive_io_errors;
        bump_stat([](RuntimeStats& s) { ++s.actuator_health.io_errors; });
        if (cfg_.actuator.latch_failsafe_on_io_errors &&
            consecutive_io_errors >= cfg_.actuator.max_io_errors_before_latch) {
          failsafe_latched = true;
          bump_stat([](RuntimeStats& s) {
            s.last_failsafe_reason = static_cast<uint32_t>(FailsafeReason::IoErrorLatch);
          });
        }
      }

      ++local_tick;
      bump_stat([failsafe_latched, cmd_age_ns](RuntimeStats& s) {
        ++s.actuator_ticks;
        s.actuator_health.failsafe_active = failsafe_latched;
        s.actuator_cmd_age_last_ns = cmd_age_ns;
        s.actuator_cmd_age_max_ns = std::max(s.actuator_cmd_age_max_ns, cmd_age_ns);
      });
      if ((local_tick % 64ULL) == 0ULL) {
        const JitterSummary js = actuator_jitter_tracker_.summary();
        bump_stat([&](RuntimeStats& s) {
          s.actuator_jitter_p50_ns = js.p50_ns;
          s.actuator_jitter_p95_ns = js.p95_ns;
          s.actuator_jitter_p99_ns = js.p99_ns;
          s.actuator_jitter_max_ns = js.max_ns;
        });
      }

      scheduled_tick_ns += period_ns;
      sleep_until_ns(scheduled_tick_ns);
    }
  }

  void igniter_worker() {
    std::string rt_err;
    if (!configure_current_thread_rt("igniter", cfg_.threads.igniter_priority, -1, &rt_err)) {
      std::cerr << "igniter_worker RT warning: " << rt_err << "\n";
    }

    const uint64_t period_ns = hz_to_period_ns(cfg_.threads.igniter_hz);

#if defined(RUNTIME_HAVE_IGNITER) && (RUNTIME_HAVE_IGNITER == 1)
    if (!cfg_.igniter.enabled || !igniter_bank_) {
      return;
    }
#else
    (void)period_ns;
    return;
#endif

    uint64_t scheduled_tick_ns = monotonic_time_ns();
    uint64_t cmd_seq_seen = 0;
    uint64_t status_seq = 0;

    while (!stop_requested_.load(std::memory_order_relaxed)) {
      const uint64_t now_ns = monotonic_time_ns();
      const uint64_t now_ms = ns_to_ms(now_ns);

#if defined(RUNTIME_HAVE_IGNITER) && (RUNTIME_HAVE_IGNITER == 1)
      IgniterCommandMsg cmd{};
      uint64_t stable_seq = 0;
      if (igniter_cmd_mailbox_.try_read(cmd, &stable_seq) && stable_seq != cmd_seq_seen) {
        cmd_seq_seen = stable_seq;

        bool accepted = false;
        if (validate_igniter_command_msg(cmd)) {
          const auto action = static_cast<IgniterCommandAction>(cmd.action);
          switch (action) {
            case IgniterCommandAction::None:
              accepted = true;
              break;
            case IgniterCommandAction::Arm:
              accepted = igniter_bank_->arm(now_ms);
              break;
            case IgniterCommandAction::Disarm:
              igniter_bank_->disarm(now_ms);
              accepted = true;
              break;
            case IgniterCommandAction::FireMask:
              accepted = igniter_bank_->fire_mask(cmd.fire_mask, cmd.duration_ms, now_ms);
              break;
            case IgniterCommandAction::ClearFault:
              igniter_bank_->clear_fault(now_ms);
              accepted = true;
              break;
          }
        }
        bump_stat([accepted](RuntimeStats& s) {
          if (accepted) {
            ++s.igniter_command_accept_count;
          } else {
            ++s.igniter_command_reject_count;
          }
        });
      }

      const bool kill_active = kill_switch_active_.load(std::memory_order_relaxed);
      const auto snap = igniter_bank_->update(now_ms, kill_active);

      IgniterStatusMsg status{};
      fill_message_header(status, ++status_seq, now_ns);
      status.armed = snap.armed ? 1U : 0U;
      status.global_fault_latched = snap.globalFaultLatched ? 1U : 0U;
      status.active_mask = snap.activeMask;
      for (std::size_t i = 0; i < kIgniterCount; ++i) {
        status.state[i] = static_cast<uint8_t>(snap.states[i]);
        status.fault[i] = static_cast<uint8_t>(snap.faults[i]);
        status.remaining_ms[i] = snap.remainingMs[i];
      }
      finalize_message_crc(status);
      igniter_status_mailbox_.write(status);

      bump_stat([&](RuntimeStats& s) {
        ++s.igniter_ticks;
        s.igniter_armed = snap.armed;
        s.igniter_global_fault_latched = snap.globalFaultLatched;
        s.igniter_active_mask = snap.activeMask;
      });
#endif

      scheduled_tick_ns += period_ns;
      sleep_until_ns(scheduled_tick_ns);
    }
  }

  void imu_worker() {
    std::string rt_err;
    if (!configure_current_thread_rt("imu", cfg_.threads.imu_priority, cfg_.threads.imu_cpu, &rt_err)) {
      std::cerr << "imu_worker RT warning: " << rt_err << "\n";
    }

    const uint64_t period_ns = hz_to_period_ns(cfg_.threads.imu_hz);
    const uint64_t miss_tolerance_ns = period_ns / 2U;
    uint64_t scheduled_tick_ns = monotonic_time_ns();
    uint64_t local_tick = 0;
    uint32_t consecutive_imu_failures = 0;
    ImuWatchdog imu_watchdog(cfg_.imu_watchdog);

    while (!stop_requested_.load(std::memory_order_relaxed)) {
      const uint64_t now_ns = monotonic_time_ns();
      const uint64_t jitter_ns = (now_ns >= scheduled_tick_ns) ? (now_ns - scheduled_tick_ns)
                                                                : (scheduled_tick_ns - now_ns);
      imu_jitter_tracker_.add(jitter_ns);
      if (now_ns > scheduled_tick_ns + miss_tolerance_ns) {
        bump_stat([](RuntimeStats& s) { ++s.imu_deadline_miss_count; });
      }

      try {
        ImuSample sample = imu_backend_->read_once(now_ns);
        if (!sample.t_ns) {
          sample.t_ns = now_ns;
        }

        const ImuWatchdogStepResult watchdog_result = imu_watchdog.on_sample(sample, now_ns);
        if (watchdog_result.force_invalid) {
          sample.valid = false;
        }

        if (watchdog_result.should_attempt_reinit) {
          bool reinit_ok = false;
          try {
            imu_backend_->stop();
            imu_backend_->start();
            reinit_ok = true;
          } catch (const std::exception&) {
            reinit_ok = false;
          }
          imu_watchdog.on_reinit_result(reinit_ok, now_ns);
          bump_stat([reinit_ok](RuntimeStats& s) {
            ++s.imu_reinit_attempt_count;
            if (reinit_ok) {
              ++s.imu_reinit_success_count;
            } else {
              ++s.imu_reinit_failure_count;
            }
          });
        }

        bump_stat([&](RuntimeStats& s) {
          s.imu_watchdog_fault_count = imu_watchdog.fault_count();
          s.imu_watchdog_zero_vector_count = imu_watchdog.zero_vector_count();
          s.imu_watchdog_flatline_count = imu_watchdog.flatline_count();
          s.imu_watchdog_degenerate_pattern_count = imu_watchdog.degenerate_pattern_count();
          s.imu_last_fault_reason = static_cast<uint32_t>(imu_watchdog.last_fault_reason());
          s.imu_watchdog_state = static_cast<uint32_t>(imu_watchdog.state());
        });

        consecutive_imu_failures = sample.valid ? 0 : (consecutive_imu_failures + 1U);
        bump_stat([consecutive_imu_failures](RuntimeStats& s) {
          s.imu_consecutive_failures = consecutive_imu_failures;
        });
        if (consecutive_imu_failures >= cfg_.timeouts.max_consecutive_imu_failures) {
          degraded_mode_active_.store(true, std::memory_order_relaxed);
          bump_stat([](RuntimeStats& s) { s.degraded_mode_active = true; });
        }

        std::lock_guard<std::mutex> lock(shared_.mutex);
        shared_.imu = sample;
      } catch (const std::exception&) {
        ++consecutive_imu_failures;
        ++sensor_health_.imu_read_errors;
        if (consecutive_imu_failures >= cfg_.timeouts.max_consecutive_imu_failures) {
          degraded_mode_active_.store(true, std::memory_order_relaxed);
          bump_stat([consecutive_imu_failures](RuntimeStats& s) {
            s.degraded_mode_active = true;
            s.imu_consecutive_failures = consecutive_imu_failures;
          });
        } else {
          bump_stat([consecutive_imu_failures](RuntimeStats& s) {
            s.imu_consecutive_failures = consecutive_imu_failures;
          });
        }
      }

      ++local_tick;
      bump_stat([](RuntimeStats& s) { ++s.imu_ticks; });
      if ((local_tick % 64ULL) == 0ULL) {
        const JitterSummary js = imu_jitter_tracker_.summary();
        bump_stat([&](RuntimeStats& s) {
          s.imu_jitter_p50_ns = js.p50_ns;
          s.imu_jitter_p95_ns = js.p95_ns;
          s.imu_jitter_p99_ns = js.p99_ns;
          s.imu_jitter_max_ns = js.max_ns;
        });
      }

      scheduled_tick_ns += period_ns;
      sleep_until_ns(scheduled_tick_ns);
    }
  }

  void i2c_hub_worker() {
    std::string rt_err;
    if (!configure_current_thread_rt("i2c-hub", cfg_.threads.i2c_priority, -1, &rt_err)) {
      std::cerr << "i2c_hub_worker RT warning: " << rt_err << "\n";
    }

    std::vector<I2cDeviceJob> jobs;
    jobs.push_back(I2cDeviceJob{
        "baro",
        static_cast<uint32_t>(std::max(1.0, 1000.0 / static_cast<double>(cfg_.threads.baro_hz))),
        100,
        monotonic_time_ns(),
        [this](uint64_t now_ns) {
          const BaroSample sample = baro_backend_->read_once(now_ns);
          std::lock_guard<std::mutex> lock(shared_.mutex);
          shared_.baro = sample;
        },
    });
    uint32_t baro_consecutive_errors = 0;
    uint64_t local_tick = 0;

    while (!stop_requested_.load(std::memory_order_relaxed)) {
      const uint64_t now_ns = monotonic_time_ns();

      std::vector<std::size_t> due;
      due.reserve(jobs.size());
      for (std::size_t i = 0; i < jobs.size(); ++i) {
        if (now_ns >= jobs[i].next_due_ns) {
          due.push_back(i);
        }
      }

      if (due.empty()) {
        uint64_t earliest_ns = UINT64_MAX;
        for (const auto& job : jobs) {
          earliest_ns = std::min(earliest_ns, job.next_due_ns);
        }
        if (earliest_ns == UINT64_MAX || earliest_ns <= now_ns) {
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        } else {
          sleep_until_ns(earliest_ns);
        }
        continue;
      }

      std::sort(due.begin(), due.end(), [&](std::size_t a, std::size_t b) {
        if (jobs[a].priority != jobs[b].priority) {
          return jobs[a].priority > jobs[b].priority;
        }
        return jobs[a].next_due_ns < jobs[b].next_due_ns;
      });

      for (std::size_t idx : due) {
        const uint64_t due_ns = jobs[idx].next_due_ns;
        const uint64_t jitter_ns = (now_ns >= due_ns) ? (now_ns - due_ns) : (due_ns - now_ns);
        baro_jitter_tracker_.add(jitter_ns);
        if (now_ns > due_ns + (ns_from_ms(jobs[idx].period_ms) / 2U)) {
          bump_stat([](RuntimeStats& s) { ++s.baro_deadline_miss_count; });
        }

        try {
          jobs[idx].poll_once(now_ns);
          if (jobs[idx].name == "baro") {
            baro_consecutive_errors = 0;
            bump_stat([](RuntimeStats& s) { s.baro_consecutive_failures = 0; });
          }
        } catch (const std::exception&) {
          ++sensor_health_.i2c_read_errors;
          if (jobs[idx].name == "baro") {
            ++baro_consecutive_errors;
            if (baro_consecutive_errors >= cfg_.timeouts.max_consecutive_baro_failures) {
              degraded_mode_active_.store(true, std::memory_order_relaxed);
              bump_stat([baro_consecutive_errors](RuntimeStats& s) {
                s.degraded_mode_active = true;
                s.baro_consecutive_failures = baro_consecutive_errors;
              });
            } else {
              bump_stat([baro_consecutive_errors](RuntimeStats& s) {
                s.baro_consecutive_failures = baro_consecutive_errors;
              });
            }
            if (baro_consecutive_errors >= cfg_.baro.recovery_error_threshold) {
              try {
                baro_backend_->stop();
              } catch (...) {
              }
              std::this_thread::sleep_for(std::chrono::milliseconds(cfg_.baro.recovery_backoff_ms));
              try {
                baro_backend_->start();
                baro_consecutive_errors = 0;
                bump_stat([](RuntimeStats& s) {
                  ++s.i2c_recovery_count;
                  s.baro_consecutive_failures = 0;
                });
              } catch (...) {
              }
            }
          }
        }

        jobs[idx].next_due_ns += ns_from_ms(jobs[idx].period_ms);
        if (jobs[idx].next_due_ns < now_ns) {
          jobs[idx].next_due_ns = now_ns + ns_from_ms(jobs[idx].period_ms);
        }
      }

      ++local_tick;
      bump_stat([](RuntimeStats& s) { ++s.i2c_ticks; });
      if ((local_tick % 32ULL) == 0ULL) {
        const JitterSummary js = baro_jitter_tracker_.summary();
        bump_stat([&](RuntimeStats& s) {
          s.baro_jitter_p50_ns = js.p50_ns;
          s.baro_jitter_p95_ns = js.p95_ns;
          s.baro_jitter_p99_ns = js.p99_ns;
          s.baro_jitter_max_ns = js.max_ns;
        });
      }
    }
  }

  void estimator_worker() {
    std::string rt_err;
    if (!configure_current_thread_rt("estimator", cfg_.threads.estimator_priority, -1, &rt_err)) {
      std::cerr << "estimator_thread RT warning: " << rt_err << "\n";
    }

    const uint64_t period_ns = hz_to_period_ns(cfg_.threads.estimator_hz);
    uint64_t next_tick = monotonic_time_ns();

    while (!stop_requested_.load(std::memory_order_relaxed)) {
      next_tick += period_ns;

      ImuSample imu;
      BaroSample baro;
      {
        std::lock_guard<std::mutex> lock(shared_.mutex);
        imu = shared_.imu;
        baro = shared_.baro;
      }

      EstimatorState state = cpp_estimator_.step(imu, baro);
      {
        std::lock_guard<std::mutex> lock(shared_.mutex);
        shared_.cpp_estimator = state;
      }

      bump_stat([](RuntimeStats& s) { ++s.estimator_ticks; });
      sleep_until_ns(next_tick);
    }
  }

  double effective_run_duration_s() const {
    if (overrides_.duration_sec_override >= 0.0) {
      return overrides_.duration_sec_override;
    }
    return cfg_.runtime.run_duration_s;
  }

  void maybe_write_status_file() const {
    if (overrides_.status_file.empty()) {
      return;
    }

    RuntimeStats s = stats_snapshot();
    const JitterSummary control_js = control_jitter_tracker_.summary();
    const JitterSummary actuator_js = actuator_jitter_tracker_.summary();
    const JitterSummary imu_js = imu_jitter_tracker_.summary();
    const JitterSummary baro_js = baro_jitter_tracker_.summary();
    s.control_jitter_p50_ns = control_js.p50_ns;
    s.control_jitter_p95_ns = control_js.p95_ns;
    s.control_jitter_p99_ns = control_js.p99_ns;
    s.control_jitter_max_ns = control_js.max_ns;
    s.actuator_jitter_p50_ns = actuator_js.p50_ns;
    s.actuator_jitter_p95_ns = actuator_js.p95_ns;
    s.actuator_jitter_p99_ns = actuator_js.p99_ns;
    s.actuator_jitter_max_ns = actuator_js.max_ns;
    s.imu_jitter_p50_ns = imu_js.p50_ns;
    s.imu_jitter_p95_ns = imu_js.p95_ns;
    s.imu_jitter_p99_ns = imu_js.p99_ns;
    s.imu_jitter_max_ns = imu_js.max_ns;
    s.baro_jitter_p50_ns = baro_js.p50_ns;
    s.baro_jitter_p95_ns = baro_js.p95_ns;
    s.baro_jitter_p99_ns = baro_js.p99_ns;
    s.baro_jitter_max_ns = baro_js.max_ns;

    std::ofstream out(overrides_.status_file);
    if (!out.is_open()) {
      std::cerr << "Failed to open status file: " << overrides_.status_file << "\n";
      return;
    }

    out << "sim_mode=" << (sim_mode_ ? "true" : "false") << "\n";
    out << "degraded_mode_active=" << (s.degraded_mode_active ? "true" : "false") << "\n";
    out << "control_ticks=" << s.control_ticks << "\n";
    out << "actuator_ticks=" << s.actuator_ticks << "\n";
    out << "igniter_ticks=" << s.igniter_ticks << "\n";
    out << "imu_ticks=" << s.imu_ticks << "\n";
    out << "i2c_ticks=" << s.i2c_ticks << "\n";
    out << "estimator_ticks=" << s.estimator_ticks << "\n";
    out << "external_estimator_accept_count=" << s.external_estimator_accept_count << "\n";
    out << "external_estimator_reject_count=" << s.external_estimator_reject_count << "\n";
    out << "external_controller_accept_count=" << s.external_controller_accept_count << "\n";
    out << "external_controller_reject_count=" << s.external_controller_reject_count << "\n";
    out << "igniter_command_accept_count=" << s.igniter_command_accept_count << "\n";
    out << "igniter_command_reject_count=" << s.igniter_command_reject_count << "\n";
    out << "failsafe_activation_count=" << s.failsafe_activation_count << "\n";
    out << "failsafe_enter_count=" << s.failsafe_enter_count << "\n";
    out << "failsafe_exit_count=" << s.failsafe_exit_count << "\n";
    out << "failsafe_cause_cmd_stale_count=" << s.failsafe_cause_cmd_stale_count << "\n";
    out << "failsafe_cause_link_down_count=" << s.failsafe_cause_link_down_count << "\n";
    out << "failsafe_cause_imu_stale_count=" << s.failsafe_cause_imu_stale_count << "\n";
    out << "failsafe_cause_killswitch_count=" << s.failsafe_cause_killswitch_count << "\n";
    out << "failsafe_cause_actuator_io_latch_count=" << s.failsafe_cause_actuator_io_latch_count << "\n";
    out << "last_failsafe_reason=" << s.last_failsafe_reason << "\n";
    out << "last_failsafe_reason_name="
        << failsafe_reason_name(static_cast<FailsafeReason>(s.last_failsafe_reason)) << "\n";
    out << "control_deadline_miss_count=" << s.control_deadline_miss_count << "\n";
    out << "actuator_deadline_miss_count=" << s.actuator_deadline_miss_count << "\n";
    out << "imu_deadline_miss_count=" << s.imu_deadline_miss_count << "\n";
    out << "baro_deadline_miss_count=" << s.baro_deadline_miss_count << "\n";
    out << "control_jitter_p50_ns=" << s.control_jitter_p50_ns << "\n";
    out << "control_jitter_p95_ns=" << s.control_jitter_p95_ns << "\n";
    out << "control_jitter_p99_ns=" << s.control_jitter_p99_ns << "\n";
    out << "control_jitter_max_ns=" << s.control_jitter_max_ns << "\n";
    out << "actuator_jitter_p50_ns=" << s.actuator_jitter_p50_ns << "\n";
    out << "actuator_jitter_p95_ns=" << s.actuator_jitter_p95_ns << "\n";
    out << "actuator_jitter_p99_ns=" << s.actuator_jitter_p99_ns << "\n";
    out << "actuator_jitter_max_ns=" << s.actuator_jitter_max_ns << "\n";
    out << "imu_jitter_p50_ns=" << s.imu_jitter_p50_ns << "\n";
    out << "imu_jitter_p95_ns=" << s.imu_jitter_p95_ns << "\n";
    out << "imu_jitter_p99_ns=" << s.imu_jitter_p99_ns << "\n";
    out << "imu_jitter_max_ns=" << s.imu_jitter_max_ns << "\n";
    out << "baro_jitter_p50_ns=" << s.baro_jitter_p50_ns << "\n";
    out << "baro_jitter_p95_ns=" << s.baro_jitter_p95_ns << "\n";
    out << "baro_jitter_p99_ns=" << s.baro_jitter_p99_ns << "\n";
    out << "baro_jitter_max_ns=" << s.baro_jitter_max_ns << "\n";
    out << "actuator_cmd_age_last_ns=" << s.actuator_cmd_age_last_ns << "\n";
    out << "actuator_cmd_age_max_ns=" << s.actuator_cmd_age_max_ns << "\n";
    out << "i2c_recovery_count=" << s.i2c_recovery_count << "\n";
    out << "imu_watchdog_fault_count=" << s.imu_watchdog_fault_count << "\n";
    out << "imu_watchdog_zero_vector_count=" << s.imu_watchdog_zero_vector_count << "\n";
    out << "imu_watchdog_flatline_count=" << s.imu_watchdog_flatline_count << "\n";
    out << "imu_watchdog_degenerate_pattern_count=" << s.imu_watchdog_degenerate_pattern_count << "\n";
    out << "imu_reinit_attempt_count=" << s.imu_reinit_attempt_count << "\n";
    out << "imu_reinit_success_count=" << s.imu_reinit_success_count << "\n";
    out << "imu_reinit_failure_count=" << s.imu_reinit_failure_count << "\n";
    out << "imu_last_fault_reason=" << s.imu_last_fault_reason << "\n";
    out << "imu_last_fault_reason_name="
        << imu_fault_reason_name(static_cast<ImuFaultReason>(s.imu_last_fault_reason)) << "\n";
    out << "imu_watchdog_state=" << s.imu_watchdog_state << "\n";
    out << "imu_watchdog_state_name="
        << imu_watchdog_state_name(static_cast<ImuWatchdogState>(s.imu_watchdog_state)) << "\n";
    out << "imu_consecutive_failures=" << s.imu_consecutive_failures << "\n";
    out << "baro_consecutive_failures=" << s.baro_consecutive_failures << "\n";
    out << "ipc_permission_reject_count=" << s.ipc_permission_reject_count << "\n";
    out << "external_msg_stale_reject_count=" << s.external_msg_stale_reject_count << "\n";
    out << "external_msg_schema_reject_count=" << s.external_msg_schema_reject_count << "\n";
    out << "external_msg_crc_reject_count=" << s.external_msg_crc_reject_count << "\n";
    out << "external_msg_time_regression_reject_count=" << s.external_msg_time_regression_reject_count
        << "\n";
    out << "killswitch_active=" << (s.killswitch_active ? "true" : "false") << "\n";
    out << "killswitch_trip_count=" << s.killswitch_trip_count << "\n";
    out << "igniter_armed=" << (s.igniter_armed ? "true" : "false") << "\n";
    out << "igniter_global_fault_latched=" << (s.igniter_global_fault_latched ? "true" : "false")
        << "\n";
    out << "igniter_active_mask=" << static_cast<uint32_t>(s.igniter_active_mask) << "\n";
    out << "sim_net_sensor_frames=" << s.sim_net_sensor_frames << "\n";
    out << "sim_net_sensor_crc_fail=" << s.sim_net_sensor_crc_fail << "\n";
    out << "sim_net_sensor_disconnects=" << s.sim_net_sensor_disconnects << "\n";
    out << "sim_net_actuator_frames=" << s.sim_net_actuator_frames << "\n";
    out << "sim_net_actuator_send_errors=" << s.sim_net_actuator_send_errors << "\n";
    out << "sim_net_actuator_clients=" << s.sim_net_actuator_clients << "\n";
    out << "sim_net_actuator_disconnects=" << s.sim_net_actuator_disconnects << "\n";
    out << "sim_net_actuator_client_connected="
        << (s.sim_net_actuator_client_connected ? "true" : "false") << "\n";
  }

  template <typename Fn>
  void bump_stat(Fn&& fn) {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    fn(stats_);
  }

  RuntimeConfig cfg_;
  RuntimeCliOverrides overrides_;

  std::atomic<bool> stop_requested_{false};
  bool sim_mode_;
  std::atomic<bool> degraded_mode_active_{false};

  SharedState shared_;
  SensorHealth sensor_health_{};

  ShmMailbox<SensorSnapshotMsg> sensor_snapshot_mailbox_;
  ShmMailbox<ExternalEstimatorStateMsg> estimator_state_mailbox_;
  ShmMailbox<ExternalControllerCommandMsg> controller_cmd_mailbox_;
  ShmMailbox<IgniterCommandMsg> igniter_cmd_mailbox_;
  ShmMailbox<IgniterStatusMsg> igniter_status_mailbox_;

  std::unique_ptr<ImuBackend> imu_backend_;
  std::unique_ptr<BaroBackend> baro_backend_;
  std::unique_ptr<ActuatorBackend> actuator_backend_;
  std::unique_ptr<SimNetLink> sim_net_link_;
#if defined(RUNTIME_HAVE_IGNITER) && (RUNTIME_HAVE_IGNITER == 1)
  std::unique_ptr<igniter::IgniterBank> igniter_bank_;
#endif

  CppEstimator cpp_estimator_;
  CppController cpp_controller_;
  ActuatorMapper actuator_mapper_;

  EstimatorSelectionState estimator_selection_state_{};
  ControllerSelectionState controller_selection_state_{};

  ControlCommand failsafe_cmd_;
  KillSwitchMonitor kill_switch_;
  std::atomic<bool> kill_switch_active_{false};
  std::atomic<bool> kill_switch_read_error_logged_{false};

  JitterTracker control_jitter_tracker_;
  JitterTracker actuator_jitter_tracker_;
  JitterTracker imu_jitter_tracker_;
  JitterTracker baro_jitter_tracker_;

  mutable std::mutex stats_mutex_;
  RuntimeStats stats_{};

  std::vector<std::thread> workers_;
};

Runtime::Runtime(RuntimeConfig cfg, RuntimeCliOverrides overrides)
    : impl_(std::make_unique<Impl>(std::move(cfg), std::move(overrides))) {}

Runtime::~Runtime() {
  if (impl_) {
    impl_->request_stop();
  }
}

void Runtime::run() { impl_->run(); }

void Runtime::request_stop() { impl_->request_stop(); }

RuntimeStats Runtime::stats_snapshot() const { return impl_->stats_snapshot(); }

}  // namespace runtime
