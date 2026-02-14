#include "runtime/common/time.hpp"
#include "runtime/config/config.hpp"
#include "runtime/ipc/messages.hpp"
#include "runtime/ipc/shm_mailbox.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>
#include <thread>

namespace {

std::atomic<bool> g_stop{false};

void signal_handler(int) { g_stop.store(true, std::memory_order_relaxed); }

void print_usage(const char* argv0) {
  std::cerr << "Usage: " << argv0 << " --config <path> [--duration-sec <seconds>]\n";
}

bool validate_header(uint32_t msg_magic, uint16_t version, uint16_t payload_bytes, uint16_t expected_payload) {
  return msg_magic == runtime::kMessageMagic && version == runtime::kMessageVersion && payload_bytes == expected_payload;
}

bool is_finite(double x) { return std::isfinite(x); }

bool validate_sensor_snapshot(const runtime::SensorSnapshotMsg& msg) {
  if (!validate_header(msg.msg_magic, msg.msg_version, msg.payload_bytes,
                       runtime::payload_size_bytes<runtime::SensorSnapshotMsg>())) {
    return false;
  }
  if (!runtime::validate_message_crc(msg)) {
    return false;
  }
  return is_finite(msg.ax_mps2) && is_finite(msg.ay_mps2) && is_finite(msg.az_mps2) && is_finite(msg.gx_rads) &&
         is_finite(msg.gy_rads) && is_finite(msg.gz_rads);
}

template <typename Mailbox>
void open_mailbox_with_retry(Mailbox& mailbox, const runtime::IpcSection& cfg, const std::string& name) {
  std::exception_ptr last_error;
  const uint32_t attempts = std::max<uint32_t>(1, cfg.open_retry_count);
  for (uint32_t i = 0; i < attempts; ++i) {
    try {
      mailbox.open(false);
      return;
    } catch (...) {
      last_error = std::current_exception();
      std::this_thread::sleep_for(std::chrono::milliseconds(cfg.open_retry_ms));
    }
  }

  std::cerr << "Failed to open mailbox '" << name << "' after " << attempts << " attempts\n";
  if (last_error) {
    std::rethrow_exception(last_error);
  }
  throw std::runtime_error("Failed to open mailbox");
}

std::array<float, 4> quat_from_euler(double roll, double pitch, double yaw) {
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

}  // namespace

int main(int argc, char** argv) {
  std::string config_path;
  double duration_sec = 0.0;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--config" && i + 1 < argc) {
      config_path = argv[++i];
      continue;
    }
    if (arg == "--duration-sec" && i + 1 < argc) {
      duration_sec = std::stod(argv[++i]);
      continue;
    }
    print_usage(argv[0]);
    return 1;
  }

  if (config_path.empty()) {
    print_usage(argv[0]);
    return 1;
  }

  try {
    const runtime::RuntimeConfig cfg = runtime::load_runtime_config(config_path);

    runtime::ShmMailbox<runtime::SensorSnapshotMsg> sensor_mailbox(cfg.ipc.sensor_snapshot_shm, false);
    runtime::ShmMailbox<runtime::ExternalEstimatorStateMsg> estimator_mailbox(cfg.ipc.estimator_state_shm, false);
    open_mailbox_with_retry(sensor_mailbox, cfg.ipc, cfg.ipc.sensor_snapshot_shm);
    open_mailbox_with_retry(estimator_mailbox, cfg.ipc, cfg.ipc.estimator_state_shm);

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    const uint64_t period_ns = runtime::hz_to_period_ns(cfg.threads.estimator_hz);
    uint64_t next_tick_ns = runtime::monotonic_time_ns();
    const uint64_t deadline_ns = (duration_sec > 0.0)
                                     ? (runtime::monotonic_time_ns() + static_cast<uint64_t>(duration_sec * 1e9))
                                     : 0;

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    uint64_t last_sensor_t_ns = 0;
    uint64_t sensor_seq_seen = 0;
    uint64_t estimator_seq = 0;

    while (!g_stop.load(std::memory_order_relaxed)) {
      if (deadline_ns > 0 && runtime::monotonic_time_ns() >= deadline_ns) {
        break;
      }

      next_tick_ns += period_ns;
      runtime::SensorSnapshotMsg sensor_msg{};
      uint64_t stable_seq = 0;
      if (sensor_mailbox.try_read(sensor_msg, &stable_seq) && stable_seq != sensor_seq_seen) {
        sensor_seq_seen = stable_seq;
        if (validate_sensor_snapshot(sensor_msg) && sensor_msg.imu_valid != 0 && sensor_msg.t_ns > 0) {
          double dt = static_cast<double>(period_ns) * 1e-9;
          if (last_sensor_t_ns > 0 && sensor_msg.t_ns > last_sensor_t_ns) {
            dt = static_cast<double>(sensor_msg.t_ns - last_sensor_t_ns) * 1e-9;
          }
          last_sensor_t_ns = sensor_msg.t_ns;

          roll += sensor_msg.gx_rads * dt;
          pitch += sensor_msg.gy_rads * dt;
          yaw += sensor_msg.gz_rads * dt;

          const double accel_roll = std::atan2(sensor_msg.ay_mps2, sensor_msg.az_mps2);
          const double accel_pitch =
              std::atan2(-sensor_msg.ax_mps2,
                         std::sqrt(sensor_msg.ay_mps2 * sensor_msg.ay_mps2 + sensor_msg.az_mps2 * sensor_msg.az_mps2));
          constexpr double kAlpha = 0.03;
          roll = (1.0 - kAlpha) * roll + kAlpha * accel_roll;
          pitch = (1.0 - kAlpha) * pitch + kAlpha * accel_pitch;
        }
      }

      runtime::ExternalEstimatorStateMsg out{};
      runtime::fill_message_header(out, ++estimator_seq, runtime::monotonic_time_ns());
      out.valid = 1;
      out.q_body_to_ned = quat_from_euler(roll, pitch, yaw);
      out.vel_ned_mps = {0.0F, 0.0F, 0.0F};
      out.pos_ned_m = {0.0F, 0.0F, 0.0F};
      runtime::finalize_message_crc(out);
      estimator_mailbox.write(out);

      runtime::sleep_until_ns(next_tick_ns);
    }

    return 0;
  } catch (const std::exception& ex) {
    std::cerr << "dummy_estimator_cpp failed: " << ex.what() << "\n";
    return 1;
  }
}
