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

bool validate_estimator_message(const runtime::PyEstimatorStateMsg& msg) {
  if (!validate_header(msg.msg_magic, msg.msg_version, msg.payload_bytes,
                       runtime::payload_size_bytes<runtime::PyEstimatorStateMsg>())) {
    return false;
  }
  if (!runtime::validate_message_crc(msg)) {
    return false;
  }
  if (msg.valid == 0) {
    return false;
  }
  const double q_norm = std::sqrt(static_cast<double>(msg.q_body_to_ned[0]) * msg.q_body_to_ned[0] +
                                  static_cast<double>(msg.q_body_to_ned[1]) * msg.q_body_to_ned[1] +
                                  static_cast<double>(msg.q_body_to_ned[2]) * msg.q_body_to_ned[2] +
                                  static_cast<double>(msg.q_body_to_ned[3]) * msg.q_body_to_ned[3]);
  return q_norm >= 0.85 && q_norm <= 1.15;
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
    pitch = std::copysign(3.14159265358979323846 / 2.0, sinp);
  } else {
    pitch = std::asin(sinp);
  }
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

    runtime::ShmMailbox<runtime::PyEstimatorStateMsg> estimator_mailbox(cfg.ipc.estimator_state_shm, false);
    runtime::ShmMailbox<runtime::PyControllerCommandMsg> controller_mailbox(cfg.ipc.controller_command_shm, false);
    open_mailbox_with_retry(estimator_mailbox, cfg.ipc, cfg.ipc.estimator_state_shm);
    open_mailbox_with_retry(controller_mailbox, cfg.ipc, cfg.ipc.controller_command_shm);

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    const uint64_t period_ns = runtime::hz_to_period_ns(cfg.threads.control_hz);
    uint64_t next_tick_ns = runtime::monotonic_time_ns();
    const uint64_t deadline_ns = (duration_sec > 0.0)
                                     ? (runtime::monotonic_time_ns() + static_cast<uint64_t>(duration_sec * 1e9))
                                     : 0;

    uint64_t estimator_seq_seen = 0;
    uint64_t controller_seq = 0;
    double roll = 0.0;
    double pitch = 0.0;

    while (!g_stop.load(std::memory_order_relaxed)) {
      if (deadline_ns > 0 && runtime::monotonic_time_ns() >= deadline_ns) {
        break;
      }

      next_tick_ns += period_ns;

      runtime::PyEstimatorStateMsg est_msg{};
      uint64_t stable_seq = 0;
      if (estimator_mailbox.try_read(est_msg, &stable_seq) && stable_seq != estimator_seq_seen) {
        estimator_seq_seen = stable_seq;
        if (validate_estimator_message(est_msg)) {
          quat_to_roll_pitch(est_msg.q_body_to_ned, roll, pitch);
        }
      }

      const auto clamp = [](double x, double lo, double hi) {
        return std::max(lo, std::min(hi, x));
      };

      const double cmd_roll = clamp(-1.1 * roll, -0.8, 0.8);
      const double cmd_pitch = clamp(-1.1 * pitch, -0.8, 0.8);

      const double s0 = clamp(cmd_roll - cmd_pitch, -1.0, 1.0);
      const double s1 = clamp(-cmd_roll - cmd_pitch, -1.0, 1.0);
      const double s2 = clamp(cmd_roll + cmd_pitch, -1.0, 1.0);
      const double s3 = clamp(-cmd_roll + cmd_pitch, -1.0, 1.0);

      runtime::PyControllerCommandMsg out{};
      runtime::fill_message_header(out, ++controller_seq, runtime::monotonic_time_ns());
      out.armed = 1;
      out.servo_norm = {static_cast<float>(s0), static_cast<float>(s1), static_cast<float>(s2), static_cast<float>(s3)};
      runtime::finalize_message_crc(out);
      controller_mailbox.write(out);

      runtime::sleep_until_ns(next_tick_ns);
    }

    return 0;
  } catch (const std::exception& ex) {
    std::cerr << "dummy_controller_cpp failed: " << ex.what() << "\n";
    return 1;
  }
}
