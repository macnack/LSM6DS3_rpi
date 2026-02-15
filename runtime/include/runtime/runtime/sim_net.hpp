#pragma once

#include "runtime/common/types.hpp"
#include "runtime/config/config.hpp"
#include "runtime/ipc/crc32.hpp"
#include "runtime/runtime/sensors.hpp"

#include <atomic>
#include <cstdint>
#include <mutex>
#include <thread>
#include <vector>

namespace runtime {

struct NetSensorSample {
  uint64_t seq = 0;
  uint64_t t_ns = 0;
  bool imu_valid = false;
  bool baro_valid = false;
  double ax = 0.0;
  double ay = 0.0;
  double az = 0.0;
  double gx = 0.0;
  double gy = 0.0;
  double gz = 0.0;
  double pressure_pa = 101325.0;
  double temperature_c = 20.0;
  uint64_t recv_ns = 0;
};

struct SimNetStats {
  uint64_t sensor_frames = 0;
  uint64_t sensor_crc_fail = 0;
  uint64_t sensor_disconnects = 0;
  uint64_t actuator_frames = 0;
  uint64_t actuator_send_errors = 0;
  uint64_t actuator_clients = 0;
  uint64_t actuator_disconnects = 0;
  bool actuator_client_connected = false;
};

class SimNetLink {
 public:
  explicit SimNetLink(const SimNetSection& cfg);
  ~SimNetLink();

  void start();
  void stop() noexcept;
  bool started() const { return started_.load(std::memory_order_relaxed); }

  bool latest_sensor(NetSensorSample& out) const;
  void publish_actuator(const ControlCommand& cmd, uint64_t now_ns);
  bool actuator_client_connected() const;

  SimNetStats stats_snapshot() const;

 private:
  void sensor_thread();
  bool connect_sensor_fd(int& fd_out);
  bool read_exact(int fd, std::vector<uint8_t>& buf, std::size_t bytes);

  void ensure_accept_client();
  void close_client(bool count_disconnect = true);

  SimNetSection cfg_;
  std::atomic<bool> stop_{false};
  std::atomic<bool> started_{false};

  // Sensor side
  std::thread sensor_thread_;
  mutable std::mutex sensor_mutex_;
  NetSensorSample latest_sensor_{};
  bool has_sensor_ = false;
  mutable std::mutex stats_mutex_;
  SimNetStats stats_{};

  // Actuator side
  int listen_fd_ = -1;
  int client_fd_ = -1;
};

class NetImuBackend final : public ImuBackend {
 public:
  NetImuBackend(SimNetLink& link, const TimeoutsSection& timeouts);
  void start() override {}
  void stop() noexcept override {}
  ImuSample read_once(uint64_t now_ns) override;

 private:
  SimNetLink& link_;
  TimeoutsSection timeouts_;
};

class NetBaroBackend final : public BaroBackend {
 public:
  NetBaroBackend(SimNetLink& link, const TimeoutsSection& timeouts);
  void start() override {}
  void stop() noexcept override {}
  BaroSample read_once(uint64_t now_ns) override;

 private:
  SimNetLink& link_;
  TimeoutsSection timeouts_;
};

}  // namespace runtime
