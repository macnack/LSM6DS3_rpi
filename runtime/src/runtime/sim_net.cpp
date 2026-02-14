#include "runtime/runtime/sim_net.hpp"

#include "runtime/common/time.hpp"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <stdexcept>

namespace runtime {

namespace {

constexpr uint32_t kNetVersion = 1;
constexpr uint32_t kSensorMagic = 0x53594D53;   // "SYMS"
constexpr uint32_t kActuatorMagic = 0x41594D53; // "SYMA"
constexpr uint16_t kSensorPayloadBytes = 92;
constexpr uint16_t kActuatorPayloadBytes = 44;
constexpr std::size_t kHeaderBytes = 12;

sockaddr_in make_addr(const std::string& host, uint16_t port) {
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  if (inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) {
    throw std::runtime_error("sim_net: invalid IPv4 host: " + host);
  }
  return addr;
}

int set_nonblock(int fd) {
  int flags = fcntl(fd, F_GETFL, 0);
  if (flags < 0) {
    return -1;
  }
  return fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}

bool write_full(int fd, const uint8_t* data, std::size_t len) {
  std::size_t sent = 0;
  while (sent < len) {
    const ssize_t n = ::send(fd, data + sent, len - sent, 0);
    if (n < 0) {
      if (errno == EINTR) {
        continue;
      }
      return false;
    }
    if (n == 0) {
      return false;
    }
    sent += static_cast<std::size_t>(n);
  }
  return true;
}

uint32_t crc32_payload(const std::vector<uint8_t>& payload) {
  if (payload.size() < 4) {
    return 0;
  }
  return crc32_ieee(payload.data(), payload.size() - 4);
}

}  // namespace

SimNetLink::SimNetLink(const SimNetSection& cfg) : cfg_(cfg) {}

SimNetLink::~SimNetLink() { stop(); }

void SimNetLink::start() {
  if (started_.exchange(true, std::memory_order_relaxed)) {
    return;
  }

  // Actuator listen socket (non-blocking)
  listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
  if (listen_fd_ < 0) {
    throw std::runtime_error("sim_net: failed to create actuator listen socket");
  }
  int enable = 1;
  ::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable));
  const auto bind_addr = make_addr(cfg_.actuator_bind_host, cfg_.actuator_port);
  if (::bind(listen_fd_, reinterpret_cast<const sockaddr*>(&bind_addr), sizeof(bind_addr)) < 0) {
    ::close(listen_fd_);
    listen_fd_ = -1;
    throw std::runtime_error("sim_net: failed to bind actuator socket");
  }
  if (::listen(listen_fd_, 1) < 0) {
    ::close(listen_fd_);
    listen_fd_ = -1;
    throw std::runtime_error("sim_net: listen failed");
  }
  (void)set_nonblock(listen_fd_);

  // Sensor client thread
  sensor_thread_ = std::thread([this] { sensor_thread(); });
}

void SimNetLink::stop() noexcept {
  stop_.store(true, std::memory_order_relaxed);
  if (sensor_thread_.joinable()) {
    sensor_thread_.join();
  }
  if (client_fd_ >= 0) {
    ::close(client_fd_);
    client_fd_ = -1;
  }
  if (listen_fd_ >= 0) {
    ::close(listen_fd_);
    listen_fd_ = -1;
  }
}

bool SimNetLink::connect_sensor_fd(int& fd_out) {
  fd_out = ::socket(AF_INET, SOCK_STREAM, 0);
  if (fd_out < 0) {
    return false;
  }
  const auto addr = make_addr(cfg_.sensor_host, cfg_.sensor_port);
  // Non-blocking connect with timeout
  (void)set_nonblock(fd_out);
  const int rc = ::connect(fd_out, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr));
  if (rc == 0) {
    return true;
  }
  if (errno != EINPROGRESS) {
    ::close(fd_out);
    fd_out = -1;
    return false;
  }

  fd_set wfds;
  FD_ZERO(&wfds);
  FD_SET(fd_out, &wfds);
  timeval tv{};
  tv.tv_sec = cfg_.connect_timeout_ms / 1000;
  tv.tv_usec = (cfg_.connect_timeout_ms % 1000) * 1000;
  const int sel = ::select(fd_out + 1, nullptr, &wfds, nullptr, &tv);
  if (sel <= 0) {
    ::close(fd_out);
    fd_out = -1;
    return false;
  }
  int err = 0;
  socklen_t len = sizeof(err);
  if (::getsockopt(fd_out, SOL_SOCKET, SO_ERROR, &err, &len) < 0 || err != 0) {
    ::close(fd_out);
    fd_out = -1;
    return false;
  }
  // Back to blocking for read loop
  int flags = fcntl(fd_out, F_GETFL, 0);
  if (flags >= 0) {
    fcntl(fd_out, F_SETFL, flags & ~O_NONBLOCK);
  }
  return true;
}

bool SimNetLink::read_exact(int fd, std::vector<uint8_t>& buf, std::size_t bytes) {
  buf.resize(bytes);
  std::size_t got = 0;
  while (got < bytes && !stop_.load(std::memory_order_relaxed)) {
    const ssize_t n = ::recv(fd, buf.data() + got, bytes - got, 0);
    if (n < 0) {
      if (errno == EINTR) {
        continue;
      }
      return false;
    }
    if (n == 0) {
      return false;
    }
    got += static_cast<std::size_t>(n);
  }
  return got == bytes;
}

void SimNetLink::sensor_thread() {
  while (!stop_.load(std::memory_order_relaxed)) {
    int fd = -1;
    if (!connect_sensor_fd(fd)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      continue;
    }

    while (!stop_.load(std::memory_order_relaxed)) {
      std::vector<uint8_t> header;
      if (!read_exact(fd, header, kHeaderBytes)) {
        break;
      }
      uint32_t magic = 0;
      uint16_t version = 0;
      uint16_t _reserved = 0;
      uint32_t payload_bytes = 0;
      std::memcpy(&magic, header.data(), sizeof(uint32_t));
      std::memcpy(&version, header.data() + 4, sizeof(uint16_t));
      std::memcpy(&_reserved, header.data() + 6, sizeof(uint16_t));
      std::memcpy(&payload_bytes, header.data() + 8, sizeof(uint32_t));
      (void)_reserved;

      if (magic != kSensorMagic || version != kNetVersion || payload_bytes != kSensorPayloadBytes) {
        break;
      }

      std::vector<uint8_t> payload;
      if (!read_exact(fd, payload, payload_bytes)) {
        break;
      }

      uint32_t crc_rx = 0;
      std::memcpy(&crc_rx, payload.data() + payload_bytes - 4, sizeof(uint32_t));
      const uint32_t crc_calc = crc32_payload(payload);
      if (crc_rx != crc_calc) {
        {
          std::lock_guard<std::mutex> lock(stats_mutex_);
          ++stats_.sensor_crc_fail;
        }
        continue;
      }

      NetSensorSample sample{};
      std::size_t off = 0;
      auto read_u64 = [&](uint64_t& out) {
        std::memcpy(&out, payload.data() + off, sizeof(uint64_t));
        off += sizeof(uint64_t);
      };
      read_u64(sample.seq);
      read_u64(sample.t_ns);
      sample.imu_valid = payload[off++] != 0;
      sample.baro_valid = payload[off++] != 0;
      off += 6;  // padding
      std::memcpy(&sample.ax, payload.data() + off, sizeof(double) * 6);
      off += sizeof(double) * 6;
      std::memcpy(&sample.pressure_pa, payload.data() + off, sizeof(double));
      off += sizeof(double);
      std::memcpy(&sample.temperature_c, payload.data() + off, sizeof(double));
      sample.recv_ns = monotonic_time_ns();

      {
        std::lock_guard<std::mutex> lock(sensor_mutex_);
        latest_sensor_ = sample;
        has_sensor_ = true;
      }
      {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        ++stats_.sensor_frames;
      }
    }

    {
      std::lock_guard<std::mutex> lock(stats_mutex_);
      ++stats_.sensor_disconnects;
    }
    if (fd >= 0) {
      ::close(fd);
      fd = -1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
}

bool SimNetLink::latest_sensor(NetSensorSample& out) const {
  std::lock_guard<std::mutex> lock(sensor_mutex_);
  if (!has_sensor_) {
    return false;
  }
  out = latest_sensor_;
  return true;
}

void SimNetLink::ensure_accept_client() {
  if (listen_fd_ < 0 || client_fd_ >= 0) {
    return;
  }
  sockaddr_in addr{};
  socklen_t len = sizeof(addr);
  const int fd = ::accept(listen_fd_, reinterpret_cast<sockaddr*>(&addr), &len);
  if (fd >= 0) {
    client_fd_ = fd;
    std::lock_guard<std::mutex> lock(stats_mutex_);
    ++stats_.actuator_clients;
  }
}

void SimNetLink::close_client() {
  if (client_fd_ >= 0) {
    ::close(client_fd_);
    client_fd_ = -1;
  }
}

void SimNetLink::publish_actuator(const ControlCommand& cmd, uint64_t now_ns) {
  ensure_accept_client();
  if (client_fd_ < 0) {
    return;
  }

  uint64_t seq = 0;
  {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    seq = stats_.actuator_frames + 1;
  }

  std::vector<uint8_t> payload(kActuatorPayloadBytes, 0);
  std::size_t off = 0;
  auto write_u64 = [&](uint64_t v) {
    std::memcpy(payload.data() + off, &v, sizeof(uint64_t));
    off += sizeof(uint64_t);
  };
  write_u64(seq);
  write_u64(now_ns);
  payload[off++] = cmd.armed ? 1U : 0U;
  off += 7;  // padding
  for (std::size_t i = 0; i < kServoCount; ++i) {
    float v = static_cast<float>(cmd.servo_norm[i]);
    std::memcpy(payload.data() + off, &v, sizeof(float));
    off += sizeof(float);
  }
  const uint32_t crc = crc32_payload(payload);
  std::memcpy(payload.data() + kActuatorPayloadBytes - 4, &crc, sizeof(uint32_t));

  std::vector<uint8_t> header(kHeaderBytes, 0);
  std::memcpy(header.data(), &kActuatorMagic, sizeof(uint32_t));
  uint16_t version = kNetVersion;
  uint16_t reserved = 0;
  uint32_t payload_bytes = kActuatorPayloadBytes;
  std::memcpy(header.data() + 4, &version, sizeof(uint16_t));
  std::memcpy(header.data() + 6, &reserved, sizeof(uint16_t));
  std::memcpy(header.data() + 8, &payload_bytes, sizeof(uint32_t));

  if (!write_full(client_fd_, header.data(), header.size()) ||
      !write_full(client_fd_, payload.data(), payload.size())) {
    {
      std::lock_guard<std::mutex> lock(stats_mutex_);
      ++stats_.actuator_send_errors;
    }
    close_client();
    return;
  }

  {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    ++stats_.actuator_frames;
  }
}

SimNetStats SimNetLink::stats_snapshot() const {
  std::lock_guard<std::mutex> lock(stats_mutex_);
  return stats_;
}

NetImuBackend::NetImuBackend(SimNetLink& link, const TimeoutsSection& timeouts)
    : link_(link), timeouts_(timeouts) {}

ImuSample NetImuBackend::read_once(uint64_t now_ns) {
  NetSensorSample sample{};
  ImuSample out{};
  if (!link_.latest_sensor(sample)) {
    return out;
  }
  const uint64_t age_ns = (now_ns > sample.t_ns) ? (now_ns - sample.t_ns) : 0;
  if (!sample.imu_valid || age_ns > ns_from_ms(timeouts_.imu_stale_ms)) {
    return out;
  }

  out.t_ns = sample.t_ns;
  out.ax_mps2 = sample.ax;
  out.ay_mps2 = sample.ay;
  out.az_mps2 = sample.az;
  out.gx_rads = sample.gx;
  out.gy_rads = sample.gy;
  out.gz_rads = sample.gz;
  out.valid = true;
  return out;
}

NetBaroBackend::NetBaroBackend(SimNetLink& link, const TimeoutsSection& timeouts)
    : link_(link), timeouts_(timeouts) {}

BaroSample NetBaroBackend::read_once(uint64_t now_ns) {
  NetSensorSample sample{};
  BaroSample out{};
  if (!link_.latest_sensor(sample)) {
    return out;
  }
  const uint64_t age_ns = (now_ns > sample.t_ns) ? (now_ns - sample.t_ns) : 0;
  if (!sample.baro_valid || age_ns > ns_from_ms(timeouts_.baro_stale_ms)) {
    return out;
  }

  out.t_ns = sample.t_ns;
  out.pressure_pa = sample.pressure_pa;
  out.temperature_c = sample.temperature_c;
  out.valid = true;
  return out;
}

}  // namespace runtime
