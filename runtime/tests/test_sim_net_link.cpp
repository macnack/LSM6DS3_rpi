#include "runtime/common/time.hpp"
#include "runtime/runtime/sim_net.hpp"

#include <arpa/inet.h>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <iostream>
#include <memory>
#include <netinet/in.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

namespace {

#define REQUIRE(cond, msg)      \
  do {                          \
    if (!(cond)) {              \
      std::cerr << msg << "\n"; \
      return false;             \
    }                           \
  } while (0)

using namespace runtime;

uint16_t pick_unused_port() {
  const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) {
    return 0;
  }
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = 0;
  if (::bind(fd, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) != 0) {
    (void)::close(fd);
    return 0;
  }
  sockaddr_in bound{};
  socklen_t len = sizeof(bound);
  if (::getsockname(fd, reinterpret_cast<sockaddr*>(&bound), &len) != 0) {
    (void)::close(fd);
    return 0;
  }
  const uint16_t port = ntohs(bound.sin_port);
  (void)::close(fd);
  return port;
}

int connect_with_retry(uint16_t port, std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    if (::inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr) != 1) {
      (void)::close(fd);
      return -1;
    }
    if (::connect(fd, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) == 0) {
      return fd;
    }
    (void)::close(fd);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return -1;
}

bool test_actuator_disconnect_metrics() {
  SimNetSection cfg{};
  cfg.enabled = true;
  cfg.sensor_host = "127.0.0.1";
  cfg.actuator_bind_host = "127.0.0.1";
  cfg.connect_timeout_ms = 100;
  uint16_t actuator_port = 0;
  std::unique_ptr<SimNetLink> link;
  for (int attempt = 0; attempt < 16; ++attempt) {
    uint16_t sensor_port = pick_unused_port();
    if (sensor_port == 0) {
      sensor_port = static_cast<uint16_t>(56123 + attempt * 2);
    }
    actuator_port = pick_unused_port();
    if (actuator_port == 0 || actuator_port == sensor_port) {
      actuator_port =
          static_cast<uint16_t>((sensor_port < 65535U) ? (sensor_port + 1U) : (sensor_port - 1U));
    }
    cfg.sensor_port = sensor_port;
    cfg.actuator_port = actuator_port;
    try {
      link = std::make_unique<SimNetLink>(cfg);
      link->start();
      break;
    } catch (const std::exception&) {
      link.reset();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  if (link == nullptr) {
    std::cout << "runtime_unit_sim_net_link: skipped (unable to bind local test socket)\n";
    return true;
  }

  int client_fd = -1;

  auto cleanup = [&]() {
    if (client_fd >= 0) {
      (void)::shutdown(client_fd, SHUT_RDWR);
      (void)::close(client_fd);
      client_fd = -1;
    }
    if (link) {
      link->stop();
    }
  };
  struct ScopeExit {
    std::function<void()> fn;
    ~ScopeExit() { fn(); }
  } cleanup_guard{cleanup};

  ControlCommand cmd{};
  cmd.valid = true;
  cmd.armed = true;
  cmd.servo_norm = {0.1, -0.1, 0.2, -0.2};

  REQUIRE(!link->actuator_client_connected(), "Sim-net actuator link should start disconnected");

  client_fd = connect_with_retry(actuator_port, std::chrono::milliseconds(800));
  REQUIRE(client_fd >= 0, "Failed to connect test client to sim-net actuator socket");

  bool connected = false;
  for (int i = 0; i < 300; ++i) {
    link->publish_actuator(cmd, monotonic_time_ns());
    if (link->actuator_client_connected()) {
      connected = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  REQUIRE(connected, "Sim-net actuator client did not become connected after publish attempts");

  const SimNetStats connected_stats = link->stats_snapshot();
  REQUIRE(connected_stats.actuator_client_connected, "Connected snapshot must report active client");
  REQUIRE(connected_stats.actuator_clients >= 1, "Connected snapshot must track accepted clients");

  (void)::shutdown(client_fd, SHUT_RDWR);
  (void)::close(client_fd);
  client_fd = -1;

  bool saw_disconnect = false;
  for (int i = 0; i < 300; ++i) {
    link->publish_actuator(cmd, monotonic_time_ns());
    const SimNetStats stats = link->stats_snapshot();
    if (stats.actuator_disconnects > 0 && !stats.actuator_client_connected) {
      saw_disconnect = true;
      REQUIRE(stats.actuator_send_errors > 0, "Disconnect detection should follow at least one send error");
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  REQUIRE(saw_disconnect, "Sim-net stats never reported actuator disconnect after peer close");

  return true;
}

}  // namespace

int main() {
  if (!test_actuator_disconnect_metrics()) {
    return EXIT_FAILURE;
  }
  std::cout << "runtime_unit_sim_net_link: ok\n";
  return EXIT_SUCCESS;
}
