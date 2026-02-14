#pragma once

#include "runtime/config/config.hpp"

#include <cstdint>

namespace runtime {

class KillSwitchMonitor {
 public:
  explicit KillSwitchMonitor(const KillSwitchSection& cfg);
  ~KillSwitchMonitor();

  KillSwitchMonitor(const KillSwitchMonitor&) = delete;
  KillSwitchMonitor& operator=(const KillSwitchMonitor&) = delete;

  void start();
  void stop() noexcept;

  [[nodiscard]] bool enabled() const noexcept;
  [[nodiscard]] bool tripped() const noexcept;

  // Returns true when kill switch is active (open NC circuit / emergency state).
  bool poll();

 private:
  void ensure_exported();
  void configure_input();
  int read_value();

  KillSwitchSection cfg_;
  int value_fd_ = -1;
  uint32_t open_count_ = 0;
  bool tripped_ = false;
  bool exported_by_us_ = false;
};

}  // namespace runtime
