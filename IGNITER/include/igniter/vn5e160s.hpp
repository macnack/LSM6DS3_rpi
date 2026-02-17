#pragma once

#include <cstdint>

namespace igniter {

enum class Fault : uint8_t {
  None = 0,
  StatusLowWhileOn = 1,
  StatusLowWhileOff = 2,
  Latched = 3,
};

class VN5E160S {
 public:
  struct Config {
    uint32_t settleMs = 5;
    bool latchFaults = true;
  };

  VN5E160S();
  explicit VN5E160S(Config cfg);

  void init(uint64_t now_ms);
  bool set(bool on, uint64_t now_ms);
  void update(bool status_ok, uint64_t now_ms);
  void force_off(uint64_t now_ms);
  void clear_fault();

  [[nodiscard]] bool is_on() const noexcept { return output_on_; }
  [[nodiscard]] Fault fault() const noexcept { return fault_; }
  [[nodiscard]] bool ok() const noexcept { return fault_ == Fault::None; }
  [[nodiscard]] uint64_t last_switch_ms() const noexcept { return last_switch_ms_; }

 private:
  Config cfg_;
  bool output_on_ = false;
  Fault fault_ = Fault::None;
  uint64_t last_switch_ms_ = 0;
};

}  // namespace igniter
