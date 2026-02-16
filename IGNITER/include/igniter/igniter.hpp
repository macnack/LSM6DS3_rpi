#pragma once

#include "igniter/vn5e160s.hpp"

#include <cstdint>

namespace igniter {

enum class State : uint8_t {
  Disarmed = 0,
  ArmedIdle = 1,
  Firing = 2,
  FaultLatched = 3,
};

class Igniter {
 public:
  struct Config {
    uint32_t maxFireMs = 2000;
    uint32_t defaultFireMs = 200;
  };

  Igniter();
  Igniter(Config cfg, VN5E160S::Config driver_cfg);

  void init(uint64_t now_ms);
  bool arm(uint64_t now_ms);
  void disarm(uint64_t now_ms);
  bool fire(uint32_t duration_ms, uint64_t now_ms);
  void off(uint64_t now_ms);
  void update(bool status_ok, uint64_t now_ms);
  void clear_fault(uint64_t now_ms);

  [[nodiscard]] State state() const noexcept { return state_; }
  [[nodiscard]] bool is_armed() const noexcept { return state_ == State::ArmedIdle || state_ == State::Firing; }
  [[nodiscard]] bool is_firing() const noexcept { return state_ == State::Firing; }
  [[nodiscard]] bool has_fault() const noexcept {
    return state_ == State::FaultLatched || driver_.fault() != Fault::None;
  }
  [[nodiscard]] bool output_on() const noexcept { return driver_.is_on(); }
  [[nodiscard]] Fault fault() const noexcept { return driver_.fault(); }
  [[nodiscard]] uint32_t remaining_ms(uint64_t now_ms) const noexcept;

 private:
  VN5E160S driver_;
  Config cfg_;
  State state_ = State::Disarmed;
  uint64_t fire_start_ms_ = 0;
  uint64_t fire_end_ms_ = 0;
};

}  // namespace igniter
