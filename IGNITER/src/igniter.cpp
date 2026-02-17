#include "igniter/igniter.hpp"

#include <algorithm>
#include <stdexcept>

namespace igniter {

Igniter::Igniter() : Igniter(Config{}, VN5E160S::Config{}) {}

Igniter::Igniter(Config cfg, VN5E160S::Config driver_cfg) : driver_(driver_cfg), cfg_(cfg) {
  if (cfg_.maxFireMs == 0) {
    throw std::invalid_argument("Igniter::Config.maxFireMs must be >= 1");
  }
  if (cfg_.defaultFireMs == 0 || cfg_.defaultFireMs > cfg_.maxFireMs) {
    throw std::invalid_argument("Igniter::Config.defaultFireMs must be in [1, maxFireMs]");
  }
}

void Igniter::init(uint64_t now_ms) {
  driver_.init(now_ms);
  driver_.set(false, now_ms);
  state_ = State::Disarmed;
  fire_start_ms_ = 0;
  fire_end_ms_ = 0;
}

bool Igniter::arm(uint64_t now_ms) {
  (void)now_ms;
  if (state_ == State::FaultLatched || driver_.fault() != Fault::None) {
    state_ = State::FaultLatched;
    return false;
  }
  state_ = State::ArmedIdle;
  return true;
}

void Igniter::disarm(uint64_t now_ms) {
  driver_.set(false, now_ms);
  state_ = State::Disarmed;
}

bool Igniter::fire(uint32_t duration_ms, uint64_t now_ms) {
  if (state_ != State::ArmedIdle) {
    return false;
  }

  uint32_t effective_ms = duration_ms;
  if (effective_ms == 0) {
    effective_ms = cfg_.defaultFireMs;
  }
  if (effective_ms > cfg_.maxFireMs) {
    return false;
  }

  if (!driver_.set(true, now_ms)) {
    state_ = State::FaultLatched;
    return false;
  }

  fire_start_ms_ = now_ms;
  fire_end_ms_ = now_ms + effective_ms;
  state_ = State::Firing;
  return true;
}

void Igniter::off(uint64_t now_ms) {
  driver_.set(false, now_ms);
  if (state_ == State::Firing) {
    state_ = State::Disarmed;
  }
}

void Igniter::update(bool status_ok, uint64_t now_ms) {
  driver_.update(status_ok, now_ms);

  if (driver_.fault() != Fault::None) {
    driver_.set(false, now_ms);
    state_ = State::FaultLatched;
    return;
  }

  if (state_ == State::Firing) {
    if (now_ms >= fire_end_ms_) {
      driver_.set(false, now_ms);
      state_ = State::ArmedIdle;
      return;
    }

    const uint64_t elapsed = (now_ms >= fire_start_ms_) ? (now_ms - fire_start_ms_) : 0;
    if (elapsed > cfg_.maxFireMs) {
      driver_.set(false, now_ms);
      state_ = State::FaultLatched;
      return;
    }
  }
}

void Igniter::clear_fault(uint64_t now_ms) {
  driver_.set(false, now_ms);
  driver_.clear_fault();
  state_ = State::Disarmed;
  fire_start_ms_ = 0;
  fire_end_ms_ = 0;
}

uint32_t Igniter::remaining_ms(uint64_t now_ms) const noexcept {
  if (state_ != State::Firing) {
    return 0;
  }
  if (now_ms >= fire_end_ms_) {
    return 0;
  }
  const uint64_t rem = fire_end_ms_ - now_ms;
  return static_cast<uint32_t>(std::min<uint64_t>(rem, UINT32_MAX));
}

}  // namespace igniter
