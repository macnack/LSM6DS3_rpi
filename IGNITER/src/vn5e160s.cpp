#include "igniter/vn5e160s.hpp"

#include <stdexcept>

namespace igniter {

VN5E160S::VN5E160S() : VN5E160S(Config{}) {}

VN5E160S::VN5E160S(Config cfg) : cfg_(cfg) {
  if (cfg_.settleMs == 0) {
    throw std::invalid_argument("VN5E160S::Config.settleMs must be >= 1");
  }
}

void VN5E160S::init(uint64_t now_ms) {
  output_on_ = false;
  fault_ = Fault::None;
  last_switch_ms_ = now_ms;
}

bool VN5E160S::set(bool on, uint64_t now_ms) {
  if (on && cfg_.latchFaults && fault_ == Fault::Latched) {
    return false;
  }
  output_on_ = on;
  last_switch_ms_ = now_ms;
  return true;
}

void VN5E160S::update(bool status_ok, uint64_t now_ms) {
  if (now_ms < last_switch_ms_ || (now_ms - last_switch_ms_) < cfg_.settleMs) {
    return;
  }

  if (status_ok) {
    return;
  }

  const Fault sampled_fault = output_on_ ? Fault::StatusLowWhileOn : Fault::StatusLowWhileOff;
  output_on_ = false;

  if (cfg_.latchFaults) {
    fault_ = Fault::Latched;
  } else {
    fault_ = sampled_fault;
  }
}

void VN5E160S::force_off(uint64_t now_ms) {
  output_on_ = false;
  last_switch_ms_ = now_ms;
}

void VN5E160S::clear_fault() { fault_ = Fault::None; }

}  // namespace igniter
