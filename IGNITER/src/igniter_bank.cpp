#include "igniter/igniter_bank.hpp"

#include <algorithm>
#include <stdexcept>

namespace igniter {

IgniterBank::IgniterBank(Config cfg)
    : cfg_(std::move(cfg)),
      out_(std::make_unique<SimGpioBatchOut>()),
      in_(std::make_unique<SimGpioBatchIn>()) {
  for (std::size_t i = 0; i < kIgniterChannels; ++i) {
    igniters_[i] = Igniter(cfg_.channels[i].igniter, cfg_.channels[i].driver);
  }
}

IgniterBank::IgniterBank(Config cfg, std::unique_ptr<GpioBatchOut> batch_out,
                         std::unique_ptr<GpioBatchIn> batch_in)
    : cfg_(std::move(cfg)), out_(std::move(batch_out)), in_(std::move(batch_in)) {
  if (!out_ || !in_) {
    throw std::invalid_argument("IgniterBank requires non-null GPIO backends");
  }
  for (std::size_t i = 0; i < kIgniterChannels; ++i) {
    igniters_[i] = Igniter(cfg_.channels[i].igniter, cfg_.channels[i].driver);
  }
}

void IgniterBank::bind_hardware(const std::string& output_chip, const std::string& status_chip,
                                const std::array<uint32_t, kIgniterChannels>& output_lines,
                                const std::array<uint32_t, kIgniterChannels>& status_lines) {
  out_->open(output_chip, output_lines);
  in_->open(status_chip, status_lines);
  io_bound_ = true;
}

void IgniterBank::init(uint64_t now_ms) {
  if (!io_bound_) {
    const std::array<uint32_t, kIgniterChannels> sim_lines{0, 1, 2, 3};
    out_->open("sim", sim_lines);
    in_->open("sim", sim_lines);
    io_bound_ = true;
  }

  for (auto& igniter : igniters_) {
    igniter.init(now_ms);
  }
  armed_ = false;
  global_fault_latched_ = false;
  commit_all_outputs();
}

bool IgniterBank::arm(uint64_t now_ms) {
  if (global_fault_latched_) {
    return false;
  }

  for (std::size_t i = 0; i < kIgniterChannels; ++i) {
    if (!cfg_.channels[i].enabled) {
      continue;
    }
    if (!igniters_[i].arm(now_ms)) {
      global_fault_latched_ = true;
      armed_ = false;
      commit_all_outputs();
      return false;
    }
  }

  armed_ = true;
  return true;
}

void IgniterBank::disarm(uint64_t now_ms) {
  for (auto& igniter : igniters_) {
    igniter.disarm(now_ms);
  }
  armed_ = false;
  commit_all_outputs();
}

bool IgniterBank::fire_mask(uint8_t fire_mask,
                            const std::array<uint32_t, kIgniterChannels>& duration_ms,
                            uint64_t now_ms) {
  if (!armed_ || global_fault_latched_) {
    return false;
  }

  uint8_t effective_mask = 0;
  for (uint8_t i = 0; i < kIgniterChannels; ++i) {
    if ((fire_mask & (1U << i)) == 0U || !enabled_channel(i)) {
      continue;
    }
    effective_mask |= static_cast<uint8_t>(1U << i);
    if (igniters_[i].state() != State::ArmedIdle) {
      return false;
    }
    if (duration_ms[i] > cfg_.channels[i].igniter.maxFireMs) {
      return false;
    }
  }

  if (effective_mask == 0U) {
    return false;
  }

  for (uint8_t i = 0; i < kIgniterChannels; ++i) {
    if ((effective_mask & (1U << i)) == 0U) {
      continue;
    }
    if (!igniters_[i].fire(duration_ms[i], now_ms)) {
      return false;
    }
  }

  commit_outputs(effective_mask);
  return true;
}

bool IgniterBank::fire_one(uint8_t channel, uint32_t duration_ms, uint64_t now_ms) {
  if (channel >= kIgniterChannels) {
    return false;
  }
  std::array<uint32_t, kIgniterChannels> durations{0, 0, 0, 0};
  durations[channel] = duration_ms;
  return fire_mask(static_cast<uint8_t>(1U << channel), durations, now_ms);
}

bool IgniterBank::fire_all(uint32_t duration_ms, uint64_t now_ms) {
  std::array<uint32_t, kIgniterChannels> durations{duration_ms, duration_ms, duration_ms, duration_ms};
  uint8_t mask = 0;
  for (uint8_t i = 0; i < kIgniterChannels; ++i) {
    if (enabled_channel(i)) {
      mask |= static_cast<uint8_t>(1U << i);
    }
  }
  return fire_mask(mask, durations, now_ms);
}

void IgniterBank::clear_fault(uint64_t now_ms) {
  for (auto& igniter : igniters_) {
    igniter.clear_fault(now_ms);
  }
  armed_ = false;
  global_fault_latched_ = false;
  commit_all_outputs();
}

IgniterBank::Snapshot IgniterBank::update(uint64_t now_ms, bool force_disarm) {
  if (force_disarm) {
    const uint8_t active_mask = snapshot(now_ms).activeMask;
    if (armed_ || active_mask != 0U) {
      disarm(now_ms);
    }
    return snapshot(now_ms);
  }

  const auto prev = current_outputs();
  const std::array<uint8_t, kIgniterChannels> status_values = in_->read_values();

  bool any_fault = false;
  for (std::size_t i = 0; i < kIgniterChannels; ++i) {
    if (!cfg_.channels[i].enabled) {
      continue;
    }
    igniters_[i].update(status_values[i] != 0U, now_ms);
    any_fault = any_fault || igniters_[i].has_fault();
  }

  if (any_fault && cfg_.faultPolicy == FaultPolicy::Global) {
    global_fault_latched_ = true;
    armed_ = false;
    for (auto& igniter : igniters_) {
      igniter.off(now_ms);
    }
  }

  const auto next = current_outputs();
  uint8_t changed_mask = 0;
  for (uint8_t i = 0; i < kIgniterChannels; ++i) {
    if (prev[i] != next[i]) {
      changed_mask |= static_cast<uint8_t>(1U << i);
    }
  }

  if (changed_mask != 0U) {
    commit_outputs(changed_mask);
  }

  return snapshot(now_ms);
}

IgniterBank::Snapshot IgniterBank::snapshot(uint64_t now_ms) const {
  Snapshot s;
  s.armed = armed_;
  s.globalFaultLatched = global_fault_latched_;

  const auto outputs = current_outputs();
  for (uint8_t i = 0; i < kIgniterChannels; ++i) {
    if (outputs[i] != 0U) {
      s.activeMask |= static_cast<uint8_t>(1U << i);
    }
    s.states[i] = igniters_[i].state();
    s.faults[i] = igniters_[i].fault();
    s.remainingMs[i] = igniters_[i].remaining_ms(now_ms);
  }

  return s;
}

bool IgniterBank::set_sim_status_values(const std::array<uint8_t, kIgniterChannels>& values) {
  auto* sim = dynamic_cast<SimGpioBatchIn*>(in_.get());
  if (sim == nullptr) {
    return false;
  }
  sim->set_values(values);
  return true;
}

void IgniterBank::commit_outputs(uint8_t mask) { out_->set_mask(mask, current_outputs()); }

void IgniterBank::commit_all_outputs() { out_->set_mask(0x0FU, current_outputs()); }

std::array<uint8_t, kIgniterChannels> IgniterBank::current_outputs() const {
  std::array<uint8_t, kIgniterChannels> out{0, 0, 0, 0};
  for (std::size_t i = 0; i < kIgniterChannels; ++i) {
    if (cfg_.channels[i].enabled && igniters_[i].output_on()) {
      out[i] = 1U;
    }
  }
  return out;
}

bool IgniterBank::enabled_channel(uint8_t channel) const {
  return channel < kIgniterChannels && cfg_.channels[channel].enabled;
}

const char* fault_name(Fault fault) noexcept {
  switch (fault) {
    case Fault::None:
      return "none";
    case Fault::StatusLowWhileOn:
      return "status_low_while_on";
    case Fault::StatusLowWhileOff:
      return "status_low_while_off";
    case Fault::Latched:
      return "latched";
  }
  return "unknown";
}

const char* state_name(State state) noexcept {
  switch (state) {
    case State::Disarmed:
      return "disarmed";
    case State::ArmedIdle:
      return "armed_idle";
    case State::Firing:
      return "firing";
    case State::FaultLatched:
      return "fault_latched";
  }
  return "unknown";
}

}  // namespace igniter
