#include "igniter/igniter_bank.hpp"

#include <array>
#include <cstdint>
#include <iostream>

namespace {

void print_snapshot(uint64_t t_ms, const igniter::IgniterBank::Snapshot& snap) {
  std::cout << "t_ms=" << t_ms << " armed=" << (snap.armed ? 1 : 0)
            << " global_fault_latched=" << (snap.globalFaultLatched ? 1 : 0)
            << " active_mask=0x" << std::hex << static_cast<uint32_t>(snap.activeMask) << std::dec << "\n";
  for (std::size_t i = 0; i < igniter::kIgniterChannels; ++i) {
    std::cout << "  ch" << i << " state=" << igniter::state_name(snap.states[i])
              << " fault=" << igniter::fault_name(snap.faults[i])
              << " remaining_ms=" << snap.remainingMs[i] << "\n";
  }
}

}  // namespace

int main() {
  igniter::IgniterBank::Config cfg{};
  cfg.faultPolicy = igniter::FaultPolicy::Global;
  for (auto& ch : cfg.channels) {
    ch.enabled = true;
    ch.driver.settleMs = 5;
    ch.driver.latchFaults = true;
    ch.igniter.defaultFireMs = 200;
    ch.igniter.maxFireMs = 2000;
  }

  igniter::IgniterBank bank(cfg);
  bank.init(0);
  if (!bank.arm(0)) {
    std::cerr << "arm failed\n";
    return 1;
  }

  const std::array<uint32_t, igniter::kIgniterChannels> durations{250, 250, 250, 250};
  if (!bank.fire_mask(0x0FU, durations, 10)) {
    std::cerr << "fire_mask failed\n";
    return 1;
  }

  std::cout << "=== nominal firing ===\n";
  for (uint64_t t_ms = 20; t_ms <= 300; t_ms += 40) {
    bank.set_sim_status_values({1, 1, 1, 1});
    print_snapshot(t_ms, bank.update(t_ms, false));
  }

  bank.clear_fault(350);
  bank.arm(350);
  bank.fire_one(2, 300, 360);

  std::cout << "=== injected fault on ch2 (status low) ===\n";
  for (uint64_t t_ms = 370; t_ms <= 520; t_ms += 30) {
    const uint8_t ch2_status = (t_ms >= 430) ? 0U : 1U;
    bank.set_sim_status_values({1, 1, ch2_status, 1});
    print_snapshot(t_ms, bank.update(t_ms, false));
  }

  return 0;
}
