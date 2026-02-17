#include "igniter/igniter.hpp"
#include "igniter/igniter_bank.hpp"

#include <array>
#include <cstdint>
#include <iostream>

int main() {
  igniter::VN5E160S::Config drv_cfg{};
  drv_cfg.settleMs = 5;
  drv_cfg.latchFaults = true;

  igniter::Igniter::Config ign_cfg{};
  ign_cfg.defaultFireMs = 150;
  ign_cfg.maxFireMs = 800;

  igniter::Igniter ign(ign_cfg, drv_cfg);
  ign.init(0);

  if (!ign.arm(0)) {
    std::cerr << "arm failed\n";
    return 1;
  }
  if (!ign.fire(200, 0)) {
    std::cerr << "fire failed\n";
    return 1;
  }

  std::cout << "t_ms,state,fault,remaining_ms\n";
  const std::array<uint8_t, 7> status_ok{1, 1, 1, 1, 1, 1, 1};
  uint64_t t_ms = 0;
  for (const uint8_t sample : status_ok) {
    t_ms += 40;
    ign.update(sample != 0U, t_ms);
    std::cout << t_ms << "," << igniter::state_name(ign.state()) << "," << igniter::fault_name(ign.fault()) << ","
              << ign.remaining_ms(t_ms) << "\n";
  }

  ign.clear_fault(t_ms);
  std::cout << "after_clear_fault,state=" << igniter::state_name(ign.state()) << "\n";
  return 0;
}
