#include "igniter/vn5e160s.hpp"
#include "igniter/igniter_bank.hpp"

#include <array>
#include <cstdint>
#include <iostream>

int main() {
  igniter::VN5E160S::Config cfg{};
  cfg.settleMs = 5;
  cfg.latchFaults = true;

  igniter::VN5E160S driver(cfg);
  driver.init(0);
  driver.set(true, 0);

  const std::array<uint8_t, 6> status_ok{1, 1, 1, 0, 1, 1};
  std::cout << "t_ms,status_ok,is_on,ok,fault\n";
  uint64_t t_ms = 0;
  for (const uint8_t sample : status_ok) {
    t_ms += 5;
    driver.update(sample != 0U, t_ms);
    std::cout << t_ms << "," << static_cast<uint32_t>(sample) << "," << (driver.is_on() ? 1 : 0) << ","
              << (driver.ok() ? 1 : 0) << "," << igniter::fault_name(driver.fault()) << "\n";
  }

  driver.clear_fault();
  std::cout << "after_clear_fault,ok=" << (driver.ok() ? 1 : 0) << "\n";
  return 0;
}
