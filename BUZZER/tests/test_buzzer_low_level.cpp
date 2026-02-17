#include "buzzer/hardware_buzzer.hpp"

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>

namespace {

#define REQUIRE(cond, msg)      \
  do {                          \
    if (!(cond)) {              \
      std::cerr << msg << "\n"; \
      return false;             \
    }                           \
  } while (0)

bool test_start_and_close_safe_off() {
  buzzer::HardwareBuzzerConfig cfg;
  cfg.simulate = true;
  cfg.force_off_on_begin = true;
  cfg.force_off_on_close = true;

  buzzer::HardwareBuzzer hw(cfg);
  hw.begin();
  REQUIRE(hw.is_open(), "HardwareBuzzer should be open after begin");
  REQUIRE(!hw.is_on(), "HardwareBuzzer should be OFF after begin");

  hw.on();
  REQUIRE(hw.is_on(), "HardwareBuzzer should turn ON");

  hw.close();
  REQUIRE(!hw.is_open(), "HardwareBuzzer should be closed");
  REQUIRE(!hw.is_on(), "HardwareBuzzer should be OFF after close");
  return true;
}

bool test_active_high_low_raw_levels() {
  buzzer::HardwareBuzzerConfig high;
  high.simulate = true;
  high.active_high = true;
  buzzer::HardwareBuzzer h(high);
  h.begin();
  REQUIRE(h.raw_level() == 0, "active_high OFF raw level should be 0");
  h.on();
  REQUIRE(h.raw_level() == 1, "active_high ON raw level should be 1");

  buzzer::HardwareBuzzerConfig low;
  low.simulate = true;
  low.active_high = false;
  buzzer::HardwareBuzzer l(low);
  l.begin();
  REQUIRE(l.raw_level() == 1, "active_low OFF raw level should be 1");
  l.on();
  REQUIRE(l.raw_level() == 0, "active_low ON raw level should be 0");
  return true;
}

bool test_watchdog_max_on_time() {
  buzzer::HardwareBuzzerConfig cfg;
  cfg.simulate = true;
  cfg.max_on_time_ms = 20;

  buzzer::HardwareBuzzer hw(cfg);
  hw.begin();
  hw.on();
  REQUIRE(hw.is_on(), "buzzer should start ON");
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  REQUIRE(!hw.is_on(), "watchdog should force OFF after max_on_time_ms");
  return true;
}

}  // namespace

int main() {
  if (!test_start_and_close_safe_off()) {
    return EXIT_FAILURE;
  }
  if (!test_active_high_low_raw_levels()) {
    return EXIT_FAILURE;
  }
  if (!test_watchdog_max_on_time()) {
    return EXIT_FAILURE;
  }
  std::cout << "buzzer_unit_low_level: ok\n";
  return EXIT_SUCCESS;
}
