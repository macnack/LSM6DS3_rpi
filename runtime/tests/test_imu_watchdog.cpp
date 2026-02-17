#include "runtime/runtime/imu_watchdog.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

#define REQUIRE(cond, msg)      \
  do {                          \
    if (!(cond)) {              \
      std::cerr << msg << "\n"; \
      return false;             \
    }                           \
  } while (0)

using namespace runtime;

ImuSample make_sample(double ax, double ay, double az, double gx, double gy, double gz) {
  ImuSample s;
  s.valid = true;
  s.ax_mps2 = ax;
  s.ay_mps2 = ay;
  s.az_mps2 = az;
  s.gx_rads = gx;
  s.gy_rads = gy;
  s.gz_rads = gz;
  return s;
}

bool test_healthy_varying_stays_healthy() {
  ImuWatchdogSection cfg;
  cfg.flatline_consecutive = 5;
  cfg.zero_vector_consecutive = 3;
  cfg.degenerate_pattern_consecutive = 5;
  ImuWatchdog wd(cfg);

  for (int i = 0; i < 20; ++i) {
    const auto r = wd.on_sample(make_sample(1.0 + 0.01 * i, -2.0 + 0.02 * i, 9.0 + 0.01 * i,
                                            0.001 * i, -0.002 * i, 0.003 * i),
                                static_cast<uint64_t>(i) * 1'000'000ULL);
    REQUIRE(!r.force_invalid, "Healthy varying stream should remain valid");
    REQUIRE(r.state == ImuWatchdogState::Healthy, "Healthy varying stream should stay in healthy state");
  }
  return true;
}

bool test_zero_vector_fault() {
  ImuWatchdogSection cfg;
  cfg.zero_vector_consecutive = 3;
  cfg.flatline_consecutive = 50;
  cfg.degenerate_pattern_consecutive = 50;
  ImuWatchdog wd(cfg);

  wd.on_sample(make_sample(1, 2, 3, 0.1, 0.2, 0.3), 0);
  auto r1 = wd.on_sample(make_sample(0, 0, 0, 0, 0, 0), 1);
  auto r2 = wd.on_sample(make_sample(0, 0, 0, 0, 0, 0), 2);
  auto r3 = wd.on_sample(make_sample(0, 0, 0, 0, 0, 0), 3);

  REQUIRE(!r1.force_invalid, "Zero vector below threshold should not fault yet");
  REQUIRE(!r2.force_invalid, "Zero vector below threshold should not fault yet (2/3)");
  REQUIRE(r3.force_invalid, "Third zero vector should fault and force invalid");
  REQUIRE(wd.fault_count() == 1, "Fault counter should increment once");
  REQUIRE(wd.zero_vector_count() >= 3, "Zero vector counter should track repeated bad samples");
  return true;
}

bool test_flatline_fault() {
  ImuWatchdogSection cfg;
  cfg.zero_vector_consecutive = 50;
  cfg.flatline_consecutive = 3;
  cfg.degenerate_pattern_consecutive = 50;
  cfg.flatline_epsilon = 1e-6;
  ImuWatchdog wd(cfg);

  wd.on_sample(make_sample(1.11, -2.22, 3.33, 0.01, -0.02, 0.03), 0);
  wd.on_sample(make_sample(1.11, -2.22, 3.33, 0.01, -0.02, 0.03), 1);
  wd.on_sample(make_sample(1.11, -2.22, 3.33, 0.01, -0.02, 0.03), 2);
  auto r = wd.on_sample(make_sample(1.11, -2.22, 3.33, 0.01, -0.02, 0.03), 3);

  REQUIRE(r.force_invalid, "Flatline threshold should fault");
  REQUIRE(wd.flatline_count() >= 3, "Flatline counter should increment");
  REQUIRE(wd.last_fault_reason() == ImuFaultReason::Flatline, "Last fault reason should be flatline");
  return true;
}

bool test_degenerate_fault() {
  ImuWatchdogSection cfg;
  cfg.zero_vector_consecutive = 50;
  cfg.flatline_consecutive = 50;
  cfg.degenerate_pattern_consecutive = 2;
  cfg.flatline_epsilon = 1e-6;
  ImuWatchdog wd(cfg);

  wd.on_sample(make_sample(4.305, 4.305, 4.305, 0.235, 0.235, 0.235), 0);
  auto r = wd.on_sample(make_sample(4.305, 4.305, 4.305, 0.235, 0.235, 0.235), 1);

  REQUIRE(r.force_invalid, "Degenerate repeated pattern should fault");
  REQUIRE(wd.degenerate_pattern_count() >= 2, "Degenerate pattern counter should increment");
  REQUIRE(wd.last_fault_reason() == ImuFaultReason::DegeneratePattern,
          "Last fault reason should be degenerate pattern");
  return true;
}

bool test_nan_inf_faults_immediately() {
  ImuWatchdogSection cfg;
  cfg.zero_vector_consecutive = 50;
  cfg.flatline_consecutive = 50;
  cfg.degenerate_pattern_consecutive = 50;
  ImuWatchdog wd(cfg);

  auto r = wd.on_sample(make_sample(std::nan(""), 0.0, 1.0, 0.0, 0.0, 0.0), 10);
  REQUIRE(r.force_invalid, "NaN should fault immediately");
  REQUIRE(wd.last_fault_reason() == ImuFaultReason::NanInf, "NaN/Inf should set nan_inf reason");
  return true;
}

bool test_recovery_needs_full_healthy_window() {
  ImuWatchdogSection cfg;
  cfg.zero_vector_consecutive = 1;
  cfg.flatline_consecutive = 50;
  cfg.degenerate_pattern_consecutive = 50;
  cfg.healthy_recovery_samples = 3;
  ImuWatchdog wd(cfg);

  auto fault = wd.on_sample(make_sample(0, 0, 0, 0, 0, 0), 0);
  REQUIRE(fault.force_invalid, "Fault should force invalid immediately at threshold=1");

  auto h1 = wd.on_sample(make_sample(1, 2, 9, 0.1, 0.2, 0.3), 1);
  auto h2 = wd.on_sample(make_sample(1.1, 2.1, 9.1, 0.11, 0.21, 0.31), 2);
  auto h3 = wd.on_sample(make_sample(1.2, 2.2, 9.2, 0.12, 0.22, 0.32), 3);

  REQUIRE(h1.force_invalid, "Recovery sample 1 should stay invalid");
  REQUIRE(h2.force_invalid, "Recovery sample 2 should stay invalid");
  REQUIRE(!h3.force_invalid, "Recovery sample 3 should clear fault");
  REQUIRE(wd.state() == ImuWatchdogState::Healthy, "Watchdog should return to healthy after window");
  return true;
}

bool test_reinit_backoff_and_max_attempts() {
  ImuWatchdogSection cfg;
  cfg.zero_vector_consecutive = 1;
  cfg.flatline_consecutive = 50;
  cfg.degenerate_pattern_consecutive = 50;
  cfg.recovery_backoff_ms = 100;
  cfg.max_reinit_attempts = 2;
  ImuWatchdog wd(cfg);

  auto fault = wd.on_sample(make_sample(0, 0, 0, 0, 0, 0), 0);
  REQUIRE(fault.should_attempt_reinit, "Fault entry should request immediate reinit attempt");

  wd.on_reinit_result(false, 0);
  auto early = wd.on_sample(make_sample(1, 2, 9, 0.1, 0.2, 0.3), 50'000'000ULL);
  REQUIRE(!early.should_attempt_reinit, "Reinit must wait for backoff interval");

  auto at_backoff = wd.on_sample(make_sample(1.1, 2.1, 9.1, 0.11, 0.21, 0.31), 100'000'000ULL);
  REQUIRE(at_backoff.should_attempt_reinit, "Reinit should fire at backoff boundary");

  wd.on_reinit_result(false, 100'000'000ULL);
  auto after_limit = wd.on_sample(make_sample(1.2, 2.2, 9.2, 0.12, 0.22, 0.32), 200'000'000ULL);
  REQUIRE(!after_limit.should_attempt_reinit, "Reinit should stop after max_reinit_attempts");
  return true;
}

bool test_no_accel_magnitude_rejection() {
  ImuWatchdogSection cfg;
  cfg.zero_vector_consecutive = 5;
  cfg.flatline_consecutive = 5;
  cfg.degenerate_pattern_consecutive = 5;
  ImuWatchdog wd(cfg);

  for (int i = 0; i < 10; ++i) {
    const auto r = wd.on_sample(make_sample(1000.0 + i, -800.0 + i, 1200.0 - i, 3.0 + i * 0.1,
                                            -2.0 + i * 0.1, 1.0 + i * 0.1),
                                static_cast<uint64_t>(i) * 1'000'000ULL);
    REQUIRE(!r.force_invalid, "High magnitude values should not be rejected by watchdog");
  }
  REQUIRE(wd.fault_count() == 0, "No accel-magnitude plausibility rule should trigger a fault");
  return true;
}

}  // namespace

int main() {
  if (!test_healthy_varying_stays_healthy()) {
    return EXIT_FAILURE;
  }
  if (!test_zero_vector_fault()) {
    return EXIT_FAILURE;
  }
  if (!test_flatline_fault()) {
    return EXIT_FAILURE;
  }
  if (!test_degenerate_fault()) {
    return EXIT_FAILURE;
  }
  if (!test_nan_inf_faults_immediately()) {
    return EXIT_FAILURE;
  }
  if (!test_recovery_needs_full_healthy_window()) {
    return EXIT_FAILURE;
  }
  if (!test_reinit_backoff_and_max_attempts()) {
    return EXIT_FAILURE;
  }
  if (!test_no_accel_magnitude_rejection()) {
    return EXIT_FAILURE;
  }

  std::cout << "runtime_unit_imu_watchdog: ok\n";
  return EXIT_SUCCESS;
}
