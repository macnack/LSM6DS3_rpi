#pragma once

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <thread>
#include <time.h>

namespace runtime {

inline uint64_t monotonic_time_ns() {
  timespec ts{};
  if (::clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
    throw std::runtime_error("clock_gettime(CLOCK_MONOTONIC) failed");
  }
  return static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ULL + static_cast<uint64_t>(ts.tv_nsec);
}

inline uint64_t ns_from_ms(uint64_t ms) { return ms * 1'000'000ULL; }

inline uint64_t hz_to_period_ns(uint32_t hz) {
  if (hz == 0) {
    throw std::invalid_argument("Hz must be > 0");
  }
  return 1'000'000'000ULL / static_cast<uint64_t>(hz);
}

inline void sleep_until_ns(uint64_t target_ns) {
#if defined(__linux__)
  timespec ts{};
  ts.tv_sec = static_cast<time_t>(target_ns / 1'000'000'000ULL);
  ts.tv_nsec = static_cast<long>(target_ns % 1'000'000'000ULL);
  while (::clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr) == EINTR) {
  }
#else
  const uint64_t now_ns = monotonic_time_ns();
  if (target_ns > now_ns) {
    std::this_thread::sleep_for(std::chrono::nanoseconds(target_ns - now_ns));
  }
#endif
}

inline void sleep_for_ns(uint64_t duration_ns) {
  std::this_thread::sleep_for(std::chrono::nanoseconds(duration_ns));
}

}  // namespace runtime
