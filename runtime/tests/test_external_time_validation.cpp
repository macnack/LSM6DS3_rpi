#include "runtime/runtime/external_time_validation.hpp"

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

bool test_accept_fresh() {
  const auto r = validate_external_timestamp(1'000'000'000ULL, 999'999'500ULL, 999'000'000ULL, 2'000'000ULL);
  REQUIRE(r == ExternalTimeValidationResult::Accept, "Expected fresh timestamp to be accepted");
  return true;
}

bool test_reject_stale() {
  const auto r = validate_external_timestamp(1'000'000'000ULL, 900'000'000ULL, 850'000'000ULL, 10'000'000ULL);
  REQUIRE(r == ExternalTimeValidationResult::RejectStale, "Expected stale timestamp to be rejected");
  return true;
}

bool test_reject_future() {
  const auto r = validate_external_timestamp(1'000'000'000ULL, 1'000'000'100ULL, 999'000'000ULL, 10'000'000ULL);
  REQUIRE(r == ExternalTimeValidationResult::RejectFuture, "Expected future timestamp to be rejected");
  return true;
}

bool test_reject_regression() {
  const auto r = validate_external_timestamp(1'000'000'000ULL, 999'000'000ULL, 999'100'000ULL, 10'000'000ULL);
  REQUIRE(r == ExternalTimeValidationResult::RejectRegression, "Expected regressing timestamp to be rejected");
  return true;
}

bool test_future_reject_does_not_poison_last_seen_sequence() {
  uint64_t last_seen = 1'000ULL;
  const auto future = validate_external_timestamp(10'000ULL, 11'000ULL, last_seen, 2'000ULL);
  REQUIRE(future == ExternalTimeValidationResult::RejectFuture, "Future sample must be rejected");
  // Emulate runtime behavior: last_seen only updates on Accept.
  if (future == ExternalTimeValidationResult::Accept) {
    last_seen = 11'000ULL;
  }

  const auto subsequent = validate_external_timestamp(12'000ULL, 10'500ULL, last_seen, 2'500ULL);
  REQUIRE(subsequent == ExternalTimeValidationResult::Accept,
          "Subsequent valid sample should be accepted when last_seen was not poisoned");
  return true;
}

}  // namespace

int main() {
  if (!test_accept_fresh()) {
    return EXIT_FAILURE;
  }
  if (!test_reject_stale()) {
    return EXIT_FAILURE;
  }
  if (!test_reject_future()) {
    return EXIT_FAILURE;
  }
  if (!test_reject_regression()) {
    return EXIT_FAILURE;
  }
  if (!test_future_reject_does_not_poison_last_seen_sequence()) {
    return EXIT_FAILURE;
  }
  std::cout << "runtime_unit_external_time_validation: ok\n";
  return EXIT_SUCCESS;
}
