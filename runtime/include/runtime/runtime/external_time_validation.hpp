#pragma once

#include <cstdint>

namespace runtime {

enum class ExternalTimeValidationResult : uint8_t {
  Accept = 0,
  RejectStale = 1,
  RejectFuture = 2,
  RejectRegression = 3,
};

ExternalTimeValidationResult validate_external_timestamp(uint64_t now_ns, uint64_t msg_t_ns,
                                                         uint64_t last_seen_t_ns,
                                                         uint64_t fresh_timeout_ns) noexcept;

}  // namespace runtime
