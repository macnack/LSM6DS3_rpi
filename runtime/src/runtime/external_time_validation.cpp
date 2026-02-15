#include "runtime/runtime/external_time_validation.hpp"

namespace runtime {

ExternalTimeValidationResult validate_external_timestamp(uint64_t now_ns, uint64_t msg_t_ns,
                                                         uint64_t last_seen_t_ns,
                                                         uint64_t fresh_timeout_ns) noexcept {
  if (msg_t_ns == 0) {
    return ExternalTimeValidationResult::RejectStale;
  }
  if (msg_t_ns > now_ns) {
    return ExternalTimeValidationResult::RejectFuture;
  }
  if ((now_ns - msg_t_ns) > fresh_timeout_ns) {
    return ExternalTimeValidationResult::RejectStale;
  }
  if (last_seen_t_ns != 0 && msg_t_ns < last_seen_t_ns) {
    return ExternalTimeValidationResult::RejectRegression;
  }
  return ExternalTimeValidationResult::Accept;
}

}  // namespace runtime
