#pragma once

#include "runtime/common/types.hpp"
#include "runtime/config/config.hpp"

namespace runtime {

class CppController {
 public:
  explicit CppController(const ControllerCppSection& cfg);

  ControlCommand step(const ImuSample& imu, const EstimatorState& est, bool armed, uint64_t now_ns);

 private:
  ControllerCppSection cfg_;
};

}  // namespace runtime
