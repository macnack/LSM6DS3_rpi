#pragma once

#include "runtime/common/types.hpp"
#include "runtime/config/config.hpp"

namespace runtime {

class CppEstimator {
 public:
  explicit CppEstimator(const EstimatorCppSection& cfg);

  EstimatorState step(const ImuSample& imu, const BaroSample& baro);

 private:
  EstimatorCppSection cfg_;
  bool initialized_ = false;
  uint64_t last_t_ns_ = 0;

  double roll_rad_ = 0.0;
  double pitch_rad_ = 0.0;
  double yaw_rad_ = 0.0;

  double last_alt_m_ = 0.0;
  bool baro_alt_initialized_ = false;
};

}  // namespace runtime
