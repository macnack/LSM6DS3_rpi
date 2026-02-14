#pragma once

#include "runtime/common/types.hpp"
#include "runtime/config/config.hpp"

#include <memory>

namespace runtime {

class ImuBackend {
 public:
  virtual ~ImuBackend() = default;
  virtual void start() = 0;
  virtual void stop() noexcept = 0;
  virtual ImuSample read_once(uint64_t now_ns) = 0;
};

class BaroBackend {
 public:
  virtual ~BaroBackend() = default;
  virtual void start() = 0;
  virtual void stop() noexcept = 0;
  virtual BaroSample read_once(uint64_t now_ns) = 0;
};

std::unique_ptr<ImuBackend> make_imu_backend(const RuntimeConfig& cfg, bool sim_mode);
std::unique_ptr<BaroBackend> make_baro_backend(const RuntimeConfig& cfg, bool sim_mode);

}  // namespace runtime
