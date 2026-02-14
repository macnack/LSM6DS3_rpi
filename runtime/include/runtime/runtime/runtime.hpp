#pragma once

#include "runtime/common/types.hpp"
#include "runtime/config/config.hpp"

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

namespace runtime {

struct RuntimeCliOverrides {
  double duration_sec_override = -1.0;
  std::string status_file;
  bool print_config = false;
};

class Runtime {
 public:
  Runtime(RuntimeConfig cfg, RuntimeCliOverrides overrides = {});
  ~Runtime();

  Runtime(const Runtime&) = delete;
  Runtime& operator=(const Runtime&) = delete;

  void run();
  void request_stop();
  [[nodiscard]] RuntimeStats stats_snapshot() const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace runtime
