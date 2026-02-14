#pragma once

#include <string>

namespace runtime {

bool configure_current_thread_rt(const std::string& name, int priority, int cpu_affinity,
                                 std::string* error_out);

}  // namespace runtime
