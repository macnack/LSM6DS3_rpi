#include "runtime/runtime/rt_thread.hpp"

#include <pthread.h>
#include <sched.h>

#include <cerrno>
#include <cstring>
#include <string>

namespace runtime {

bool configure_current_thread_rt(const std::string& name, int priority, int cpu_affinity,
                                 std::string* error_out) {
  if (!name.empty()) {
#if defined(__APPLE__)
    (void)::pthread_setname_np(name.substr(0, 15).c_str());
#else
    (void)::pthread_setname_np(::pthread_self(), name.substr(0, 15).c_str());
#endif
  }

#if defined(__linux__)
  if (cpu_affinity >= 0) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(static_cast<unsigned>(cpu_affinity), &cpuset);
    if (::pthread_setaffinity_np(::pthread_self(), sizeof(cpuset), &cpuset) != 0) {
      if (error_out != nullptr) {
        *error_out = "pthread_setaffinity_np failed (errno=" + std::to_string(errno) + ": " +
                     std::strerror(errno) + ")";
      }
      return false;
    }
  }
#else
  (void)cpu_affinity;
#endif

  if (priority <= 0) {
    return true;
  }

  sched_param param{};
  param.sched_priority = priority;
  if (::pthread_setschedparam(::pthread_self(), SCHED_FIFO, &param) != 0) {
    if (error_out != nullptr) {
      *error_out = "pthread_setschedparam(SCHED_FIFO) failed (errno=" + std::to_string(errno) +
                   ": " + std::strerror(errno) + ")";
    }
    return false;
  }

  return true;
}

}  // namespace runtime
