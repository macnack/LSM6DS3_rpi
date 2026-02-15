#include "runtime/config/config.hpp"
#include "runtime/runtime/runtime.hpp"

#include <csignal>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <memory>
#include <string>

namespace {

runtime::Runtime* g_runtime = nullptr;

void handle_signal(int) {
  if (g_runtime != nullptr) {
    g_runtime->request_stop();
  }
}

void print_usage(const char* argv0) {
  std::cerr << "Usage: " << argv0
            << " --config <path> [--duration-sec <seconds>] [--status-file <path>] [--print-config]\n";
}

}  // namespace

int main(int argc, char** argv) {
  std::string config_path;
  runtime::RuntimeCliOverrides overrides;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--config" && i + 1 < argc) {
      config_path = argv[++i];
      continue;
    }
    if (arg == "--duration-sec" && i + 1 < argc) {
      overrides.duration_sec_override = std::stod(argv[++i]);
      continue;
    }
    if (arg == "--status-file" && i + 1 < argc) {
      overrides.status_file = argv[++i];
      continue;
    }
    if (arg == "--print-config") {
      overrides.print_config = true;
      continue;
    }

    print_usage(argv[0]);
    return EXIT_FAILURE;
  }

  if (config_path.empty()) {
    print_usage(argv[0]);
    return EXIT_FAILURE;
  }

  try {
    runtime::RuntimeConfig cfg = runtime::load_runtime_config(config_path);

    if (overrides.print_config) {
      std::cout << runtime::runtime_config_to_string(cfg) << std::flush;
    }

    runtime::Runtime rt(std::move(cfg), overrides);
    g_runtime = &rt;

    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);
    std::signal(SIGPIPE, SIG_IGN);

    rt.run();
    g_runtime = nullptr;
    return EXIT_SUCCESS;
  } catch (const std::exception& ex) {
    std::cerr << "rt_core failed: " << ex.what() << "\n";
    return EXIT_FAILURE;
  }
}
