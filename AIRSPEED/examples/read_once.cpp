#include "ms4525do/ms4525do.hpp"

#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>

int main() {
  ms4525do::Ms4525do::Calibration calibration{};
  calibration.p_min_psi = -1.0;
  calibration.p_max_psi = 1.0;
  calibration.output_type = ms4525do::Ms4525do::OutputType::kTypeB_5To95;

  ms4525do::Ms4525do sensor("/dev/i2c-1", 0x28, 2, calibration);

  try {
    sensor.begin();
    const auto reading = sensor.read();

    std::cout << std::fixed << std::setprecision(6)
              << "pressure_psi=" << reading.pressure_psi << "\n"
              << "pressure_pa=" << reading.pressure_pa << "\n"
              << "temperature_c=" << reading.temperature_c << "\n"
              << "pressure_counts=" << reading.pressure_counts << "\n"
              << "temperature_counts=" << reading.temperature_counts << "\n"
              << "status=" << static_cast<int>(reading.status) << "\n";
    sensor.close();
    return EXIT_SUCCESS;
  } catch (const std::exception& ex) {
    std::cerr << "MS4525DO read failed: " << ex.what() << "\n";
    sensor.close();
    return EXIT_FAILURE;
  }
}
