#include "bmp390/bmp390.hpp"

#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>

int main() {
  bmp390::Bmp390 sensor("/dev/i2c-1", 0x77);

  try {
    sensor.begin();
    const auto reading = sensor.read();
    std::cout << std::fixed << std::setprecision(3)
              << "temperature_c=" << reading.temperature_c << "\n"
              << "pressure_pa=" << reading.pressure_pa << "\n"
              << "pressure_hpa=" << sensor.read_pressure_hpa() << "\n";
    sensor.close(true);
    return EXIT_SUCCESS;
  } catch (const std::exception& ex) {
    std::cerr << "BMP390 read failed: " << ex.what() << '\n';
    sensor.close(true);
    return EXIT_FAILURE;
  }
}
