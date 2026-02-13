#include "lsm6ds3/lsm6ds3.hpp"

#include <array>
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>

namespace {

void print_vec3(const char* label, const std::array<double, 3>& values) {
  std::cout << label << ": ["
            << std::fixed << std::setprecision(6)
            << values[0] << ", " << values[1] << ", " << values[2]
            << "]\n";
}

}  // namespace

int main() {
  lsm6ds3::Lsm6ds3 sensor("/dev/i2c-1", 0x6A);

  try {
    sensor.begin();
    const auto accel_mps2 = sensor.read_accel_si();
    const auto gyro_rads = sensor.read_gyro_si();

    print_vec3("accel (m/s^2)", accel_mps2);
    print_vec3("gyro (rad/s)", gyro_rads);

    sensor.close(true);
    return EXIT_SUCCESS;
  } catch (const std::exception& ex) {
    std::cerr << "LSM6DS3 read failed: " << ex.what() << '\n';
    sensor.close(true);
    return EXIT_FAILURE;
  }
}
