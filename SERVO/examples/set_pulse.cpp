#include "servo/hardware_pwm.hpp"

#include <exception>
#include <iostream>

int main() {
  try {
    servo::Servo pwm_servo(0, 0);
    pwm_servo.begin();
    pwm_servo.set_angle_deg(0.0);
    std::cout << "Servo set to 0 deg at pulse " << pwm_servo.pulse_width_us() << " us" << std::endl;
    pwm_servo.close();
    return 0;
  } catch (const std::exception& ex) {
    std::cerr << "Error: " << ex.what() << std::endl;
    return 1;
  }
}
