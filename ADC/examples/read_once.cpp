#include "ads1115/ads1115.hpp"

#include <cstdint>
#include <exception>
#include <iostream>

int main() {
  try {
    ads1115::Ads1115 adc;
    adc.begin();
    const int16_t sample = adc.read_adc(0);
    std::cout << "ADS1115 CH0 raw=" << sample << "\n";
    adc.close();
    return 0;
  } catch (const std::exception& ex) {
    std::cerr << "ADS1115 read_once failed: " << ex.what() << "\n";
    return 1;
  }
}
