#pragma once

#include <cstddef>
#include <cstdint>

namespace runtime {

constexpr std::size_t kServoCount = 4;
constexpr uint32_t kMessageMagic = 0x52544331U;  // "RTC1"
constexpr uint16_t kMessageVersion = 1;

}  // namespace runtime
