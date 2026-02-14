#include "runtime/ipc/crc32.hpp"

namespace runtime {

uint32_t crc32_ieee(const uint8_t* data, std::size_t len) {
  static uint32_t table[256] = {};
  static bool initialized = false;

  if (!initialized) {
    for (uint32_t i = 0; i < 256; ++i) {
      uint32_t c = i;
      for (int j = 0; j < 8; ++j) {
        if ((c & 1U) != 0U) {
          c = 0xEDB88320U ^ (c >> 1U);
        } else {
          c >>= 1U;
        }
      }
      table[i] = c;
    }
    initialized = true;
  }

  uint32_t crc = 0xFFFFFFFFU;
  for (std::size_t i = 0; i < len; ++i) {
    const uint8_t idx = static_cast<uint8_t>((crc ^ data[i]) & 0xFFU);
    crc = table[idx] ^ (crc >> 8U);
  }
  return crc ^ 0xFFFFFFFFU;
}

}  // namespace runtime
