#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace runtime {

uint32_t crc32_ieee(const uint8_t* data, std::size_t len);

}  // namespace runtime
