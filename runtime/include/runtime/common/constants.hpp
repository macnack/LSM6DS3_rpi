#pragma once

#include <cstddef>
#include <cstdint>

namespace runtime {

constexpr std::size_t kServoCount = 4;
// IPC review note:
// - Bump kMessageVersion for wire-format breaking changes (field layout/order/type/size, header semantics, CRC scope).
// - Change kMessageMagic only for a new protocol family/channel that must be hard-isolated from existing decoders.
constexpr uint32_t kMessageMagic = 0x52544331U;  // "RTC1"
constexpr uint16_t kMessageVersion = 1;

}  // namespace runtime
