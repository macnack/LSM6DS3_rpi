#pragma once

#include "runtime/common/constants.hpp"
#include "runtime/ipc/crc32.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

namespace runtime {

#pragma pack(push, 1)

struct SensorSnapshotMsg {
  uint32_t msg_magic = kMessageMagic;
  uint16_t msg_version = kMessageVersion;
  uint16_t payload_bytes = 0;
  uint64_t seq = 0;
  uint64_t t_ns = 0;
  uint8_t imu_valid = 0;
  uint8_t baro_valid = 0;
  uint8_t reserved0[6] = {0, 0, 0, 0, 0, 0};
  double ax_mps2 = 0.0;
  double ay_mps2 = 0.0;
  double az_mps2 = 0.0;
  double gx_rads = 0.0;
  double gy_rads = 0.0;
  double gz_rads = 0.0;
  double pressure_pa = 101325.0;
  double temperature_c = 20.0;
  uint32_t crc32 = 0;
};

struct ExternalEstimatorStateMsg {
  uint32_t msg_magic = kMessageMagic;
  uint16_t msg_version = kMessageVersion;
  uint16_t payload_bytes = 0;
  uint64_t seq = 0;
  uint64_t t_ns = 0;
  uint8_t valid = 0;
  uint8_t reserved0[7] = {0, 0, 0, 0, 0, 0, 0};
  std::array<float, 4> q_body_to_ned{1.0F, 0.0F, 0.0F, 0.0F};
  std::array<float, 3> vel_ned_mps{0.0F, 0.0F, 0.0F};
  std::array<float, 3> pos_ned_m{0.0F, 0.0F, 0.0F};
  uint32_t crc32 = 0;
};

struct ExternalControllerCommandMsg {
  uint32_t msg_magic = kMessageMagic;
  uint16_t msg_version = kMessageVersion;
  uint16_t payload_bytes = 0;
  uint64_t seq = 0;
  uint64_t t_ns = 0;
  uint8_t armed = 0;
  uint8_t reserved0[7] = {0, 0, 0, 0, 0, 0, 0};
  std::array<float, 4> servo_norm{0.0F, 0.0F, 0.0F, 0.0F};
  uint32_t crc32 = 0;
};

enum class IgniterCommandAction : uint8_t {
  None = 0,
  Arm = 1,
  Disarm = 2,
  FireMask = 3,
  ClearFault = 4,
};

struct IgniterCommandMsg {
  uint32_t msg_magic = kMessageMagic;
  uint16_t msg_version = kMessageVersion;
  uint16_t payload_bytes = 0;
  uint64_t seq = 0;
  uint64_t t_ns = 0;
  uint8_t action = static_cast<uint8_t>(IgniterCommandAction::None);
  uint8_t fire_mask = 0;
  uint8_t reserved0[6] = {0, 0, 0, 0, 0, 0};
  std::array<uint32_t, kIgniterCount> duration_ms{0, 0, 0, 0};
  uint32_t crc32 = 0;
};

struct IgniterStatusMsg {
  uint32_t msg_magic = kMessageMagic;
  uint16_t msg_version = kMessageVersion;
  uint16_t payload_bytes = 0;
  uint64_t seq = 0;
  uint64_t t_ns = 0;
  uint8_t armed = 0;
  uint8_t global_fault_latched = 0;
  uint8_t active_mask = 0;
  uint8_t reserved0[5] = {0, 0, 0, 0, 0};
  std::array<uint8_t, kIgniterCount> state{0, 0, 0, 0};
  std::array<uint8_t, kIgniterCount> fault{0, 0, 0, 0};
  std::array<uint32_t, kIgniterCount> remaining_ms{0, 0, 0, 0};
  uint32_t crc32 = 0;
};

#pragma pack(pop)

// Backward-compatible aliases. Prefer External* names in new code.
using PyEstimatorStateMsg = ExternalEstimatorStateMsg;
using PyControllerCommandMsg = ExternalControllerCommandMsg;

template <typename Msg>
inline constexpr uint16_t payload_size_bytes() {
  return static_cast<uint16_t>(offsetof(Msg, crc32) - offsetof(Msg, seq));
}

template <typename Msg>
inline void fill_message_header(Msg& msg, uint64_t seq, uint64_t t_ns) {
  static_assert(std::is_trivially_copyable<Msg>::value, "Msg must be trivially copyable");
  msg.msg_magic = kMessageMagic;
  msg.msg_version = kMessageVersion;
  msg.payload_bytes = payload_size_bytes<Msg>();
  msg.seq = seq;
  msg.t_ns = t_ns;
}

template <typename Msg>
inline uint32_t compute_message_crc(const Msg& msg) {
  static_assert(std::is_trivially_copyable<Msg>::value, "Msg must be trivially copyable");
  return crc32_ieee(reinterpret_cast<const uint8_t*>(&msg), offsetof(Msg, crc32));
}

template <typename Msg>
inline void finalize_message_crc(Msg& msg) {
  msg.crc32 = compute_message_crc(msg);
}

template <typename Msg>
inline bool validate_message_crc(const Msg& msg) {
  return compute_message_crc(msg) == msg.crc32;
}

}  // namespace runtime
