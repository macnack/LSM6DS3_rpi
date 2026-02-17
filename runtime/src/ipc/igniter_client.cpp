#include "runtime/ipc/igniter_client.hpp"

#include "runtime/common/time.hpp"

#include <stdexcept>

namespace runtime {

IgniterCommandClient::IgniterCommandClient(std::string mailbox_name)
    : mailbox_(std::move(mailbox_name), false) {}

void IgniterCommandClient::open() { mailbox_.open(false); }

void IgniterCommandClient::close() noexcept { mailbox_.close(); }

void IgniterCommandClient::arm() { send(IgniterCommandAction::Arm, 0, {0, 0, 0, 0}); }

void IgniterCommandClient::disarm() { send(IgniterCommandAction::Disarm, 0, {0, 0, 0, 0}); }

void IgniterCommandClient::clear_fault() {
  send(IgniterCommandAction::ClearFault, 0, {0, 0, 0, 0});
}

void IgniterCommandClient::fire_mask(uint8_t mask,
                                     const std::array<uint32_t, kIgniterCount>& duration_ms) {
  send(IgniterCommandAction::FireMask, mask, duration_ms);
}

void IgniterCommandClient::fire_one(uint8_t channel, uint32_t duration_ms) {
  if (channel >= kIgniterCount) {
    throw std::invalid_argument("fire_one channel must be in [0, kIgniterCount)");
  }
  std::array<uint32_t, kIgniterCount> duration{0, 0, 0, 0};
  duration[channel] = duration_ms;
  send(IgniterCommandAction::FireMask, static_cast<uint8_t>(1U << channel), duration);
}

void IgniterCommandClient::fire_all(uint32_t duration_ms) {
  std::array<uint32_t, kIgniterCount> duration{duration_ms, duration_ms, duration_ms, duration_ms};
  send(IgniterCommandAction::FireMask, 0x0FU, duration);
}

void IgniterCommandClient::send(IgniterCommandAction action, uint8_t mask,
                                const std::array<uint32_t, kIgniterCount>& duration_ms) {
  IgniterCommandMsg msg{};
  fill_message_header(msg, ++seq_, monotonic_time_ns());
  msg.action = static_cast<uint8_t>(action);
  msg.fire_mask = mask;
  msg.duration_ms = duration_ms;
  finalize_message_crc(msg);
  mailbox_.write(msg);
}

IgniterStatusClient::IgniterStatusClient(std::string mailbox_name)
    : mailbox_(std::move(mailbox_name), false) {}

void IgniterStatusClient::open() { mailbox_.open(false); }

void IgniterStatusClient::close() noexcept { mailbox_.close(); }

bool IgniterStatusClient::try_read(IgniterStatusMsg& out, uint64_t* stable_seq) const {
  return mailbox_.try_read(out, stable_seq);
}

}  // namespace runtime
