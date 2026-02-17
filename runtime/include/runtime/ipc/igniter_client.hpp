#pragma once

#include "runtime/ipc/messages.hpp"
#include "runtime/ipc/shm_mailbox.hpp"

#include <array>
#include <cstdint>
#include <string>

namespace runtime {

class IgniterCommandClient {
 public:
  explicit IgniterCommandClient(std::string mailbox_name = "/rt_igniter_command_v1");

  void open();
  void close() noexcept;

  void arm();
  void disarm();
  void clear_fault();
  void fire_mask(uint8_t mask, const std::array<uint32_t, kIgniterCount>& duration_ms);
  void fire_one(uint8_t channel, uint32_t duration_ms);
  void fire_all(uint32_t duration_ms);

 private:
  void send(IgniterCommandAction action, uint8_t mask,
            const std::array<uint32_t, kIgniterCount>& duration_ms);

  uint64_t seq_ = 0;
  ShmMailbox<IgniterCommandMsg> mailbox_;
};

class IgniterStatusClient {
 public:
  explicit IgniterStatusClient(std::string mailbox_name = "/rt_igniter_status_v1");

  void open();
  void close() noexcept;
  [[nodiscard]] bool try_read(IgniterStatusMsg& out, uint64_t* stable_seq = nullptr) const;

 private:
  ShmMailbox<IgniterStatusMsg> mailbox_;
};

}  // namespace runtime
