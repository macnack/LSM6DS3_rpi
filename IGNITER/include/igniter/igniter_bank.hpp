#pragma once

#include "igniter/gpio_batch.hpp"
#include "igniter/igniter.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <string>

namespace igniter {

enum class FaultPolicy : uint8_t {
  Global = 0,
  Isolated = 1,
};

struct ChannelConfig {
  bool enabled = false;
  Igniter::Config igniter{};
  VN5E160S::Config driver{};
};

class IgniterBank {
 public:
  struct Config {
    FaultPolicy faultPolicy = FaultPolicy::Global;
    std::array<ChannelConfig, kIgniterChannels> channels{};
  };

  struct Snapshot {
    bool armed = false;
    bool globalFaultLatched = false;
    uint8_t activeMask = 0;
    std::array<State, kIgniterChannels> states{
        State::Disarmed,
        State::Disarmed,
        State::Disarmed,
        State::Disarmed,
    };
    std::array<Fault, kIgniterChannels> faults{
        Fault::None,
        Fault::None,
        Fault::None,
        Fault::None,
    };
    std::array<uint32_t, kIgniterChannels> remainingMs{0, 0, 0, 0};
  };

  explicit IgniterBank(Config cfg);
  IgniterBank(Config cfg, std::unique_ptr<GpioBatchOut> batch_out,
              std::unique_ptr<GpioBatchIn> batch_in);

  void bind_hardware(const std::string& output_chip, const std::string& status_chip,
                     const std::array<uint32_t, kIgniterChannels>& output_lines,
                     const std::array<uint32_t, kIgniterChannels>& status_lines);

  void init(uint64_t now_ms);
  bool arm(uint64_t now_ms);
  void disarm(uint64_t now_ms);
  bool fire_mask(uint8_t fire_mask,
                 const std::array<uint32_t, kIgniterChannels>& duration_ms,
                 uint64_t now_ms);
  bool fire_one(uint8_t channel, uint32_t duration_ms, uint64_t now_ms);
  bool fire_all(uint32_t duration_ms, uint64_t now_ms);
  void clear_fault(uint64_t now_ms);
  Snapshot update(uint64_t now_ms, bool force_disarm = false);

  [[nodiscard]] Snapshot snapshot(uint64_t now_ms) const;

  // For simulation/testing only (works with SimGpioBatchIn backend).
  bool set_sim_status_values(const std::array<uint8_t, kIgniterChannels>& values);

 private:
  void commit_outputs(uint8_t mask);
  void commit_all_outputs();
  [[nodiscard]] std::array<uint8_t, kIgniterChannels> current_outputs() const;
  [[nodiscard]] bool enabled_channel(uint8_t channel) const;

  Config cfg_;
  std::array<Igniter, kIgniterChannels> igniters_;
  std::unique_ptr<GpioBatchOut> out_;
  std::unique_ptr<GpioBatchIn> in_;
  bool io_bound_ = false;
  bool armed_ = false;
  bool global_fault_latched_ = false;
};

const char* fault_name(Fault fault) noexcept;
const char* state_name(State state) noexcept;

}  // namespace igniter
