#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace buzzer {

enum class PhaseState : uint8_t {
  Off = 0,
  On = 1,
};

struct Phase {
  PhaseState state = PhaseState::Off;
  uint32_t duration_ms = 0;
};

using Pattern = std::vector<Phase>;

constexpr uint32_t kRepeatForever = 0xFFFFFFFFU;

enum class Priority : uint8_t {
  Info = 10,
  Alarm = 20,
  Critical = 30,
};

enum class BusyPolicy : uint8_t {
  Queue = 0,
  Replace = 1,
  DropIfBusy = 2,
};

struct PlayOptions {
  uint32_t repeat = 1;
  uint32_t gap_ms = 0;
  Priority priority = Priority::Info;
  BusyPolicy policy = BusyPolicy::Queue;
  uint32_t throttle_ms = 0;
  std::string token;
};

enum class EventId : uint8_t {
  Arming = 0,
  ArmingFailure = 1,
  Disarmed = 2,
  GyroInitDone = 3,
  ReadyToArm = 4,
};

enum class AlarmId : uint8_t {
  BatteryFailsafe = 0,
  EkfFailure = 1,
  LostVehicle = 2,
  EvacuationTemporal3 = 3,
  MissingSos = 4,
};

struct SignalDefinition {
  Pattern pattern;
  PlayOptions options;
};

struct EnumClassHash {
  template <typename T>
  std::size_t operator()(T value) const noexcept {
    return static_cast<std::size_t>(value);
  }
};

struct SignalProfile {
  std::unordered_map<EventId, SignalDefinition, EnumClassHash> events;
  std::unordered_map<AlarmId, SignalDefinition, EnumClassHash> alarms;
  std::unordered_map<std::string, uint32_t> base_times_ms;
  std::string name;
};

[[nodiscard]] const char* event_name(EventId id) noexcept;
[[nodiscard]] const char* alarm_name(AlarmId id) noexcept;
[[nodiscard]] const char* priority_name(Priority p) noexcept;
[[nodiscard]] const char* busy_policy_name(BusyPolicy p) noexcept;

[[nodiscard]] std::optional<EventId> parse_event_id(const std::string& value);
[[nodiscard]] std::optional<AlarmId> parse_alarm_id(const std::string& value);
[[nodiscard]] std::optional<Priority> parse_priority(const std::string& value);
[[nodiscard]] std::optional<BusyPolicy> parse_busy_policy(const std::string& value);

[[nodiscard]] bool is_pattern_valid(const Pattern& pattern) noexcept;

}  // namespace buzzer
