#pragma once

#include "buzzer/hardware_buzzer.hpp"
#include "buzzer/patterns.hpp"
#include "buzzer/profile.hpp"

#include <chrono>
#include <cstddef>
#include <memory>
#include <string>

namespace buzzer {

class BuzzerEngine {
 public:
  explicit BuzzerEngine(HardwareBuzzerConfig hw_cfg = {});
  explicit BuzzerEngine(std::shared_ptr<BuzzerOutput> output);
  ~BuzzerEngine();

  BuzzerEngine(const BuzzerEngine&) = delete;
  BuzzerEngine& operator=(const BuzzerEngine&) = delete;

  void begin();
  void close() noexcept;

  void play(const Pattern& pattern, PlayOptions options = {}, const std::string& name = "custom");
  void stop();
  [[nodiscard]] bool is_playing() const noexcept;
  [[nodiscard]] std::string current() const;

  void notify(EventId id);
  void set_alarm(AlarmId id, bool active = true);
  void mute(bool enabled = true);
  void silence_until(std::chrono::steady_clock::time_point deadline);
  void silence_for(std::chrono::milliseconds duration);

  void set_profile(const SignalProfile& profile);
  void load_profile_json(const std::string& path);

  [[nodiscard]] SignalProfile profile_snapshot() const;
  void set_queue_limit(std::size_t limit);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

[[nodiscard]] BuzzerEngine& default_engine();
void notify(EventId id);
void set_alarm(AlarmId id, bool active = true);
void mute(bool enabled = true);
void silence_until(std::chrono::steady_clock::time_point deadline);

}  // namespace buzzer
