#include "buzzer/engine.hpp"

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace {

#define REQUIRE(cond, msg)      \
  do {                          \
    if (!(cond)) {              \
      std::cerr << msg << "\n"; \
      return false;             \
    }                           \
  } while (0)

class FakeOutput final : public buzzer::BuzzerOutput {
 public:
  void begin() override {
    std::lock_guard<std::mutex> lock(mutex_);
    open_ = true;
    on_ = false;
  }

  void close() noexcept override {
    std::lock_guard<std::mutex> lock(mutex_);
    on_ = false;
    open_ = false;
  }

  void on() override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!open_) {
      throw std::runtime_error("FakeOutput not open");
    }
    on_ = true;
    ++on_count_;
  }

  void off() override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!open_) {
      return;
    }
    on_ = false;
    ++off_count_;
  }

  [[nodiscard]] bool is_on() const noexcept override {
    std::lock_guard<std::mutex> lock(mutex_);
    return on_;
  }

  [[nodiscard]] bool is_open() const noexcept override {
    std::lock_guard<std::mutex> lock(mutex_);
    return open_;
  }

  [[nodiscard]] int on_count() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return on_count_;
  }

  [[nodiscard]] int off_count() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return off_count_;
  }

 private:
  mutable std::mutex mutex_;
  bool open_ = false;
  bool on_ = false;
  int on_count_ = 0;
  int off_count_ = 0;
};

bool wait_until(const std::function<bool()>& pred, std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (pred()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return pred();
}

buzzer::Pattern make_pattern(uint32_t on_ms, uint32_t off_ms = 0) {
  buzzer::Pattern p;
  p.push_back(buzzer::Phase{buzzer::PhaseState::On, on_ms});
  if (off_ms > 0) {
    p.push_back(buzzer::Phase{buzzer::PhaseState::Off, off_ms});
  }
  return p;
}

bool test_forever_play_and_stop() {
  auto output = std::make_shared<FakeOutput>();
  buzzer::BuzzerEngine engine(output);
  engine.begin();

  buzzer::PlayOptions opts;
  opts.repeat = buzzer::kRepeatForever;
  engine.play(make_pattern(40, 40), opts, "forever");

  REQUIRE(wait_until([&] { return engine.is_playing(); }, std::chrono::milliseconds(100)),
          "forever playback should start");

  engine.stop();
  REQUIRE(wait_until([&] { return !engine.is_playing(); }, std::chrono::milliseconds(100)),
          "stop() should stop playback immediately");
  REQUIRE(!output->is_on(), "output should be OFF after stop");

  engine.close();
  return true;
}

bool test_preemption_critical_over_info() {
  auto output = std::make_shared<FakeOutput>();
  buzzer::BuzzerEngine engine(output);
  engine.begin();

  buzzer::PlayOptions info;
  info.priority = buzzer::Priority::Info;
  info.policy = buzzer::BusyPolicy::Queue;
  engine.play(make_pattern(500), info, "info_long");

  REQUIRE(wait_until([&] { return engine.current() == "info_long"; }, std::chrono::milliseconds(80)),
          "info_long should become current");

  buzzer::PlayOptions critical;
  critical.priority = buzzer::Priority::Critical;
  critical.policy = buzzer::BusyPolicy::Queue;
  engine.play(make_pattern(120), critical, "critical_now");

  REQUIRE(wait_until([&] { return engine.current() == "critical_now"; }, std::chrono::milliseconds(120)),
          "critical request should preempt info request");

  engine.stop();
  engine.close();
  return true;
}

bool test_info_does_not_preempt_alarm() {
  auto output = std::make_shared<FakeOutput>();
  buzzer::BuzzerEngine engine(output);
  engine.begin();

  engine.set_alarm(buzzer::AlarmId::LostVehicle, true);
  REQUIRE(wait_until([&] { return engine.current() == "LOST_VEHICLE"; }, std::chrono::milliseconds(120)),
          "LOST_VEHICLE alarm should start");

  buzzer::PlayOptions info;
  info.priority = buzzer::Priority::Info;
  info.policy = buzzer::BusyPolicy::Queue;
  engine.play(make_pattern(100), info, "info_event");

  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  REQUIRE(engine.current() == "LOST_VEHICLE", "info event must not preempt active alarm");

  engine.set_alarm(buzzer::AlarmId::LostVehicle, false);
  engine.stop();
  engine.close();
  return true;
}

bool test_replace_and_throttle() {
  auto output = std::make_shared<FakeOutput>();
  buzzer::BuzzerEngine engine(output);
  engine.begin();

  buzzer::PlayOptions first;
  first.policy = buzzer::BusyPolicy::Queue;
  first.priority = buzzer::Priority::Info;
  engine.play(make_pattern(120), first, "busy");

  buzzer::PlayOptions queued;
  queued.policy = buzzer::BusyPolicy::Queue;
  queued.token = "evt.token";
  queued.priority = buzzer::Priority::Info;
  engine.play(make_pattern(80), queued, "queued_old");

  buzzer::PlayOptions replace = queued;
  replace.policy = buzzer::BusyPolicy::Replace;
  engine.play(make_pattern(80), replace, "queued_new");

  REQUIRE(wait_until([&] { return engine.current() == "queued_new"; }, std::chrono::milliseconds(400)),
          "REPLACE should keep only latest token payload");

  buzzer::PlayOptions throttled;
  throttled.policy = buzzer::BusyPolicy::Queue;
  throttled.token = "throttle.same";
  throttled.throttle_ms = 1000;
  engine.play(make_pattern(20), throttled, "throttle_1");
  engine.play(make_pattern(20), throttled, "throttle_2");

  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  REQUIRE(output->on_count() >= 1, "at least one throttled event should play");

  engine.stop();
  engine.close();
  return true;
}

}  // namespace

int main() {
  if (!test_forever_play_and_stop()) {
    return EXIT_FAILURE;
  }
  if (!test_preemption_critical_over_info()) {
    return EXIT_FAILURE;
  }
  if (!test_info_does_not_preempt_alarm()) {
    return EXIT_FAILURE;
  }
  if (!test_replace_and_throttle()) {
    return EXIT_FAILURE;
  }
  std::cout << "buzzer_unit_engine: ok\n";
  return EXIT_SUCCESS;
}
