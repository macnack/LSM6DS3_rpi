#include "buzzer/engine.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>
#include <unordered_map>

namespace buzzer {
namespace {

int priority_value(Priority p) { return static_cast<int>(p); }

std::size_t alarm_index(AlarmId id) { return static_cast<std::size_t>(id); }

}  // namespace

struct BuzzerEngine::Impl {
  struct Request {
    Pattern pattern;
    PlayOptions options;
    std::string name;
    bool is_alarm = false;
    AlarmId alarm_id = AlarmId::BatteryFailsafe;
  };

  struct Cursor {
    std::size_t phase_index = 0;
    uint32_t repeats_done = 0;
    bool in_gap = false;
    bool phase_started = false;
    std::chrono::steady_clock::time_point deadline{};
  };

  explicit Impl(std::shared_ptr<BuzzerOutput> out)
      : output(std::move(out)), profile(make_default_profile()) {
    alarms_active.fill(false);
  }

  ~Impl() { shutdown(); }

  void ensure_running() {
    std::lock_guard<std::mutex> lock(mutex);
    if (running) {
      return;
    }

    output->begin();
    output->off();

    stop_requested = false;
    running = true;
    worker = std::thread([this] { worker_loop(); });
  }

  void shutdown() noexcept {
    std::thread local;
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (!running && !worker.joinable()) {
        return;
      }
      stop_requested = true;
      running = false;
      cv.notify_all();
      local = std::move(worker);
    }

    if (local.joinable()) {
      local.join();
    }

    try {
      output->off();
      output->close();
    } catch (...) {
    }
  }

  [[nodiscard]] bool any_alarm_active_locked() const {
    return std::any_of(alarms_active.begin(), alarms_active.end(), [](bool active) { return active; });
  }

  [[nodiscard]] std::optional<AlarmId> highest_alarm_locked() const {
    std::optional<AlarmId> best;
    int best_priority = -1;
    for (std::size_t i = 0; i < alarms_active.size(); ++i) {
      if (!alarms_active[i]) {
        continue;
      }
      const AlarmId candidate = static_cast<AlarmId>(i);
      const auto it = profile.alarms.find(candidate);
      if (it == profile.alarms.end()) {
        continue;
      }
      const int p = priority_value(it->second.options.priority);
      if (!best.has_value() || p > best_priority ||
          (p == best_priority && static_cast<std::size_t>(candidate) < static_cast<std::size_t>(*best))) {
        best = candidate;
        best_priority = p;
      }
    }
    return best;
  }

  [[nodiscard]] Request make_alarm_request_locked(AlarmId id) const {
    const auto it = profile.alarms.find(id);
    if (it == profile.alarms.end()) {
      throw std::runtime_error("No signal definition for alarm");
    }
    Request req;
    req.pattern = it->second.pattern;
    req.options = it->second.options;
    if (req.options.repeat == 0) {
      req.options.repeat = kRepeatForever;
    }
    req.name = alarm_name(id);
    req.is_alarm = true;
    req.alarm_id = id;
    if (req.options.token.empty()) {
      req.options.token = std::string("alarm.") + req.name;
    }
    return req;
  }

  [[nodiscard]] Request make_event_request_locked(EventId id) const {
    const auto it = profile.events.find(id);
    if (it == profile.events.end()) {
      throw std::runtime_error("No signal definition for event");
    }
    Request req;
    req.pattern = it->second.pattern;
    req.options = it->second.options;
    req.name = event_name(id);
    req.is_alarm = false;
    if (req.options.token.empty()) {
      req.options.token = std::string("event.") + req.name;
    }
    return req;
  }

  void reconcile_alarm_locked() {
    const auto top_alarm = highest_alarm_locked();
    if (!top_alarm.has_value()) {
      if (pending_preempt.has_value() && pending_preempt->is_alarm) {
        pending_preempt.reset();
      }
      if (current.has_value() && current->is_alarm) {
        current.reset();
        cursor = Cursor{};
        output->off();
      }
      return;
    }

    const Request top_req = make_alarm_request_locked(*top_alarm);
    if (!current.has_value()) {
      pending_preempt = top_req;
      return;
    }

    if (!current->is_alarm || current->alarm_id != top_req.alarm_id) {
      pending_preempt = top_req;
    }
  }

  [[nodiscard]] bool throttled_locked(const Request& req,
                                      const std::chrono::steady_clock::time_point now) const {
    if (req.options.throttle_ms == 0 || req.options.token.empty()) {
      return false;
    }
    const auto it = throttle_last.find(req.options.token);
    if (it == throttle_last.end()) {
      return false;
    }
    const auto elapsed =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - it->second).count();
    return elapsed >= 0 && elapsed < static_cast<long long>(req.options.throttle_ms);
  }

  void mark_emitted_locked(const Request& req,
                           const std::chrono::steady_clock::time_point now) {
    if (req.options.throttle_ms == 0 || req.options.token.empty()) {
      return;
    }
    throttle_last[req.options.token] = now;
  }

  void enqueue_locked(Request req) {
    if (queue_limit == 0 || queue.size() >= queue_limit) {
      return;
    }
    queue.push_back(std::move(req));
  }

  void apply_replace_locked(const Request& req) {
    if (!req.options.token.empty()) {
      queue.erase(std::remove_if(queue.begin(), queue.end(), [&](const Request& queued) {
                    return queued.options.token == req.options.token;
                  }),
                  queue.end());
    }
    enqueue_locked(req);
  }

  void schedule_request_locked(Request req, bool from_alarm_path) {
    if (!is_pattern_valid(req.pattern)) {
      return;
    }

    const auto now = std::chrono::steady_clock::now();
    if (!from_alarm_path && throttled_locked(req, now)) {
      return;
    }

    if (from_alarm_path) {
      pending_preempt = std::move(req);
      cv.notify_all();
      return;
    }

    const bool busy = current.has_value() || pending_preempt.has_value() ||
                      (any_alarm_active_locked() && !req.is_alarm);
    if (!busy) {
      pending_preempt = std::move(req);
      mark_emitted_locked(*pending_preempt, now);
      cv.notify_all();
      return;
    }

    if (current.has_value()) {
      const int req_priority = priority_value(req.options.priority);
      const int cur_priority = priority_value(current->options.priority);
      if (req_priority > cur_priority) {
        pending_preempt = std::move(req);
        mark_emitted_locked(*pending_preempt, now);
        cv.notify_all();
        return;
      }
    }

    switch (req.options.policy) {
      case BusyPolicy::Queue:
        enqueue_locked(req);
        break;
      case BusyPolicy::Replace:
        apply_replace_locked(req);
        break;
      case BusyPolicy::DropIfBusy:
        break;
    }

    mark_emitted_locked(req, now);
    cv.notify_all();
  }

  [[nodiscard]] bool should_wake_locked() const {
    return stop_requested || pending_preempt.has_value() || !running;
  }

  void advance_cursor_locked() {
    cursor.phase_started = false;

    if (!current.has_value()) {
      return;
    }

    if (cursor.in_gap) {
      cursor.in_gap = false;
      cursor.phase_index = 0;
      return;
    }

    ++cursor.phase_index;
    if (cursor.phase_index < current->pattern.size()) {
      return;
    }

    cursor.phase_index = 0;
    const bool forever = current->options.repeat == kRepeatForever;
    if (forever || (cursor.repeats_done + 1U) < current->options.repeat) {
      ++cursor.repeats_done;
      if (current->options.gap_ms > 0) {
        cursor.in_gap = true;
      }
      return;
    }

    output->off();
    current.reset();
    cursor = Cursor{};
  }

  void worker_loop() {
    std::unique_lock<std::mutex> lock(mutex);

    while (!stop_requested) {
      reconcile_alarm_locked();

      if (pending_preempt.has_value()) {
        current = std::move(pending_preempt);
        pending_preempt.reset();
        cursor = Cursor{};
        output->off();
      }

      if (!current.has_value()) {
        if (!queue.empty()) {
          current = std::move(queue.front());
          queue.pop_front();
          cursor = Cursor{};
          output->off();
        } else {
          cv.wait(lock, [&] {
            return stop_requested || pending_preempt.has_value() || !queue.empty() || any_alarm_active_locked();
          });
          continue;
        }
      }

      const auto now = std::chrono::steady_clock::now();
      if (muted || now < silence_until) {
        output->off();
        if (muted) {
          cv.wait(lock, [&] { return should_wake_locked() || !muted; });
        } else {
          cv.wait_until(lock, silence_until, [&] {
            return should_wake_locked() || muted || std::chrono::steady_clock::now() >= silence_until;
          });
        }
        continue;
      }

      if (!cursor.phase_started) {
        if (cursor.in_gap) {
          output->off();
          cursor.deadline = now + std::chrono::milliseconds(current->options.gap_ms);
          cursor.phase_started = true;
        } else {
          if (current->pattern.empty()) {
            current.reset();
            cursor = Cursor{};
            continue;
          }
          const Phase& phase = current->pattern[cursor.phase_index];
          if (phase.state == PhaseState::On) {
            output->on();
          } else {
            output->off();
          }
          cursor.deadline = now + std::chrono::milliseconds(phase.duration_ms);
          cursor.phase_started = true;
        }
      }

      cv.wait_until(lock, cursor.deadline, [&] {
        return stop_requested || pending_preempt.has_value() || muted ||
               std::chrono::steady_clock::now() < silence_until;
      });

      if (stop_requested) {
        break;
      }
      if (pending_preempt.has_value()) {
        continue;
      }
      if (muted || std::chrono::steady_clock::now() < silence_until) {
        continue;
      }
      if (std::chrono::steady_clock::now() < cursor.deadline) {
        continue;
      }

      advance_cursor_locked();
    }

    output->off();
    current.reset();
    pending_preempt.reset();
    queue.clear();
  }

  std::shared_ptr<BuzzerOutput> output;
  SignalProfile profile;

  mutable std::mutex mutex;
  std::condition_variable cv;
  std::thread worker;

  bool running = false;
  bool stop_requested = false;
  bool muted = false;
  std::chrono::steady_clock::time_point silence_until =
      std::chrono::steady_clock::time_point::min();

  std::size_t queue_limit = 64;
  std::deque<Request> queue;
  std::optional<Request> current;
  std::optional<Request> pending_preempt;
  Cursor cursor{};

  std::array<bool, 5> alarms_active{};
  std::unordered_map<std::string, std::chrono::steady_clock::time_point> throttle_last;
};

BuzzerEngine::BuzzerEngine(HardwareBuzzerConfig hw_cfg)
    : impl_(std::make_unique<Impl>(std::make_shared<HardwareBuzzer>(std::move(hw_cfg)))) {}

BuzzerEngine::BuzzerEngine(std::shared_ptr<BuzzerOutput> output)
    : impl_(std::make_unique<Impl>(std::move(output))) {
  if (impl_->output == nullptr) {
    throw std::invalid_argument("BuzzerEngine requires non-null BuzzerOutput");
  }
}

BuzzerEngine::~BuzzerEngine() { close(); }

void BuzzerEngine::begin() { impl_->ensure_running(); }

void BuzzerEngine::close() noexcept { impl_->shutdown(); }

void BuzzerEngine::play(const Pattern& pattern, PlayOptions options, const std::string& name) {
  begin();
  Impl::Request req;
  req.pattern = pattern;
  req.options = std::move(options);
  req.name = name.empty() ? "custom" : name;

  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->schedule_request_locked(std::move(req), false);
}

void BuzzerEngine::stop() {
  begin();
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->queue.clear();
  impl_->current.reset();
  impl_->pending_preempt.reset();
  impl_->cursor = Impl::Cursor{};
  impl_->alarms_active.fill(false);
  impl_->output->off();
  impl_->cv.notify_all();
}

bool BuzzerEngine::is_playing() const noexcept {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->current.has_value() || impl_->pending_preempt.has_value();
}

std::string BuzzerEngine::current() const {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (impl_->pending_preempt.has_value()) {
    return impl_->pending_preempt->name;
  }
  if (impl_->current.has_value()) {
    return impl_->current->name;
  }
  return {};
}

void BuzzerEngine::notify(EventId id) {
  begin();
  std::lock_guard<std::mutex> lock(impl_->mutex);
  auto req = impl_->make_event_request_locked(id);
  impl_->schedule_request_locked(std::move(req), false);
}

void BuzzerEngine::set_alarm(AlarmId id, bool active) {
  begin();
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->alarms_active[alarm_index(id)] = active;
  impl_->reconcile_alarm_locked();
  impl_->cv.notify_all();
}

void BuzzerEngine::mute(bool enabled) {
  begin();
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->muted = enabled;
  if (enabled) {
    impl_->output->off();
  }
  impl_->cv.notify_all();
}

void BuzzerEngine::silence_until(std::chrono::steady_clock::time_point deadline) {
  begin();
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (deadline > impl_->silence_until) {
    impl_->silence_until = deadline;
  }
  impl_->output->off();
  impl_->cv.notify_all();
}

void BuzzerEngine::silence_for(std::chrono::milliseconds duration) {
  silence_until(std::chrono::steady_clock::now() + duration);
}

void BuzzerEngine::set_profile(const SignalProfile& profile) {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->profile = profile;
  impl_->reconcile_alarm_locked();
  impl_->cv.notify_all();
}

void BuzzerEngine::load_profile_json(const std::string& path) {
  set_profile(load_profile_json_file(path));
}

SignalProfile BuzzerEngine::profile_snapshot() const {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->profile;
}

void BuzzerEngine::set_queue_limit(std::size_t limit) {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->queue_limit = limit;
  while (impl_->queue.size() > impl_->queue_limit) {
    impl_->queue.pop_front();
  }
}

BuzzerEngine& default_engine() {
  static BuzzerEngine engine;
  return engine;
}

void notify(EventId id) { default_engine().notify(id); }

void set_alarm(AlarmId id, bool active) { default_engine().set_alarm(id, active); }

void mute(bool enabled) { default_engine().mute(enabled); }

void silence_until(std::chrono::steady_clock::time_point deadline) {
  default_engine().silence_until(deadline);
}

}  // namespace buzzer
