#include "buzzer/profile.hpp"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

namespace buzzer {
namespace {

std::string normalize_symbol(const std::string& value) {
  std::string out;
  out.reserve(value.size());
  for (const unsigned char ch : value) {
    if (std::isalnum(ch)) {
      out.push_back(static_cast<char>(std::toupper(ch)));
    }
  }
  return out;
}

struct JsonValue {
  using array_t = std::vector<JsonValue>;
  using object_t = std::map<std::string, JsonValue>;

  std::variant<std::nullptr_t, bool, double, std::string, array_t, object_t> data;

  [[nodiscard]] bool is_null() const { return std::holds_alternative<std::nullptr_t>(data); }
  [[nodiscard]] bool is_bool() const { return std::holds_alternative<bool>(data); }
  [[nodiscard]] bool is_number() const { return std::holds_alternative<double>(data); }
  [[nodiscard]] bool is_string() const { return std::holds_alternative<std::string>(data); }
  [[nodiscard]] bool is_array() const { return std::holds_alternative<array_t>(data); }
  [[nodiscard]] bool is_object() const { return std::holds_alternative<object_t>(data); }

  [[nodiscard]] const bool& as_bool() const { return std::get<bool>(data); }
  [[nodiscard]] const double& as_number() const { return std::get<double>(data); }
  [[nodiscard]] const std::string& as_string() const { return std::get<std::string>(data); }
  [[nodiscard]] const array_t& as_array() const { return std::get<array_t>(data); }
  [[nodiscard]] const object_t& as_object() const { return std::get<object_t>(data); }
};

class JsonParser {
 public:
  explicit JsonParser(std::string text) : text_(std::move(text)) {}

  JsonValue parse() {
    skip_ws();
    JsonValue value = parse_value();
    skip_ws();
    if (!eof()) {
      throw std::runtime_error("Trailing data after JSON root");
    }
    return value;
  }

 private:
  [[nodiscard]] bool eof() const { return pos_ >= text_.size(); }

  [[nodiscard]] char peek() const {
    if (eof()) {
      return '\0';
    }
    return text_[pos_];
  }

  [[nodiscard]] char get() {
    if (eof()) {
      throw std::runtime_error("Unexpected end of JSON");
    }
    return text_[pos_++];
  }

  void skip_ws() {
    while (!eof()) {
      const unsigned char ch = static_cast<unsigned char>(text_[pos_]);
      if (!std::isspace(ch)) {
        break;
      }
      ++pos_;
    }
  }

  void expect(char expected) {
    const char got = get();
    if (got != expected) {
      throw std::runtime_error(std::string("Expected '") + expected + "' but got '" + got + "'");
    }
  }

  JsonValue parse_value() {
    skip_ws();
    const char ch = peek();
    if (ch == '{') {
      return parse_object();
    }
    if (ch == '[') {
      return parse_array();
    }
    if (ch == '"') {
      return JsonValue{parse_string()};
    }
    if (ch == 't' || ch == 'f') {
      return JsonValue{parse_bool()};
    }
    if (ch == 'n') {
      parse_null();
      return JsonValue{nullptr};
    }
    if (ch == '-' || (ch >= '0' && ch <= '9')) {
      return JsonValue{parse_number()};
    }
    throw std::runtime_error(std::string("Unexpected JSON token '") + ch + "'");
  }

  JsonValue parse_object() {
    JsonValue::object_t out;
    expect('{');
    skip_ws();
    if (peek() == '}') {
      get();
      return JsonValue{out};
    }

    while (true) {
      skip_ws();
      if (peek() != '"') {
        throw std::runtime_error("Expected object key string");
      }
      const std::string key = parse_string();
      skip_ws();
      expect(':');
      skip_ws();
      out[key] = parse_value();
      skip_ws();
      if (peek() == '}') {
        get();
        break;
      }
      expect(',');
    }

    return JsonValue{out};
  }

  JsonValue parse_array() {
    JsonValue::array_t out;
    expect('[');
    skip_ws();
    if (peek() == ']') {
      get();
      return JsonValue{out};
    }

    while (true) {
      skip_ws();
      out.push_back(parse_value());
      skip_ws();
      if (peek() == ']') {
        get();
        break;
      }
      expect(',');
    }
    return JsonValue{out};
  }

  std::string parse_string() {
    expect('"');
    std::string out;
    while (true) {
      if (eof()) {
        throw std::runtime_error("Unexpected end of JSON string");
      }
      const char ch = get();
      if (ch == '"') {
        break;
      }
      if (ch == '\\') {
        if (eof()) {
          throw std::runtime_error("Unexpected end of JSON escape sequence");
        }
        const char esc = get();
        switch (esc) {
          case '"':
          case '\\':
          case '/':
            out.push_back(esc);
            break;
          case 'b':
            out.push_back('\b');
            break;
          case 'f':
            out.push_back('\f');
            break;
          case 'n':
            out.push_back('\n');
            break;
          case 'r':
            out.push_back('\r');
            break;
          case 't':
            out.push_back('\t');
            break;
          case 'u':
            throw std::runtime_error("\\u escapes are not supported in this parser");
          default:
            throw std::runtime_error("Invalid JSON escape sequence");
        }
        continue;
      }
      out.push_back(ch);
    }
    return out;
  }

  bool parse_bool() {
    if (text_.compare(pos_, 4, "true") == 0) {
      pos_ += 4;
      return true;
    }
    if (text_.compare(pos_, 5, "false") == 0) {
      pos_ += 5;
      return false;
    }
    throw std::runtime_error("Invalid boolean literal");
  }

  void parse_null() {
    if (text_.compare(pos_, 4, "null") != 0) {
      throw std::runtime_error("Invalid null literal");
    }
    pos_ += 4;
  }

  double parse_number() {
    const std::size_t start = pos_;
    if (peek() == '-') {
      ++pos_;
    }
    while (!eof() && std::isdigit(static_cast<unsigned char>(peek()))) {
      ++pos_;
    }
    if (!eof() && peek() == '.') {
      ++pos_;
      while (!eof() && std::isdigit(static_cast<unsigned char>(peek()))) {
        ++pos_;
      }
    }
    if (!eof() && (peek() == 'e' || peek() == 'E')) {
      ++pos_;
      if (!eof() && (peek() == '+' || peek() == '-')) {
        ++pos_;
      }
      while (!eof() && std::isdigit(static_cast<unsigned char>(peek()))) {
        ++pos_;
      }
    }

    const std::string token = text_.substr(start, pos_ - start);
    try {
      return std::stod(token);
    } catch (const std::exception&) {
      throw std::runtime_error("Invalid number token: " + token);
    }
  }

  std::string text_;
  std::size_t pos_ = 0;
};

uint32_t as_u32(const JsonValue& value, const std::string& context) {
  if (!value.is_number()) {
    throw std::runtime_error(context + " must be a number");
  }
  const double d = value.as_number();
  if (d < 0.0 || d > static_cast<double>(UINT32_MAX)) {
    throw std::runtime_error(context + " out of uint32 range");
  }
  return static_cast<uint32_t>(d);
}

std::string as_string(const JsonValue& value, const std::string& context) {
  if (!value.is_string()) {
    throw std::runtime_error(context + " must be a string");
  }
  return value.as_string();
}

const JsonValue::object_t& as_object(const JsonValue& value, const std::string& context) {
  if (!value.is_object()) {
    throw std::runtime_error(context + " must be an object");
  }
  return value.as_object();
}

const JsonValue::array_t& as_array(const JsonValue& value, const std::string& context) {
  if (!value.is_array()) {
    throw std::runtime_error(context + " must be an array");
  }
  return value.as_array();
}

uint32_t resolve_duration(const JsonValue& value,
                         const std::unordered_map<std::string, uint32_t>& base_times,
                         const std::string& context) {
  if (value.is_number()) {
    return as_u32(value, context);
  }
  if (value.is_string()) {
    const auto key = value.as_string();
    const auto it = base_times.find(key);
    if (it == base_times.end()) {
      throw std::runtime_error(context + ": unknown base time key '" + key + "'");
    }
    return it->second;
  }
  throw std::runtime_error(context + " must be number or base-time key string");
}

Pattern parse_pattern_json(const JsonValue& value,
                           const std::unordered_map<std::string, uint32_t>& base_times,
                           const std::string& context) {
  Pattern out;
  const auto& array = as_array(value, context);
  for (std::size_t i = 0; i < array.size(); ++i) {
    const auto& phase_value = array[i];
    std::string state;
    uint32_t ms = 0;
    if (phase_value.is_array()) {
      const auto& pair = as_array(phase_value, context + "[" + std::to_string(i) + "]");
      if (pair.size() != 2) {
        throw std::runtime_error(context + "[" + std::to_string(i) + "] must have [state, duration]");
      }
      state = as_string(pair[0], context + "[" + std::to_string(i) + "].state");
      ms = resolve_duration(pair[1], base_times,
                            context + "[" + std::to_string(i) + "].duration");
    } else {
      const auto& obj = as_object(phase_value, context + "[" + std::to_string(i) + "]");
      const auto state_it = obj.find("state");
      const auto ms_it = obj.find("ms");
      if (state_it == obj.end() || ms_it == obj.end()) {
        throw std::runtime_error(context + "[" + std::to_string(i) + "] requires keys state/ms");
      }
      state = as_string(state_it->second, context + "[" + std::to_string(i) + "].state");
      ms = resolve_duration(ms_it->second, base_times,
                            context + "[" + std::to_string(i) + "].ms");
    }

    const std::string normalized_state = normalize_symbol(state);
    if (normalized_state == "ON") {
      out.push_back(Phase{PhaseState::On, ms});
    } else if (normalized_state == "OFF") {
      out.push_back(Phase{PhaseState::Off, ms});
    } else {
      throw std::runtime_error(context + "[" + std::to_string(i) + "] has invalid state '" + state + "'");
    }
  }
  return out;
}

SignalDefinition parse_signal_definition(const JsonValue& value,
                                         const std::unordered_map<std::string, uint32_t>& base_times,
                                         const std::string& context,
                                         bool alarm_default_repeat_forever) {
  const auto& obj = as_object(value, context);
  const auto pattern_it = obj.find("pattern");
  if (pattern_it == obj.end()) {
    throw std::runtime_error(context + ": missing required key pattern");
  }

  SignalDefinition def{};
  def.pattern = parse_pattern_json(pattern_it->second, base_times, context + ".pattern");

  if (!is_pattern_valid(def.pattern)) {
    throw std::runtime_error(context + ": invalid pattern");
  }

  def.options.repeat = alarm_default_repeat_forever ? kRepeatForever : 1U;
  def.options.gap_ms = 0;
  def.options.priority = alarm_default_repeat_forever ? Priority::Alarm : Priority::Info;
  def.options.policy = BusyPolicy::Queue;

  const auto repeat_it = obj.find("repeat");
  if (repeat_it != obj.end()) {
    if (repeat_it->second.is_string() && normalize_symbol(repeat_it->second.as_string()) == "FOREVER") {
      def.options.repeat = kRepeatForever;
    } else {
      def.options.repeat = as_u32(repeat_it->second, context + ".repeat");
    }
  }

  const auto gap_it = obj.find("gap_ms");
  if (gap_it != obj.end()) {
    def.options.gap_ms = resolve_duration(gap_it->second, base_times, context + ".gap_ms");
  }

  const auto priority_it = obj.find("priority");
  if (priority_it != obj.end()) {
    const auto p = parse_priority(as_string(priority_it->second, context + ".priority"));
    if (!p.has_value()) {
      throw std::runtime_error(context + ": invalid priority");
    }
    def.options.priority = *p;
  }

  const auto policy_it = obj.find("policy");
  if (policy_it != obj.end()) {
    const auto p = parse_busy_policy(as_string(policy_it->second, context + ".policy"));
    if (!p.has_value()) {
      throw std::runtime_error(context + ": invalid policy");
    }
    def.options.policy = *p;
  }

  const auto throttle_it = obj.find("throttle_ms");
  if (throttle_it != obj.end()) {
    def.options.throttle_ms = resolve_duration(throttle_it->second, base_times, context + ".throttle_ms");
  }

  const auto token_it = obj.find("token");
  if (token_it != obj.end()) {
    def.options.token = as_string(token_it->second, context + ".token");
  }

  return def;
}

Pattern make_sos_pattern(uint32_t short_ms, uint32_t long_ms, uint32_t gap_ms) {
  Pattern pattern;
  const auto push_short = [&]() {
    pattern.push_back(Phase{PhaseState::On, short_ms});
    pattern.push_back(Phase{PhaseState::Off, gap_ms});
  };
  const auto push_long = [&]() {
    pattern.push_back(Phase{PhaseState::On, long_ms});
    pattern.push_back(Phase{PhaseState::Off, gap_ms});
  };

  push_short();
  push_short();
  push_short();
  pattern.back().duration_ms = gap_ms * 3U;

  push_long();
  push_long();
  push_long();
  pattern.back().duration_ms = gap_ms * 3U;

  push_short();
  push_short();
  push_short();
  pattern.back().duration_ms = gap_ms;
  return pattern;
}

std::string json_escape(const std::string& input) {
  std::string out;
  out.reserve(input.size() + 8);
  for (const char ch : input) {
    switch (ch) {
      case '\\':
        out += "\\\\";
        break;
      case '"':
        out += "\\\"";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\r':
        out += "\\r";
        break;
      case '\t':
        out += "\\t";
        break;
      default:
        out.push_back(ch);
        break;
    }
  }
  return out;
}

}  // namespace

const char* event_name(EventId id) noexcept {
  switch (id) {
    case EventId::Arming:
      return "ARMING";
    case EventId::ArmingFailure:
      return "ARMING_FAILURE";
    case EventId::Disarmed:
      return "DISARMED";
    case EventId::GyroInitDone:
      return "GYRO_INIT_DONE";
    case EventId::ReadyToArm:
      return "READY_TO_ARM";
  }
  return "UNKNOWN_EVENT";
}

const char* alarm_name(AlarmId id) noexcept {
  switch (id) {
    case AlarmId::BatteryFailsafe:
      return "BATTERY_FAILSAFE";
    case AlarmId::EkfFailure:
      return "EKF_FAILURE";
    case AlarmId::LostVehicle:
      return "LOST_VEHICLE";
    case AlarmId::EvacuationTemporal3:
      return "EVACUATION_TEMPORAL3";
    case AlarmId::MissingSos:
      return "MISSING_SOS";
  }
  return "UNKNOWN_ALARM";
}

const char* priority_name(Priority p) noexcept {
  switch (p) {
    case Priority::Info:
      return "INFO";
    case Priority::Alarm:
      return "ALARM";
    case Priority::Critical:
      return "CRITICAL";
  }
  return "UNKNOWN";
}

const char* busy_policy_name(BusyPolicy p) noexcept {
  switch (p) {
    case BusyPolicy::Queue:
      return "QUEUE";
    case BusyPolicy::Replace:
      return "REPLACE";
    case BusyPolicy::DropIfBusy:
      return "DROP_IF_BUSY";
  }
  return "UNKNOWN";
}

std::optional<EventId> parse_event_id(const std::string& value) {
  const std::string normalized = normalize_symbol(value);
  if (normalized == "ARMING") {
    return EventId::Arming;
  }
  if (normalized == "ARMINGFAILURE") {
    return EventId::ArmingFailure;
  }
  if (normalized == "DISARMED") {
    return EventId::Disarmed;
  }
  if (normalized == "GYROINITDONE") {
    return EventId::GyroInitDone;
  }
  if (normalized == "READYTOARM") {
    return EventId::ReadyToArm;
  }
  return std::nullopt;
}

std::optional<AlarmId> parse_alarm_id(const std::string& value) {
  const std::string normalized = normalize_symbol(value);
  if (normalized == "BATTERYFAILSAFE") {
    return AlarmId::BatteryFailsafe;
  }
  if (normalized == "EKFFAILURE") {
    return AlarmId::EkfFailure;
  }
  if (normalized == "LOSTVEHICLE") {
    return AlarmId::LostVehicle;
  }
  if (normalized == "EVACUATIONTEMPORAL3") {
    return AlarmId::EvacuationTemporal3;
  }
  if (normalized == "MISSINGSOS") {
    return AlarmId::MissingSos;
  }
  return std::nullopt;
}

std::optional<Priority> parse_priority(const std::string& value) {
  const std::string normalized = normalize_symbol(value);
  if (normalized == "INFO") {
    return Priority::Info;
  }
  if (normalized == "ALARM") {
    return Priority::Alarm;
  }
  if (normalized == "CRITICAL") {
    return Priority::Critical;
  }
  return std::nullopt;
}

std::optional<BusyPolicy> parse_busy_policy(const std::string& value) {
  const std::string normalized = normalize_symbol(value);
  if (normalized == "QUEUE") {
    return BusyPolicy::Queue;
  }
  if (normalized == "REPLACE") {
    return BusyPolicy::Replace;
  }
  if (normalized == "DROPIFBUSY") {
    return BusyPolicy::DropIfBusy;
  }
  return std::nullopt;
}

bool is_pattern_valid(const Pattern& pattern) noexcept {
  if (pattern.empty()) {
    return false;
  }
  return std::all_of(pattern.begin(), pattern.end(), [](const Phase& phase) {
    return phase.duration_ms > 0;
  });
}

SignalProfile make_default_profile() {
  SignalProfile profile;
  profile.name = "ardupilot_like_v1";
  profile.base_times_ms = {
      {"SHORT_MS", 120},
      {"MEDIUM_MS", 300},
      {"LONG_MS", 700},
      {"VERY_LONG_MS", 3000},
      {"GAP_MS", 120},
      {"PAUSE_3S_MS", 3000},
  };

  auto make_def = [](Pattern pattern, Priority priority, uint32_t repeat, uint32_t gap_ms,
                     uint32_t throttle_ms, BusyPolicy policy, std::string token) {
    SignalDefinition def;
    def.pattern = std::move(pattern);
    def.options.priority = priority;
    def.options.repeat = repeat;
    def.options.gap_ms = gap_ms;
    def.options.throttle_ms = throttle_ms;
    def.options.policy = policy;
    def.options.token = std::move(token);
    return def;
  };

  profile.events[EventId::Arming] = make_def({Phase{PhaseState::On, 3000}}, Priority::Info, 1, 0,
                                             250, BusyPolicy::Queue, "event.arming");
  profile.events[EventId::ArmingFailure] = make_def({Phase{PhaseState::On, 700}}, Priority::Alarm,
                                                    1, 0, 250, BusyPolicy::Replace,
                                                    "event.arming_failure");
  profile.events[EventId::Disarmed] = make_def({Phase{PhaseState::On, 180}}, Priority::Info, 1, 0,
                                               250, BusyPolicy::Queue, "event.disarmed");
  profile.events[EventId::GyroInitDone] = make_def(
      {Phase{PhaseState::On, 80}, Phase{PhaseState::Off, 80}, Phase{PhaseState::On, 80},
       Phase{PhaseState::Off, 80}, Phase{PhaseState::On, 80}, Phase{PhaseState::Off, 80},
       Phase{PhaseState::On, 80}, Phase{PhaseState::Off, 80}, Phase{PhaseState::On, 80},
       Phase{PhaseState::Off, 80}, Phase{PhaseState::On, 80}, Phase{PhaseState::Off, 80},
       Phase{PhaseState::On, 80}},
      Priority::Info, 1, 0, 100, BusyPolicy::DropIfBusy, "event.gyro_init_done");
  profile.events[EventId::ReadyToArm] = make_def(
      {Phase{PhaseState::On, 120}, Phase{PhaseState::Off, 120}, Phase{PhaseState::On, 120},
       Phase{PhaseState::Off, 120}, Phase{PhaseState::On, 120}, Phase{PhaseState::Off, 120},
       Phase{PhaseState::On, 900}},
      Priority::Info, 1, 0, 100, BusyPolicy::Queue, "event.ready_to_arm");

  profile.alarms[AlarmId::BatteryFailsafe] = make_def(
      {Phase{PhaseState::On, 120}, Phase{PhaseState::Off, 2880}}, Priority::Alarm,
      kRepeatForever, 0, 0, BusyPolicy::Replace, "alarm.battery_failsafe");
  profile.alarms[AlarmId::EkfFailure] = make_def(
      {Phase{PhaseState::On, 450}, Phase{PhaseState::Off, 150}, Phase{PhaseState::On, 300},
       Phase{PhaseState::Off, 120}, Phase{PhaseState::On, 150}, Phase{PhaseState::Off, 120},
       Phase{PhaseState::On, 150}, Phase{PhaseState::Off, 1710}},
      Priority::Alarm, kRepeatForever, 0, 0, BusyPolicy::Replace, "alarm.ekf_failure");
  profile.alarms[AlarmId::LostVehicle] = make_def(
      {Phase{PhaseState::On, 150}, Phase{PhaseState::Off, 150}, Phase{PhaseState::On, 150},
       Phase{PhaseState::Off, 2700}},
      Priority::Critical, kRepeatForever, 0, 0, BusyPolicy::Replace, "alarm.lost_vehicle");
  profile.alarms[AlarmId::EvacuationTemporal3] = make_def(
      {Phase{PhaseState::On, 500}, Phase{PhaseState::Off, 500}, Phase{PhaseState::On, 500},
       Phase{PhaseState::Off, 500}, Phase{PhaseState::On, 500}, Phase{PhaseState::Off, 1500}},
      Priority::Critical, kRepeatForever, 0, 0, BusyPolicy::Replace,
      "alarm.evacuation_temporal3");
  profile.alarms[AlarmId::MissingSos] = make_def(make_sos_pattern(120, 700, 120),
                                                 Priority::Critical, kRepeatForever, 3000,
                                                 0, BusyPolicy::Replace, "alarm.missing_sos");

  return profile;
}

SignalProfile load_profile_json_text(const std::string& text) {
  const JsonValue root = JsonParser(text).parse();
  const auto& obj = as_object(root, "root");

  SignalProfile profile = make_default_profile();

  const auto name_it = obj.find("name");
  if (name_it != obj.end()) {
    profile.name = as_string(name_it->second, "name");
  }

  if (const auto bt_it = obj.find("base_times_ms"); bt_it != obj.end()) {
    const auto& bt_obj = as_object(bt_it->second, "base_times_ms");
    profile.base_times_ms.clear();
    for (const auto& [key, val] : bt_obj) {
      profile.base_times_ms[key] = as_u32(val, "base_times_ms." + key);
    }
  }

  if (const auto events_it = obj.find("events"); events_it != obj.end()) {
    profile.events.clear();
    const auto& events_obj = as_object(events_it->second, "events");
    for (const auto& [event_name_key, event_def_json] : events_obj) {
      const auto event = parse_event_id(event_name_key);
      if (!event.has_value()) {
        throw std::runtime_error("Unknown event key in profile: " + event_name_key);
      }
      SignalDefinition def = parse_signal_definition(
          event_def_json, profile.base_times_ms, "events." + event_name_key, false);
      if (def.options.token.empty()) {
        def.options.token = "event." + normalize_symbol(event_name_key);
      }
      profile.events[*event] = std::move(def);
    }
  }

  if (const auto alarms_it = obj.find("alarms"); alarms_it != obj.end()) {
    profile.alarms.clear();
    const auto& alarms_obj = as_object(alarms_it->second, "alarms");
    for (const auto& [alarm_name_key, alarm_def_json] : alarms_obj) {
      const auto alarm = parse_alarm_id(alarm_name_key);
      if (!alarm.has_value()) {
        throw std::runtime_error("Unknown alarm key in profile: " + alarm_name_key);
      }
      SignalDefinition def = parse_signal_definition(
          alarm_def_json, profile.base_times_ms, "alarms." + alarm_name_key, true);
      if (def.options.token.empty()) {
        def.options.token = "alarm." + normalize_symbol(alarm_name_key);
      }
      profile.alarms[*alarm] = std::move(def);
    }
  }

  return profile;
}

SignalProfile load_profile_json_file(const std::string& path) {
  std::ifstream in(path);
  if (!in.is_open()) {
    throw std::runtime_error("Failed to open profile file: " + path);
  }
  std::ostringstream oss;
  oss << in.rdbuf();
  return load_profile_json_text(oss.str());
}

std::string profile_to_json(const SignalProfile& profile) {
  std::ostringstream out;
  out << "{\n";
  out << "  \"name\": \"" << json_escape(profile.name) << "\",\n";

  out << "  \"base_times_ms\": {\n";
  bool first_bt = true;
  for (const auto& [key, value] : profile.base_times_ms) {
    if (!first_bt) {
      out << ",\n";
    }
    first_bt = false;
    out << "    \"" << json_escape(key) << "\": " << value;
  }
  out << "\n  },\n";

  auto dump_signal_map = [&out](auto begin, auto end, const auto& key_name_fn) {
    bool first_signal = true;
    for (auto it = begin; it != end; ++it) {
      if (!first_signal) {
        out << ",\n";
      }
      first_signal = false;
      out << "    \"" << key_name_fn(it->first) << "\": {\n";
      out << "      \"pattern\": [";
      for (std::size_t i = 0; i < it->second.pattern.size(); ++i) {
        const Phase& phase = it->second.pattern[i];
        if (i > 0) {
          out << ", ";
        }
        out << "[\"" << (phase.state == PhaseState::On ? "ON" : "OFF") << "\", "
            << phase.duration_ms << "]";
      }
      out << "],\n";
      out << "      \"repeat\": "
          << (it->second.options.repeat == kRepeatForever
                  ? std::string("\"forever\"")
                  : std::to_string(it->second.options.repeat))
          << ",\n";
      out << "      \"gap_ms\": " << it->second.options.gap_ms << ",\n";
      out << "      \"priority\": \"" << priority_name(it->second.options.priority) << "\",\n";
      out << "      \"policy\": \"" << busy_policy_name(it->second.options.policy) << "\",\n";
      out << "      \"throttle_ms\": " << it->second.options.throttle_ms << ",\n";
      out << "      \"token\": \"" << json_escape(it->second.options.token) << "\"\n";
      out << "    }";
    }
    out << "\n";
  };

  out << "  \"events\": {\n";
  dump_signal_map(profile.events.begin(), profile.events.end(),
                  [](EventId id) { return std::string(event_name(id)); });
  out << "  },\n";

  out << "  \"alarms\": {\n";
  dump_signal_map(profile.alarms.begin(), profile.alarms.end(),
                  [](AlarmId id) { return std::string(alarm_name(id)); });
  out << "  }\n";

  out << "}\n";
  return out.str();
}

}  // namespace buzzer
