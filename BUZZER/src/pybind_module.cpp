#include "buzzer/engine.hpp"

#include <cctype>
#include <cstdint>

#include <pybind11/chrono.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

namespace {

uint32_t resolve_duration(py::handle value, const std::unordered_map<std::string, uint32_t>& base_times,
                          const std::string& context) {
  if (py::isinstance<py::int_>(value)) {
    const long long v = py::cast<long long>(value);
    if (v < 0 || v > static_cast<long long>(UINT32_MAX)) {
      throw py::value_error(context + " out of uint32 range");
    }
    return static_cast<uint32_t>(v);
  }
  if (py::isinstance<py::str>(value)) {
    const std::string key = py::cast<std::string>(value);
    const auto it = base_times.find(key);
    if (it == base_times.end()) {
      throw py::key_error("Unknown base-time key: " + key);
    }
    return it->second;
  }
  throw py::type_error(context + " must be int or base-time key");
}

buzzer::Pattern parse_pattern(py::handle pattern_obj,
                              const std::unordered_map<std::string, uint32_t>& base_times,
                              const std::string& context) {
  if (!py::isinstance<py::sequence>(pattern_obj)) {
    throw py::type_error(context + " must be a sequence");
  }

  buzzer::Pattern pattern;
  const py::sequence seq = py::reinterpret_borrow<py::sequence>(pattern_obj);
  for (std::size_t i = 0; i < seq.size(); ++i) {
    const py::handle item = seq[i];
    std::string state;
    uint32_t ms = 0;

    if (py::isinstance<py::dict>(item)) {
      const py::dict obj = py::reinterpret_borrow<py::dict>(item);
      if (!obj.contains("state") || !obj.contains("ms")) {
        throw py::value_error(context + "[" + std::to_string(i) + "] requires state/ms");
      }
      state = py::cast<std::string>(obj["state"]);
      ms = resolve_duration(obj["ms"], base_times,
                            context + "[" + std::to_string(i) + "].ms");
    } else if (py::isinstance<py::sequence>(item)) {
      const py::sequence pair = py::reinterpret_borrow<py::sequence>(item);
      if (pair.size() != 2) {
        throw py::value_error(context + "[" + std::to_string(i) + "] must be [state, duration]");
      }
      state = py::cast<std::string>(pair[0]);
      ms = resolve_duration(pair[1], base_times,
                            context + "[" + std::to_string(i) + "].duration");
    } else {
      throw py::type_error(context + "[" + std::to_string(i) + "] must be dict or [state, duration]");
    }

    std::string normalized;
    normalized.reserve(state.size());
    for (char ch : state) {
      if (std::isalnum(static_cast<unsigned char>(ch))) {
        normalized.push_back(static_cast<char>(std::toupper(static_cast<unsigned char>(ch))));
      }
    }

    buzzer::PhaseState phase_state;
    if (normalized == "ON") {
      phase_state = buzzer::PhaseState::On;
    } else if (normalized == "OFF") {
      phase_state = buzzer::PhaseState::Off;
    } else {
      throw py::value_error(context + "[" + std::to_string(i) + "] invalid state");
    }

    pattern.push_back(buzzer::Phase{phase_state, ms});
  }

  if (!buzzer::is_pattern_valid(pattern)) {
    throw py::value_error(context + " is empty or has zero-length phases");
  }
  return pattern;
}

buzzer::PlayOptions parse_options(py::handle obj,
                                  const std::unordered_map<std::string, uint32_t>& base_times,
                                  const std::string& context,
                                  bool alarm_default_forever,
                                  buzzer::Priority default_priority,
                                  const std::string& default_token) {
  buzzer::PlayOptions options;
  options.repeat = alarm_default_forever ? buzzer::kRepeatForever : 1U;
  options.priority = default_priority;
  options.policy = buzzer::BusyPolicy::Queue;
  options.token = default_token;

  if (obj.is_none()) {
    return options;
  }

  if (!py::isinstance<py::dict>(obj)) {
    throw py::type_error(context + " must be dict");
  }

  const py::dict d = py::reinterpret_borrow<py::dict>(obj);
  if (d.contains("repeat")) {
    const py::handle repeat_v = d["repeat"];
    if (py::isinstance<py::str>(repeat_v)) {
      const std::string rep = py::cast<std::string>(repeat_v);
      if (rep == "forever" || rep == "FOREVER") {
        options.repeat = buzzer::kRepeatForever;
      } else {
        throw py::value_error(context + ".repeat invalid string");
      }
    } else {
      options.repeat = resolve_duration(repeat_v, base_times, context + ".repeat");
    }
  }
  if (d.contains("gap_ms")) {
    options.gap_ms = resolve_duration(d["gap_ms"], base_times, context + ".gap_ms");
  }
  if (d.contains("priority")) {
    const auto p = buzzer::parse_priority(py::cast<std::string>(d["priority"]));
    if (!p.has_value()) {
      throw py::value_error(context + ".priority invalid");
    }
    options.priority = *p;
  }
  if (d.contains("policy")) {
    const auto p = buzzer::parse_busy_policy(py::cast<std::string>(d["policy"]));
    if (!p.has_value()) {
      throw py::value_error(context + ".policy invalid");
    }
    options.policy = *p;
  }
  if (d.contains("throttle_ms")) {
    options.throttle_ms =
        resolve_duration(d["throttle_ms"], base_times, context + ".throttle_ms");
  }
  if (d.contains("token")) {
    options.token = py::cast<std::string>(d["token"]);
  }

  return options;
}

buzzer::SignalDefinition parse_signal_definition(py::handle obj,
                                                 const std::unordered_map<std::string, uint32_t>& base_times,
                                                 const std::string& context,
                                                 bool alarm_default_forever,
                                                 buzzer::Priority default_priority,
                                                 const std::string& default_token) {
  if (!py::isinstance<py::dict>(obj)) {
    throw py::type_error(context + " must be dict");
  }
  const py::dict d = py::reinterpret_borrow<py::dict>(obj);
  if (!d.contains("pattern")) {
    throw py::value_error(context + " missing pattern");
  }

  buzzer::SignalDefinition def;
  def.pattern = parse_pattern(d["pattern"], base_times, context + ".pattern");
  def.options = parse_options(d, base_times, context, alarm_default_forever, default_priority,
                              default_token);
  return def;
}

buzzer::SignalProfile profile_from_dict(const py::dict& src) {
  buzzer::SignalProfile profile = buzzer::make_default_profile();

  if (src.contains("name")) {
    profile.name = py::cast<std::string>(src["name"]);
  }

  if (src.contains("base_times_ms")) {
    profile.base_times_ms.clear();
    const py::dict bt = py::cast<py::dict>(src["base_times_ms"]);
    for (auto item : bt) {
      profile.base_times_ms[py::cast<std::string>(item.first)] = py::cast<uint32_t>(item.second);
    }
  }

  if (src.contains("events")) {
    profile.events.clear();
    const py::dict events = py::cast<py::dict>(src["events"]);
    for (auto item : events) {
      const std::string key = py::cast<std::string>(item.first);
      const auto eid = buzzer::parse_event_id(key);
      if (!eid.has_value()) {
        throw py::key_error("Unknown event key: " + key);
      }
      profile.events[*eid] = parse_signal_definition(
          item.second, profile.base_times_ms, "events." + key, false,
          buzzer::Priority::Info, "event." + key);
    }
  }

  if (src.contains("alarms")) {
    profile.alarms.clear();
    const py::dict alarms = py::cast<py::dict>(src["alarms"]);
    for (auto item : alarms) {
      const std::string key = py::cast<std::string>(item.first);
      const auto aid = buzzer::parse_alarm_id(key);
      if (!aid.has_value()) {
        throw py::key_error("Unknown alarm key: " + key);
      }
      profile.alarms[*aid] = parse_signal_definition(
          item.second, profile.base_times_ms, "alarms." + key, true,
          buzzer::Priority::Alarm, "alarm." + key);
    }
  }

  return profile;
}

}  // namespace

PYBIND11_MODULE(_buzzer, m) {
  py::register_exception<buzzer::GpioError>(m, "GpioError");

  py::enum_<buzzer::PhaseState>(m, "PhaseState")
      .value("OFF", buzzer::PhaseState::Off)
      .value("ON", buzzer::PhaseState::On);

  py::class_<buzzer::Phase>(m, "Phase")
      .def(py::init<>())
      .def(py::init<buzzer::PhaseState, uint32_t>(), py::arg("state"), py::arg("duration_ms"))
      .def_readwrite("state", &buzzer::Phase::state)
      .def_readwrite("duration_ms", &buzzer::Phase::duration_ms);

  py::enum_<buzzer::Priority>(m, "Priority")
      .value("INFO", buzzer::Priority::Info)
      .value("ALARM", buzzer::Priority::Alarm)
      .value("CRITICAL", buzzer::Priority::Critical);

  py::enum_<buzzer::BusyPolicy>(m, "BusyPolicy")
      .value("QUEUE", buzzer::BusyPolicy::Queue)
      .value("REPLACE", buzzer::BusyPolicy::Replace)
      .value("DROP_IF_BUSY", buzzer::BusyPolicy::DropIfBusy);

  py::enum_<buzzer::EventId>(m, "EventId")
      .value("ARMING", buzzer::EventId::Arming)
      .value("ARMING_FAILURE", buzzer::EventId::ArmingFailure)
      .value("DISARMED", buzzer::EventId::Disarmed)
      .value("GYRO_INIT_DONE", buzzer::EventId::GyroInitDone)
      .value("READY_TO_ARM", buzzer::EventId::ReadyToArm);

  py::enum_<buzzer::AlarmId>(m, "AlarmId")
      .value("BATTERY_FAILSAFE", buzzer::AlarmId::BatteryFailsafe)
      .value("EKF_FAILURE", buzzer::AlarmId::EkfFailure)
      .value("LOST_VEHICLE", buzzer::AlarmId::LostVehicle)
      .value("EVACUATION_TEMPORAL3", buzzer::AlarmId::EvacuationTemporal3)
      .value("MISSING_SOS", buzzer::AlarmId::MissingSos);

  py::class_<buzzer::PlayOptions>(m, "PlayOptions")
      .def(py::init<>())
      .def_readwrite("repeat", &buzzer::PlayOptions::repeat)
      .def_readwrite("gap_ms", &buzzer::PlayOptions::gap_ms)
      .def_readwrite("priority", &buzzer::PlayOptions::priority)
      .def_readwrite("policy", &buzzer::PlayOptions::policy)
      .def_readwrite("throttle_ms", &buzzer::PlayOptions::throttle_ms)
      .def_readwrite("token", &buzzer::PlayOptions::token);

  py::class_<buzzer::HardwareBuzzerConfig>(m, "HardwareBuzzerConfig")
      .def(py::init<>())
      .def_readwrite("chip_path", &buzzer::HardwareBuzzerConfig::chip_path)
      .def_readwrite("line", &buzzer::HardwareBuzzerConfig::line)
      .def_readwrite("active_high", &buzzer::HardwareBuzzerConfig::active_high)
      .def_readwrite("max_on_time_ms", &buzzer::HardwareBuzzerConfig::max_on_time_ms)
      .def_readwrite("force_off_on_begin", &buzzer::HardwareBuzzerConfig::force_off_on_begin)
      .def_readwrite("force_off_on_close", &buzzer::HardwareBuzzerConfig::force_off_on_close)
      .def_readwrite("simulate", &buzzer::HardwareBuzzerConfig::simulate);

  py::class_<buzzer::HardwareBuzzer>(m, "HardwareBuzzer")
      .def(py::init<buzzer::HardwareBuzzerConfig>(), py::arg("config") = buzzer::HardwareBuzzerConfig{})
      .def("begin", &buzzer::HardwareBuzzer::begin, py::call_guard<py::gil_scoped_release>())
      .def("close", &buzzer::HardwareBuzzer::close, py::call_guard<py::gil_scoped_release>())
      .def("on", &buzzer::HardwareBuzzer::on, py::call_guard<py::gil_scoped_release>())
      .def("off", &buzzer::HardwareBuzzer::off, py::call_guard<py::gil_scoped_release>())
      .def("is_on", &buzzer::HardwareBuzzer::is_on)
      .def("is_open", &buzzer::HardwareBuzzer::is_open)
      .def("raw_level", &buzzer::HardwareBuzzer::raw_level)
      .def("__enter__", [](buzzer::HardwareBuzzer& self) -> buzzer::HardwareBuzzer& {
        self.begin();
        return self;
      }, py::return_value_policy::reference_internal)
      .def("__exit__", [](buzzer::HardwareBuzzer& self, py::object, py::object, py::object) {
        self.close();
        return false;
      });

  py::class_<buzzer::BuzzerEngine>(m, "BuzzerEngine")
      .def(py::init<buzzer::HardwareBuzzerConfig>(), py::arg("hw_cfg") = buzzer::HardwareBuzzerConfig{})
      .def("begin", &buzzer::BuzzerEngine::begin, py::call_guard<py::gil_scoped_release>())
      .def("close", &buzzer::BuzzerEngine::close, py::call_guard<py::gil_scoped_release>())
      .def("play", &buzzer::BuzzerEngine::play, py::arg("pattern"),
           py::arg("options") = buzzer::PlayOptions{}, py::arg("name") = "custom",
           py::call_guard<py::gil_scoped_release>())
      .def("stop", &buzzer::BuzzerEngine::stop, py::call_guard<py::gil_scoped_release>())
      .def("is_playing", &buzzer::BuzzerEngine::is_playing)
      .def("current", &buzzer::BuzzerEngine::current)
      .def("notify", &buzzer::BuzzerEngine::notify, py::arg("event_id"),
           py::call_guard<py::gil_scoped_release>())
      .def("set_alarm", &buzzer::BuzzerEngine::set_alarm, py::arg("alarm_id"), py::arg("active") = true,
           py::call_guard<py::gil_scoped_release>())
      .def("mute", &buzzer::BuzzerEngine::mute, py::arg("enabled") = true,
           py::call_guard<py::gil_scoped_release>())
      .def("silence_for", [](buzzer::BuzzerEngine& self, double seconds) {
        if (seconds < 0.0) {
          throw py::value_error("seconds must be >= 0");
        }
        self.silence_for(std::chrono::milliseconds(static_cast<long long>(seconds * 1000.0)));
      }, py::arg("seconds"), py::call_guard<py::gil_scoped_release>())
      .def("set_queue_limit", &buzzer::BuzzerEngine::set_queue_limit, py::arg("limit"))
      .def("load_profile_json", &buzzer::BuzzerEngine::load_profile_json, py::arg("path"),
           py::call_guard<py::gil_scoped_release>())
      .def("set_profile_from_dict", [](buzzer::BuzzerEngine& self, const py::dict& d) {
        self.set_profile(profile_from_dict(d));
      })
      .def("profile_as_json", [](buzzer::BuzzerEngine& self) {
        return buzzer::profile_to_json(self.profile_snapshot());
      })
      .def("__enter__", [](buzzer::BuzzerEngine& self) -> buzzer::BuzzerEngine& {
        self.begin();
        return self;
      }, py::return_value_policy::reference_internal)
      .def("__exit__", [](buzzer::BuzzerEngine& self, py::object, py::object, py::object) {
        self.close();
        return false;
      });

  m.def("make_default_profile_json", []() {
    return buzzer::profile_to_json(buzzer::make_default_profile());
  });
  m.def("load_profile_json_text", [](const std::string& text) {
    return buzzer::profile_to_json(buzzer::load_profile_json_text(text));
  }, py::arg("text"));
  m.def("event_name", [](buzzer::EventId id) { return std::string(buzzer::event_name(id)); });
  m.def("alarm_name", [](buzzer::AlarmId id) { return std::string(buzzer::alarm_name(id)); });
}
