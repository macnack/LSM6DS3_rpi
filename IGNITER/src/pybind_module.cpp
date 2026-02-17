#include "igniter/igniter_bank.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

namespace {

class IgniterBankSim {
 public:
  explicit IgniterBankSim(igniter::IgniterBank::Config cfg) : bank_(std::move(cfg)) {}

  void init(uint64_t now_ms) { bank_.init(now_ms); }
  bool arm(uint64_t now_ms) { return bank_.arm(now_ms); }
  void disarm(uint64_t now_ms) { bank_.disarm(now_ms); }
  bool fire_mask(uint8_t mask, const std::array<uint32_t, igniter::kIgniterChannels>& durations,
                 uint64_t now_ms) {
    return bank_.fire_mask(mask, durations, now_ms);
  }
  bool fire_one(uint8_t channel, uint32_t duration_ms, uint64_t now_ms) {
    return bank_.fire_one(channel, duration_ms, now_ms);
  }
  bool fire_all(uint32_t duration_ms, uint64_t now_ms) { return bank_.fire_all(duration_ms, now_ms); }
  void clear_fault(uint64_t now_ms) { bank_.clear_fault(now_ms); }
  igniter::IgniterBank::Snapshot update(uint64_t now_ms, bool force_disarm) {
    return bank_.update(now_ms, force_disarm);
  }
  igniter::IgniterBank::Snapshot snapshot(uint64_t now_ms) const { return bank_.snapshot(now_ms); }
  bool set_status_values(const std::array<uint8_t, igniter::kIgniterChannels>& values) {
    return bank_.set_sim_status_values(values);
  }

 private:
  igniter::IgniterBank bank_;
};

}  // namespace

PYBIND11_MODULE(_igniter, m) {
  m.doc() = "Igniter safety/state-machine bindings";

  py::enum_<igniter::Fault>(m, "Fault")
      .value("NONE", igniter::Fault::None)
      .value("STATUS_LOW_WHILE_ON", igniter::Fault::StatusLowWhileOn)
      .value("STATUS_LOW_WHILE_OFF", igniter::Fault::StatusLowWhileOff)
      .value("LATCHED", igniter::Fault::Latched);

  py::enum_<igniter::State>(m, "State")
      .value("DISARMED", igniter::State::Disarmed)
      .value("ARMED_IDLE", igniter::State::ArmedIdle)
      .value("FIRING", igniter::State::Firing)
      .value("FAULT_LATCHED", igniter::State::FaultLatched);

  py::enum_<igniter::FaultPolicy>(m, "FaultPolicy")
      .value("GLOBAL", igniter::FaultPolicy::Global)
      .value("ISOLATED", igniter::FaultPolicy::Isolated);

  py::class_<igniter::VN5E160S::Config>(m, "Vn5Config")
      .def(py::init<>())
      .def_readwrite("settle_ms", &igniter::VN5E160S::Config::settleMs)
      .def_readwrite("latch_faults", &igniter::VN5E160S::Config::latchFaults);

  py::class_<igniter::Igniter::Config>(m, "IgniterConfig")
      .def(py::init<>())
      .def_readwrite("max_fire_ms", &igniter::Igniter::Config::maxFireMs)
      .def_readwrite("default_fire_ms", &igniter::Igniter::Config::defaultFireMs);

  py::class_<igniter::ChannelConfig>(m, "ChannelConfig")
      .def(py::init<>())
      .def_readwrite("enabled", &igniter::ChannelConfig::enabled)
      .def_readwrite("igniter", &igniter::ChannelConfig::igniter)
      .def_readwrite("driver", &igniter::ChannelConfig::driver);

  py::class_<igniter::IgniterBank::Config>(m, "IgniterBankConfig")
      .def(py::init<>())
      .def_readwrite("fault_policy", &igniter::IgniterBank::Config::faultPolicy)
      .def_readwrite("channels", &igniter::IgniterBank::Config::channels);

  py::class_<igniter::IgniterBank::Snapshot>(m, "IgniterSnapshot")
      .def_readonly("armed", &igniter::IgniterBank::Snapshot::armed)
      .def_readonly("global_fault_latched", &igniter::IgniterBank::Snapshot::globalFaultLatched)
      .def_readonly("active_mask", &igniter::IgniterBank::Snapshot::activeMask)
      .def_readonly("states", &igniter::IgniterBank::Snapshot::states)
      .def_readonly("faults", &igniter::IgniterBank::Snapshot::faults)
      .def_readonly("remaining_ms", &igniter::IgniterBank::Snapshot::remainingMs);

  py::class_<igniter::VN5E160S>(m, "VN5E160S")
      .def(py::init<igniter::VN5E160S::Config>(), py::arg("config") = igniter::VN5E160S::Config{})
      .def("init", &igniter::VN5E160S::init)
      .def("set", &igniter::VN5E160S::set)
      .def("update", &igniter::VN5E160S::update)
      .def("force_off", &igniter::VN5E160S::force_off)
      .def("clear_fault", &igniter::VN5E160S::clear_fault)
      .def("is_on", &igniter::VN5E160S::is_on)
      .def("fault", &igniter::VN5E160S::fault)
      .def("ok", &igniter::VN5E160S::ok);

  py::class_<igniter::Igniter>(m, "Igniter")
      .def(py::init<igniter::Igniter::Config, igniter::VN5E160S::Config>(),
           py::arg("config") = igniter::Igniter::Config{},
           py::arg("driver_config") = igniter::VN5E160S::Config{})
      .def("init", &igniter::Igniter::init)
      .def("arm", &igniter::Igniter::arm)
      .def("disarm", &igniter::Igniter::disarm)
      .def("fire", &igniter::Igniter::fire)
      .def("off", &igniter::Igniter::off)
      .def("update", &igniter::Igniter::update)
      .def("clear_fault", &igniter::Igniter::clear_fault)
      .def("state", &igniter::Igniter::state)
      .def("is_armed", &igniter::Igniter::is_armed)
      .def("is_firing", &igniter::Igniter::is_firing)
      .def("has_fault", &igniter::Igniter::has_fault)
      .def("fault", &igniter::Igniter::fault)
      .def("remaining_ms", &igniter::Igniter::remaining_ms);

  py::class_<IgniterBankSim>(m, "IgniterBank")
      .def(py::init<igniter::IgniterBank::Config>(), py::arg("config") = igniter::IgniterBank::Config{})
      .def("init", &IgniterBankSim::init)
      .def("arm", &IgniterBankSim::arm)
      .def("disarm", &IgniterBankSim::disarm)
      .def("fire_mask", &IgniterBankSim::fire_mask)
      .def("fire_one", &IgniterBankSim::fire_one)
      .def("fire_all", &IgniterBankSim::fire_all)
      .def("clear_fault", &IgniterBankSim::clear_fault)
      .def("update", &IgniterBankSim::update, py::arg("now_ms"), py::arg("force_disarm") = false)
      .def("snapshot", &IgniterBankSim::snapshot)
      .def("set_status_values", &IgniterBankSim::set_status_values);

  m.def("state_name", [](igniter::State s) { return igniter::state_name(s); });
  m.def("fault_name", [](igniter::Fault f) { return igniter::fault_name(f); });
}
