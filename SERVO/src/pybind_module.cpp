#include "servo/hardware_pwm.hpp"

#include <pybind11/pybind11.h>

namespace py = pybind11;

PYBIND11_MODULE(_servo, m) {
  py::register_exception<servo::PwmError>(m, "PwmError");

  py::class_<servo::HardwarePwmConfig>(m, "HardwarePwmConfig")
      .def(py::init<>())
      .def_readwrite("chip", &servo::HardwarePwmConfig::chip)
      .def_readwrite("channel", &servo::HardwarePwmConfig::channel)
      .def_readwrite("period_ns", &servo::HardwarePwmConfig::period_ns)
      .def_readwrite("duty_cycle_ns", &servo::HardwarePwmConfig::duty_cycle_ns)
      .def_readwrite("invert_polarity", &servo::HardwarePwmConfig::invert_polarity)
      .def_readwrite("enabled_on_begin", &servo::HardwarePwmConfig::enabled_on_begin)
      .def_readwrite("unexport_on_close", &servo::HardwarePwmConfig::unexport_on_close)
      .def_readwrite("use_channel_lock", &servo::HardwarePwmConfig::use_channel_lock)
      .def_readwrite("retries", &servo::HardwarePwmConfig::retries);

  py::class_<servo::HardwarePwm>(m, "HardwarePwm")
      .def(py::init<servo::HardwarePwmConfig>(), py::arg("config") = servo::HardwarePwmConfig{})
      .def("begin", &servo::HardwarePwm::begin, py::call_guard<py::gil_scoped_release>())
      .def("close", &servo::HardwarePwm::close, py::call_guard<py::gil_scoped_release>())
      .def("is_open", &servo::HardwarePwm::is_open)
      .def("set_enabled", &servo::HardwarePwm::set_enabled,
           py::call_guard<py::gil_scoped_release>())
      .def("set_period_ns", &servo::HardwarePwm::set_period_ns,
           py::call_guard<py::gil_scoped_release>())
      .def("set_duty_cycle_ns", &servo::HardwarePwm::set_duty_cycle_ns,
           py::call_guard<py::gil_scoped_release>())
      .def("period_ns", &servo::HardwarePwm::period_ns, py::call_guard<py::gil_scoped_release>())
      .def("duty_cycle_ns", &servo::HardwarePwm::duty_cycle_ns,
           py::call_guard<py::gil_scoped_release>())
      .def("enabled", &servo::HardwarePwm::enabled, py::call_guard<py::gil_scoped_release>())
      .def("chip", &servo::HardwarePwm::chip)
      .def("channel", &servo::HardwarePwm::channel)
      .def("channel_path", &servo::HardwarePwm::channel_path);

  py::class_<servo::ServoConfig>(m, "ServoConfig")
      .def(py::init<>())
      .def_readwrite("min_angle_deg", &servo::ServoConfig::min_angle_deg)
      .def_readwrite("max_angle_deg", &servo::ServoConfig::max_angle_deg)
      .def_readwrite("frequency_hz", &servo::ServoConfig::frequency_hz)
      .def_readwrite("min_pulse_width_us", &servo::ServoConfig::min_pulse_width_us)
      .def_readwrite("max_pulse_width_us", &servo::ServoConfig::max_pulse_width_us)
      .def_readwrite("neutral_pulse_width_us", &servo::ServoConfig::neutral_pulse_width_us)
      .def_readwrite("invert_polarity", &servo::ServoConfig::invert_polarity)
      .def_readwrite("enabled_on_begin", &servo::ServoConfig::enabled_on_begin)
      .def_readwrite("unexport_on_close", &servo::ServoConfig::unexport_on_close)
      .def_readwrite("use_channel_lock", &servo::ServoConfig::use_channel_lock)
      .def_readwrite("retries", &servo::ServoConfig::retries);

  py::class_<servo::Servo>(m, "Servo")
      .def(py::init<uint32_t, uint32_t, servo::ServoConfig>(), py::arg("chip"), py::arg("channel"),
           py::arg("config") = servo::ServoConfig{})
      .def("begin", &servo::Servo::begin, py::call_guard<py::gil_scoped_release>())
      .def("close", &servo::Servo::close, py::call_guard<py::gil_scoped_release>())
      .def("is_open", &servo::Servo::is_open)
      .def("set_angle_deg", &servo::Servo::set_angle_deg, py::call_guard<py::gil_scoped_release>())
      .def("set_pulse_width_us", &servo::Servo::set_pulse_width_us,
           py::call_guard<py::gil_scoped_release>())
      .def("set_enabled", &servo::Servo::set_enabled, py::call_guard<py::gil_scoped_release>())
      .def("pulse_width_us", &servo::Servo::pulse_width_us)
      .def("angle_deg", &servo::Servo::angle_deg)
      .def("chip", &servo::Servo::chip)
      .def("channel", &servo::Servo::channel);
}
