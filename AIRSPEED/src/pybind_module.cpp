#include "ms4525do/ms4525do.hpp"

#include <pybind11/pybind11.h>

namespace py = pybind11;

PYBIND11_MODULE(_ms4525do, m) {
  py::register_exception<ms4525do::I2cError>(m, "I2cError");

  py::enum_<ms4525do::Ms4525do::ReadPolicy>(m, "ReadPolicy")
      .value("REQUIRE_FRESH", ms4525do::Ms4525do::ReadPolicy::kRequireFresh)
      .value("ALLOW_STALE", ms4525do::Ms4525do::ReadPolicy::kAllowStale)
      .export_values();

  py::enum_<ms4525do::Ms4525do::OutputType>(m, "OutputType")
      .value("TYPE_A_10_TO_90", ms4525do::Ms4525do::OutputType::kTypeA_10To90)
      .value("TYPE_B_5_TO_95", ms4525do::Ms4525do::OutputType::kTypeB_5To95)
      .export_values();

  py::enum_<ms4525do::Ms4525do::Status>(m, "Status")
      .value("NORMAL", ms4525do::Ms4525do::Status::kNormal)
      .value("COMMAND_MODE", ms4525do::Ms4525do::Status::kCommandMode)
      .value("STALE_DATA", ms4525do::Ms4525do::Status::kStaleData)
      .value("DIAGNOSTIC_FAULT", ms4525do::Ms4525do::Status::kDiagnosticFault)
      .export_values();

  py::class_<ms4525do::Ms4525do::Calibration>(m, "Calibration")
      .def(py::init<>())
      .def_readwrite("p_min_psi", &ms4525do::Ms4525do::Calibration::p_min_psi)
      .def_readwrite("p_max_psi", &ms4525do::Ms4525do::Calibration::p_max_psi)
      .def_readwrite("output_type", &ms4525do::Ms4525do::Calibration::output_type);

  py::class_<ms4525do::Ms4525do::Reading>(m, "Reading")
      .def_readonly("pressure_psi", &ms4525do::Ms4525do::Reading::pressure_psi)
      .def_readonly("pressure_pa", &ms4525do::Ms4525do::Reading::pressure_pa)
      .def_readonly("temperature_c", &ms4525do::Ms4525do::Reading::temperature_c)
      .def_readonly("pressure_counts", &ms4525do::Ms4525do::Reading::pressure_counts)
      .def_readonly("temperature_counts", &ms4525do::Ms4525do::Reading::temperature_counts)
      .def_readonly("status", &ms4525do::Ms4525do::Reading::status);

  py::class_<ms4525do::Ms4525do>(m, "Ms4525do")
      .def(py::init<std::string, uint8_t, unsigned int, ms4525do::Ms4525do::Calibration, unsigned int,
                    unsigned int>(),
           py::arg("bus_path") = "/dev/i2c-1", py::arg("address") = 0x28, py::arg("retries") = 2,
           py::arg("calibration") = ms4525do::Ms4525do::Calibration(),
           py::arg("max_stale_retries") = 3, py::arg("stale_retry_delay_ms") = 2)
      .def("begin", &ms4525do::Ms4525do::begin, py::call_guard<py::gil_scoped_release>())
      .def("close", &ms4525do::Ms4525do::close, py::call_guard<py::gil_scoped_release>())
      .def("set_calibration", &ms4525do::Ms4525do::set_calibration, py::arg("calibration"),
           py::call_guard<py::gil_scoped_release>())
      .def("calibration", &ms4525do::Ms4525do::calibration, py::call_guard<py::gil_scoped_release>())
      .def("read", &ms4525do::Ms4525do::read,
           py::arg("policy") = ms4525do::Ms4525do::ReadPolicy::kRequireFresh,
           py::call_guard<py::gil_scoped_release>())
      .def("read_pressure_psi", &ms4525do::Ms4525do::read_pressure_psi,
           py::call_guard<py::gil_scoped_release>())
      .def("read_pressure_pa", &ms4525do::Ms4525do::read_pressure_pa,
           py::call_guard<py::gil_scoped_release>())
      .def("read_temperature_c", &ms4525do::Ms4525do::read_temperature_c,
           py::call_guard<py::gil_scoped_release>());
}
