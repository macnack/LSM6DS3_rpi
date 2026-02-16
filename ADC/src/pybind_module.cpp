#include "ads1115/ads1115.hpp"

#include <pybind11/functional.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;

PYBIND11_MODULE(_ads1115, m) {
  py::register_exception<ads1115::I2cError>(m, "I2cError");

  py::enum_<ads1115::Ads1115::Mode>(m, "Mode")
      .value("CONTINUOUS", ads1115::Ads1115::Mode::kContinuous)
      .value("SINGLE_SHOT", ads1115::Ads1115::Mode::kSingleShot)
      .export_values();

  py::enum_<ads1115::Ads1115::DataRate>(m, "DataRate")
      .value("SPS_8", ads1115::Ads1115::DataRate::k8Sps)
      .value("SPS_16", ads1115::Ads1115::DataRate::k16Sps)
      .value("SPS_32", ads1115::Ads1115::DataRate::k32Sps)
      .value("SPS_64", ads1115::Ads1115::DataRate::k64Sps)
      .value("SPS_128", ads1115::Ads1115::DataRate::k128Sps)
      .value("SPS_250", ads1115::Ads1115::DataRate::k250Sps)
      .value("SPS_475", ads1115::Ads1115::DataRate::k475Sps)
      .value("SPS_860", ads1115::Ads1115::DataRate::k860Sps)
      .export_values();

  py::enum_<ads1115::Ads1115::Mux>(m, "Mux")
      .value("DIFF_AIN0_AIN1", ads1115::Ads1115::Mux::kDiffAin0Ain1)
      .value("DIFF_AIN0_AIN3", ads1115::Ads1115::Mux::kDiffAin0Ain3)
      .value("DIFF_AIN1_AIN3", ads1115::Ads1115::Mux::kDiffAin1Ain3)
      .value("DIFF_AIN2_AIN3", ads1115::Ads1115::Mux::kDiffAin2Ain3)
      .value("SINGLE_AIN0", ads1115::Ads1115::Mux::kSingleAin0)
      .value("SINGLE_AIN1", ads1115::Ads1115::Mux::kSingleAin1)
      .value("SINGLE_AIN2", ads1115::Ads1115::Mux::kSingleAin2)
      .value("SINGLE_AIN3", ads1115::Ads1115::Mux::kSingleAin3)
      .export_values();

  py::class_<ads1115::Ads1115::ComparatorConfig>(m, "ComparatorConfig")
      .def(py::init<>())
      .def_readwrite("low_threshold", &ads1115::Ads1115::ComparatorConfig::low_threshold)
      .def_readwrite("high_threshold", &ads1115::Ads1115::ComparatorConfig::high_threshold)
      .def_readwrite("window_mode", &ads1115::Ads1115::ComparatorConfig::window_mode)
      .def_readwrite("active_high", &ads1115::Ads1115::ComparatorConfig::active_high)
      .def_readwrite("latching", &ads1115::Ads1115::ComparatorConfig::latching)
      .def_readwrite("queue", &ads1115::Ads1115::ComparatorConfig::queue);

  py::class_<ads1115::Ads1115>(m, "Ads1115")
      .def(py::init<std::string, uint8_t, unsigned int>(), py::arg("bus_path") = "/dev/i2c-1",
           py::arg("address") = 0x48, py::arg("retries") = 2)
      .def("begin", &ads1115::Ads1115::begin, py::call_guard<py::gil_scoped_release>())
      .def("close", &ads1115::Ads1115::close, py::call_guard<py::gil_scoped_release>())
      .def("set_mode", &ads1115::Ads1115::set_mode, py::arg("mode"),
           py::call_guard<py::gil_scoped_release>())
      .def("set_data_rate", &ads1115::Ads1115::set_data_rate, py::arg("rate"),
           py::call_guard<py::gil_scoped_release>())
      .def("set_gain", &ads1115::Ads1115::set_gain, py::arg("gain_code"),
           py::call_guard<py::gil_scoped_release>())
      .def("set_mux", &ads1115::Ads1115::set_mux, py::arg("mux"),
           py::call_guard<py::gil_scoped_release>())
      .def("configure_comparator", &ads1115::Ads1115::configure_comparator, py::arg("config"),
           py::call_guard<py::gil_scoped_release>())
      .def("read_adc", &ads1115::Ads1115::read_adc, py::arg("channel"),
           py::call_guard<py::gil_scoped_release>())
      .def("read_voltage", &ads1115::Ads1115::read_voltage, py::arg("channel"),
           py::call_guard<py::gil_scoped_release>())
      .def("read_differential_adc", &ads1115::Ads1115::read_differential_adc, py::arg("channel"),
           py::call_guard<py::gil_scoped_release>())
      .def("comparator_voltage", &ads1115::Ads1115::comparator_voltage, py::arg("channel"),
           py::call_guard<py::gil_scoped_release>())
      .def("read_latest", &ads1115::Ads1115::read_latest, py::call_guard<py::gil_scoped_release>())
      .def("read_latest_voltage", &ads1115::Ads1115::read_latest_voltage,
           py::call_guard<py::gil_scoped_release>())
      .def("poll_continuous", &ads1115::Ads1115::poll_continuous, py::arg("on_sample"),
           py::arg("poll_interval_ms") = 1, py::arg("max_samples") = 0);

  // Legacy DFRobot-style gain constants.
  m.attr("ADS1115_REG_CONFIG_PGA_6_144V") = py::int_(0x00);
  m.attr("ADS1115_REG_CONFIG_PGA_4_096V") = py::int_(0x02);
  m.attr("ADS1115_REG_CONFIG_PGA_2_048V") = py::int_(0x04);
  m.attr("ADS1115_REG_CONFIG_PGA_1_024V") = py::int_(0x06);
  m.attr("ADS1115_REG_CONFIG_PGA_0_512V") = py::int_(0x08);
  m.attr("ADS1115_REG_CONFIG_PGA_0_256V") = py::int_(0x0A);

  // DFRobot-style DR constants (CONFIG.DR[7:5] bits).
  m.attr("ADS1115_REG_CONFIG_DR_8SPS") = py::int_(0x00);
  m.attr("ADS1115_REG_CONFIG_DR_16SPS") = py::int_(0x20);
  m.attr("ADS1115_REG_CONFIG_DR_32SPS") = py::int_(0x40);
  m.attr("ADS1115_REG_CONFIG_DR_64SPS") = py::int_(0x60);
  m.attr("ADS1115_REG_CONFIG_DR_128SPS") = py::int_(0x80);
  m.attr("ADS1115_REG_CONFIG_DR_250SPS") = py::int_(0xA0);
  m.attr("ADS1115_REG_CONFIG_DR_475SPS") = py::int_(0xC0);
  m.attr("ADS1115_REG_CONFIG_DR_860SPS") = py::int_(0xE0);
}
