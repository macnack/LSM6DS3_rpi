#include "bmp390/bmp390.hpp"

#include <pybind11/pybind11.h>

namespace py = pybind11;

PYBIND11_MODULE(_bmp390, m) {
  py::register_exception<bmp390::I2cError>(m, "I2cError");

  py::class_<bmp390::Bmp390::Reading>(m, "Reading")
      .def_readonly("temperature_c", &bmp390::Bmp390::Reading::temperature_c)
      .def_readonly("pressure_pa", &bmp390::Bmp390::Reading::pressure_pa);

  py::class_<bmp390::Bmp390>(m, "Bmp390")
      .def(py::init<std::string, uint8_t, unsigned int>(), py::arg("bus_path") = "/dev/i2c-1",
           py::arg("address") = 0x77, py::arg("retries") = 2)
      .def("begin", &bmp390::Bmp390::begin, py::call_guard<py::gil_scoped_release>())
      .def("close", &bmp390::Bmp390::close, py::arg("power_down") = true,
           py::call_guard<py::gil_scoped_release>())
      .def("configure_default", &bmp390::Bmp390::configure_default,
           py::call_guard<py::gil_scoped_release>())
      .def("set_sampling", &bmp390::Bmp390::set_sampling, py::arg("pressure_oversampling"),
           py::arg("temperature_oversampling"), py::arg("odr"), py::arg("iir_filter"),
           py::call_guard<py::gil_scoped_release>())
      .def("read", &bmp390::Bmp390::read, py::call_guard<py::gil_scoped_release>())
      .def("read_temperature_c", &bmp390::Bmp390::read_temperature_c,
           py::call_guard<py::gil_scoped_release>())
      .def("read_pressure_pa", &bmp390::Bmp390::read_pressure_pa,
           py::call_guard<py::gil_scoped_release>())
      .def("read_pressure_hpa", &bmp390::Bmp390::read_pressure_hpa,
           py::call_guard<py::gil_scoped_release>());

  m.attr("BMP3_NO_OVERSAMPLING") = py::int_(0x00);
  m.attr("BMP3_OVERSAMPLING_2X") = py::int_(0x01);
  m.attr("BMP3_OVERSAMPLING_4X") = py::int_(0x02);
  m.attr("BMP3_OVERSAMPLING_8X") = py::int_(0x03);
  m.attr("BMP3_OVERSAMPLING_16X") = py::int_(0x04);
  m.attr("BMP3_OVERSAMPLING_32X") = py::int_(0x05);

  m.attr("BMP3_ODR_25_HZ") = py::int_(0x03);
  m.attr("BMP3_IIR_FILTER_COEFF_3") = py::int_(0x02);
}
