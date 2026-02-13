#include "lsm6ds3/lsm6ds3.hpp"
#include "lsm6ds3/lsm6ds3_spi.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

PYBIND11_MODULE(_lsm6ds3, m) {
  using lsm6ds3::Lsm6ds3;
  using lsm6ds3::Lsm6ds3Spi;

  py::register_exception<lsm6ds3::I2cError>(m, "I2cError");
  py::register_exception<lsm6ds3::SpiError>(m, "SpiError");

  py::enum_<Lsm6ds3::AccelOdr>(m, "AccelOdr")
      .value("POWER_DOWN", Lsm6ds3::AccelOdr::PowerDown)
      .value("HZ_12_5", Lsm6ds3::AccelOdr::Hz12_5)
      .value("HZ_26", Lsm6ds3::AccelOdr::Hz26)
      .value("HZ_52", Lsm6ds3::AccelOdr::Hz52)
      .value("HZ_104", Lsm6ds3::AccelOdr::Hz104)
      .value("HZ_208", Lsm6ds3::AccelOdr::Hz208)
      .value("HZ_416", Lsm6ds3::AccelOdr::Hz416)
      .value("HZ_833", Lsm6ds3::AccelOdr::Hz833);

  py::enum_<Lsm6ds3::GyroOdr>(m, "GyroOdr")
      .value("POWER_DOWN", Lsm6ds3::GyroOdr::PowerDown)
      .value("HZ_12_5", Lsm6ds3::GyroOdr::Hz12_5)
      .value("HZ_26", Lsm6ds3::GyroOdr::Hz26)
      .value("HZ_52", Lsm6ds3::GyroOdr::Hz52)
      .value("HZ_104", Lsm6ds3::GyroOdr::Hz104)
      .value("HZ_208", Lsm6ds3::GyroOdr::Hz208)
      .value("HZ_416", Lsm6ds3::GyroOdr::Hz416)
      .value("HZ_833", Lsm6ds3::GyroOdr::Hz833);

  py::enum_<Lsm6ds3::AccelScale>(m, "AccelScale")
      .value("G2", Lsm6ds3::AccelScale::G2)
      .value("G4", Lsm6ds3::AccelScale::G4)
      .value("G8", Lsm6ds3::AccelScale::G8)
      .value("G16", Lsm6ds3::AccelScale::G16);

  py::enum_<Lsm6ds3::GyroScale>(m, "GyroScale")
      .value("DPS_250", Lsm6ds3::GyroScale::Dps245)
      .value("DPS_500", Lsm6ds3::GyroScale::Dps500)
      .value("DPS_1000", Lsm6ds3::GyroScale::Dps1000)
      .value("DPS_2000", Lsm6ds3::GyroScale::Dps2000);

  py::class_<Lsm6ds3>(m, "Lsm6ds3")
      .def(py::init<std::string, uint8_t, unsigned int>(), py::arg("bus_path") = "/dev/i2c-1",
           py::arg("address") = 0x6A, py::arg("retries") = 2)
      .def("begin", &Lsm6ds3::begin, py::call_guard<py::gil_scoped_release>())
      .def("close", &Lsm6ds3::close, py::arg("power_down") = true,
           py::call_guard<py::gil_scoped_release>())
      .def("enable_accel", &Lsm6ds3::enable_accel, py::call_guard<py::gil_scoped_release>())
      .def("enable_gyro", &Lsm6ds3::enable_gyro, py::call_guard<py::gil_scoped_release>())
      .def("set_accel_odr", &Lsm6ds3::set_accel_odr, py::call_guard<py::gil_scoped_release>())
      .def("set_gyro_odr", &Lsm6ds3::set_gyro_odr, py::call_guard<py::gil_scoped_release>())
      .def("set_accel_scale", &Lsm6ds3::set_accel_scale,
           py::call_guard<py::gil_scoped_release>())
      .def("set_gyro_scale", &Lsm6ds3::set_gyro_scale, py::call_guard<py::gil_scoped_release>())
      .def("read_accel_raw", &Lsm6ds3::read_accel_raw, py::call_guard<py::gil_scoped_release>())
      .def("read_gyro_raw", &Lsm6ds3::read_gyro_raw, py::call_guard<py::gil_scoped_release>())
      .def("read_accel", &Lsm6ds3::read_accel_si, py::call_guard<py::gil_scoped_release>())
      .def("read_gyro", &Lsm6ds3::read_gyro_si, py::call_guard<py::gil_scoped_release>());

  py::class_<Lsm6ds3Spi>(m, "Lsm6ds3Spi")
      .def(py::init<std::string, uint32_t, uint8_t, unsigned int>(),
           py::arg("device_path") = "/dev/spidev0.0", py::arg("speed_hz") = 5000000,
           py::arg("mode") = 3, py::arg("retries") = 2)
      .def("begin", &Lsm6ds3Spi::begin, py::call_guard<py::gil_scoped_release>())
      .def("close", &Lsm6ds3Spi::close, py::arg("power_down") = true,
           py::call_guard<py::gil_scoped_release>())
      .def("enable_accel", &Lsm6ds3Spi::enable_accel, py::call_guard<py::gil_scoped_release>())
      .def("enable_gyro", &Lsm6ds3Spi::enable_gyro, py::call_guard<py::gil_scoped_release>())
      .def("set_accel_odr", &Lsm6ds3Spi::set_accel_odr, py::call_guard<py::gil_scoped_release>())
      .def("set_gyro_odr", &Lsm6ds3Spi::set_gyro_odr, py::call_guard<py::gil_scoped_release>())
      .def("set_accel_scale", &Lsm6ds3Spi::set_accel_scale,
           py::call_guard<py::gil_scoped_release>())
      .def("set_gyro_scale", &Lsm6ds3Spi::set_gyro_scale, py::call_guard<py::gil_scoped_release>())
      .def("read_accel_raw", &Lsm6ds3Spi::read_accel_raw,
           py::call_guard<py::gil_scoped_release>())
      .def("read_gyro_raw", &Lsm6ds3Spi::read_gyro_raw,
           py::call_guard<py::gil_scoped_release>())
      .def("read_accel", &Lsm6ds3Spi::read_accel_si, py::call_guard<py::gil_scoped_release>())
      .def("read_gyro", &Lsm6ds3Spi::read_gyro_si, py::call_guard<py::gil_scoped_release>());
}
