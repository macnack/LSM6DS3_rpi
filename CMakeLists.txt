cmake_minimum_required(VERSION 3.20)
project(rpi_sensors_superbuild LANGUAGES C CXX)
include(CTest)

option(BUILD_IMU "Build the IMU (LSM6DS3-family) library and examples" ON)
option(BUILD_BARO "Build the barometer (BMP390) library and examples" ON)
option(BUILD_SERVO "Build the hardware PWM servo library and examples" ON)
option(BUILD_RUNTIME "Build the runtime (rt_core + IPC + tests)" ON)

# Default to C++-only builds at the top level. Enable explicitly if needed.
set(LSM6DS3_BUILD_PYTHON OFF CACHE BOOL "Build pybind11 module for IMU" )
set(BMP390_BUILD_PYTHON OFF CACHE BOOL "Build pybind11 module for BARO" )
set(SERVO_BUILD_PYTHON OFF CACHE BOOL "Build pybind11 module for SERVO" )

if(BUILD_IMU)
  add_subdirectory(IMU)
endif()

if(BUILD_BARO)
  add_subdirectory(BARO)
endif()

if(BUILD_SERVO)
  add_subdirectory(SERVO)
endif()

if(BUILD_RUNTIME)
  add_subdirectory(runtime)
endif()
