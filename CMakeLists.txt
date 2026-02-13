cmake_minimum_required(VERSION 3.20)
project(rpi_sensors_superbuild LANGUAGES C CXX)

option(BUILD_IMU "Build the IMU (LSM6DS3-family) library and examples" ON)
option(BUILD_BARO "Build the barometer (BMP390) library and examples" ON)

# Default to C++-only builds at the top level. Enable explicitly if needed.
set(LSM6DS3_BUILD_PYTHON OFF CACHE BOOL "Build pybind11 module for IMU" )
set(BMP390_BUILD_PYTHON OFF CACHE BOOL "Build pybind11 module for BARO" )

if(BUILD_IMU)
  add_subdirectory(IMU)
endif()

if(BUILD_BARO)
  add_subdirectory(BARO)
endif()
