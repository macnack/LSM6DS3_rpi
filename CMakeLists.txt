cmake_minimum_required(VERSION 3.20)
project(rpi_sensors_superbuild LANGUAGES C CXX)
include(CTest)

option(BUILD_IMU "Build the IMU (LSM6DS3-family) library and examples" ON)
option(BUILD_BARO "Build the barometer (BMP390) library and examples" ON)
option(BUILD_SERVO "Build the hardware PWM servo library and examples" ON)
option(BUILD_ADC "Build the ADS1115 ADC library and examples" ON)
option(BUILD_AIRSPEED "Build the MS4525DO airspeed library and examples" ON)
option(BUILD_IGNITER "Build the synchronized igniter library and examples" ON)
option(BUILD_BUZZER "Build the buzzer (LPB2418A-style) library and examples" ON)
option(BUILD_RUNTIME "Build the runtime (rt_core + IPC + tests)" ON)

# Default to C++-only builds at the top level. Enable explicitly if needed.
set(LSM6DS3_BUILD_PYTHON OFF CACHE BOOL "Build pybind11 module for IMU" )
set(BMP390_BUILD_PYTHON OFF CACHE BOOL "Build pybind11 module for BARO" )
set(SERVO_BUILD_PYTHON OFF CACHE BOOL "Build pybind11 module for SERVO" )
set(ADS1115_BUILD_PYTHON OFF CACHE BOOL "Build pybind11 module for ADC" )
set(MS4525DO_BUILD_PYTHON OFF CACHE BOOL "Build pybind11 module for AIRSPEED" )
set(IGNITER_BUILD_PYTHON OFF CACHE BOOL "Build pybind11 module for IGNITER" )
set(BUZZER_BUILD_PYTHON OFF CACHE BOOL "Build pybind11 module for BUZZER" )

if(BUILD_IMU)
  add_subdirectory(IMU)
endif()

if(BUILD_BARO)
  add_subdirectory(BARO)
endif()

if(BUILD_SERVO)
  add_subdirectory(SERVO)
endif()

if(BUILD_ADC)
  add_subdirectory(ADC)
endif()

if(BUILD_AIRSPEED)
  add_subdirectory(AIRSPEED)
endif()

if(BUILD_IGNITER)
  add_subdirectory(IGNITER)
endif()

if(BUILD_BUZZER)
  add_subdirectory(BUZZER)
endif()

if(BUILD_RUNTIME)
  add_subdirectory(runtime)
endif()
