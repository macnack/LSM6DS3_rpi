cmake_minimum_required(VERSION 3.20)
project(servo_rpi LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(SERVO_BUILD_PYTHON "Build pybind11 module" ON)

add_library(servo STATIC
  src/hardware_pwm.cpp
)

set_target_properties(servo PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(servo
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_options(servo PRIVATE -Wall -Wextra -Wpedantic)

add_executable(servo_set_pulse examples/set_pulse.cpp)
target_link_libraries(servo_set_pulse PRIVATE servo)

if(SERVO_BUILD_PYTHON)
  if(POLICY CMP0148)
    cmake_policy(SET CMP0148 NEW)
  endif()
  set(PYBIND11_FINDPYTHON ON)
  find_package(Python COMPONENTS Interpreter Development REQUIRED)
  find_package(pybind11 CONFIG REQUIRED)

  pybind11_add_module(_servo src/pybind_module.cpp)
  target_link_libraries(_servo PRIVATE servo)
  target_include_directories(_servo PRIVATE include)

  install(TARGETS _servo LIBRARY DESTINATION servo_rpi)
endif()

install(TARGETS servo ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
