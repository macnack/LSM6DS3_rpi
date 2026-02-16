cmake_minimum_required(VERSION 3.20)
project(ads1115_rpi LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(ADS1115_BUILD_PYTHON "Build pybind11 module" ON)

add_library(ads1115 STATIC
  src/linux_i2c.cpp
  src/ads1115.cpp
)

set_target_properties(ads1115 PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(ads1115
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_options(ads1115 PRIVATE -Wall -Wextra -Wpedantic)

add_executable(ads1115_read_once examples/read_once.cpp)
target_link_libraries(ads1115_read_once PRIVATE ads1115)

if(BUILD_TESTING)
  add_executable(ads1115_tests tests/test_ads1115.cpp)
  target_link_libraries(ads1115_tests PRIVATE ads1115)
  add_test(NAME ads1115_tests COMMAND ads1115_tests)
endif()

if(ADS1115_BUILD_PYTHON)
  if(POLICY CMP0148)
    cmake_policy(SET CMP0148 NEW)
  endif()
  set(PYBIND11_FINDPYTHON ON)
  find_package(Python COMPONENTS Interpreter Development REQUIRED)
  find_package(pybind11 CONFIG REQUIRED)

  pybind11_add_module(_ads1115 src/pybind_module.cpp)
  target_link_libraries(_ads1115 PRIVATE ads1115)
  target_include_directories(_ads1115 PRIVATE include)

  install(TARGETS _ads1115 LIBRARY DESTINATION ADS1115_rpi)
endif()

install(TARGETS ads1115 ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
