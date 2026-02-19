cmake_minimum_required(VERSION 3.20)
project(ms4525do_rpi LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(MS4525DO_BUILD_PYTHON "Build pybind11 module" ON)

add_library(ms4525do STATIC
  src/linux_i2c.cpp
  src/ms4525do.cpp
)

set_target_properties(ms4525do PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(ms4525do
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_options(ms4525do PRIVATE -Wall -Wextra -Wpedantic)

add_executable(ms4525do_read_once examples/read_once.cpp)
target_link_libraries(ms4525do_read_once PRIVATE ms4525do)

if(BUILD_TESTING)
  add_executable(ms4525do_tests tests/test_ms4525do.cpp)
  target_link_libraries(ms4525do_tests PRIVATE ms4525do)
  add_test(NAME ms4525do_tests COMMAND ms4525do_tests)
endif()

if(MS4525DO_BUILD_PYTHON)
  if(POLICY CMP0148)
    cmake_policy(SET CMP0148 NEW)
  endif()
  set(PYBIND11_FINDPYTHON ON)
  find_package(Python COMPONENTS Interpreter Development REQUIRED)
  find_package(pybind11 CONFIG REQUIRED)

  pybind11_add_module(_ms4525do src/pybind_module.cpp)
  target_link_libraries(_ms4525do PRIVATE ms4525do)
  target_include_directories(_ms4525do PRIVATE include)

  install(TARGETS _ms4525do LIBRARY DESTINATION ms4525do_rpi)
endif()

install(TARGETS ms4525do ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
