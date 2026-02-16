cmake_minimum_required(VERSION 3.20)
project(igniter_rpi LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(IGNITER_BUILD_PYTHON "Build pybind11 module" ON)

find_package(PkgConfig QUIET)
set(IGNITER_HAVE_LIBGPIOD 0)
if(PkgConfig_FOUND)
  pkg_check_modules(LIBGPIOD QUIET libgpiod)
  if(LIBGPIOD_FOUND)
    set(IGNITER_HAVE_LIBGPIOD 1)
  endif()
endif()

add_library(igniter STATIC
  src/gpio_batch.cpp
  src/vn5e160s.cpp
  src/igniter.cpp
  src/igniter_bank.cpp
)

set_target_properties(igniter PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(igniter
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_definitions(igniter PRIVATE IGNITER_HAVE_LIBGPIOD=${IGNITER_HAVE_LIBGPIOD})
target_compile_options(igniter PRIVATE -Wall -Wextra -Wpedantic)

if(IGNITER_HAVE_LIBGPIOD)
  target_include_directories(igniter PRIVATE ${LIBGPIOD_INCLUDE_DIRS})
  target_link_libraries(igniter PRIVATE ${LIBGPIOD_LINK_LIBRARIES})
endif()

if(IGNITER_BUILD_PYTHON)
  if(POLICY CMP0148)
    cmake_policy(SET CMP0148 NEW)
  endif()
  set(PYBIND11_FINDPYTHON ON)
  find_package(Python COMPONENTS Interpreter Development REQUIRED)
  find_package(pybind11 CONFIG REQUIRED)

  pybind11_add_module(_igniter src/pybind_module.cpp)
  target_link_libraries(_igniter PRIVATE igniter)
  target_include_directories(_igniter PRIVATE include)

  install(TARGETS _igniter LIBRARY DESTINATION igniter_rpi)
endif()

if(BUILD_TESTING)
  add_executable(igniter_unit_bank tests/test_igniter_bank.cpp)
  target_link_libraries(igniter_unit_bank PRIVATE igniter)
  target_compile_options(igniter_unit_bank PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME igniter_unit_bank COMMAND igniter_unit_bank)
endif()

install(TARGETS igniter ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
