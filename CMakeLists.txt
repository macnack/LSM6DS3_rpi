cmake_minimum_required(VERSION 3.20)
project(lsm6ds3_rpi LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(LSM6DS3_BUILD_PYTHON "Build pybind11 module" ON)

add_library(lsm6ds3 STATIC
  src/linux_i2c.cpp
  src/lsm6ds3.cpp
)

set_target_properties(lsm6ds3 PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(lsm6ds3
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_options(lsm6ds3 PRIVATE -Wall -Wextra -Wpedantic)


add_executable(lsm6ds3_read_once examples/read_once.cpp)
target_link_libraries(lsm6ds3_read_once PRIVATE lsm6ds3)

if(LSM6DS3_BUILD_PYTHON)
  find_package(pybind11 CONFIG REQUIRED)

  pybind11_add_module(_lsm6ds3 src/pybind_module.cpp)
  target_link_libraries(_lsm6ds3 PRIVATE lsm6ds3)
  target_include_directories(_lsm6ds3 PRIVATE include)

  install(TARGETS _lsm6ds3 LIBRARY DESTINATION lsm6ds3_rpi)
endif()

install(TARGETS lsm6ds3 ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
