cmake_minimum_required(VERSION 3.20)
project(bmp390_rpi LANGUAGES C CXX)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(BMP390_BUILD_PYTHON "Build pybind11 module" ON)

add_library(bmp390 STATIC
  src/linux_i2c.cpp
  src/bmp390.cpp
  I2C/bmp3.c
)

set_target_properties(bmp390 PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(bmp390
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/I2C
)

target_compile_definitions(bmp390 PRIVATE BMP3_FLOAT_COMPENSATION)
target_compile_options(bmp390 PRIVATE -Wall -Wextra -Wpedantic)

add_executable(bmp390_read_once examples/read_once.cpp)
target_link_libraries(bmp390_read_once PRIVATE bmp390)

if(BMP390_BUILD_PYTHON)
  find_package(pybind11 CONFIG REQUIRED)

  pybind11_add_module(_bmp390 src/pybind_module.cpp)
  target_link_libraries(_bmp390 PRIVATE bmp390)
  target_include_directories(_bmp390 PRIVATE include)

  install(TARGETS _bmp390 LIBRARY DESTINATION bmp390_rpi)
endif()

install(TARGETS bmp390 ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
