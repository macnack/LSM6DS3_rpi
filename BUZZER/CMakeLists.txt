cmake_minimum_required(VERSION 3.20)
project(buzzer_rpi LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(BUZZER_BUILD_PYTHON "Build pybind11 module" ON)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBGPIOD REQUIRED libgpiod)

add_library(buzzer STATIC
  src/hardware_buzzer.cpp
  src/engine.cpp
  src/profile.cpp
)

set_target_properties(buzzer PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(buzzer
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${LIBGPIOD_INCLUDE_DIRS}
)

target_link_libraries(buzzer
  PUBLIC
    pthread
  PRIVATE
    ${LIBGPIOD_LINK_LIBRARIES}
)

target_compile_options(buzzer PRIVATE -Wall -Wextra -Wpedantic)

add_executable(buzzer_play_profile examples/play_profile.cpp)
target_link_libraries(buzzer_play_profile PRIVATE buzzer)

if(BUILD_TESTING)
  add_executable(buzzer_unit_low_level tests/test_buzzer_low_level.cpp)
  target_link_libraries(buzzer_unit_low_level PRIVATE buzzer)
  target_compile_options(buzzer_unit_low_level PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME buzzer_unit_low_level COMMAND buzzer_unit_low_level)

  add_executable(buzzer_unit_engine tests/test_buzzer_engine.cpp)
  target_link_libraries(buzzer_unit_engine PRIVATE buzzer)
  target_compile_options(buzzer_unit_engine PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME buzzer_unit_engine COMMAND buzzer_unit_engine)

  find_package(Python3 COMPONENTS Interpreter REQUIRED)

  add_test(
    NAME buzzer_profile_py
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_buzzer_profile.py
      --package-dir ${CMAKE_CURRENT_SOURCE_DIR}/python
      --profile ${CMAKE_CURRENT_SOURCE_DIR}/profiles/ardupilot_like_v1.json
  )

  add_test(
    NAME buzzer_cli_py
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_cli.py
      --package-dir ${CMAKE_CURRENT_SOURCE_DIR}/python
      --profile ${CMAKE_CURRENT_SOURCE_DIR}/profiles/ardupilot_like_v1.json
  )
endif()

if(BUZZER_BUILD_PYTHON)
  if(POLICY CMP0148)
    cmake_policy(SET CMP0148 NEW)
  endif()
  set(PYBIND11_FINDPYTHON ON)
  find_package(Python COMPONENTS Interpreter Development REQUIRED)
  find_package(pybind11 CONFIG REQUIRED)

  pybind11_add_module(_buzzer src/pybind_module.cpp)
  target_link_libraries(_buzzer PRIVATE buzzer)
  target_include_directories(_buzzer PRIVATE include ${LIBGPIOD_INCLUDE_DIRS})

  install(TARGETS _buzzer LIBRARY DESTINATION buzzer_rpi)
endif()

install(TARGETS buzzer ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY profiles/ DESTINATION share/buzzer_rpi)
