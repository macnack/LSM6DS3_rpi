cmake_minimum_required(VERSION 3.20)
project(runtime LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(runtime_core STATIC
  src/config/toml_parser.cpp
  src/config/config_loader.cpp
  src/ipc/crc32.cpp
  src/ipc/shm_mailbox.cpp
  src/runtime/fallback.cpp
  src/runtime/estimator.cpp
  src/runtime/controller.cpp
  src/runtime/actuator.cpp
  src/runtime/sensors.cpp
  src/runtime/sim_net.cpp
  src/runtime/external_time_validation.cpp
  src/runtime/kill_switch.cpp
  src/runtime/rt_thread.cpp
  src/runtime/runtime.cpp
)

target_include_directories(runtime_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(runtime_core
  PUBLIC
    servo
    pthread
)

if(UNIX AND NOT APPLE)
  target_link_libraries(runtime_core PUBLIC rt)
endif()

set(RUNTIME_HAVE_IMU 0)
set(RUNTIME_HAVE_BARO 0)
if(TARGET lsm6ds3)
  target_link_libraries(runtime_core PUBLIC lsm6ds3)
  set(RUNTIME_HAVE_IMU 1)
endif()
if(TARGET bmp390)
  target_link_libraries(runtime_core PUBLIC bmp390)
  set(RUNTIME_HAVE_BARO 1)
endif()

target_compile_definitions(runtime_core PRIVATE
  RUNTIME_HAVE_IMU=${RUNTIME_HAVE_IMU}
  RUNTIME_HAVE_BARO=${RUNTIME_HAVE_BARO}
)

target_compile_options(runtime_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(rt_core src/main.cpp)
target_link_libraries(rt_core PRIVATE runtime_core)
target_compile_options(rt_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(dummy_estimator_cpp src/dummy_estimator.cpp)
target_link_libraries(dummy_estimator_cpp PRIVATE runtime_core)
target_compile_options(dummy_estimator_cpp PRIVATE -Wall -Wextra -Wpedantic)

add_executable(dummy_controller_cpp src/dummy_controller.cpp)
target_link_libraries(dummy_controller_cpp PRIVATE runtime_core)
target_compile_options(dummy_controller_cpp PRIVATE -Wall -Wextra -Wpedantic)

if(BUILD_TESTING)
  add_executable(runtime_unit_fallback tests/test_fallback.cpp)
  target_link_libraries(runtime_unit_fallback PRIVATE runtime_core)
  target_compile_options(runtime_unit_fallback PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME runtime_unit_fallback COMMAND runtime_unit_fallback)

  add_executable(runtime_unit_ipc tests/test_ipc_mailbox.cpp)
  target_link_libraries(runtime_unit_ipc PRIVATE runtime_core)
  target_compile_options(runtime_unit_ipc PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME runtime_unit_ipc COMMAND runtime_unit_ipc)

  add_executable(runtime_unit_external_time_validation tests/test_external_time_validation.cpp)
  target_link_libraries(runtime_unit_external_time_validation PRIVATE runtime_core)
  target_compile_options(runtime_unit_external_time_validation PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME runtime_unit_external_time_validation COMMAND runtime_unit_external_time_validation)

  find_package(Python3 COMPONENTS Interpreter REQUIRED)
  configure_file(tests/smoke_rt_core.toml.in ${CMAKE_CURRENT_BINARY_DIR}/smoke_rt_core.toml @ONLY)
  add_test(
    NAME runtime_smoke_python_dev
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/smoke_python_dev.py
      --rt-core $<TARGET_FILE:rt_core>
      --config ${CMAKE_CURRENT_BINARY_DIR}/smoke_rt_core.toml
      --dummy-estimator ${CMAKE_CURRENT_SOURCE_DIR}/python/dummy_estimator.py
      --dummy-controller ${CMAKE_CURRENT_SOURCE_DIR}/python/dummy_controller.py
  )
  set_tests_properties(runtime_smoke_python_dev PROPERTIES TIMEOUT 45)
endif()
