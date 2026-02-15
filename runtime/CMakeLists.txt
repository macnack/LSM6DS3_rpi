cmake_minimum_required(VERSION 3.20)
project(runtime LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(RUNTIME_BUILD_IPC_CODEC_PYBIND "Build the _runtime_ipc_codec pybind11 module" ON)
option(RUNTIME_REQUIRE_PYBIND "Fail configure if pybind11 codec module/tests cannot be enabled" OFF)

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
  src/runtime/failsafe_diagnostics.cpp
  src/runtime/kill_switch.cpp
  src/runtime/rt_thread.cpp
  src/runtime/runtime.cpp
)
set_target_properties(runtime_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

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

  add_executable(runtime_unit_runtime_failsafe_causes tests/test_runtime_failsafe_causes.cpp)
  target_link_libraries(runtime_unit_runtime_failsafe_causes PRIVATE runtime_core)
  target_compile_options(runtime_unit_runtime_failsafe_causes PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME runtime_unit_runtime_failsafe_causes COMMAND runtime_unit_runtime_failsafe_causes)

  add_executable(runtime_unit_sim_net_link tests/test_sim_net_link.cpp)
  target_link_libraries(runtime_unit_sim_net_link PRIVATE runtime_core)
  target_compile_options(runtime_unit_sim_net_link PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME runtime_unit_sim_net_link COMMAND runtime_unit_sim_net_link)

  find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

  if(RUNTIME_REQUIRE_PYBIND AND NOT RUNTIME_BUILD_IPC_CODEC_PYBIND)
    message(FATAL_ERROR "RUNTIME_REQUIRE_PYBIND=ON requires RUNTIME_BUILD_IPC_CODEC_PYBIND=ON")
  endif()

  set(RUNTIME_IPC_CODEC_MODULE_BUILT OFF)
  if(RUNTIME_BUILD_IPC_CODEC_PYBIND)
    if(POLICY CMP0148)
      cmake_policy(SET CMP0148 NEW)
    endif()
    set(PYBIND11_FINDPYTHON ON)
    find_package(pybind11 CONFIG QUIET)
    if(pybind11_FOUND)
      pybind11_add_module(_runtime_ipc_codec src/python/runtime_ipc_codec.cpp)
      target_link_libraries(_runtime_ipc_codec PRIVATE runtime_core)
      target_compile_options(_runtime_ipc_codec PRIVATE -Wall -Wextra -Wpedantic)
      set_target_properties(_runtime_ipc_codec PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
      set(RUNTIME_IPC_CODEC_MODULE_BUILT ON)

      add_test(
        NAME runtime_unit_ipc_codec_cpp_to_python
        COMMAND
          ${Python3_EXECUTABLE}
          ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_ipc_codec_cpp_to_python.py
          --module-dir ${CMAKE_CURRENT_BINARY_DIR}
      )

      add_test(
        NAME runtime_unit_ipc_codec_python_to_cpp
        COMMAND
          ${Python3_EXECUTABLE}
          ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_ipc_codec_python_to_cpp.py
          --module-dir ${CMAKE_CURRENT_BINARY_DIR}
      )
    elseif(RUNTIME_REQUIRE_PYBIND)
      message(FATAL_ERROR "pybind11 not found but RUNTIME_REQUIRE_PYBIND=ON")
    else()
      message(WARNING "pybind11 not found; skipping _runtime_ipc_codec module and cross-language IPC codec tests")
    endif()
  else()
    message(STATUS "RUNTIME_BUILD_IPC_CODEC_PYBIND=OFF; using pure-Python runtime IPC codec only")
  endif()

  add_test(
    NAME runtime_unit_python_ipc_codec
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_python_ipc_codec.py
      --backend python
      --expect-backend python
  )
  if(RUNTIME_IPC_CODEC_MODULE_BUILT)
    add_test(
      NAME runtime_unit_python_ipc_codec_cpp_backend
      COMMAND
        ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_python_ipc_codec.py
        --module-dir ${CMAKE_CURRENT_BINARY_DIR}
        --backend cpp
        --expect-backend cpp
    )
  endif()

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
