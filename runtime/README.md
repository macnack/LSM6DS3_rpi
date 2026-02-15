# RT Runtime (`rt_core`)

`rt_core` is the runtime process that owns deterministic control tick timing and actuator outputs.
It supports C++ native estimator/controller and external development workers (Python or C++) over shared-memory IPC.

## Development Guide

For implementation-focused extension workflows (new workers, additional I2C devices, custom estimator/controller), see:

- `runtime/DEVELOPMENT.md`

## Architecture

Workers in `rt_core`:

- `control_thread` (highest priority): fixed-rate control tick, estimator/controller source selection, publishes actuator command.
- `actuator_worker`: exclusive owner of 4-servo actuator outputs with clamp, slew, arm/disarm, and failsafe handling.
- `imu_worker`: high-rate IMU sampling (`SPI`) or simulated IMU in sim mode.
- `i2c_hub_worker`: I2C job registry scheduler (BARO now; extensible for future sensors).
- `estimator_thread`: C++ native estimator producer (always available as fallback path).

External workers (optional):

- `runtime/python/dummy_estimator.py`
- `runtime/python/dummy_controller.py`
- `build/runtime/dummy_estimator_cpp`
- `build/runtime/dummy_controller_cpp`

## Data Flow

`sensors -> estimator -> controller -> actuator_worker`

- `imu_worker` and `i2c_hub_worker` publish latest sensor data into runtime shared state.
- `estimator_thread` computes C++ estimator state from sensor streams.
- `control_thread` selects estimator source and controller source by mode and freshness policies.
- `actuator_worker` maps normalized commands to calibrated PWM and writes exactly 4 channels.

## Modes

- `estimator_mode = cpp_native | python_dev | cpp_dev`
- `controller_mode = cpp_native | python_dev | cpp_dev`

Selection policies:

- Estimator (`python_dev` or `cpp_dev`): `fresh external -> last valid external -> cpp fallback -> failsafe/degraded`
- Controller (`python_dev` or `cpp_dev`): `fresh external -> last valid external -> failsafe`

C++ control tick always runs and never blocks on IPC.

## IPC Schema

Mailboxes use a sequence-counter latest-value protocol.
Primary transport is POSIX shared memory (`shm_open + mmap`).

- seq counter outside payload in shared memory
- writer sets odd seq during write and even seq when stable
- reader uses double-read seq check; non-blocking reads only

### POSIX SHM Fallback Behavior

`rt_core` is designed for POSIX SHM first. In some restricted environments (sandbox, container policy, macOS permission profile), `shm_open` may be denied.

When that happens, runtime transparently falls back to file-backed shared mappings in `/tmp/<mailbox>.mailbox`:

- protocol is unchanged (same seq counter semantics and payload structs)
- Python workers and C++ runtime still interoperate with the same mailbox names
- production Linux deployment should still prefer native POSIX SHM

Mailbox names (defaults):

- `/rt_sensor_snapshot_v1` (`SensorSnapshotMsg`: RT -> external estimator worker)
- `/rt_estimator_state_v1` (`ExternalEstimatorStateMsg`: external estimator worker -> RT)
- `/rt_controller_command_v1` (`ExternalControllerCommandMsg`: external controller worker -> RT)

Message validation:

- fixed header fields (`msg_magic`, `msg_version`, `payload_bytes`)
- CRC32 over payload bytes excluding `crc32`
- freshness checks with monotonic timestamps
- range/NaN checks and quaternion norm checks

## Config

Runtime is TOML-only (`--config <path>`). Required sections:

- `[runtime]`, `[modes]`, `[threads]`, `[timeouts]`, `[ipc]`, `[imu]`, `[baro]`, `[actuator]`
- `[servo0]`, `[servo1]`, `[servo2]`, `[servo3]`
- `[estimator_cpp]`, `[controller_cpp]`

Optional sections:

- `[killswitch]` (hardware NC emergency input)

Important BARO/I2C keys in `[baro]`:

- `recovery_error_threshold`: consecutive read failures before BARO stop/start recovery
- `recovery_backoff_ms`: wait between stop/start recovery attempts

Kill switch keys in `[killswitch]`:

- `enabled`: enable hardware kill switch monitoring
- `gpio`: Linux GPIO number (`/sys/class/gpio/gpioN`)
- `nc_closed_value`: expected input value when NC loop is healthy (`0` or `1`)
- `debounce_samples`: consecutive open samples required to trip
- `latch_on_trip`: if true, kill switch remains tripped until restart

Security keys in `[security]`:

- `require_local_ipc_permissions`: enforce strict mailbox ownership/permissions before IPC opens
- `require_loopback_sim_net`: require `127.0.0.1` sim TCP endpoints

## Kill Switch (NC)

Kill switch is implemented as a **normally closed** hardware loop on a GPIO input.

- healthy/closed loop: GPIO value must match `nc_closed_value`
- open loop (wire break / E-stop event): runtime trips kill switch
- trip action: controller command is forced to failsafe and actuator outputs remain in failsafe gate

This path is checked from the control tick and propagated to actuator worker with a latched safety state.

Note: sysfs GPIO is used for broad compatibility. Configure pull-up/down and wiring externally so NC healthy state is deterministic.

## I2C Recovery Policy

`i2c_hub_worker` applies BARO recovery policy:

1. count consecutive BARO poll failures
2. when failures reach `baro.recovery_error_threshold`:
3. stop BARO backend
4. sleep `baro.recovery_backoff_ms`
5. restart BARO backend
6. increment `i2c_recovery_count` metric

Control loop continues to run during I2C recovery.

## Metrics and Jitter

Runtime status file now includes:

- deadline misses: control, actuator, imu, baro
- jitter percentiles (`p50`, `p95`, `p99`, `max`) for control, actuator, imu, baro scheduling
- actuator command age (`last`, `max`)
- `i2c_recovery_count`
- kill switch state and trip count
- external worker accept/reject counters
- failsafe diagnostics (`failsafe_activation_count`, enter/exit events, per-cause counts)
- sim_net actuator link diagnostics (`sim_net_actuator_disconnects`, `sim_net_actuator_client_connected`)

Example configs:

- `runtime/config/rt_core_cpp_native.toml`
- `runtime/config/rt_core_python_dev.toml`
- `runtime/config/rt_core_sim_python_dev.toml`
- `runtime/config/rt_core_sim_cpp_dev.toml`

## Build

From repo root:

```bash
cmake -S . -B build \
  -DBUILD_RUNTIME=ON \
  -DLSM6DS3_BUILD_PYTHON=OFF \
  -DBMP390_BUILD_PYTHON=OFF \
  -DSERVO_BUILD_PYTHON=OFF
cmake --build build -j
```

Codec backend selection for Python workers (`runtime/python/ipc_common.py`):

- default: `auto` (use `_runtime_ipc_codec` if importable and schema-compatible, else pure Python)
- force Python: `RUNTIME_IPC_CODEC_BACKEND=python`
- force C++ codec: `RUNTIME_IPC_CODEC_BACKEND=cpp` (startup fails if module is missing/incompatible)

To require pybind codec build at configure time:

```bash
cmake -S . -B build \
  -DBUILD_RUNTIME=ON \
  -DRUNTIME_BUILD_IPC_CODEC_PYBIND=ON \
  -DRUNTIME_REQUIRE_PYBIND=ON \
  -Dpybind11_DIR="$(python3 -m pybind11 --cmakedir)"
```

Portable no-hardware build (sim runtime only):

```bash
cmake -S . -B build \
  -DBUILD_RUNTIME=ON \
  -DBUILD_IMU=OFF \
  -DBUILD_BARO=OFF \
  -DBUILD_SERVO=ON \
  -DLSM6DS3_BUILD_PYTHON=OFF \
  -DBMP390_BUILD_PYTHON=OFF \
  -DSERVO_BUILD_PYTHON=OFF
cmake --build build -j
```

## Run

C++ native path:

```bash
./build/runtime/rt_core --config ./runtime/config/rt_core_cpp_native.toml
```

Python development path:

```bash
./build/runtime/rt_core --config ./runtime/config/rt_core_python_dev.toml
python3 ./runtime/python/dummy_estimator.py --config ./runtime/config/rt_core_python_dev.toml
python3 ./runtime/python/dummy_controller.py --config ./runtime/config/rt_core_python_dev.toml
```

No-hardware end-to-end simulation:

```bash
./build/runtime/rt_core --config ./runtime/config/rt_core_sim_python_dev.toml
python3 ./runtime/python/dummy_estimator.py --config ./runtime/config/rt_core_sim_python_dev.toml
python3 ./runtime/python/dummy_controller.py --config ./runtime/config/rt_core_sim_python_dev.toml
```

C++ development path in separate terminals:

```bash
./build/runtime/rt_core --config ./runtime/config/rt_core_sim_cpp_dev.toml --print-config
./build/runtime/dummy_estimator_cpp --config ./runtime/config/rt_core_sim_cpp_dev.toml
./build/runtime/dummy_controller_cpp --config ./runtime/config/rt_core_sim_cpp_dev.toml
```

MuJoCo TCP bridge (sim_net):

```bash
./build/runtime/rt_core --config ./runtime/config/rt_core_sim_mujoco_tcp.toml --status-file /tmp/rt_status.txt
python3 dummy_rocket_sim/rt_bridge.py --sensor-port 56000 --actuator-port 56001 --headless
```

Production recommendation:

- keep `sim_net.enabled = false` in hardware production configs
- keep `runtime.allow_auto_sim_fallback = false` to fail fast instead of silently switching modes

Live link health monitor (sensor/actuator rates, stalls, disconnects):

```bash
python3 tools/rt_status_report.py /tmp/rt_status.txt --monitor --interval 0.5
```

Shareable runtime snapshot (for Slack/issues):

```bash
# Generate a compact token from one status snapshot.
python3 tools/rt_status_report.py /tmp/rt_status.txt --once --share

# Teammates can open the token directly.
rt-status-report --open-share rtshare1_<token>
```

Growth event hook for share funnel metrics:

- `share_snapshot_created`
- `share_snapshot_opened`
- Optional sink override: `RT_STATUS_REPORT_EVENT_LOG=/tmp/rt_status_report_events.jsonl`

CSV recorder (sensor + estimator + controller snapshots):

```bash
python3 tools/rt_csv_recorder.py \
  --config ./runtime/config/rt_core_sim_python_dev.toml \
  --output /tmp/rt_samples.csv \
  --duration-sec 20
```

With editable install (`python3 -m pip install -e .`), you can also run:

```bash
rt-csv-recorder --config ./runtime/config/rt_core_sim_python_dev.toml --output /tmp/rt_samples.csv
```

Useful CLI options:

- `--duration-sec <seconds>`
- `--status-file <path>`
- `--print-config`

## Tests

```bash
ctest --test-dir build --output-on-failure -R runtime_
```

CTest targets:

- `runtime_unit_fallback`
- `runtime_unit_ipc`
- `runtime_unit_external_time_validation`
- `runtime_unit_runtime_failsafe_causes`
- `runtime_unit_sim_net_link`
- `runtime_unit_ipc_codec_cpp_to_python`
- `runtime_unit_ipc_codec_python_to_cpp`
- `runtime_unit_python_ipc_codec`
- `runtime_unit_python_ipc_codec_cpp_backend` (when `_runtime_ipc_codec` is built)
- `runtime_smoke_python_dev`

## Notes

- `rt_core` is the owner/creator/unlinker of mailbox objects.
- Python workers retry opening IPC mailboxes until available.
- In restricted environments where POSIX shared memory is blocked, implementation falls back to `/tmp/*.mailbox` mmap while preserving the same sequence-counter protocol.
