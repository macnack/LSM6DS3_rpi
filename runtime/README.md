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
- `igniter_worker`: synchronized 4-channel igniter service (`VN5E160S` + latch/fault logic + IPC command/status).
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
- `/rt_igniter_command_v1` (`IgniterCommandMsg`: external client -> RT igniter service)
- `/rt_igniter_status_v1` (`IgniterStatusMsg`: RT igniter service -> external client)

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
- `[sim_net]` (TCP bridge mode)
- `[imu_watchdog]` (invalid-stream detection and IMU auto-reinit)
- `[igniter]`, `[igniter0]`, `[igniter1]`, `[igniter2]`, `[igniter3]` (4-channel synchronized igniter service)

Important BARO/I2C keys in `[baro]`:

- `recovery_error_threshold`: consecutive read failures before BARO stop/start recovery
- `recovery_backoff_ms`: wait between stop/start recovery attempts

Important IMU keys in `[imu]`:

- `accel_odr`: accelerometer ODR (`power_down`, `12.5hz`, `26hz`, `52hz`, `104hz`, `208hz`, `416hz`, `833hz`)
- `gyro_odr`: gyroscope ODR (`power_down`, `12.5hz`, `26hz`, `52hz`, `104hz`, `208hz`, `416hz`, `833hz`)
- `accel_scale`: accelerometer full-scale (`2g`, `4g`, `8g`, `16g`)
- `gyro_scale`: gyroscope full-scale (`245dps`, `500dps`, `1000dps`, `2000dps`)

IMU watchdog keys in `[imu_watchdog]`:

- `enabled`: enable invalid-stream watchdog and auto-recovery logic
- `zero_vector_consecutive`: consecutive all-zero samples before fault
- `flatline_consecutive`: consecutive unchanged samples (within epsilon) before fault
- `degenerate_pattern_consecutive`: consecutive `ax≈ay≈az` and `gx≈gy≈gz` samples before fault
- `flatline_epsilon`: epsilon for flatline/degenerate comparisons
- `healthy_recovery_samples`: required consecutive healthy samples before clearing fault
- `recovery_backoff_ms`: wait between IMU stop/start reinit attempts
- `max_reinit_attempts`: maximum reinit attempts (`0` means unlimited)

Kill switch keys in `[killswitch]`:

- `enabled`: enable hardware kill switch monitoring
- `gpio`: Linux GPIO number (`/sys/class/gpio/gpioN`)
- `nc_closed_value`: expected input value when NC loop is healthy (`0` or `1`)
- `debounce_samples`: consecutive open samples required to trip
- `latch_on_trip`: if true, kill switch remains tripped until restart

Security keys in `[security]`:

- `require_local_ipc_permissions`: enforce strict mailbox ownership/permissions before IPC opens
- `require_loopback_sim_net`: require `127.0.0.1` sim TCP endpoints

Igniter keys in `[igniter]`:

- `enabled`: enable igniter worker and IPC service
- `use_hardware`: use hardware GPIO backend (libgpiod) instead of sim backend
- `fault_policy`: `global` or `isolated`
- `settle_ms`: post-switch status filter window
- `latch_faults`: latch VN5 fault state
- `default_fire_ms`: fire duration used when duration=0
- `max_fire_ms`: hard safety cap
- `command_shm`: igniter command mailbox name
- `status_shm`: igniter status mailbox name

Igniter channel keys in `[igniter0..3]`:

- `enabled`: channel is managed by igniter worker
- `input_chip`, `input_line`: VN5 INPUT GPIO chip path + line offset
- `status_chip`, `status_line`: VN5 STATUS GPIO chip path + line offset

Synchronization constraints:

- when `[igniter].enabled=true`, all `igniter0..3` channels must be enabled
- all `input_chip` values must be identical (single gpiochip batch write path)
- all `status_chip` values must be identical
- input and status lines must be unique per channel

This enforces single-batch group triggering for `fire_mask` / `fire_all`.

## Igniter Service (4 Channels)

`igniter_worker` runs independently from control/actuator workers and handles:

- command IPC (`arm`, `disarm`, `fire_mask`, `clear_fault`)
- batch output writes for synchronized multi-channel fire
- per-channel STATUS sampling and fault transitions
- status IPC publication (state/fault/remaining duration per channel)

If kill switch is active, igniters are forced to disarmed/off state.

## Raspberry Pi 5 / CM5 GPIO Selection Guide

For synchronized igniter channels on CM5/RPi5:

1. detect chips:
   - `gpiodetect`
2. identify RP1 user-GPIO chip by label (usually `pinctrl-rp1`)
3. inspect available lines:
   - `gpioinfo <gpiochipX>`
4. choose four INPUT lines and four STATUS lines on the same chip
5. set `input_chip`/`status_chip` to that chip path (for example `/dev/gpiochip4`)

Do not hardcode `gpiochip0`; resolve by label on the target image.

References:

- https://pip-assets.raspberrypi.com/categories/685-whitepapers-app-notes/documents/RP-006553-WP/A-history-of-GPIO-usage-on-Raspberry-Pi-devices-and-current-best-practices
- https://libgpiod.readthedocs.io/en/stable/gpioset.html
- https://www.kernel.org/doc/html/latest/driver-api/gpio/using-gpio.html

## Igniter Preflight Checklist

Before live test/flight with igniters:

1. validate config loads cleanly with `[igniter].enabled=true`
2. verify all `igniter0..3` channels are enabled and use one `input_chip`
3. verify all `status_chip` values are identical and all lines are unique
4. run `arm` then `disarm` command path and confirm status updates in `/rt_igniter_status_v1`
5. run `fire-mask --mask 0x0F` only with safe load and confirm simultaneous transition on all channels
6. if any fault is latched, run `clear-fault` and re-check `disarmed` state before re-arming

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

## IMU Watchdog and Recovery Policy

`imu_worker` applies IMU invalid-stream detection before publishing `imu_valid`:

1. reject NaN/Inf IMU fields immediately
2. detect consecutive all-zero vectors
3. detect consecutive flatline samples across all 6 IMU channels
4. detect consecutive degenerate patterns where accel axes collapse (`ax≈ay≈az`) and gyro axes collapse (`gx≈gy≈gz`)

On fault, runtime forces `imu_valid=false`, so the existing control-loop stale/invalid handling enters failsafe.
Runtime then performs periodic IMU backend `stop()` + `start()` reinit attempts using `imu_watchdog.recovery_backoff_ms`
until healthy samples are sustained for `imu_watchdog.healthy_recovery_samples`.

No accel-magnitude plausibility check is used by this watchdog.

## Metrics and Jitter

Runtime status file now includes:

- deadline misses: control, actuator, imu, baro
- igniter worker counters (`igniter_ticks`, command accept/reject)
- jitter percentiles (`p50`, `p95`, `p99`, `max`) for control, actuator, imu, baro scheduling
- actuator command age (`last`, `max`)
- `i2c_recovery_count`
- IMU watchdog counters (`imu_watchdog_fault_count`, zero/flatline/degenerate counters)
- IMU reinit counters (`imu_reinit_attempt_count`, `imu_reinit_success_count`, `imu_reinit_failure_count`)
- IMU watchdog state and fault reason (`imu_watchdog_state_name`, `imu_last_fault_reason_name`)
- kill switch state and trip count
- igniter state (`igniter_armed`, `igniter_global_fault_latched`, `igniter_active_mask`)
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
  -DSERVO_BUILD_PYTHON=OFF \
  -DIGNITER_BUILD_PYTHON=OFF
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
  -DSERVO_BUILD_PYTHON=OFF \
  -DIGNITER_BUILD_PYTHON=OFF
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

Igniter IPC client examples:

```bash
python3 ./runtime/python/igniter_cli.py --config ./runtime/config/rt_core_cpp_native.toml arm
python3 ./runtime/python/igniter_cli.py --config ./runtime/config/rt_core_cpp_native.toml fire-mask --mask 0x0F --d0 200 --d1 200 --d2 200 --d3 200
python3 ./runtime/python/igniter_cli.py --config ./runtime/config/rt_core_cpp_native.toml status
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
- `runtime_unit_igniter_config_validation`
- `runtime_unit_ipc_codec_cpp_to_python`
- `runtime_unit_ipc_codec_python_to_cpp`
- `runtime_unit_python_ipc_codec`
- `runtime_unit_python_ipc_codec_cpp_backend` (when `_runtime_ipc_codec` is built)
- `runtime_smoke_python_dev`

## Notes

- `rt_core` is the owner/creator/unlinker of mailbox objects.
- Python workers retry opening IPC mailboxes until available.
- In restricted environments where POSIX shared memory is blocked, implementation falls back to `/tmp/*.mailbox` mmap while preserving the same sequence-counter protocol.
