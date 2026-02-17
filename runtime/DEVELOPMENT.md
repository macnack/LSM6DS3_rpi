# Runtime Development Guide

This guide explains how to extend `rt_core` safely:

- add a new runtime worker
- add more I2C devices in `i2c_hub_worker`
- implement your own estimator/controller (native C++ and external workers)

## 1) Branch Workflow (Before You Start)

Use a dedicated branch for each change. Branch names should use the `codex/` prefix.

Recommended naming:

- `codex/runtime-add-gps-worker`
- `codex/runtime-i2c-mag-sensor`
- `codex/runtime-ipc-schema-v2`

Commands:

```bash
git fetch origin
git checkout main
git pull --ff-only
git checkout -b codex/<short-kebab-change-name>
```

Before opening a merge request:

```bash
git status
git add <files>
git commit -m "runtime: <what changed>"
git push -u origin codex/<short-kebab-change-name>
```

## 2) Quick Architecture Map

Main runtime entrypoints:

- `runtime/src/main.cpp`: CLI and process startup (`Runtime::run()`).
- `runtime/src/runtime/runtime.cpp`: worker threads, scheduling, failover, IPC integration.
- `runtime/src/runtime/sensors.cpp`: hardware/sim sensor backends.

Core worker startup is in:

- `runtime/src/runtime/runtime.cpp` -> `start_workers()`

Current workers:

- `control_worker`
- `actuator_worker`
- `igniter_worker`
- `imu_worker`
- `i2c_hub_worker`
- `estimator_worker`

## 3) Where to Add New `cpp/hpp` Files

For runtime core code:

- headers: `runtime/include/runtime/...`
- sources: `runtime/src/...`
- build wiring: `runtime/CMakeLists.txt` (`runtime_core` source list)

Typical pattern:

1. Add API/type header in `runtime/include/runtime/<module>/<name>.hpp`.
2. Add implementation in `runtime/src/<module>/<name>.cpp`.
3. Add `.cpp` to `add_library(runtime_core STATIC ...)` in `runtime/CMakeLists.txt`.
4. Include and use from `runtime/src/runtime/runtime.cpp`.

If config is needed:

1. Add config struct fields in `runtime/include/runtime/config/config.hpp`.
2. Parse/validate keys in `runtime/src/config/config_loader.cpp`.
3. Add keys in sample TOML configs under `runtime/config/`.

## 4) Add a New Runtime Worker Thread

Use this when you want a dedicated periodic thread (for example GPS, logger, custom comms).

### Exact functions to edit

In `runtime/src/runtime/runtime.cpp`, add your worker in these exact places:

1. Declare the worker method in `class Runtime::Impl` private section:
   - add `void <name>_worker();` next to existing worker methods.
2. Implement `void Runtime::Impl::<name>_worker()` in the worker block (same area as `imu_worker`, `i2c_hub_worker`, `estimator_worker`).
3. Register thread start in `start_workers()`:
   - add `workers_.push_back(std::thread([this] { <name>_worker(); }));`
4. Do not change `join_workers()` logic unless special behavior is needed:
   - it already joins every thread in `workers_`.
5. If worker publishes data, add fields to `SharedState` and lock around reads/writes.
6. If worker exposes metrics, update:
   - `RuntimeStats` in `runtime/include/runtime/common/types.hpp`
   - status output in `maybe_write_status_file()`

### Step-by-step

1. Add shared state needed by the worker in `SharedState` inside `runtime/src/runtime/runtime.cpp`.
2. Add runtime stats counters in `RuntimeStats` (`runtime/include/runtime/common/types.hpp`) if you need status visibility.
3. Add thread frequency/priority config fields in:
   - `runtime/include/runtime/config/config.hpp` (`ThreadsSection`)
   - `runtime/src/config/config_loader.cpp` (allowed keys + parsing)
   - `runtime/config/*.toml` examples
4. Implement `void <name>_worker()` in `Runtime::Impl` (`runtime/src/runtime/runtime.cpp`).
5. Register it in `start_workers()` and ensure `join_workers()` naturally joins it.
6. Add status-file output fields in `make_status_text()` if you want them in `--status-file`.

### Threading rules to follow

- Never block `control_worker`.
- Keep worker loops periodic (`hz_to_period_ns` + `sleep_until_ns`).
- Use short mutex sections around shared state.
- On recoverable I/O errors: increment counters and continue.
- On fatal errors: set `stop_requested_` and let runtime shutdown cleanly.

## 5) Add More I2C Devices to `i2c_hub_worker`

Current design already supports a job registry in `runtime/src/runtime/runtime.cpp`:

- `struct I2cDeviceJob { name, period_ms, priority, next_due_ns, poll_once }`
- `i2c_hub_worker()` builds a `std::vector<I2cDeviceJob> jobs`

Right now only `baro` is registered. To add a new I2C device:

### Step-by-step

1. Create backend interface + implementation:
   - Extend `runtime/include/runtime/runtime/sensors.hpp` (new backend type if needed).
   - Implement hardware/sim versions in `runtime/src/runtime/sensors.cpp`.
2. Add sample type for produced data in `runtime/include/runtime/common/types.hpp`.
3. Add corresponding fields in `SharedState` (`runtime/src/runtime/runtime.cpp`).
4. Add config section/keys (bus, address, rate, timeouts):
   - `runtime/include/runtime/config/config.hpp`
   - `runtime/src/config/config_loader.cpp`
   - `runtime/config/*.toml`
5. In `i2c_hub_worker()`, push a new `I2cDeviceJob` with its own period and `poll_once`.
6. In the job execution block, add per-device error/recovery counters and update `RuntimeStats`.
7. If needed by external workers, extend `SensorSnapshotMsg` in `runtime/include/runtime/ipc/messages.hpp` and update Python codec/helpers in `runtime/python/ipc_common.py`.

## Igniter Service Notes (Synchronized 4-Channel)

Igniter service uses dedicated IPC messages:

- `IgniterCommandMsg`
- `IgniterStatusMsg`

Command path is intentionally separate from controller mailbox to keep pyrotechnic control isolated.

### GPIO synchronization requirements

For synchronized firing (`fire_mask`/`fire_all`) with low skew:

1. all igniter INPUT lines must be on one gpiochip
2. all igniter STATUS lines must be on one gpiochip
3. each line must be unique
4. runtime config validation enforces these constraints when `[igniter].enabled=true`

### CM5 / RPi5 chip selection

Do not assume `gpiochip0`.

Recommended workflow:

1. `gpiodetect`
2. identify chip label `pinctrl-rp1`
3. `gpioinfo <gpiochipX>`
4. assign `input_chip` / `status_chip` in `igniter0..3` sections accordingly

### Preflight configuration checklist

Before any live igniter test:

1. confirm config validation passes with `[igniter].enabled=true`
2. confirm all 4 channels are enabled and unique line offsets are used
3. confirm `fault_policy`, `settle_ms`, `default_fire_ms`, `max_fire_ms` match test card
4. run `arm` -> `status` -> `disarm` smoke path over IPC
5. validate simultaneous `fire_mask` timing on hardware (logic analyzer target: inter-channel skew `<= 1 ms`)

### Important compatibility note

If you change IPC message structs:

- keep `msg_magic/msg_version/payload_bytes` and CRC handling intact
- update both C++ and Python encode/decode paths
- run IPC tests (`runtime/tests/test_ipc_codec_cpp_to_python.py`, `runtime/tests/test_ipc_codec_python_to_cpp.py`)

## 6) Extending `SensorSnapshotMsg` (Required Steps)

When adding a new sensor field (for example magnetometer, GPS-derived altitude, etc), you must update both runtime and worker codecs.

### C++ updates

1. Add fields to `SensorSnapshotMsg` in `runtime/include/runtime/ipc/messages.hpp` (before `crc32`).
2. Populate those fields where the snapshot is produced (`control_worker` in `runtime/src/runtime/runtime.cpp`).
3. If validation depends on new fields, extend validation in workers that consume snapshots.

### Python updates

1. Update `SENSOR_MSG_STRUCT` layout in `runtime/python/ipc_common.py`.
2. Update `SENSOR_PAYLOAD_BYTES` in `runtime/python/ipc_common.py`.
3. Extend `SensorSnapshot` dataclass and decode helpers in `runtime/python/ipc_common.py`.
4. Update any worker scripts that read snapshot fields (`runtime/python/dummy_estimator.py`, `runtime/python/dummy_controller.py`, and custom workers).

### Build/test updates

1. If using `_runtime_ipc_codec` (pybind), rebuild runtime so the module exports new struct sizes.
2. Run IPC compatibility tests:
   - `runtime/tests/test_ipc_codec_cpp_to_python.py`
   - `runtime/tests/test_ipc_codec_python_to_cpp.py`
   - `runtime/tests/test_python_ipc_codec.py`

## 7) Wire Protocol Versioning (`constants.hpp`)

Versioning source of truth:

- C++: `runtime/include/runtime/common/constants.hpp`
- Python: `runtime/python/ipc_common.py`

Rules:

1. Wire-format breaking change -> bump:
   - `kMessageVersion` in `runtime/include/runtime/common/constants.hpp`
   - `MESSAGE_VERSION` in `runtime/python/ipc_common.py`
2. Keep `kMessageMagic`/`MESSAGE_MAGIC` unchanged unless intentionally creating a new protocol family.
3. Rebuild all C++ binaries and pybind codec module after version/schema changes.
4. Do not run mixed versions (new runtime with old workers, or vice versa).

Examples of breaking changes:

- changing field order, type, alignment, or size in `messages.hpp`
- changing CRC scope or header semantics
- changing payload byte expectations used by decoders

## 8) Schema Change Checklist

Run this checklist whenever you change any IPC message in `runtime/include/runtime/ipc/messages.hpp`.

- [ ] Updated C++ message struct(s) in `runtime/include/runtime/ipc/messages.hpp`.
- [ ] Updated producer/consumer logic in `runtime/src/runtime/runtime.cpp` (and any worker using those fields).
- [ ] Updated Python codec layout/constants in `runtime/python/ipc_common.py`:
  - `*_MSG_STRUCT`
  - `*_PAYLOAD_BYTES`
  - encode/decode helpers + dataclasses
- [ ] Updated C++/Python message versions:
  - `kMessageVersion` in `runtime/include/runtime/common/constants.hpp`
  - `MESSAGE_VERSION` in `runtime/python/ipc_common.py`
- [ ] Rebuilt runtime and codec module.
- [ ] Ran IPC tests:
  - `runtime/tests/test_ipc_codec_cpp_to_python.py`
  - `runtime/tests/test_ipc_codec_python_to_cpp.py`
  - `runtime/tests/test_python_ipc_codec.py`
- [ ] Verified runtime + workers are on matching schema/version (no mixed old/new binaries).

## 9) Implement Your Own Estimator

You have two paths.

### A) Native C++ estimator (`cpp_native`)

Main files:

- `runtime/include/runtime/runtime/estimator.hpp`
- `runtime/src/runtime/estimator.cpp`

Flow:

1. Extend `EstimatorCppSection` config in `runtime/include/runtime/config/config.hpp`.
2. Parse new estimator params in `runtime/src/config/config_loader.cpp`.
3. Implement algorithm in `CppEstimator::step(const ImuSample&, const BaroSample&)`.
4. Keep output valid:
   - finite quaternion
   - quaternion norm near 1
   - monotonic timestamps
5. Tune `estimator_hz` and timeouts in runtime config.

`Runtime::Impl::estimator_worker()` already calls `cpp_estimator_.step(...)`.

### B) External estimator worker (`python_dev` or `cpp_dev`)

Reference implementations:

- Python: `runtime/python/dummy_estimator.py`
- C++: `runtime/src/dummy_estimator.cpp`

Requirements:

1. Read `SensorSnapshotMsg` mailbox.
2. Write `ExternalEstimatorStateMsg` mailbox.
3. Fill header via helpers and finalize CRC.
4. Publish at/above configured estimator rate.
5. Respect freshness window (`timeouts.estimator_fresh_ms` / `estimator_hold_ms`).

Set mode in TOML:

- `[modes] estimator_mode = "python_dev"` or `"cpp_dev"`

## 10) Implement Your Own Controller

Again, two paths.

### A) Native C++ controller (`cpp_native`)

Main files:

- `runtime/include/runtime/runtime/controller.hpp`
- `runtime/src/runtime/controller.cpp`

Flow:

1. Extend `ControllerCppSection` in `runtime/include/runtime/config/config.hpp`.
2. Parse params in `runtime/src/config/config_loader.cpp`.
3. Implement control law in `CppController::step(...)`.
4. Always output `servo_norm` in `[-1.0, 1.0]`.
5. Keep behavior safe on invalid estimator/input (return stable/failsafe-compatible command).

### B) External controller worker (`python_dev` or `cpp_dev`)

Reference implementations:

- Python: `runtime/python/dummy_controller.py`
- C++: `runtime/src/dummy_controller.cpp`

Requirements:

1. Read estimator mailbox and/or sensor mailbox as needed.
2. Write `ExternalControllerCommandMsg`.
3. Set `armed` intentionally.
4. Keep servo commands finite and in range.
5. Publish faster than `timeouts.controller_fresh_ms`.

Set mode in TOML:

- `[modes] controller_mode = "python_dev"` or `"cpp_dev"`

## 11) Main Worker Runner vs `main.cpp`

If you are adding/changing runtime workers, edit:

- `runtime/src/runtime/runtime.cpp`

`runtime/src/main.cpp` normally does not change unless you need new CLI flags or startup behavior.

## 12) Build And Test Before Merging

Use this gate before merge.

1. Configure with tests enabled:
```bash
cmake -S . -B build -DBUILD_RUNTIME=ON -DBUILD_TESTING=ON
```
2. Build:
```bash
cmake --build build -j
```
3. Run runtime test suite:
```bash
ctest --test-dir build --output-on-failure -R runtime
```
4. If you changed IPC schema/codecs, run Python IPC tests explicitly:
```bash
python3 runtime/tests/test_ipc_codec_cpp_to_python.py --module-dir build/runtime
python3 runtime/tests/test_ipc_codec_python_to_cpp.py --module-dir build/runtime
python3 runtime/tests/test_python_ipc_codec.py --backend python --expect-backend python
```
5. Smoke run runtime with config:
```bash
./build/runtime/rt_core --config ./runtime/config/rt_core_cpp_native.toml --print-config
```
6. Check status output (`tools/rt_status_report.py` or `--status-file`) for regressions.

## 13) Minimal Validation Checklist

After changes:

1. Build:
```bash
cmake -S . -B build -DBUILD_RUNTIME=ON
cmake --build build -j
```
2. Run runtime with config:
```bash
./build/runtime/rt_core --config ./runtime/config/rt_core_cpp_native.toml --print-config
```
3. Run tests you touched (at minimum runtime unit tests / IPC codec tests if schema changed).
4. Check status output (`tools/rt_status_report.py` or `--status-file`) for new counters/health signals.
