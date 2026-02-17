# BUZZER (`buzzer_rpi`)

Linux-first C++17 and Python 3.13 buzzer package for LPB2418A-style ON/OFF signaling on Raspberry Pi.

Features:

- `libgpiod` backend (`/dev/gpiochipN`) with safe OFF behavior on `begin()` and `close()`.
- Active-high/active-low polarity support.
- Optional watchdog (`max_on_time_ms`) that forces OFF after the configured ON window.
- Non-blocking scheduler with priorities, preemption, busy policies (`QUEUE`, `REPLACE`, `DROP_IF_BUSY`), and per-token throttling.
- Drone event/alarm API (`notify`, `set_alarm`) with latched alarm handling.
- JSON profile loader in C++; YAML support in Python CLI (`PyYAML`) via dict mapping.

## Build (C++)

From repo root:

```bash
cmake -S . -B build -DBUILD_BUZZER=ON -DBUZZER_BUILD_PYTHON=OFF
cmake --build build -j
```

Standalone:

```bash
cmake -S BUZZER -B build_buzzer -DBUZZER_BUILD_PYTHON=OFF
cmake --build build_buzzer -j
```

## Python package

```bash
cd BUZZER
python -m build
python -m pip install dist/buzzer_rpi-*.whl
```

Optional YAML support:

```bash
python -m pip install "buzzer_rpi[yaml]"
```

## CLI

- `buzzer-play`
- `buzzer-signal`
- `buzzer-profile-lint`

Examples:

```bash
buzzer-play --pattern "on:120,off:120,on:900" --line 18 --chip /dev/gpiochip0
buzzer-signal --notify READY_TO_ARM --profile BUZZER/profiles/ardupilot_like_v1.json
buzzer-signal --alarm LOST_VEHICLE --duration-s 5 --profile BUZZER/profiles/ardupilot_like_v1.json
buzzer-profile-lint BUZZER/profiles/ardupilot_like_v1.json --print-json
```

## Default profile

Default profile file:

- `BUZZER/profiles/ardupilot_like_v1.json`

Contains event and latched alarm patterns for:

- `ARMING`, `ARMING_FAILURE`, `DISARMED`, `GYRO_INIT_DONE`, `READY_TO_ARM`
- `BATTERY_FAILSAFE`, `EKF_FAILURE`, `LOST_VEHICLE`, `EVACUATION_TEMPORAL3`, `MISSING_SOS`

## Hardware note

LPB2418A must be driven through an appropriate transistor/driver stage if GPIO current limits require it. The package controls logic level only.
