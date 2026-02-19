# ms4525do_rpi

Linux-first C++17 and Python 3.13 library for TE `DS_MS4525DO` airspeed sensors over I2C on Raspberry Pi 5.

## Features

- Linux I2C transport using `ioctl(I2C_RDWR)` with repeated retries for transient bus errors.
- MS4525DO 4-byte frame decode:
  - status bits (`NORMAL`, `COMMAND_MODE`, `STALE_DATA`, `DIAGNOSTIC_FAULT`)
  - 14-bit pressure counts
  - 11-bit temperature counts
- Read policy:
  - default `REQUIRE_FRESH` retries stale frames (default: 3 retries, 2 ms delay)
  - raises stale-data timeout if fresh data is not observed
  - optional `ALLOW_STALE` for diagnostic workflows
- Transfer function scaling for both output variants:
  - Type A: 10%..90% counts (`1638..14746`)
  - Type B: 5%..95% counts (`819..15563`)
- Thread-safe C++ driver and pybind11 bindings packaged as `ms4525do_rpi`.

## Build (C++)

```bash
cmake -S . -B build -DMS4525DO_BUILD_PYTHON=OFF
cmake --build build -j
./build/ms4525do_read_once
```

## Build wheel (Python 3.13)

```bash
python3.13 -m venv .venv
. .venv/bin/activate
python -m pip install -U pip build
python -m build
python -m pip install dist/ms4525do_rpi-*.whl
```

## Python usage

```python
from ms4525do_rpi import Calibration, Ms4525do, OutputType, ReadPolicy

cal = Calibration()
cal.p_min_psi = -1.0
cal.p_max_psi = 1.0
cal.output_type = OutputType.TYPE_B_5_TO_95

sensor = Ms4525do(bus_path="/dev/i2c-1", address=0x28, calibration=cal)
sensor.begin()
reading = sensor.read(policy=ReadPolicy.REQUIRE_FRESH)
print(reading.pressure_pa, reading.temperature_c, reading.status)
sensor.close()
```

## Run CLI streamer

```bash
ms4525do-stream --bus /dev/i2c-1 --address 0x28 --hz 20 --p-min-psi -1 --p-max-psi 1 --output-type b
```

CSV output columns:
- `timestamp`: Unix epoch seconds (float)
- `pressure_psi`: Differential pressure in PSI
- `pressure_pa`: Differential pressure in Pascal
- `temperature_c`: Sensor temperature in Celsius
- `pressure_counts`: Raw 14-bit pressure counts
- `temperature_counts`: Raw 11-bit temperature counts
- `status`: Raw status code (`0..3`)

## Run Airspeed Calibration CLI

Generic command for future multi-sensor support (currently `ms4525do`):

```bash
airspeed-calibrate --sensor ms4525do --bus /dev/i2c-1 --address 0x28 --zero-samples 300 --verify-threshold-pa 50
```

The calibration CLI persists reverse-state bi-directionally:
- success -> `reverse_ports: false`
- negative direction -> `reverse_ports: true`

If negative pressure direction is detected, the CLI logs failure details and persists a reverse-flag state file:

```bash
airspeed-calibrate --sensor ms4525do --reverse-state-file output/airspeed_reverse_state.json
```

Machine-readable output:

```bash
airspeed-calibrate --sensor ms4525do --non-interactive --json
```

Robust statistical mode (`--experimental`) enables outlier-resistant zero-offset estimation and rolling verification:

```bash
airspeed-calibrate --sensor ms4525do --experimental --non-interactive --json
```

By default calibration accepts only `Status.NORMAL` samples. For diagnostics you can opt in to stale samples:

```bash
airspeed-calibrate --sensor ms4525do --allow-stale
```
