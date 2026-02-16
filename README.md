# Raspberry Pi IMU + BARO + SERVO + ADC Superbuild

This repository contains:

- `IMU/`: LSM6DS3-family IMU library (I2C + SPI).
- `BARO/`: BMP390 barometer library (I2C).
- `SERVO/`: Hardware PWM servo library (Raspberry Pi 5 Linux).
- `ADC/`: ADS1115 ADC library (I2C).
- `runtime/`: RT core runtime (`rt_core`) with workers, IPC, fallback policies, and tests.

Each subproject can be built independently, or you can build all enabled modules at once from the repo root.

## Build Both (C++)

From the repository root:

```bash
cmake -S . -B build -DLSM6DS3_BUILD_PYTHON=OFF -DBMP390_BUILD_PYTHON=OFF -DSERVO_BUILD_PYTHON=OFF -DADS1115_BUILD_PYTHON=OFF
cmake --build build -j
```

Executables will be in `build/`, for example:

- `build/lsm6ds3_read_once`
- `build/lsm6ds3_read_once_spi`
- `build/bmp390_read_once`
- `build/servo_set_pulse`
- `build/ads1115_read_once`

## Build Runtime

Runtime on Linux/Raspberry Pi with hardware modules enabled:

```bash
cmake -S . -B build \
  -DBUILD_RUNTIME=ON \
  -DLSM6DS3_BUILD_PYTHON=OFF \
  -DBMP390_BUILD_PYTHON=OFF \
  -DSERVO_BUILD_PYTHON=OFF \
  -DADS1115_BUILD_PYTHON=OFF
cmake --build build -j
```

Runtime portable/sim build (no IMU/BARO compile dependency):

```bash
cmake -S . -B build \
  -DBUILD_RUNTIME=ON \
  -DBUILD_IMU=OFF \
  -DBUILD_BARO=OFF \
  -DBUILD_SERVO=ON \
  -DLSM6DS3_BUILD_PYTHON=OFF \
  -DBMP390_BUILD_PYTHON=OFF \
  -DSERVO_BUILD_PYTHON=OFF \
  -DADS1115_BUILD_PYTHON=OFF
cmake --build build -j
```

Runtime executable:

- `build/runtime/rt_core`
- `build/runtime/dummy_estimator_cpp`
- `build/runtime/dummy_controller_cpp`

Runtime development configs:

- `runtime/config/rt_core_sim_python_dev.toml` (Python dummy workers)
- `runtime/config/rt_core_sim_cpp_dev.toml` (C++ dummy workers)

## Build Only One

IMU only:

```bash
cmake -S . -B build -DBUILD_IMU=ON -DBUILD_BARO=OFF
cmake --build build -j
```

BARO only:

```bash
cmake -S . -B build -DBUILD_IMU=OFF -DBUILD_BARO=ON
cmake --build build -j
```

SERVO only:

```bash
cmake -S . -B build -DBUILD_IMU=OFF -DBUILD_BARO=OFF -DBUILD_SERVO=ON
cmake --build build -j
```

ADC only:

```bash
cmake -S . -B build -DBUILD_IMU=OFF -DBUILD_BARO=OFF -DBUILD_SERVO=OFF -DBUILD_ADC=ON
cmake --build build -j
```

## Python Modules (Optional)

If you want the Python modules at the root build, enable them explicitly and make sure `pybind11` is available in your environment:

```bash
cmake -S . -B build -DLSM6DS3_BUILD_PYTHON=ON -DBMP390_BUILD_PYTHON=ON -DSERVO_BUILD_PYTHON=ON -DADS1115_BUILD_PYTHON=ON
cmake --build build -j
```

## Python Packages (PEP 668-safe on Raspberry Pi OS)

You can use a single venv at the repo root, then build wheels in each subproject:

```bash
python3.13 -m venv .venv
. .venv/bin/activate
python -m pip install -U pip build

cd IMU
python -m build
python -m pip install dist/lsm6ds3_rpi-*.whl

cd ../BARO
python -m build
python -m pip install dist/bmp390_rpi-*.whl

cd ../ADC
python -m build
python -m pip install dist/ADS1115_rpi-*.whl
```

If you want to interact with the runtime helpers (including `rt-status-report`), install the repo in editable mode before running those tools:

```bash
python3 -m pip install -e .
```

The editable install also exposes the dummy worker scripts that mirror your C++ runtime modules. Once installed you can run:

```bash
dummy-controller --config runtime/config/rt_config.toml
dummy-estimator --config runtime/config/rt_config.toml
```

Each script accepts `--duration-sec` if you want to stop after a fixed runtime, and the configuration file is the same TOML that the C++ runtime uses for IPC paths/hz values.

### One-Command Root Build (emits all wheels)

The root `pyproject.toml` includes a custom build hook that runs `python -m build` in `IMU/`, `BARO/`, `SERVO/`, and `ADC/` and places all wheels into the root `dist/` folder. It also creates a small `rpi_sensors` wheel (metadata only).

```bash
python3.13 -m venv .venv
. .venv/bin/activate
python -m pip install -U pip build
python -m build
```

Outputs in `dist/`:

- `rpi_sensors-*.whl`
- `lsm6ds3_rpi-*.whl`
- `bmp390_rpi-*.whl`
- `servo_rpi-*.whl`
- `ADS1115_rpi-*.whl`

For more Python packaging and runtime usage details, see:

- `IMU/README.md`
- `BARO/README.md`
- `SERVO/README.md`
- `ADC/README.md`
- `runtime/README.md`
