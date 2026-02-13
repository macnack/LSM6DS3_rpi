# Raspberry Pi IMU + BARO Superbuild

This repository contains:

- `IMU/`: LSM6DS3-family IMU library (I2C + SPI).
- `BARO/`: BMP390 barometer library (I2C).

Each subproject can be built independently, or you can build both at once from the repo root.

## Build Both (C++)

From the repository root:

```bash
cmake -S . -B build -DLSM6DS3_BUILD_PYTHON=OFF -DBMP390_BUILD_PYTHON=OFF
cmake --build build -j
```

Executables will be in `build/`, for example:

- `build/lsm6ds3_read_once`
- `build/lsm6ds3_read_once_spi`
- `build/bmp390_read_once`

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

## Python Modules (Optional)

If you want the Python modules at the root build, enable them explicitly and make sure `pybind11` is available in your environment:

```bash
cmake -S . -B build -DLSM6DS3_BUILD_PYTHON=ON -DBMP390_BUILD_PYTHON=ON
cmake --build build -j
```

## Python Packages (PEP 668-safe on Raspberry Pi OS)

The PEP 668-safe wheel builds are handled per subproject. Use a venv and run the build in each folder:

```bash
cd IMU
python3.13 -m venv .venv
. .venv/bin/activate
python -m pip install -U pip build
python -m build
python -m pip install dist/lsm6ds3_rpi-*.whl

cd ../BARO
python3.13 -m venv .venv
. .venv/bin/activate
python -m pip install -U pip build
python -m build
python -m pip install dist/bmp390_rpi-*.whl
```

For more Python packaging and runtime usage details, see:

- `IMU/README.md`
- `BARO/README.md`
