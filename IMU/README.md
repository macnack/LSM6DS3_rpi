# lsm6ds3_rpi

Linux-first C++17 and Python 3.13 library for ST LSM6DS3-family IMUs on Raspberry Pi 5 over I2C or SPI.

## Features

- Linux I2C transport using `ioctl(I2C_RDWR)` combined transactions.
- Linux SPI transport using `ioctl(SPI_IOC_MESSAGE)` full-duplex transactions (`mode=3` default).
- WHO_AM_I validation (`0x69` LSM6DS3, `0x6A` ISM330DLC, `0x6B` LSM6DSR).
- Default I2C bus `/dev/i2c-1` and default SPI device `/dev/spidev0.0`.
- Burst reads (6 bytes) for accel and gyro with register auto-increment.
- Thread-safe transports with retries for transient I/O failures.
- Python bindings exposing both `Lsm6ds3` (I2C) and `Lsm6ds3Spi` (SPI).

## Raspberry Pi I2C setup

1. Enable I2C:
   ```bash
   sudo raspi-config nonint do_i2c 0
   sudo reboot
   ```
2. Verify bus/device:
   ```bash
   ls /dev/i2c-1
   sudo i2cdetect -y 1
   ```
3. Typical wiring:
   - `3V3` -> LSM6DS3 `VDD`
   - `GND` -> `GND`
   - `GPIO2/SDA1` -> `SDA`
   - `GPIO3/SCL1` -> `SCL`
   - `SA0` low => `0x6A`, high => `0x6B`

## Raspberry Pi SPI setup

1. Enable SPI:
   ```bash
   sudo raspi-config nonint do_spi 0
   sudo reboot
   ```
2. Verify bus/device:
   ```bash
   ls /dev/spidev0.0
   ```
3. Typical 4-wire wiring:
   - `3V3` -> `VDD`
   - `GND` -> `GND`
   - `GPIO11/SCLK` -> `SCL/SPC`
   - `GPIO10/MOSI` -> `SDA/SDI`
   - `GPIO9/MISO` -> `SDO/SA0`
   - `GPIO8/CE0` -> `CS`

## Build Python package (PEP 668-safe on Raspberry Pi OS)

Raspberry Pi OS marks system Python as externally managed, so use a virtual environment:

```bash
python3.13 -m venv .venv
. .venv/bin/activate
python -m pip install -U pip build
python -m build
python -m pip install dist/lsm6ds3_rpi-*.whl
```

If you intentionally install into system Python, you must pass `--break-system-packages` (not recommended).

## Run CLI streamer

I2C:

```bash
lsm6ds3-stream --interface i2c --bus /dev/i2c-1 --address 0x6A --hz 20
```

SPI:

```bash
lsm6ds3-stream --interface spi --spi-device /dev/spidev0.0 --spi-speed 5000000 --spi-mode 3 --hz 20
```

Output format:

```text
timestamp,ax,ay,az,gx,gy,gz
```

- `a*` in m/s²
- `g*` in rad/s

## C++ example (robust, one-shot read)

A complete I2C example is available at `examples/read_once.cpp`.
A complete SPI example is available at `examples/read_once_spi.cpp`.

Build and run it after building the static library:

```bash
cmake -S . -B build -DLSM6DS3_BUILD_PYTHON=OFF
cmake --build build -j
./build/lsm6ds3_read_once
./build/lsm6ds3_read_once_spi
```

It prints one accel sample (m/s²) and one gyro sample (rad/s).
