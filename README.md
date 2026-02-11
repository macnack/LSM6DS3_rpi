# lsm6ds3_rpi

Linux-first C++17 and Python 3.13 library for ST LSM6DS3 over I2C on Raspberry Pi 5.

## Features

- I2C-only driver using `ioctl(I2C_RDWR)` combined transactions.
- WHO_AM_I validation (`0x0F == 0x69` for LSM6DS3 or `0x6B` for LSM6DSR) during startup.
- Default bus `/dev/i2c-1`, default address `0x6A`.
- Burst reads (6 bytes) for accel and gyro with register auto-increment.
- Thread-safe Linux transport with retries for transient I/O failures.
- Python bindings via pybind11 and scikit-build-core.

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

```bash
lsm6ds3-stream --bus /dev/i2c-1 --address 0x6A --hz 20
```

Output format:

```text
timestamp,ax,ay,az,gx,gy,gz
```

- `a*` in m/s²
- `g*` in rad/s

## C++ quick sketch

```cpp
#include "lsm6ds3/lsm6ds3.hpp"

int main() {
  lsm6ds3::Lsm6ds3 imu;
  imu.begin();
  auto accel = imu.read_accel_si();
  auto gyro = imu.read_gyro_si();
  imu.close();
  return 0;
}
```
