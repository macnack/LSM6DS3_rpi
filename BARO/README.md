# bmp390_rpi

Linux-first C++17 and Python 3.13 library for Bosch BMP390 over I2C on Raspberry Pi 5.

## Features

- I2C-only transport with `ioctl(I2C_RDWR)` combined transactions and repeated-start reads.
- Default bus `/dev/i2c-1`, default BMP390 I2C address `0x77`.
- Bosch Sensortec BMP3 Sensor API integration (`bmp3.c/.h/.defs.h`) with minimal wrapper code.
- Thread-safe transport and driver (`std::mutex`) with bounded retries for transient failures.
- RAII file descriptor lifecycle and descriptive exception messages.
- pybind11 bindings packaged with scikit-build-core as `bmp390_rpi`.

## Build (C++)

```bash
cmake -S . -B build -DBMP390_BUILD_PYTHON=OFF
cmake --build build -j
./build/bmp390_read_once
```

## Build wheel (Python 3.13)

```bash
python3.13 -m venv .venv
. .venv/bin/activate
python -m pip install -U pip build
python -m build
python -m pip install dist/bmp390_rpi-*.whl
```

## Python usage

```python
from bmp390_rpi import Bmp390

sensor = Bmp390(bus_path="/dev/i2c-1", address=0x77)
sensor.begin()
reading = sensor.read()
print(reading.temperature_c, reading.pressure_pa)
sensor.close()
```

## Run CLI streamer

```bash
bmp390-stream --bus /dev/i2c-1 --address 0x77 --hz 20
```

CSV output columns:
- `timestamp`: Unix epoch seconds (float)
- `temperature_c`: Temperature in Celsius
- `pressure_pa`: Pressure in Pascal
- `pressure_hpa`: Pressure in hectopascal
