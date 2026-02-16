# ADS1115_rpi

Linux-first C++17 and Python 3.13 library for ADS1115 over I2C on Raspberry Pi 5.

## Features

- Linux I2C transport with `ioctl(I2C_RDWR)` and repeated-start register reads.
- Default bus `/dev/i2c-1`, default ADS1115 address `0x48`.
- Thread-safe transactions with bounded retries for transient errors.
- RAII file descriptor lifecycle and descriptive exception messages.
- pybind11 bindings packaged with scikit-build-core as `ADS1115_rpi`.
- Compatibility wrapper `ADS1115` with DFRobot-style methods:
  `set_gain()`, `read_voltage()`, `comparator_voltage()`.

## Build (C++)

```bash
cmake -S . -B build -DADS1115_BUILD_PYTHON=OFF
cmake --build build -j
./build/ads1115_read_once
```

## Build wheel (Python 3.13)

```bash
python3.13 -m venv .venv
. .venv/bin/activate
python -m pip install -U pip build
python -m build
python -m pip install dist/ADS1115_rpi-*.whl
```

## Python usage

```python
from ADS1115_rpi import Ads1115

adc = Ads1115(bus_path="/dev/i2c-1", address=0x48)
adc.begin()
value = adc.read_adc(0)
print(value)
adc.close()
```

## New API snippets (mode, DR, mux, latest/non-blocking)

C++:

```cpp
#include "ads1115/ads1115.hpp"

#include <iostream>

int main() {
  ads1115::Ads1115 adc("/dev/i2c-1", 0x48);
  adc.begin();

  // Continuous + non-blocking latest reads.
  adc.set_mode(ads1115::Ads1115::Mode::kContinuous);
  adc.set_data_rate(ads1115::Ads1115::DataRate::k64Sps);
  adc.set_mux(ads1115::Ads1115::Mux::kSingleAin0);

  const int16_t raw = adc.read_latest();
  const double volts = adc.read_latest_voltage();
  std::cout << "raw=" << raw << " volts=" << volts << "\n";

  // Poll loop with callback. Return false to stop.
  adc.poll_continuous(
      [](int16_t sample) {
        std::cout << "sample=" << sample << "\n";
        return true;
      },
      10,   // poll interval ms
      100); // max samples (0 = unlimited)

  adc.close();
  return 0;
}
```

Python (direct bindings):

```python
from ADS1115_rpi import Ads1115, Mode, DataRate, Mux

adc = Ads1115("/dev/i2c-1", 0x48)
adc.begin()
adc.set_mode(Mode.CONTINUOUS)
adc.set_data_rate(DataRate.SPS_64)
adc.set_mux(Mux.SINGLE_AIN0)

print(adc.read_latest())          # raw int16
print(adc.read_latest_voltage())  # volts

def on_sample(raw: int) -> bool:
    print(raw)
    return True

adc.poll_continuous(on_sample, poll_interval_ms=10, max_samples=20)
adc.close()
```

Python (DFRobot-style wrapper):

```python
from ADS1115_rpi import (
    ADS1115,
    ADS1115_REG_CONFIG_DR_64SPS,
)

adc = ADS1115()
adc.begin()
adc.set_mode("continuous")
adc.set_data_rate(ADS1115_REG_CONFIG_DR_64SPS)
print(adc.read_latest())  # {"r": <millivolts>}

adc.stream_continuous(lambda raw: True, poll_interval_ms=10, max_samples=20)
adc.close()
```

Compatibility usage:

```python
from ADS1115_rpi import ADS1115, ADS1115_REG_CONFIG_PGA_6_144V

adc = ADS1115()
adc.set_gain(ADS1115_REG_CONFIG_PGA_6_144V)
print(adc.read_voltage(0))        # {"r": <millivolts>}
print(adc.comparator_voltage(0))  # {"r": <millivolts differential>}
adc.close()
```

## Run CLI examples

```bash
# Single-device voltage example (legacy demo_read_voltage style)
ads1115-stream --example read-voltage --address 0x48 --gain 0x00 --hz 2

# Two-device cascade example (legacy demo_read_voltage_cascade style)
ads1115-stream --example read-voltage-cascade --address 0x48 --address2 0x49 --gain 0x00 --hz 1

# Differential/comparator example (legacy demo_comparator_voltage style)
ads1115-stream --example comparator-voltage --address 0x48 --gain 0x00 --hz 1
```
