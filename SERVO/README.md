# servo_rpi

Linux-first C++17 and Python 3.13 library for hardware PWM servo output on Raspberry Pi 5.

## Features

- Native C++17 implementation (not a Python port) in `namespace servo`.
- Linux sysfs PWM backend (`/sys/class/pwm/pwmchipN/pwmM`) with bounded retries.
- Thread-safe API for intra-process use (`std::mutex` protected).
- Optional cross-process channel lock (auto path: `/run/lock`, `/var/lock`, `/dev/shm`, `/tmp`) to prevent accidental channel sharing.
- High-level `Servo` class with angle-to-pulse mapping and configurable limits.
- pybind11 bindings packaged as `servo_rpi` for Python 3.13.

## Raspberry Pi 5 setup

1. Build and install the 4-channel PWM overlay (from your `rpi_config` flow):
   ```bash
   sudo apt update
   sudo apt install -y device-tree-compiler
   dtc -@ -I dts -O dtb -o ./pwm_overlay/pwm-4chan.dtbo ./pwm_overlay/pwm-4chan.dts
   sudo cp ./pwm_overlay/pwm-4chan.dtbo /boot/firmware/overlays/
   ```
2. Enable overlay in `/boot/firmware/config.txt`:
   ```ini
   dtparam=audio=off
   dtoverlay=pwm-4chan
   ```
3. Reboot and confirm PWM sysfs nodes:
   ```bash
   ls /sys/class/pwm/pwmchip*
   for c in /sys/class/pwm/pwmchip*; do echo "$c npwm=$(cat "$c/npwm")"; done
   ```
4. Ensure your user can write PWM sysfs files (typically root or udev rule).
   If you get `Permission denied` on `/sys/class/pwm/...`, run commands as root:
   ```bash
   sudo "$(command -v servo-pwm-set)" 12 20000000 1500000
   sudo "$(command -v servo-pwm-multi-cycle)"
   ```

## Run Without Root (Recommended)

Do a one-time permission setup, then run normal commands without `sudo`.

1. Add your user to `gpio` group:
   ```bash
   sudo usermod -aG gpio "$USER"
   ```
2. Install udev rule:
   ```bash
   sudo cp ./rpi_config/pwm/60-servo-pwm.rules /etc/udev/rules.d/
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   ```
3. Re-login (or reboot).
4. Runtime `pinctrl` is disabled by default (recommended for non-root when overlay already configures mux).
   Enable it only if you explicitly need runtime mux switching:
   ```bash
   export SERVO_PWM_PINCTRL=1
   ```

Pin/function mapping used by included examples:

- GPIO12 -> PWM channel 0 (`a0`)
- GPIO13 -> PWM channel 1 (`a0`)
- GPIO18 -> PWM channel 2 (`a3`)
- GPIO19 -> PWM channel 3 (`a3`)

## Build (C++)

```bash
cmake -S . -B build -DSERVO_BUILD_PYTHON=OFF
cmake --build build -j
./build/servo_set_pulse
```

## PWM Python Examples

Two Python examples using `servo_rpi` bindings are included:

- `examples/pwmset.py`: set one GPIO PWM channel (`<pin> <period_ns|off> <duty_ns>`).
- `examples/pwm_multi_cycle.py`: drive GPIO 12/13/18/19 with a repeating duty sequence.

Install `servo_rpi` wheel first (or run from an environment where `servo_rpi` is already installed).

Installed console scripts:

- `servo-pwm-set`
- `servo-pwm-multi-cycle`

Note: these CLI tools keep PWM enabled after each write so servo pulses continue between updates.
By default, channel lock files are created in the first writable directory from:
`/run/lock`, `/var/lock`, `/dev/shm`, `/tmp`.
You can force a lock directory with:
```bash
export SERVO_PWM_LOCK_DIR=/run/lock
```

Run:

```bash
# One channel
servo-pwm-set 12 20000000 1500000

# Stop one channel
servo-pwm-set 12 off

# Multi-channel loop
servo-pwm-multi-cycle

# If lock-file creation fails (for example /tmp policy), disable lock file:
servo-pwm-set --no-lock 12 20000000 1500000
servo-pwm-multi-cycle --no-lock

# Runtime pinctrl is OFF by default. Enable only when needed:
servo-pwm-set --pinctrl 12 20000000 1500000
servo-pwm-multi-cycle --pinctrl

# Or run wrappers from examples/
python3 examples/pwmset.py 12 20000000 1500000
python3 examples/pwm_multi_cycle.py
```

## Build wheel (Python 3.13)

```bash
python3.13 -m venv .venv
. .venv/bin/activate
python -m pip install -U pip build
python -m build
python -m pip install dist/servo_rpi-*.whl
```

## Python usage

```python
from servo_rpi import Servo

s = Servo(chip=0, channel=0)
s.begin()
s.set_angle_deg(15.0)
s.set_pulse_width_us(1600)
s.close()
```

## Notes for parallel runtime

- Use one `Servo`/`HardwarePwm` instance per hardware PWM channel.
- Do not share the same channel across independent workers unless intentionally serialized.
- Keep actuator writes in a dedicated control thread to align with the main runtime plan.
