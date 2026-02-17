# IGNITER

Synchronized 4-channel igniter primitives for Raspberry Pi runtimes.

Main components:

- `VN5E160S`: low-level fault filtering and latch behavior
- `Igniter`: non-blocking arm/fire/off state machine
- `IgniterBank`: coordinated 4-channel control with batch GPIO writes

The hardware backend uses libgpiod when available at build time.

## Developer Notes (Production GPIO)

Implementacja produkcyjna: libgpiod request linii na jednym chipie.

To keep multi-channel firing synchronized (`fire_mask` / `fire_all`), all four INPUT lines must be
requested in one batch from a single `gpiochip`.

Recommended validation workflow on target device (CM5/RPi5):

1. List available GPIO chips:

```bash
gpiodetect
```

2. Pick user GPIO chip by label (usually `pinctrl-rp1` on CM5/RPi5).

3. Inspect available lines on selected chip:

```bash
gpioinfo gpiochipX
```

4. Verify each candidate line offset exists on the same chip:

```bash
CHIP=gpiochipX
for L in 17 18 27 22 5 6 13 19; do
  gpioinfo "${CHIP}" "${L}"
done
```

If all commands succeed, all checked offsets belong to the same chip. If any fails, the line is not
available on that chip.

5. Use the same chip path in igniter config for all channels:

```toml
[igniter0]
input_chip = "/dev/gpiochipX"
status_chip = "/dev/gpiochipX"

[igniter1]
input_chip = "/dev/gpiochipX"
status_chip = "/dev/gpiochipX"

[igniter2]
input_chip = "/dev/gpiochipX"
status_chip = "/dev/gpiochipX"

[igniter3]
input_chip = "/dev/gpiochipX"
status_chip = "/dev/gpiochipX"
```

Do not hardcode `gpiochip0`; always resolve by chip label on target OS image.

### TOML checker tool

Package includes a helper script that validates runtime TOML chip consistency:

- all `input_chip` values on one chip
- all `status_chip` values on one chip
- `input_chip` and `status_chip` on the same chip
- optional CM5/RPi5 check: selected chip label is `pinctrl-rp1`

Run:

```bash
igniter-gpiochip-check /path/to/runtime.toml
```

Template file:

```bash
IGNITER/tools/igniter_runtime_template.toml
```

Without package install (from repo root):

```bash
python3 IGNITER/tools/igniter_gpiochip_check.py /path/to/runtime.toml
```

Useful options:

```bash
# if you want to skip label check (still validates chip consistency)
igniter-gpiochip-check /path/to/runtime.toml --no-require-rp1

# non-standard section prefix or channel count
igniter-gpiochip-check /path/to/runtime.toml --channel-prefix igniter --channels 4
```

## C++ examples

Built executables:

- `igniter_vn5e160s_demo`
- `igniter_state_machine_demo`
- `igniter_bank_demo`

Run from repo root after build:

```bash
./build/IGNITER/igniter_vn5e160s_demo
./build/IGNITER/igniter_state_machine_demo
./build/IGNITER/igniter_bank_demo
```

## Python CLI

`igniter_rpi` exposes one CLI with subcommands for all three classes:

- `vn5e160s`
- `igniter`
- `bank`

Examples:

```bash
igniter-demo vn5e160s --status-seq 1,1,1,0,1 --step-ms 5
igniter-demo igniter --duration-ms 200 --status-seq 1,1,1,1,1,1 --step-ms 40
igniter-demo bank --mask 0x0F --duration-ms 250 --steps 12 --step-ms 30
igniter-demo bank --fault-channel 2 --fault-step 5
```
