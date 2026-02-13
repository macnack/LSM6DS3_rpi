"""CLI tools for servo_rpi PWM control."""

from __future__ import annotations

import argparse
import glob
import os
import re
import signal
import subprocess
import sys
import time
from dataclasses import dataclass

from . import HardwarePwm, HardwarePwmConfig


@dataclass(frozen=True)
class PinMap:
    channel: int
    alt_func: str


PIN_MAP: dict[int, PinMap] = {
    12: PinMap(channel=0, alt_func="a0"),
    13: PinMap(channel=1, alt_func="a0"),
    18: PinMap(channel=2, alt_func="a3"),
    19: PinMap(channel=3, alt_func="a3"),
}

PERIOD_DEFAULT = 20_000_000
GPIOS = (12, 13, 18, 19)
DUTIES_DEFAULT = (2_000_000, 1_500_000, 1_000_000, 1_500_000)


def first_pwmchip_path() -> str:
    matches = sorted(glob.glob("/sys/class/pwm/pwmchip*"))
    if not matches:
        raise RuntimeError("No pwmchip found.")
    return matches[0]


def ensure_pwm_permissions() -> None:
    chip_path = first_pwmchip_path()
    export_path = os.path.join(chip_path, "export")
    if not os.access(export_path, os.W_OK):
        raise PermissionError(
            f"No write permission for '{export_path}'. "
            "Run as root or install udev rules for PWM sysfs access."
        )


def parse_chip_index(chip_path: str) -> int:
    match = re.search(r"pwmchip(\d+)$", chip_path)
    if not match:
        raise RuntimeError(f"Unexpected pwmchip path: {chip_path}")
    return int(match.group(1))


def run_pinctrl(pin: int, func: str) -> None:
    subprocess.run(["pinctrl", "set", str(pin), func], check=True)


def unexport_channel(chip_path: str, channel: int) -> None:
    unexport_path = os.path.join(chip_path, "unexport")
    with open(unexport_path, "w", encoding="ascii") as fh:
        fh.write(str(channel))


def disable_pwm(pin: int, use_lock: bool = True) -> None:
    mapping = PIN_MAP[pin]
    chip_path = first_pwmchip_path()
    chip_index = parse_chip_index(chip_path)

    cfg = HardwarePwmConfig()
    cfg.chip = chip_index
    cfg.channel = mapping.channel
    cfg.enabled_on_begin = False
    cfg.unexport_on_close = False
    cfg.use_channel_lock = use_lock

    pwm = HardwarePwm(cfg)
    pwm.begin()
    pwm.set_enabled(False)
    pwm.close()

    run_pinctrl(pin, "no")

    channel_path = os.path.join(chip_path, f"pwm{mapping.channel}")
    if os.path.isdir(channel_path):
        try:
            unexport_channel(chip_path, mapping.channel)
        except OSError:
            pass


def set_pwm(pin: int, period_ns: int, duty_ns: int, use_lock: bool = True) -> None:
    if period_ns <= 0 or duty_ns < 0:
        raise ValueError("period_ns must be > 0 and duty_ns must be >= 0")
    if duty_ns > period_ns:
        raise ValueError("duty_ns must be <= period_ns")

    mapping = PIN_MAP[pin]
    chip_path = first_pwmchip_path()
    chip_index = parse_chip_index(chip_path)

    run_pinctrl(pin, mapping.alt_func)

    cfg = HardwarePwmConfig()
    cfg.chip = chip_index
    cfg.channel = mapping.channel
    cfg.period_ns = period_ns
    cfg.duty_cycle_ns = duty_ns
    cfg.enabled_on_begin = True
    cfg.unexport_on_close = False
    cfg.use_channel_lock = use_lock

    pwm = HardwarePwm(cfg)
    pwm.begin()
    pwm.set_duty_cycle_ns(duty_ns)
    pwm.set_enabled(True)

    print(
        f"OK: {chip_path} GPIO{pin} -> pwm{mapping.channel} "
        f"({mapping.alt_func}) period={period_ns} duty={duty_ns}"
    )


def _parse_pwmset_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog="servo-pwm-set",
        description="Set one PWM output: servo-pwm-set <pin> <period_ns|off> [duty_ns]",
    )
    parser.add_argument("pin", type=int, choices=sorted(PIN_MAP.keys()))
    parser.add_argument("period", help="PWM period in ns (example: 20000000) or 'off'")
    parser.add_argument("duty", nargs="?", help="PWM duty cycle in ns (example: 1500000)")
    parser.add_argument(
        "--no-lock",
        action="store_true",
        help="Disable per-channel lock file (use only if lock file path is not writable).",
    )
    return parser.parse_args(argv)


def _format_error(exc: Exception) -> str:
    text = str(exc)
    if isinstance(exc, PermissionError) or "Permission denied" in text or "errno=13" in text:
        return (
            f"{text}\n"
            "Hint: run with root privileges, for example:\n"
            "  sudo \"$(command -v servo-pwm-set)\" 12 20000000 1500000\n"
            "  sudo \"$(command -v servo-pwm-multi-cycle)\"\n"
            "Or configure udev rules to allow PWM sysfs writes without sudo."
        )
    return text


def pwmset_main(argv: list[str] | None = None) -> int:
    args = _parse_pwmset_args(argv)
    try:
        ensure_pwm_permissions()
        if args.period == "off":
            disable_pwm(args.pin, use_lock=not args.no_lock)
            return 0

        if args.duty is None:
            raise ValueError("duty_ns is required unless period is 'off'")

        period_ns = int(args.period)
        duty_ns = int(args.duty)
        set_pwm(args.pin, period_ns, duty_ns, use_lock=not args.no_lock)
        return 0
    except Exception as exc:
        print(f"ERROR: {_format_error(exc)}", file=sys.stderr)
        return 1


def _parse_multi_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog="servo-pwm-multi-cycle")
    parser.add_argument("--period", type=int, default=PERIOD_DEFAULT)
    parser.add_argument("--wait-s", type=float, default=1.0)
    parser.add_argument(
        "--cycles",
        type=int,
        default=0,
        help="0 means infinite loop, >0 limits the number of full duty cycles.",
    )
    parser.add_argument(
        "--duties",
        type=int,
        nargs="+",
        default=list(DUTIES_DEFAULT),
        help="Duty sequence in ns.",
    )
    parser.add_argument(
        "--no-lock",
        action="store_true",
        help="Disable per-channel lock file (use only if lock file path is not writable).",
    )
    return parser.parse_args(argv)


def _cleanup_all(use_lock: bool) -> None:
    print("Disabling PWM outputs...")
    for gpio in GPIOS:
        try:
            disable_pwm(gpio, use_lock=use_lock)
        except Exception:
            pass


def pwm_multi_cycle_main(argv: list[str] | None = None) -> int:
    try:
        args = _parse_multi_args(argv)
        ensure_pwm_permissions()

        def _handle_signal(signum: int, _frame: object) -> None:
            print(f"Received signal {signum}, stopping...")
            _cleanup_all(use_lock=not args.no_lock)
            raise SystemExit(0)

        signal.signal(signal.SIGINT, _handle_signal)
        signal.signal(signal.SIGTERM, _handle_signal)

        cycle = 0
        while True:
            cycle += 1
            for duty in args.duties:
                print(
                    f"Setting duty={duty} (period={args.period}) on GPIOs: {' '.join(map(str, GPIOS))}"
                )
                for gpio in GPIOS:
                    set_pwm(gpio, args.period, duty, use_lock=not args.no_lock)
                time.sleep(args.wait_s)

            if args.cycles > 0 and cycle >= args.cycles:
                break

        return 0
    except Exception as exc:
        print(f"ERROR: {_format_error(exc)}", file=sys.stderr)
        return 1
