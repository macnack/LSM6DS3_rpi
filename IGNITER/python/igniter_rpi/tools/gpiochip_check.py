from __future__ import annotations

import argparse
import re
import subprocess
import sys
import tomllib
from pathlib import Path


def _normalize_chip_name(value: str) -> str:
    raw = value.strip()
    if raw.startswith("/dev/"):
        raw = raw.rsplit("/", 1)[-1]
    return raw


def _load_toml(path: Path) -> dict[str, object]:
    with path.open("rb") as fh:
        data = tomllib.load(fh)
    if not isinstance(data, dict):
        raise ValueError("TOML root must be a table")
    return data


def _detect_chip_labels() -> dict[str, str]:
    proc = subprocess.run(
        ["gpiodetect"],
        check=False,
        capture_output=True,
        text=True,
    )
    if proc.returncode != 0:
        raise RuntimeError(f"gpiodetect failed: {proc.stderr.strip() or proc.stdout.strip()}")

    labels: dict[str, str] = {}
    pattern = re.compile(r"^(gpiochip\d+)\s+\[(.+?)\]\s+\(\d+\s+lines\)")
    for line in proc.stdout.splitlines():
        match = pattern.match(line.strip())
        if match is None:
            continue
        labels[match.group(1)] = match.group(2)
    return labels


def _validate_config(
    cfg: dict[str, object],
    channel_prefix: str,
    channels: int,
    require_rp1: bool,
) -> tuple[bool, list[str]]:
    errors: list[str] = []
    input_chips: list[str] = []
    status_chips: list[str] = []

    for idx in range(channels):
        sec_name = f"{channel_prefix}{idx}"
        sec_raw = cfg.get(sec_name)
        if not isinstance(sec_raw, dict):
            errors.append(f"missing section [{sec_name}]")
            continue

        input_chip = sec_raw.get("input_chip")
        status_chip = sec_raw.get("status_chip")
        if not isinstance(input_chip, str) or not input_chip.strip():
            errors.append(f"[{sec_name}] missing non-empty input_chip")
        else:
            input_chips.append(_normalize_chip_name(input_chip))
        if not isinstance(status_chip, str) or not status_chip.strip():
            errors.append(f"[{sec_name}] missing non-empty status_chip")
        else:
            status_chips.append(_normalize_chip_name(status_chip))

    if errors:
        return False, errors

    input_set = sorted(set(input_chips))
    status_set = sorted(set(status_chips))

    if len(input_set) != 1:
        errors.append(f"input_chip is not one-chip: found {input_set}")
    if len(status_set) != 1:
        errors.append(f"status_chip is not one-chip: found {status_set}")
    if input_set and status_set and input_set[0] != status_set[0]:
        errors.append(
            f"input_chip ({input_set[0]}) and status_chip ({status_set[0]}) are on different chips"
        )

    if require_rp1 and not errors and input_set:
        labels = _detect_chip_labels()
        selected = input_set[0]
        label = labels.get(selected)
        if label is None:
            errors.append(
                f"selected chip '{selected}' not found in gpiodetect output; "
                "use chip name like /dev/gpiochipX from target device"
            )
        elif label != "pinctrl-rp1":
            errors.append(
                f"selected chip '{selected}' has label '{label}', expected 'pinctrl-rp1' on CM5/RPi5"
            )

    return len(errors) == 0, errors


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Validate igniter TOML chip consistency: all input_chip on one chip, "
            "all status_chip on one chip, and input/status on the same chip."
        )
    )
    parser.add_argument("toml", help="Path to runtime TOML config")
    parser.add_argument("--channel-prefix", default="igniter", help="Channel section prefix (default: igniter)")
    parser.add_argument("--channels", type=int, default=4, help="Number of igniter channel sections (default: 4)")
    parser.add_argument(
        "--require-rp1",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Require selected gpiochip label to be pinctrl-rp1 (default: true)",
    )
    args = parser.parse_args()

    cfg_path = Path(args.toml)
    if not cfg_path.exists():
        print(f"ERROR: config not found: {cfg_path}", file=sys.stderr)
        return 1
    if args.channels <= 0:
        print("ERROR: --channels must be >= 1", file=sys.stderr)
        return 1

    try:
        cfg = _load_toml(cfg_path)
        ok, problems = _validate_config(
            cfg,
            channel_prefix=args.channel_prefix,
            channels=args.channels,
            require_rp1=bool(args.require_rp1),
        )
    except Exception as exc:
        print(f"ERROR: validation failed: {exc}", file=sys.stderr)
        return 1

    if not ok:
        print("IGNITER CONFIG CHECK: FAIL")
        for item in problems:
            print(f"- {item}")
        return 1

    print("IGNITER CONFIG CHECK: PASS")
    print(f"- file: {cfg_path}")
    print(f"- channels: {args.channels}")
    print("- all input_chip values are on one chip")
    print("- all status_chip values are on one chip")
    print("- input_chip and status_chip are on the same chip")
    if args.require_rp1:
        print("- selected chip label verified as pinctrl-rp1")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

