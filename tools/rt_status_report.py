#!/usr/bin/env python3
"\"\"\"Dump rt_status.txt with colors plus ready-to-read jitter/tick tables.\"\""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Iterable

COLOR = {
    "reset": "\033[0m",
    "red": "\033[31m",
    "yellow": "\033[33m",
    "green": "\033[32m",
    "cyan": "\033[36m",
}

WARNING_KEYS = {"failsafe_activation_count", "killswitch_active"}
CRITICAL_SUFFIXES = ("_deadline_miss_count", "_reject_count", "_trip_count")
JITTER_THRESHOLDS = {"p50": 80000, "p95": 120000, "p99": 200000, "max": 300000}
METRIC_LABELS = {"p50": "P50", "p95": "P95", "p99": "P99", "max": "Max"}
JITTER_COMPONENTS = ("control", "actuator", "imu", "baro")
JITTER_METRICS = ("p50", "p95", "p99", "max")
JITTER_KEYS = {
    f"{component}_jitter_{metric}_ns"
    for component in JITTER_COMPONENTS
    for metric in JITTER_METRICS
}


def paint(text: str, color: str) -> str:
    return f"{COLOR[color]}{text}{COLOR['reset']}"


def classify(key: str, value: str) -> str:
    lower = key.lower()
    if any(lower.endswith(suffix) for suffix in CRITICAL_SUFFIXES):
        try:
            return "red" if int(value) > 0 else "green"
        except ValueError:
            return "yellow"
    if lower in WARNING_KEYS:
        return "red" if value.strip().lower() not in ("false", "0") else "green"
    if "jitter" in lower:
        for label, threshold in JITTER_THRESHOLDS.items():
            if lower.endswith(label):
                try:
                    j = int(value)
                except ValueError:
                    return "yellow"
                if j > threshold:
                    return "red" if label == "max" else "yellow"
                return "green"
    return "cyan"


def print_kv(status: dict[str, str], ignore: set[str]) -> None:
    for raw_key, raw_value in status.items():
        if raw_key in ignore:
            continue
        color = classify(raw_key, raw_value)
        key = paint(f"{raw_key:>38}", "cyan")
        value = paint(raw_value, color)
        print(f"{key} = {value}")


def jitter_rows(status: dict[str, str]) -> Iterable[tuple[str, dict[str, str]]]:
    for component in JITTER_COMPONENTS:
        row = {}
        for metric in JITTER_METRICS:
            key = f"{component}_jitter_{metric}_ns"
            row[metric] = status.get(key, "—")
        yield component.title(), row


def ticks_rows(status: dict[str, str]) -> Iterable[tuple[str, str]]:
    for key in sorted(k for k in status if k.endswith("_ticks")):
        name = key[: -len("_ticks")]
        yield name.title(), status[key]


def format_table_header(columns: Iterable[str], widths: dict[str, int]) -> str:
    pieces = [f"{col:>{widths[col]}}" for col in columns]
    return " | ".join(pieces)


def print_jitter_table(status: dict[str, str]) -> None:
    header = ("Component", "P50", "P95", "P99", "Max")
    widths = {"Component": 10, "P50": 12, "P95": 12, "P99": 12, "Max": 12}
    print("\nJitter summary (ns):")
    print(format_table_header(header, widths))
    print("-" * (sum(widths.values()) + (len(header) - 1) * 3))
    for component, row in jitter_rows(status):
        parts = [f"{component:>{widths['Component']}}"]
        for metric in ("p50", "p95", "p99", "max"):
            raw = row[metric]
            color = classify(f"{component.lower()}_jitter_{metric}_ns", raw) if raw != "—" else "cyan"
            if raw.isdigit():
                label = METRIC_LABELS[metric]
                formatted = f"{int(raw):{widths[label]},.0f}"
            else:
                label = METRIC_LABELS[metric]
                formatted = raw.rjust(widths[label])
            parts.append(paint(formatted, color))
        print(" | ".join(parts))


def print_ticks_table(status: dict[str, str]) -> None:
    header = ("Component", "Ticks")
    widths = {"Component": 12, "Ticks": 16}
    print("\nTick counts:")
    print(format_table_header(header, widths))
    print("-" * (sum(widths.values()) + 3))
    for component, value in ticks_rows(status):
        try:
            count = int(value)
            formatted = f"{count:{widths['Ticks']},d}"
        except ValueError:
            formatted = value.rjust(widths["Ticks"])
        print(f"{component:>{widths['Component']}} | {paint(formatted, 'cyan')}")


def parse_status(path: Path) -> dict[str, str]:
    data: dict[str, str] = {}
    for line in path.read_text().splitlines():
        if not line or "=" not in line:
            continue
        key, value = line.split("=", 1)
        data[key.strip()] = value.strip()
    return data


def main() -> None:
    parser = argparse.ArgumentParser(description="Pretty-print rt_status.txt with tables.")
    parser.add_argument("path", nargs="?", type=Path, default=Path("/tmp/rt_status.txt"))
    args = parser.parse_args()
    if not args.path.exists():
        raise SystemExit(f"{args.path} not found")

    status = parse_status(args.path)
    tick_keys = {k for k in status if k.endswith("_ticks")}
    ignore = JITTER_KEYS | tick_keys
    print_kv(status, ignore)
    print_jitter_table(status)
    print_ticks_table(status)


if __name__ == "__main__":
    main()
