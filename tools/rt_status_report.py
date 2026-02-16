#!/usr/bin/env python3
"""Dump rt_status.txt with colors plus ready-to-read jitter/tick tables."""

from __future__ import annotations

import argparse
import base64
import json
import os
import sys
import time
import zlib
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

COLOR = {
    "reset": "\033[0m",
    "red": "\033[31m",
    "yellow": "\033[33m",
    "green": "\033[32m",
    "cyan": "\033[36m",
    "blue": "\033[34m",
    "white": "\033[37m",
}

WARNING_KEYS = {
    "failsafe_activation_count",
    "failsafe_enter_count",
    "failsafe_exit_count",
    "failsafe_cause_cmd_stale_count",
    "failsafe_cause_link_down_count",
    "failsafe_cause_imu_stale_count",
    "failsafe_cause_killswitch_count",
    "failsafe_cause_actuator_io_latch_count",
    "killswitch_active",
    "degraded_mode_active",
    "imu_watchdog_fault_count",
    "imu_reinit_failure_count",
}
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
DEADLINE_COMPONENTS = (*JITTER_COMPONENTS, "i2c")
DEADLINE_KEYS = {f"{component}_deadline_miss_count" for component in DEADLINE_COMPONENTS}
WORKER_COMPONENTS = (
    ("Estimator", "external_estimator", "python_estimator"),
    ("Controller", "external_controller", "python_controller"),
)
WORKER_KEYS = {
    f"{prefix}_{suffix}"
    for _, primary, legacy in WORKER_COMPONENTS
    for prefix in (primary, legacy)
    for suffix in ("accept_count", "reject_count")
}
FAILSAFE_ROWS = (
    ("Active Ticks", "failsafe_activation_count"),
    ("Enter Events", "failsafe_enter_count"),
    ("Exit Events", "failsafe_exit_count"),
    ("Cause: cmd stale", "failsafe_cause_cmd_stale_count"),
    ("Cause: link down", "failsafe_cause_link_down_count"),
    ("Cause: imu stale", "failsafe_cause_imu_stale_count"),
    ("Cause: killswitch", "failsafe_cause_killswitch_count"),
    ("Cause: io latch", "failsafe_cause_actuator_io_latch_count"),
)
FAILSAFE_KEYS = {key for _, key in FAILSAFE_ROWS}
SIM_NET_ROWS = (
    (
        "Sensor",
        "sim_net_sensor_frames",
        "sim_net_sensor_crc_fail",
        "sim_net_sensor_disconnects",
        None,
        None,
    ),
    (
        "Actuator",
        "sim_net_actuator_frames",
        "sim_net_actuator_send_errors",
        "sim_net_actuator_disconnects",
        "sim_net_actuator_clients",
        "sim_net_actuator_client_connected",
    ),
)
SIM_NET_KEYS = {k for row in SIM_NET_ROWS for k in row[1:] if k is not None}
SHARE_SCHEMA_VERSION = 1
SHARE_TOKEN_PREFIX = f"rtshare{SHARE_SCHEMA_VERSION}_"
SHARE_PROTOCOL_PREFIX = "rtshare://"
SHARE_METRIC_KEYS = (
    "sim_mode",
    "degraded_mode_active",
    "killswitch_active",
    "last_failsafe_reason_name",
    "failsafe_activation_count",
    "failsafe_enter_count",
    "failsafe_exit_count",
    "control_deadline_miss_count",
    "actuator_deadline_miss_count",
    "imu_deadline_miss_count",
    "baro_deadline_miss_count",
    "i2c_deadline_miss_count",
    "control_jitter_p95_ns",
    "control_jitter_max_ns",
    "actuator_jitter_p95_ns",
    "actuator_jitter_max_ns",
    "sim_net_sensor_disconnects",
    "sim_net_actuator_disconnects",
    "sim_net_actuator_client_connected",
)
SHARE_PRIMARY_COUNTERS = (
    "failsafe_enter_count",
    "control_deadline_miss_count",
    "actuator_deadline_miss_count",
    "imu_deadline_miss_count",
    "baro_deadline_miss_count",
    "i2c_deadline_miss_count",
    "external_estimator_reject_count",
    "external_controller_reject_count",
)
SHARE_SUMMARY_ROWS = (
    ("Sim mode", "sim_mode"),
    ("Health", "health"),
    ("Failsafe enters", "failsafe_enter_count"),
    ("Last failsafe", "last_failsafe_reason_name"),
    ("Control miss", "control_deadline_miss_count"),
    ("Actuator miss", "actuator_deadline_miss_count"),
    ("Estimator reject", "external_estimator_reject_count"),
    ("Controller reject", "external_controller_reject_count"),
    ("Control jitter p95", "control_jitter_p95_ns"),
    ("Control jitter max", "control_jitter_max_ns"),
)
DEFAULT_EVENT_LOG_PATH = Path("/tmp/rt_status_report_events.jsonl")
ENABLE_COLOR = True


def paint(text: str, color: str) -> str:
    if not ENABLE_COLOR:
        return text
    return f"{COLOR[color]}{text}{COLOR['reset']}"


def classify(key: str, value: str) -> str:
    lower = key.lower()
    if lower == "imu_watchdog_state_name":
        mapped = value.strip().lower()
        if mapped == "healthy":
            return "green"
        if mapped in ("suspect", "recovering"):
            return "yellow"
        if mapped == "faulted":
            return "red"
        return "yellow"
    if lower == "imu_last_fault_reason_name":
        return "green" if value.strip().lower() == "none" else "yellow"
    if lower == "sim_net_actuator_client_connected":
        return "green" if value.strip().lower() in ("true", "1", "yes") else "yellow"
    if any(lower.endswith(suffix) for suffix in CRITICAL_SUFFIXES):
        try:
            return "red" if int(value) > 0 else "green"
        except ValueError:
            return "yellow"
    if lower in WARNING_KEYS:
        return "red" if value.strip().lower() not in ("false", "0") else "green"
    if lower == "last_failsafe_reason_name":
        return "green" if value.strip().lower() == "none" else "yellow"
    if lower in ("imu_consecutive_failures", "baro_consecutive_failures"):
        try:
            return "red" if int(value) > 0 else "green"
        except ValueError:
            return "yellow"
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
    row_colors = ("cyan", "blue")
    for idx, (raw_key, raw_value) in enumerate(status.items()):
        if raw_key in ignore:
            continue
        color = classify(raw_key, raw_value)
        key = paint(f"{raw_key:>38}", row_colors[idx % len(row_colors)])
        value = paint(raw_value, color)
        print(f"{key} = {value}")


def jitter_rows(status: dict[str, str]) -> Iterable[tuple[str, dict[str, str]]]:
    for component in JITTER_COMPONENTS:
        row = {}
        for metric in JITTER_METRICS:
            key = f"{component}_jitter_{metric}_ns"
            row[metric] = status.get(key, "—")
        yield component.title(), row


def format_table_header(columns: Iterable[str], widths: dict[str, int]) -> str:
    pieces = [f"{col:>{widths[col]}}" for col in columns]
    return " | ".join(pieces)


def print_deadline_tick_table(status: dict[str, str]) -> None:
    header = ("Component", "Misses", "Ticks")
    widths = {"Component": 12, "Misses": 8, "Ticks": 16}
    print("\nDeadline misses & tick counts:")
    print(format_table_header(header, widths))
    print("-" * (sum(widths.values()) + (len(header) - 1) * 3))
    for idx, component in enumerate(DEADLINE_COMPONENTS):
        miss_key = f"{component}_deadline_miss_count"
        ticks_key = f"{component}_ticks"
        misses = status.get(miss_key, "—")
        ticks = status.get(ticks_key, "—")
        miss_color = classify(miss_key, misses if misses != "—" else "0")
        tick_color = classify(ticks_key, ticks if ticks != "—" else "0")
        miss_formatted = (
            f"{int(misses):{widths['Misses']},d}"
            if misses.isdigit()
            else misses.rjust(widths["Misses"])
        )
        tick_formatted = (
            f"{int(ticks):{widths['Ticks']},d}"
            if ticks.isdigit()
            else ticks.rjust(widths["Ticks"])
        )
        component_label = paint(f"{component.title():>{widths['Component']}}", "white")
        row = " | ".join(
            [
                component_label,
                paint(miss_formatted, miss_color),
                paint(tick_formatted, tick_color),
            ]
        )
        print(row)


def print_worker_table(status: dict[str, str]) -> None:
    header = ("Component", "Accept", "Reject")
    widths = {"Component": 20, "Accept": 12, "Reject": 12}
    print("\nExternal worker counters:")
    print(format_table_header(header, widths))
    print("-" * (sum(widths.values()) + (len(header) - 1) * 3))
    for label, primary_prefix, legacy_prefix in WORKER_COMPONENTS:
        accept_key = f"{primary_prefix}_accept_count"
        reject_key = f"{primary_prefix}_reject_count"
        accept_value = status.get(accept_key)
        reject_value = status.get(reject_key)

        if accept_value is None:
            accept_key = f"{legacy_prefix}_accept_count"
            accept_value = status.get(accept_key, "—")
        if reject_value is None:
            reject_key = f"{legacy_prefix}_reject_count"
            reject_value = status.get(reject_key, "—")

        accept_color = classify(accept_key, accept_value if accept_value != "—" else "0")
        reject_color = classify(reject_key, reject_value if reject_value != "—" else "0")
        formatted_accept = (
            f"{int(accept_value):{widths['Accept']},d}"
            if accept_value.isdigit()
            else accept_value.rjust(widths["Accept"])
        )
        formatted_reject = (
            f"{int(reject_value):{widths['Reject']},d}"
            if reject_value.isdigit()
            else reject_value.rjust(widths["Reject"])
        )
        row = " | ".join(
            [
                paint(f"{label:>{widths['Component']}}", "white"),
                paint(formatted_accept, accept_color),
                paint(formatted_reject, reject_color),
            ]
        )
        print(row)


def print_failsafe_table(status: dict[str, str]) -> None:
    header = ("Counter", "Value")
    widths = {"Counter": 32, "Value": 14}
    print("\nFailsafe diagnostics:")
    print(format_table_header(header, widths))
    print("-" * (sum(widths.values()) + (len(header) - 1) * 3))
    for label, key in FAILSAFE_ROWS:
        value = status.get(key, "—")
        color = classify(key, value if value != "—" else "0")
        if value.isdigit():
            value_fmt = f"{int(value):{widths['Value']},d}"
        else:
            value_fmt = value.rjust(widths["Value"])
        print(
            " | ".join(
                [
                    paint(f"{label:>{widths['Counter']}}", "white"),
                    paint(value_fmt, color),
                ]
            )
        )


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


def print_sim_net_table(status: dict[str, str]) -> None:
    header = ("Flow", "Frames", "Errors/CRC", "Disconnects", "Clients", "Connected")
    widths = {
        "Flow": 10,
        "Frames": 14,
        "Errors/CRC": 16,
        "Disconnects": 14,
        "Clients": 12,
        "Connected": 11,
    }
    print("\nSim net (TCP bridge):")
    print(format_table_header(header, widths))
    print("-" * (sum(widths.values()) + (len(header) - 1) * 3))
    for label, frames_key, error_key, disconnects_key, clients_key, connected_key in SIM_NET_ROWS:
        frames = status.get(frames_key, "—")
        errors = status.get(error_key, "—")
        disconnects = status.get(disconnects_key, "—")
        clients = status.get(clients_key, "—") if clients_key is not None else "—"
        connected = status.get(connected_key, "—") if connected_key is not None else "—"
        frames_fmt = f"{int(frames):{widths['Frames']},d}" if frames.isdigit() else frames.rjust(widths["Frames"])
        errors_fmt = f"{int(errors):{widths['Errors/CRC']},d}" if errors.isdigit() else errors.rjust(widths["Errors/CRC"])
        disconnects_fmt = (
            f"{int(disconnects):{widths['Disconnects']},d}"
            if disconnects.isdigit()
            else disconnects.rjust(widths["Disconnects"])
        )
        clients_fmt = f"{int(clients):{widths['Clients']},d}" if clients.isdigit() else clients.rjust(widths["Clients"])
        connected_fmt = connected.rjust(widths["Connected"])
        frames_color = classify(frames_key, frames if frames != "—" else "0")
        errors_color = classify(error_key, errors if errors != "—" else "0")
        disconnects_color = classify(disconnects_key, disconnects if disconnects != "—" else "0")
        clients_color = classify(clients_key, clients if clients != "—" else "0") if clients_key else "cyan"
        connected_color = classify(connected_key, connected) if connected_key else "cyan"
        row = " | ".join(
            [
                paint(f"{label:>{widths['Flow']}}", "white"),
                paint(frames_fmt, frames_color),
                paint(errors_fmt, errors_color),
                paint(disconnects_fmt, disconnects_color),
                paint(clients_fmt, clients_color),
                paint(connected_fmt, connected_color),
            ]
        )
        print(row)


def parse_status(path: Path) -> dict[str, str]:
    data: dict[str, str] = {}
    for line in path.read_text().splitlines():
        if not line or "=" not in line:
            continue
        key, value = line.split("=", 1)
        data[key.strip()] = value.strip()
    return data


def parse_int(status: dict[str, str], key: str) -> int | None:
    raw = status.get(key)
    if raw is None:
        return None
    try:
        return int(raw)
    except ValueError:
        return None


def resolve_counter(status: dict[str, str], primary: str, legacy: str) -> str:
    if primary in status:
        return status[primary]
    return status.get(legacy, "0")


def share_health(status: dict[str, str]) -> str:
    if status.get("killswitch_active", "false").strip().lower() not in ("false", "0", "no"):
        return "red"
    if status.get("degraded_mode_active", "false").strip().lower() not in ("false", "0", "no"):
        return "yellow"
    if status.get("last_failsafe_reason_name", "none").strip().lower() != "none":
        return "yellow"
    for key in SHARE_PRIMARY_COUNTERS:
        value = parse_int(status, key)
        if value is not None and value > 0:
            return "yellow"
    return "green"


def build_share_snapshot(status: dict[str, str], source_path: Path) -> dict[str, object]:
    payload: dict[str, str] = {key: status.get(key, "n/a") for key in SHARE_METRIC_KEYS}
    payload["external_estimator_reject_count"] = resolve_counter(
        status,
        "external_estimator_reject_count",
        "python_estimator_reject_count",
    )
    payload["external_controller_reject_count"] = resolve_counter(
        status,
        "external_controller_reject_count",
        "python_controller_reject_count",
    )
    return {
        "v": SHARE_SCHEMA_VERSION,
        "created_unix": int(time.time()),
        "source": source_path.name,
        "health": share_health(payload),
        "metrics": payload,
    }


def _urlsafe_b64_decode(raw: str) -> bytes:
    padding = "=" * ((4 - len(raw) % 4) % 4)
    return base64.urlsafe_b64decode(raw + padding)


def encode_share_token(snapshot: dict[str, object]) -> str:
    payload = json.dumps(snapshot, separators=(",", ":"), sort_keys=True).encode("utf-8")
    compressed = zlib.compress(payload, level=9)
    token = base64.urlsafe_b64encode(compressed).decode("ascii").rstrip("=")
    return f"{SHARE_TOKEN_PREFIX}{token}"


def decode_share_token(raw_token: str) -> dict[str, object]:
    token = raw_token.strip()
    if token.startswith(SHARE_PROTOCOL_PREFIX):
        token = token[len(SHARE_PROTOCOL_PREFIX) :]
    if token.startswith(SHARE_TOKEN_PREFIX):
        token = token[len(SHARE_TOKEN_PREFIX) :]
    if not token:
        raise ValueError("Share token is empty")
    try:
        compressed = _urlsafe_b64_decode(token)
        payload = zlib.decompress(compressed)
        data = json.loads(payload.decode("utf-8"))
    except (ValueError, zlib.error, json.JSONDecodeError) as exc:
        raise ValueError("Share token is invalid") from exc
    if not isinstance(data, dict):
        raise ValueError("Share token payload is invalid")
    if data.get("v") != SHARE_SCHEMA_VERSION:
        raise ValueError(f"Unsupported share token version: {data.get('v')!r}")
    metrics = data.get("metrics")
    if not isinstance(metrics, dict):
        raise ValueError("Share token payload is missing metrics")
    return data


def parse_event_log_path(raw_path: str) -> Path:
    raw = raw_path.strip()
    if not raw:
        return DEFAULT_EVENT_LOG_PATH
    return Path(raw)


def log_growth_event(event: str, fields: dict[str, str | int]) -> None:
    sink = parse_event_log_path(os.environ.get("RT_STATUS_REPORT_EVENT_LOG", str(DEFAULT_EVENT_LOG_PATH)))
    record: dict[str, str | int] = {"event": event, "ts_unix": int(time.time())}
    record.update(fields)
    try:
        sink.parent.mkdir(parents=True, exist_ok=True)
        with sink.open("a", encoding="utf-8") as fh:
            fh.write(json.dumps(record, separators=(",", ":"), sort_keys=True) + "\n")
    except OSError:
        # Event logging should never break runtime diagnostics output.
        return


def share_health_color(health: str) -> str:
    if health == "red":
        return "red"
    if health == "yellow":
        return "yellow"
    return "green"


def print_share_snapshot(snapshot: dict[str, object]) -> None:
    metrics = snapshot.get("metrics")
    if not isinstance(metrics, dict):
        raise SystemExit("Share snapshot is missing metrics")

    created_unix = snapshot.get("created_unix")
    created_at = "unknown"
    if isinstance(created_unix, int):
        created_at = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(created_unix))

    source = snapshot.get("source", "unknown")
    health = str(snapshot.get("health", "unknown")).lower()
    print(paint("Shared runtime snapshot", "white"))
    print(f"source: {source}")
    print(f"created: {created_at}")
    print(f"health: {paint(health.upper(), share_health_color(health))}")

    header = ("Metric", "Value")
    widths = {"Metric": 24, "Value": 20}
    print("")
    print(format_table_header(header, widths))
    print("-" * (sum(widths.values()) + (len(header) - 1) * 3))
    for label, key in SHARE_SUMMARY_ROWS:
        raw = str(metrics.get(key, "n/a")) if key != "health" else health
        value_color = share_health_color(raw) if key == "health" else classify(key, raw)
        row = " | ".join(
            [
                paint(f"{label:>{widths['Metric']}}", "white"),
                paint(f"{raw:>{widths['Value']}}", value_color),
            ]
        )
        print(row)


def emit_share_success(snapshot: dict[str, object], token: str, base_url: str | None) -> None:
    metrics = snapshot.get("metrics")
    if not isinstance(metrics, dict):
        raise SystemExit("Share snapshot metrics unavailable")

    health = str(snapshot.get("health", "unknown"))
    print("\nShare snapshot ready:")
    print(f"token: {token}")
    if base_url:
        url = f"{base_url.rstrip('/')}/{token}"
        print(f"url: {url}")
    print(f"open command: rt-status-report --open-share {token}")
    log_growth_event(
        "share_snapshot_created",
        {
            "health": health,
            "sim_mode": str(metrics.get("sim_mode", "n/a")),
            "failsafe_enters": str(metrics.get("failsafe_enter_count", "n/a")),
        },
    )


@dataclass
class MonitorState:
    prev_status: dict[str, str] | None = None
    prev_sample_ts: float = 0.0
    sensor_last_seen_ts: float | None = None
    actuator_last_seen_ts: float | None = None
    sensor_last_seen_wall: float | None = None
    actuator_last_seen_wall: float | None = None


def health_from_age(age_s: float | None, warn_sec: float, stall_sec: float) -> tuple[str, str]:
    if age_s is None:
        return "red", "NO_DATA"
    if age_s >= stall_sec:
        return "red", "STALLED"
    if age_s >= warn_sec:
        return "yellow", "SLOW"
    return "green", "OK"


def format_last_seen(age_s: float | None, wall_ts: float | None) -> str:
    if age_s is None or wall_ts is None:
        return "never"
    wall = time.strftime("%H:%M:%S", time.localtime(wall_ts))
    return f"{wall} ({age_s:4.1f}s ago)"


def monitor_text(value: int | None, width: int) -> str:
    if value is None:
        return "n/a".rjust(width)
    return f"{value:{width},d}"


def monitor_rate_text(rate: float | None, width: int) -> str:
    if rate is None:
        return "n/a".rjust(width)
    return f"{rate:>{width}.1f}"


def print_link_health(
    path: Path,
    state: MonitorState,
    warn_sec: float,
    stall_sec: float,
) -> None:
    now_wall = time.time()
    now_mono = time.monotonic()
    status = parse_status(path)

    sensor_frames = parse_int(status, "sim_net_sensor_frames")
    actuator_frames = parse_int(status, "sim_net_actuator_frames")
    sensor_crc = parse_int(status, "sim_net_sensor_crc_fail")
    sensor_disconnects = parse_int(status, "sim_net_sensor_disconnects")
    actuator_errors = parse_int(status, "sim_net_actuator_send_errors")
    actuator_disconnects = parse_int(status, "sim_net_actuator_disconnects")
    actuator_clients = parse_int(status, "sim_net_actuator_clients")
    actuator_connected_raw = status.get("sim_net_actuator_client_connected")
    actuator_connected: bool | None = None
    if actuator_connected_raw is not None:
        actuator_connected = actuator_connected_raw.strip().lower() in ("true", "1", "yes")

    sensor_rate: float | None = None
    actuator_rate: float | None = None
    if state.prev_status is not None and state.prev_sample_ts > 0.0:
        dt_s = max(now_mono - state.prev_sample_ts, 1e-6)
        prev_sensor_frames = parse_int(state.prev_status, "sim_net_sensor_frames")
        prev_actuator_frames = parse_int(state.prev_status, "sim_net_actuator_frames")
        if sensor_frames is not None and prev_sensor_frames is not None:
            sensor_rate = max(sensor_frames - prev_sensor_frames, 0) / dt_s
        if actuator_frames is not None and prev_actuator_frames is not None:
            actuator_rate = max(actuator_frames - prev_actuator_frames, 0) / dt_s

    if state.prev_status is None:
        if sensor_frames is not None and sensor_frames > 0:
            state.sensor_last_seen_ts = now_mono
            state.sensor_last_seen_wall = now_wall
        if actuator_frames is not None and actuator_frames > 0:
            state.actuator_last_seen_ts = now_mono
            state.actuator_last_seen_wall = now_wall
    else:
        prev_sensor_frames = parse_int(state.prev_status, "sim_net_sensor_frames")
        prev_actuator_frames = parse_int(state.prev_status, "sim_net_actuator_frames")
        if (
            sensor_frames is not None
            and prev_sensor_frames is not None
            and sensor_frames > prev_sensor_frames
        ):
            state.sensor_last_seen_ts = now_mono
            state.sensor_last_seen_wall = now_wall
        if (
            actuator_frames is not None
            and prev_actuator_frames is not None
            and actuator_frames > prev_actuator_frames
        ):
            state.actuator_last_seen_ts = now_mono
            state.actuator_last_seen_wall = now_wall

    sensor_age = None if state.sensor_last_seen_ts is None else max(0.0, now_mono - state.sensor_last_seen_ts)
    actuator_age = None if state.actuator_last_seen_ts is None else max(0.0, now_mono - state.actuator_last_seen_ts)

    sensor_health_color, sensor_health_text = health_from_age(sensor_age, warn_sec, stall_sec)
    actuator_health_color, actuator_health_text = health_from_age(actuator_age, warn_sec, stall_sec)
    if actuator_connected is False:
        actuator_health_color = "yellow"
        actuator_health_text = "NO_CLIENT"

    status_age = max(0.0, now_wall - path.stat().st_mtime)
    if status_age >= stall_sec:
        status_color = "red"
    elif status_age >= warn_sec:
        status_color = "yellow"
    else:
        status_color = "green"

    print(paint("Link health monitor (live)", "white"))
    print(f"status file: {path}")
    print(
        "status age: "
        + paint(f"{status_age:4.1f}s", status_color)
        + " | sim_mode="
        + status.get("sim_mode", "n/a")
    )

    header = ("Flow", "Frames", "Rate(fps)", "Last Seen", "Health", "Notes")
    widths = {"Flow": 10, "Frames": 14, "Rate(fps)": 10, "Last Seen": 23, "Health": 10, "Notes": 36}
    print("")
    print(format_table_header(header, widths))
    print("-" * (sum(widths.values()) + (len(header) - 1) * 3))

    sensor_notes = (
        f"crc={sensor_crc if sensor_crc is not None else 'n/a'} "
        f"disconnects={sensor_disconnects if sensor_disconnects is not None else 'n/a'}"
    )
    sensor_row = " | ".join(
        [
            paint(f"{'Sensor':>{widths['Flow']}}", "white"),
            paint(monitor_text(sensor_frames, widths["Frames"]), "cyan"),
            paint(monitor_rate_text(sensor_rate, widths["Rate(fps)"]), "cyan"),
            paint(f"{format_last_seen(sensor_age, state.sensor_last_seen_wall):>{widths['Last Seen']}}", "cyan"),
            paint(f"{sensor_health_text:>{widths['Health']}}", sensor_health_color),
            paint(f"{sensor_notes:>{widths['Notes']}}", "cyan"),
        ]
    )
    print(sensor_row)

    actuator_notes = (
        f"send_err={actuator_errors if actuator_errors is not None else 'n/a'} "
        f"disconnects={actuator_disconnects if actuator_disconnects is not None else 'n/a'} "
        f"clients={actuator_clients if actuator_clients is not None else 'n/a'} "
        f"connected={actuator_connected_raw if actuator_connected_raw is not None else 'n/a'}"
    )
    actuator_row = " | ".join(
        [
            paint(f"{'Actuator':>{widths['Flow']}}", "white"),
            paint(monitor_text(actuator_frames, widths["Frames"]), "cyan"),
            paint(monitor_rate_text(actuator_rate, widths["Rate(fps)"]), "cyan"),
            paint(f"{format_last_seen(actuator_age, state.actuator_last_seen_wall):>{widths['Last Seen']}}", "cyan"),
            paint(f"{actuator_health_text:>{widths['Health']}}", actuator_health_color),
            paint(f"{actuator_notes:>{widths['Notes']}}", "cyan"),
        ]
    )
    print(actuator_row)

    state.prev_status = status
    state.prev_sample_ts = now_mono


def print_report(path: Path) -> dict[str, str]:
    status = parse_status(path)
    tick_keys = {k for k in status if k.endswith("_ticks")}
    ignore = JITTER_KEYS | DEADLINE_KEYS | WORKER_KEYS | FAILSAFE_KEYS | tick_keys | SIM_NET_KEYS
    print_kv(status, ignore)
    print_worker_table(status)
    print_failsafe_table(status)
    print_deadline_tick_table(status)
    print_jitter_table(status)
    if status.get("sim_mode", "").lower() == "true":
        print_sim_net_table(status)
    return status


def main() -> None:
    parser = argparse.ArgumentParser(description="Pretty-print rt_status.txt or run live link monitor.")
    parser.add_argument("path", nargs="?", type=Path, default=Path("/tmp/rt_status.txt"))
    parser.add_argument("--once", action="store_true", help="Run the report once and exit (default loops).")
    parser.add_argument(
        "--no-color",
        action="store_true",
        help="Disable ANSI colors (recommended for systemd/journald).",
    )
    parser.add_argument(
        "--no-clear",
        action="store_true",
        help="Do not clear screen between refreshes.",
    )
    parser.add_argument(
        "--systemd",
        action="store_true",
        help="Enable service-friendly defaults (equivalent to --no-color --no-clear).",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.5,
        help="Seconds between refreshes in watch mode.",
    )
    parser.add_argument(
        "--monitor",
        action="store_true",
        help="Run live sim_net link monitor with frame rates and stall detection.",
    )
    parser.add_argument(
        "--warn-sec",
        type=float,
        default=1.5,
        help="Age threshold (seconds) for warning-level stale links in monitor mode.",
    )
    parser.add_argument(
        "--stall-sec",
        type=float,
        default=3.0,
        help="Age threshold (seconds) for stall-level link alerts in monitor mode.",
    )
    parser.add_argument(
        "--share",
        action="store_true",
        help="Generate a compact share token for the current snapshot (requires --once).",
    )
    parser.add_argument(
        "--share-url-base",
        default=None,
        help="Optional base URL prefix for rendered share links.",
    )
    parser.add_argument(
        "--open-share",
        default=None,
        help="Decode and display a previously generated share token.",
    )
    args = parser.parse_args()
    if args.systemd:
        args.no_color = True
        args.no_clear = True
    if not sys.stdout.isatty():
        args.no_clear = True
    if os.environ.get("NO_COLOR") is not None:
        args.no_color = True

    global ENABLE_COLOR
    ENABLE_COLOR = not args.no_color

    if args.open_share and args.share:
        raise SystemExit("Use either --share or --open-share, not both")

    if args.open_share:
        try:
            snapshot = decode_share_token(args.open_share)
        except ValueError as exc:
            raise SystemExit(str(exc)) from exc
        print_share_snapshot(snapshot)
        log_growth_event(
            "share_snapshot_opened",
            {
                "health": str(snapshot.get("health", "n/a")),
                "source": str(snapshot.get("source", "n/a")),
            },
        )
        return

    if args.share and not args.once:
        raise SystemExit("--share requires --once")

    if not args.path.exists():
        raise SystemExit(f"{args.path} not found")
    if args.warn_sec <= 0 or args.stall_sec <= 0:
        raise SystemExit("--warn-sec and --stall-sec must be > 0")
    if args.warn_sec >= args.stall_sec:
        raise SystemExit("--warn-sec must be < --stall-sec")

    if args.once:
        if args.monitor:
            print_link_health(args.path, MonitorState(), args.warn_sec, args.stall_sec)
        else:
            status = print_report(args.path)
            if args.share:
                snapshot = build_share_snapshot(status, args.path)
                token = encode_share_token(snapshot)
                emit_share_success(snapshot, token, args.share_url_base)
        return

    command = "cls" if os.name == "nt" else "clear"
    monitor_state = MonitorState()
    try:
        while True:
            if not args.no_clear:
                os.system(command)
            if args.monitor:
                print_link_health(args.path, monitor_state, args.warn_sec, args.stall_sec)
            else:
                print_report(args.path)
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\nStopped refresh.")


if __name__ == "__main__":
    main()
