#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import signal
import time
try:
    from runtime.python.ipc_common import (
        CONTROLLER_MSG_STRUCT,
        SENSOR_MSG_STRUCT,
        SensorSnapshot,
        ShmMailbox,
        MailboxConfig,
        decode_sensor_snapshot,
        encode_controller_command,
        monotonic_ns,
        parse_int,
        parse_simple_toml,
        parse_string,
    )
except ModuleNotFoundError:
    from ipc_common import (
        CONTROLLER_MSG_STRUCT,
        SENSOR_MSG_STRUCT,
        SensorSnapshot,
        ShmMailbox,
        MailboxConfig,
        decode_sensor_snapshot,
        encode_controller_command,
        monotonic_ns,
        parse_int,
        parse_simple_toml,
        parse_string,
    )


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def main() -> int:
    parser = argparse.ArgumentParser(description="Dummy Python controller worker")
    parser.add_argument("--config", required=True)
    parser.add_argument("--duration-sec", type=float, default=0.0)
    parser.add_argument("--sine-hz", type=float, default=0.5)
    parser.add_argument("--duty-min", type=float, default=0.2)
    parser.add_argument("--duty-max", type=float, default=0.8)
    parser.add_argument("--telemetry-print-hz", type=float, default=5.0)
    args = parser.parse_args()

    cfg = parse_simple_toml(args.config)
    ipc = cfg["ipc"]
    threads = cfg["threads"]

    hz = parse_int(threads.get("control_hz", "200"))
    period_s = 1.0 / max(1.0, float(hz))

    retry_ms = parse_int(ipc.get("open_retry_ms", "50"))
    retry_count = parse_int(ipc.get("open_retry_count", "200"))

    sensor_mb = ShmMailbox(
        MailboxConfig(parse_string(ipc["sensor_snapshot_shm"]), retries=retry_count, retry_sleep_s=retry_ms / 1000.0),
        SENSOR_MSG_STRUCT.size,
    )
    cmd_mb = ShmMailbox(
        MailboxConfig(parse_string(ipc["controller_command_shm"]), retries=retry_count, retry_sleep_s=retry_ms / 1000.0),
        CONTROLLER_MSG_STRUCT.size,
    )

    sensor_mb.open_existing()
    cmd_mb.open_existing()

    if args.sine_hz <= 0.0:
        raise ValueError("--sine-hz must be > 0")
    if args.duty_min < 0.0 or args.duty_max > 1.0 or args.duty_min >= args.duty_max:
        raise ValueError("--duty-min/--duty-max must satisfy 0 <= duty-min < duty-max <= 1")
    if args.telemetry_print_hz < 0.0:
        raise ValueError("--telemetry-print-hz must be >= 0")

    duty_center = 0.5 * (args.duty_min + args.duty_max)
    duty_amplitude = 0.5 * (args.duty_max - args.duty_min)
    phase_offsets = (0.0, 0.5 * math.pi, math.pi, 1.5 * math.pi)
    print_period_s = 0.0 if args.telemetry_print_hz == 0.0 else (1.0 / args.telemetry_print_hz)

    running = True

    def _stop(_sig: int, _frame: object) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    seq = 0
    start = time.monotonic()
    next_print_t = start
    last_sensor: SensorSnapshot | None = None

    try:
        while running:
            if args.duration_sec > 0 and (time.monotonic() - start) >= args.duration_sec:
                break

            payload = sensor_mb.try_read()
            if payload is not None:
                sample = decode_sensor_snapshot(payload)
                if sample is not None:
                    last_sensor = sample

            phase = 2.0 * math.pi * args.sine_hz * (time.monotonic() - start)
            duty_values = [duty_center + duty_amplitude * math.sin(phase + p) for p in phase_offsets]
            s0, s1, s2, s3 = [clamp((2.0 * duty) - 1.0, -1.0, 1.0) for duty in duty_values]
            seq += 1
            now_ns = monotonic_ns()
            msg = encode_controller_command(
                seq=seq,
                t_ns=now_ns,
                armed=True,
                servo_norm=(s0, s1, s2, s3),
            )
            cmd_mb.write(msg)
            now_wall = time.monotonic()
            if print_period_s > 0.0 and now_wall >= next_print_t:
                if last_sensor is None:
                    print("[sensor] waiting for sensor snapshots...")
                else:
                    imu_text = (
                        f"ax={last_sensor.ax_mps2:+.3f} ay={last_sensor.ay_mps2:+.3f} az={last_sensor.az_mps2:+.3f} "
                        f"gx={last_sensor.gx_rads:+.3f} gy={last_sensor.gy_rads:+.3f} gz={last_sensor.gz_rads:+.3f} "
                        if last_sensor.imu_valid
                        else "imu=invalid"
                    )
                    baro_text = (
                        f"p={last_sensor.pressure_pa:.1f} Pa temp={last_sensor.temperature_c:.2f} C"
                        if last_sensor.baro_valid
                        else "baro=invalid"
                    )
                    print(f"[sensor] seq={last_sensor.seq} t_ns={last_sensor.t_ns} {imu_text} {baro_text}")
                next_print_t = now_wall + print_period_s

            time.sleep(period_s)
    finally:
        sensor_mb.close()
        cmd_mb.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
