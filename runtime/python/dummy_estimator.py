#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import signal
import struct
import time
try:
    from runtime.python.ipc_common import (
        ESTIMATOR_MSG_STRUCT,
        ESTIMATOR_PAYLOAD_BYTES,
        MESSAGE_MAGIC,
        MESSAGE_VERSION,
        SENSOR_MSG_STRUCT,
        ShmMailbox,
        MailboxConfig,
        finalize_crc,
        monotonic_ns,
        parse_float,
        parse_int,
        parse_simple_toml,
        parse_string,
    )
except ModuleNotFoundError:
    from ipc_common import (
    ESTIMATOR_MSG_STRUCT,
    ESTIMATOR_PAYLOAD_BYTES,
    MESSAGE_MAGIC,
    MESSAGE_VERSION,
    SENSOR_MSG_STRUCT,
    ShmMailbox,
    MailboxConfig,
    finalize_crc,
    monotonic_ns,
    parse_float,
    parse_int,
    parse_simple_toml,
    parse_string,
    )


def quat_from_euler(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return w, x, y, z


def main() -> int:
    parser = argparse.ArgumentParser(description="Dummy Python estimator worker")
    parser.add_argument("--config", required=True)
    parser.add_argument("--duration-sec", type=float, default=0.0)
    args = parser.parse_args()

    cfg = parse_simple_toml(args.config)
    ipc = cfg["ipc"]
    threads = cfg["threads"]

    hz = parse_int(threads.get("estimator_hz", "200"))
    period_s = 1.0 / max(1.0, float(hz))

    retry_ms = parse_int(ipc.get("open_retry_ms", "50"))
    retry_count = parse_int(ipc.get("open_retry_count", "200"))

    sensor_mb = ShmMailbox(
        MailboxConfig(parse_string(ipc["sensor_snapshot_shm"]), retries=retry_count, retry_sleep_s=retry_ms / 1000.0),
        SENSOR_MSG_STRUCT.size,
    )
    est_mb = ShmMailbox(
        MailboxConfig(parse_string(ipc["estimator_state_shm"]), retries=retry_count, retry_sleep_s=retry_ms / 1000.0),
        ESTIMATOR_MSG_STRUCT.size,
    )

    sensor_mb.open_existing()
    est_mb.open_existing()

    running = True

    def _stop(_sig: int, _frame: object) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    seq = 0
    roll = 0.0
    pitch = 0.0
    yaw = 0.0
    last_t = 0

    start = time.monotonic()
    try:
        while running:
            if args.duration_sec > 0 and (time.monotonic() - start) >= args.duration_sec:
                break

            payload = sensor_mb.try_read()
            now_ns = monotonic_ns()
            if payload is not None:
                values = SENSOR_MSG_STRUCT.unpack(payload)
                (
                    magic,
                    version,
                    payload_bytes,
                    sensor_seq,
                    t_ns,
                    imu_valid,
                    baro_valid,
                    ax,
                    ay,
                    az,
                    gx,
                    gy,
                    gz,
                    pressure_pa,
                    temp_c,
                    crc,
                ) = values
                _ = (sensor_seq, baro_valid, pressure_pa, temp_c)

                if magic == MESSAGE_MAGIC and version == MESSAGE_VERSION and payload_bytes == 88 and imu_valid:
                    if last_t and t_ns > last_t:
                        dt = (t_ns - last_t) * 1e-9
                    else:
                        dt = period_s
                    last_t = t_ns

                    roll += gx * dt
                    pitch += gy * dt
                    yaw += gz * dt

                    accel_roll = math.atan2(ay, az)
                    accel_pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
                    alpha = 0.03
                    roll = (1.0 - alpha) * roll + alpha * accel_roll
                    pitch = (1.0 - alpha) * pitch + alpha * accel_pitch

            q = quat_from_euler(roll, pitch, yaw)
            seq += 1
            msg_without_crc = ESTIMATOR_MSG_STRUCT.pack(
                MESSAGE_MAGIC,
                MESSAGE_VERSION,
                ESTIMATOR_PAYLOAD_BYTES,
                seq,
                now_ns,
                1,
                q[0],
                q[1],
                q[2],
                q[3],
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0,
            )
            crc = finalize_crc(msg_without_crc[:-4])
            msg = msg_without_crc[:-4] + struct.pack("<I", crc)
            est_mb.write(msg)

            time.sleep(period_s)
    finally:
        sensor_mb.close()
        est_mb.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
