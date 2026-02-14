#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import signal
import struct
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from runtime.python.ipc_common import (
    CONTROLLER_MSG_STRUCT,
    CONTROLLER_PAYLOAD_BYTES,
    ESTIMATOR_MSG_STRUCT,
    MESSAGE_MAGIC,
    MESSAGE_VERSION,
    ShmMailbox,
    MailboxConfig,
    finalize_crc,
    monotonic_ns,
    parse_int,
    parse_simple_toml,
    parse_string,
)


def roll_pitch_from_quat(qw: float, qx: float, qy: float, qz: float) -> tuple[float, float]:
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)
    return roll, pitch


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def main() -> int:
    parser = argparse.ArgumentParser(description="Dummy Python controller worker")
    parser.add_argument("--config", required=True)
    parser.add_argument("--duration-sec", type=float, default=0.0)
    args = parser.parse_args()

    cfg = parse_simple_toml(args.config)
    ipc = cfg["ipc"]
    threads = cfg["threads"]

    hz = parse_int(threads.get("control_hz", "200"))
    period_s = 1.0 / max(1.0, float(hz))

    retry_ms = parse_int(ipc.get("open_retry_ms", "50"))
    retry_count = parse_int(ipc.get("open_retry_count", "200"))

    est_mb = ShmMailbox(
        MailboxConfig(parse_string(ipc["estimator_state_shm"]), retries=retry_count, retry_sleep_s=retry_ms / 1000.0),
        ESTIMATOR_MSG_STRUCT.size,
    )
    cmd_mb = ShmMailbox(
        MailboxConfig(parse_string(ipc["controller_command_shm"]), retries=retry_count, retry_sleep_s=retry_ms / 1000.0),
        CONTROLLER_MSG_STRUCT.size,
    )

    est_mb.open_existing()
    cmd_mb.open_existing()

    running = True

    def _stop(_sig: int, _frame: object) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    seq = 0
    start = time.monotonic()

    try:
        while running:
            if args.duration_sec > 0 and (time.monotonic() - start) >= args.duration_sec:
                break

            roll = 0.0
            pitch = 0.0
            payload = est_mb.try_read()
            if payload is not None:
                values = ESTIMATOR_MSG_STRUCT.unpack(payload)
                (
                    magic,
                    version,
                    payload_bytes,
                    est_seq,
                    t_ns,
                    valid,
                    qw,
                    qx,
                    qy,
                    qz,
                    vx,
                    vy,
                    vz,
                    px,
                    py,
                    pz,
                    crc,
                ) = values
                _ = (est_seq, t_ns, vx, vy, vz, px, py, pz, crc)

                if magic == MESSAGE_MAGIC and version == MESSAGE_VERSION and payload_bytes == 64 and valid:
                    roll, pitch = roll_pitch_from_quat(qw, qx, qy, qz)

            cmd_roll = clamp(-1.1 * roll, -0.8, 0.8)
            cmd_pitch = clamp(-1.1 * pitch, -0.8, 0.8)

            s0 = clamp(cmd_roll - cmd_pitch, -1.0, 1.0)
            s1 = clamp(-cmd_roll - cmd_pitch, -1.0, 1.0)
            s2 = clamp(cmd_roll + cmd_pitch, -1.0, 1.0)
            s3 = clamp(-cmd_roll + cmd_pitch, -1.0, 1.0)

            seq += 1
            now_ns = monotonic_ns()
            msg_without_crc = CONTROLLER_MSG_STRUCT.pack(
                MESSAGE_MAGIC,
                MESSAGE_VERSION,
                CONTROLLER_PAYLOAD_BYTES,
                seq,
                now_ns,
                1,
                s0,
                s1,
                s2,
                s3,
                0,
            )
            crc = finalize_crc(msg_without_crc[:-4])
            msg = msg_without_crc[:-4] + struct.pack("<I", crc)
            cmd_mb.write(msg)

            time.sleep(period_s)
    finally:
        est_mb.close()
        cmd_mb.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
