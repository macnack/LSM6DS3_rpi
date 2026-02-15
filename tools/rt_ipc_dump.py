#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

try:
    from runtime.python.ipc_common import (
        CONTROLLER_MSG_STRUCT,
        ESTIMATOR_MSG_STRUCT,
        MESSAGE_MAGIC,
        MESSAGE_VERSION,
        SENSOR_MSG_STRUCT,
        ShmMailbox,
        MailboxConfig,
    )
except ModuleNotFoundError:
    sys.path.append(str(Path(__file__).resolve().parents[1] / "runtime" / "python"))
    from ipc_common import (
        CONTROLLER_MSG_STRUCT,
        ESTIMATOR_MSG_STRUCT,
        MESSAGE_MAGIC,
        MESSAGE_VERSION,
        SENSOR_MSG_STRUCT,
        ShmMailbox,
        MailboxConfig,
    )


def dump_sensor(payload: bytes) -> None:
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
        _crc,
    ) = SENSOR_MSG_STRUCT.unpack(payload)
    if magic != MESSAGE_MAGIC or version != MESSAGE_VERSION:
        return
    print(
        "sensor",
        f"seq={sensor_seq}",
        f"t_ns={t_ns}",
        f"imu_valid={imu_valid}",
        f"baro_valid={baro_valid}",
        f"ax={ax:.3f}",
        f"ay={ay:.3f}",
        f"az={az:.3f}",
        f"gx={gx:.3f}",
        f"gy={gy:.3f}",
        f"gz={gz:.3f}",
        f"pressure_pa={pressure_pa:.1f}",
        f"temp_c={temp_c:.2f}",
    )


def dump_estimator(payload: bytes) -> None:
    (
        magic,
        version,
        payload_bytes,
        seq,
        t_ns,
        valid,
        q0,
        q1,
        q2,
        q3,
        pn,
        pe,
        pd,
        vn,
        ve,
        vd,
        _crc,
    ) = ESTIMATOR_MSG_STRUCT.unpack(payload)
    if magic != MESSAGE_MAGIC or version != MESSAGE_VERSION:
        return
    print(
        "estimator",
        f"seq={seq}",
        f"t_ns={t_ns}",
        f"valid={valid}",
        f"q=({q0:.3f},{q1:.3f},{q2:.3f},{q3:.3f})",
        f"pos_ned=({pn:.2f},{pe:.2f},{pd:.2f})",
        f"vel_ned=({vn:.2f},{ve:.2f},{vd:.2f})",
    )


def dump_controller(payload: bytes) -> None:
    (
        magic,
        version,
        payload_bytes,
        seq,
        t_ns,
        armed,
        s0,
        s1,
        s2,
        s3,
        _crc,
    ) = CONTROLLER_MSG_STRUCT.unpack(payload)
    if magic != MESSAGE_MAGIC or version != MESSAGE_VERSION:
        return
    print(
        "controller",
        f"seq={seq}",
        f"t_ns={t_ns}",
        f"armed={armed}",
        f"servo_norm=({s0:.3f},{s1:.3f},{s2:.3f},{s3:.3f})",
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Dump runtime IPC mailboxes.")
    parser.add_argument(
        "--type",
        choices=("sensor", "estimator", "controller"),
        required=True,
        help="Mailbox type to dump.",
    )
    parser.add_argument(
        "--name",
        default=None,
        help="Override mailbox name (e.g. /rt_sensor_snapshot_v1).",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.05,
        help="Polling interval in seconds.",
    )
    args = parser.parse_args()

    if args.type == "sensor":
        name = args.name or "/rt_sensor_snapshot_v1"
        struct_size = SENSOR_MSG_STRUCT.size
        printer = dump_sensor
    elif args.type == "estimator":
        name = args.name or "/rt_estimator_state_v1"
        struct_size = ESTIMATOR_MSG_STRUCT.size
        printer = dump_estimator
    else:
        name = args.name or "/rt_controller_command_v1"
        struct_size = CONTROLLER_MSG_STRUCT.size
        printer = dump_controller

    mb = ShmMailbox(MailboxConfig(name, retries=200, retry_sleep_s=0.05), struct_size)
    mb.open_existing()

    try:
        while True:
            payload = mb.try_read()
            if payload is not None:
                printer(payload)
            time.sleep(max(0.0, args.interval))
    finally:
        mb.close()


if __name__ == "__main__":
    raise SystemExit(main())
