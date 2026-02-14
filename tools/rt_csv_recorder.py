#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import signal
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
        finalize_crc,
        monotonic_ns,
        parse_int,
        parse_simple_toml,
        parse_string,
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
        finalize_crc,
        monotonic_ns,
        parse_int,
        parse_simple_toml,
        parse_string,
    )

SENSOR_PAYLOAD_BYTES = 88
ESTIMATOR_PAYLOAD_BYTES = 64
CONTROLLER_PAYLOAD_BYTES = 40


CSV_COLUMNS = [
    "host_mono_ns",
    "host_wall_s",
    "updated_sensor",
    "updated_estimator",
    "updated_controller",
    "sensor_seq",
    "sensor_t_ns",
    "sensor_crc_ok",
    "sensor_imu_valid",
    "sensor_baro_valid",
    "sensor_ax_mps2",
    "sensor_ay_mps2",
    "sensor_az_mps2",
    "sensor_gx_rads",
    "sensor_gy_rads",
    "sensor_gz_rads",
    "sensor_pressure_pa",
    "sensor_temp_c",
    "est_seq",
    "est_t_ns",
    "est_crc_ok",
    "est_valid",
    "est_qw",
    "est_qx",
    "est_qy",
    "est_qz",
    "est_vn_mps",
    "est_ve_mps",
    "est_vd_mps",
    "est_pn_m",
    "est_pe_m",
    "est_pd_m",
    "ctrl_seq",
    "ctrl_t_ns",
    "ctrl_crc_ok",
    "ctrl_armed",
    "ctrl_s0_norm",
    "ctrl_s1_norm",
    "ctrl_s2_norm",
    "ctrl_s3_norm",
]


def parse_sensor(payload: bytes) -> dict[str, object] | None:
    values = SENSOR_MSG_STRUCT.unpack(payload)
    (
        magic,
        version,
        payload_bytes,
        seq,
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
    if magic != MESSAGE_MAGIC or version != MESSAGE_VERSION or payload_bytes != SENSOR_PAYLOAD_BYTES:
        return None
    crc_ok = int(crc == finalize_crc(payload[:-4]))
    return {
        "sensor_seq": seq,
        "sensor_t_ns": t_ns,
        "sensor_crc_ok": crc_ok,
        "sensor_imu_valid": imu_valid,
        "sensor_baro_valid": baro_valid,
        "sensor_ax_mps2": ax,
        "sensor_ay_mps2": ay,
        "sensor_az_mps2": az,
        "sensor_gx_rads": gx,
        "sensor_gy_rads": gy,
        "sensor_gz_rads": gz,
        "sensor_pressure_pa": pressure_pa,
        "sensor_temp_c": temp_c,
    }


def parse_estimator(payload: bytes) -> dict[str, object] | None:
    values = ESTIMATOR_MSG_STRUCT.unpack(payload)
    (
        magic,
        version,
        payload_bytes,
        seq,
        t_ns,
        valid,
        qw,
        qx,
        qy,
        qz,
        vn,
        ve,
        vd,
        pn,
        pe,
        pd,
        crc,
    ) = values
    if magic != MESSAGE_MAGIC or version != MESSAGE_VERSION or payload_bytes != ESTIMATOR_PAYLOAD_BYTES:
        return None
    crc_ok = int(crc == finalize_crc(payload[:-4]))
    return {
        "est_seq": seq,
        "est_t_ns": t_ns,
        "est_crc_ok": crc_ok,
        "est_valid": valid,
        "est_qw": qw,
        "est_qx": qx,
        "est_qy": qy,
        "est_qz": qz,
        "est_vn_mps": vn,
        "est_ve_mps": ve,
        "est_vd_mps": vd,
        "est_pn_m": pn,
        "est_pe_m": pe,
        "est_pd_m": pd,
    }


def parse_controller(payload: bytes) -> dict[str, object] | None:
    values = CONTROLLER_MSG_STRUCT.unpack(payload)
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
        crc,
    ) = values
    if magic != MESSAGE_MAGIC or version != MESSAGE_VERSION or payload_bytes != CONTROLLER_PAYLOAD_BYTES:
        return None
    crc_ok = int(crc == finalize_crc(payload[:-4]))
    return {
        "ctrl_seq": seq,
        "ctrl_t_ns": t_ns,
        "ctrl_crc_ok": crc_ok,
        "ctrl_armed": armed,
        "ctrl_s0_norm": s0,
        "ctrl_s1_norm": s1,
        "ctrl_s2_norm": s2,
        "ctrl_s3_norm": s3,
    }


def mailbox_names_from_args(args: argparse.Namespace) -> tuple[str, str, str, int, int]:
    retry_ms = 50
    retry_count = 200

    if args.config:
        cfg = parse_simple_toml(args.config)
        ipc = cfg["ipc"]
        retry_ms = parse_int(ipc.get("open_retry_ms", "50"))
        retry_count = parse_int(ipc.get("open_retry_count", "200"))
        sensor_name = parse_string(ipc["sensor_snapshot_shm"])
        estimator_name = parse_string(ipc["estimator_state_shm"])
        controller_name = parse_string(ipc["controller_command_shm"])
    else:
        sensor_name = "/rt_sensor_snapshot_v1"
        estimator_name = "/rt_estimator_state_v1"
        controller_name = "/rt_controller_command_v1"

    if args.sensor_name:
        sensor_name = args.sensor_name
    if args.estimator_name:
        estimator_name = args.estimator_name
    if args.controller_name:
        controller_name = args.controller_name

    return sensor_name, estimator_name, controller_name, retry_ms, retry_count


def main() -> int:
    parser = argparse.ArgumentParser(description="Record runtime IPC snapshots to CSV for offline debugging.")
    parser.add_argument("--config", default=None, help="Runtime TOML config to load mailbox names/retry settings.")
    parser.add_argument("--output", type=Path, required=True, help="CSV output path.")
    parser.add_argument("--append", action="store_true", help="Append to existing CSV instead of truncating.")
    parser.add_argument("--duration-sec", type=float, default=0.0, help="Stop after this many seconds (0 = run until Ctrl+C).")
    parser.add_argument("--interval", type=float, default=0.01, help="Polling interval in seconds.")
    parser.add_argument("--sensor-name", default=None, help="Override sensor mailbox name.")
    parser.add_argument("--estimator-name", default=None, help="Override estimator mailbox name.")
    parser.add_argument("--controller-name", default=None, help="Override controller mailbox name.")
    args = parser.parse_args()

    if args.interval < 0.0:
        raise SystemExit("--interval must be >= 0")
    if args.duration_sec < 0.0:
        raise SystemExit("--duration-sec must be >= 0")

    sensor_name, estimator_name, controller_name, retry_ms, retry_count = mailbox_names_from_args(args)

    sensor_mb = ShmMailbox(
        MailboxConfig(sensor_name, retries=retry_count, retry_sleep_s=retry_ms / 1000.0),
        SENSOR_MSG_STRUCT.size,
    )
    est_mb = ShmMailbox(
        MailboxConfig(estimator_name, retries=retry_count, retry_sleep_s=retry_ms / 1000.0),
        ESTIMATOR_MSG_STRUCT.size,
    )
    ctrl_mb = ShmMailbox(
        MailboxConfig(controller_name, retries=retry_count, retry_sleep_s=retry_ms / 1000.0),
        CONTROLLER_MSG_STRUCT.size,
    )

    sensor_mb.open_existing()
    est_mb.open_existing()
    ctrl_mb.open_existing()

    running = True

    def _stop(_sig: int, _frame: object) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    mode = "a" if args.append else "w"

    latest: dict[str, object] = {}
    prev_sensor_seq: int | None = None
    prev_est_seq: int | None = None
    prev_ctrl_seq: int | None = None
    rows_written = 0

    start_mono = time.monotonic()
    try:
        with args.output.open(mode, newline="", encoding="utf-8") as fh:
            writer = csv.DictWriter(fh, fieldnames=CSV_COLUMNS)
            if not args.append or fh.tell() == 0:
                writer.writeheader()
                fh.flush()

            while running:
                if args.duration_sec > 0 and (time.monotonic() - start_mono) >= args.duration_sec:
                    break

                updated_sensor = 0
                updated_est = 0
                updated_ctrl = 0

                sensor_payload = sensor_mb.try_read()
                if sensor_payload is not None:
                    parsed = parse_sensor(sensor_payload)
                    if parsed is not None:
                        seq = int(parsed["sensor_seq"])
                        if prev_sensor_seq is None or seq != prev_sensor_seq:
                            latest.update(parsed)
                            prev_sensor_seq = seq
                            updated_sensor = 1

                est_payload = est_mb.try_read()
                if est_payload is not None:
                    parsed = parse_estimator(est_payload)
                    if parsed is not None:
                        seq = int(parsed["est_seq"])
                        if prev_est_seq is None or seq != prev_est_seq:
                            latest.update(parsed)
                            prev_est_seq = seq
                            updated_est = 1

                ctrl_payload = ctrl_mb.try_read()
                if ctrl_payload is not None:
                    parsed = parse_controller(ctrl_payload)
                    if parsed is not None:
                        seq = int(parsed["ctrl_seq"])
                        if prev_ctrl_seq is None or seq != prev_ctrl_seq:
                            latest.update(parsed)
                            prev_ctrl_seq = seq
                            updated_ctrl = 1

                if updated_sensor or updated_est or updated_ctrl:
                    row: dict[str, object] = {k: latest.get(k, "") for k in CSV_COLUMNS}
                    row["host_mono_ns"] = monotonic_ns()
                    row["host_wall_s"] = f"{time.time():.6f}"
                    row["updated_sensor"] = updated_sensor
                    row["updated_estimator"] = updated_est
                    row["updated_controller"] = updated_ctrl
                    writer.writerow(row)
                    rows_written += 1
                    if rows_written % 100 == 0:
                        fh.flush()

                time.sleep(args.interval)
            fh.flush()
    finally:
        sensor_mb.close()
        est_mb.close()
        ctrl_mb.close()

    print(f"Wrote {rows_written} rows to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
