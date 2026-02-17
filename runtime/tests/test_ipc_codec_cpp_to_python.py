#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import os
import sys
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--module-dir", required=True)
    args = parser.parse_args()

    module_dir = Path(args.module_dir).resolve()
    repo_root = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(module_dir))
    sys.path.insert(0, str(repo_root))

    import _runtime_ipc_codec as cpp_codec  # noqa: WPS433
    os.environ["RUNTIME_IPC_CODEC_BACKEND"] = "python"
    sys.modules.pop("runtime.python.ipc_common", None)
    from runtime.python.ipc_common import decode_igniter_status, decode_sensor_snapshot  # noqa: WPS433

    blob = cpp_codec.encode_sensor_snapshot(
        seq=42,
        t_ns=123_456_789,
        imu_valid=True,
        baro_valid=True,
        ax_mps2=1.25,
        ay_mps2=-2.5,
        az_mps2=9.81,
        gx_rads=0.11,
        gy_rads=-0.22,
        gz_rads=0.33,
        pressure_pa=100_500.0,
        temperature_c=21.5,
    )

    sample = decode_sensor_snapshot(blob)
    if sample is None:
        raise RuntimeError("Python decode_sensor_snapshot rejected valid C++-encoded sensor payload")

    if sample.seq != 42 or sample.t_ns != 123_456_789:
        raise RuntimeError(f"Unexpected header decode: seq={sample.seq}, t_ns={sample.t_ns}")
    if not sample.imu_valid or not sample.baro_valid:
        raise RuntimeError("Expected imu_valid/baro_valid to be true")
    if not math.isclose(sample.ax_mps2, 1.25, rel_tol=0, abs_tol=1e-9):
        raise RuntimeError(f"ax_mps2 mismatch: {sample.ax_mps2}")
    if not math.isclose(sample.gy_rads, -0.22, rel_tol=0, abs_tol=1e-9):
        raise RuntimeError(f"gy_rads mismatch: {sample.gy_rads}")
    if not math.isclose(sample.temperature_c, 21.5, rel_tol=0, abs_tol=1e-9):
        raise RuntimeError(f"temperature_c mismatch: {sample.temperature_c}")

    ign_blob = cpp_codec.encode_igniter_status(
        seq=77,
        t_ns=987_654,
        armed=True,
        global_fault_latched=False,
        active_mask=0x03,
        s0=2,
        s1=2,
        s2=1,
        s3=1,
        f0=0,
        f1=0,
        f2=3,
        f3=0,
        r0=100,
        r1=100,
        r2=0,
        r3=0,
    )
    ign_status = decode_igniter_status(ign_blob)
    if ign_status is None:
        raise RuntimeError("Python decode_igniter_status rejected valid C++-encoded payload")
    if ign_status.seq != 77 or ign_status.t_ns != 987_654:
        raise RuntimeError(f"Unexpected igniter status header decode: {ign_status}")
    if ign_status.active_mask != 0x03 or ign_status.state[0] != 2 or ign_status.fault[2] != 3:
        raise RuntimeError(f"Unexpected igniter decoded payload: {ign_status}")

    print("runtime_unit_ipc_codec_cpp_to_python: ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
