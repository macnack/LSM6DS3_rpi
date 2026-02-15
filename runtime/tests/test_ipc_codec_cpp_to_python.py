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
    from runtime.python.ipc_common import decode_sensor_snapshot  # noqa: WPS433

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

    print("runtime_unit_ipc_codec_cpp_to_python: ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
