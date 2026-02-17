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
    from runtime.python.ipc_common import (  # noqa: WPS433
        encode_controller_command,
        encode_estimator_state,
        encode_igniter_command,
    )

    cmd_blob = encode_controller_command(
        seq=9,
        t_ns=555,
        armed=True,
        servo_norm=(0.1, -0.2, 0.3, -0.4),
    )
    cmd = cpp_codec.decode_controller_command(cmd_blob)
    if cmd is None:
        raise RuntimeError("C++ decode_controller_command rejected valid Python-encoded command")
    if cmd["seq"] != 9 or cmd["t_ns"] != 555 or cmd["armed"] is not True:
        raise RuntimeError(f"Unexpected decoded command header: {cmd}")
    if not math.isclose(cmd["servo_norm"][2], 0.3, rel_tol=0, abs_tol=1e-7):
        raise RuntimeError(f"servo_norm[2] mismatch: {cmd['servo_norm'][2]}")

    ign_blob = encode_igniter_command(
        seq=10,
        t_ns=666,
        action=3,
        fire_mask=0x0F,
        duration_ms=(100, 200, 300, 400),
    )
    ign = cpp_codec.decode_igniter_command(ign_blob)
    if ign is None:
        raise RuntimeError("C++ decode_igniter_command rejected valid Python-encoded command")
    if ign["seq"] != 10 or ign["t_ns"] != 666 or ign["action"] != 3 or ign["fire_mask"] != 0x0F:
        raise RuntimeError(f"Unexpected decoded igniter header: {ign}")
    if ign["duration_ms"][3] != 400:
        raise RuntimeError(f"igniter duration mismatch: {ign['duration_ms']}")

    est_blob = encode_estimator_state(
        seq=11,
        t_ns=777,
        valid=True,
        q_body_to_ned=(0.7071067, 0.0, 0.7071067, 0.0),
        vel_ned_mps=(1.0, -2.0, 3.0),
        pos_ned_m=(10.0, 20.0, -30.0),
    )
    est = cpp_codec.decode_estimator_state(est_blob)
    if est is None:
        raise RuntimeError("C++ decode_estimator_state rejected valid Python-encoded estimator payload")
    if est["seq"] != 11 or est["t_ns"] != 777 or est["valid"] is not True:
        raise RuntimeError(f"Unexpected decoded estimator header: {est}")
    if not math.isclose(est["q_body_to_ned"][0], 0.7071067, rel_tol=0, abs_tol=1e-6):
        raise RuntimeError(f"q[0] mismatch: {est['q_body_to_ned'][0]}")
    if not math.isclose(est["pos_ned_m"][2], -30.0, rel_tol=0, abs_tol=1e-6):
        raise RuntimeError(f"pos[2] mismatch: {est['pos_ned_m'][2]}")

    print("runtime_unit_ipc_codec_python_to_cpp: ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
