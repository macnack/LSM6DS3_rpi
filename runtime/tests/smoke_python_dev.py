#!/usr/bin/env python3
from __future__ import annotations

import argparse
import subprocess
import tempfile
import time
from pathlib import Path


def parse_status_file(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    for raw in path.read_text(encoding="utf-8").splitlines():
        if "=" not in raw:
            continue
        k, v = raw.split("=", 1)
        values[k.strip()] = v.strip()
    return values


def main() -> int:
    parser = argparse.ArgumentParser(description="Smoke test: rt_core + Python dummy workers")
    parser.add_argument("--rt-core", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--dummy-estimator", required=True)
    parser.add_argument("--dummy-controller", required=True)
    args = parser.parse_args()

    with tempfile.TemporaryDirectory(prefix="rt_smoke_") as td:
      status_file = Path(td) / "status.txt"

      rt_cmd = [
          args.rt_core,
          "--config",
          args.config,
          "--duration-sec",
          "5",
          "--status-file",
          str(status_file),
      ]

      rt_proc = subprocess.Popen(rt_cmd)
      try:
          time.sleep(0.8)

          est_proc = subprocess.Popen(
              [
                  "python3",
                  args.dummy_estimator,
                  "--config",
                  args.config,
                  "--duration-sec",
                  "4",
              ]
          )
          ctrl_proc = subprocess.Popen(
              [
                  "python3",
                  args.dummy_controller,
                  "--config",
                  args.config,
                  "--duration-sec",
                  "4",
              ]
          )

          est_rc = est_proc.wait(timeout=20)
          ctrl_rc = ctrl_proc.wait(timeout=20)
          rt_rc = rt_proc.wait(timeout=20)
      finally:
          if rt_proc.poll() is None:
              rt_proc.terminate()

      if rt_rc != 0:
          raise RuntimeError(f"rt_core exited with {rt_rc}")
      if est_rc != 0:
          raise RuntimeError(f"dummy_estimator exited with {est_rc}")
      if ctrl_rc != 0:
          raise RuntimeError(f"dummy_controller exited with {ctrl_rc}")

      if not status_file.exists():
          raise RuntimeError("status file was not created by rt_core")

      s = parse_status_file(status_file)
      required = [
          "control_ticks",
          "external_estimator_accept_count",
          "external_controller_accept_count",
          "degraded_mode_active",
          "last_failsafe_reason_name",
      ]
      for key in required:
          if key not in s:
              raise RuntimeError(f"Missing '{key}' in status file")

      if int(s["control_ticks"]) <= 0:
          raise RuntimeError("control_ticks did not advance")
      if int(s["external_estimator_accept_count"]) <= 0:
          raise RuntimeError("external_estimator_accept_count did not advance")
      if int(s["external_controller_accept_count"]) <= 0:
          raise RuntimeError("external_controller_accept_count did not advance")
      if s["degraded_mode_active"].lower() != "false":
          raise RuntimeError("degraded_mode_active unexpectedly true in smoke run")

      print("runtime_smoke_python_dev: ok")
      return 0


if __name__ == "__main__":
    raise SystemExit(main())
