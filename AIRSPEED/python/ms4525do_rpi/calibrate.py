import argparse
import json
import math
import os
import statistics
import sys
import tempfile
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from . import Calibration, Ms4525do, OutputType, Status
from .cli import _parse_address, _parse_output_type


def _parse_positive_int(value: str) -> int:
  try:
    parsed = int(value, 0)
  except ValueError as exc:
    raise argparse.ArgumentTypeError(f"invalid integer '{value}'") from exc
  if parsed <= 0:
    raise argparse.ArgumentTypeError("value must be > 0")
  return parsed


def _parse_positive_float(value: str) -> float:
  try:
    parsed = float(value)
  except ValueError as exc:
    raise argparse.ArgumentTypeError(f"invalid float '{value}'") from exc
  if not math.isfinite(parsed) or parsed <= 0:
    raise argparse.ArgumentTypeError("value must be a finite number > 0")
  return parsed


def _build_parser() -> argparse.ArgumentParser:
  parser = argparse.ArgumentParser(
      description="Calibrate airspeed differential pressure sensors"
  )
  parser.add_argument("--sensor", default="ms4525do", choices=("ms4525do",),
                      help="Sensor backend to calibrate")
  parser.add_argument("--bus", default="/dev/i2c-1", help="I2C bus path for MS4525DO")
  parser.add_argument("--address", type=_parse_address, default=0x28,
                      help="I2C address (hex like 0x28)")
  parser.add_argument("--p-min-psi", type=float, default=-1.0,
                      help="Pressure transfer function minimum pressure (psi)")
  parser.add_argument("--p-max-psi", type=float, default=1.0,
                      help="Pressure transfer function maximum pressure (psi)")
  parser.add_argument("--output-type", type=_parse_output_type, default=OutputType.TYPE_B_5_TO_95,
                      help="Transfer function output type: a (10%%..90%%) or b (5%%..95%%)")
  parser.add_argument("--zero-samples", type=_parse_positive_int, default=300,
                      help="Number of at-rest samples to average for zero-offset")
  parser.add_argument("--verify-threshold-pa", type=_parse_positive_float, default=50.0,
                      help="Required corrected pressure magnitude for positive-pressure verification")
  parser.add_argument("--verify-timeout-sec", type=_parse_positive_float, default=30.0,
                      help="Timeout waiting for pressure verification")
  parser.add_argument("--sample-hz", type=_parse_positive_float, default=100.0,
                      help="Sampling rate used during calibration")
  parser.add_argument("--non-interactive", action="store_true",
                      help="Skip interactive prompt before pressure-direction verification")
  parser.add_argument("--json", action="store_true",
                      help="Print final calibration as JSON only")
  parser.add_argument("--reverse-state-file", default="output/airspeed_reverse_state.json",
                      help="Path to persist reverse-port flag state")
  parser.add_argument("--persist-reverse-on-negative", action=argparse.BooleanOptionalAction,
                      default=True,
                      help="Persist reverse-port flag when negative direction is detected")
  parser.add_argument("--experimental", action="store_true",
                      help="Enable robust statistical estimators for noisy samples")
  parser.add_argument("--allow-stale", action="store_true",
                      help="Accept stale samples during calibration (diagnostic mode only)")
  return parser


def _status_is_usable(status: Any, allow_stale: bool) -> bool:
  if status == Status.NORMAL:
    return True
  return allow_stale and status == Status.STALE_DATA


def _log(args: argparse.Namespace, message: str, *, error: bool = False) -> None:
  if args.json and not error:
    return
  stream = sys.stderr if error else sys.stdout
  print(message, file=stream)


def _create_ms4525do_sensor(args: argparse.Namespace) -> Ms4525do:
  if args.p_max_psi <= args.p_min_psi:
    raise ValueError("--p-max-psi must be greater than --p-min-psi")

  calibration = Calibration()
  calibration.p_min_psi = args.p_min_psi
  calibration.p_max_psi = args.p_max_psi
  calibration.output_type = args.output_type

  return Ms4525do(bus_path=args.bus, address=args.address, calibration=calibration)


def _collect_zero_offset_pa(sensor: Ms4525do, samples: int, period_s: float,
                            experimental: bool, allow_stale: bool) -> tuple[float, dict[str, Any]]:
  collected = 0
  usable_pressures: list[float] = []
  skipped_unusable = 0
  while collected < samples:
    reading = sensor.read()
    if _status_is_usable(reading.status, allow_stale):
      usable_pressures.append(reading.pressure_pa)
      collected += 1
    else:
      skipped_unusable += 1
    time.sleep(period_s)

  if experimental:
    median = statistics.median(usable_pressures)
    abs_dev = [abs(v - median) for v in usable_pressures]
    mad = statistics.median(abs_dev)

    if mad > 0.0:
      robust_sigma = 1.4826 * mad
      filtered = [v for v in usable_pressures if abs(v - median) <= (3.0 * robust_sigma)]
    else:
      filtered = list(usable_pressures)

    filtered_sorted = sorted(filtered)
    trim = int(0.1 * len(filtered_sorted))
    if len(filtered_sorted) - (2 * trim) >= 3:
      trimmed = filtered_sorted[trim:len(filtered_sorted) - trim]
    else:
      trimmed = filtered_sorted

    offset = statistics.fmean(trimmed)
    stats = {
        "mode": "experimental",
        "raw_samples": len(usable_pressures),
        "filtered_samples": len(filtered),
        "trimmed_samples": len(trimmed),
        "median_pa": median,
        "mad_pa": mad,
        "skipped_unusable": skipped_unusable,
    }
  else:
    offset = statistics.fmean(usable_pressures)
    stats = {
        "mode": "standard",
        "raw_samples": len(usable_pressures),
        "skipped_unusable": skipped_unusable,
    }

  if abs(offset) < 1e-8:
    offset = 1e-8
  return offset, stats


def _verify_positive_direction(sensor: Ms4525do, offset_pa: float, threshold_pa: float,
                               timeout_sec: float, period_s: float,
                               experimental: bool, allow_stale: bool) -> tuple[str, float, int]:
  start = time.monotonic()
  latest_corrected = 0.0
  usable_samples = 0
  rolling: list[float] = []
  rolling_size = 15
  while time.monotonic() - start < timeout_sec:
    reading = sensor.read()
    if _status_is_usable(reading.status, allow_stale):
      usable_samples += 1
      latest_corrected = reading.pressure_pa - offset_pa

      if not experimental:
        if latest_corrected >= threshold_pa:
          return "positive", latest_corrected, usable_samples
        if latest_corrected <= -threshold_pa:
          return "negative", latest_corrected, usable_samples
      else:
        rolling.append(latest_corrected)
        if len(rolling) > rolling_size:
          rolling.pop(0)

        if latest_corrected <= -1.5 * threshold_pa:
          return "negative", latest_corrected, usable_samples

        if len(rolling) >= 10:
          mean_corr = statistics.fmean(rolling)
          if mean_corr >= threshold_pa:
            return "positive", mean_corr, usable_samples
          if mean_corr <= -threshold_pa:
            return "negative", mean_corr, usable_samples
    time.sleep(period_s)
  return "timeout", latest_corrected, usable_samples


def _persist_reverse_state(path: str, result: dict[str, Any]) -> None:
  state_path = Path(path)
  state_path.parent.mkdir(parents=True, exist_ok=True)
  fd = -1
  tmp_path = ""
  try:
    fd, tmp_path = tempfile.mkstemp(prefix=f".{state_path.name}.", suffix=".tmp",
                                    dir=str(state_path.parent))
    with os.fdopen(fd, "w", encoding="utf-8") as f:
      json.dump(result, f, indent=2, sort_keys=True)
      f.write("\n")
    fd = -1
    os.replace(tmp_path, state_path)
    tmp_path = ""
  finally:
    if fd != -1:
      os.close(fd)
    if tmp_path and os.path.exists(tmp_path):
      os.unlink(tmp_path)


def main() -> int:
  parser = _build_parser()
  args = parser.parse_args()

  try:
    sensor = _create_ms4525do_sensor(args)
  except ValueError as exc:
    parser.error(str(exc))

  period_s = 1.0 / args.sample_hz

  _log(args, "[cal] Starting airspeed calibration")
  if args.allow_stale:
    _log(args, "[cal] WARN: --allow-stale enabled; stale samples may reduce calibration quality")
  _log(args, "[cal] Keep pitot/static ports still: collecting zero-pressure baseline")

  offset_pa = 0.0
  offset_stats: dict[str, Any] = {}
  outcome = "exception"
  measured_pa = 0.0
  verify_samples = 0
  failure_reason = "exception"
  exception_message = ""

  try:
    sensor.begin()
    offset_pa, offset_stats = _collect_zero_offset_pa(
        sensor, args.zero_samples, period_s, args.experimental, args.allow_stale
    )
    _log(args, f"[cal] Zero offset estimate: {offset_pa:.6f} Pa ({offset_stats['mode']})")

    if not args.non_interactive and not args.json:
      input("[cal] Blow into the dynamic port and press Enter to continue...")

    outcome, measured_pa, verify_samples = _verify_positive_direction(
        sensor=sensor,
        offset_pa=offset_pa,
        threshold_pa=args.verify_threshold_pa,
        timeout_sec=args.verify_timeout_sec,
        period_s=period_s,
        experimental=args.experimental,
        allow_stale=args.allow_stale,
    )
  except Exception as exc:
    exception_message = str(exc)
    _log(args, f"[cal] FAILED: exception during calibration: {exception_message}", error=True)
  finally:
    try:
      sensor.close()
    except Exception as exc:
      _log(args, f"[cal] WARN: failed to close sensor cleanly: {exc}", error=True)

  reverse_ports = False
  if outcome == "positive":
    failure_reason = "none"
    _log(args, f"[cal] Positive pressure check: OK ({measured_pa:.3f} Pa)")
  elif outcome == "negative":
    reverse_ports = True
    failure_reason = "negative_direction"
    _log(args, f"[cal] FAILED: negative pressure difference detected ({measured_pa:.3f} Pa)", error=True)
    _log(args, "[cal] FAILED: static/dynamic ports appear reversed.", error=True)
    _log(args, "[cal] FAILED: swap ports or apply reverse flag in your flight stack.", error=True)
  elif outcome == "timeout":
    failure_reason = "verify_timeout"
    _log(args, f"[cal] FAILED: verification timed out (last corrected={measured_pa:.3f} Pa)", error=True)
    _log(args, "[cal] FAILED: could not produce enough positive differential pressure.", error=True)
  else:
    failure_reason = "exception"

  result = {
      "sensor": args.sensor,
      "timestamp_utc": datetime.now(timezone.utc).isoformat(),
      "offset_pa": offset_pa,
      "verify_threshold_pa": args.verify_threshold_pa,
      "measured_corrected_pa": measured_pa,
      "bus": args.bus,
      "address": int(args.address),
      "p_min_psi": args.p_min_psi,
      "p_max_psi": args.p_max_psi,
      "output_type": "a" if args.output_type == OutputType.TYPE_A_10_TO_90 else "b",
      "verify_outcome": outcome,
      "verify_samples": verify_samples,
      "experimental": bool(args.experimental),
      "allow_stale": bool(args.allow_stale),
      "offset_stats": offset_stats,
      "reverse_ports": reverse_ports,
      "reverse_state_file": args.reverse_state_file,
      "failure_reason": failure_reason,
  }
  if exception_message:
    result["exception"] = exception_message

  if outcome == "positive":
    reverse_state = {
        "sensor": args.sensor,
        "timestamp_utc": result["timestamp_utc"],
        "reverse_ports": False,
        "reason": "calibrated_ok",
        "bus": args.bus,
        "address": int(args.address),
    }
    try:
      _persist_reverse_state(args.reverse_state_file, reverse_state)
      _log(args, f"[cal] Persisted reverse flag at {args.reverse_state_file} (reverse_ports=false)")
    except Exception as exc:
      _log(args, f"[cal] FAILED: could not persist reverse flag: {exc}", error=True)
  elif reverse_ports and args.persist_reverse_on_negative:
    reverse_state = {
        "sensor": args.sensor,
        "timestamp_utc": result["timestamp_utc"],
        "reverse_ports": True,
        "reason": failure_reason,
        "bus": args.bus,
        "address": int(args.address),
    }
    try:
      _persist_reverse_state(args.reverse_state_file, reverse_state)
      _log(args, f"[cal] Persisted reverse flag at {args.reverse_state_file} (reverse_ports=true)")
    except Exception as exc:
      _log(args, f"[cal] FAILED: could not persist reverse flag: {exc}", error=True)

  if args.json:
    print(json.dumps(result))
  else:
    _log(args, f"[cal] RESULT: {json.dumps(result)}")
  if outcome != "positive":
    _log(args, f"[cal] FAILED SUMMARY: reason={failure_reason} outcome={outcome}", error=True)

  return 0 if outcome == "positive" else 2


if __name__ == "__main__":
  sys.exit(main())
