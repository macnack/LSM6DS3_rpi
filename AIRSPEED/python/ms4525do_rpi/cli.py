import argparse
import math
import signal
import sys
import time

from . import Calibration, Ms4525do, OutputType


def _parse_address(address_arg: str) -> int:
  try:
    address = int(address_arg, 0)
  except ValueError as exc:
    raise argparse.ArgumentTypeError(
        f"invalid I2C address '{address_arg}'; use values like 0x28"
    ) from exc

  if not (0 <= address <= 0x7F):
    raise argparse.ArgumentTypeError(
        f"I2C address out of range (0x00-0x7F): 0x{address:X}"
    )

  return address


def _parse_hz(hz_arg: str) -> float:
  try:
    hz = float(hz_arg)
  except ValueError as exc:
    raise argparse.ArgumentTypeError(f"invalid --hz value '{hz_arg}'") from exc

  if not math.isfinite(hz) or hz <= 0:
    raise argparse.ArgumentTypeError("--hz must be a finite positive number")

  return hz


def _parse_output_type(value: str) -> OutputType:
  lowered = value.strip().lower()
  if lowered == "a":
    return OutputType.TYPE_A_10_TO_90
  if lowered == "b":
    return OutputType.TYPE_B_5_TO_95
  raise argparse.ArgumentTypeError("output type must be 'a' or 'b'")


def main() -> int:
  parser = argparse.ArgumentParser(
      description="Stream MS4525DO differential pressure and temperature values"
  )
  parser.add_argument("--bus", default="/dev/i2c-1", help="I2C bus path")
  parser.add_argument("--address", type=_parse_address, default=0x28,
                      help="I2C address (hex like 0x28)")
  parser.add_argument("--hz", type=_parse_hz, default=20.0, help="Output streaming rate")
  parser.add_argument("--p-min-psi", type=float, default=-1.0,
                      help="Pressure transfer function minimum pressure (psi)")
  parser.add_argument("--p-max-psi", type=float, default=1.0,
                      help="Pressure transfer function maximum pressure (psi)")
  parser.add_argument("--output-type", type=_parse_output_type, default=OutputType.TYPE_B_5_TO_95,
                      help="Transfer function output type: a (10%%..90%%) or b (5%%..95%%)")
  args = parser.parse_args()

  if args.p_max_psi <= args.p_min_psi:
    parser.error("--p-max-psi must be greater than --p-min-psi")

  calibration = Calibration()
  calibration.p_min_psi = args.p_min_psi
  calibration.p_max_psi = args.p_max_psi
  calibration.output_type = args.output_type

  period = 1.0 / args.hz
  running = True

  def handle_sigint(_signum, _frame):
    nonlocal running
    running = False

  signal.signal(signal.SIGINT, handle_sigint)

  sensor = Ms4525do(bus_path=args.bus, address=args.address, calibration=calibration)
  try:
    sensor.begin()
    print("timestamp,pressure_psi,pressure_pa,temperature_c,pressure_counts,temperature_counts,status")
    while running:
      ts = time.time()
      reading = sensor.read()
      print(
          f"{ts:.6f},{reading.pressure_psi:.6f},{reading.pressure_pa:.6f},"
          f"{reading.temperature_c:.6f},{reading.pressure_counts},"
          f"{reading.temperature_counts},{int(reading.status)}"
      )
      time.sleep(period)
  except KeyboardInterrupt:
    return 0
  finally:
    try:
      sensor.close()
    except Exception:
      pass

  return 0


if __name__ == "__main__":
  sys.exit(main())
