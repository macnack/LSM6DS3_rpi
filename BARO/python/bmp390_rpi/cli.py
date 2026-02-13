import argparse
import math
import signal
import sys
import time

from . import Bmp390


def _parse_address(address_arg: str) -> int:
  try:
    address = int(address_arg, 0)
  except ValueError as exc:
    raise argparse.ArgumentTypeError(
        f"invalid I2C address '{address_arg}'; use values like 0x77"
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


def main() -> int:
  parser = argparse.ArgumentParser(description="Stream BMP390 pressure/temperature values")
  parser.add_argument("--bus", default="/dev/i2c-1", help="I2C bus path")
  parser.add_argument("--address", type=_parse_address, default=0x77,
                      help="I2C address (hex like 0x77)")
  parser.add_argument("--hz", type=_parse_hz, default=20.0, help="Output streaming rate")
  args = parser.parse_args()

  period = 1.0 / args.hz
  running = True

  def handle_sigint(_signum, _frame):
    nonlocal running
    running = False

  signal.signal(signal.SIGINT, handle_sigint)

  sensor = Bmp390(bus_path=args.bus, address=args.address)
  try:
    sensor.begin()
    print("timestamp,temperature_c,pressure_pa,pressure_hpa")
    while running:
      ts = time.time()
      reading = sensor.read()
      pressure_hpa = reading.pressure_pa / 100.0
      print(f"{ts:.6f},{reading.temperature_c:.6f},{reading.pressure_pa:.6f},{pressure_hpa:.6f}")
      time.sleep(period)
  except KeyboardInterrupt:
    return 0
  finally:
    try:
      sensor.close(power_down=True)
    except Exception:
      # Avoid masking read/stream exceptions with a shutdown failure.
      pass

  return 0


if __name__ == "__main__":
  sys.exit(main())
