import argparse
import math
import signal
import sys
import time

from . import Lsm6ds3, Lsm6ds3Spi


def _parse_address(address_arg: str) -> int:
  try:
    address = int(address_arg, 0)
  except ValueError as exc:
    raise argparse.ArgumentTypeError(
        f"invalid I2C address '{address_arg}'; use values like 0x6A"
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


def _parse_spi_speed(speed_arg: str) -> int:
  try:
    speed = int(speed_arg, 0)
  except ValueError as exc:
    raise argparse.ArgumentTypeError(f"invalid --spi-speed value '{speed_arg}'") from exc

  if speed <= 0:
    raise argparse.ArgumentTypeError("--spi-speed must be a positive integer")

  return speed


def _parse_spi_mode(mode_arg: str) -> int:
  try:
    mode = int(mode_arg, 0)
  except ValueError as exc:
    raise argparse.ArgumentTypeError(f"invalid --spi-mode value '{mode_arg}'") from exc

  if mode < 0 or mode > 3:
    raise argparse.ArgumentTypeError("--spi-mode must be 0, 1, 2, or 3")

  return mode


def main() -> int:
  parser = argparse.ArgumentParser(description="Stream LSM6DS3 accel/gyro values")
  parser.add_argument("--interface", choices=("i2c", "spi"), default="i2c",
                      help="Bus interface to use")
  parser.add_argument("--bus", default="/dev/i2c-1", help="I2C bus path (for --interface i2c)")
  parser.add_argument("--address", type=_parse_address, default=0x6A,
                      help="I2C address (hex like 0x6A, for --interface i2c)")
  parser.add_argument("--spi-device", default="/dev/spidev0.0",
                      help="SPI device path (for --interface spi)")
  parser.add_argument("--spi-speed", type=_parse_spi_speed, default=5000000,
                      help="SPI clock speed in Hz (for --interface spi)")
  parser.add_argument("--spi-mode", type=_parse_spi_mode, default=3,
                      help="SPI mode 0..3 (for --interface spi)")
  parser.add_argument("--hz", type=_parse_hz, default=20.0, help="Output streaming rate")
  args = parser.parse_args()

  period = 1.0 / args.hz
  running = True

  def handle_sigint(_signum, _frame):
    nonlocal running
    running = False

  signal.signal(signal.SIGINT, handle_sigint)

  if args.interface == "spi":
    sensor = Lsm6ds3Spi(device_path=args.spi_device, speed_hz=args.spi_speed, mode=args.spi_mode)
  else:
    sensor = Lsm6ds3(bus_path=args.bus, address=args.address)
  try:
    sensor.begin()
    print("timestamp,ax,ay,az,gx,gy,gz")
    while running:
      ts = time.time()
      ax, ay, az = sensor.read_accel()
      gx, gy, gz = sensor.read_gyro()
      print(f"{ts:.6f},{ax:.6f},{ay:.6f},{az:.6f},{gx:.6f},{gy:.6f},{gz:.6f}")
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
