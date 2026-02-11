import argparse
import signal
import sys
import time

from . import Lsm6ds3


def main() -> int:
  parser = argparse.ArgumentParser(description="Stream LSM6DS3 accel/gyro values")
  parser.add_argument("--bus", default="/dev/i2c-1", help="I2C bus path")
  parser.add_argument("--address", default="0x6A", help="I2C address (hex like 0x6A)")
  parser.add_argument("--hz", type=float, default=20.0, help="Output streaming rate")
  args = parser.parse_args()

  address = int(args.address, 0)
  period = 1.0 / args.hz if args.hz > 0 else 0.05
  running = True

  def handle_sigint(_signum, _frame):
    nonlocal running
    running = False

  signal.signal(signal.SIGINT, handle_sigint)

  sensor = Lsm6ds3(bus_path=args.bus, address=address)
  sensor.begin()

  print("timestamp,ax,ay,az,gx,gy,gz")
  try:
    while running:
      ts = time.time()
      ax, ay, az = sensor.read_accel()
      gx, gy, gz = sensor.read_gyro()
      print(f"{ts:.6f},{ax:.6f},{ay:.6f},{az:.6f},{gx:.6f},{gy:.6f},{gz:.6f}")
      time.sleep(period)
  finally:
    sensor.close(power_down=True)

  return 0


if __name__ == "__main__":
  sys.exit(main())
