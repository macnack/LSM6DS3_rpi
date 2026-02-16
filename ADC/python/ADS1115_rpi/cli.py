import argparse
import math
import signal
import sys
import time

from . import (
    ADS1115,
    ADS1115_REG_CONFIG_PGA_0_256V,
    ADS1115_REG_CONFIG_PGA_0_512V,
    ADS1115_REG_CONFIG_PGA_1_024V,
    ADS1115_REG_CONFIG_PGA_2_048V,
    ADS1115_REG_CONFIG_PGA_4_096V,
    ADS1115_REG_CONFIG_PGA_6_144V,
)


def _parse_address(address_arg: str) -> int:
    try:
        address = int(address_arg, 0)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            f"invalid I2C address '{address_arg}'; use values like 0x48"
        ) from exc

    if not (0 <= address <= 0x7F):
        raise argparse.ArgumentTypeError(
            f"I2C address out of range (0x00-0x7F): 0x{address:X}"
        )

    return address


def _parse_gain(gain_arg: str) -> int:
    try:
        gain = int(gain_arg, 0)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            f"invalid gain '{gain_arg}'; use one of 0x00,0x02,0x04,0x06,0x08,0x0A"
        ) from exc

    allowed = {
        ADS1115_REG_CONFIG_PGA_6_144V,
        ADS1115_REG_CONFIG_PGA_4_096V,
        ADS1115_REG_CONFIG_PGA_2_048V,
        ADS1115_REG_CONFIG_PGA_1_024V,
        ADS1115_REG_CONFIG_PGA_0_512V,
        ADS1115_REG_CONFIG_PGA_0_256V,
    }
    if gain not in allowed:
        raise argparse.ArgumentTypeError("gain must be one of 0x00,0x02,0x04,0x06,0x08,0x0A")

    return gain


def _parse_hz(hz_arg: str) -> float:
    try:
        hz = float(hz_arg)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"invalid --hz value '{hz_arg}'") from exc

    if not math.isfinite(hz) or hz <= 0:
        raise argparse.ArgumentTypeError("--hz must be a finite positive number")

    return hz


def _parse_count(count_arg: str) -> int:
    try:
        count = int(count_arg, 0)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"invalid --count value '{count_arg}'") from exc

    if count < 0:
        raise argparse.ArgumentTypeError("--count must be >= 0")

    return count


def _should_stop(iteration: int, count: int, running: bool) -> bool:
    if not running:
        return True
    if count == 0:
        return False
    return iteration >= count


def _run_read_voltage(sensor: ADS1115, hz: float, count: int) -> int:
    period = 1.0 / hz
    running = True

    def handle_sigint(_signum, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, handle_sigint)

    i = 0
    while not _should_stop(i, count, running):
        adc0 = sensor.read_voltage(0)["r"]
        adc1 = sensor.read_voltage(1)["r"]
        adc2 = sensor.read_voltage(2)["r"]
        adc3 = sensor.read_voltage(3)["r"]
        print(f"A0:{adc0}mV A1:{adc1}mV A2:{adc2}mV A3:{adc3}mV")
        i += 1
        time.sleep(period)

    return 0


def _run_read_voltage_cascade(sensor1: ADS1115, sensor2: ADS1115, hz: float, count: int) -> int:
    period = 1.0 / hz
    running = True

    def handle_sigint(_signum, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, handle_sigint)

    i = 0
    while not _should_stop(i, count, running):
        adc1_vals = [sensor1.read_voltage(ch)["r"] for ch in range(4)]
        adc2_vals = [sensor2.read_voltage(ch)["r"] for ch in range(4)]
        print(
            f"ADC1 A0:{adc1_vals[0]}mV A1:{adc1_vals[1]}mV A2:{adc1_vals[2]}mV A3:{adc1_vals[3]}mV"
        )
        print(
            f"ADC2 A0:{adc2_vals[0]}mV A1:{adc2_vals[1]}mV A2:{adc2_vals[2]}mV A3:{adc2_vals[3]}mV"
        )
        i += 1
        time.sleep(period)

    return 0


def _run_comparator_voltage(sensor: ADS1115, hz: float, count: int) -> int:
    period = 1.0 / hz
    running = True

    def handle_sigint(_signum, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, handle_sigint)

    labels = {
        0: "A0 and A1",
        1: "A0 and A3",
        2: "A1 and A3",
        3: "A2 and A3",
    }

    i = 0
    while not _should_stop(i, count, running):
        for ch in range(4):
            diff_mv = sensor.comparator_voltage(ch)["r"]
            print(f"The voltage difference between {labels[ch]}: {diff_mv}mV")
        i += 1
        time.sleep(period)

    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run ADS1115 example-style demos (single, cascade, comparator)"
    )
    parser.add_argument(
        "--example",
        choices=("read-voltage", "read-voltage-cascade", "comparator-voltage"),
        default="read-voltage",
        help="Example behavior to run",
    )
    parser.add_argument("--bus", default="/dev/i2c-1", help="I2C bus path")
    parser.add_argument(
        "--address",
        type=_parse_address,
        default=0x48,
        help="Primary ADS1115 I2C address (hex like 0x48)",
    )
    parser.add_argument(
        "--address2",
        type=_parse_address,
        default=0x49,
        help="Second ADS1115 I2C address for cascade example",
    )
    parser.add_argument(
        "--gain",
        type=_parse_gain,
        default=ADS1115_REG_CONFIG_PGA_6_144V,
        help="Gain code: 0x00,0x02,0x04,0x06,0x08,0x0A",
    )
    parser.add_argument("--hz", type=_parse_hz, default=2.0, help="Loop rate")
    parser.add_argument(
        "--count",
        type=_parse_count,
        default=0,
        help="Number of loop iterations (0 means run until Ctrl+C)",
    )
    args = parser.parse_args()

    sensor1 = ADS1115(bus_path=args.bus, address=args.address)
    sensor2 = None

    try:
        sensor1.begin()
        sensor1.set_gain(args.gain)

        if args.example == "read-voltage":
            return _run_read_voltage(sensor1, args.hz, args.count)

        if args.example == "read-voltage-cascade":
            sensor2 = ADS1115(bus_path=args.bus, address=args.address2)
            sensor2.begin()
            sensor2.set_gain(args.gain)
            return _run_read_voltage_cascade(sensor1, sensor2, args.hz, args.count)

        return _run_comparator_voltage(sensor1, args.hz, args.count)

    except KeyboardInterrupt:
        return 0
    finally:
        try:
            sensor1.close()
        except Exception:
            pass
        if sensor2 is not None:
            try:
                sensor2.close()
            except Exception:
                pass


if __name__ == "__main__":
    sys.exit(main())
