#!/usr/bin/env python3
from __future__ import annotations

import argparse

from runtime.python.igniter_client import IgniterClient


def _parse_mask(text: str) -> int:
    value = int(text, 0)
    if value < 0 or value > 0x0F:
        raise argparse.ArgumentTypeError("mask must be in [0x0, 0xF]")
    return value


def main() -> int:
    parser = argparse.ArgumentParser(description="Igniter IPC client")
    parser.add_argument("--config", required=True)

    sub = parser.add_subparsers(dest="cmd", required=True)
    sub.add_parser("arm")
    sub.add_parser("disarm")
    sub.add_parser("clear-fault")

    p_fire_all = sub.add_parser("fire-all")
    p_fire_all.add_argument("--duration-ms", type=int, required=True)

    p_fire_one = sub.add_parser("fire-one")
    p_fire_one.add_argument("--channel", type=int, required=True)
    p_fire_one.add_argument("--duration-ms", type=int, required=True)

    p_fire_mask = sub.add_parser("fire-mask")
    p_fire_mask.add_argument("--mask", type=_parse_mask, required=True)
    p_fire_mask.add_argument("--d0", type=int, default=0)
    p_fire_mask.add_argument("--d1", type=int, default=0)
    p_fire_mask.add_argument("--d2", type=int, default=0)
    p_fire_mask.add_argument("--d3", type=int, default=0)

    p_status = sub.add_parser("status")
    p_status.add_argument("--timeout-s", type=float, default=0.5)

    args = parser.parse_args()

    client = IgniterClient.from_toml(args.config)
    client.open()
    try:
        if args.cmd == "arm":
            client.arm()
        elif args.cmd == "disarm":
            client.disarm()
        elif args.cmd == "clear-fault":
            client.clear_fault()
        elif args.cmd == "fire-all":
            client.fire_all(args.duration_ms)
        elif args.cmd == "fire-one":
            client.fire_one(args.channel, args.duration_ms)
        elif args.cmd == "fire-mask":
            client.fire_mask(args.mask, (args.d0, args.d1, args.d2, args.d3))
        elif args.cmd == "status":
            status = client.read_status_blocking(timeout_s=args.timeout_s)
            if status is None:
                print("status: none")
            else:
                print(status)
        else:
            raise RuntimeError(f"unknown command: {args.cmd}")
    finally:
        client.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
