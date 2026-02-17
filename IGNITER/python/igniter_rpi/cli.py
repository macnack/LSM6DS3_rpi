from __future__ import annotations

import argparse
import sys
from typing import Iterable

from . import (
    ChannelConfig,
    FaultPolicy,
    Igniter,
    IgniterBank,
    IgniterBankConfig,
    IgniterConfig,
    Vn5Config,
    VN5E160S,
    fault_name,
    state_name,
)


def _parse_u32(value: str) -> int:
    parsed = int(value, 0)
    if parsed < 0:
        raise argparse.ArgumentTypeError("value must be >= 0")
    return parsed


def _parse_channel(value: str) -> int:
    parsed = int(value, 0)
    if parsed < 0 or parsed > 3:
        raise argparse.ArgumentTypeError("channel must be in range 0..3")
    return parsed


def _parse_mask(value: str) -> int:
    parsed = int(value, 0)
    if parsed < 0 or parsed > 0x0F:
        raise argparse.ArgumentTypeError("mask must be in range 0x0..0xF")
    return parsed


def _parse_status_seq(raw: str) -> tuple[int, ...]:
    parts = [part.strip() for part in raw.split(",") if part.strip()]
    if not parts:
        raise argparse.ArgumentTypeError("status sequence cannot be empty")
    out = []
    for part in parts:
        if part not in ("0", "1"):
            raise argparse.ArgumentTypeError("status sequence values must be 0 or 1")
        out.append(int(part))
    return tuple(out)


def _build_vn5_config(args: argparse.Namespace) -> Vn5Config:
    cfg = Vn5Config()
    cfg.settle_ms = args.settle_ms
    cfg.latch_faults = args.latch_faults
    return cfg


def _run_vn5(args: argparse.Namespace) -> int:
    cfg = _build_vn5_config(args)
    dev = VN5E160S(cfg)

    dev.init(0)
    dev.set(args.initial_on, 0)

    print("t_ms,status_ok,is_on,ok,fault")
    t_ms = 0
    for sample in args.status_seq:
        t_ms += args.step_ms
        dev.update(bool(sample), t_ms)
        print(f"{t_ms},{sample},{int(dev.is_on())},{int(dev.ok())},{fault_name(dev.fault())}")

    if args.clear_fault_at_end:
        dev.clear_fault()
        print(f"after_clear_fault,ok={int(dev.ok())},fault={fault_name(dev.fault())}")

    return 0


def _build_igniter(args: argparse.Namespace) -> Igniter:
    ign_cfg = IgniterConfig()
    ign_cfg.max_fire_ms = args.max_fire_ms
    ign_cfg.default_fire_ms = args.default_fire_ms
    return Igniter(ign_cfg, _build_vn5_config(args))


def _run_igniter(args: argparse.Namespace) -> int:
    ign = _build_igniter(args)
    ign.init(0)

    if not ign.arm(0):
        print("arm failed", file=sys.stderr)
        return 1

    if not ign.fire(args.duration_ms, 0):
        print("fire failed", file=sys.stderr)
        return 1

    print("t_ms,status_ok,state,fault,remaining_ms")
    t_ms = 0
    for sample in args.status_seq:
        t_ms += args.step_ms
        ign.update(bool(sample), t_ms)
        print(
            f"{t_ms},{sample},{state_name(ign.state())},{fault_name(ign.fault())},{ign.remaining_ms(t_ms)}"
        )

    if args.clear_fault_at_end:
        ign.clear_fault(t_ms)
        print(f"after_clear_fault,state={state_name(ign.state())},fault={fault_name(ign.fault())}")

    return 0


def _build_bank_config(args: argparse.Namespace) -> IgniterBankConfig:
    cfg = IgniterBankConfig()
    cfg.fault_policy = FaultPolicy.GLOBAL if args.fault_policy == "global" else FaultPolicy.ISOLATED

    channels = []
    for _ in range(4):
        ch = ChannelConfig()
        ch.enabled = True
        ch.igniter.max_fire_ms = args.max_fire_ms
        ch.igniter.default_fire_ms = args.default_fire_ms
        ch.driver.settle_ms = args.settle_ms
        ch.driver.latch_faults = args.latch_faults
        channels.append(ch)
    cfg.channels = channels
    return cfg


def _fmt_values(values: Iterable[object]) -> str:
    return "[" + ",".join(str(v) for v in values) + "]"


def _run_bank(args: argparse.Namespace) -> int:
    bank = IgniterBank(_build_bank_config(args))
    bank.init(0)

    if not bank.arm(0):
        print("arm failed", file=sys.stderr)
        return 1

    durations = (args.duration_ms, args.duration_ms, args.duration_ms, args.duration_ms)
    if not bank.fire_mask(args.mask, durations, 0):
        print("fire_mask failed", file=sys.stderr)
        return 1

    print("t_ms,active_mask,armed,global_fault,states,faults,remaining_ms")
    for step in range(1, args.steps + 1):
        t_ms = step * args.step_ms
        statuses = [1, 1, 1, 1]
        if args.fault_channel is not None and step >= args.fault_step:
            statuses[args.fault_channel] = 0

        bank.set_status_values((statuses[0], statuses[1], statuses[2], statuses[3]))
        snap = bank.update(t_ms, False)

        state_names = [state_name(s) for s in snap.states]
        fault_names = [fault_name(f) for f in snap.faults]
        print(
            f"{t_ms},0x{snap.active_mask:01X},{int(snap.armed)},{int(snap.global_fault_latched)},"
            f"{_fmt_values(state_names)},{_fmt_values(fault_names)},{_fmt_values(snap.remaining_ms)}"
        )

    return 0


def _add_common_timing_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--settle-ms", type=_parse_u32, default=5)
    parser.add_argument("--latch-faults", action=argparse.BooleanOptionalAction, default=True)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="IGNITER simulation CLI for VN5E160S, Igniter and IgniterBank"
    )
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_vn5 = sub.add_parser("vn5e160s", help="Run VN5E160S low-level simulation")
    _add_common_timing_args(p_vn5)
    p_vn5.add_argument("--initial-on", action=argparse.BooleanOptionalAction, default=True)
    p_vn5.add_argument("--status-seq", type=_parse_status_seq, default=(1, 1, 1, 0, 1))
    p_vn5.add_argument("--step-ms", type=_parse_u32, default=5)
    p_vn5.add_argument("--clear-fault-at-end", action="store_true")
    p_vn5.set_defaults(func=_run_vn5)

    p_ign = sub.add_parser("igniter", help="Run single Igniter state-machine simulation")
    _add_common_timing_args(p_ign)
    p_ign.add_argument("--default-fire-ms", type=_parse_u32, default=200)
    p_ign.add_argument("--max-fire-ms", type=_parse_u32, default=2000)
    p_ign.add_argument("--duration-ms", type=_parse_u32, default=200)
    p_ign.add_argument("--status-seq", type=_parse_status_seq, default=(1, 1, 1, 1, 1, 1, 1))
    p_ign.add_argument("--step-ms", type=_parse_u32, default=40)
    p_ign.add_argument("--clear-fault-at-end", action="store_true")
    p_ign.set_defaults(func=_run_igniter)

    p_bank = sub.add_parser("bank", help="Run 4-channel IgniterBank simulation")
    _add_common_timing_args(p_bank)
    p_bank.add_argument("--fault-policy", choices=("global", "isolated"), default="global")
    p_bank.add_argument("--mask", type=_parse_mask, default=0x0F)
    p_bank.add_argument("--duration-ms", type=_parse_u32, default=250)
    p_bank.add_argument("--default-fire-ms", type=_parse_u32, default=200)
    p_bank.add_argument("--max-fire-ms", type=_parse_u32, default=2000)
    p_bank.add_argument("--steps", type=_parse_u32, default=12)
    p_bank.add_argument("--step-ms", type=_parse_u32, default=30)
    p_bank.add_argument("--fault-channel", type=_parse_channel, default=None)
    p_bank.add_argument("--fault-step", type=_parse_u32, default=5)
    p_bank.set_defaults(func=_run_bank)

    args = parser.parse_args()
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
