"""CLI tools for buzzer_rpi."""

from __future__ import annotations

import argparse
import json
import signal
import sys
import time
from pathlib import Path

from . import AlarmId, BuzzerEngine, EventId, HardwareBuzzerConfig, Phase, PhaseState, PlayOptions, Priority, BusyPolicy


def _normalize_name(value: str) -> str:
    return "".join(ch for ch in value.upper() if ch.isalnum())


_EVENT_MAP = {
    "ARMING": EventId.ARMING,
    "ARMINGFAILURE": EventId.ARMING_FAILURE,
    "DISARMED": EventId.DISARMED,
    "GYROINITDONE": EventId.GYRO_INIT_DONE,
    "READYTOARM": EventId.READY_TO_ARM,
}

_ALARM_MAP = {
    "BATTERYFAILSAFE": AlarmId.BATTERY_FAILSAFE,
    "EKFFAILURE": AlarmId.EKF_FAILURE,
    "LOSTVEHICLE": AlarmId.LOST_VEHICLE,
    "EVACUATIONTEMPORAL3": AlarmId.EVACUATION_TEMPORAL3,
    "MISSINGSOS": AlarmId.MISSING_SOS,
}

_PRIORITY_MAP = {
    "INFO": Priority.INFO,
    "ALARM": Priority.ALARM,
    "CRITICAL": Priority.CRITICAL,
}

_POLICY_MAP = {
    "QUEUE": BusyPolicy.QUEUE,
    "REPLACE": BusyPolicy.REPLACE,
    "DROPIFBUSY": BusyPolicy.DROP_IF_BUSY,
}


def _parse_pattern(pattern_text: str) -> list[Phase]:
    phases: list[Phase] = []
    for chunk in pattern_text.split(","):
        part = chunk.strip()
        if not part:
            continue
        if ":" not in part:
            raise ValueError(f"invalid phase '{part}', expected STATE:MS")
        state_text, ms_text = part.split(":", 1)
        state_norm = _normalize_name(state_text)
        if state_norm == "ON":
            state = PhaseState.ON
        elif state_norm == "OFF":
            state = PhaseState.OFF
        else:
            raise ValueError(f"invalid phase state '{state_text}'")
        ms = int(ms_text)
        if ms <= 0:
            raise ValueError("phase duration must be > 0 ms")
        phases.append(Phase(state, ms))

    if not phases:
        raise ValueError("pattern must contain at least one phase")
    return phases


def _load_profile_data(path: str) -> dict:
    p = Path(path)
    text = p.read_text(encoding="utf-8")
    suffix = p.suffix.lower()
    if suffix in (".yaml", ".yml"):
        try:
            import yaml  # type: ignore
        except Exception as exc:  # pragma: no cover - environment-specific
            raise RuntimeError("PyYAML is required to load YAML profiles") from exc
        data = yaml.safe_load(text)
    else:
        data = json.loads(text)

    if not isinstance(data, dict):
        raise ValueError("profile root must be a JSON/YAML object")
    return data


def _event_from_str(value: str) -> EventId:
    key = _normalize_name(value)
    if key not in _EVENT_MAP:
        raise ValueError(f"unknown event '{value}'")
    return _EVENT_MAP[key]


def _alarm_from_str(value: str) -> AlarmId:
    key = _normalize_name(value)
    if key not in _ALARM_MAP:
        raise ValueError(f"unknown alarm '{value}'")
    return _ALARM_MAP[key]


def _priority_from_str(value: str) -> Priority:
    key = _normalize_name(value)
    if key not in _PRIORITY_MAP:
        raise ValueError(f"unknown priority '{value}'")
    return _PRIORITY_MAP[key]


def _policy_from_str(value: str) -> BusyPolicy:
    key = _normalize_name(value)
    if key not in _POLICY_MAP:
        raise ValueError(f"unknown policy '{value}'")
    return _POLICY_MAP[key]


def _make_hw_cfg(args: argparse.Namespace) -> HardwareBuzzerConfig:
    cfg = HardwareBuzzerConfig()
    cfg.chip_path = args.chip
    cfg.line = args.line
    cfg.active_high = not args.active_low
    cfg.max_on_time_ms = args.max_on_time_ms
    cfg.simulate = args.simulate
    return cfg


def _maybe_apply_profile(engine: BuzzerEngine, profile_path: str | None) -> None:
    if not profile_path:
        return
    engine.set_profile_from_dict(_load_profile_data(profile_path))


def _wait_until_idle(engine: BuzzerEngine, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s if timeout_s > 0 else None
    while engine.is_playing():
        if deadline is not None and time.monotonic() >= deadline:
            return False
        time.sleep(0.01)
    return True


def _build_play_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="buzzer-play", description="Play a buzzer pattern")
    p.add_argument("--pattern", required=True, help="Pattern: on:150,off:150,on:300")
    p.add_argument("--repeat", default="1", help="Repeat count or 'forever'")
    p.add_argument("--gap-ms", type=int, default=0)
    p.add_argument("--priority", default="INFO", choices=["INFO", "ALARM", "CRITICAL"])
    p.add_argument("--policy", default="QUEUE", choices=["QUEUE", "REPLACE", "DROP_IF_BUSY"])
    p.add_argument("--throttle-ms", type=int, default=0)
    p.add_argument("--token", default="")
    p.add_argument("--name", default="cli-play")
    p.add_argument("--wait", action="store_true", help="Wait until playback finishes")
    p.add_argument("--timeout-s", type=float, default=10.0)
    p.add_argument("--profile", default=None, help="JSON/YAML profile path")
    p.add_argument("--chip", default="/dev/gpiochip0")
    p.add_argument("--line", type=int, default=18)
    p.add_argument("--active-low", action="store_true")
    p.add_argument("--max-on-time-ms", type=int, default=0)
    p.add_argument("--simulate", action="store_true", help="Do not touch hardware")
    return p


def buzzer_play_main(argv: list[str] | None = None) -> int:
    args = _build_play_parser().parse_args(argv)
    try:
        pattern = _parse_pattern(args.pattern)
        opts = PlayOptions()
        if isinstance(args.repeat, str) and args.repeat.lower() == "forever":
            opts.repeat = 0xFFFFFFFF
        else:
            opts.repeat = int(args.repeat)
        opts.gap_ms = args.gap_ms
        opts.priority = _priority_from_str(args.priority)
        opts.policy = _policy_from_str(args.policy)
        opts.throttle_ms = args.throttle_ms
        opts.token = args.token

        cfg = _make_hw_cfg(args)
        with BuzzerEngine(cfg) as engine:
            _maybe_apply_profile(engine, args.profile)
            engine.play(pattern, opts, args.name)
            if args.wait:
                ok = _wait_until_idle(engine, args.timeout_s)
                if not ok:
                    raise TimeoutError("playback timeout reached")
        return 0
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


def _build_signal_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="buzzer-signal", description="Trigger buzzer event/alarm")
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--notify", help="Event ID")
    g.add_argument("--alarm", help="Alarm ID to activate")
    g.add_argument("--clear-alarm", help="Alarm ID to clear")
    p.add_argument("--duration-s", type=float, default=0.0, help="For --alarm: keep active for N seconds (0=until Ctrl+C)")
    p.add_argument("--wait", action="store_true", help="For --notify: wait until playback finishes")
    p.add_argument("--timeout-s", type=float, default=10.0)
    p.add_argument("--profile", default=None, help="JSON/YAML profile path")
    p.add_argument("--chip", default="/dev/gpiochip0")
    p.add_argument("--line", type=int, default=18)
    p.add_argument("--active-low", action="store_true")
    p.add_argument("--max-on-time-ms", type=int, default=0)
    p.add_argument("--simulate", action="store_true", help="Do not touch hardware")
    return p


def buzzer_signal_main(argv: list[str] | None = None) -> int:
    args = _build_signal_parser().parse_args(argv)
    try:
        cfg = _make_hw_cfg(args)
        with BuzzerEngine(cfg) as engine:
            _maybe_apply_profile(engine, args.profile)

            if args.notify:
                engine.notify(_event_from_str(args.notify))
                if args.wait:
                    ok = _wait_until_idle(engine, args.timeout_s)
                    if not ok:
                        raise TimeoutError("notify timeout reached")
                return 0

            if args.clear_alarm:
                engine.set_alarm(_alarm_from_str(args.clear_alarm), False)
                return 0

            alarm = _alarm_from_str(args.alarm)
            engine.set_alarm(alarm, True)

            if args.duration_s > 0:
                time.sleep(args.duration_s)
                engine.set_alarm(alarm, False)
                return 0

            running = True

            def _stop(_sig: int, _frame: object) -> None:
                nonlocal running
                running = False

            signal.signal(signal.SIGINT, _stop)
            signal.signal(signal.SIGTERM, _stop)
            while running:
                time.sleep(0.2)
            engine.set_alarm(alarm, False)
            return 0
    except KeyboardInterrupt:
        return 0
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


def _build_lint_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="buzzer-profile-lint", description="Validate buzzer JSON/YAML profile")
    p.add_argument("profile", help="Path to profile .json/.yaml")
    p.add_argument("--print-json", action="store_true", help="Print normalized profile JSON")
    return p


def buzzer_profile_lint_main(argv: list[str] | None = None) -> int:
    args = _build_lint_parser().parse_args(argv)
    try:
        data = _load_profile_data(args.profile)
        engine = BuzzerEngine(HardwareBuzzerConfig())
        engine.set_profile_from_dict(data)
        if args.print_json:
            print(engine.profile_as_json())
        else:
            print("Profile OK")
        return 0
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


__all__ = [
    "buzzer_play_main",
    "buzzer_signal_main",
    "buzzer_profile_lint_main",
]
