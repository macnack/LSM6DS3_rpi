#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
import types
from pathlib import Path


def install_fake_extension() -> None:
    fake = types.ModuleType("buzzer_rpi._buzzer")

    class _EnumValue:
        def __init__(self, name: str):
            self.name = name

        def __repr__(self) -> str:
            return self.name

    class EventId:
        ARMING = _EnumValue("ARMING")
        ARMING_FAILURE = _EnumValue("ARMING_FAILURE")
        DISARMED = _EnumValue("DISARMED")
        GYRO_INIT_DONE = _EnumValue("GYRO_INIT_DONE")
        READY_TO_ARM = _EnumValue("READY_TO_ARM")

    class AlarmId:
        BATTERY_FAILSAFE = _EnumValue("BATTERY_FAILSAFE")
        EKF_FAILURE = _EnumValue("EKF_FAILURE")
        LOST_VEHICLE = _EnumValue("LOST_VEHICLE")
        EVACUATION_TEMPORAL3 = _EnumValue("EVACUATION_TEMPORAL3")
        MISSING_SOS = _EnumValue("MISSING_SOS")

    class Priority:
        INFO = _EnumValue("INFO")
        ALARM = _EnumValue("ALARM")
        CRITICAL = _EnumValue("CRITICAL")

    class BusyPolicy:
        QUEUE = _EnumValue("QUEUE")
        REPLACE = _EnumValue("REPLACE")
        DROP_IF_BUSY = _EnumValue("DROP_IF_BUSY")

    class PhaseState:
        OFF = _EnumValue("OFF")
        ON = _EnumValue("ON")

    class Phase:
        def __init__(self, state=None, duration_ms=0):
            self.state = state
            self.duration_ms = duration_ms

    class PlayOptions:
        def __init__(self):
            self.repeat = 1
            self.gap_ms = 0
            self.priority = Priority.INFO
            self.policy = BusyPolicy.QUEUE
            self.throttle_ms = 0
            self.token = ""

    class HardwareBuzzerConfig:
        def __init__(self):
            self.chip_path = "/dev/gpiochip0"
            self.line = 18
            self.active_high = True
            self.max_on_time_ms = 0
            self.force_off_on_begin = True
            self.force_off_on_close = True
            self.simulate = False

    class GpioError(RuntimeError):
        pass

    class HardwareBuzzer:
        def __init__(self, *_args, **_kwargs):
            pass

    class BuzzerEngine:
        def __init__(self, *_args, **_kwargs):
            self._playing = False
            self._current = ""
            self._profile = {}

        def __enter__(self):
            return self

        def __exit__(self, exc_type, exc, tb):
            return False

        def set_profile_from_dict(self, profile: dict) -> None:
            self._profile = profile

        def profile_as_json(self) -> str:
            return json.dumps(self._profile)

        def play(self, _pattern, _opts, name):
            self._playing = True
            self._current = name
            self._playing = False
            self._current = ""

        def notify(self, _event_id):
            self._playing = True
            self._current = "event"
            self._playing = False
            self._current = ""

        def set_alarm(self, _alarm_id, active=True):
            self._playing = active
            self._current = "alarm" if active else ""

        def is_playing(self):
            return self._playing

        def current(self):
            return self._current

        def stop(self):
            self._playing = False
            self._current = ""

        def set_queue_limit(self, _limit):
            return None

    fake.EventId = EventId
    fake.AlarmId = AlarmId
    fake.Priority = Priority
    fake.BusyPolicy = BusyPolicy
    fake.PhaseState = PhaseState
    fake.Phase = Phase
    fake.PlayOptions = PlayOptions
    fake.HardwareBuzzerConfig = HardwareBuzzerConfig
    fake.HardwareBuzzer = HardwareBuzzer
    fake.BuzzerEngine = BuzzerEngine
    fake.GpioError = GpioError
    fake.alarm_name = lambda x: str(x)
    fake.event_name = lambda x: str(x)
    fake.make_default_profile_json = lambda: "{}"
    fake.load_profile_json_text = lambda text: text

    sys.modules["buzzer_rpi._buzzer"] = fake


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--package-dir", required=True)
    parser.add_argument("--profile", required=True)
    args = parser.parse_args()

    pkg_dir = Path(args.package_dir)
    if str(pkg_dir) not in sys.path:
        sys.path.insert(0, str(pkg_dir))

    install_fake_extension()

    from buzzer_rpi import cli  # pylint: disable=import-error

    assert cli.buzzer_play_main([
        "--pattern", "on:120,off:120,on:120",
        "--simulate",
    ]) == 0

    assert cli.buzzer_signal_main([
        "--notify", "READY_TO_ARM",
        "--profile", args.profile,
        "--simulate",
    ]) == 0

    assert cli.buzzer_profile_lint_main([
        args.profile,
        "--print-json",
    ]) == 0

    assert cli.buzzer_play_main([
        "--pattern", "invalid",
        "--simulate",
    ]) != 0

    print("test_cli: ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
