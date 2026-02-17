#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
import types
from pathlib import Path


def install_fake_extension() -> None:
    fake = types.ModuleType("buzzer_rpi._buzzer")

    class _EnumBase:
        def __init__(self, name: str):
            self.name = name

        def __repr__(self) -> str:
            return self.name

    class AlarmId:
        BATTERY_FAILSAFE = _EnumBase("BATTERY_FAILSAFE")
        EKF_FAILURE = _EnumBase("EKF_FAILURE")
        LOST_VEHICLE = _EnumBase("LOST_VEHICLE")
        EVACUATION_TEMPORAL3 = _EnumBase("EVACUATION_TEMPORAL3")
        MISSING_SOS = _EnumBase("MISSING_SOS")

    class EventId:
        ARMING = _EnumBase("ARMING")
        ARMING_FAILURE = _EnumBase("ARMING_FAILURE")
        DISARMED = _EnumBase("DISARMED")
        GYRO_INIT_DONE = _EnumBase("GYRO_INIT_DONE")
        READY_TO_ARM = _EnumBase("READY_TO_ARM")

    class BusyPolicy:
        QUEUE = _EnumBase("QUEUE")
        REPLACE = _EnumBase("REPLACE")
        DROP_IF_BUSY = _EnumBase("DROP_IF_BUSY")

    class Priority:
        INFO = _EnumBase("INFO")
        ALARM = _EnumBase("ALARM")
        CRITICAL = _EnumBase("CRITICAL")

    class PhaseState:
        OFF = _EnumBase("OFF")
        ON = _EnumBase("ON")

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
            self.simulate = True

    class GpioError(RuntimeError):
        pass

    class HardwareBuzzer:
        def __init__(self, *_args, **_kwargs):
            pass

    class BuzzerEngine:
        def __init__(self, *_args, **_kwargs):
            self._profile = {}

        def set_profile_from_dict(self, profile: dict) -> None:
            self._profile = profile

        def profile_as_json(self) -> str:
            return json.dumps(self._profile)

    fake.AlarmId = AlarmId
    fake.EventId = EventId
    fake.BusyPolicy = BusyPolicy
    fake.Priority = Priority
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

    data = cli._load_profile_data(args.profile)
    assert isinstance(data, dict)
    assert "events" in data
    assert "alarms" in data
    assert "ARMING" in data["events"]
    assert "BATTERY_FAILSAFE" in data["alarms"]

    rc = cli.buzzer_profile_lint_main([args.profile, "--print-json"])
    assert rc == 0

    print("test_buzzer_profile: ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
