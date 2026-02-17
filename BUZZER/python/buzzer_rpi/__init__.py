"""Python bindings for LPB2418A-style buzzer control on Raspberry Pi."""

from ._buzzer import (  # type: ignore[attr-defined]
    AlarmId,
    BusyPolicy,
    BuzzerEngine,
    EventId,
    GpioError,
    HardwareBuzzer,
    HardwareBuzzerConfig,
    Phase,
    PhaseState,
    PlayOptions,
    Priority,
    alarm_name,
    event_name,
    load_profile_json_text,
    make_default_profile_json,
)

__all__ = [
    "GpioError",
    "HardwareBuzzerConfig",
    "HardwareBuzzer",
    "PhaseState",
    "Phase",
    "Priority",
    "BusyPolicy",
    "PlayOptions",
    "EventId",
    "AlarmId",
    "BuzzerEngine",
    "make_default_profile_json",
    "load_profile_json_text",
    "event_name",
    "alarm_name",
]
