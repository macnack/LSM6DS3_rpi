#!/usr/bin/env python3
"""Compatibility wrapper for servo_rpi CLI."""

from servo_rpi.cli import pwm_multi_cycle_main


if __name__ == "__main__":
    raise SystemExit(pwm_multi_cycle_main())
