#!/usr/bin/env python3
"""Compatibility wrapper for servo_rpi CLI."""

from servo_rpi.cli import pwmset_main


if __name__ == "__main__":
    raise SystemExit(pwmset_main())
