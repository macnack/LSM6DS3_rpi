"""Python bindings for LSM6DS3 on Linux I2C."""

from ._lsm6ds3 import (  # type: ignore[attr-defined]
    AccelOdr,
    AccelScale,
    GyroOdr,
    GyroScale,
    I2cError,
    Lsm6ds3,
)

__all__ = [
    "Lsm6ds3",
    "I2cError",
    "AccelOdr",
    "GyroOdr",
    "AccelScale",
    "GyroScale",
]
