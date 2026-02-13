"""Python bindings for LSM6DS3 on Linux I2C/SPI."""

from ._lsm6ds3 import (  # type: ignore[attr-defined]
    AccelOdr,
    AccelScale,
    GyroOdr,
    GyroScale,
    I2cError,
    Lsm6ds3,
    Lsm6ds3Spi,
    SpiError,
)

__all__ = [
    "Lsm6ds3",
    "Lsm6ds3Spi",
    "I2cError",
    "SpiError",
    "AccelOdr",
    "GyroOdr",
    "AccelScale",
    "GyroScale",
]
