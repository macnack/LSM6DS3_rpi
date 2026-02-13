"""Python bindings for BMP390 on Linux I2C."""

from ._bmp390 import (  # type: ignore[attr-defined]
    BMP3_IIR_FILTER_COEFF_3,
    BMP3_NO_OVERSAMPLING,
    BMP3_ODR_25_HZ,
    BMP3_OVERSAMPLING_16X,
    BMP3_OVERSAMPLING_2X,
    BMP3_OVERSAMPLING_32X,
    BMP3_OVERSAMPLING_4X,
    BMP3_OVERSAMPLING_8X,
    Bmp390,
    I2cError,
    Reading,
)

__all__ = [
    "Bmp390",
    "Reading",
    "I2cError",
    "BMP3_NO_OVERSAMPLING",
    "BMP3_OVERSAMPLING_2X",
    "BMP3_OVERSAMPLING_4X",
    "BMP3_OVERSAMPLING_8X",
    "BMP3_OVERSAMPLING_16X",
    "BMP3_OVERSAMPLING_32X",
    "BMP3_ODR_25_HZ",
    "BMP3_IIR_FILTER_COEFF_3",
]
