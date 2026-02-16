"""Python bindings for ADS1115 on Linux I2C.

`Ads1115` exposes the direct pybind API.
`ADS1115` provides a DFRobot-style compatibility wrapper.
"""

from __future__ import annotations

from ._ads1115 import (  # type: ignore[attr-defined]
    ADS1115_REG_CONFIG_DR_128SPS,
    ADS1115_REG_CONFIG_DR_16SPS,
    ADS1115_REG_CONFIG_DR_250SPS,
    ADS1115_REG_CONFIG_DR_32SPS,
    ADS1115_REG_CONFIG_DR_475SPS,
    ADS1115_REG_CONFIG_DR_64SPS,
    ADS1115_REG_CONFIG_DR_860SPS,
    ADS1115_REG_CONFIG_DR_8SPS,
    ADS1115_REG_CONFIG_PGA_0_256V,
    ADS1115_REG_CONFIG_PGA_0_512V,
    ADS1115_REG_CONFIG_PGA_1_024V,
    ADS1115_REG_CONFIG_PGA_2_048V,
    ADS1115_REG_CONFIG_PGA_4_096V,
    ADS1115_REG_CONFIG_PGA_6_144V,
    Ads1115,
    ComparatorConfig,
    DataRate,
    I2cError,
    Mode,
    Mux,
)

_GAIN_TO_FULL_SCALE_MV = {
    ADS1115_REG_CONFIG_PGA_6_144V: 6144.0,
    ADS1115_REG_CONFIG_PGA_4_096V: 4096.0,
    ADS1115_REG_CONFIG_PGA_2_048V: 2048.0,
    ADS1115_REG_CONFIG_PGA_1_024V: 1024.0,
    ADS1115_REG_CONFIG_PGA_0_512V: 512.0,
    ADS1115_REG_CONFIG_PGA_0_256V: 256.0,
}

_DATA_RATE_TO_ENUM = {
    ADS1115_REG_CONFIG_DR_8SPS: DataRate.SPS_8,
    ADS1115_REG_CONFIG_DR_16SPS: DataRate.SPS_16,
    ADS1115_REG_CONFIG_DR_32SPS: DataRate.SPS_32,
    ADS1115_REG_CONFIG_DR_64SPS: DataRate.SPS_64,
    ADS1115_REG_CONFIG_DR_128SPS: DataRate.SPS_128,
    ADS1115_REG_CONFIG_DR_250SPS: DataRate.SPS_250,
    ADS1115_REG_CONFIG_DR_475SPS: DataRate.SPS_475,
    ADS1115_REG_CONFIG_DR_860SPS: DataRate.SPS_860,
}


class ADS1115:
    """Compatibility layer with the legacy DFRobot Python API shape."""

    def __init__(self, bus_path: str = "/dev/i2c-1", address: int = 0x48, retries: int = 2):
        self._bus_path = bus_path
        self._address = address
        self._retries = retries
        self._started = False

        self.channel = 0
        self._mode = "single"
        self._gain_code = ADS1115_REG_CONFIG_PGA_2_048V
        self._data_rate_code = ADS1115_REG_CONFIG_DR_128SPS
        self._conversion_mode = Mode.SINGLE_SHOT
        self._comparator_config = ComparatorConfig()

        self._dev = Ads1115(bus_path=bus_path, address=address, retries=retries)

    def begin(self) -> None:
        if not self._started:
            self._dev.begin()
            self._dev.set_mode(self._conversion_mode)
            self._dev.set_data_rate(_DATA_RATE_TO_ENUM[self._data_rate_code])
            self._dev.set_gain(self._gain_code)
            self._dev.configure_comparator(self._comparator_config)
            self._started = True

    def close(self) -> None:
        self._dev.close()
        self._started = False

    def set_addr_ADS1115(self, addr: int) -> None:
        if not (0 <= addr <= 0x7F):
            raise ValueError("I2C address must be 7-bit (0x00..0x7F)")

        was_started = self._started
        if was_started:
            self.close()

        self._address = addr
        self._dev = Ads1115(bus_path=self._bus_path, address=self._address, retries=self._retries)

        if was_started:
            self.begin()

    def set_gain(self, gain: int) -> None:
        if gain not in _GAIN_TO_FULL_SCALE_MV:
            raise ValueError("invalid gain code")
        self._gain_code = gain
        self._dev.set_gain(gain)

    def set_mode(self, mode: str) -> None:
        normalized = str(mode).strip().lower()
        if normalized in ("single", "single_shot", "single-shot"):
            self._conversion_mode = Mode.SINGLE_SHOT
        elif normalized in ("continuous", "continous"):
            self._conversion_mode = Mode.CONTINUOUS
        else:
            raise ValueError("mode must be one of: single_shot, continuous")
        self._dev.set_mode(self._conversion_mode)

    def set_data_rate(self, dr_code: int) -> None:
        dr = int(dr_code)
        if dr not in _DATA_RATE_TO_ENUM:
            raise ValueError("invalid data-rate code")
        self._data_rate_code = dr
        self._dev.set_data_rate(_DATA_RATE_TO_ENUM[dr])

    def set_mux(self, mux: Mux) -> None:
        self._dev.set_mux(mux)

    def set_channel(self, channel: int) -> int:
        if not (0 <= channel <= 3):
            raise ValueError("channel must be in range 0..3")
        self.channel = int(channel)
        return self.channel

    def set_single(self) -> None:
        self._mode = "single"

    def set_differential(self) -> None:
        self._mode = "differential"

    def configure_comparator(
        self,
        low_threshold: int = -32768,
        high_threshold: int = 32767,
        *,
        window_mode: bool = False,
        active_high: bool = False,
        latching: bool = False,
        queue: int = 0x03,
    ) -> None:
        if not (-32768 <= int(low_threshold) <= 32767):
            raise ValueError("low_threshold must be in int16 range [-32768, 32767]")
        if not (-32768 <= int(high_threshold) <= 32767):
            raise ValueError("high_threshold must be in int16 range [-32768, 32767]")
        if not (0 <= int(queue) <= 3):
            raise ValueError("queue must be in range 0..3")

        cfg = ComparatorConfig()
        cfg.low_threshold = int(low_threshold)
        cfg.high_threshold = int(high_threshold)
        cfg.window_mode = bool(window_mode)
        cfg.active_high = bool(active_high)
        cfg.latching = bool(latching)
        cfg.queue = int(queue)

        self._comparator_config = cfg
        self._dev.configure_comparator(cfg)

    def _volts_to_millivolts_int(self, volts: float) -> int:
        return int(round(volts * 1000.0))

    def _ensure_started(self) -> None:
        if not self._started:
            self.begin()

    def read_value(self):
        self._ensure_started()
        if self._mode == "differential":
            volts = self._dev.comparator_voltage(self.channel)
        else:
            volts = self._dev.read_voltage(self.channel)
        return {"r": self._volts_to_millivolts_int(volts)}

    def read_adc(self, channel: int) -> int:
        self._ensure_started()
        return int(self._dev.read_adc(int(channel)))

    def read_latest(self):
        self._ensure_started()
        return {"r": self._volts_to_millivolts_int(self._dev.read_latest_voltage())}

    def stream_continuous(self, callback, *, poll_interval_ms: int = 1, max_samples: int = 0) -> int:
        self._ensure_started()
        self._dev.set_mode(Mode.CONTINUOUS)
        count = 0

        def _cb(raw: int) -> bool:
            nonlocal count
            count += 1
            return bool(callback(raw))

        self._dev.poll_continuous(_cb, int(poll_interval_ms), int(max_samples))
        return count

    def read_voltage(self, channel: int):
        self.set_channel(channel)
        self.set_single()
        return self.read_value()

    def comparator_voltage(self, channel: int):
        self.set_channel(channel)
        self.set_differential()
        return self.read_value()


__all__ = [
    "Ads1115",
    "ADS1115",
    "ComparatorConfig",
    "Mode",
    "DataRate",
    "Mux",
    "I2cError",
    "ADS1115_REG_CONFIG_PGA_6_144V",
    "ADS1115_REG_CONFIG_PGA_4_096V",
    "ADS1115_REG_CONFIG_PGA_2_048V",
    "ADS1115_REG_CONFIG_PGA_1_024V",
    "ADS1115_REG_CONFIG_PGA_0_512V",
    "ADS1115_REG_CONFIG_PGA_0_256V",
    "ADS1115_REG_CONFIG_DR_8SPS",
    "ADS1115_REG_CONFIG_DR_16SPS",
    "ADS1115_REG_CONFIG_DR_32SPS",
    "ADS1115_REG_CONFIG_DR_64SPS",
    "ADS1115_REG_CONFIG_DR_128SPS",
    "ADS1115_REG_CONFIG_DR_250SPS",
    "ADS1115_REG_CONFIG_DR_475SPS",
    "ADS1115_REG_CONFIG_DR_860SPS",
]
