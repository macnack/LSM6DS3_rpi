from __future__ import annotations

import time
from dataclasses import dataclass

from runtime.python.ipc_common import (
    IGNITER_COMMAND_MSG_STRUCT,
    IGNITER_STATUS_MSG_STRUCT,
    MailboxConfig,
    ShmMailbox,
    decode_igniter_status,
    encode_igniter_command,
    monotonic_ns,
    parse_int,
    parse_simple_toml,
    parse_string,
)


class IgniterAction:
    NONE = 0
    ARM = 1
    DISARM = 2
    FIRE_MASK = 3
    CLEAR_FAULT = 4


@dataclass
class IgniterClientConfig:
    command_shm: str
    status_shm: str
    retries: int = 200
    retry_sleep_s: float = 0.05


class IgniterClient:
    def __init__(self, cfg: IgniterClientConfig):
        self._cfg = cfg
        self._cmd = ShmMailbox(
            MailboxConfig(cfg.command_shm, retries=cfg.retries, retry_sleep_s=cfg.retry_sleep_s),
            IGNITER_COMMAND_MSG_STRUCT.size,
        )
        self._status = ShmMailbox(
            MailboxConfig(cfg.status_shm, retries=cfg.retries, retry_sleep_s=cfg.retry_sleep_s),
            IGNITER_STATUS_MSG_STRUCT.size,
        )
        self._seq = 0

    @staticmethod
    def from_toml(path: str) -> "IgniterClient":
        cfg = parse_simple_toml(path)
        igniter = cfg.get("igniter", {})
        ipc = cfg.get("ipc", {})
        retries = parse_int(ipc.get("open_retry_count", "200"))
        retry_ms = parse_int(ipc.get("open_retry_ms", "50"))
        return IgniterClient(
            IgniterClientConfig(
                command_shm=parse_string(igniter.get("command_shm", '"/rt_igniter_command_v1"')),
                status_shm=parse_string(igniter.get("status_shm", '"/rt_igniter_status_v1"')),
                retries=retries,
                retry_sleep_s=retry_ms / 1000.0,
            )
        )

    def open(self) -> None:
        self._cmd.open_existing()
        self._status.open_existing()

    def close(self) -> None:
        self._status.close()
        self._cmd.close()

    def _send(self, action: int, fire_mask: int = 0, duration_ms: tuple[int, int, int, int] = (0, 0, 0, 0)) -> None:
        self._seq += 1
        payload = encode_igniter_command(
            seq=self._seq,
            t_ns=monotonic_ns(),
            action=action,
            fire_mask=fire_mask,
            duration_ms=duration_ms,
        )
        self._cmd.write(payload)

    def arm(self) -> None:
        self._send(IgniterAction.ARM)

    def disarm(self) -> None:
        self._send(IgniterAction.DISARM)

    def clear_fault(self) -> None:
        self._send(IgniterAction.CLEAR_FAULT)

    def fire_mask(self, mask: int, durations_ms: tuple[int, int, int, int]) -> None:
        self._send(IgniterAction.FIRE_MASK, fire_mask=mask, duration_ms=durations_ms)

    def fire_one(self, channel: int, duration_ms: int) -> None:
        if channel < 0 or channel >= 4:
            raise ValueError("channel must be in [0, 3]")
        durations = [0, 0, 0, 0]
        durations[channel] = int(duration_ms)
        self.fire_mask(1 << channel, (durations[0], durations[1], durations[2], durations[3]))

    def fire_all(self, duration_ms: int) -> None:
        d = int(duration_ms)
        self.fire_mask(0x0F, (d, d, d, d))

    def read_status(self):
        payload = self._status.try_read()
        if payload is None:
            return None
        return decode_igniter_status(payload)

    def read_status_blocking(self, timeout_s: float = 1.0, poll_s: float = 0.01):
        deadline = time.monotonic() + max(0.0, timeout_s)
        while time.monotonic() <= deadline:
            status = self.read_status()
            if status is not None:
                return status
            time.sleep(max(0.001, poll_s))
        return None
