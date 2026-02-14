from __future__ import annotations

import struct
import time
import zlib
from dataclasses import dataclass
import mmap
from pathlib import Path
from multiprocessing import shared_memory
try:
    from multiprocessing import resource_tracker
except ImportError:  # pragma: no cover
    resource_tracker = None

MESSAGE_MAGIC = 0x52544331
MESSAGE_VERSION = 1

SENSOR_MSG_STRUCT = struct.Struct("<IHHQQBB6xddddddddI")
ESTIMATOR_MSG_STRUCT = struct.Struct("<IHHQQB7x4f3f3fI")
CONTROLLER_MSG_STRUCT = struct.Struct("<IHHQQB7x4fI")

SENSOR_PAYLOAD_BYTES = 88
ESTIMATOR_PAYLOAD_BYTES = 64
CONTROLLER_PAYLOAD_BYTES = 40


@dataclass
class MailboxConfig:
    name: str
    retries: int = 200
    retry_sleep_s: float = 0.05


class ShmMailbox:
    def __init__(self, cfg: MailboxConfig, payload_size: int):
        self._cfg = cfg
        self._payload_size = payload_size
        self._size = 8 + payload_size
        self._shm: shared_memory.SharedMemory | None = None
        self._mmap: mmap.mmap | None = None
        self._file = None

    @staticmethod
    def _normalize_name(name: str) -> str:
        return name[1:] if name.startswith("/") else name

    @staticmethod
    def _fallback_file_path(name: str) -> Path:
        normalized = ShmMailbox._normalize_name(name).replace("/", "_")
        if not normalized:
            normalized = "runtime_mailbox"
        return Path("/tmp") / f"{normalized}.mailbox"

    def _open_file_fallback(self) -> bool:
        path = self._fallback_file_path(self._cfg.name)
        if not path.exists():
            return False
        fh = path.open("r+b")
        self._file = fh
        self._mmap = mmap.mmap(fh.fileno(), self._size, access=mmap.ACCESS_WRITE)
        return True

    @staticmethod
    def _open_existing_shm_no_track(name: str) -> shared_memory.SharedMemory:
        # Python 3.13+ supports track=False; for older versions, unregister manually.
        try:
            return shared_memory.SharedMemory(name=name, create=False, track=False)
        except TypeError:
            shm = shared_memory.SharedMemory(name=name, create=False)
            if resource_tracker is not None:
                try:
                    shm_name = getattr(shm, "_name", f"/{name}")
                    resource_tracker.unregister(shm_name, "shared_memory")
                except Exception:
                    pass
            return shm

    def open_existing(self) -> None:
        last_err: Exception | None = None
        name = self._normalize_name(self._cfg.name)
        for _ in range(max(1, self._cfg.retries)):
            try:
                self._shm = self._open_existing_shm_no_track(name)
                return
            except (FileNotFoundError, PermissionError, OSError) as exc:
                last_err = exc
                try:
                    if self._open_file_fallback():
                        return
                except OSError as fb_exc:
                    last_err = fb_exc
                time.sleep(self._cfg.retry_sleep_s)
        raise RuntimeError(
            f"Failed to open shared memory '{self._cfg.name}'. Start rt_core first and keep it running."
        ) from last_err

    def close(self) -> None:
        if self._mmap is not None:
            self._mmap.close()
            self._mmap = None
        if self._file is not None:
            self._file.close()
            self._file = None
        if self._shm is not None:
            self._shm.close()
            self._shm = None

    def _buffer(self):
        if self._shm is not None:
            return self._shm.buf
        if self._mmap is not None:
            return self._mmap
        raise RuntimeError("Mailbox not open")

    def write(self, payload: bytes) -> None:
        if len(payload) != self._payload_size:
            raise ValueError(f"Payload size mismatch: expected {self._payload_size}, got {len(payload)}")

        buf = self._buffer()
        seq = struct.unpack_from("<Q", buf, 0)[0]
        if seq & 1:
            seq += 1

        struct.pack_into("<Q", buf, 0, seq + 1)
        buf[8 : 8 + self._payload_size] = payload
        struct.pack_into("<Q", buf, 0, seq + 2)

    def try_read(self) -> bytes | None:
        buf = self._buffer()
        seq_a = struct.unpack_from("<Q", buf, 0)[0]
        if seq_a & 1:
            return None

        payload = bytes(buf[8 : 8 + self._payload_size])
        seq_b = struct.unpack_from("<Q", buf, 0)[0]
        if seq_a != seq_b or (seq_b & 1):
            return None
        return payload


def monotonic_ns() -> int:
    return time.clock_gettime_ns(time.CLOCK_MONOTONIC)


def parse_simple_toml(path: str) -> dict[str, dict[str, str]]:
    result: dict[str, dict[str, str]] = {}
    section = ""
    with open(path, "r", encoding="utf-8") as fh:
        for raw_line in fh:
            line = raw_line.split("#", 1)[0].strip()
            if not line:
                continue
            if line.startswith("[") and line.endswith("]"):
                section = line[1:-1].strip()
                result.setdefault(section, {})
                continue
            if "=" not in line:
                continue
            key, value = line.split("=", 1)
            key = key.strip()
            value = value.strip()
            result.setdefault(section, {})[key] = value
    return result


def parse_bool(text: str) -> bool:
    return text.lower() == "true"


def parse_int(text: str) -> int:
    return int(text, 0)


def parse_float(text: str) -> float:
    return float(text)


def parse_string(text: str) -> str:
    text = text.strip()
    if text.startswith('"') and text.endswith('"'):
        return text[1:-1]
    return text


def finalize_crc(payload_without_crc: bytes) -> int:
    return zlib.crc32(payload_without_crc) & 0xFFFFFFFF
