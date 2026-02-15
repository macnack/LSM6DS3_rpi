from __future__ import annotations

import importlib
import os
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

# IPC review note:
# - Bump MESSAGE_VERSION for wire-format breaking changes (field layout/order/type/size, header semantics, CRC scope).
# - Change MESSAGE_MAGIC only for a new protocol family/channel that must be hard-isolated from existing decoders.
MESSAGE_MAGIC = 0x52544331
MESSAGE_VERSION = 1

SENSOR_MSG_STRUCT = struct.Struct("<IHHQQBB6xddddddddI")
ESTIMATOR_MSG_STRUCT = struct.Struct("<IHHQQB7x4f3f3fI")
CONTROLLER_MSG_STRUCT = struct.Struct("<IHHQQB7x4fI")

SENSOR_PAYLOAD_BYTES = 88
ESTIMATOR_PAYLOAD_BYTES = 64
CONTROLLER_PAYLOAD_BYTES = 40
CODEC_API_VERSION = 1

_CODEC_BACKEND_ENV = "RUNTIME_IPC_CODEC_BACKEND"
_CODEC_BACKEND_AUTO = "auto"
_CODEC_BACKEND_PYTHON = "python"
_CODEC_BACKEND_CPP = "cpp"
_VALID_CODEC_BACKENDS = {_CODEC_BACKEND_AUTO, _CODEC_BACKEND_PYTHON, _CODEC_BACKEND_CPP}


@dataclass
class MailboxConfig:
    name: str
    retries: int = 200
    retry_sleep_s: float = 0.05


@dataclass
class SensorSnapshot:
    seq: int
    t_ns: int
    imu_valid: bool
    baro_valid: bool
    ax_mps2: float
    ay_mps2: float
    az_mps2: float
    gx_rads: float
    gy_rads: float
    gz_rads: float
    pressure_pa: float
    temperature_c: float


def _requested_codec_backend() -> str:
    raw = os.environ.get(_CODEC_BACKEND_ENV, _CODEC_BACKEND_AUTO).strip().lower()
    if raw not in _VALID_CODEC_BACKENDS:
        allowed = ", ".join(sorted(_VALID_CODEC_BACKENDS))
        raise RuntimeError(f"{_CODEC_BACKEND_ENV} must be one of: {allowed}. Got '{raw}'")
    return raw


def _validate_cpp_codec(codec: object) -> None:
    expected_values = {
        "CODEC_API_VERSION": CODEC_API_VERSION,
        "MESSAGE_MAGIC": MESSAGE_MAGIC,
        "MESSAGE_VERSION": MESSAGE_VERSION,
        "SENSOR_MSG_SIZE": SENSOR_MSG_STRUCT.size,
        "ESTIMATOR_MSG_SIZE": ESTIMATOR_MSG_STRUCT.size,
        "CONTROLLER_MSG_SIZE": CONTROLLER_MSG_STRUCT.size,
        "SENSOR_PAYLOAD_BYTES": SENSOR_PAYLOAD_BYTES,
        "ESTIMATOR_PAYLOAD_BYTES": ESTIMATOR_PAYLOAD_BYTES,
        "CONTROLLER_PAYLOAD_BYTES": CONTROLLER_PAYLOAD_BYTES,
    }
    mismatches: list[str] = []
    for attr, expected in expected_values.items():
        if not hasattr(codec, attr):
            mismatches.append(f"{attr}=<missing> expected={expected}")
            continue
        got = int(getattr(codec, attr))
        if got != expected:
            mismatches.append(f"{attr}={got} expected={expected}")
    if mismatches:
        joined = "; ".join(mismatches)
        raise RuntimeError(f"Incompatible _runtime_ipc_codec module: {joined}")


def _load_cpp_codec() -> object:
    codec = importlib.import_module("_runtime_ipc_codec")
    _validate_cpp_codec(codec)
    return codec


_CPP_CODEC: object | None = None
_CODEC_BACKEND = _CODEC_BACKEND_PYTHON
_CODEC_IMPORT_ERROR: Exception | None = None

_codec_request = _requested_codec_backend()
if _codec_request in (_CODEC_BACKEND_AUTO, _CODEC_BACKEND_CPP):
    try:
        _CPP_CODEC = _load_cpp_codec()
        _CODEC_BACKEND = _CODEC_BACKEND_CPP
    except Exception as exc:
        _CODEC_IMPORT_ERROR = exc
        if _codec_request == _CODEC_BACKEND_CPP:
            raise RuntimeError("RUNTIME_IPC_CODEC_BACKEND=cpp requested, but _runtime_ipc_codec is unavailable") from exc


def ipc_codec_backend() -> str:
    return _CODEC_BACKEND


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


def _decode_sensor_snapshot_python(payload: bytes, *, check_crc: bool = True) -> SensorSnapshot | None:
    if len(payload) != SENSOR_MSG_STRUCT.size:
        return None
    values = SENSOR_MSG_STRUCT.unpack(payload)
    (
        magic,
        version,
        payload_bytes,
        seq,
        t_ns,
        imu_valid,
        baro_valid,
        ax,
        ay,
        az,
        gx,
        gy,
        gz,
        pressure_pa,
        temp_c,
        crc,
    ) = values
    if magic != MESSAGE_MAGIC or version != MESSAGE_VERSION or payload_bytes != SENSOR_PAYLOAD_BYTES:
        return None
    if check_crc and crc != finalize_crc(payload[:-4]):
        return None
    return SensorSnapshot(
        seq=seq,
        t_ns=t_ns,
        imu_valid=bool(imu_valid),
        baro_valid=bool(baro_valid),
        ax_mps2=ax,
        ay_mps2=ay,
        az_mps2=az,
        gx_rads=gx,
        gy_rads=gy,
        gz_rads=gz,
        pressure_pa=pressure_pa,
        temperature_c=temp_c,
    )


def _sensor_snapshot_from_cpp_dict(sample: dict[str, object]) -> SensorSnapshot | None:
    try:
        return SensorSnapshot(
            seq=int(sample["seq"]),
            t_ns=int(sample["t_ns"]),
            imu_valid=bool(sample["imu_valid"]),
            baro_valid=bool(sample["baro_valid"]),
            ax_mps2=float(sample["ax_mps2"]),
            ay_mps2=float(sample["ay_mps2"]),
            az_mps2=float(sample["az_mps2"]),
            gx_rads=float(sample["gx_rads"]),
            gy_rads=float(sample["gy_rads"]),
            gz_rads=float(sample["gz_rads"]),
            pressure_pa=float(sample["pressure_pa"]),
            temperature_c=float(sample["temperature_c"]),
        )
    except (KeyError, TypeError, ValueError):
        return None


def decode_sensor_snapshot(payload: bytes, *, check_crc: bool = True) -> SensorSnapshot | None:
    if _CPP_CODEC is not None and check_crc:
        decoded = _CPP_CODEC.decode_sensor_snapshot(payload)
        if decoded is None:
            return None
        return _sensor_snapshot_from_cpp_dict(decoded)
    return _decode_sensor_snapshot_python(payload, check_crc=check_crc)


def _encode_controller_command_python(seq: int, t_ns: int, armed: bool, servo_norm: tuple[float, float, float, float]) -> bytes:
    msg_without_crc = CONTROLLER_MSG_STRUCT.pack(
        MESSAGE_MAGIC,
        MESSAGE_VERSION,
        CONTROLLER_PAYLOAD_BYTES,
        seq,
        t_ns,
        1 if armed else 0,
        servo_norm[0],
        servo_norm[1],
        servo_norm[2],
        servo_norm[3],
        0,
    )
    crc = finalize_crc(msg_without_crc[:-4])
    return msg_without_crc[:-4] + struct.pack("<I", crc)


def encode_controller_command(seq: int, t_ns: int, armed: bool, servo_norm: tuple[float, float, float, float]) -> bytes:
    if _CPP_CODEC is not None:
        return _CPP_CODEC.encode_controller_command(seq, t_ns, armed, servo_norm[0], servo_norm[1], servo_norm[2], servo_norm[3])
    return _encode_controller_command_python(seq, t_ns, armed, servo_norm)


def _encode_estimator_state_python(
    seq: int,
    t_ns: int,
    valid: bool,
    q_body_to_ned: tuple[float, float, float, float],
    vel_ned_mps: tuple[float, float, float] = (0.0, 0.0, 0.0),
    pos_ned_m: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> bytes:
    msg_without_crc = ESTIMATOR_MSG_STRUCT.pack(
        MESSAGE_MAGIC,
        MESSAGE_VERSION,
        ESTIMATOR_PAYLOAD_BYTES,
        seq,
        t_ns,
        1 if valid else 0,
        q_body_to_ned[0],
        q_body_to_ned[1],
        q_body_to_ned[2],
        q_body_to_ned[3],
        vel_ned_mps[0],
        vel_ned_mps[1],
        vel_ned_mps[2],
        pos_ned_m[0],
        pos_ned_m[1],
        pos_ned_m[2],
        0,
    )
    crc = finalize_crc(msg_without_crc[:-4])
    return msg_without_crc[:-4] + struct.pack("<I", crc)


def encode_estimator_state(
    seq: int,
    t_ns: int,
    valid: bool,
    q_body_to_ned: tuple[float, float, float, float],
    vel_ned_mps: tuple[float, float, float] = (0.0, 0.0, 0.0),
    pos_ned_m: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> bytes:
    if _CPP_CODEC is not None:
        return _CPP_CODEC.encode_estimator_state(
            seq,
            t_ns,
            valid,
            q_body_to_ned[0],
            q_body_to_ned[1],
            q_body_to_ned[2],
            q_body_to_ned[3],
            vel_ned_mps[0],
            vel_ned_mps[1],
            vel_ned_mps[2],
            pos_ned_m[0],
            pos_ned_m[1],
            pos_ned_m[2],
        )
    return _encode_estimator_state_python(seq, t_ns, valid, q_body_to_ned, vel_ned_mps, pos_ned_m)
