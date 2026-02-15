#!/usr/bin/env python3
from __future__ import annotations

import argparse
import importlib
import math
import os
import struct
import sys
import unittest
from pathlib import Path

_BOOTSTRAP = argparse.ArgumentParser(add_help=False)
_BOOTSTRAP.add_argument("--backend", choices=("auto", "python", "cpp"), default="auto")
_BOOTSTRAP.add_argument("--expect-backend", choices=("python", "cpp"), default=None)
_BOOTSTRAP.add_argument("--module-dir", default=None)
_BOOTSTRAP_ARGS, _UNKNOWN = _BOOTSTRAP.parse_known_args()
sys.argv = [sys.argv[0], *_UNKNOWN]

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
if _BOOTSTRAP_ARGS.module_dir:
    module_dir = str(Path(_BOOTSTRAP_ARGS.module_dir).resolve())
    if module_dir not in sys.path:
        sys.path.insert(0, module_dir)

os.environ["RUNTIME_IPC_CODEC_BACKEND"] = _BOOTSTRAP_ARGS.backend
sys.modules.pop("runtime.python.ipc_common", None)
IPC = importlib.import_module("runtime.python.ipc_common")


class IpcCodecTests(unittest.TestCase):
    def _pack_sensor(self, *, magic: int = IPC.MESSAGE_MAGIC, version: int = IPC.MESSAGE_VERSION,
                     crc_corrupt: bool = False) -> bytes:
        seq = 7
        t_ns = 123456789
        imu_valid = 1
        baro_valid = 1
        payload = bytearray(
            IPC.SENSOR_MSG_STRUCT.pack(
                magic,
                version,
                IPC.SENSOR_PAYLOAD_BYTES,
                seq,
                t_ns,
                imu_valid,
                baro_valid,
                1.1,
                -2.2,
                9.81,
                0.1,
                0.2,
                -0.3,
                100123.0,
                24.5,
                0,
            )
        )
        crc = IPC.finalize_crc(payload[:-4])
        if crc_corrupt:
            crc ^= 0xFFFFFFFF
        struct.pack_into("<I", payload, len(payload) - 4, crc)
        return bytes(payload)

    def test_backend_selection(self) -> None:
        if _BOOTSTRAP_ARGS.expect_backend is None:
            self.skipTest("--expect-backend not provided for this invocation")
        self.assertEqual(IPC.ipc_codec_backend(), _BOOTSTRAP_ARGS.expect_backend)

    def test_decode_sensor_snapshot_accepts_valid_payload(self) -> None:
        payload = self._pack_sensor()
        sample = IPC.decode_sensor_snapshot(payload)
        self.assertIsNotNone(sample)
        assert sample is not None
        self.assertEqual(sample.seq, 7)
        self.assertEqual(sample.t_ns, 123456789)
        self.assertTrue(sample.imu_valid)
        self.assertTrue(sample.baro_valid)
        self.assertAlmostEqual(sample.ax_mps2, 1.1, places=6)
        self.assertAlmostEqual(sample.gy_rads, 0.2, places=6)
        self.assertAlmostEqual(sample.pressure_pa, 100123.0, places=6)

    def test_decode_sensor_snapshot_rejects_bad_crc(self) -> None:
        payload = self._pack_sensor(crc_corrupt=True)
        self.assertIsNone(IPC.decode_sensor_snapshot(payload))

    def test_decode_sensor_snapshot_skip_crc_uses_python_path(self) -> None:
        payload = self._pack_sensor(crc_corrupt=True)
        sample = IPC.decode_sensor_snapshot(payload, check_crc=False)
        self.assertIsNotNone(sample)
        assert sample is not None
        self.assertEqual(sample.seq, 7)

    def test_decode_sensor_snapshot_rejects_bad_header(self) -> None:
        payload = self._pack_sensor(version=99)
        self.assertIsNone(IPC.decode_sensor_snapshot(payload))

    def test_encode_controller_command(self) -> None:
        msg = IPC.encode_controller_command(seq=9, t_ns=555, armed=True, servo_norm=(0.1, -0.2, 0.3, -0.4))
        fields = IPC.CONTROLLER_MSG_STRUCT.unpack(msg)
        self.assertEqual(fields[0], IPC.MESSAGE_MAGIC)
        self.assertEqual(fields[1], IPC.MESSAGE_VERSION)
        self.assertEqual(fields[2], IPC.CONTROLLER_PAYLOAD_BYTES)
        self.assertEqual(fields[3], 9)
        self.assertEqual(fields[4], 555)
        self.assertEqual(fields[5], 1)
        self.assertAlmostEqual(fields[6], 0.1, places=6)
        self.assertAlmostEqual(fields[9], -0.4, places=6)
        self.assertEqual(fields[10], IPC.finalize_crc(msg[:-4]))

    def test_encode_estimator_state(self) -> None:
        q = (0.7071067, 0.0, 0.7071067, 0.0)
        vel = (1.0, -2.0, 3.0)
        pos = (10.0, 20.0, -30.0)
        msg = IPC.encode_estimator_state(seq=11, t_ns=777, valid=True, q_body_to_ned=q, vel_ned_mps=vel, pos_ned_m=pos)
        fields = IPC.ESTIMATOR_MSG_STRUCT.unpack(msg)
        self.assertEqual(fields[0], IPC.MESSAGE_MAGIC)
        self.assertEqual(fields[1], IPC.MESSAGE_VERSION)
        self.assertEqual(fields[2], IPC.ESTIMATOR_PAYLOAD_BYTES)
        self.assertEqual(fields[3], 11)
        self.assertEqual(fields[4], 777)
        self.assertEqual(fields[5], 1)
        self.assertTrue(math.isclose(fields[6], q[0], rel_tol=0, abs_tol=1e-5))
        self.assertTrue(math.isclose(fields[10], vel[0], rel_tol=0, abs_tol=1e-6))
        self.assertTrue(math.isclose(fields[13], pos[0], rel_tol=0, abs_tol=1e-6))
        self.assertEqual(fields[16], IPC.finalize_crc(msg[:-4]))


if __name__ == "__main__":
    unittest.main()
