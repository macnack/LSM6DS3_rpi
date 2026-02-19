import importlib
import json
import pathlib
import sys
import types

import pytest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
PYTHON_PACKAGE_DIR = REPO_ROOT / "python"
if str(PYTHON_PACKAGE_DIR) not in sys.path:
  sys.path.insert(0, str(PYTHON_PACKAGE_DIR))


@pytest.fixture(autouse=True)
def clear_package_modules():
  for module_name in [
      "ms4525do_rpi",
      "ms4525do_rpi.cli",
      "ms4525do_rpi.calibrate",
  ]:
    sys.modules.pop(module_name, None)
  yield


def install_fake_extension(monkeypatch, sensor_cls):
  class FakeCalibration:
    def __init__(self):
      self.p_min_psi = -1.0
      self.p_max_psi = 1.0
      self.output_type = "b"

  class FakeOutputType:
    TYPE_A_10_TO_90 = "a"
    TYPE_B_5_TO_95 = "b"

  class FakeReadPolicy:
    REQUIRE_FRESH = 0
    ALLOW_STALE = 1

  class FakeStatus:
    NORMAL = 0
    STALE_DATA = 2
    COMMAND_MODE = 1
    DIAGNOSTIC_FAULT = 3

  fake_ext = types.ModuleType("ms4525do_rpi._ms4525do")
  fake_ext.Ms4525do = sensor_cls
  fake_ext.I2cError = RuntimeError
  fake_ext.Calibration = FakeCalibration
  fake_ext.ReadPolicy = FakeReadPolicy
  fake_ext.OutputType = FakeOutputType
  fake_ext.Status = FakeStatus
  fake_ext.Reading = object
  monkeypatch.setitem(sys.modules, "ms4525do_rpi._ms4525do", fake_ext)


def test_calibrate_success_json(monkeypatch, capsys, tmp_path):
  class Reading:
    def __init__(self, pressure_pa):
      self.pressure_pa = pressure_pa
      self.status = 0

  class FakeSensor:
    def __init__(self, *args, **kwargs):
      self.i = 0

    def begin(self):
      return None

    def read(self):
      self.i += 1
      if self.i <= 5:
        return Reading(0.2)  # zero offset phase
      return Reading(80.0)  # positive verification phase

    def close(self):
      return None

  install_fake_extension(monkeypatch, FakeSensor)
  mod = importlib.import_module("ms4525do_rpi.calibrate")
  monkeypatch.setattr(mod.time, "sleep", lambda *_a, **_k: None)
  reverse_file = tmp_path / "tmp_reverse_success.json"
  reverse_file.write_text(json.dumps({"reverse_ports": True}), encoding="utf-8")
  monkeypatch.setattr("sys.argv", [
      "airspeed-calibrate",
      "--json",
      "--non-interactive",
      "--zero-samples", "5",
      "--verify-threshold-pa", "50",
      "--verify-timeout-sec", "2",
      "--sample-hz", "100",
      "--reverse-state-file", str(reverse_file),
  ])

  assert mod.main() == 0
  payload = json.loads(capsys.readouterr().out.strip())
  assert payload["sensor"] == "ms4525do"
  assert payload["offset_pa"] > 0.0
  persisted = json.loads(pathlib.Path(payload["reverse_state_file"]).read_text())
  assert persisted["reverse_ports"] is False
  pathlib.Path(payload["reverse_state_file"]).unlink(missing_ok=True)


def test_calibrate_negative_direction_fails_and_persists_reverse(monkeypatch, tmp_path):
  class Reading:
    def __init__(self, pressure_pa):
      self.pressure_pa = pressure_pa
      self.status = 0

  class FakeSensor:
    def __init__(self, *args, **kwargs):
      self.i = 0

    def begin(self):
      return None

    def read(self):
      self.i += 1
      if self.i <= 5:
        return Reading(0.1)
      return Reading(-80.0)

    def close(self):
      return None

  install_fake_extension(monkeypatch, FakeSensor)
  mod = importlib.import_module("ms4525do_rpi.calibrate")
  monkeypatch.setattr(mod.time, "sleep", lambda *_a, **_k: None)
  state_path = tmp_path / "reverse_state.json"
  monkeypatch.setattr("sys.argv", [
      "airspeed-calibrate",
      "--non-interactive",
      "--zero-samples", "5",
      "--verify-threshold-pa", "50",
      "--verify-timeout-sec", "2",
      "--sample-hz", "100",
      "--reverse-state-file", str(state_path),
  ])

  assert mod.main() == 2
  persisted = json.loads(state_path.read_text())
  assert persisted["reverse_ports"] is True
  assert persisted["reason"] == "negative_direction"


def test_calibrate_negative_direction_no_persist_leaves_existing_state(monkeypatch, tmp_path):
  class Reading:
    def __init__(self, pressure_pa):
      self.pressure_pa = pressure_pa
      self.status = 0

  class FakeSensor:
    def __init__(self, *args, **kwargs):
      self.i = 0

    def begin(self):
      return None

    def read(self):
      self.i += 1
      if self.i <= 5:
        return Reading(0.1)
      return Reading(-80.0)

    def close(self):
      return None

  install_fake_extension(monkeypatch, FakeSensor)
  mod = importlib.import_module("ms4525do_rpi.calibrate")
  monkeypatch.setattr(mod.time, "sleep", lambda *_a, **_k: None)
  state_path = tmp_path / "reverse_state.json"
  original = {"reverse_ports": False, "reason": "seed"}
  state_path.write_text(json.dumps(original), encoding="utf-8")
  monkeypatch.setattr("sys.argv", [
      "airspeed-calibrate",
      "--non-interactive",
      "--zero-samples", "5",
      "--verify-threshold-pa", "50",
      "--verify-timeout-sec", "2",
      "--sample-hz", "100",
      "--no-persist-reverse-on-negative",
      "--reverse-state-file", str(state_path),
  ])

  assert mod.main() == 2
  persisted = json.loads(state_path.read_text())
  assert persisted == original


def test_calibrate_experimental_is_robust_to_outlier(monkeypatch, capsys):
  class Reading:
    def __init__(self, pressure_pa):
      self.pressure_pa = pressure_pa
      self.status = 0

  class FakeSensor:
    def __init__(self, *args, **kwargs):
      self.i = 0

    def begin(self):
      return None

    def read(self):
      self.i += 1
      baseline = [0.2, 0.1, 300.0, 0.1, 0.2, 0.0, 0.1]
      if self.i <= len(baseline):
        return Reading(baseline[self.i - 1])
      return Reading(80.0)

    def close(self):
      return None

  install_fake_extension(monkeypatch, FakeSensor)
  mod = importlib.import_module("ms4525do_rpi.calibrate")
  monkeypatch.setattr(mod.time, "sleep", lambda *_a, **_k: None)
  monkeypatch.setattr("sys.argv", [
      "airspeed-calibrate",
      "--json",
      "--non-interactive",
      "--experimental",
      "--zero-samples", "7",
      "--verify-threshold-pa", "50",
      "--verify-timeout-sec", "2",
      "--sample-hz", "100",
  ])

  assert mod.main() == 0
  payload = json.loads(capsys.readouterr().out.strip())
  assert payload["experimental"] is True
  assert payload["offset_stats"]["mode"] == "experimental"


def test_calibrate_timeout_logs_failure_reason(monkeypatch, capsys):
  class Reading:
    def __init__(self, pressure_pa):
      self.pressure_pa = pressure_pa
      self.status = 0

  class FakeSensor:
    def __init__(self, *args, **kwargs):
      self.i = 0

    def begin(self):
      return None

    def read(self):
      self.i += 1
      if self.i <= 5:
        return Reading(0.1)
      return Reading(1.0)

    def close(self):
      return None

  install_fake_extension(monkeypatch, FakeSensor)
  mod = importlib.import_module("ms4525do_rpi.calibrate")
  monkeypatch.setattr(mod.time, "sleep", lambda *_a, **_k: None)
  monkeypatch.setattr("sys.argv", [
      "airspeed-calibrate",
      "--non-interactive",
      "--zero-samples", "5",
      "--verify-threshold-pa", "50",
      "--verify-timeout-sec", "0.01",
      "--sample-hz", "100",
  ])

  assert mod.main() == 2
  err = capsys.readouterr().err
  assert "FAILED SUMMARY: reason=verify_timeout outcome=timeout" in err


def test_calibrate_stale_acceptance_requires_allow_stale(monkeypatch):
  class Reading:
    def __init__(self, pressure_pa, status):
      self.pressure_pa = pressure_pa
      self.status = status

  class FakeSensor:
    def __init__(self, *args, **kwargs):
      self.i = 0

    def begin(self):
      return None

    def read(self):
      self.i += 1
      if self.i <= 5:
        return Reading(0.1, 0)  # normal for zero-offset
      return Reading(80.0, 2)  # stale during verify phase

    def close(self):
      return None

  install_fake_extension(monkeypatch, FakeSensor)
  mod = importlib.import_module("ms4525do_rpi.calibrate")
  monkeypatch.setattr(mod.time, "sleep", lambda *_a, **_k: None)

  monkeypatch.setattr("sys.argv", [
      "airspeed-calibrate",
      "--non-interactive",
      "--zero-samples", "5",
      "--verify-threshold-pa", "50",
      "--verify-timeout-sec", "0.01",
      "--sample-hz", "100",
  ])
  assert mod.main() == 2

  monkeypatch.setattr("sys.argv", [
      "airspeed-calibrate",
      "--non-interactive",
      "--allow-stale",
      "--zero-samples", "5",
      "--verify-threshold-pa", "50",
      "--verify-timeout-sec", "0.5",
      "--sample-hz", "100",
  ])
  assert mod.main() == 0
