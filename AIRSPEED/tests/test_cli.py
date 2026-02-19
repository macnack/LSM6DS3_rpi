import importlib
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
  for module_name in ["ms4525do_rpi", "ms4525do_rpi.cli"]:
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

  fake_ext = types.ModuleType("ms4525do_rpi._ms4525do")
  fake_ext.Ms4525do = sensor_cls
  fake_ext.I2cError = RuntimeError
  fake_ext.Calibration = FakeCalibration
  fake_ext.ReadPolicy = FakeReadPolicy
  fake_ext.OutputType = FakeOutputType
  fake_ext.Status = object
  fake_ext.Reading = object
  monkeypatch.setitem(sys.modules, "ms4525do_rpi._ms4525do", fake_ext)


def test_cli_stream_outputs_header_and_closes_on_interrupt(monkeypatch, capsys):
  class FakeReading:
    pressure_psi = 0.1
    pressure_pa = 689.4757
    temperature_c = 25.0
    pressure_counts = 9000
    temperature_counts = 1000
    status = 0

  class FakeSensor:
    def __init__(self, *args, **kwargs):
      self.close_calls = 0

    def begin(self):
      return None

    def read(self):
      raise KeyboardInterrupt

    def close(self):
      self.close_calls += 1
      return None

  install_fake_extension(monkeypatch, FakeSensor)
  cli = importlib.import_module("ms4525do_rpi.cli")
  monkeypatch.setattr(cli.time, "sleep", lambda *_args, **_kwargs: None)
  monkeypatch.setattr("sys.argv", ["ms4525do-stream", "--hz", "100"])

  assert FakeReading.status == 0
  assert cli.main() == 0

  out = capsys.readouterr().out
  assert "timestamp,pressure_psi,pressure_pa,temperature_c,pressure_counts,temperature_counts,status" in out


def test_cli_rejects_non_positive_hz(monkeypatch):
  class FakeSensor:
    def __init__(self, *args, **kwargs):
      return None

  install_fake_extension(monkeypatch, FakeSensor)
  cli = importlib.import_module("ms4525do_rpi.cli")
  monkeypatch.setattr("sys.argv", ["ms4525do-stream", "--hz", "0"])

  with pytest.raises(SystemExit) as exc_info:
    cli.main()

  assert exc_info.value.code == 2


def test_cli_rejects_out_of_range_i2c_address(monkeypatch):
  class FakeSensor:
    def __init__(self, *args, **kwargs):
      return None

  install_fake_extension(monkeypatch, FakeSensor)
  cli = importlib.import_module("ms4525do_rpi.cli")
  monkeypatch.setattr("sys.argv", ["ms4525do-stream", "--address", "0x80"])

  with pytest.raises(SystemExit) as exc_info:
    cli.main()

  assert exc_info.value.code == 2
