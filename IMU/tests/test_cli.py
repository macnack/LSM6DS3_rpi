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
  for module_name in ["lsm6ds3_rpi", "lsm6ds3_rpi.cli"]:
    sys.modules.pop(module_name, None)
  yield


def install_fake_extension(monkeypatch, sensor_cls, spi_sensor_cls=None):
  if spi_sensor_cls is None:
    spi_sensor_cls = sensor_cls

  fake_ext = types.ModuleType("lsm6ds3_rpi._lsm6ds3")
  fake_ext.Lsm6ds3 = sensor_cls
  fake_ext.Lsm6ds3Spi = spi_sensor_cls
  fake_ext.I2cError = RuntimeError
  fake_ext.SpiError = RuntimeError
  fake_ext.AccelOdr = object
  fake_ext.GyroOdr = object
  fake_ext.AccelScale = object
  fake_ext.GyroScale = object
  monkeypatch.setitem(sys.modules, "lsm6ds3_rpi._lsm6ds3", fake_ext)


def test_cli_stream_outputs_header_and_closes_on_interrupt(monkeypatch, capsys):
  class FakeSensor:
    def __init__(self, *args, **kwargs):
      self.close_calls = 0

    def begin(self):
      return None

    def read_accel(self):
      return (1.0, 2.0, 3.0)

    def read_gyro(self):
      raise KeyboardInterrupt

    def close(self, power_down=True):
      self.close_calls += 1
      return None

  install_fake_extension(monkeypatch, FakeSensor)
  pkg = importlib.import_module("lsm6ds3_rpi")
  cli = importlib.import_module("lsm6ds3_rpi.cli")
  monkeypatch.setattr(cli.time, "sleep", lambda *_args, **_kwargs: None)
  monkeypatch.setattr("sys.argv", ["lsm6ds3-stream", "--hz", "100"])

  assert cli.main() == 0

  out = capsys.readouterr().out
  assert pkg.Lsm6ds3 is FakeSensor
  assert "timestamp,ax,ay,az,gx,gy,gz" in out


def test_cli_stream_can_select_spi_interface(monkeypatch):
  class FakeI2cSensor:
    def __init__(self, *args, **kwargs):
      self.close_calls = 0

    def begin(self):
      return None

    def read_accel(self):
      return (0.0, 0.0, 0.0)

    def read_gyro(self):
      raise KeyboardInterrupt

    def close(self, power_down=True):
      self.close_calls += 1
      return None

  class FakeSpiSensor:
    init_calls = 0

    def __init__(self, *args, **kwargs):
      FakeSpiSensor.init_calls += 1
      self.close_calls = 0

    def begin(self):
      return None

    def read_accel(self):
      return (1.0, 2.0, 3.0)

    def read_gyro(self):
      raise KeyboardInterrupt

    def close(self, power_down=True):
      self.close_calls += 1
      return None

  install_fake_extension(monkeypatch, FakeI2cSensor, FakeSpiSensor)
  cli = importlib.import_module("lsm6ds3_rpi.cli")
  monkeypatch.setattr("sys.argv", [
      "lsm6ds3-stream",
      "--interface", "spi",
      "--spi-device", "/dev/spidev0.0",
      "--spi-speed", "2000000",
      "--spi-mode", "3",
      "--hz", "100",
  ])

  assert cli.main() == 0
  assert FakeSpiSensor.init_calls == 1


def test_cli_rejects_non_positive_hz(monkeypatch):
  class FakeSensor:
    def __init__(self, *args, **kwargs):
      return None

  install_fake_extension(monkeypatch, FakeSensor)
  cli = importlib.import_module("lsm6ds3_rpi.cli")
  monkeypatch.setattr("sys.argv", ["lsm6ds3-stream", "--hz", "0"])

  with pytest.raises(SystemExit) as exc_info:
    cli.main()

  assert exc_info.value.code == 2


def test_cli_rejects_out_of_range_i2c_address(monkeypatch):
  class FakeSensor:
    def __init__(self, *args, **kwargs):
      return None

  install_fake_extension(monkeypatch, FakeSensor)
  cli = importlib.import_module("lsm6ds3_rpi.cli")
  monkeypatch.setattr("sys.argv", ["lsm6ds3-stream", "--address", "0x80"])

  with pytest.raises(SystemExit) as exc_info:
    cli.main()

  assert exc_info.value.code == 2
