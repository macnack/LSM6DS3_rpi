import importlib
import sys
import types


def test_cli_smoke(monkeypatch, capsys):
  class FakeSensor:
    def __init__(self, *args, **kwargs):
      self.calls = 0

    def begin(self):
      return None

    def read_accel(self):
      self.calls += 1
      return (1.0, 2.0, 3.0)

    def read_gyro(self):
      raise KeyboardInterrupt

    def close(self, power_down=True):
      return None

  fake_ext = types.ModuleType("lsm6ds3_rpi._lsm6ds3")
  fake_ext.Lsm6ds3 = FakeSensor
  fake_ext.I2cError = RuntimeError
  fake_ext.AccelOdr = object
  fake_ext.GyroOdr = object
  fake_ext.AccelScale = object
  fake_ext.GyroScale = object

  monkeypatch.setitem(sys.modules, "lsm6ds3_rpi._lsm6ds3", fake_ext)

  pkg = importlib.import_module("lsm6ds3_rpi")
  cli = importlib.import_module("lsm6ds3_rpi.cli")
  monkeypatch.setattr(cli.time, "sleep", lambda *_args, **_kwargs: None)
  monkeypatch.setattr("sys.argv", ["lsm6ds3-stream", "--hz", "100"])

  try:
    cli.main()
  except KeyboardInterrupt:
    pass

  out = capsys.readouterr().out
  assert pkg.Lsm6ds3 is FakeSensor
  assert "timestamp,ax,ay,az,gx,gy,gz" in out
