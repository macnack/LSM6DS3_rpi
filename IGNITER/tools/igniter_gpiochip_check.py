#!/usr/bin/env python3
from __future__ import annotations

import importlib.util
from pathlib import Path


def _load_main():
    here = Path(__file__).resolve()
    module_path = here.parent.parent / "python" / "igniter_rpi" / "tools" / "gpiochip_check.py"
    spec = importlib.util.spec_from_file_location("_igniter_gpiochip_check", module_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load checker module: {module_path}")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.main


if __name__ == "__main__":
    main = _load_main()
    raise SystemExit(main())
