from __future__ import annotations

import subprocess
import sys
from pathlib import Path


def test_rt_ipc_dump_help_runs_from_repo_root_without_editable_install() -> None:
    repo_root = Path(__file__).resolve().parents[2]
    script = repo_root / "tools" / "rt_ipc_dump.py"

    result = subprocess.run(
        [sys.executable, str(script), "--help"],
        cwd=repo_root,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert "Dump runtime IPC mailboxes." in result.stdout
