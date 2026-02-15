from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

from tools import rt_status_report


def test_share_token_round_trip_preserves_metrics() -> None:
    status = {
        "sim_mode": "true",
        "degraded_mode_active": "false",
        "killswitch_active": "false",
        "last_failsafe_reason_name": "None",
        "failsafe_enter_count": "0",
        "control_deadline_miss_count": "0",
        "actuator_deadline_miss_count": "1",
        "external_estimator_reject_count": "2",
        "external_controller_reject_count": "0",
        "control_jitter_p95_ns": "80000",
        "control_jitter_max_ns": "130000",
    }

    snapshot = rt_status_report.build_share_snapshot(status, Path("/tmp/rt_status.txt"))
    token = rt_status_report.encode_share_token(snapshot)
    decoded = rt_status_report.decode_share_token(token)

    assert decoded["v"] == rt_status_report.SHARE_SCHEMA_VERSION
    assert decoded["source"] == "rt_status.txt"
    assert decoded["health"] == "yellow"
    metrics = decoded["metrics"]
    assert isinstance(metrics, dict)
    assert metrics["actuator_deadline_miss_count"] == "1"
    assert metrics["external_estimator_reject_count"] == "2"


def test_open_share_cli_logs_open_event(tmp_path: Path) -> None:
    status = {
        "sim_mode": "false",
        "degraded_mode_active": "false",
        "killswitch_active": "false",
        "last_failsafe_reason_name": "None",
        "failsafe_enter_count": "0",
    }
    snapshot = rt_status_report.build_share_snapshot(status, Path("/tmp/share_status.txt"))
    token = rt_status_report.encode_share_token(snapshot)

    repo_root = Path(__file__).resolve().parents[2]
    script = repo_root / "tools" / "rt_status_report.py"
    event_log = tmp_path / "events.jsonl"

    env = os.environ.copy()
    env["RT_STATUS_REPORT_EVENT_LOG"] = str(event_log)
    result = subprocess.run(
        [sys.executable, str(script), "--no-color", "--open-share", token],
        cwd=repo_root,
        capture_output=True,
        text=True,
        check=False,
        env=env,
    )

    assert result.returncode == 0, result.stderr
    assert "Shared runtime snapshot" in result.stdout

    records = [json.loads(line) for line in event_log.read_text().splitlines()]
    assert any(record.get("event") == "share_snapshot_opened" for record in records)
