import os
import subprocess
import sys
from pathlib import Path

from hatchling.builders.hooks.plugin.interface import BuildHookInterface


class CustomBuildHook(BuildHookInterface):
    def initialize(self, version, build_data):
        target = build_data.get("target_name")
        if target != "wheel":
            return

        if os.environ.get("RPI_SENSORS_SKIP_SUBBUILDS") == "1":
            return

        root = Path(self.root)
        dist_dir = root / "dist"
        dist_dir.mkdir(parents=True, exist_ok=True)

        subprojects = [
            root / "IMU",
            root / "BARO",
        ]

        for subproject in subprojects:
            subprocess.run(
                [
                    sys.executable,
                    "-m",
                    "build",
                    "--wheel",
                    "--outdir",
                    str(dist_dir),
                ],
                cwd=subproject,
                check=True,
            )
