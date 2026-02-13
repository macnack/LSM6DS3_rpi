import os
import subprocess
import sys
from pathlib import Path

from hatchling.builders.hooks.plugin.interface import BuildHookInterface


class BuildHook(BuildHookInterface):
    def initialize(self, version, build_data):
        target = build_data.get("target_name")
        if target and target != "wheel":
            return

        if os.environ.get("RPI_SENSORS_SKIP_SUBBUILDS") == "1":
            return

        root = Path(self.root)
        dist_dir = root / "dist"
        dist_dir.mkdir(parents=True, exist_ok=True)

        subprojects = [root / "IMU", root / "BARO"]

        for subproject in subprojects:
            if not subproject.exists():
                raise FileNotFoundError(
                    f"Missing subproject directory: {subproject}. "
                    "Ensure IMU/ and BARO/ are included in the sdist."
                )
            subprocess.run(
                [
                    sys.executable,
                    "-m",
                    "build",
                    "--wheel",
                    "--outdir",
                    str(dist_dir),
                    str(subproject),
                ],
                cwd=root,
                check=True,
            )


def get_build_hook():
    return BuildHook
