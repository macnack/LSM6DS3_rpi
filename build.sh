#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  cat <<'USAGE'
Usage:
  ./build.sh [preset]

Presets:
  runtime  default runtime-focused build
  sim      runtime sim/portable build (IMU/BARO disabled at compile time)
USAGE
  exit 0
fi

exec "${ROOT_DIR}/rt.sh" build "$@"
