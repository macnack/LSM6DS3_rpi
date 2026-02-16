#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE_NAME="${IMAGE_NAME:-lsm6ds3-rpi-build}"
CONTAINER_NAME="${CONTAINER_NAME:-lsm6ds3-rpi-dev}"
PLATFORM="${PLATFORM:-linux/arm64}"
DOCKERFILE="${DOCKERFILE:-Dockerfile.rpi}"

usage() {
  cat <<USAGE
Usage: ./run.sh [--no-build]

Builds (unless --no-build) the Pi-like image and starts a reusable dev container.

Env overrides:
  IMAGE_NAME      (default: lsm6ds3-rpi-build)
  CONTAINER_NAME  (default: lsm6ds3-rpi-dev)
  PLATFORM        (default: linux/arm64)
  DOCKERFILE      (default: Dockerfile.rpi)
USAGE
}

NO_BUILD=0
if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  usage
  exit 0
fi
if [[ "${1:-}" == "--no-build" ]]; then
  NO_BUILD=1
elif [[ $# -gt 0 ]]; then
  echo "Unknown argument: $1" >&2
  usage >&2
  exit 1
fi

if [[ ! -f "$ROOT_DIR/$DOCKERFILE" ]]; then
  echo "Missing $DOCKERFILE in $ROOT_DIR" >&2
  exit 1
fi

if [[ "$NO_BUILD" -eq 0 ]]; then
  docker buildx build \
    --platform "$PLATFORM" \
    -f "$ROOT_DIR/$DOCKERFILE" \
    -t "$IMAGE_NAME" \
    --load \
    "$ROOT_DIR"
fi

if docker container inspect "$CONTAINER_NAME" >/dev/null 2>&1; then
  if [[ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER_NAME")" == "true" ]]; then
    echo "Container '$CONTAINER_NAME' is already running."
  else
    docker start "$CONTAINER_NAME" >/dev/null
    echo "Started existing container '$CONTAINER_NAME'."
  fi
else
  docker run -d \
    --name "$CONTAINER_NAME" \
    --platform "$PLATFORM" \
    -v "$ROOT_DIR:/work" \
    -w /work \
    "$IMAGE_NAME" \
    tail -f /dev/null >/dev/null
  echo "Created and started container '$CONTAINER_NAME'."
fi

echo "Use ./enter.sh to open a shell inside the container."
