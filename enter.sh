#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${CONTAINER_NAME:-lsm6ds3-rpi-dev}"

if ! docker container inspect "$CONTAINER_NAME" >/dev/null 2>&1; then
  echo "Container '$CONTAINER_NAME' does not exist." >&2
  echo "Run ./run.sh first." >&2
  exit 1
fi

if [[ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER_NAME")" != "true" ]]; then
  docker start "$CONTAINER_NAME" >/dev/null
fi

exec docker exec -it "$CONTAINER_NAME" bash
