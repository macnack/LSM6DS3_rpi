#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${ROOT_DIR}/build"

RT_CORE_BIN="${BUILD_DIR}/runtime/rt_core"
CPP_EST_BIN="${BUILD_DIR}/runtime/dummy_estimator_cpp"
CPP_CTRL_BIN="${BUILD_DIR}/runtime/dummy_controller_cpp"
PY_EST_SCRIPT="${ROOT_DIR}/runtime/python/dummy_estimator.py"
PY_CTRL_SCRIPT="${ROOT_DIR}/runtime/python/dummy_controller.py"

VALID_RUN_PRESETS="native | pydev | sim-py | sim-cpp | sim-mujoco"
VALID_BUILD_PRESETS="runtime | sim"

usage() {
  cat <<'USAGE'
Usage:
  ./rt.sh help
  ./rt.sh build [preset]
  ./rt.sh run <preset> [--print-config] [--status-file <path>]
  ./rt.sh run-estimator <preset>
  ./rt.sh run-controller <preset>
  ./rt.sh paths

Run presets:
  native      runtime/config/rt_core_cpp_native.toml
  pydev       runtime/config/rt_core_python_dev.toml
  sim-py      runtime/config/rt_core_sim_python_dev.toml
  sim-cpp     runtime/config/rt_core_sim_cpp_dev.toml
  sim-mujoco  runtime/config/rt_core_sim_mujoco_tcp.toml

Build presets:
  runtime  Build runtime-focused host binaries (default)
  sim      Build runtime with portable/sim modules (no IMU/BARO compile dep)
USAGE
}

die() {
  echo "error: $*" >&2
  exit 1
}

require_file() {
  local p="$1"
  [[ -f "$p" ]] || die "missing file: $p"
}

require_executable_file() {
  local p="$1"
  [[ -x "$p" ]] || die "missing executable: $p (run './rt.sh build')"
}

require_cmd() {
  local c="$1"
  command -v "$c" >/dev/null 2>&1 || die "required command not found: $c"
}

config_for_preset() {
  local preset="$1"
  case "$preset" in
    native) echo "${ROOT_DIR}/runtime/config/rt_core_cpp_native.toml" ;;
    pydev) echo "${ROOT_DIR}/runtime/config/rt_core_python_dev.toml" ;;
    sim-py) echo "${ROOT_DIR}/runtime/config/rt_core_sim_python_dev.toml" ;;
    sim-cpp) echo "${ROOT_DIR}/runtime/config/rt_core_sim_cpp_dev.toml" ;;
    sim-mujoco) echo "${ROOT_DIR}/runtime/config/rt_core_sim_mujoco_tcp.toml" ;;
    *) return 1 ;;
  esac
}

detect_jobs() {
  if command -v nproc >/dev/null 2>&1; then
    nproc
    return
  fi
  if command -v sysctl >/dev/null 2>&1; then
    sysctl -n hw.ncpu 2>/dev/null || echo 4
    return
  fi
  echo 4
}

cmd_build() {
  local preset="${1:-runtime}"
  local jobs
  jobs="$(detect_jobs)"

  require_cmd cmake

  case "$preset" in
    runtime)
      cmake -S "$ROOT_DIR" -B "$BUILD_DIR" \
        -DBUILD_IGNITER=ON \
        -DBUILD_RUNTIME=ON \
        -DLSM6DS3_BUILD_PYTHON=OFF \
        -DBMP390_BUILD_PYTHON=OFF \
        -DSERVO_BUILD_PYTHON=OFF \
        -DIGNITER_BUILD_PYTHON=OFF \
        -DADS1115_BUILD_PYTHON=OFF
      ;;
    sim)
      cmake -S "$ROOT_DIR" -B "$BUILD_DIR" \
        -DBUILD_IGNITER=ON \
        -DBUILD_RUNTIME=ON \
        -DBUILD_IMU=OFF \
        -DBUILD_BARO=OFF \
        -DBUILD_SERVO=ON \
        -DLSM6DS3_BUILD_PYTHON=OFF \
        -DBMP390_BUILD_PYTHON=OFF \
        -DSERVO_BUILD_PYTHON=OFF \
        -DIGNITER_BUILD_PYTHON=OFF \
        -DADS1115_BUILD_PYTHON=OFF
      ;;
    *)
      die "unknown build preset '$preset'. valid: ${VALID_BUILD_PRESETS}"
      ;;
  esac

  cmake --build "$BUILD_DIR" -j"$jobs"
}

cmd_run() {
  local preset="$1"
  local config
  local print_config=0
  local status_file=""

  config="$(config_for_preset "$preset")" || die "unknown run preset '$preset'. valid: ${VALID_RUN_PRESETS}"
  require_file "$config"
  require_executable_file "$RT_CORE_BIN"

  shift
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --print-config)
        print_config=1
        shift
        ;;
      --status-file)
        [[ $# -ge 2 ]] || die "--status-file requires a path"
        status_file="$2"
        shift 2
        ;;
      *)
        die "unknown option for run: $1"
        ;;
    esac
  done

  local cmd=("$RT_CORE_BIN" "--config" "$config")
  if [[ "$print_config" -eq 1 ]]; then
    cmd+=("--print-config")
  fi
  if [[ -n "$status_file" ]]; then
    cmd+=("--status-file" "$status_file")
  fi

  exec "${cmd[@]}"
}

cmd_run_estimator() {
  local preset="$1"
  local config
  config="$(config_for_preset "$preset")" || die "unknown run preset '$preset'. valid: ${VALID_RUN_PRESETS}"
  require_file "$config"

  case "$preset" in
    pydev|sim-py)
      require_cmd python3
      require_file "$PY_EST_SCRIPT"
      exec python3 "$PY_EST_SCRIPT" --config "$config"
      ;;
    sim-cpp)
      require_executable_file "$CPP_EST_BIN"
      exec "$CPP_EST_BIN" --config "$config"
      ;;
    native|sim-mujoco)
      die "preset '$preset' does not use external estimator worker"
      ;;
    *)
      die "unknown run preset '$preset'. valid: ${VALID_RUN_PRESETS}"
      ;;
  esac
}

cmd_run_controller() {
  local preset="$1"
  local config
  config="$(config_for_preset "$preset")" || die "unknown run preset '$preset'. valid: ${VALID_RUN_PRESETS}"
  require_file "$config"

  case "$preset" in
    pydev|sim-py)
      require_cmd python3
      require_file "$PY_CTRL_SCRIPT"
      exec python3 "$PY_CTRL_SCRIPT" --config "$config"
      ;;
    sim-cpp)
      require_executable_file "$CPP_CTRL_BIN"
      exec "$CPP_CTRL_BIN" --config "$config"
      ;;
    native|sim-mujoco)
      die "preset '$preset' does not use external controller worker"
      ;;
    *)
      die "unknown run preset '$preset'. valid: ${VALID_RUN_PRESETS}"
      ;;
  esac
}

cmd_paths() {
  cat <<EOF
root_dir: $ROOT_DIR
build_dir: $BUILD_DIR
rt_core: $RT_CORE_BIN
dummy_estimator_cpp: $CPP_EST_BIN
dummy_controller_cpp: $CPP_CTRL_BIN
dummy_estimator_py: $PY_EST_SCRIPT
dummy_controller_py: $PY_CTRL_SCRIPT
config_native: ${ROOT_DIR}/runtime/config/rt_core_cpp_native.toml
config_pydev: ${ROOT_DIR}/runtime/config/rt_core_python_dev.toml
config_sim_py: ${ROOT_DIR}/runtime/config/rt_core_sim_python_dev.toml
config_sim_cpp: ${ROOT_DIR}/runtime/config/rt_core_sim_cpp_dev.toml
config_sim_mujoco: ${ROOT_DIR}/runtime/config/rt_core_sim_mujoco_tcp.toml
EOF
}

main() {
  local sub="${1:-help}"
  shift || true

  case "$sub" in
    help|-h|--help)
      usage
      ;;
    build)
      cmd_build "${1:-runtime}"
      ;;
    run)
      [[ $# -ge 1 ]] || die "run requires a preset. valid: ${VALID_RUN_PRESETS}"
      cmd_run "$@"
      ;;
    run-estimator)
      [[ $# -eq 1 ]] || die "run-estimator requires exactly one preset"
      cmd_run_estimator "$1"
      ;;
    run-controller)
      [[ $# -eq 1 ]] || die "run-controller requires exactly one preset"
      cmd_run_controller "$1"
      ;;
    paths)
      [[ $# -eq 0 ]] || die "paths takes no arguments"
      cmd_paths
      ;;
    *)
      die "unknown command '$sub'. run './rt.sh help'"
      ;;
  esac
}

main "$@"
