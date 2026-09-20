#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="${1:-$(pwd)}"
PLUGIN="${WORKSPACE_DIR}/devel/lib/librotors_gazebo_multirotor_base_plugin.so"

status=0

check_contains() {
  local label="$1"
  local value="$2"
  local expected="$3"
  if [[ "${value}" == *"${expected}"* ]]; then
    echo "OK: ${label}: ${value}"
  else
    echo "ERROR: ${label}: expected ${expected}, got ${value}" >&2
    status=1
  fi
}

if command -v gazebo >/dev/null 2>&1; then
  gz_version="$(gazebo --version 2>/dev/null || true)"
  check_contains "Gazebo runtime" "${gz_version}" "11"
else
  echo "ERROR: gazebo executable not found" >&2
  status=1
fi

if command -v pkg-config >/dev/null 2>&1 && pkg-config --exists gazebo; then
  gz_pkg="$(pkg-config --modversion gazebo)"
  check_contains "Gazebo pkg-config" "${gz_pkg}" "11"
else
  echo "ERROR: pkg-config cannot find gazebo" >&2
  status=1
fi

if [[ -n "${GAZEBO_PLUGIN_PATH:-}" ]]; then
  if [[ "${GAZEBO_PLUGIN_PATH}" == *"${WORKSPACE_DIR}"* ]]; then
    echo "OK: GAZEBO_PLUGIN_PATH contains current workspace"
  else
    echo "WARN: GAZEBO_PLUGIN_PATH is set and may contain stale workspaces: ${GAZEBO_PLUGIN_PATH}" >&2
  fi
else
  echo "OK: GAZEBO_PLUGIN_PATH is clean before sourcing workspace"
fi

if [[ ! -f "${PLUGIN}" ]]; then
  echo "ERROR: missing RotorS plugin: ${PLUGIN}" >&2
  status=1
else
  mav_line="$(ldd "${PLUGIN}" | grep libmav_msgs || true)"
  gazebo_line="$(ldd "${PLUGIN}" | grep "libgazebo_.*\.so\.11" | head -n 1 || true)"
  if [[ -n "${mav_line}" && "${mav_line}" != *"not found"* ]]; then
    echo "OK: RotorS protobuf library linked: ${mav_line}"
  else
    echo "ERROR: libmav_msgs.so is not linked or not found" >&2
    status=1
  fi
  if [[ -n "${gazebo_line}" ]]; then
    echo "OK: RotorS plugin links Gazebo 11: ${gazebo_line}"
  else
    echo "ERROR: RotorS plugin is not linked against Gazebo 11" >&2
    status=1
  fi
fi

exit "${status}"
