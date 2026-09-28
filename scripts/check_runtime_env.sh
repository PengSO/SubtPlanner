#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="${1:-$(pwd)}"
PLUGIN="${WORKSPACE_DIR}/devel/lib/librotors_gazebo_multirotor_base_plugin.so"

status=0
garden_vars=(
  CMAKE_PREFIX_PATH
  GZ_CONFIG_PATH
  GZ_FILE_PATH
  GZ_GUI_PLUGIN_PATH
  GZ_PARTITION
  GZ_SIM_RESOURCE_PATH
  GZ_SIM_SYSTEM_PLUGIN_PATH
  IGN_CONFIG_PATH
  IGN_FILE_PATH
  IGN_GAZEBO_RESOURCE_PATH
  IGN_GAZEBO_SYSTEM_PLUGIN_PATH
  IGN_GUI_PLUGIN_PATH
  LD_LIBRARY_PATH
  PKG_CONFIG_PATH
  AMENT_PREFIX_PATH
  COLCON_PREFIX_PATH
)

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

for var_name in "${garden_vars[@]}"; do
  if [[ -n "${!var_name:-}" ]]; then
    if [[ "${!var_name}" == *garden* || "${!var_name}" == *ros_gz* || "${!var_name}" == *ignition* || "${!var_name}" == *gz-* || "${!var_name}" == *omniplanner* ]]; then
      echo "WARN: ${var_name} may contain Gazebo Sim / ROS-GZ paths: ${!var_name}" >&2
    fi
  fi
done

if [[ -n "${ROS_DISTRO:-}" && "${ROS_DISTRO}" != "noetic" ]]; then
  echo "WARN: ROS_DISTRO is ${ROS_DISTRO}, expected noetic for this workspace" >&2
fi

if [[ ! -f "${PLUGIN}" ]]; then
  echo "ERROR: missing RotorS plugin: ${PLUGIN}" >&2
  status=1
else
  mav_line="$(ldd "${PLUGIN}" | grep librotors_gazebo_mav_msgs || true)"
  gazebo_line="$(ldd "${PLUGIN}" | grep "libgazebo_.*\.so\.11" | head -n 1 || true)"
  if [[ -n "${mav_line}" && "${mav_line}" != *"not found"* ]]; then
    echo "OK: RotorS protobuf library linked: ${mav_line}"
  else
    echo "ERROR: librotors_gazebo_mav_msgs.so is not linked or not found" >&2
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
