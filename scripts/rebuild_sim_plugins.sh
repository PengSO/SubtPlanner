#!/usr/bin/env bash
set -euo pipefail
WORKSPACE_DIR="${1:-$(pwd)}"
if [[ ! -d "${WORKSPACE_DIR}/src" ]]; then
  echo "ERROR: pass the workspace root, for example: src/SubtPlanner/scripts/rebuild_sim_plugins.sh ~/SubtPlanner_ws" >&2
  exit 1
fi
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/setup_gazebo_classic_env.sh"
cd "${WORKSPACE_DIR}"
catkin clean -y rotors_gazebo_plugins rotors_gazebo subt_planner_gazebo
if [[ -d "${WORKSPACE_DIR}/devel" ]]; then
  find "${WORKSPACE_DIR}/devel" -maxdepth 3 -name 'libmav_msgs.so*' -delete
fi
catkin build rotors_gazebo_plugins rotors_gazebo subt_planner_gazebo
catkin build
source devel/setup.bash

"${SCRIPT_DIR}/check_runtime_env.sh" "${WORKSPACE_DIR}"
