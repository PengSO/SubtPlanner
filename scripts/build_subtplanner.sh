#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="${1:-$(pwd)}"
if [[ ! -d "${WORKSPACE_DIR}/src" ]]; then
  echo "ERROR: pass the workspace root, for example: src/SubtPlanner/scripts/build_subtplanner.sh ~/SubtPlanner_ws" >&2
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/setup_gazebo_classic_env.sh"

cd "${WORKSPACE_DIR}"
rosdep install --from-paths src --ignore-src -r -y
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
source devel/setup.bash
"${SCRIPT_DIR}/check_runtime_env.sh" "${WORKSPACE_DIR}"
