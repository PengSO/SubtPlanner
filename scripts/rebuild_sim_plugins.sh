#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="${1:-$(pwd)}"
if [[ ! -d "${WORKSPACE_DIR}/src" ]]; then
  echo "ERROR: pass the workspace root, for example: scripts/rebuild_sim_plugins.sh ~/SubtPlanner_ws" >&2
  exit 1
fi

unset GAZEBO_PLUGIN_PATH
unset GAZEBO_MODEL_PATH
source /usr/share/gazebo/setup.sh
source /opt/ros/noetic/setup.bash

cd "${WORKSPACE_DIR}"
catkin clean -y rotors_gazebo_plugins rotors_gazebo subt_planner_gazebo
catkin build rotors_gazebo_plugins rotors_gazebo subt_planner_gazebo
catkin build
source devel/setup.bash
"${WORKSPACE_DIR}/src/scripts/check_runtime_env.sh" "${WORKSPACE_DIR}"
