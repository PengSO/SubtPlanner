#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="${1:-$(pwd)}"
source /opt/ros/noetic/setup.bash
source "${WORKSPACE_DIR}/devel/setup.bash"
"${WORKSPACE_DIR}/src/scripts/check_runtime_env.sh" "${WORKSPACE_DIR}"
roslaunch subt_planner urban_corridor.launch rviz_en:=false
