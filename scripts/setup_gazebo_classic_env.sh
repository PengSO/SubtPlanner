#!/usr/bin/env bash

# Source this file before building or launching the ROS Noetic demo from a shell
# that may have previously loaded Gazebo Sim / ROS-GZ / ROS 2 environments.
#
# Usage:
#   source src/scripts/setup_gazebo_classic_env.sh

unset GAZEBO_PLUGIN_PATH
unset GAZEBO_MODEL_PATH
unset GAZEBO_RESOURCE_PATH
unset GAZEBO_MASTER_URI

unset GZ_CONFIG_PATH
unset GZ_FILE_PATH
unset GZ_GUI_PLUGIN_PATH
unset GZ_PARTITION
unset GZ_SIM_RESOURCE_PATH
unset GZ_SIM_SYSTEM_PLUGIN_PATH
unset GZ_TRANSPORT_TOPIC_STATISTICS

unset IGN_CONFIG_PATH
unset IGN_FILE_PATH
unset IGN_GAZEBO_RESOURCE_PATH
unset IGN_GAZEBO_SYSTEM_PLUGIN_PATH
unset IGN_GUI_PLUGIN_PATH
unset IGN_PARTITION
unset IGN_TRANSPORT_TOPIC_STATISTICS

unset AMENT_PREFIX_PATH
unset COLCON_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset LD_LIBRARY_PATH
unset PKG_CONFIG_PATH
unset PYTHONPATH
unset ROS_VERSION
unset ROS_DISTRO
unset ROS_PYTHON_VERSION

source /usr/share/gazebo/setup.sh
source /opt/ros/noetic/setup.bash
