#!/usr/bin/env bash
set -euo pipefail

sudo apt update
sudo apt install -y \
  build-essential cmake git git-lfs curl wget \
  python3-catkin-tools python3-vcstool python3-rosdep python3-pip \
  libboost-all-dev libeigen3-dev libgoogle-glog-dev libgflags-dev \
  libprotobuf-dev protobuf-compiler libyaml-cpp-dev libsqlite3-dev libzmq3-dev \
  libpcl-dev libopencv-dev qtbase5-dev \
  gazebo11 libgazebo11-dev \
  ros-noetic-desktop-full \
  ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control \
  ros-noetic-joy ros-noetic-octomap ros-noetic-octomap-msgs ros-noetic-octomap-ros \
  ros-noetic-pcl-ros ros-noetic-pcl-conversions ros-noetic-cv-bridge \
  ros-noetic-tf ros-noetic-tf2-ros ros-noetic-tf-conversions \
  ros-noetic-interactive-markers ros-noetic-rviz ros-noetic-xacro

sudo rosdep init 2>/dev/null || true
rosdep update
git lfs install
