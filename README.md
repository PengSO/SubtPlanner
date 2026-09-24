<div align="center">
  <h1>SubtPlanner</h1>
  <p><strong>Structure-Guided Sampling and Branch-Aware Repositioning for UAV Autonomous Subterranean Exploration</strong></p>
</div>

<p align="center">
  <a href="VIDEO_URL"><img alt="Video" src="https://img.shields.io/badge/Video-YouTube-ff0033"/></a>
  <a href="PAPER_URL"><img alt="Paper" src="https://img.shields.io/badge/Paper-ICRA%202027-2f80ed"/></a>
</p>

## ✨ Overview

**SubtPlanner** is a cognitive-map-based hierarchical active planner for UAV autonomous exploration in subterranean environments. It couples short-horizon local exploration with long-term topological memory, allowing the robot to keep exploring efficiently in long corridors, branch-rich tunnels, caves, and dead-end-prone spaces.

<p align="center">
  <img src="SubtPlanner/docs/Method_Overview.jpg" width="640" alt="SubtPlanner method overview"/>
</p>

The planner is organized around three method concepts:

- **Cognitive map**: an incremental representation composed of a local exploration graph and a global topological graph.
- **Local exploration**: structure-guided sampling using passable directions, field-of-view gain, and safety corridors.
- **Global repositioning**: branch-aware frontier selection with dead-zone handling, U-shaped escape, and return-path reasoning.

## 🗞️ Updates

- **2026.09** Corridor demo, RViz profile, terminal status view, and paper-aligned package layout are available.
- **2026.09** Core planner packages are organized into `subt_planner`, `local_exploration`, `global_repositioning`, `simulation`, and `third_party`.

## 🎞️ Simulation Preview

<p align="center">
  <img src="../docs/assets/simulation_scenarios.jpg" width="640" alt="Subterranean simulation scenarios"/>
</p>

<p align="center">
  <img src="../docs/Experiment_Simulation_Trajectories_Maps.jpg" width="640" alt="Exploration trajectories and maps"/>
</p>

## ⚙️ Quick Start

### 🧪 Tested Environment

SubtPlanner is maintained on the same style of ROS/Gazebo stack used by many subterranean exploration systems.

| Component | Version |
| --- | --- |
| OS | Ubuntu 20.04 |
| ROS | Noetic |
| Simulator | Gazebo Classic 11 |
| Build system | `catkin_tools` |
| Compiler | GCC 9 / C++17 |

### 📦 1. Prerequisites

Required base environment: Ubuntu 20.04, ROS Noetic, and Gazebo Classic 11.

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git git-lfs curl wget \
  python3-catkin-tools python3-vcstool python3-rosdep python3-pip \
  libboost-all-dev libeigen3-dev libgoogle-glog-dev libgflags-dev \
  libprotobuf-dev protobuf-compiler libyaml-cpp-dev libsqlite3-dev libzmq3-dev \
  libpcl-dev libopencv-dev qtbase5-dev

git lfs install
gazebo --version
pkg-config --modversion gazebo
```

### 📥 2. Create Workspace

```bash
mkdir -p ~/SubtPlanner_ws/src
cd ~/SubtPlanner_ws/src
git clone https://github.com/PengSO/SubtPlanner.git .
```

If clone checkout failed with `git-lfs: not found`:

```bash
cd ~/SubtPlanner_ws/src
git lfs pull
git restore --source=HEAD :/
```

### 🛠️ 3. Build

```bash
cd ~/SubtPlanner_ws
./src/scripts/build_subtplanner.sh ~/SubtPlanner_ws
```

Manual build:

```bash
cd ~/SubtPlanner_ws
source ./src/scripts/setup_gazebo_classic_env.sh
rosdep install --from-paths src --ignore-src -r -y
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
source devel/setup.bash
./src/scripts/check_runtime_env.sh ~/SubtPlanner_ws
```

Rebuild simulator plugins after changing Gazebo environments:

```bash
cd ~/SubtPlanner_ws
./src/scripts/rebuild_sim_plugins.sh ~/SubtPlanner_ws
```

### 🚁 4. Run Corridor Demo

Recommended once per machine:

```bash
echo 'source ~/SubtPlanner_ws/src/scripts/setup_gazebo_classic_env.sh' >> ~/.bashrc
source ~/.bashrc
```

Run in each new terminal:

```bash
cd ~/SubtPlanner_ws
source devel/setup.bash
roslaunch subt_planner urban_corridor.launch
```

Temporary shell:

```bash
cd ~/SubtPlanner_ws
source ./src/scripts/setup_gazebo_classic_env.sh
source devel/setup.bash
roslaunch subt_planner urban_corridor.launch
```

Headless smoke test:

```bash
cd ~/SubtPlanner_ws
source devel/setup.bash
roslaunch subt_planner urban_corridor.launch rviz_en:=false
```

## 📖 Citation

If this project supports your research, please cite the associated paper:

```bibtex
@inproceedings{subtplanner2027,
  title     = {SubtPlanner: Structure-Guided Sampling and Branch-Aware Repositioning for UAV Autonomous Subterranean Exploration},
  author    = {Peng, Song and Zhang, Hao and Li, Han and Jiang, Weicheng and Gou, Guohua and Sui, Haigang},
  booktitle = {IEEE International Conference on Robotics and Automation},
  year      = {2027}
}
```

## 🙏 Acknowledgements

We thank the following projects for their valuable influence on subterranean
exploration research and simulation workflows:

- [GBPlanner2](https://github.com/ntnu-arl/gbplanner_ros/wiki)
- [OmniPlanner](https://github.com/ntnu-arl/gbplanner_ros)
