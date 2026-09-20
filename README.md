<div align="center">
  <h1>SubtPlanner</h1>
  <h3>Structure-Guided Sampling and Branch-Aware Repositioning for UAV Autonomous Subterranean Exploration</h3>
   <h1> </h1>


  <p>
    <a href="#quick-start"><img alt="ROS" src="https://img.shields.io/badge/ROS-Noetic-22314E"/></a>
    <a href="#quick-start"><img alt="Demo" src="https://img.shields.io/badge/Demo-Corridor-1f8f6a"/></a>
    <a href="#code-organization"><img alt="Modules" src="https://img.shields.io/badge/Modules-Cognitive%20Map%20%7C%20Local%20Exploration%20%7C%20Global%20Repositioning-d97735"/></a>
    <a href="#citation"><img alt="Paper" src="https://img.shields.io/badge/Paper-ICRA%202027-lightgrey"/></a>
  </p>
</div>

<p align="center">
  <img src="../docs/assets/method_overview.jpg" width="640" alt="SubtPlanner method overview"/>
</p>

## Overview

**SubtPlanner** is a cognitive-map-based hierarchical active planner for UAV autonomous exploration in subterranean environments. It couples short-horizon local exploration with long-term topological memory, allowing the robot to keep exploring efficiently in long corridors, branch-rich tunnels, caves, and dead-end-prone spaces.

The planner is organized around three method concepts:

- **Cognitive map**: an incremental representation composed of a local exploration graph and a global topological graph.
- **Local exploration**: structure-guided sampling using passable directions, field-of-view gain, and safety corridors.
- **Global repositioning**: branch-aware frontier selection with dead-zone handling, U-shaped escape, and return-path reasoning.

## Updates

- **2026.09** Corridor demo, RViz profile, terminal status view, and paper-aligned package layout are available.
- **2026.09** Core planner packages are organized into `subt_planner`, `local_exploration`, `global_repositioning`, `simulation`, and `third_party`.

## Highlights

### Cognitive-map-based autonomous exploration

SubtPlanner maintains a cognitive map that connects local exploration decisions with a long-term topological memory. The planner is designed for long, branch-rich, and dead-end-prone subterranean scenes where purely local exploration is easily trapped.

### Structure-guided local viewpoint sampling

Local viewpoints are sampled around feasible passage directions and evaluated with information gain, safety, heading consistency, and reachability. This keeps local exploration focused while preserving enough candidates for cluttered tunnel geometry.

### Branch-aware global repositioning

When local gain degrades, the planner switches to branch-aware global reasoning. Frontier candidates are grouped by topological branch, unproductive dead-zone regions are suppressed, and the planner can escape from cul-de-sac geometry before selecting a new exploration branch.

## Method

### Structure-Guided Local Exploration

<p align="center">
  <img src="../docs/assets/local_exploration.jpg" width="680" alt="Structure-guided local exploration"/>
</p>

The local planner builds a short-horizon exploration graph around the current robot state. Candidate viewpoints are concentrated around feasible passage directions instead of being produced by dense blind sampling. The planner then evaluates information gain, reachability, heading consistency, and safety before selecting the local reference path.

### Branch-Aware Global Repositioning

<p align="center">
  <img src="../docs/assets/global_repositioning.jpg" width="640" alt="Branch-aware global repositioning"/>
</p>

When local exploration becomes low-gain or trapped, global repositioning reasons over the topological graph. It groups frontier candidates by branch, suppresses repeatedly unproductive dead-zone regions, and can prepend a U-shaped escape maneuver before returning to local exploration.

## Simulation Preview

<p align="center">
  <img src="../docs/assets/simulation_scenarios.jpg" width="640" alt="Subterranean simulation scenarios"/>
</p>

<p align="center">
  <img src="../docs/assets/simulation_trajectories_maps.jpg" width="640" alt="Exploration trajectories and maps"/>
</p>

## Contents

- [Overview](#overview)
- [Method](#method)
- [Quick Start](#quick-start)
- [Expected Result](#expected-result)
- [Code Organization](#code-organization)
- [Configuration](#configuration)
- [Known Issues](#known-issues)
- [Roadmap](#roadmap)
- [Development Notes](#development-notes)
- [Citation](#citation)
- [Acknowledgements](#acknowledgements)

## Quick Start

### Tested Environment

SubtPlanner is maintained on the same style of ROS/Gazebo stack used by many subterranean exploration systems.

| Component | Version |
| --- | --- |
| OS | Ubuntu 20.04 |
| ROS | Noetic |
| Simulator | Gazebo Classic 11 |
| Build system | `catkin_tools` |
| Compiler | GCC 9 / C++17 |

### 1. System Dependencies

Install the ROS and system packages before building the workspace. This list is intentionally explicit because development machines often already contain packages pulled in by previous GBPlanner3, RotorS, or voxblox workspaces.

```bash
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
```

Initialize `rosdep` if it has not been initialized on the machine:

```bash
sudo rosdep init 2>/dev/null || true
rosdep update
```

Enable Git LFS for optional simulation assets:

```bash
git lfs install
```

### Gazebo ABI Note

RotorS Gazebo plugins must be compiled against the same Gazebo ABI that will load them at runtime. The original RotorS ecosystem was often built around older Gazebo releases, including Gazebo 9, while Ubuntu 20.04 with ROS Noetic uses Gazebo Classic 11. Do not reuse prebuilt Gazebo 9 RotorS plugins with Gazebo 11.

Before building SubtPlanner on a clean machine, make sure Gazebo 11 is the active simulator:

```bash
gazebo --version
pkg-config --modversion gazebo
```

Both commands should report Gazebo 11.x. If your shell was previously used for GBPlanner3, RotorS, or another simulator workspace, remove stale plugin paths before building:

```bash
unset GAZEBO_PLUGIN_PATH
unset GAZEBO_MODEL_PATH
source /usr/share/gazebo/setup.sh
source /opt/ros/noetic/setup.bash
```

The `rotors_gazebo_plugins` package in this workspace is built from source and links against the local Gazebo 11 libraries during `catkin build`.

### 2. Create Workspace

```bash
mkdir -p ~/SubtPlanner_ws/src
cd ~/SubtPlanner_ws/src
git clone <SUBTPLANNER_REPOSITORY_URL> .
```

If the repository is cloned as a folder named `SubtPlanner`, use this layout instead:

```bash
mkdir -p ~/SubtPlanner_ws/src
cd ~/SubtPlanner_ws/src
git clone <SUBTPLANNER_REPOSITORY_URL> SubtPlanner
```

### 3. Simulation Assets

The planner code is small, but full subterranean Gazebo scenes are not. The current full asset set is about **2.0 GB**, mostly from DARPA/OSRF scene meshes and textures:

| Asset group | Approx. size | Used by |
| --- | ---: | --- |
| `underground_corridor` | 21 MB | `urban_corridor.launch` |
| DARPA cave pieces | 600+ MB | `natural_cave.launch` |
| `niosh_seg01` | 315 MB | `mine_tunnel.launch` |
| `megacavern` | 951 MB | `darpa_subt.launch` |

For GitHub release, keep the main repository lightweight and publish large models separately, following the pattern used by GBPlanner3-style releases:

- Keep the corridor demo asset in the main repository or in a small `subtplanner_demo_assets` bundle.
- Put full scene assets in a separate `subtplanner_world_assets` repository using Git LFS, or attach them as versioned GitHub Release/Zenodo archives.
- Do not commit generated Gazebo caches, build folders, or converted debug meshes.

Expected asset package location:

```text
src/simulation/subt_planner_world_assets/
  package.xml
  CMakeLists.txt
  models/
    underground_corridor/
    niosh_seg01/              # optional full scene
    megacavern/               # optional full scene
    Cave Starting Area/       # optional full scene
    ...
```

For a minimal public demo, only `underground_corridor` is required.

### 4. Install ROS Package Dependencies

From the workspace root:

```bash
cd ~/SubtPlanner_ws
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

Some third-party libraries are vendored under `src/third_party` to make the planner easier to build without depending on an existing GBPlanner3 workspace. Keep them under `third_party` and avoid mixing them into planner packages.

### 5. Build

For a first build, or after changing Gazebo versions, build from a clean workspace so that RotorS plugins are compiled against Gazebo 11 instead of stale Gazebo 9 headers or libraries. The recommended release build entry is:

```bash
cd ~/SubtPlanner_ws
./src/scripts/build_subtplanner.sh ~/SubtPlanner_ws
```

Manual equivalent:

```bash
cd ~/SubtPlanner_ws
unset GAZEBO_PLUGIN_PATH
unset GAZEBO_MODEL_PATH
source /usr/share/gazebo/setup.sh
source /opt/ros/noetic/setup.bash
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
source devel/setup.bash
./src/scripts/check_runtime_env.sh ~/SubtPlanner_ws
```

If you previously built the workspace against another Gazebo version, clean the simulator packages before rebuilding:

```bash
./src/scripts/rebuild_sim_plugins.sh ~/SubtPlanner_ws
```

The runtime check verifies that RotorS plugins link against Gazebo 11 and that `libmav_msgs.so`, the RotorS protobuf helper library, is visible to Gazebo at runtime.

### 6. Run Corridor Demo

```bash
cd ~/SubtPlanner_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch subt_planner urban_corridor.launch
```

Headless smoke test:

```bash
roslaunch subt_planner urban_corridor.launch rviz_en:=false
```

Optional full scenes, after installing the full asset bundle:

```bash
roslaunch subt_planner mine_tunnel.launch
roslaunch subt_planner natural_cave.launch
roslaunch subt_planner darpa_subt.launch
```

The default RViz profile keeps the view lightweight: only `Displays` and `Task Control Pane` are shown, the grid is disabled by default, and display groups follow the method concepts: cognitive map, local exploration, and global repositioning.

## Expected Result

A healthy corridor run should show the following progression:

```text
[Mission]             SubtPlanner startup banner
[Mission]             Switch to auto mode
[Local Exploration]   Triggered
[Cognitive Map]       Formed a graph with ... vertices and ... edges
[Local Exploration]   Path selected / local navigation continues
```

In RViz, the display tree is organized around the paper components:

- **Cognitive Map**: topological graph, exploration graph, frontiers, and selected paths.
- **Local Exploration**: local candidate viewpoints, local path, gain-related markers, and nearby map evidence.
- **Global Repositioning**: branch target, dead-zone recovery markers, and global return/repositioning path.

The robot should start moving after the first local planning iteration. For a headless smoke test:

```bash
roslaunch subt_planner urban_corridor.launch rviz_en:=false
```

## Code Organization

The repository is arranged by the method decomposition used in the paper. The top-level packages are intentionally small in number so that the exploration logic can be inspected without walking through a large ROS workspace.

```text
src/
  subt_planner/             Mission-level planner, cognitive map, ROS execution, visualization, messages, and demo configs
  local_exploration/        Structure-guided local exploration graph construction, gain evaluation, and local path generation
  global_repositioning/     Branch-aware global target reasoning, dead-zone escape geometry, and repositioning-region utilities
  simulation/               Gazebo corridor demo assets, robot models, sensors, and launch files
  third_party/              External libraries kept separate from planner-specific code
```

Inside `subt_planner`, the release layout separates the paper-facing planner substrate from ROS support code:

```text
subt_planner/
  exploration_manager/      Main ROS node, mission state machine, launch files, behavior-tree XML, and scene configs
  cognitive_map/graph_model/ Cognitive graph, exploration/topological graph data structures, search, trajectories, and parameters
  cognitive_map/voxel_map/  Voxel-map wrapper over voxblox backends
  visualization/            RViz marker publishing for cognitive map, local exploration, and global repositioning
  execution/                Planner-control interface and runtime bridge
  messages/                 ROS messages and services used by the planner
  rviz_panel/               Operator control plugin
```

The current bridge for branch-aware global repositioning is compiled with `subt_planner` because it needs mission state, local frontier verification, and topological-map access at the same time. The decision logic uses the `global_repositioning` package for repositioning-region geometry and orientation search, while the long-term target selection remains visible in `exploration_manager` as mission-level coordination.

## Configuration

Scene parameters live under `src/subt_planner/exploration_manager/configs/<scene>/`.

| File | Purpose |
| --- | --- |
| `subt_planner_config.yaml` | Robot, sensor, bounded-space, sampling, local exploration, and global repositioning parameters |
| `voxblox_sim_config.yaml` | Voxel mapping backend settings |
| `planner_control_interface_sim_config.yaml` | Trajectory execution bridge settings |
| `ui.rviz` | RViz display profile |

The corridor configuration is the reference demo profile. The compact corridor profile exposes the parameters used by the demo, while the advanced profile keeps a broader experiment template for ablation and stress testing.

## Known Issues

- The corridor scene is the maintained release demo. Other migrated scenes are kept as experiment templates and may require additional asset or sensor tuning before use.
- RViz visualization is grouped by method components, but the displayed topics still depend on the active launch profile and enabled planner modules.
- Third-party packages may emit CMake deprecation warnings on ROS Noetic. These warnings do not affect the corridor demo.
- RotorS Gazebo plugins must be rebuilt from source against Gazebo 11. Prebuilt Gazebo 9 plugins are ABI-incompatible with the Noetic/Gazebo 11 runtime.

## Troubleshooting

| Symptom | First checks |
| --- | --- |
| RViz opens but shows no map or markers | Confirm `source devel/setup.bash`, keep `rviz_en:=true`, and check that `/rmf_obelix/velodyne_points` is publishing. |
| Planner starts but does not move | Check for `Switch to auto mode`, `[Local Exploration] Triggered`, and graph formation messages in the terminal. |
| Map is empty in corridor demo | Confirm the launch file is `urban_corridor.launch` and that Gazebo loaded the `subt_planner_gazebo` corridor world. |
| Gazebo crashes when loading `librotors_gazebo_*` | Check `gazebo --version`; rebuild `rotors_gazebo_plugins` from source against Gazebo 11 and remove stale Gazebo 9 paths from `GAZEBO_PLUGIN_PATH`. |
| `undefined symbol: _ZN14gz_sensor_msgs9ActuatorsC1Ev` | Clean and rebuild `rotors_gazebo_plugins`; the RotorS protobuf helper library `libmav_msgs.so` must be exported and linked next to the Gazebo plugins. Check with `ldd devel/lib/librotors_gazebo_multirotor_base_plugin.so | grep mav_msgs`. |
| Build warnings from googletest or voxblox | These are third-party warnings on ROS Noetic and do not affect the corridor demo. |

## Release Notes

Open-source release preparation is tracked in [`docs/OPEN_SOURCE_RELEASE.md`](docs/OPEN_SOURCE_RELEASE.md). The short version is:

- keep the main repository lightweight and corridor-demo ready;
- keep full subterranean assets outside the main repository;
- build RotorS Gazebo plugins from source against Gazebo Classic 11;
- validate new machines with `scripts/check_runtime_env.sh`.

## Roadmap

- Add a clean cave demo profile after validating map loading, planner triggering, and RViz displays.
- Provide a compact parameter-tuning guide for local exploration, branch-aware repositioning, and dead-zone escape.
- Add recorded demo bags or screenshots for the reference corridor run.
- Add CI-style build instructions for release verification.

## Development Notes

- Put exploration-graph expansion, local viewpoint sampling, FOV gain scoring, and local path selection in `local_exploration`.
- Put branch-aware geometry, dead-zone escape construction, and orientation search in `global_repositioning`; keep mission-level target selection in `subt_planner/exploration_manager` when it needs shared planner state.
- Put ROS scheduling, mission tree logic, service routing, shared state, cognitive/voxel map wrappers, execution integration, visualization, and operator controls in `subt_planner`.
- Put scene assets and Gazebo launch files in `simulation`.
- Keep planner-specific code out of `third_party`.

## Citation

If this project supports your research, please cite the associated paper:

```bibtex
@inproceedings{subtplanner2027,
  title     = {SubtPlanner: Structure-Guided Sampling and Branch-Aware Repositioning for UAV Autonomous Subterranean Exploration},
  author    = {Peng, Song and Zhang, Hao and Li, Han and Jiang, Weicheng and Gou, Guohua and Sui, Haigang},
  booktitle = {IEEE International Conference on Robotics and Automation},
  year      = {2027}
}
```

## Acknowledgements

SubtPlanner builds on the ROS, Gazebo, behavior-tree, and voxblox ecosystems. The repository keeps these dependencies separated under `third_party` so that the exploration planner itself remains easier to inspect and extend.
