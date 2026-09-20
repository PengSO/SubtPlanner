# SubtPlanner World Assets

This package contains Gazebo model assets used by the SubtPlanner simulation demos.
The planner itself does not depend on the full asset set at compile time, but Gazebo
requires the referenced models at runtime.

## Recommended Release Policy

Large subterranean scenes should not be committed directly to the main planner
repository. A clean public release should use one of the following options:

- a separate `subtplanner_world_assets` repository with Git LFS enabled;
- a versioned GitHub Release archive;
- a DOI-backed archive such as Zenodo for camera-ready reproducibility.

The main planner repository should keep only lightweight launch files, world files,
configuration files, and optionally the small corridor demo model.

## Asset Groups

| Model group | Approx. size | Scene |
| --- | ---: | --- |
| `underground_corridor` | 21 MB | `urban_corridor.launch` |
| `niosh_seg01` | 315 MB | `mine_tunnel.launch` |
| DARPA cave modular pieces | 600+ MB | `natural_cave.launch` |
| `megacavern` | 951 MB | `darpa_subt.launch` |

## Runtime Layout

Gazebo launch files expect this package to expose its models through:

```bash
$(find subt_planner_world_assets)/models
```

The launch files append that path to `GAZEBO_MODEL_PATH`, so users only need to
place the asset package under the ROS workspace and rebuild or re-source the
workspace.
