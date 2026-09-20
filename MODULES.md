# Module Boundaries

SubtPlanner is split into four visible source groups plus vendored dependencies.

## subt_planner

The `subt_planner` tree is the integration layer. It owns the ROS entrypoint, mission tree nodes, service callbacks, shared planner state, cognitive-map support code, voxel-map wrapper, message interfaces, execution bridge, configuration files, and operator UI.

Use this package for code that coordinates more than one planner component or exposes the system to ROS.

## local_exploration

`local_exploration` contains the structure-guided local explorer. It builds and evaluates the local exploration graph, samples local viewpoints, scores field-of-view gain, forms safe local paths, and publishes local exploration visualization markers.

This package should not select long-range global frontiers or own return-home behavior.

## global_repositioning

`global_repositioning` contains topology-aware global target selection and recovery. It handles branch-aware frontier ranking, dead-zone escape, return/repositioning path selection, and construction of local repositioning regions.

This package should not expand the local exploration graph.

## simulation

`simulation` contains the migrated corridor demo and supporting robot/sensor/Gazebo packages. The corridor launch path is intentionally shallow under `simulation/subt_planner_gazebo/launch`.

## third_party

`third_party` contains external dependencies. Keep planner contributions out of this tree unless a dependency must be patched explicitly.
