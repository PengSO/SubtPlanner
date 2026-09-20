# Underground Corridor Gazebo Model

This folder is the drop-in Gazebo model template for a corridor scene converted
from your `.pcd` file.

## Required mesh output

Place your converted mesh here:

- `meshes/underground_corridor.dae`

That is the only mandatory file for the template to resolve in Gazebo.

## Local helper script

This workspace now includes a small helper at:

- `src/simulation/subt_planner_world_assets/tools/pcd_scene_tool.py`

Use it to inspect a `.pcd` and print heuristic `subt_planner` bounds:

```bash
python3 src/simulation/subt_planner_world_assets/tools/pcd_scene_tool.py inspect /abs/path/scene.pcd
```

Use it to clean and reconstruct a mesh, then export `ply`, `obj`, and a
geometry-only `dae`, plus a Gazebo-friendly `stl`:

```bash
python3 src/simulation/subt_planner_world_assets/tools/pcd_scene_tool.py mesh \
  /abs/path/scene.pcd \
  --out-dir src/simulation/subt_planner_world_assets/models/underground_corridor/meshes \
  --basename underground_corridor
```

The generated `underground_corridor.stl` is the most reliable default for this
workspace's Gazebo setup. If the result is too dense for Gazebo, lower
`--target-triangles` or increase `--voxel-size`.

## Recommended conversion pipeline

1. Clean the point cloud in CloudCompare or PCL.
   - Remove isolated outliers.
   - Downsample if the cloud is very dense.
   - Make sure units are meters.
2. Estimate normals.
3. Reconstruct a watertight or near-watertight mesh.
   - Poisson reconstruction is the easiest starting point.
   - Ball Pivoting may preserve thin structures better if normals are good.
4. Simplify the mesh.
   - Keep the visual mesh detailed enough to preserve corridor walls.
   - If Gazebo becomes slow, create a second low-poly collision mesh.
5. Export to `dae` in a Z-up frame.

## Optional assets

If your DAE references textures, keep them relative to this model folder, for
example:

- `materials/textures/...`

## Typical issues and where to fix them

- Wrong scale:
  edit `model.sdf` and change the mesh `<scale>`.
- Wrong world origin:
  edit `gzc/worlds/underground_corridor.world` and change the `<pose>` of the
  model include.
- Gazebo physics too slow:
  create a decimated collision mesh and point the `<collision>` mesh URI to it.
- Corridor is upside down or rotated:
  fix the axis during export to DAE, or rotate the mesh in Blender before
  export. Avoid compensating large frame errors only in the world pose.
