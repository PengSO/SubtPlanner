# Urban Corridor Gazebo Model

This folder contains the Gazebo model used by the `urban_corridor` demo.

## Mesh

The meshes used by `model.sdf` are:

- `meshes/underground_corridor.stl`
- `meshes/underground_corridor_visual.stl`

The world file resolves this model through `GAZEBO_MODEL_PATH`.

## Optional assets

If your DAE references textures, keep them relative to this model folder, for
example:

- `materials/textures/...`

## Typical issues and where to fix them

- Wrong scale:
  edit `model.sdf` and change the mesh `<scale>`.
- Wrong world origin:
  edit `subt_planner_gazebo/worlds/urban_corridor.world`.
- Gazebo physics too slow:
  create a decimated collision mesh and point the `<collision>` mesh URI to it.
