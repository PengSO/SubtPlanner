# Open-Source Release Checklist

This checklist describes how to publish SubtPlanner without carrying hidden local
workspace assumptions or oversized Gazebo assets into the main repository.

## 1. Repository Split

Publish the project in two layers.

### Main Repository

Keep source code, launch files, scene configs, the lightweight corridor demo, and
release scripts:

```text
SubtPlanner/
  README.md
  MODULES.md
  .gitignore
  docs/
  scripts/
  subt_planner/
  local_exploration/
  global_repositioning/
  simulation/
    subt_planner_gazebo/
    lidar_description/
    lidar_gazebo_plugins/
    rotors_simulator/
    subt_planner_world_assets/
      README.md
      package.xml
      CMakeLists.txt
      models/underground_corridor/
  third_party/
```

### External Full Asset Bundle

Do not put the full 2 GB subterranean assets in the main repository. Publish them
as a separate Git LFS repository, GitHub Release archive, or Zenodo archive:

```text
subtplanner_world_assets_full/
  models/niosh_seg01/
  models/megacavern/
  models/Cave Starting Area/
  models/Cave Straight 01/
  ...
```

The target runtime layout after extraction should be:

```text
SubtPlanner_ws/src/simulation/subt_planner_world_assets/models/<full-scene-models>
```

## 2. Pre-Upload Cleanup

Run these checks before creating the first public commit:

```bash
cd ~/SubtPlanner_ws/src
find . -name build -o -name devel -o -name install -o -name logs
find . -name __pycache__ -o -name "*.pyc"
du -sh simulation/subt_planner_world_assets/models/* | sort -h
```

Only `models/underground_corridor` should stay in the main repository by default.
Full scene models should be removed from the main repository copy before upload
or kept ignored by `.gitignore`.

## 3. First Main Repository Upload

For the first public upload, use a clean staging copy instead of committing
directly from a long-lived development workspace:

```bash
mkdir -p ~/release_staging
rsync -a --delete --exclude='.git' --exclude='build' --exclude='devel' \
  --exclude='install' --exclude='logs' ~/SubtPlanner_ws/src/ ~/release_staging/SubtPlanner/
cd ~/release_staging/SubtPlanner
```

Then create the main repository commit:

```bash
git init
git add README.md MODULES.md .gitignore docs scripts \
  subt_planner local_exploration global_repositioning simulation third_party
git status
```

Check that these are not staged in the main repository:

```text
simulation/subt_planner_world_assets/models/megacavern/
simulation/subt_planner_world_assets/models/niosh_seg01/
simulation/subt_planner_world_assets/models/Cave*/
```

Then commit and push:

```bash
git commit -m "Initial SubtPlanner release"
git branch -M main
git remote add origin https://github.com/PengSO/SubtPlanner.git
git push -u origin main
```

## 4. Full Asset Upload Option A: GitHub Release Archive

Create an archive outside the main repository:

```bash
cd ~/SubtPlanner_ws/src/simulation/subt_planner_world_assets
tar --exclude=".catkin_tools" -czf ~/subtplanner_world_assets_full.tar.gz \
  models/niosh_seg01 \
  models/megacavern \
  models/Cave* \
  models/Base\ Station \
  models/JanSport\ Backpack\ Red \
  models/Rescue\ Randy\ Sitting \
  models/Samsung\ J8\ Black
```

Attach `subtplanner_world_assets_full.tar.gz` to a versioned GitHub Release.

## 5. Full Asset Upload Option B: Separate Git LFS Repository

```bash
mkdir -p ~/subtplanner_world_assets_full/models
cp -a ~/SubtPlanner_ws/src/simulation/subt_planner_world_assets/models/niosh_seg01 ~/subtplanner_world_assets_full/models/
cp -a ~/SubtPlanner_ws/src/simulation/subt_planner_world_assets/models/megacavern ~/subtplanner_world_assets_full/models/
cp -a ~/SubtPlanner_ws/src/simulation/subt_planner_world_assets/models/Cave* ~/subtplanner_world_assets_full/models/
cd ~/subtplanner_world_assets_full
git init
git lfs install
git lfs track "*.dae" "*.stl" "*.obj" "*.png" "*.jpg" "*.jpeg" "*.tga" "*.bmp"
git add .gitattributes models
git commit -m "Add full SubtPlanner simulation assets"
git remote add origin <ASSET_REPOSITORY_URL>
git push -u origin main
```

## 6. New-Machine Validation

On a clean machine, validate the public instructions with:

```bash
cd ~/SubtPlanner_ws
./src/scripts/install_system_deps.sh
./src/scripts/build_subtplanner.sh ~/SubtPlanner_ws
./src/scripts/check_runtime_env.sh ~/SubtPlanner_ws
roslaunch subt_planner urban_corridor.launch rviz_en:=false
```

If Gazebo reports `undefined symbol: _ZN14gz_sensor_msgs9ActuatorsC1Ev`, rebuild
simulation plugins:

```bash
./src/scripts/rebuild_sim_plugins.sh ~/SubtPlanner_ws
```

The runtime check must report that `libmav_msgs.so` is linked and that RotorS
plugins link against Gazebo 11.
