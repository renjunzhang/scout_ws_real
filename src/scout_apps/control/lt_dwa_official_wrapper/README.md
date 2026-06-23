# lt_dwa_official_wrapper

Scout-owned wrapper for running the official LT-DWA ROS Noetic core through an isolated worker process.

## Safety boundary

- Official LT-DWA source is kept source-only under `third_party/LT_DWA`.
- Do **not** symlink or copy `third_party/LT_DWA` into catkin `src/`; upstream contains generic package names such as `local_planner` and `navigation` that can conflict with this workspace.
- This wrapper package is the only Scout-facing runtime integration point.
- Default launch behavior is shadow-only: no `/cmd_vel` unless both `enable_actuated_output:=true` and `publish_cmd_vel:=true` are explicitly supplied.
- Worker-side `/tf` and `/tf_static` are remapped to sandbox topics by default.

## Layout

```text
third_party/LT_DWA/                                      # official upstream snapshot, source-only
src/scout_apps/control/lt_dwa_official_wrapper/          # Scout wrapper/bridge/worker
tools/lt_dwa/local_planner_runtime/local_planner/        # runtime planning.config/data for official core
src/scout_apps/control/lt_dwa_official_vendor_deps/      # catkin deps copied from official LT-DWA: obstacle_msgs, local_map_generation
```

## Build

Use an explicit catkin whitelist so the official source-only tree is not built accidentally:

```bash
source /opt/ros/noetic/setup.bash
cd /home/geist/scout_ws
catkin_make \
  -DCATKIN_WHITELIST_PACKAGES="obstacle_msgs;local_map_generation;lt_dwa_official_wrapper" \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DLT_DWA_WRAPPER_ENABLE_OFFICIAL_CORE=ON
source /home/geist/scout_ws/devel/setup.bash
```

Development-machine equivalent replaces `/home/geist/scout_ws` with the local checkout path.

## Runtime environment

Before launching official-core mode, make sure the runtime `local_planner` package takes precedence over any conflicting package:

```bash
export SCOUT_WS_ROOT=/home/geist/scout_ws
export ROS_PACKAGE_PATH=$SCOUT_WS_ROOT/tools/lt_dwa/local_planner_runtime:$ROS_PACKAGE_PATH
```

This lets `ros::package::getPath("local_planner")` resolve to the wrapper runtime package, not to any unrelated workspace package.

## Launch checks

Package/launch parse check:

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
rospack find lt_dwa_official_wrapper
rospack find local_map_generation
roslaunch --nodes lt_dwa_official_wrapper scout_sop_shadow_integration.launch
```

Shadow-only Scout SOP integration:

```bash
export SCOUT_WS_ROOT=/home/geist/scout_ws
export ROS_PACKAGE_PATH=$SCOUT_WS_ROOT/tools/lt_dwa/local_planner_runtime:$ROS_PACKAGE_PATH
roslaunch lt_dwa_official_wrapper scout_sop_shadow_integration.launch \
  enable_actuated_output:=false \
  publish_cmd_vel:=false
```

Actuated output must remain an explicit, reviewed action:

```bash
roslaunch lt_dwa_official_wrapper scout_sop_cmd_vel_benchmark.launch \
  enable_actuated_output:=true \
  publish_cmd_vel:=true
```

Do not use the actuated command on the real robot without separate safety review, speed/acceleration limits, E-stop readiness, and a bounded test window.
