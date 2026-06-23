# LT-DWA official wrapper plan for Scout

This document records the Scout-side integration boundary for the official LT-DWA ROS Noetic source vendored at `third_party/LT_DWA/`.

## Current status

- Official upstream source is vendored at `third_party/LT_DWA/`.
- The vendor tree is source-only and stays outside catkin-scanned `src/`.
- `third_party/LT_DWA/.git` is stripped; the code is tracked as ordinary source in this repository.
- Scout-owned wrapper package has been staged at `src/scout_apps/control/lt_dwa_official_wrapper/`.
- Required official catkin dependencies are copied into an isolated deps directory:
  - `src/scout_apps/control/lt_dwa_official_vendor_deps/obstacle_msgs/`
  - `src/scout_apps/control/lt_dwa_official_vendor_deps/local_map_generation/`
- Runtime `local_planner` package is staged at `tools/lt_dwa/local_planner_runtime/` so official-core preflight resolves `planning.config` and `data/` without exposing the whole upstream tree to catkin.
- Main-repo migration check on 2026-06-24 passed:
  - build PASS with `LT_DWA_WRAPPER_ENABLE_OFFICIAL_CORE=ON`
  - wrapper tests PASS: `126 tests, 0 errors, 0 failures, 0 skipped`
  - `roslaunch --nodes` PASS for shadow and benchmark launch overlays

This means the integration is no longer just a pending adapter idea. The current path is:

```text
Scout ROS topics
  -> map-frame odom adapter
  -> lt_dwa_official_wrapper bridge
  -> isolated lt_dwa_worker process
  -> official SeedPolicy::forward(...)
  -> structured worker result
  -> shadow command by default, or gated /cmd_vel only when explicitly enabled
```

## Required boundary

Allowed boundary:

```text
third_party/LT_DWA/ source-only vendor
  + Scout-owned lt_dwa_official_wrapper
  + isolated worker process
  + explicit runtime local_planner package precedence
  + default shadow-only output
```

Forbidden boundary:

```text
third_party/LT_DWA/* directly symlinked into src/
third_party/LT_DWA/local_planner replacing existing local_planner packages
official LT-DWA upstream edited in place for Scout runtime behavior
slosh monitor topics -> LT-DWA control input
wrapper bypassing benchmark preflight/readiness/freshness gates
real robot /cmd_vel enabled by default
```

## Build and runtime notes

Build only the wrapper and the two required vendor deps:

```bash
source /opt/ros/noetic/setup.bash
cd /home/geist/scout_ws
catkin_make \
  -DCATKIN_WHITELIST_PACKAGES="obstacle_msgs;local_map_generation;lt_dwa_official_wrapper" \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DLT_DWA_WRAPPER_ENABLE_OFFICIAL_CORE=ON
source /home/geist/scout_ws/devel/setup.bash
```

Launch official-core mode only after prepending the runtime package:

```bash
export SCOUT_WS_ROOT=/home/geist/scout_ws
export ROS_PACKAGE_PATH=$SCOUT_WS_ROOT/tools/lt_dwa/local_planner_runtime:$ROS_PACKAGE_PATH
```

Default real-machine launch remains shadow-only:

```bash
roslaunch lt_dwa_official_wrapper scout_sop_shadow_integration.launch \
  enable_actuated_output:=false \
  publish_cmd_vel:=false
```

## Remaining gates before formal real-machine closed-loop use

1. Confirm real-machine sensors, odom, TF, costmap, and `/scout/global_path_fixed` match the wrapper launch contract.
2. Confirm `rospack find local_planner` resolves to `tools/lt_dwa/local_planner_runtime/local_planner` in the LT-DWA launch terminal.
3. Confirm `local_map_generation` advertises `/local_map_generation/service` through the wrapper launch.
4. Run shadow-only diagnostics first; verify status, worker latency, command freshness, and no `/cmd_vel` publisher.
5. Treat any real-machine `/cmd_vel` test as a separate safety-reviewed action with bounded test window, conservative limits, E-stop readiness, and explicit launch args.
6. Keep `/slosh/*` and `/benchmark/slosh_monitor/*` evaluation-only; LT-DWA must not subscribe to them as control inputs.

## Refresh policy

When refreshing the vendor snapshot:

1. Inspect upstream commit and record it in `SCOUT_VENDOR_NOTES.md`.
2. Keep upstream `.git` metadata stripped if using ordinary vendored source, or pin any future submodule under `third_party/LT_DWA` only.
3. Preserve the Scout wrapper/process boundary; do not edit official upstream to depend on Scout-specific topics.
4. Re-run build/tests and launch parse checks before moving the updated snapshot toward sim or real-machine testing.
