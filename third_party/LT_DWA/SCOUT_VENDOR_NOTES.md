# Scout vendor notes for LT-DWA

Upstream source: <https://github.com/flztiii/LT_DWA>

Vendored commit inspected before import: `6f49cce`.

Import date: 2026-06-19.

Vendor method: upstream `.git` metadata stripped; source tracked as ordinary repo files.

Local modifications policy: keep upstream source unchanged where possible; put Scout-specific notes and adapter/wrapper plans in `SCOUT_*.md` files.

This directory is intentionally kept under `third_party/LT_DWA` instead of the catkin-scanned `src/` tree. The upstream repository contains catkin packages such as `local_planner` and `navigation`, which conflict with existing packages in this workspace. Do not symlink or copy this whole tree into catkin `src/`.

Current Scout integration status:

- source available for real-machine `git pull`: yes
- built by the main catkin workspace as a whole tree: no
- official upstream source path in repo: `third_party/LT_DWA/`
- Scout runtime wrapper path: `src/scout_apps/control/lt_dwa_official_wrapper/`
- runtime `local_planner` package path: `tools/lt_dwa/local_planner_runtime/`
- catkin vendor deps copied from upstream: `src/scout_apps/control/lt_dwa_official_vendor_deps/obstacle_msgs/`, `src/scout_apps/control/lt_dwa_official_vendor_deps/local_map_generation/`
- wrapper default: shadow-only; `/cmd_vel` requires explicit `enable_actuated_output:=true` and `publish_cmd_vel:=true`
- main-repo migration check on 2026-06-24: build PASS; wrapper tests PASS (`126 tests, 0 errors, 0 failures, 0 skipped`); launch parse PASS for `scout_sop_shadow_integration.launch` and `scout_sop_cmd_vel_benchmark.launch`

Real-machine transfer rule:

- Do not hand-copy `/data/a/lt_dwa_official_repro_ws/src/LT_DWA` or `/data/a/lt_dwa_wrapper_ws` to the robot.
- Transfer by pushing/pulling the main repository with this vendor snapshot, wrapper package, runtime package, and isolated catkin vendor deps.
- Keep `third_party/LT_DWA` outside `src/` even if the vendor method later changes from ordinary tracked source to a git submodule.

Refresh procedure:

1. Inspect the upstream commit before import and update the commit line above.
2. Keep the vendor tree outside `src/`; do not symlink `third_party/LT_DWA` into catkin scan paths.
3. Strip upstream `.git` metadata if using ordinary vendored source, or keep any future submodule pinned under `third_party/` only.
4. Rebuild only the wrapper/deps whitelist before runtime checks:

   ```bash
   catkin_make \
     -DCATKIN_WHITELIST_PACKAGES="obstacle_msgs;local_map_generation;lt_dwa_official_wrapper" \
     -DCMAKE_BUILD_TYPE=RelWithDebInfo \
     -DLT_DWA_WRAPPER_ENABLE_OFFICIAL_CORE=ON
   ```

5. Before launching official-core mode, prepend the runtime package:

   ```bash
   export SCOUT_WS_ROOT=/home/geist/scout_ws
   export ROS_PACKAGE_PATH=$SCOUT_WS_ROOT/tools/lt_dwa/local_planner_runtime:$ROS_PACKAGE_PATH
   ```
