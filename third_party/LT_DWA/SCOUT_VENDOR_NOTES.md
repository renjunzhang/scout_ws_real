# Scout vendor notes for LT-DWA

Upstream source: <https://github.com/flztiii/LT_DWA>

Vendored commit inspected before import: `6f49cce`.

This directory is intentionally kept under `third_party/LT_DWA` instead of the
catkin-scanned `src/` tree. The upstream repository contains catkin packages such
as `local_planner` and `navigation`, which conflict with existing packages in
this workspace. Keep this source tree as **source-only** until a separate adapter
and smoke gate are implemented.

Current benchmark status:

- source available for real-machine `git pull`: yes
- built by the main catkin workspace: no
- benchmark adapter: pending
- strict-fresh runnable baseline: no
