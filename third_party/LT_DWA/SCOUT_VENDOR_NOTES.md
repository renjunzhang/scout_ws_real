# Scout vendor notes for LT-DWA

Upstream source: <https://github.com/flztiii/LT_DWA>

Vendored commit inspected before import: `6f49cce`.

Import date: 2026-06-19.

Vendor method: upstream `.git` metadata stripped; source tracked as ordinary repo files.

Local modifications policy: keep upstream source unchanged where possible; put Scout-specific notes and adapter plans in `SCOUT_*.md` files.

This directory is intentionally kept under `third_party/LT_DWA` instead of the
catkin-scanned `src/` tree. The upstream repository contains catkin packages such
as `local_planner` and `navigation`, which conflict with existing packages in
this workspace. Keep this source tree as **source-only** until a separate adapter
and smoke gate are implemented.

Current benchmark status:

- source available for real-machine `git pull`: yes
- built by the main catkin workspace: no
- benchmark adapter: pending
- planned adapter location: `src/scout_apps/control/lt_dwa_adapter/` (not created yet)
- adapter plan / smoke-gate checklist: `SCOUT_ADAPTER_PLAN.md`
- strict-fresh runnable baseline: no

Refresh procedure:

1. Inspect the upstream commit before import and update the commit line above.
2. Keep the vendor tree outside `src/`; do not symlink `third_party/LT_DWA` into catkin scan paths.
3. Strip upstream `.git` metadata before committing the snapshot.
4. Re-run `bench_check_advanced_baseline_readiness.py --planner lt_dwa`; it should remain gated until the separate adapter package and smoke gate are implemented.
