# LT-DWA adapter plan for Scout comparison benchmark

This document is a non-runtime integration plan. It does **not** make LT-DWA a
runnable benchmark baseline and does **not** change the main catkin workspace.

## Current status

- Upstream source is vendored at `third_party/LT_DWA/`.
- The vendor tree is source-only and stays outside catkin-scanned `src/`.
- `third_party/LT_DWA/.git` is stripped; the code is tracked as ordinary source
  in this repository.
- Readiness gate remains `LT_DWA_ADAPTER_NOT_READY`.
- LT-DWA must not enter main or supplementary result tables until an adapter and
  smoke gate pass.

## Planned adapter boundary

Future adapter package path, if implemented:

```text
src/scout_apps/control/lt_dwa_adapter/
```

The adapter must be a separate Scout-owned package. It must not symlink or copy
LT-DWA's conflicting catkin packages directly into `src/` as-is.

Expected boundary:

```text
benchmark fixed/global path + costmap/odom inputs
  -> lt_dwa_adapter Scout-owned wrapper
  -> isolated LT-DWA planning call or process boundary
  -> observable cmd_vel-compatible output for benchmark runner
  -> shared rosbag / metrics / report
```

Forbidden boundary:

```text
third_party/LT_DWA/* directly symlinked into src/
third_party/LT_DWA/local_planner replacing existing local_planner packages
slosh monitor topics -> LT-DWA control input
adapter bypassing shared benchmark preflight/readiness/freshness gates
```

## Smoke-gate checklist before runnable use

LT-DWA can only move from source-only candidate to runnable comparison after all
items below are true:

1. Adapter package exists outside `third_party/LT_DWA`.
2. Adapter package name does not conflict with existing workspace packages.
3. Adapter consumes the same fixed/global path and environment information class
   allowed for other online local-planner baselines.
4. `/cmd_vel` output is observable through the benchmark runner/gate without
   changing the production `/cmd_vel` chain.
5. Slosh monitor outputs remain evaluation-only and are not adapter inputs.
6. Current-sim smoke passes with metrics extraction.
7. Strict-fresh one-case-per-sim smoke passes with freshness evidence.
8. `bench_check_advanced_baseline_readiness.py --planner lt_dwa` no longer emits
   `LT_DWA_ADAPTER_NOT_READY`.
9. `bench_preflight.py --planner lt_dwa --dry-run` passes with the adapter
   declared ready.
10. Main-table admission still requires common limits, frozen parameters,
    freshness, monitor isolation, and command-gate clamp ratio checks.

## Refresh policy

When refreshing the vendor snapshot:

1. Inspect upstream commit and record it in `SCOUT_VENDOR_NOTES.md`.
2. Keep upstream `.git` metadata stripped from `third_party/LT_DWA/`.
3. Preserve source-only status unless the adapter gate is implemented in a
   separate Scout-owned package.
4. Run the LT-DWA readiness checker and expect source-only gating until the
   adapter is intentionally completed.
