# Benchmark policy configs

This directory contains paper-facing benchmark policy/configuration files for the SPMPC comparison workflow.

These files are intentionally declarative. They do **not** change planner runtime behavior, launch defaults, `/cmd_vel` publication, SPMPC OCP inputs, Gazebo worlds, maps, TF, URDF, or real-robot launch behavior.

## Phase 0 files

- `capability_matrix.yaml` — method category/capability declarations for fair reporting.
- `common_limits.yaml` — common velocity/acceleration targets and planner-specific mapping notes.
- `common_environment.yaml` — shared sim/path/timing/freshness assumptions.
- `information_access_policy.yaml` — allowed and forbidden information channels per method category.
- `tuning_protocol.yaml` — train/validation/test split, tuning budget, and parameter-freeze requirements.

- `external_slosh_observer_policy.yaml` — monitor-only external slosh observer boundary.
- `slosh_model_monitor_policy.yaml` — slosh monitor namespace, allowed consumers, and leakage redlines.
- `failure_taxonomy.yaml` — failure/status codes for dependency, planner, task, fairness, monitor, and analysis outcomes.
- `main_table_inclusion.yaml` — strict main-table entry rules and appendix-only categories.
- `profile_tracking_common.yaml` — shared profile tracker and CSV schema for Hamaguchi/Lim-style baselines.
- `canonical_fixed_path_p2.yaml` — canonical endpoint/template/common-limit declaration used to align profile-baseline runs with earlier fixed-path simulation records.

The high-level monitor-only information boundary is also recorded in `information_access_policy.yaml`. Phase 0 dry-run checkers validate these files without launching runtime systems.

## Safety red lines

- Slosh monitor outputs are monitor-only and must not feed planners, profile generators during a test case, command gates, SPMPC OCP parameters, or `/cmd_vel` production.
- Formal fixed-path comparisons require strict fresh-sim evidence. A currently running simulator can only produce current-sim diagnostics.
- Simulation work defaults to `/data/a/scout_sim_replacement` per the isolation SOP.
- No broad `killall` / `pkill` cleanup is allowed in benchmark scripts; scripts may only stop child PIDs they own.

## Profile baseline policy

Hamaguchi/Lim-style baselines are offline profile generators. Their configs live in `../profile_baselines/`, and their generator implementations live in the isolated `src/scout_apps/control/scout_profile_baselines/` package. `scout_local_planner` remains the common external-profile tracker and keeps only legacy wrapper entrypoints.

- They may read only a fixed path, frozen common limits, and offline liquid-model parameters.
- They must output the shared profile CSV schema declared in `profile_tracking_common.yaml`.
- They must use the shared profile tracker, command policy, and metrics chain.
- They are supplementary slosh-profile baselines, not online local-planner main-table entries.
- The Hamaguchi first version is a fixed-path input-shaped approximation; it does not redesign path geometry from the paper's curved-path construction.

## Canonical profile-suite endpoint and freshness evidence

`canonical_fixed_path_p2.yaml` records the first profile-baseline integration endpoint used in earlier fixed-path SPMPC records: `goal=(5.0, 0.0, 0.0)`, `s_curve`, `start_heading=current`, amplitude ratio `0.18`, max amplitude `1.20`, smoothing `3`, and common limits `0.8/1.2/0.6/1.2`. `bench_check_profile_endpoint.py` checks new Hamaguchi/Lim suite runs against this declaration before strict fresh results are accepted.

Older `/data/a/scout_sim_replacement` strict fresh manifests use reachability columns such as `pre_ros_reachable` and `post_gazebo_reachable`. Convert those manifests with `bench_write_freshness_evidence.py` before passing them to `bench_check_freshness.py`; the checker itself only accepts the explicit strict evidence fields and must not infer strict freshness from an already-running simulator.

## Advanced baseline readiness policy

`lt_dwa` and `mpc_planner` are readiness-gated advanced candidates. LT-DWA source is vendored at `third_party/LT_DWA/` for real-machine `git pull`, but it is source-only because upstream package names (`local_planner`, `navigation`) conflict with this workspace; readiness remains `LT_DWA_ADAPTER_NOT_READY`. If `src/mpc_planner` lacks generated solver artifacts or required dependencies, it is reported as `MPC_PLANNER_NOT_READY`. These dependency/readiness outcomes are not algorithm failures and must not be promoted into runnable suites without a separate smoke gate.

## Dependency policy

Phase 0/profile-readiness checks should only require Python standard library plus PyYAML for validation scripts. Do not add `jsonschema`, `pydantic`, `numpy`, `scipy`, TOPPRA, CasADi, IPOPT, or other new dependencies unless explicitly approved.
