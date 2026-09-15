# Trajectory MPCC profiles

`raw_mpcc` is the original path/ordinary MPCC identity. It has no new geometry objective or trajectory plan, and has no liquid cost, liquid constraint, recovery, risk governor, complete-stop liquid handoff, or NoState ablation. It must consume the same explicit physical region YAML as the comparison method so spatial feasibility remains matched; region use does not enable the new geometry objective.

`geometry_mpcc`, `planned_mpcc`, and `planned_slosh` are templates. Before use, provide an explicit region YAML through `region_config` and a plan file for the planned profiles. An empty region or plan is rejected by the node; no boundary is inferred from the costmap or navigation path. All profiles explicitly use the same ordinary non-liquid weights (`w_lag=0.2`, `w_progress=0.2`, `w_v=1.0`, `w_vs=0.3`, `v_ref=0.25`, `w_control=0.1`, `w_accel=0`, `w_smooth=0.1`) and `w_alpha=0.1`, `w_du_a=0.1`, `w_du_vs=0.1`. Geometry/planned use `w_contour=0.02`, `goal_weight=2.0`, `curvature_rate_weight=0.01`, and disable reference curvature speed limiting; raw keeps ordinary `w_contour=1.0`, the geometry objective disabled, and `goal_weight=0`. The goal term uses the configured position and yaw scales (defaults `0.3` m and `1.0` rad). `planned_slosh` selects `B_slosh`; its matched liquid-off comparison is `planned_mpcc` with the same progress reference, geometry, region, and non-liquid weights.

Example planner launch (the normal ROS1 sensor/execution stack must already be running):

```bash
roslaunch spmpc_local_planner trajectory_mpcc.launch profile:=planned_mpcc region_config:=/path/region.yaml plan_file:=/path/plan.json task_overlay_file:=/path/task_deadline.yaml
roslaunch spmpc_local_planner trajectory_mpcc.launch profile:=planned_slosh region_config:=/path/region.yaml plan_file:=/path/plan.json planner_variant:=B_slosh task_overlay_file:=/path/task_deadline.yaml
```

The final merged configuration must contain `planning.task_deadline_sec` greater than zero. This can come directly from `TASK_CONFIG` or from either overlay; ordinary weights overlays may omit it. The recorder only records an already running node; it does not launch ROS or move the robot. Its manifest marks `recorded_launch_args` as declared and verifies the live private namespace with `rosparam get` before opening rosbag. The supplied region file must enable a non-empty region for every profile, including raw.

Validate recording inputs without starting ROS or motion:

```bash
VALIDATE_ONLY=true PROFILE=planned_mpcc REGION_CONFIG=/path/region.yaml PLAN_FILE=/path/plan.json TASK_OVERLAY_FILE=/path/task_overlay.yaml ./scripts/record_trajectory_mpcc_comparison.sh
```

The latched `/spmpc/debug/planning_config` JSON is the authoritative profile record. Record it together with `/spmpc/debug/effective_config`, `/spmpc/debug/pre_solve_snapshot`, `/spmpc/debug/predicted_horizon`, and `/spmpc/debug/control_cycle_audit`.

All four profiles inherit the shared terminal goal/stop tolerances, stop window, jerk limit, evaluation window, and `acados/rti_iterations=5` from `trajectory_common.yaml`; these values are deliberately absent from individual profiles. The common deadline is `0`; recording validates that the final merged overlays provide a positive deadline. Five RTI iterations repeat SQP RTI work and must be checked against the 30 Hz cycle budget on the target machine.

The recorder freezes a JSON manifest with the complete merged configuration, ordered artifact copies and SHA256 hashes, Git SHA/status/diff, and declared launch arguments. `recording_contract.py` contains the merge and live-parameter checks; its self-contained tests cover overlay order, a deadline supplied by only one of two overlays, and live profile mismatch rejection.

The supplied `corner_task.example.yaml` and `corner_region.example.yaml` match the
software corner scenario in `test/native/scenarios/corner.json`. They are sample
task geometry, not a measured hardware map. The shared `actual_v_min=-0.002` permits
bounded negative measured drift while every command remains nonnegative.

The latched planning JSON contains planning fields only (Boost property_tree
serializes scalars as strings); the recording manifest plus verified live private
parameters supplies the complete launch configuration. Native trial reports
freeze their own explicit configuration and do not validate a ROS launch.

The default geometry weight is a software candidate. A larger value of 1.0 made
the corner controller resist final yaw correction and fail the deadline; preserve
that tradeoff when tuning. The same plan in `planned_mpcc` and `planned_slosh`
is the matched lower-controller liquid ablation. To isolate upper liquid planning,
generate a second task with `objective.liquid=0` and `objective.liquid_terminal=0`
and run both lower modes on both plans; the four supplied profiles alone are not
this factorial ablation.
