# lt_dwa_adapter

Scout-owned LT-DWA-style local-planner adapter for benchmark-only comparison runs.

This package is intentionally separate from `third_party/LT_DWA`. The upstream LT-DWA repository remains a source-only reference because its ROS packages use generic names such as `local_planner` and `navigation`, assume demo services/files, and contain runtime side effects that are not safe to import directly into this workspace. This adapter reimplements the relevant planner idea in Scout-owned code:

- dynamic-window sampling around the current `v, omega` state;
- acceleration-limited commands using the common benchmark limits;
- layered rollout of unicycle trajectories;
- deterministic top-K pruning at every rollout layer;
- occupancy-grid collision and clearance scoring;
- path/heading/incremental-progress/terminal/smoothness/speed scoring;
- a deterministic path-tracking seed added to the LT-DWA lattice for short fixed-path stability;
- first-command output from the best rollout.

## Safety defaults

The adapter is shadow-only by default:

```yaml
publish_cmd_vel: false
cmd_vel_topic: /lt_dwa/shadow_cmd_vel
shadow_cmd_topic: /baseline/lt_dwa/shadow_cmd_vel
```

With `publish_cmd_vel=false`, the node does not publish the configured `cmd_vel_topic`; it only publishes the shadow command topic and benchmark status/plan topics. Closed-loop simulation suites must explicitly pass both `publish_cmd_vel:=true` and a deliberate command topic such as `/cmd_vel`.

This package does not subscribe to `/slosh/*` or `/benchmark/slosh_monitor/*`. Liquid/slosh monitor outputs are evaluation-only and are not planner inputs.

## Topics

Inputs:

- `/odom` (`nav_msgs/Odometry`)
- `/map` (`nav_msgs/OccupancyGrid`)
- `/scout/global_path_fixed` (`nav_msgs/Path`)
- `/scout/goal` (`geometry_msgs/PoseStamped`, optional; the path endpoint is also used)
- TF from `plan_target_frame` to `base_frame` (simulation defaults use `odom` for the fixed-path controller frame to avoid localization-map jumps during the smoke gate)

Outputs:

- `/baseline/lt_dwa/shadow_cmd_vel` (`geometry_msgs/Twist`)
- `/baseline/lt_dwa/status` (`std_msgs/String`, latched)
- `/baseline/lt_dwa/global_plan` (`nav_msgs/Path`, latched)
- `/baseline/lt_dwa/local_plan` (`nav_msgs/Path`)
- configured `cmd_vel_topic` only when `publish_cmd_vel=true`

## Fair-comparison limits

The default simulation config aligns with the benchmark common limits:

```yaml
v_max_mps: 0.8
omega_max_radps: 1.2
a_max_mps2: 0.6
alpha_max_radps2: 1.2
allow_reverse: false
xy_goal_tolerance: 0.20
yaw_goal_tolerance: 0.30
```

## Launch

Shadow-only launch:

```bash
roslaunch lt_dwa_adapter lt_dwa_adapter.launch publish_cmd_vel:=false
```

Closed-loop simulation launch must be done only inside the isolated simulation workflow and should pass an explicit command topic:

```bash
roslaunch lt_dwa_adapter lt_dwa_adapter.launch publish_cmd_vel:=true cmd_vel_topic:=/cmd_vel
```

Do not use this launch as a real-robot default. Real-robot testing requires a separate, explicit safety review and gate.
