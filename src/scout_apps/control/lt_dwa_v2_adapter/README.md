# LT-DWA-v2 adapter

Scout-owned LT-DWA-v2 experimental local-planner adapter.

This package is intentionally isolated from both:

- ROS DWA / `dwa_local_planner`
- `third_party/LT_DWA`

The intended design is **DWA engineering framework + LT-DWA long-horizon sampling/scoring ideas**:

- DWA contributes the local-planner engineering pattern: path/progress tracking, dynamic-window sampling, rollout, collision/cost checks, critic-style scoring, oscillation handling, diagnostics, and safe command publication boundaries.
- LT-DWA contributes only the algorithmic idea of long-horizon candidate expansion and long-term trajectory scoring.
- Runtime code in this package must remain Scout-owned and independently reviewable.

## Safety defaults

This package is not a real-robot default and is not eligible for formal benchmark main tables until its smoke gates pass.

Default behavior must remain shadow-only:

```yaml
runtime:
  publish_cmd_vel: false

topics:
  cmd_vel_topic: /lt_dwa_v2/shadow_cmd_vel
  shadow_cmd_topic: /baseline/lt_dwa_v2/shadow_cmd_vel
```

Do not change `/cmd_vel` production-chain defaults from this package. Do not relax command guard limits here. Do not modify SPMPC OCP inputs here.

This package must not subscribe to or consume `/slosh/*` or `/benchmark/slosh_monitor/*`. Slosh monitor outputs are evaluation-only and must not influence planner, profile generator, command gate, OCP, or `/cmd_vel` paths.

## Planned module layout

```text
include/lt_dwa_v2_adapter/
  core/      data contracts, grouped config, planner status
  geometry/  arc-length path reference and 2D planning transforms
  world/     occupancy/collision adapter boundary
  rollout/   dynamic-window command sampling and trajectory propagation
  scoring/   split scoring terms and score aggregation
  search/    long-horizon LT-DWA-v2 candidate expansion and pruning
  ros/       parameter loading, diagnostics formatting, ROS wrapper

src/
  geometry/
  world/
  rollout/
  scoring/
  search/
  ros/
```

## Current state

This first micro-patch creates only the isolated package skeleton, grouped sim config, launch wrapper, and minimal node. Planner modules are placeholders and are not yet connected to benchmark suite entries.

Future patches should implement one layer at a time and keep `lt_dwa`, `lt_dwa_v2`, ROS DWA, and `third_party/LT_DWA` separate.

## Simulation map rule

Any map-based visualization, diagnostic run, or smoke run must explicitly use the isolated simulation SOP map:

```bash
MAP_FILE=/data/a/scout_sim_replacement/maps/proxy_world_manual_saved_20260611_154348.pbstream
```
