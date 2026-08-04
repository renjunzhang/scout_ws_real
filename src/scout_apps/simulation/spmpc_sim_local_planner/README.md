# Source-separated S-MPCC simulation target

`spmpc_sim_local_planner` owns the simulation controller library, node,
messages, configuration, fixed-profile runtime, campaign tools, ACADOS solver
assets and H0 attach/environment launchers.  Its node is
`/sim_spmpc_local_planner`; diagnostics are `/sim_spmpc/*`.

The package has no build, link, launch, or runtime dependency on the real
robot controller package.  Its fork provenance and deliberate ABI changes are
in `SIM_FORK_ORIGIN.md`; `tests/test_source_isolation.py` verifies the boundary
including the rebuilt binary RUNPATH.

The simulation-owned H_proxy monitor has only executed `/odom` as an input.
Its `/slosh/height` output is recorded strictly as `H_proxy`; it is not an
independent liquid plant or physical-primary signal.

Historical R7 data is retained as shared-target evidence and must not be
renamed as source-separated.  A new R8 freeze, GO receipt, timing admission,
smoke and matrix are required before the isolated target can support a new
campaign.

## R8 source freeze / GO boundary

`scripts/smpcc_sim_source_separation.py` derives an R8 registry of this
package's source, headers, messages, model/codegen inputs, copied ACADOS
artifacts, configuration, launch/runtime tooling, node binary and library.
The R8 freeze and master must cross-bind that registry; a separate immutable
GO receipt must bind it again plus the sim-only build, source-isolation,
environment-isolation, H_proxy-monitor, controller-gate and R8 release tests.
Use `scripts/smpcc_sim_r8_release.py gate --freeze ... --master ...` only to
inspect that source boundary.  Its PASS is explicitly **not** matrix GO: it
does not replace timing, independent liquid-plant/fidelity/firewall, frozen
matrix, or live-smoke admission evidence.

## Fresh environment boundary

`scripts/launch_sim_environment.sh` is the only environment-only entry.  It
requires distinct, unused loopback ROS/Gazebo ports and starts the package's
own `smpcc_sim_environment.launch`, which owns only Gazebo, the proxy robot,
the actuator clamp, and read-only Cartographer localization.  It does not
include the legacy proxy navigation launch.  Before launch it replaces the
broad workspace ROS package path with an allowlist; therefore
`spmpc_local_planner`, `spmpc_experiments`, and
`scout_mini_proxy_nav_adapter` are not discoverable to that shell.  At the
environment handoff it verifies that neither a publisher on the configured
controller command topic nor one on the configured reference-path topic has
appeared.  A simulation-owned controller/path launch is a separate, explicit
later operation.

## H0 development path publisher

`smpcc_sim_h0_fixed_path_publisher.launch` is an H0-only runtime source for
development smoke work.  It takes one `/odom` sample in the isolated simulated
ROS master, generates the old open-field H0 path to `(5.0, 0.0, 0.0)` by
default, and latches it on `/scout/global_path_fixed`.  It never starts a
planner or publishes `/cmd_vel`; formal H1/L1 rows must instead use frozen JSON
path replay.  Launching it requires `h0_development_ack:=true`.
