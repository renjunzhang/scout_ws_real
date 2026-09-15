# Native SPMPC tests

This harness builds the ROS-independent core and tests with CMake/Ninja. The
default is the stub build, which excludes acados backends and uses
`/tmp/spmpc_native_build_stub`. Run it with:

```bash
scripts/run_native_tests.sh
```

The real solver build is explicit and isolated in
`/tmp/spmpc_native_build_real`:

```bash
export ACADOS_SOURCE_DIR=/home/zrj/.cache/scout_spmpc_dev/acados_full
export TERA_PATH=$ACADOS_SOURCE_DIR/bin/t_renderer
export LD_LIBRARY_PATH=$ACADOS_SOURCE_DIR/lib:$LD_LIBRARY_PATH
scripts/run_native_tests.sh --with-acados
```

`--with-acados` validates that `ACADOS_SOURCE_DIR/lib/libacados.so` exists and
that both generated solver shared libraries are present. The CTest target has
an RPATH and test environment for the acados library directory, so direct
`ctest --test-dir /tmp/spmpc_native_build_real` also resolves the libraries.
No mode installs packages or accesses the network.

The Python generator must run with CasADi 3.7.2 and the acados v0.5.4 source
tree. Generate B0 and slosh sequentially; each invocation also synchronizes
the shared `src/core/generated/ocp_cost_*` files:

```bash
source /home/zrj/.cache/scout_spmpc_dev/venv/bin/activate
export ACADOS_SOURCE_DIR=/home/zrj/.cache/scout_spmpc_dev/acados_full
export TERA_PATH=$ACADOS_SOURCE_DIR/bin/t_renderer
python src/scout_apps/control/spmpc_local_planner/scripts/acados/generate_spmpc_acados.py --model b0
python src/scout_apps/control/spmpc_local_planner/scripts/acados/generate_spmpc_acados.py --model slosh
```

The ROS package itself is a ROS1 catkin package (`find_package(catkin)`,
`roscpp`, and `catkin_package` in its CMake file). This host provides ROS2
Jazzy (`ros2`) and has no `roscore` or catkin, so this harness does not claim
to build the ROS1 node. ROS logging used by the native slosh adapter is a
logging-only test shim; it does not emulate ROS time, messages, or transport.

`geometry_trial` is a deterministic model-in-the-loop executable with four
modes: `raw`, `geometry`, `planned`, and `planned_slosh`. It loads the complete
task plan, including region, deadline, motion limits, actuator and liquid
parameters; it does not replace those values with a built-in room or deadline.
The JSON report contains the effective stage-0 parameter manifest, completion
time and lateness, final goal position/yaw errors, actual velocity bounds,
wall-time and solver-time p95/max, and the final evaluation sample time.
Curvature energy and path length are integrated only before the evaluation end
time. Modal height is sampled at controller nodes and is not a dense trace.
On any failed solve or bound violation the trial writes a failed horizon CSV
and stops at that row, preserving the failure evidence.

Run the frozen three-task/four-mode/actuator-mismatch matrix from the repository root:

```bash
/home/zrj/.cache/scout_spmpc_dev/venv/bin/python scripts/run_geometry_trials.py \
  --native-dir /tmp/spmpc_native_build_real --output-dir /tmp/new_geometry_trials
```

Use a new output directory. `--plans-dir` reuses and revalidates previously frozen
plans; `--suffix-reoptimize` additionally tries full-state remaining-task optimization.
All groups share each input task and evaluation window. Native defaults freeze
curvature weight 0.05 (raw: 0), change weight 0.01 (raw: 0), contour 0.02 (raw: 1),
goal weight 2 (raw: 0), and five RTI iterations. The native manifest is explicit; it
is not loaded from ROS YAML. The supplied tasks match the default calibrated cup;
other liquid coefficients are rejected until the harness is given their matching
physical cup configuration.

The planned modes use the same upper plan. Only the lower liquid objective differs;
this is not a full upper/lower factorial ablation. Liquid feedback uses ideal plant
state, and modal peaks are sampled at 30 Hz. Results are model-in-the-loop evidence.
See `results/20260916_geometry_trials.csv`: 22/24 complete, with both geometry-only
corner trials failing. Lower curve energy has not established lower liquid peaks
than raw, and observed maximum cycle times exceed the 30 Hz budget.
