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
`(cd /tmp/spmpc_native_build_real && ctest --output-on-failure)` also resolves
the libraries and works with ROS Noetic's CTest 3.16.
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

The simulated plant starts from the plan's declared `task.start_state`, checked
against the first optimized sample (28 finite entries, maximum difference
2e-6). Numerical optimizer residuals in the first sample are not previously
published actuator commands; a nonzero declared initial history is preserved.

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
The original [matrix](results/20260916_geometry_trials.csv) and the post-review
[matrix](results/20260916_review_fixes_geometry_trials.csv) both complete 22/24,
with both geometry-only corner trials failing (`ACADOS_SOLVE_FAILED_4`). The new
[identity record](results/20260916_review_fixes_geometry_identity.json) keeps the
run Git/diff, solver/binary hashes and plan validation identities. Maximum cycle
time in the new run is 82.2 ms; timings across runs are not a performance comparison.
Lower curve energy has not established lower liquid peaks than raw.

Post-review checks passed 17 stub CTest programs and all 18 real-acados programs
(the replay diagnostics program was rerun after correcting two old test assumptions).
The focused Python run passed 49 checks; two roslaunch-dependent checks could not
run on this host. See the [repair record](../../docs/实物实验注意事项/后续改进/20260916_局部规划器五项修复与回归.md)
for coverage and remaining ROS1/physical validation boundaries.

## 2026-09-16 执行链收敛回归

最新生产核心 `c7cd62a` 的 stub 18/18、真实 acados 19/19 个 CTest 程序通过；
预测高度旁路及相关 Python 定向检查35项通过。四次单拐角回归的
[身份和全部结果](results/20260916_execution_contract_regression.json)另存，未覆盖上述历史矩阵。
B0/Full完成；两个geometry组仍在10 s发生 `ACADOS_SOLVE_FAILED_4`，本轮改为
可信停车接管，最终 `TASK_DEADLINE_MISSED`。不能只看最终状态而漏记早先求解失败。

native 默认不设置在线求解预算，固定次数用于名义模型回归；本轮最长核心耗时69.83 ms，
不能据此宣称30 Hz通过。ROS路径才设置周期预算和发布时效门。
修复边界、参数及新验证顺序见[执行链报告](../../docs/实物实验注意事项/后续改进/20260916_执行链收敛修复与定向回归.md)。
