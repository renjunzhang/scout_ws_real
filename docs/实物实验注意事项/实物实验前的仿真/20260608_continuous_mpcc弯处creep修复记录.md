# 20260608 continuous MPCC 弯处 creep 修复记录

## 1. 背景

本记录对应 omega 进入 acados OCP 状态、`alpha=omega_dot` 作为控制并硬约束 `|alpha|<=1.2 rad/s^2` 后，B0 continuous/acados smoke 中出现的弯处停滞问题。

诊断对象：

```text
/data/a/spmpc_omega_smoke/B0.bag
```

关键现象：

```text
status: B0_ACADOS_OK 为主，只有少量 ACADOS_SOLVE_FAILED_4
solver_time_ms: 约 2–9 ms，满足 30 Hz 实时性
progress: 约 5.5% -> 18.6% 后停住
cmd_v: 前 2s 约 0.21 m/s，后段降到约 0.002 m/s
cmd_omega: 曾到约 1.065 rad/s，接近急弯所需 v*kappa
```

因此问题不是“求解器卡死”，而是 OCP 在弯处主动减速后掉进 creep 坑。

## 2. 机制判断

当前新 OCP 中：

```text
omega_dot = alpha
|alpha| <= 1.2 rad/s^2
|omega| <= 1.2 rad/s
```

急弯处要跟线需要较大 `omega≈v*kappa`。旧模型里 `omega` 是控制量，可以在相邻周期内快速跳变；新模型中 `omega` 是状态，只能通过 `alpha` 逐步拉升。

当进入高曲率起步段时：

```text
需要较大 omega
-> alpha hard cap 使 omega 无法瞬时到位
-> contour / omega / alpha / control cost 上升
-> 优化器降低 v 和 v_s 来降低所需 omega
-> 当前 progress 只是 -w_progress*v_s 的单边线性奖励
-> 停住只少拿奖励，不会被显式二次惩罚
-> v_s/cmd_v 越降越小，最终 creep
```

这说明 `alpha` cap 不是“坏掉”，它暴露了原 progress objective 的停滞坑：cap 修的是 chattering，不自动修停滞。

## 3. 本次修复

本次只修改 planner / algorithm / config，不修改仿真环境。

核心改动：将 path-speed objective 模块化为：

```text
j_path_speed = -w_progress * (v_s / vs_max)
             +  w_vs * ((v_s - v_ref) / vs_max)^2
```

其中：

```text
w_progress = 0.2
w_vs       = 0.3
v_ref      = 0.25 m/s
```

解释：

- `w_progress` 保留原来的向前推进奖励。
- `w_vs` 专门表示虚拟路径进度速度 tracking penalty。
- `v_ref` 是可配置的参考进度速度，不写死在 solver 内。
- `v_s≈0` 现在会显式产生 tracking penalty，不再只是“少拿一点进度奖励”。
- 第一版所有变体统一使用同一组 `w_vs/v_ref`，避免把 objective 修复和变体调参混在一起。

## 4. 涉及文件

主要代码文件：

```text
src/scout_apps/control/spmpc_local_planner/scripts/acados/spmpc_acados_model.py
src/scout_apps/control/spmpc_local_planner/scripts/acados/spmpc_acados_cost.py
src/scout_apps/control/spmpc_local_planner/scripts/acados/generate_spmpc_acados.py
src/scout_apps/control/spmpc_local_planner/include/spmpc_local_planner/core/variant_config.h
src/scout_apps/control/spmpc_local_planner/src/ros/spmpc_local_planner_ros.cpp
src/scout_apps/control/spmpc_local_planner/src/solvers/continuous_mpcc_solver_acados.cpp
src/scout_apps/control/spmpc_local_planner/config/planner/variants.yaml
```

参数契约变化：

```text
B0    NP: 21 -> 22
slosh NP: 29 -> 30
新增参数: v_ref
```

诊断口径：

```text
/mpc/cost_breakdown layout 不变。
J_progress: 记录归一化 stage 进度奖励。
J_v: 记录新增 v_s tracking penalty。
```

## 5. 已完成验证

CasADi 装配检查：

```bash
cd /home/a/scout_ws/src/scout_apps/control/spmpc_local_planner/scripts/acados
python3 generate_spmpc_acados.py --check --model b0
python3 generate_spmpc_acados.py --check --model slosh
```

结果：

```text
b0:    nx=6  nu=3  np=22
slosh: nx=10 nu=3  np=30
stage_cost shape = (1, 1)
terminal_cost shape = (1, 1)
path-speed: w_progress=0.2000 w_vs=0.3000 v_ref=0.2500
```

acados codegen：

```bash
source /home/a/acados_venv/bin/activate
export ACADOS_SOURCE_DIR=/home/a/acados
export LD_LIBRARY_PATH=/home/a/acados/lib:${LD_LIBRARY_PATH:-}
python3 generate_spmpc_acados.py --model b0
python3 generate_spmpc_acados.py --model slosh
```

生成结果：

```text
SPMPC_B0_NP    = 22
SPMPC_SLOSH_NP = 30
```

构建验证：

```bash
source /home/a/acados_venv/bin/activate
export ACADOS_SOURCE_DIR=/home/a/acados
export LD_LIBRARY_PATH=/home/a/acados/lib:${LD_LIBRARY_PATH:-}
cd /home/a/scout_ws
catkin_make --force-cmake --pkg spmpc_local_planner
```

结果：通过。

测试：

```bash
catkin_make run_tests_spmpc_local_planner
```

结果：

```text
TerminalController: 6/6 passed
DiffDriveFlatnessWarmStart: 3/3 passed
总计 9/9 passed
```

备注：按旧项目红线命令尝试 `catkin_make --force-cmake --pkg scout_local_planner` 时，当前工作区实际白名单包为 `spmpc_local_planner`，因此旧目标 `scout_local_planner/all` 不存在；已用当前包名完成构建验证。

## 6. 追加修复：加入物理速度 tracking

第一版只加入 `v_s` tracking 后，复查：

```text
/data/a/spmpc_omega_smoke_vref/B0.bag
```

现象变为：

```text
progress: 旧 18.6% -> 新 42.5%
cmd_v: 末段仍约为 0
J_v: 末段约 0.0293，正好对应 w_vs*((0-0.25)/0.8)^2
```

这说明 `v_s` tracking 生效了，但它只约束虚拟路径进度速度。continuous MPCC 中 `v` 与 `v_s` 是两个变量：

```text
v_dot = a      # 物理速度状态
s_dot = v_s    # 虚拟路径进度速度
```

如果只惩罚 `v_s`，OCP 仍可让物理速度 `v/cmd_v` 塌到 0，并支付较小的 `J_v` 代价。因此继续加入物理速度项：

```text
j_path_speed = -w_progress * (v_s / vs_max)
             +  w_v  * ((v   - v_ref) / v_max)^2
             +  w_vs * ((v_s - v_ref) / vs_max)^2
```

当前统一初值：

```text
w_v  = 1.0
w_vs = 0.3
v_ref = 0.25 m/s
```

参数维度更新：

```text
B0    NP: 22 -> 23
slosh NP: 30 -> 31
```

验证结果：

```text
generate_spmpc_acados.py --check --model b0    OK, np=23
generate_spmpc_acados.py --check --model slosh OK, np=31
重新生成 B0/slosh acados solver OK
catkin_make --force-cmake --pkg spmpc_local_planner OK
catkin_make run_tests_spmpc_local_planner OK, 9/9 passed
```

诊断口径：

```text
/mpc/cost_breakdown layout 不变。
J_v 现在表示物理速度 v 和虚拟进度速度 v_s 的 tracking penalty 合计。
```

## 7. 尚未完成的验证

尚未运行 formal fresh-sim。后续如果跑 B0/B_slosh/B_ours 验证，仍需严格遵守：

```text
1. 不修改仿真环境，只调整 planner / algorithm / config。
2. 每个 case 单独 fresh sim。
3. 启动仿真后等待 30s，让定位恢复。
4. 每次只跑一个 planner/case。
5. 关闭仿真后等待 30s，再开始下一次。
6. 超过约 1 分钟仍未完成按失败处理。
```

建议后续首先只跑 B0 smoke，检查：

```text
progress 是否明显超过本轮 42.5% 停滞点
cmd_v 是否不再长时间塌到约 0.002 m/s 或 0
solver_time_ms 是否仍明显小于 33 ms
status 是否仍以 B0_ACADOS_OK 为主
J_v 是否在弯处反映 v 与 v_s 偏离 v_ref 的合计惩罚
```

## 8. 后续调参建议

当前值只是保守起点：

```text
w_vs  = 0.3
v_ref = 0.25 m/s
```

如果仍然 creep，可以小扫：

```text
w_vs: 0.2 / 0.3 / 0.5
v_ref: 0.20 / 0.25 / 0.30
```

如果出现过弯太激进或 slosh 增大，优先降低 `v_ref`，不要第一时间提高 `alpha_max`。`alpha_max=1.2` 是这次 omega-state OCP 用来约束角加速度、消除 chattering 的核心改动。

## 9. current-start P2_s_curve 与 alpha 消融入口

current-start 路径生成方式已确认：使用 `template_fixed_path_generator.py --template s_curve --start-heading current`，路径起点来自当前 `base_link` TF，终点固定为 P2 terminal goal。该方式下 B0 fresh-sim 仍失败：

```text
bag: /data/a/spmpc_omega_alpha_b0_p2_current_start/20260608_151408_spmpc_B0_P2_s_curve_current_start_run1/20260608_151408_spmpc_B0_P2_s_curve_current_start_run1.bag
path length: 约 8.57 m
progress: 约 0.0005 -> 0.352 m
status: 主要 B0_ACADOS_OK，少量 ACADOS_SOLVE_FAILED_4
terminal mode: TRACKING
solver_time_ms: mean 约 3.76 ms，max 约 7.75 ms
cmd_v: 末段约 0
```

该结果说明：即使消除“固定 JSON 起点与实际车位姿不完全一致”的影响，B0 在 `alpha_max=1.2` 下仍会在前段曲率处进入低速/stall 解。为验证该现象是否由角加速度硬约束触发，已增加运行时诊断入口：

```text
SPMPC_ALPHA_MAX=-1.0  # 默认，不覆盖 platform yaml，保持 alpha_max=1.2
SPMPC_ALPHA_MAX=6.0   # 诊断用，放宽 alpha_max
```

该消融只用于判断机制，不作为 formal/common-limit 结果；若放宽后走通，说明 stall 与 omega-state OCP 的 `alpha` 动态约束强相关，后续仍应回到 `alpha_max=1.2` 修 objective / warm-start / reference-speed 结构。

## 10. 固化 current-start fixed-terminal 流程与 reference 可行性检查

为避免每轮手动先生成 JSON 再调用 suite，`run_fixed_path_spmpc_suite.sh` 增加：

```text
PATH_SOURCE_MODE=replay       # 默认：回放已有 PATH_FILE
PATH_SOURCE_MODE=stable_goal  # 当前 base_link 位姿作起点，固定 goal 作终点，生成 JSON 后再 replay
```

`stable_goal` 模式每个 case 的顺序为：

```text
1. template_fixed_path_generator.py 根据当前 TF + 固定 GOAL_X/Y/YAW 生成 fixed path JSON
2. 保存到 run 目录下 *_generated_path.json
3. 停掉 generator
4. fixed_global_path_runner.py --mode replay 回放该 JSON
5. 启动 SPMPC 并录包
```

这样实际闭环仍使用固定 JSON reference，不是边跑边重新生成路径；meta 中 `path_file` 指向实际 replay 的 JSON。

同时新增 `analyze_fixed_path_feasibility.py`，用于在跑 B0 前/后检查 reference：

```text
max |kappa|
min turning radius
omega_req = v_ref * kappa
a_lat_req = v_ref^2 * kappa
first 0.5/1.0/2.0m prefix window
curvature band length fraction
```

该检查的目的不是修改仿真环境，而是量化 current-start template 是否一开始就生成了对 `omega_max/alpha` 动态过激的 reference。

同时 `run_fixed_path_spmpc_suite.sh` 已加入 `RUN_TIMEOUT_SEC` hard stop，默认 `60s`。计时从 planner launch 后开始；如果等待 ready 与录包尚未结束但达到上限，脚本会打印 `[timeout]` 并清理 rosbag、planner、path publisher、slosh monitor/generator。`RUN_TIMEOUT_SEC=0` 可关闭该上限。该限制用于落实“超过约 1 分钟按失败处理”，不改变仿真环境。20260608_161621 这次 bag 实际记录到 `/spmpc/status` 与 `/cmd_vel`，但脚本 readiness probe 报了假阴性；已将 probe 单次等待从 `1s` 放宽到 `3s`，避免误报。

验证记录：

```text
python3 -m py_compile analyze_fixed_path_feasibility.py path_profile_utils.py: OK
bash -n run_fixed_path_spmpc_suite.sh: OK
rosrun scout_local_planner analyze_fixed_path_feasibility.py --help: OK
catkin_make --force-cmake -DCATKIN_WHITELIST_PACKAGES="scout_local_planner" --pkg scout_local_planner: OK
catkin_make --force-cmake -DCATKIN_WHITELIST_PACKAGES="spmpc_local_planner" --pkg spmpc_local_planner: OK
```

对 current-start alpha6 生成路径的检查显示：`length≈8.46m`，`max|kappa|≈4.82 1/m`，`v_ref=0.25` 时 `max|omega_req|≈1.204 rad/s`，已经略高于 `omega_max=1.2`；前 `2m` 内即达到该峰值。因此 current-start template 的前段曲率确实需要纳入后续 reference 约束/平滑修复。

## 11. primitive direct-omega 对照与 continuous legacy 后端

后续一次 fresh-sim 诊断显示，`SPMPC_SOLVER_BACKEND=primitive` 在同一 current-start fixed-terminal P2 `s_curve` 下可以跑通。这说明仿真环境、stable-goal 生成/replay、60s hard stop 不是主要原因；direct-omega 控制链路本身能处理该 reference。与此同时，`continuous_mpcc_acados` 在 `alpha_max=1.2` 和一次性 `alpha=80` 诊断下仍只推进很短距离后 stall，说明“瞬时打舵/alpha 限制”不是唯一解释，还涉及 continuous OCP 的状态/控制布局、objective、projection 或初始化。

为避免全量回退或污染当前 omega-state/alpha-control 主线，新增 B0-only 诊断后端：

```text
SPMPC_SOLVER_BACKEND=continuous_mpcc_direct_omega_legacy
```

该后端独立生成 `spmpc_b0_direct_omega_legacy` acados solver，模型口径为：

```text
x = [px, py, theta, v, s]
u = [a, omega, v_s]
theta_dot = omega
cmd_omega = u0[1]
```

它保留当前 `w_v/w_vs/v_ref` 防 creep 速度项，但恢复旧式 `omega` 直接控制与 `du_omega/omega_prev` stage-0 平滑项。该后端只用于 B0 诊断，不作为 slosh formal/common-limit 论文结果；slosh variant 误用时显式失败，避免产生伪 slosh 结果。

验证记录：

```text
python3 scripts/acados/generate_spmpc_acados.py --model b0 --check: OK, nx=6 nu=3 np=23
python3 scripts/acados/generate_spmpc_acados.py --model slosh --check: OK, nx=10 nu=3 np=31
python3 scripts/acados/generate_spmpc_acados.py --model b0_direct_omega_legacy --check: OK, nx=5 nu=3 np=24
python3 scripts/acados/generate_spmpc_acados.py --model b0_direct_omega_legacy: OK
catkin_make --force-cmake -DCATKIN_WHITELIST_PACKAGES="spmpc_local_planner" --pkg spmpc_local_planner: OK
catkin_make run_tests_spmpc_local_planner: OK, 9/9 passed
roslaunch spmpc_local_planner spmpc_fixed_path.launch solver_backend:=continuous_mpcc_direct_omega_legacy planner_variant:=B0 --files: OK
```

fresh-sim B0 验证结果：

```text
run: /data/a/spmpc_b0_p2_continuous_direct_omega_legacy_diag/20260608_173709_spmpc_B0_P2_s_curve_current_start_legacy_run1
solver_backend(meta): continuous_mpcc_direct_omega_legacy
solver_backend(topic): continuous_mpcc_direct_omega_legacy
path length: 8.594 m
omega_req_abs_max(v_ref=0.25): 1.094 rad/s (< omega_max=1.2)
first GOAL_REACHED: 约 33.81 s
progress_s: 0.00029 -> 0.97764
status: B0_ACADOS_DIRECT_OMEGA_LEGACY_OK=998, GOAL_REACHED=743, ACADOS_DIRECT_OMEGA_LEGACY_SOLVE_FAILED_4=17
terminal: TRACKING=816, TERMINAL_SLOWDOWN=109, TERMINAL_CAPTURE_STOP=89, REACHED=743
cmd_v max: 0.445 m/s, last5s mean=0
cmd_omega: [-1.2, 1.2] rad/s
solver_time_ms mean/max: 1.82 / 7.62
odom path/displacement: 8.72 / 7.16 m
```

脚本仍打印 `[timeout] planner 启动后达到 60s`，这是 `RUN_TIMEOUT_SEC=60` 的录包 hard stop；bag 中已经在约 33.81s 进入 `GOAL_REACHED` 并保持停车，因此该 run 按跟踪结果应判为 success。结论：continuous MPCC 在 direct-omega OCP 形态下能跑通同一 current-start fixed-terminal P2 `s_curve`；当前 omega-state/alpha-control 主线的 stall 不是由 fixed-path suite 或仿真环境导致，而是新 OCP 状态/控制布局与 objective/reference/初始化交互导致。
