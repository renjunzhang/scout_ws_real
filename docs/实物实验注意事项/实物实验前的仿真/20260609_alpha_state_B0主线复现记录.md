# 20260609 alpha-state B0 主线复现记录

## 1. 结论

本次 fresh sim 复现的是：

```text
backend = continuous_mpcc_acados
variant = B0
corridor_enable = false
obstacle_enable = false
homotopy_enable = false
alpha_max = platform 默认值 1.2
```

结果：未到点，且不是 RouteB 那种成功到点；alpha-state 主线仍存在严重问题。本次表现不是 20260608 中记录的 `progress≈42%` creep，而是更早在 `progress≈15.7%` 后进度冻结，实际机器人偏离路径并跑到路径右侧/反方向区域。

因此，本次支持以下判断：

```text
RouteB 能走通不代表 alpha-state 主线已修复。
alpha-state continuous_mpcc_acados 仍需要作为头号问题单独定位。
```

## 2. Run 元信息

| 项目 | 值 |
|---|---|
| run dir | `/data/a/scout_spmpc_experiments/raw/2026-06-09/01_alpha_state_mainline_failure/spmpc_alpha_repro/20260609_124323_spmpc_B0_P2_alpha_B0_repro_run1` |
| bag | `/data/a/scout_spmpc_experiments/raw/2026-06-09/01_alpha_state_mainline_failure/spmpc_alpha_repro/20260609_124323_spmpc_B0_P2_alpha_B0_repro_run1/20260609_124323_spmpc_B0_P2_alpha_B0_repro_run1.bag` |
| generated path | `/data/a/scout_spmpc_experiments/raw/2026-06-09/01_alpha_state_mainline_failure/spmpc_alpha_repro/20260609_124323_spmpc_B0_P2_alpha_B0_repro_run1/20260609_124323_spmpc_B0_P2_alpha_B0_repro_run1_generated_path.json` |
| backend | `continuous_mpcc_acados` |
| variant | `B0` |
| git hash in meta | `310ef0d` |
| alpha_max_override | `-1.0`，即使用 platform 默认 `alpha_max=1.2` |
| path length | 约 `8.568 m` |
| max abs curvature | 约 `4.149 1/m` |
| max abs curvature location | `s≈1.85 m` |
| `v=0.25` omega demand | 约 `1.04 rad/s` |
| `v=0.30` omega demand | 约 `1.24 rad/s`，超过 `omega_max=1.2` |

启动日志确认 Stage 0 guard 口径正确：

```text
backend=continuous_mpcc_acados
role=SPMPC mainline continuous MPCC
variant=B0
slosh=false
obstacle=false
homotopy=false
corridor=false
corridor_hard_bound=false
```

## 3. 运行指标

| 指标 | 值 |
|---|---:|
| status OK | `B0_ACADOS_OK = 1699` |
| solve failed | `ACADOS_SOLVE_FAILED_4 = 57` |
| first GOAL_REACHED | 无 |
| terminal mode | 全程 `TRACKING` |
| progress_s | `0.0 -> 0.1575` |
| max progress_s | `0.1575` |
| max progress time | bag 相对时间约 `24.27 s` |
| solver time mean | `5.37 ms` |
| solver time max | `11.21 ms` |
| cmd_v mean, t=5~30s | `0.154 m/s` |
| max cmd_v | `0.8 m/s` |
| max abs cmd_omega | `1.2 rad/s` |
| slosh_height max | `16.77 mm` |

`warm_start_status` 全部为 `OK`，`warm_start_valid_ratio=1.0`，未使用 fallback。因此这次不是 warm-start 生成器直接报 invalid，而是在 OCP 求解与执行闭环中偏离。

## 4. 时间线

| t | odom x | odom y | yaw | cmd_v | cmd_omega | progress_s | status |
|---:|---:|---:|---:|---:|---:|---:|---|
| `0 s` | `2.735` | `0.069` | `3.104` | `0.018` | `-0.041` | `0.0000` | `B0_ACADOS_OK` |
| `5 s` | `2.609` | `0.131` | `2.530~2.3` | `0.000` | `-0.054` | `0.0161` | `B0_ACADOS_OK` |
| `10 s` | `2.518` | `0.180` | `1.574` | `0.000` | `-0.217` | `0.0266` | `B0_ACADOS_OK` |
| `15 s` | `2.407` | `0.328` | `0.473~-0.3` | `0.000` | `0.000` | `0.0350` | `ACADOS_SOLVE_FAILED_4` 附近 |
| `20 s` | `2.426` | `0.209` | `-1.519` | `0.000` | `-0.051` | `0.0403` | `B0_ACADOS_OK` |
| `22 s` | `2.108` | `0.089` | `-2.648` | `0.356` | `-0.095` | `0.0658` | `B0_ACADOS_OK` |
| `24 s` | `1.711` | `-0.578` | `-1.234` | `0.470` | `0.549` | `0.1514` | `B0_ACADOS_OK` |
| `25 s` | `2.101` | `-1.069` | 约 `-0.4~0.1` | `0.800` | `0.722` | `0.1575` | `B0_ACADOS_OK` |
| `30 s` | `3.869` | `-0.927` | `0.638` | `0.438` | `0.074` | `0.1575` | `B0_ACADOS_OK` |
| `40 s` | `5.616` | `3.772` | `0.001` | `0.016` | `0.041` | `0.1575` | `B0_ACADOS_OK` |
| `60 s` | `5.628` | `3.771` | `0.001` | `0.020` | `0.045` | `0.1575` | `B0_ACADOS_OK` |

实际 odom 位移约 `4.70 m`，累计运动距离约 `10.07 m`，但 `progress_s` 在 `0.1575` 后不再增加。说明机器人不是简单停在原地，而是偏离 reference corridor / path projection 后，progress projector 卡在已达到的最小进度附近。

## 5. 与 RouteB B0 的对照

| 项目 | RouteB B0 | alpha-state B0 |
|---|---:|---:|
| backend | `continuous_mpcc_direct_omega_legacy` | `continuous_mpcc_acados` |
| progress_s | `0.0002 -> 0.9783` | `0.0 -> 0.1575` |
| GOAL_REACHED | `802` | `0` |
| solver mean/max | `1.76 / 6.68 ms` | `5.37 / 11.21 ms` |
| failed status | `15` | `57` |
| outcome | 成功到点 | 偏离路径，未到点 |

这个对照进一步说明：

```text
仿真环境、stable_goal path、RouteB continuous MPCC 链路本身不是主要问题。
问题集中在 alpha-state OCP / objective / state-control layout / closed-loop command 交互。
```

## 6. 下一步定位建议

不建议马上调 slosh 权重或切 B_slosh。先做 alpha-state B0 的最小定位：

```text
1. 固定同一个 generated path，先 replay，不再每次 stable_goal 生成新 path；
2. 对比 direct-omega 和 alpha-state 在同一路径前 25s 的 cmd_v / cmd_omega / odom yaw / progress；
3. 导出 alpha-state OCP 第一拍 u0=[a, alpha, v_s]、状态 omega、cmd_omega 积分结果；
4. 检查 alpha-state warm start 是否真的给了合理 alpha 序列，而不是只给 omega 形状；
5. 检查 progress projector 在偏离路径后的 min_progress_s 锁定行为，避免把“跑飞”误判为“creep”。
```

本次现象更准确地说是：

```text
alpha-state B0 在起步高曲率段无法形成稳定闭环跟随，
短暂低速/原地转向后偏离 reference，progress 冻结在 15.7%。
```

它和 20260608 记录中的 `42% creep` 同属 alpha-state 主线问题，但这次 fresh sim 更像 “path departure + progress freeze”，不是单纯慢速 creep。

## 7. 同一路径 replay 复查（用户手动执行）

用户随后按建议用同一个 generated path 分别 replay 了 alpha-state 与 RouteB：

| 项目 | alpha-state replay | RouteB replay |
|---|---|---|
| bag | `/data/a/scout_spmpc_experiments/raw/2026-06-09/01_alpha_state_mainline_failure/spmpc_alpha_repro_replay/20260609_134237_spmpc_B0_P2_alpha_B0_replay_same_path_run1/20260609_134237_spmpc_B0_P2_alpha_B0_replay_same_path_run1.bag` | `/data/a/scout_spmpc_experiments/raw/2026-06-09/00_routeb_baseline/spmpc_routeb_replay/20260609_134622_spmpc_B0_P2_routeb_rate8_on_alpha_path_run1/20260609_134622_spmpc_B0_P2_routeb_rate8_on_alpha_path_run1.bag` |
| backend | `continuous_mpcc_acados` | `continuous_mpcc_direct_omega_legacy` |
| variant | `B0` | `B0` |
| path length | `8.5682 m` | `8.5682 m` |
| max abs curvature | `4.1486 1/m @ s≈1.848 m` | `4.1486 1/m @ s≈1.848 m` |
| progress_s | `0.00493 -> 0.11672` | `0.00165 -> 0.98043` |
| first goal | 无 | status `GOAL_REACHED` at `t≈34.17 s` |
| status counts | `B0_ACADOS_OK=1416`, `ACADOS_SOLVE_FAILED_4=80` | `B0_ACADOS_DIRECT_OMEGA_OK=914`, `ACADOS_DIRECT_OMEGA_SOLVE_FAILED_4=33`, `GOAL_REACHED=838` |
| terminal mode | 全程 `TRACKING` | `TRACKING -> TERMINAL_SLOWDOWN -> TERMINAL_CAPTURE_STOP -> REACHED` |
| solver mean/max | `4.936 / 9.940 ms` | `1.586 / 7.037 ms` |
| cmd_v mean, t=5~30s | `0.021 m/s` | `0.308 m/s` |
| max abs cmd_omega | `1.2 rad/s` | `1.2 rad/s` |
| odom displacement / cumulative | `0.486 / 3.963 m` | `7.122 / 8.823 m` |

结论：

```text
同一路径 replay 排除了“路径文件本身不可走”的解释。
RouteB direct-omega 在同一路径上仍能成功到点；alpha-state B0 仍失败，且这次更早冻结在 progress≈11.7%。
```

前几秒的控制差异很关键：alpha-state replay 起步 `cmd_omega≈0.041 rad/s`，而 RouteB replay 起步 `cmd_omega≈1.056 rad/s`。这与两套 OCP 的结构差异一致：

```text
alpha-state:
  x 包含 omega，u[1]=alpha=omega_dot；cmd_omega = measured_omega + alpha_0 * dt
  默认 alpha_max=1.2 rad/s^2 时，单步只能把 omega 改约 0.04 rad/s

RouteB direct-omega:
  u[1]=omega；OCP 可直接选择接近曲率需求的 omega，出口再做 rate limit
```

结合该 path 前段 `v=0.30 m/s` 时曲率角速度需求约 `1.24 rad/s`，alpha-state 的硬角加速度约束/omega-state 初值成为主要嫌疑，而不是 slosh 或 obstacle/corridor/homotopy。

## 8. 代码层嫌疑点

本次 replay 后，定位优先级应调整为：

1. **alpha-state 硬角加速度约束是否过强**  
   `scripts/acados/spmpc_acados_constraints.py:31-46` 对 alpha-state 设置 `u=[a,alpha,v_s]` 且 `alpha∈[-alpha_max,alpha_max]`、`omega state∈[-omega_max,omega_max]`；RouteB 在同文件 `13-26` 是 `u=[a,omega,v_s]`，没有 OCP 内的 alpha 硬约束。

2. **alpha-state 首帧 omega 从实测 omega 积分，而不是直接采用规划 omega**  
   `src/solvers/continuous_mpcc_solver_acados.cpp:558-571` 将 `x0[5]` 固定为 `input.robot.omega`；`696-702` 下发 `cmd_omega = input.robot.omega + u0[1] * dt`。这解释了起步只能从接近 0 的 measured omega 缓慢爬升。

3. **RouteB 能成功是因为直接 omega OCP 有更强首帧角速度能力**  
   `src/solvers/continuous_mpcc_direct_omega_legacy_solver_acados.cpp:406-417` 中 direct-omega 的 `u0[1]` 就是规划角速度，随后才在出口 rate limit 到上一帧命令附近。

4. **alpha-state warm-start 可能存在语义错位**  
   `include/.../continuous_mpcc_solver_acados.h:38` 的 `u_prev_` 注释仍写 `[a, omega, v_s]`；但 alpha-state control 读取处 `src/solvers/continuous_mpcc_solver_acados.cpp:186-189` 已是 `[a, alpha, v_s]`。`makeWarmStartInput()` 在 `255-260` 把 `u_prev[1]` 赋给 `previous_omega`，而 solve 成功后 `696-702` 又把 `u0[1]`（alpha）存回 `u_prev_[1]`。这会导致 warm-start generator 收到“上一周期 alpha”却当作“上一周期 omega”使用，是高曲率段 warm-start 异常的强嫌疑。

下一步最小实验建议不是马上改 slosh 权重，而是固定同一路径做 alpha-state B0 的角加速度诊断：

```text
A. continuous_mpcc_acados + B0 + 同一路径 + alpha_max=8.0
   若显著改善/到点，说明 OCP 内 alpha 硬约束是主导因素。

B. 代码侧修正 u_prev_ 语义：alpha-state 分开保存上一周期 alpha 与上一周期下发 omega，避免 previous_omega 被 alpha 污染。
```

## 9. alpha_max=8.0 replay 结果：仍失败

用户随后按 A 分支补跑：

| 项目 | 值 |
|---|---|
| bag | `/data/a/scout_spmpc_experiments/raw/2026-06-09/01_alpha_state_mainline_failure/spmpc_alpha_repro_alpha8/20260609_154815_spmpc_B0_P2_alpha_B0_replay_same_path_alpha8_run1/20260609_154815_spmpc_B0_P2_alpha_B0_replay_same_path_alpha8_run1.bag` |
| backend | `continuous_mpcc_acados` |
| variant | `B0` |
| alpha_max_override | `8.0` |
| path length | `8.5682 m` |
| max abs curvature | `4.1486 1/m @ s≈1.848 m` |
| progress_s | `0.00426 -> 0.11467` |
| first progress > 0.1 | `t≈53.15 s` |
| first goal | 无 |
| status counts | `B0_ACADOS_OK=1691`, `ACADOS_SOLVE_FAILED_4=66` |
| terminal mode | 全程 `TRACKING` |
| warm_start_status | 全部 `OK` |
| solver mean/max | `5.460 / 11.117 ms` |
| cmd_v mean, t=5~30s | `0.026 m/s` |
| max abs cmd_omega | `1.2 rad/s` |
| odom displacement / cumulative | `0.990 / 3.623 m` |

planner log 确认：

```text
/spmpc_local_planner/robot/alpha_max: 8.0
backend=continuous_mpcc_acados
variant=B0
features: slosh=false obstacle=false homotopy=false corridor=false
```

但实际表现没有恢复。与默认 alpha replay 相比，`progress≈0.11672` 与 `progress≈0.11467` 基本同量级，且仍无 `GOAL_REACHED`。因此：

```text
单纯把 alpha_max 从 1.2 提到 8.0 不能解决 alpha-state B0 同路径失败。
下一步不应进入 fresh sim 连续跑，也不应进入实物；应转入代码侧修正/定位。
```

当前优先分支变为 B，但进一步读代码后，真实问题比 `u_prev_[1]` 注释错位更直接：flatness warm start 本身仍按旧 direct-omega 语义生成。

## 10. 代码侧修正：让 flatness warm-start 对齐 alpha-state OCP

进一步检查发现：

```text
DiffDriveFlatnessWarmStart 旧行为：
  只填 control.omega = kappa * v
  不填 state.omega
  不填 control.alpha

continuous_mpcc_acados 注入行为：
  x[5] <- WarmStartState::omega
  u[1] <- WarmStartControl::alpha
```

因此曲率 yaw-rate seed 实际留在 legacy/debug 字段 `control.omega`，没有进入 alpha-state OCP 初值。`warm_start_status=OK` 只能说明旧检查通过，不能说明 alpha-state 字段已经合理。

已做的外科手术式修正：

```text
1. DiffDriveFlatnessWarmStart 从 input.robot.omega 开始生成 rate-bounded state.omega 序列；
2. 按 omega 递推关系生成 control.alpha：
   states[k+1].omega ≈ states[k].omega + controls[k].alpha * dt；
3. params.alpha_max 传入 WarmStartBounds::omega_rate_max，用作 warm-start alpha bound；
4. slosh rollout、finite check、max_omega 与横向加速度诊断改用 state.omega；
5. control.omega 仅保留为 legacy/debug mirror。
```

验证结果：

```text
catkin_make --pkg spmpc_local_planner 通过；
catkin_make run_tests_spmpc_local_planner_gtest_test_diff_drive_flatness_warm_start 通过 4/4；
catkin_make --pkg scout_local_planner 仍因 build 目标缺失失败：没有规则可制作目标“scout_apps/control/scout_local_planner/all”。
```

下一步仍然不是实物，而是让用户用修正后的代码重跑同一路径 replay gate。通过后再进入 fresh sim 连续验证。

## 11. warm-start 修正后同一路径 replay：仍未通过 gate

用户随后用修正后的代码补跑同一路径 replay：

| 项目 | 值 |
|---|---|
| bag | `/data/a/scout_spmpc_experiments/2026-06-09/01_after_warmstart_fix/20260609_165100_spmpc_B0_P2_alpha_B0_replay_warmstart_fix_alpha8_run1/20260609_165100_spmpc_B0_P2_alpha_B0_replay_warmstart_fix_alpha8_run1.bag` |
| backend | `continuous_mpcc_acados` |
| variant | `B0` |
| alpha_max_override | `8.0` |
| path length | `8.5682 m` |
| max abs curvature | `4.1486 1/m @ s≈1.848 m` |

结果仍未到点：

| 指标 | pre-fix alpha8 replay | warm-start fix + alpha8 replay | RouteB same-path replay |
|---|---:|---:|---:|
| progress_s | `0.00426 -> 0.11467` | `0.00516 -> 0.19246` | `0.00165 -> 0.98043` |
| first progress > 0.1 | `t≈53.15 s` | `t≈31.35 s` | `t≈7.14 s` |
| first GOAL_REACHED | 无 | 无 | `t≈34.17 s` |
| status OK | `B0_ACADOS_OK=1691` | `B0_ACADOS_OK=1653` | `B0_ACADOS_DIRECT_OMEGA_OK=914` |
| solve failed | `ACADOS_SOLVE_FAILED_4=66` | `ACADOS_SOLVE_FAILED_4=104` | `ACADOS_DIRECT_OMEGA_SOLVE_FAILED_4=33` |
| terminal mode | 全程 `TRACKING` | 全程 `TRACKING` | `TRACKING -> ... -> REACHED` |
| cmd_v mean, t=5~30s | `0.026 m/s` | `0.024 m/s` | `0.308 m/s` |
| max cmd_v | `0.623 m/s` | `0.800 m/s` | `0.582 m/s` |
| max abs cmd_omega | `1.2 rad/s` | `1.144 rad/s` | `1.2 rad/s` |
| solver mean/max | `5.46 / 11.12 ms` | `6.08 / 14.60 ms` | `1.59 / 7.04 ms` |
| odom displacement / cumulative | `0.990 / 3.623 m` | `5.670 / 6.621 m` | `7.122 / 8.823 m` |

时间线显示：warm-start 修正后并不是恢复跟踪，而是前 `~25 s` 基本停在起点附近低速/原地调整；`t≈30 s` 左右突然给到较大 `cmd_v`，随后机器人明显偏离 reference，`progress_s` 在 `0.19246` 附近冻结。按全局路径最近距离粗算，cross-track sampled max 约 `5.21 m`，因此这次应判为：

```text
同一路径 replay gate 仍失败；且失败形态是 path departure + progress freeze，不能进入 fresh-sim 连续验证或实物试验。
```

`/spmpc/debug/warm_start_status` 仍全程 `OK`，`/spmpc/debug/warm_start` 显示 `used_flatness=1`、`max_omega=1.2`、`used_slosh_rollout=1`，说明修正后的 warm-start 管线在运行，但这还不足以解决 alpha-state 闭环问题。

与 20260608 creep 记录的关系：20260608 主要是 `v/v_s` tracking 后仍在弯处低速停滞；这次同路径 replay 则先长时间起步困难，随后跑飞并被 projection/progress 锁在前段。两者都指向 alpha-state `omega` 状态 + `alpha` 控制结构与 objective/projection/闭环 command 的交互问题，不能再归因于单纯 `alpha_max` 或单纯 flatness warm-start 字段错位。

下一步建议：不要继续扫 slosh 权重，也不要进实物；先增加 alpha-state 第一拍/预测轨迹诊断，导出 `x0.omega`、warm-start `state.omega/control.alpha`、OCP `u0=[a,alpha,v_s]`、`cmd_omega=input.robot.omega+alpha*dt`、reference `kappa/v*kappa`、contour/lag error 与 nearest-path distance，用来判断是 stage-0 动态能力、objective 权重、projection 锁定还是局部轨迹方向导致跑飞。