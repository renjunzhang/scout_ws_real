# 2026-06-09 alpha-state 主线实物前 No-Go 与 catkin 白名单问题

## 1. 当前总判断

截至 2026-06-09，正式实物试验结论仍是：

```text
NO-GO：不要进入正式地面闭环 / 带液体 / 论文口径实物采集。
```

原因不是单个编译命令失败，而是主线后端 `continuous_mpcc_acados` 的 alpha-state 闭环还没有完成实物前 gate。默认/前序口径 replay 仍失败；runtime-bounds 诊断版默认 `alpha_max=1.2` 仍失败；最新 `alpha_max=8.0` 口径同路径 Gate 1 已有 3 次独立样本中的 2 次 PASS / 1 次 FAIL，说明部分可复现但稳定性不足；同时尚未补齐 path-departure / hard-zero 等实物安全门：

```text
warm-start fix + alpha8 replay:
  progress_s = 0.00516 -> 0.19246
  GOAL_REACHED = 0
  terminal mode = 全程 TRACKING
  表现 = path departure + progress freeze

subagent Gate 1 fresh-sim replay:
  progress_s = 0.0028602513 -> 0.0028689418
  GOAL_REACHED = 0
  terminal mode = 全程 TRACKING
  表现 = 70s 内几乎无有效推进

runtime-bounds 诊断版 Gate 1 fresh-sim replay:
  progress_s = 0.002255 -> 0.008541
  GOAL_REACHED = 0
  terminal mode = 全程 TRACKING
  runtime_bounds.alpha_max = 1.2
  generated_bounds.alpha_max = 1.2
  表现 = 新增 debug topics 正常录包，但仍卡在起点附近

alpha8 runtime-bounds Gate 1 fresh-sim replay:
  run1 PASS: progress_s = 0.002821 -> 0.977385, first GOAL_REACHED ≈ 50.11s
  run2 FAIL: progress_s = 0.000715 -> 0.004223, first GOAL_REACHED = 无
  run3 PASS: progress_s = 0.000713 -> 0.980470, first GOAL_REACHED ≈ 45.06s
  runtime_bounds.alpha_max ≈ 8.0
  generated_bounds.alpha_max ≈ 1.2
  表现 = 3 次中 2 PASS / 1 FAIL，alpha8 部分可复现但稳定性不足
```

因此当前只建议做低风险链路检查，例如：

```text
1. 架空轮；
2. 无液体；
3. 低速；
4. 急停在手；
5. 只确认 launch / topic / rosbag / 相机 / slosh monitor 是否工作。
```

不建议做：

```text
1. 地面正式闭环跑车；
2. 带液体实物试验；
3. B0 / B_slosh / B_ours 论文口径实物对比采集；
4. 用 RouteB direct-omega 成功替代 alpha-state 主线结论。
```

## 2. 问题总览

| 编号 | 问题 | 当前证据 | 对实物的影响 | 状态 |
|---|---|---|---|---|
| P0 | alpha-state B0 同路径 gate 尚未稳定通过 | warm-start 修正后最高只到 `0.19246`；默认 runtime-bounds 诊断版 replay 只到 `0.008541`；alpha8 runtime-bounds 版 3 次为 `2 PASS / 1 FAIL`，成功样本到点约 `50.11s`、`45.06s`，失败样本 max `progress_s=0.004223` | alpha8 部分可复现但稳定性不足；仍需失败机理诊断/更多复现与安全门 | 部分通过 |
| P0 | 存在 path departure + progress freeze | odom 位移 `5.67 m`，但 progress 冻结，粗略 cross-track max `≈5.21 m` | 实物有跑偏风险 | 未解决 |
| P0 | RouteB 成功不能代表主线成功 | RouteB 是 `u=[a,omega,v_s]`，主线是 `u=[a,alpha,v_s]` | 不能作为放行依据 | 已澄清 |
| P1 | warm-start 修正只解决字段语义，不等于闭环通过 | 单测通过，但 replay 仍失败 | 需要继续定位 OCP/command/projection | 部分解决 |
| P1 | catkin 红线命令失败被 whitelist/cache 干扰 | `CATKIN_WHITELIST_PACKAGES=spmpc_local_planner`，无 `scout_local_planner/all` 目标 | 红线验证未完成，但不是源码编译错误证据 | 需重跑 |
| P1 | fallback / previous-solution 与 alpha-state 一致性仍有风险 | fallback 可能不满足 `omega_dot=alpha` | 实物前应修或 gate 掉 | 待处理 |
| P2 | slosh/RGB/rosbag 链路只部分确认 | monitor topic 可记录，但真实液面指标需 RGB 对齐 | 带液体前需 dry-run | 待验证 |

## 3. 问题一：alpha-state 主线同路径 replay 仍失败

### 3.1 失败记录

参考记录：

```text
docs/实物实验注意事项/实物实验前的仿真/20260609_alpha_state_B0主线复现记录.md
```

当前关键 replay：

```text
/data/a/scout_spmpc_experiments/2026-06-09/01_after_warmstart_fix/20260609_165100_spmpc_B0_P2_alpha_B0_replay_warmstart_fix_alpha8_run1/20260609_165100_spmpc_B0_P2_alpha_B0_replay_warmstart_fix_alpha8_run1.bag
```

运行口径：

```text
backend = continuous_mpcc_acados
variant = B0
alpha_max_override = 8.0
path length = 8.5682 m
max |kappa| = 4.1486 1/m @ s≈1.848 m
```

结果：

| 指标 | 值 |
|---|---:|
| `progress_s` | `0.00516 -> 0.19246` |
| `GOAL_REACHED` | `0` |
| terminal mode | 全程 `TRACKING` |
| status | `B0_ACADOS_OK=1653`, `ACADOS_SOLVE_FAILED_4=104` |
| solver mean/max | `6.08 / 14.60 ms` |
| `cmd_v` mean, t=5~30s | `0.024 m/s` |
| max `cmd_v` | `0.800 m/s` |
| max abs `cmd_omega` | `1.144 rad/s` |
| odom displacement / cumulative | `5.670 / 6.621 m` |
| 粗略 cross-track max | `≈5.21 m` |

### 3.2 与前序 replay 对比

| 指标 | 默认 alpha replay | pre-fix alpha8 replay | warm-start fix + alpha8 replay | RouteB same-path replay |
|---|---:|---:|---:|---:|
| backend | `continuous_mpcc_acados` | `continuous_mpcc_acados` | `continuous_mpcc_acados` | `continuous_mpcc_direct_omega_legacy` |
| `progress_s` | `0.00493 -> 0.11672` | `0.00426 -> 0.11467` | `0.00516 -> 0.19246` | `0.00165 -> 0.98043` |
| first progress > 0.1 | 未单列 | `t≈53.15 s` | `t≈31.35 s` | `t≈7.14 s` |
| first `GOAL_REACHED` | 无 | 无 | 无 | `t≈34.17 s` |
| status failed | `ACADOS_SOLVE_FAILED_4=80` | `ACADOS_SOLVE_FAILED_4=66` | `ACADOS_SOLVE_FAILED_4=104` | `ACADOS_DIRECT_OMEGA_SOLVE_FAILED_4=33` |
| `cmd_v` mean, t=5~30s | `0.021 m/s` | `0.026 m/s` | `0.024 m/s` | `0.308 m/s` |
| odom displacement / cumulative | `0.486 / 3.963 m` | `0.990 / 3.623 m` | `5.670 / 6.621 m` | `7.122 / 8.823 m` |

解释：

```text
warm-start 修正后 progress 从约 0.115 提到约 0.192，有改善；
但仍远低于 progress_s > 0.95 / GOAL_REACHED 的 gate，且出现明显跑偏。
```

因此这条 replay 不能作为实物前通过证据。

注意：当前 warm-start fix 后补跑的是 `alpha_max=8.0` replay；默认 `alpha_max=1.2` 口径仍需在后续 gate 中单独确认。但 `alpha_max=8.0` 都未到点，已经足够维持 No-Go。

### 3.3 子 agent Gate 1 fresh-sim replay 复跑

2026-06-09 晚间，按 `docs/实物实验注意事项/Subagent.md` 约束补跑 1 个安全约束子 agent Gate 1 replay。仿真启动命令使用用户指定的 `launch_sim_nav_stack.sh` 口径，启动后等待 30s，planner 跟踪 70s 超时判失败，结束后关闭本次仿真并等待 30s。

run 目录：

```text
/data/a/scout_spmpc_experiments/raw/2026-06-09/02_gate1_replay_subagent/20260609_1case/20260609_231045_spmpc_B0_P2_alpha_B0_replay_warmstart_fix_alpha8_rearchive_run1
```

复跑使用的同路径文件：

```text
/data/a/scout_spmpc_experiments/raw/2026-06-09/01_alpha_state_mainline_failure/spmpc_alpha_repro/20260609_124323_spmpc_B0_P2_alpha_B0_repro_run1/20260609_124323_spmpc_B0_P2_alpha_B0_repro_run1_generated_path.json
```

结果：

| 指标 | 值 |
|---|---:|
| verdict | `FAIL` |
| `progress_s` | `0.0028602513 -> 0.0028689418` |
| max `progress_s` | `0.0028689418` |
| `GOAL_REACHED` | `0` |
| terminal mode | 全程 `TRACKING`，计数 `2057` |
| status | `B0_ACADOS_OK=2043`, `ACADOS_SOLVE_FAILED_4=14`, `WAITING_FOR_ODOM=1` |
| solver mean/max | `5.576 / 11.206 ms` |
| `cmd_v` mean, t=5~30s | `0.000794 m/s` |
| max `cmd_v` | `0.0690 m/s` |
| max abs `cmd_omega` | `1.1169 rad/s` |
| odom displacement / cumulative | `0.0766 / 0.3872 m` |
| sampled path departure max / end | `0.1676 / 0.1512 m` |
| warm_start_status | `OK=2057` |

解释：

```text
这次不是“跑偏后 progress freeze 到 0.19”，而是更早阶段就几乎完全没有有效推进。
求解时间预算未爆，warm_start_status 全程 OK，但闭环推进失败。
```

因此 Gate 1 继续失败，正式实物仍保持 NO-GO。

### 3.4 子 agent 离线诊断结论

对子 agent Gate 1 replay 的 bag / meta / planner log 做离线诊断后，当前结论分层如下。

实验事实：

```text
1. backend / variant / mode 可核对为 continuous_mpcc_acados / B0 / fixed_path；
2. /spmpc/solver_backend 记录为 continuous_mpcc_acados；
3. progress_s 在起点后约 0.05s 达到最大值 0.0028689418，之后 5~70s 不再增长；
4. cmd_v 近零，但 cmd_omega 可到 1.1169 rad/s；
5. turn_only 帧显著多于 move_only / move_turn；
6. warm_start_status=OK 全程，used_flatness=1，used_fallback=0；
7. terminal 全程 TRACKING，envelope_active=0；
8. 正常求解周期里 cmd_v_pre_clamp == cmd_v_post_clamp。
```

离线推断：

```text
1. cmd_v 近零的主因不是 terminal clamp、WAITING、wrapper 后级压低；
2. 更像是 continuous_mpcc_acados + B0 正常求解周期本身给出低推进输出；
3. 当前闭环表现支持“高转向、低推进 / 近原地转向”的失败形态；
4. 这次失败更像起始附近 progress/projection 区域卡住，而不是先跑偏几米再 freeze。
```

仍未验证：

```text
1. bag 中 /spmpc/debug/warm_start 只是聚合诊断，不含逐拍 state.omega / control.alpha / v_s；
2. 尚不能把 alpha-state seed 与 reference-snapped theta 不一致写成已证实事实；
3. alpha_max_override=8.0 能确认进入 wrapper/runtime 记录口径，但仍不能仅凭 bag/log 证明已进入 acados OCP lbu/ubu。
```

离线诊断报告：

```text
/data/a/scout_spmpc_experiments/raw/2026-06-09/02_gate1_replay_subagent/20260609_1case/20260609_231045_spmpc_B0_P2_alpha_B0_replay_warmstart_fix_alpha8_rearchive_run1/20260609_231045_spmpc_B0_P2_alpha_B0_replay_warmstart_fix_alpha8_rearchive_run1_offline_diagnostics.md
```

### 3.5 runtime-bounds 诊断版 Gate 1 fresh-sim replay

2026-06-10，运行时 acados bounds 显式下发与首拍诊断 topic 接入后，按 `docs/实物实验注意事项/Subagent.md` 约束又补跑 1 个同路径 Gate 1 replay。仿真 fresh 启动后等待 30s，只跑 1 个 case，70s 未到点判失败，结束后安全关闭本次启动进程并等待 30s。

run 目录：

```text
/data/a/scout_spmpc_experiments/raw/2026-06-10/03_gate1_replay_runtime_bounds/20260610_010202_spmpc_B0_P2_alpha_B0_gate1_runtime_bounds_run1
```

复跑使用的同路径文件：

```text
/data/a/scout_spmpc_experiments/raw/2026-06-09/01_alpha_state_mainline_failure/spmpc_alpha_repro/20260609_124323_spmpc_B0_P2_alpha_B0_repro_run1/20260609_124323_spmpc_B0_P2_alpha_B0_repro_run1_generated_path.json
```

结果：

| 指标 | 值 |
|---|---:|
| verdict | `FAIL` |
| `progress_s` | `0.002255 -> 0.008541` |
| max `progress_s` | `0.008541` |
| `GOAL_REACHED` | `0` |
| terminal mode | 全程 `TRACKING`，计数 `2724` |
| status | `B0_ACADOS_OK=2719`, `ACADOS_SOLVE_FAILED_4=5` |
| warm_start_status | `OK=2724` |
| solver mean/max | `5.207 / 10.879 ms` |
| `cmd_v` early 5s mean/max | `0.000199 / 0.024 m/s` |
| `cmd_v` full mean/max | `0.0008 / 0.144369 m/s` |
| `cmd_omega` early 5s mean/max | `0.079312 / 0.689282 rad/s` |
| `cmd_omega` full mean/max | `0.017552 / 1.175397 rad/s` |

新增 topic 录包情况：

```text
/spmpc/debug/runtime_bounds       2724 条
/spmpc/debug/generated_bounds     2724 条
/spmpc/debug/first_shot_summary   2724 条
```

关键诊断：

```text
runtime_bounds.alpha_max   = 1.2
generated_bounds.alpha_max = 1.2
first u0_a/u0_alpha/u0_vs  = -0.008475 / 1.2 / 0.0
first cmd_v_pre/post       = 0.0 / 0.0
first cmd_omega_pre/post   = 0.04 / 0.04
x1(v,omega,s)              = 0.0 / 0.04 / 0.0193
x2(v,omega,s)              = 0.0 / 0.08 / 0.0193
x3(v,omega,s)              = 0.0 / 0.12 / 0.0193
```

解释：

```text
1. 新增 runtime/generated/first-shot 诊断 topic 发布与录包链路正常；
2. 本次运行口径是默认 alpha_max=1.2，不是 alpha_max=8.0；
3. 首拍已显示 u0_alpha 打到上界、u0_vs 近零、cmd_v 近零、cmd_omega 非零；
4. 失败形态仍是“可转向、低推进、起点附近几乎不走”；
5. 该 replay 进一步确认 alpha-state 主线仍未通过 Gate 1，不能作为实物放行依据。
```

分析报告：

```text
/data/a/scout_spmpc_experiments/raw/2026-06-10/03_gate1_replay_runtime_bounds/20260610_010202_spmpc_B0_P2_alpha_B0_gate1_runtime_bounds_run1/20260610_010202_spmpc_B0_P2_alpha_B0_gate1_runtime_bounds_run1_analysis.md
```

### 3.6 alpha8 runtime-bounds 版 Gate 1 fresh-sim replay

2026-06-10，按同一安全 SOP 单独补跑 `continuous_mpcc_acados + B0 + alpha_max:=8.0` 同路径 Gate 1 replay。子 agent 先核对 `spmpc_fixed_path.launch` 真实参数名为 `alpha_max`，再启动 fresh sim；仿真启动后等待 30s，只跑 1 个 case，结束后安全关闭本次启动进程并等待 30s。

run 目录：

```text
/data/a/scout_spmpc_experiments/raw/2026-06-10/04_gate1_replay_runtime_bounds_alpha8/20260610_012718_spmpc_B0_P2_alpha_B0_gate1_runtime_bounds_alpha8_run1
```

结果：

| 指标 | 值 |
|---|---:|
| verdict | `PASS` |
| first `GOAL_REACHED` | `≈50.11 s` |
| first terminal `REACHED` | `≈50.11 s` |
| `progress_s` | `0.002821 -> 0.977385` |
| max `progress_s` | `0.977385` |
| status | `WAITING_FOR_ODOM=1`, `B0_ACADOS_OK=1344`, `ACADOS_SOLVE_FAILED_4=160`, `GOAL_REACHED=811` |
| warm_start_status | `OK=2315` |
| terminal mode | `TRACKING=1308`, `TERMINAL_SLOWDOWN=107`, `TERMINAL_CAPTURE_STOP=89`, `REACHED=811` |
| solver mean/max | `3.565 / 12.427 ms` |
| `cmd_v` early 5s mean/max | `0.190268 / 0.596466 m/s` |
| `cmd_v` until first goal mean/max | `0.396202 / 0.8 m/s` |
| `abs(cmd_omega)` early 5s mean/max | `0.298169 / 1.2 rad/s` |
| `abs(cmd_omega)` until first goal mean/max | `0.588127 / 1.2 rad/s` |

关键诊断：

```text
runtime_bounds.alpha_max   ≈ 8.0
generated_bounds.alpha_max ≈ 1.2
first u0_a/u0_alpha/u0_vs  = -0.007414 / 8.0 / 0.023924
first cmd_v_pre/post       ≈ 7.88e-10 / 7.88e-10
first cmd_omega_pre/post   ≈ 0.266983 / 0.266983
第 11 条样本 cmd_v_post     ≈ 0.080727
第 11 条样本 cmd_omega_post ≈ 0.799227
```

解释：

```text
1. alpha8 运行时约束已进入诊断链：runtime_bounds.alpha_max≈8.0；
2. generated_bounds 仍保持烘焙参考 alpha_max≈1.2，符合预期；
3. 与默认 alpha=1.2 失败 replay 不同，alpha8 同路径 Gate 1 单次达到 GOAL_REACHED；
4. 该结果说明 alpha8 口径值得继续复现，但单次 replay 不能解除实物 No-Go；
5. 后续仍需 2~3 次独立 fresh-sim alpha8 复现，并补齐 path-departure safety gate、hard-zero command path、dry-run 链路后，才能讨论实物放行。
```

分析报告：

```text
/data/a/scout_spmpc_experiments/raw/2026-06-10/04_gate1_replay_runtime_bounds_alpha8/20260610_012718_spmpc_B0_P2_alpha_B0_gate1_runtime_bounds_alpha8_run1/20260610_012718_spmpc_B0_P2_alpha_B0_gate1_runtime_bounds_alpha8_run1_analysis.md
```

### 3.7 alpha8 独立 fresh-sim 复现批次

继续按同一安全 SOP 串行补跑 2 次 `continuous_mpcc_acados + B0 + alpha_max:=8.0` 同路径 Gate 1 fresh-sim 复现。每次 fresh 启动后等待 30s，只跑 1 个 case，70s gate，结束后安全关闭本次启动进程并等待 30s。

batch 目录：

```text
/data/a/scout_spmpc_experiments/raw/2026-06-10/05_gate1_alpha8_independent_repro
```

新增两次复现结果：

| run | verdict | first goal time | max `progress_s` | runtime alpha | generated alpha |
|---|---|---:|---:|---:|---:|
| run2 | `FAIL` | 无 | `0.004223` | `8.0` | `1.2` |
| run3 | `PASS` | `45.06s` | `0.980470` | `8.0` | `1.2` |

与 run1 合并后的 alpha8 同路径 Gate 1 样本：

| run | verdict | first goal time | max `progress_s` |
|---|---|---:|---:|
| run1 | `PASS` | `50.11s` | `0.977385` |
| run2 | `FAIL` | 无 | `0.004223` |
| run3 | `PASS` | `45.06s` | `0.980470` |

关键观察：

```text
1. 3 次 alpha8 样本中 2 次 PASS / 1 次 FAIL，INVALID_ALPHA8=0；
2. run1 / run3 两个成功样本的 first goal time 分别约 50.11s 和 45.06s，成功形态接近；
3. run2 是有效 alpha8 配置，但仍卡在起点附近，max progress_s 只有 0.004223；
4. alpha8 不是“单次偶然唯一成功”，但仍不能称为稳定通过；
5. 正式实物仍应保持 NO-GO。
```

batch summary：

```text
/data/a/scout_spmpc_experiments/raw/2026-06-10/05_gate1_alpha8_independent_repro/20260610_gate1_alpha8_independent_repro_batch_summary.md
```

### 3.8 alpha8 run2 失败离线诊断

对子 agent 生成的 run1/run2/run3 bag / meta / report 做只读离线对比后，run2 失败的当前判断如下。

诊断目录：

```text
/data/a/scout_spmpc_experiments/raw/2026-06-10/06_alpha8_run2_failure_diagnostics/20260610_121338_alpha8_run2_offline_diagnostics
```

诊断报告：

```text
/data/a/scout_spmpc_experiments/raw/2026-06-10/06_alpha8_run2_failure_diagnostics/20260610_121338_alpha8_run2_offline_diagnostics/alpha8_run2_offline_diagnostics.md
```

已证实事实：

```text
1. run2 不是 INVALID_ALPHA8：runtime_bounds.alpha_max=8.0，generated_bounds.alpha_max≈1.2；
2. run1/run2/run3 的 backend、variant、path JSON、spawn 口径、planner 参数、record topics 一致；
3. run2 与 run3 首拍 x0_s/progress_s 几乎相同，不支持“一开始投影到完全错误 path 段”；
4. run2 在前 0.3~5s 有过非零 cmd_v/cmd_omega，不是起步就完全零命令；
5. run2 在约 4.97s 达到 progress_s=0.004223 后冻结，x0_s 长期停在约 0.03619；
6. run2 全程 warm_start_status=OK、terminal=TRACKING，solver time 正常，solve fail 次数不比 PASS 样本更坏。
```

关键对比：

| 指标 | run1 PASS | run2 FAIL | run3 PASS |
|---|---:|---:|---:|
| max `progress_s` | `0.977385` | `0.004223` | `0.980470` |
| first goal | `50.11s` | 无 | `45.06s` |
| `progress_s` 5s | `0.106222` | `0.004223` | `0.105054` |
| 首次 `progress_s>0.01` | `3.00s` | 无 | `3.45s` |
| 0~5s `cmd_v` mean | `0.188` | `0.0395` | `0.176` |
| 5~15s `cmd_v` mean | `0.713` | `7.1e-05` | `0.621` |
| 0~30s odom displacement | `3.340m` | `0.268m` | `4.871m` |

高置信推断：

```text
run2 最可能不是 alpha8 配置失效、path 消息缺失或 ROS 启动异常；
更像是在与 run3 几乎相同的起点投影下，前 5s 内落入“短暂转向/前冲后 s 卡在前端、随后 cmd_v/cmd_omega 塌零”的近起点低推进局部锁死解。
```

下一步最小诊断建议：

```text
1. 只看 run2 0~6s，补导出 nearest-path distance / projection s / contour error / lag error / reference kappa；
2. 补 warm-start state.omega / control.alpha / v_s 与 local trajectory 前几拍 px/py/theta/omega/s；
3. 先确认锁死来自 projection 侧还是 OCP 局部解侧，再考虑改控制逻辑或权重；
4. 正式实物仍保持 NO-GO。
```

### 3.9 alpha8 run2 0~6s projection / OCP 聚焦诊断

按上一节建议，针对 run2 的 `0~6s` 做只读离线 projection / OCP 聚焦诊断，并与 run1/run3 PASS 对比。该分析没有启动仿真、planner 或 `/cmd_vel`。

诊断目录：

```text
/data/a/scout_spmpc_experiments/raw/2026-06-10/07_alpha8_run2_0_6s_projection_diagnostics
```

诊断报告：

```text
/data/a/scout_spmpc_experiments/raw/2026-06-10/07_alpha8_run2_0_6s_projection_diagnostics/run2_0_6s_projection_diagnostics.md
```

关键对比：

| 指标 | run1 PASS | run2 FAIL | run3 PASS |
|---|---:|---:|---:|
| `progress_s` 6s | `0.177138` | `0.004223` | `0.187167` |
| first `progress_s>=0.005` | `0.44s` | 无 | `3.25s` |
| first `progress_s>=0.1` | `4.94s` | 无 | `4.95s` |
| 5s 独立投影 `s_abs` | `0.9679m` | `0.0m` | `0.9688m` |
| 5s nearest distance | `0.015m` | `0.336m` | `0.143m` |
| 5s `cmd_v_post` | `0.525` | `0.004` | `0.698` |
| 5s solver `x0_s` | `0.910` | `0.036` | `0.900` |

聚焦事实：

```text
1. run2 的 path front 独立投影在 0~6s 内基本没有向前移动；
2. run2 在 0.35s 短暂出现 u0_vs≈0.148、cmd_v_post≈0.119、cmd_omega_post≈1.111，但 1s 后 progress 已冻结；
3. run2 到 5~6s 仍离 path front 约 0.33m，yaw error 约 0.81~0.96rad；
4. run2 的 local trajectory 前几拍在 3~6s 也投影回 path front，而不是规划出沿路径前进的局部轨迹；
5. 起点附近独立参考曲率为 0，因此当前证据不支持“第一厘米就是高曲率导致锁死”；
6. run2 的 `J_contour/J_lag` 在 3~6s 未爆炸，说明局部代价可能把 front-clamped 低推进解视为可接受。
```

当前更具体的判断：

```text
run2 更像 projection、heading/yaw-rate 对齐与 OCP 目标耦合形成的 near-start low-progress basin：机器人短暂转向/前冲后没有获得 planner 接受的持续 progress，随后 solver 输入和局部轨迹一起回到 path front 附近的低推进解。
```

### 3.10 已加 debug-only 话题：给别人请教时重点看什么

2026-06-10 已按上面建议加了只读诊断，不改控制逻辑、不改权重、不改 acados codegen、不改已有 topic layout。它只把 planner 内部近起点状态额外发布到 bag，供下一次 alpha8 replay 或 run2-style 0~6s 分析使用。

新增 topic：

```text
/spmpc/debug/projector
/spmpc/debug/stage0_reference
/spmpc/debug/local_traj_head
/spmpc/debug/warm_start_head
```

字段含义：

```text
/spmpc/debug/projector
  raw_valid,raw_s,raw_distance,raw_signed_distance,raw_x,raw_y,raw_yaw,
  guarded_valid,guarded_s,guarded_distance,guarded_signed_distance,guarded_x,guarded_y,guarded_yaw,
  min_progress_s,monotonic_clip_applied

  目的：区分“纯最近投影 raw_s 卡在 path front”还是“min_progress_s / monotonic guard 把投影夹住”。

/spmpc/debug/stage0_reference
  s0,ref_x,ref_y,ref_yaw,ref_kappa,robot_x,robot_y,robot_yaw,yaw_error,contour_error,lag_error

  目的：看 stage0 处 robot 与 OCP reference 的 yaw/contour/lag 误差，判断 front-clamped 解是否在局部代价里看起来仍可接受。

/spmpc/debug/local_traj_head
  valid0,x0,y0,yaw0,v0,omega0,s0,proj_s0,proj_distance0,proj_signed_distance0,contour0,lag0,yaw_error0,
  valid1,...,
  valid2,...

  目的：看求解后的局部轨迹前 3 拍是否也投影回 path front，还是已经规划出向前沿路径推进的分支。

/spmpc/debug/warm_start_head
  valid0,state_s0,state_omega0,control_alpha0,control_vs0,
  valid1,state_s1,state_omega1,control_alpha1,control_vs1,
  valid2,state_s2,state_omega2,control_alpha2,control_vs2

  目的：看实际选中的 warm-start 前 3 拍是否已经带有近零 v_s、高 omega 或大 alpha，从而诱导 solver 进入 near-start low-progress basin。
```

下一次需要采集的证据：

```text
1. 同路径 alpha8 fresh-sim replay，至少包含现有 /spmpc/debug/runtime_bounds、/spmpc/debug/first_shot_summary，以及以上 4 个新 topic；
2. 重点截取 0~6s：尤其是 0.35s、1s、3s、5s、6s；
3. 对比 PASS 与 FAIL：raw_s/guarded_s 是否都在 path front、monotonic_clip_applied 是否触发、stage0 yaw/contour/lag 是否变大、local_traj_head 的 proj_s 是否回到 0、warm_start_head 的 v_s/omega/alpha 是否异常。
```

请别人帮忙时可以直接问：

```text
continuous_mpcc_acados alpha-state 现在 alpha_max=8.0 已确认进入 runtime bounds，3 次同路径 Gate 1 为 2 PASS / 1 FAIL。
失败 run2 在 0~6s 内 progress_s 卡在约 0.004，外部独立投影和局部轨迹都回到 path front，cmd_v 约 5s 后塌零。
新增 debug topic 将 planner 内部 raw/guarded projector、stage0 yaw/contour/lag、local trajectory 前 3 拍投影、warm-start 前 3 拍 omega/alpha/v_s 都录出来。
请判断主因更像：
1. projector/front-clamp 或 min-progress 逻辑；
2. stage0 yaw/contour/lag 局部 basin；
3. warm-start head seed 诱导；
4. OCP objective 对 progress 的激励不足；
5. 上述几项的耦合。
在没有进一步证据前，不建议直接调权重或进入正式实物。
```

本轮非仿真验证已完成：

```text
catkin_make --force-cmake -DCATKIN_WHITELIST_PACKAGES="spmpc_local_planner" --pkg spmpc_local_planner        通过
catkin_make run_tests_spmpc_local_planner_gtest_test_diff_drive_flatness_warm_start --force-cmake -DCATKIN_WHITELIST_PACKAGES="spmpc_local_planner"        5/5 通过
catkin_make --force-cmake -DCATKIN_WHITELIST_PACKAGES="scout_local_planner" --pkg scout_local_planner        通过
catkin_make --force-cmake -DCATKIN_WHITELIST_PACKAGES="spmpc_local_planner" --pkg spmpc_local_planner        已恢复并通过
```

注意：这只是诊断可观测性增强，还没有新的 fresh-sim replay 证据。正式实物仍保持 **NO-GO**。

### 3.11 debug topic 首次 replay 结果：INVALID_ALPHA8，不作为 Gate 1 证据

2026-06-10 已尝试跑 1 轮带新增 debug topic 的 alpha8 同路径 fresh-sim replay。该 run 完成了安全启动/关闭流程，也成功把新增 topic 录进 bag，但不能作为有效 start-to-end Gate 1 样本。

run 目录：

```text
/data/a/scout_spmpc_experiments/raw/2026-06-10/08_gate1_alpha8_debug_topics_replay/20260610_133051_spmpc_B0_P2_alpha_B0_gate1_alpha8_debug_topics_run1
```

bag / 报告：

```text
/data/a/scout_spmpc_experiments/raw/2026-06-10/08_gate1_alpha8_debug_topics_replay/20260610_133051_spmpc_B0_P2_alpha_B0_gate1_alpha8_debug_topics_run1/20260610_133051_spmpc_B0_P2_alpha_B0_gate1_alpha8_debug_topics_run1.bag
/data/a/scout_spmpc_experiments/raw/2026-06-10/08_gate1_alpha8_debug_topics_replay/20260610_133051_spmpc_B0_P2_alpha_B0_gate1_alpha8_debug_topics_run1/20260610_133051_spmpc_B0_P2_alpha_B0_gate1_alpha8_debug_topics_run1_analysis.md
/data/a/scout_spmpc_experiments/raw/2026-06-10/08_gate1_alpha8_debug_topics_replay/20260610_133051_spmpc_B0_P2_alpha_B0_gate1_alpha8_debug_topics_run1/20260610_133051_spmpc_B0_P2_alpha_B0_gate1_alpha8_debug_topics_run1_run_summary.md
```

结果：

```text
verdict = INVALID_ALPHA8
first GOAL_REACHED = INITIALIZED 后约 0.15s
progress_s start/end/max = 0.984452 / 0.995755 / 0.995755
```

判 invalid 的原因：

```text
1. planner log 参数里 robot/alpha_max = 8.0；
2. 但 bag 内 /spmpc/debug/runtime_bounds 与 /spmpc/debug/generated_bounds 全程为全零数组，不能核对 runtime_bounds.alpha_max≈8.0；
3. first odom 已经在路径终点附近：距路径起点约 7.1786m，距路径终点约 0.2409m；
4. 因此该 run 几乎立刻 GOAL_REACHED，不是从路径起点到终点的有效 Gate 1 replay。
```

新增 4 个 topic 的录包计数：

```text
/spmpc/debug/projector          2061
/spmpc/debug/stage0_reference   2061
/spmpc/debug/local_traj_head    2061
/spmpc/debug/warm_start_head    2061
```

但这些 topic 在本次 bag 中 payload 全程为 0。当前解释是：由于 robot 起始时已经接近路径终点，planner 很快进入 reached/非正常求解路径，导致无法用这些 payload 分析 0~6s near-start lock 机理。

该 run 的意义仅限于：

```text
1. 新增 topic 名能进入 rosbag record；
2. 本次不是有效 alpha8 Gate 1 PASS；
3. 不改变正式实物 NO-GO；
4. 下一次若继续跑，应先确保 fresh sim 起始 odom/路径起点一致，且 runtime_bounds topic 有非零有效样本后，才分析 debug payload。
```

### 3.12 起点/runtime_bounds 双 precheck replay：precheck 通过，bag 到点，但计时口径异常

2026-06-10 随后又跑 1 轮带硬性 precheck 的 alpha8 同路径 fresh-sim replay。该轮先确认起点和 runtime bounds，再进入正式 replay。

run 目录：

```text
/data/a/scout_spmpc_experiments/raw/2026-06-10/09_gate1_alpha8_debug_topics_checked_replay/20260610_134504_spmpc_B0_P2_alpha_B0_gate1_alpha8_debug_checked_run1
```

bag / 报告：

```text
/data/a/scout_spmpc_experiments/raw/2026-06-10/09_gate1_alpha8_debug_topics_checked_replay/20260610_134504_spmpc_B0_P2_alpha_B0_gate1_alpha8_debug_checked_run1/20260610_134504_spmpc_B0_P2_alpha_B0_gate1_alpha8_debug_checked_run1.bag
/data/a/scout_spmpc_experiments/raw/2026-06-10/09_gate1_alpha8_debug_topics_checked_replay/20260610_134504_spmpc_B0_P2_alpha_B0_gate1_alpha8_debug_checked_run1/20260610_134504_spmpc_B0_P2_alpha_B0_gate1_alpha8_debug_checked_run1_analysis.md
/data/a/scout_spmpc_experiments/raw/2026-06-10/09_gate1_alpha8_debug_topics_checked_replay/20260610_134504_spmpc_B0_P2_alpha_B0_gate1_alpha8_debug_checked_run1/20260610_134504_spmpc_B0_P2_alpha_B0_gate1_alpha8_debug_checked_run1_run_summary.md
```

precheck 结果：

```text
precheck_start = PASS
  odom 到 path 起点 = 0.0320m
  odom 到 path 终点 = 7.4113m

precheck_bounds = PASS
  runtime_bounds.alpha_max = 8.0
  generated_bounds.alpha_max = 1.2000000476837158
  runtime_bounds 非全零
```

bag 内结果：

```text
progress_s start/end/max = 0.000367 / 0.981806 / 0.981806
first GOAL_REACHED ≈ first progress sample 后 38.10s
status counts = WAITING_FOR_ODOM=1, B0_ACADOS_OK=1017, ACADOS_SOLVE_FAILED_4=129, GOAL_REACHED=903
terminal mode counts = TRACKING=1006, TERMINAL_SLOWDOWN=39, TERMINAL_CAPTURE_STOP=98, REACHED=903
```

新增 4 个 debug topic 均有效录包，且 payload 非零：

```text
/spmpc/debug/projector          2046, payload nonzero = yes
/spmpc/debug/stage0_reference   2046, payload nonzero = yes
/spmpc/debug/local_traj_head    2046, payload nonzero = yes
/spmpc/debug/warm_start_head    2046, payload nonzero = yes
```

runtime/generated bounds 样本：

```text
runtime  = [-0.6, 0.6, -8.0, 8.0, 0.0, 0.8, 0.0, 0.8, -1.2, 1.2]
generated = [-0.6, 0.6, -1.2, 1.2, 0.0, 0.8, 0.0, 0.8, -1.2, 1.2]
```

注意事项：

```text
1. 这次没有复现 run2 near-start lock；约 5s 时 progress_s≈0.03497，之后能继续推进并到点。
2. 但 subagent 开始 formal 计时时，/spmpc/terminal/mode 已经是 REACHED，progress_s≈0.9810。
3. 因此该样本不应包装成严格有效的 Gate 1 PASS；它更适合作为“precheck 通过 + alpha8 能到点 + 新 debug payload 有效”的证据。
4. 后续若要正式计数，需要把 precheck 与 formal 计时边界收紧，避免 formal timer 启动时已经 REACHED。
```

该 run 不改变正式实物结论：alpha8 到点能力进一步得到支持，但主线仍未形成严格稳定的 fresh-sim Gate 1 统计，正式实物仍保持 **NO-GO**。

### 3.13 warm-start `v_s = Δs/dt` 外科修正：只修进度通道一致性，不放行实物

请别人看完 `3.10~3.12` 后给出的一个具体可执行建议是：先修 warm-start 中 `v_s` 的种子来源。这个建议有道理，因为 alpha-state OCP 的进度动态是：

```text
s_dot = v_s
```

但之前 flatness warm-start 和 conservative fallback 都把 `control.v_s` 从物理车速 `state.v` 派生。这样在起步 `robot.v=0` 时，可能出现 `states[k+1].s > states[k].s`，但 `control.v_s=0` 的不一致种子。该不一致不会单独证明 run2 near-start lock 的根因，但它是明确的代码级问题，应该先修。

本轮外科修改：

```text
1. DiffDriveFlatnessWarmStart::generate：control.v_s 改为 (states[k+1].s - states[k].s) / dt，再走原有 v_s clamp / bound_violation_count；
2. continuous_mpcc_acados conservative fallback：所有 states 生成后，用相邻 state.s 的 Δs/dt 回填 controls[k].v_s，并保持 [0, params.v_max] clamp；
3. 扩展 DiffDriveFlatnessWarmStart gtest：在 robot.v=0、v_s_max=0.12 的场景下，逐拍断言 control.v_s 等于 clamped Δs/dt。
```

已通过的非仿真验证：

```text
catkin_make --force-cmake -DCATKIN_WHITELIST_PACKAGES="spmpc_local_planner" --pkg spmpc_local_planner        通过
catkin_make run_tests_spmpc_local_planner_gtest_test_diff_drive_flatness_warm_start --force-cmake -DCATKIN_WHITELIST_PACKAGES="spmpc_local_planner"        5/5 通过
catkin_make --force-cmake -DCATKIN_WHITELIST_PACKAGES="scout_local_planner" --pkg scout_local_planner        通过
catkin_make --force-cmake -DCATKIN_WHITELIST_PACKAGES="spmpc_local_planner" --pkg spmpc_local_planner        已恢复并通过
git diff --check        通过
```

注意边界：

```text
1. 本修正只让 warm-start / fallback 的 path-progress control 与自身 state seed 一致；
2. 没有改 objective、权重、projector、terminal、cmd safety、acados codegen 或任何 topic layout；
3. 没有添加 warm-start residual 字段，避免改 /spmpc/debug/warm_start 既有 layout；
4. 仍需补跑严格 fresh-sim Gate 1 才能判断它是否改善 near-start low-progress basin；
5. 正式实物结论仍是 NO-GO。
```

### 3.14 `v_s=Δs/dt` 后严格 fresh-sim Gate 1 replay：FAIL，near-start lock 复现

2026-06-10 按安全 SOP 调用 subagent 跑了 1 轮 `continuous_mpcc_acados + B0 + alpha_max=8.0` 严格 fresh-sim Gate 1 replay。该轮在 formal 计时前做了起点、runtime bounds、timer boundary 三个 precheck，避免把“已接近终点 / 已 REACHED”的样本包装成 PASS。

run 目录：

```text
/data/a/scout_spmpc_experiments/raw/2026-06-10/10_gate1_alpha8_after_vs_dt_fix_strict_replay/20260610_145721_spmpc_B0_alpha8_gate1_strict_replay
```

bag / analysis：

```text
/data/a/scout_spmpc_experiments/raw/2026-06-10/10_gate1_alpha8_after_vs_dt_fix_strict_replay/20260610_145721_spmpc_B0_alpha8_gate1_strict_replay/20260610_145721_spmpc_B0_alpha8_gate1_strict_replay.bag
/data/a/scout_spmpc_experiments/raw/2026-06-10/10_gate1_alpha8_after_vs_dt_fix_strict_replay/20260610_145721_spmpc_B0_alpha8_gate1_strict_replay/analysis/analysis.md
/data/a/scout_spmpc_experiments/raw/2026-06-10/10_gate1_alpha8_after_vs_dt_fix_strict_replay/20260610_145721_spmpc_B0_alpha8_gate1_strict_replay/analysis/run_summary.md
/data/a/scout_spmpc_experiments/raw/2026-06-10/10_gate1_alpha8_after_vs_dt_fix_strict_replay/20260610_145721_spmpc_B0_alpha8_gate1_strict_replay/analysis/run_summary.json
```

precheck 结果：

```text
precheck_start = PASS
  odom 到 path 起点 = 0.091m
  odom 到 path 终点 = 7.389m

precheck_bounds = PASS
  runtime_bounds.alpha_max = 8.0
  generated_bounds.alpha_max = 1.2

precheck_timer_boundary = PASS
  terminal = TRACKING
  progress_s = 0.006390
```

严格 Gate 1 结果：

```text
verdict = FAIL
strict timer start = precheck 后第一条 terminal != REACHED 且 progress_s <= 0.1 的有效样本
strict timer actual trigger = terminal topic
first GOAL_REACHED / terminal REACHED = 无
strict timeout = 70.012s
progress_s start/end/max = 0.006002 / 0.006390 / 0.006477
status_counts = WAITING_FOR_ODOM:1, B0_ACADOS_OK:7097, ACADOS_SOLVE_FAILED_4:38
terminal_mode_counts = TRACKING:7134
near_start_lock_reproduced = yes
```

新增 4 个 debug topic 均有效录包且 payload 非零：

```text
/spmpc/debug/projector          7135 msgs, nonzero 7135
/spmpc/debug/stage0_reference   7135 msgs, nonzero 7135
/spmpc/debug/local_traj_head    7135 msgs, nonzero 7097
/spmpc/debug/warm_start_head    7135 msgs, nonzero 7135
```

结论：

```text
1. `v_s=Δs/dt` 修正后，strict alpha8 fresh-sim Gate 1 仍 FAIL；
2. alpha8 runtime bounds 已有效进入运行口径，但 robot 仍在 70s 内卡在 progress_s≈0.006 附近；
3. near-start low-progress lock 已在严格 precheck / 严格 timer 口径下复现；
4. 因此问题不能再归结为“起点无效、runtime alpha8 无效、计时边界异常、或 warm-start v_s 单点 bug”；
5. 下一步应基于本轮有效 debug topics 做 0~6s / 全程离线机理诊断，再决定 projector / objective / recovery / safety gate 的改法；
6. 正式实物结论继续 NO-GO。
```

### 3.15 strict replay 离线机理诊断：warm-start 已前进，但 OCP 解与 projector 仍锁在 path front

对 `3.14` 的有效 FAIL bag 做了只读离线诊断，未启动仿真、未发 `/cmd_vel`、未改代码。诊断输出：

```text
/data/a/scout_spmpc_experiments/raw/2026-06-10/10_gate1_alpha8_after_vs_dt_fix_strict_replay/20260610_145721_spmpc_B0_alpha8_gate1_strict_replay/analysis/near_start_lock_diagnostics/parse_near_start_lock.py
/data/a/scout_spmpc_experiments/raw/2026-06-10/10_gate1_alpha8_after_vs_dt_fix_strict_replay/20260610_145721_spmpc_B0_alpha8_gate1_strict_replay/analysis/near_start_lock_diagnostics/key_time_samples.csv
/data/a/scout_spmpc_experiments/raw/2026-06-10/10_gate1_alpha8_after_vs_dt_fix_strict_replay/20260610_145721_spmpc_B0_alpha8_gate1_strict_replay/analysis/near_start_lock_diagnostics/near_start_lock_summary.json
/data/a/scout_spmpc_experiments/raw/2026-06-10/10_gate1_alpha8_after_vs_dt_fix_strict_replay/20260610_145721_spmpc_B0_alpha8_gate1_strict_replay/analysis/near_start_lock_diagnostics/near_start_lock_diagnostics.md
```

核心证据：

```text
1. progress_s 在 strict 70s 窗口内完全锁住：start/max/end = 0.006390 / 0.006390 / 0.006390；
2. warm_start_head 已不是旧 bug 的零 v_s：前 3 拍 control_vs = 0.56 / 0.56 / 0.56，state_s = 0.054748 / 0.073414 / 0.092081；
3. 但 first_shot / OCP 解的 u0_vs 仍为 0.0，说明非零 warm-start 没有转化为非零求解输出；
4. projector raw_s 全程 = 0.0，guarded_s 全程 = 0.05474757，monotonic_clip_applied 全程触发（2100/2100）；
5. local_traj_head 前 3 个点 solver s 全程都停在 0.05474757，proj_s 全程都投回 0.0；
6. 因此不是“solver s 已前进但几何投影回 path front”，而是 solved s 和几何投影都被困在前端；
7. cmd_v 各窗口基本为 0（约 1e-10 量级），cmd_omega 小幅非零（约 0.0025~0.0048 rad/s），表现为轻微原地 heading 活动但无平移；
8. stage0 长期处于不佳起点 basin：yaw_error≈0.984~0.990rad，contour_error≈-0.080~-0.074m，lag_error≈0.152~0.158m。
```

当前可排除：

```text
1. 无效起点；
2. alpha8 runtime bounds 未进入；
3. formal timer 已 REACHED 的计时异常；
4. debug payload 缺失；
5. 旧 warm-start v_s=state.v bug 作为唯一根因。
```

更可能的排序假设：

```text
1. projector / path-front geometry lock：raw 和 guarded projection 都被困在 path front 小邻域；
2. stage0 yaw/contour/lag start basin：优化器更偏向原地 heading 修正或静止，而不是主动推进 s；
3. progress objective 激励不足或耦合不够：warm-start 有非零 v_s，但求解输出仍把 u0_vs 压到 0；
4. alpha-state omega/heading dynamics：角速度/heading 修正通道存在，但平移推进被抑制；
5. 缺少 start-lock recovery / safety 行为：在 tracking 名义正常但 progress 长期低于阈值时没有显式恢复或安全退出。
```

下一步建议先做诊断/设计，不要直接进实物：

```text
1. 追加或离线提取 projector 最近段 index / front clamp 决策，确认 raw_s=0 的几何原因；
2. 关联 cost 中 progress 项、first_shot u0_vs、solved x1/x2/x3_s，判断优化器是否主动拒绝向前推进；
3. 设计 start-basin escape / recovery gate，例如 progress_s 长期低于阈值且 cmd_v≈0 时进入受控恢复或 hard-zero 安全状态；
4. 如要改 objective/projector，应先出小范围方案并保留严格 fresh-sim Gate 1 回归。
```

正式实物结论继续 **NO-GO**。

### 3.16 StartLockRecovery detector-only：只做锁死显式观测，不做命令恢复

基于 `3.15` 的 near-start lock 证据，本轮增加了 `StartLockRecovery` 的 detector-only 观测层。它的定位是工程诊断 / 后续 recovery gate 的前置观测，不是 OCP 理论修复，也不是实物放行依据。

实现边界：

```text
1. 默认关闭：start_lock_recovery.enable=false；
2. detect_only=true：只计算 active/mode/debug，不覆盖 /cmd_vel；
3. 使用 progress_abs_s（绝对米制 s）做阈值，不使用归一化 progress_s；
4. 不伪造 progress，不改变 projector/front clamp，不改 objective/weight，不改 acados codegen；
5. 不改任何已有 topic layout，只新增独立 topic。
```

新增 topic：

```text
/spmpc/start_lock/active
/spmpc/start_lock/mode
/spmpc/start_lock/debug
```

`/spmpc/start_lock/debug` 固定字段：

```text
enabled,detect_only,active,near_start,stall_progress,cmd_suppressed,
warmstart_requests_motion,solver_rejects_progress,monotonic_clip_active,
projection_distance_unsafe,stall_time_sec,active_count,progress_abs_s,
progress_delta_s,projector_raw_s,projector_guarded_s,guard_minus_raw_s,
projector_distance,cmd_v,robot_v,warm_start_v_s0,first_shot_u0_vs
```

触发语义：只有在近起点绝对进度长时间不增长、`cmd_v` 近零、warm-start 首拍 `v_s` 明确要求前进、first-shot `u0_v_s` 近零、且按配置要求存在 monotonic clip 时，才进入 `ACTIVE_START_LOCK`。投影距离超过 `max_projection_distance_m` 时只报 `UNSAFE_PROJECTION_DISTANCE`，不把它包装成普通 start-lock active。

新增单测覆盖：默认关闭不触发、持续 near-start lock 达到 dwell 后触发、progress 恢复后清除、离开 start window 不触发、无 warm-start/solver mismatch 不触发、缺少 monotonic clip 不触发、projection distance unsafe 不触发普通 active、goal reached 不触发。

注意：该 detector 只是把 `3.15` 中“warm-start 要前进但 OCP/投影仍锁死”的条件显式化。正式实物仍保持 **NO-GO**；下一步如果要做 recovery/override，必须另起补丁并先补 hard-zero / path-departure safety。

## 4. 问题二：RouteB 成功只能作 diagnostic，不能放行主线

RouteB 成功记录：

```text
docs/实物实验注意事项/实物实验前的仿真/20260609_RouteB_B0成功到点与入弯高晃动基线记录.md
```

RouteB 使用：

```text
backend = continuous_mpcc_direct_omega_legacy
x = [px, py, theta, v, s]
u = [a, omega, v_s]
```

alpha-state 主线使用：

```text
backend = continuous_mpcc_acados
x = [px, py, theta, v, s, omega]
u = [a, alpha, v_s]
alpha = d(omega)/dt
```

RouteB 同路径到点说明：

```text
1. 仿真环境不是主要问题；
2. fixed path replay 链路不是主要问题；
3. 这条 path 对 direct-omega continuous MPCC 是可走的。
```

但不能说明：

```text
continuous_mpcc_acados alpha-state 主线已经安全可实物。
```

## 5. 问题三：warm-start 修正已完成，但只解决了一个必要条件

### 5.1 已完成修正

本轮外科修正内容：

```text
1. DiffDriveFlatnessWarmStart 从 input.robot.omega 开始生成 state.omega；
2. 生成 control.alpha，使 states[k+1].omega ≈ states[k].omega + controls[k].alpha * dt；
3. params.alpha_max 传入 WarmStartBounds::omega_rate_max；
4. slosh rollout、finite check、max_omega、横向加速度诊断改用 state.omega；
5. control.omega 仅作为 legacy/debug mirror；
6. u_prev_ 注释与语义澄清为 [a, alpha, v_s]。
```

已通过验证：

```bash
catkin_make --pkg spmpc_local_planner
catkin_make run_tests_spmpc_local_planner_gtest_test_diff_drive_flatness_warm_start
```

结果：

```text
spmpc_local_planner 构建通过；
DiffDriveFlatnessWarmStart gtest 4/4 通过。
```

### 5.2 但 replay 证明修正还不充分

warm-start 修正后的 replay 中：

```text
/spmpc/debug/warm_start_status = OK 全程
/spmpc/debug/warm_start used_flatness = 1
/spmpc/debug/warm_start max_omega = 1.2
```

这说明修正后的 warm-start 管线进入运行，但闭环仍失败。因此当前主要嫌疑应从“warm-start 字段没有进入 acados”升级为：

```text
alpha-state OCP / objective / projection / closed-loop command 的交互问题。
```

实物前应继续诊断，而不是直接调 slosh 权重或进入实物。

## 6. 问题四：alpha-state 仍存在的技术风险

### 6.1 stage-0 yaw-rate 能力与 command 积分口径

alpha-state 主线下发命令的核心口径是：

```text
cmd_omega = input.robot.omega + u0[1] * dt
u0[1] = alpha
```

如果起步高曲率段需要较大 `omega≈v*kappa`，而当前实测 `omega` 接近 0，则第一拍只能通过 `alpha * dt` 改变 yaw-rate。即使 `alpha_max=8.0`，这条 replay 仍失败，说明问题不只是上限大小，还可能包括 objective / contour / lag / progress / local trajectory 的局部解形态。

### 6.2 path departure 后 progress projection 锁定

warm-start 修正后的 replay 出现：

```text
odom 已经移动 5.67 m；
progress_s 只到 0.19246；
cross-track sampled max ≈ 5.21 m。
```

这说明失败时不是单纯停在原地，而是偏离 reference 后 progress projector 锁在前段。实物中这会对应更高风险的“跑偏但 planner 仍认为未到点”。

### 6.3 reference-snapped pose 与 alpha-rate-limited omega 可能不一致

alpha-state OCP 模型：

```text
px_dot    = v cos(theta)
py_dot    = v sin(theta)
theta_dot = omega
v_dot     = a
s_dot     = v_s
omega_dot = alpha
```

当前 flatness warm-start 中，`px / py / theta` 仍主要贴 reference spline，而 `omega / alpha` 按 rate limit 递推。当 alpha 饱和时，可能出现：

```text
theta 已经贴到参考弯道方向，omega 却因为 alpha 上限还没爬上来。
```

这会让 warm-start seed 在动态上不完全满足 `theta_dot = omega`。

### 6.4 fallback / previous-solution 仍需检查一致性

实物前应检查或修正：

```text
1. conservative fallback 是否满足 omega_dot = alpha；
2. previous-solution fallback shift 后是否重算 states[1].omega / controls[0].alpha；
3. alpha_max <= 0、NaN、非有限值是否 fail fast；
4. replay/fresh-sim 中是否靠 fallback 掩盖主路径问题。
```

## 7. 问题五：catkin 红线失败是 whitelist/cache 问题，但仍需补验证

### 7.1 现象

项目红线命令：

```bash
catkin_make --pkg scout_local_planner
```

失败核心：

```text
没有规则可制作目标“scout_apps/control/scout_local_planner/all”
```

### 7.2 判断

当前不应把它解释成“本轮改坏了 `scout_local_planner` 源码”。已知原因是 build cache / whitelist：

```text
build/CMakeCache.txt:
CATKIN_WHITELIST_PACKAGES:STRING=spmpc_local_planner

build/catkin_generated/order_packages.cmake:
当前 ordered package 只有 spmpc_local_planner
```

源码包仍存在：

```text
src/scout_apps/control/scout_local_planner/package.xml
package name = scout_local_planner
```

更准确的记录方式：

```text
scout_local_planner 红线未完成；当前失败原因是 build graph 被 spmpc_local_planner 白名单限制，导致 scout_local_planner/all 目标未生成。
```

### 7.3 建议命令

验证 `scout_local_planner` 本身：

```bash
catkin_make --force-cmake -DCATKIN_WHITELIST_PACKAGES=scout_local_planner --pkg scout_local_planner
```

恢复全工作区构建图：

```bash
catkin_make --force-cmake -DCATKIN_WHITELIST_PACKAGES=""
```

回到只编译 SPMPC：

```bash
catkin_make --force-cmake -DCATKIN_WHITELIST_PACKAGES=spmpc_local_planner --pkg spmpc_local_planner
```

注意：即使 `scout_local_planner` 重新验证通过，也不能改变 alpha-state 主线 replay 未通过的 No-Go 结论。执行 `scout_local_planner` 红线验证后，若继续开发 SPMPC，需要重新 `force-cmake` 回到 `spmpc_local_planner` whitelist，否则后续 SPMPC 单包构建图也可能被切走。

## 8. 实物前剩余 gate

### 8.1 必须先完成的仿真 gate

当前最短放行链路应为：

```text
Gate 1: continuous_mpcc_acados + B0 + 同一路径 replay 到点
Gate 2: Gate 1 通过后，B0 在 2~3 个独立 fresh-sim run 中到点
Gate 3: Gate 2 通过后，fresh-sim B_slosh / B_ours 到点且无明显跑偏
Gate 4: slosh monitor / RGB / rosbag 记录链路 dry-run 确认
Gate 5: 无液体低速实物 dry-run
Gate 6: 带液体正式实物试验
```

### 8.2 Gate 1 成功标准

同一路径 replay 至少应满足：

```text
1. GOAL_REACHED > 0；
2. progress_s > 0.95，最好到 0.97+；
3. terminal mode 进入 REACHED；
4. 不再冻结在 progress_s≈0.003 / 0.11 / 0.19；
5. 不出现明显 path departure；
6. cmd_v 不长期塌到接近 0；
7. solver_time_ms max < 33 ms；
8. warm_start_status 不是长期 fallback 或异常失败。
```

### 8.3 Gate 2 fresh-sim SOP

必须遵守已有 SOP：

```text
1. fresh Gazebo/RViz；
2. 启动仿真后等待 30s，让定位恢复；
3. 一次只跑一个 case；
4. 只调整 planner / algorithm / config，不改仿真环境；
5. 跑完关闭仿真；
6. 关闭后等待 30s，再开始下一次；
7. 超过约 1 分钟仍未完成按失败处理。
```

## 9. 当前 Go / No-Go 表

| 项目 | 当前状态 | 结论 |
|---|---|---|
| `spmpc_local_planner` 构建 | 通过 | OK |
| warm-start 单测 | 4/4 通过 | OK，但只覆盖局部生成器 |
| `scout_local_planner` 红线 | 因 whitelist/cache 目标缺失失败 | 需用正确 whitelist 重跑 |
| RouteB B0 同路径 replay | 成功到点，`progress_s=0.98043` | 只能作 diagnostic / high-slosh baseline |
| alpha-state B0 默认 replay | 未到点，`progress_s=0.11672` | 阻塞正式实物 |
| alpha-state B0 alpha8 replay | 未到点，`progress_s=0.11467` | 阻塞正式实物 |
| alpha-state B0 warm-start fix + alpha8 replay | 未到点，`progress_s=0.19246`，且跑偏 | 阻塞正式实物 |
| alpha-state B0 subagent Gate 1 fresh-sim replay | 未到点，`progress_s=0.0028689418`，70s 内几乎无有效推进 | 阻塞正式实物 |
| alpha-state B0 fresh-sim | 当前修正后 Gate 1 replay 仍失败，暂不应进入 2~3 个独立 fresh-sim run gate | 阻塞正式实物 |
| B_slosh / B_ours fresh-sim | 未到该阶段 | 阻塞论文口径实物 |
| slosh/RGB 记录链路 | 部分确认 | 带液体前仍需 dry-run |

## 10. 下一步建议

在 Gate 1 通过前，不要继续扫 slosh 权重，也不要进入实物；等 alpha-state B0 replay/fresh-sim gate 过后，再回到 slosh 权重与 B_slosh / B_ours 对比。当前优先做 alpha-state 第一拍与预测轨迹诊断：

```text
1. 导出 x0.omega；
2. 导出 warm-start state.omega / control.alpha；
3. 导出 OCP u0=[a, alpha, v_s]；
4. 导出 cmd_omega=input.robot.omega + alpha*dt；
5. 导出 reference kappa / v*kappa；
6. 导出 contour error / lag error；
7. 导出 nearest-path distance / projection s；
8. 导出 local trajectory 前几拍的位置、朝向、omega。
```

目标是区分当前失败主因：

```text
A. stage-0 yaw-rate 动态能力不足；
B. objective 权重导致错误局部解；
C. progress projection 锁定导致跑偏后无法恢复；
D. local trajectory 初始方向 / theta / omega 不一致；
E. fallback / previous-solution 仍注入不一致初值。
```

## 11. 一句话总结

当前不是“只差实物确认”，而是还卡在实物前最关键的仿真门槛：

```text
continuous_mpcc_acados alpha-state 主线必须先在同一路径 replay 到点，再进入 fresh-sim 连续验证。
```

在这个 gate 通过前，正式实物试验保持 No-Go。