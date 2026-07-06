# 20260705 SPMPC 实物 fixed-path N=3：RGB 真值与 internal model 综合分析

> 数据目录：`/media/a/ZRJ/slosh_bags/20260705_fixed_path_compare`  
> RGB 标定：`/media/a/ZRJ/slosh_bags/20260629_calib/red_3ruler.yaml`  
> 分析脚本与 CSV/JSON 输出：`docs/Claude/分析数据/20260705_fixed_path_compare/`  
> 统计窗口：`first ACADOS_OK -> first GOAL_REACHED`，并额外按 `/spmpc/debug/stage0_reference.s0` 取 path progress `10%~90%`。  
> RGB 主指标：按固定流程使用 `h_mm_max_lcr_smooth_corr`，即 `H_vis(t)=abs(rolling_median(max(h_left,h_center,h_right), window=5))`。

本次只做离线数据分析，没有修改 MPC、GeoRef、OSCRS 或控制链路代码。安全红线检查重点看 `GOAL_REACHED`、`zero_due_to_tracking_safety`、delay compensation、hard constraint 是否符合正式比较设定。

---

## 1. 处理记录与输出文件

本次生成/使用的主要离线分析文件：

```text
docs/Claude/分析数据/20260705_fixed_path_compare/analyze_20260705_fixed_path_n3.py
docs/Claude/分析数据/20260705_fixed_path_compare/20260705_fixed_path_n3_run_metrics.csv
docs/Claude/分析数据/20260705_fixed_path_compare/20260705_fixed_path_n3_cost_component_stats.csv
docs/Claude/分析数据/20260705_fixed_path_compare/20260705_fixed_path_n3_method_aggregate.csv
docs/Claude/分析数据/20260705_fixed_path_compare/20260705_fixed_path_n3_cost_component_aggregate.csv
docs/Claude/分析数据/20260705_fixed_path_compare/20260705_fixed_path_n3_config_audit.csv
docs/Claude/分析数据/20260705_fixed_path_compare/20260705_fixed_path_n3_reductions_vs_B0.csv
docs/Claude/分析数据/20260705_fixed_path_compare/20260705_fixed_path_n3_reductions_vs_B0_long.csv
docs/Claude/分析数据/20260705_fixed_path_compare/20260705_fixed_path_n3_analysis_summary.json
```

RGB 推理输出在：

```text
/media/a/ZRJ/slosh_bags/20260705_fixed_path_compare/red_visual_analysis_20260705/red_infer
```

注意：第一次完整 rosbag 读取时外接盘出现过 USB 断连导致 `Input/output error`。后续外接盘重新挂载后重跑成功；分析脚本只读取到 first `GOAL_REACHED` 后约 `0.5s`，避免读取与本统计无关的 post-goal 后段 chunk，但统计窗口仍严格使用 `first ACADOS_OK -> first GOAL_REACHED`。

---

## 2. 配置、到点与安全复核

### 2.1 9 个 formal bag 均成功到点

| method | runs | GOAL_REACHED |
|---|---:|---:|
| B0 | 3 | 3/3 |
| B_slosh | 3 | 3/3 |
| B_ours | 3 | 3/3 |
| total | 9 | 9/9 |

### 2.2 effective_config 全部符合正式比较设定

`20260705_fixed_path_n3_config_audit.csv` 复核结果：`all_requested_config_ok = 1` for `9/9`。

| item | target | result |
|---|---:|---:|
| `V_REF` / `v_ref` | `0.20` | 9/9 OK |
| `alpha_max` | `1.2` | 9/9 OK |
| `shared_linear_accel_max` | `0.6` | 9/9 OK |
| `shared_angular_rate_max` | `1.2` | 9/9 OK |
| `shared_angular_accel_max` | `1.2` | 9/9 OK |
| `delay_phase_mode_code` | `3.0` = `fixed_closed_loop` | 9/9 OK |
| `delay_linear_sec` | `0.15` | 9/9 OK |
| `delay_angular_sec` | `0.22` | 9/9 OK |
| `slosh_constraint_enable` | `0` | 9/9 OK |

RGB 尺寸为 `1920x1080`，符合固定流程的 paper-level RGB truth 输入要求。

### 2.3 delay compensation 与 command safety

| window | method | delay applied frac | published zero frac | tracking safety zero frac | linear limited frac | angular rate limited frac | angular accel limited frac |
|---|---|---:|---:|---:|---:|---:|---:|
| full | B0 | `1.000±0.000` | `0.000±0.000` | `0.000±0.000` | `0.0148±0.0034` | `0.0000±0.0000` | `0.0003±0.0006` |
| full | B_slosh | `1.000±0.000` | `0.000±0.000` | `0.000±0.000` | `0.0076±0.0021` | `0.0000±0.0000` | `0.0015±0.0014` |
| full | B_ours | `1.000±0.000` | `0.000±0.000` | `0.000±0.000` | `0.0096±0.0017` | `0.0000±0.0000` | `0.0003±0.0005` |
| path 10%~90% | B0 | `1.000±0.000` | `0.000±0.000` | `0.000±0.000` | `0.0064±0.0016` | `0.0000±0.0000` | `0.0000±0.0000` |
| path 10%~90% | B_slosh | `1.000±0.000` | `0.000±0.000` | `0.000±0.000` | `0.0029±0.0014` | `0.0000±0.0000` | `0.0008±0.0014` |
| path 10%~90% | B_ours | `1.000±0.000` | `0.000±0.000` | `0.000±0.000` | `0.0101±0.0024` | `0.0000±0.0000` | `0.0004±0.0007` |

结论：

```text
1. /spmpc/debug/delay_phase、/spmpc/debug/predicted_state、/spmpc/debug/solver_input_state 均有样本；
2. delay_compensation_applied_frac = 1.0；
3. fixed closed-loop 0.15 / 0.22 确实进入 solver input；
4. zero_due_to_tracking_safety = 0，published_zero_frac = 0；
5. 本组正式比较没有触发 tracking safety 停车。
```

---

## 3. N=3 主指标：full window

| method | N/success | goal time from ACADOS | model p95 | model max | model RMS | RGB `H_vis` p95 | RGB `H_vis` max | RGB `H_vis` RMS | projection p95 | projection max | `|omega|` p95 | `|omega|` max |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| B0 | 3/3 | `33.753±0.355s` | `1.259±0.013mm` | `3.657±1.391mm` | `0.619±0.036mm` | `0.943±0.105mm` | `2.398±0.482mm` | `0.512±0.056mm` | `2.829±0.109cm` | `4.584±0.581cm` | `0.256±0.012rad/s` | `0.450±0.015rad/s` |
| B_slosh | 3/3 | `36.851±0.231s` | `0.696±0.071mm` | `1.564±0.287mm` | `0.343±0.024mm` | `0.828±0.000mm` | `2.162±0.405mm` | `0.488±0.008mm` | `3.279±0.207cm` | `6.992±1.910cm` | `0.267±0.014rad/s` | `0.448±0.005rad/s` |
| B_ours | 3/3 | `38.073±0.403s` | `0.902±0.011mm` | `1.950±0.414mm` | `0.443±0.007mm` | `0.690±0.138mm` | `1.715±0.389mm` | `0.372±0.051mm` | `3.333±0.168cm` | `5.908±1.120cm` | `0.213±0.003rad/s` | `0.269±0.002rad/s` |

full window 观察：

- internal model：`B_slosh < B_ours < B0`，`B_slosh` 抑制最强。
- RGB 主指标：`B_ours` 的 `H_vis` p95/max/RMS 最低，说明实物视觉侧的液面峰值在当前 N=3 中更偏向支持 `B_ours` 的平滑控制收益。
- `B_ours` 的 `|omega|` p95/max 明显最低，符合“更平滑”的设计目标。

---

## 4. N=3 主指标：path progress 10%~90%

| method | N/success | goal time from ACADOS | model p95 | model max | model RMS | RGB `H_vis` p95 | RGB `H_vis` max | RGB `H_vis` RMS | projection p95 | projection max | `|omega|` p95 | `|omega|` max |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| B0 | 3/3 | `33.753±0.355s` | `1.100±0.034mm` | `2.102±0.222mm` | `0.547±0.016mm` | `1.009±0.214mm` | `2.398±0.482mm` | `0.541±0.081mm` | `2.947±0.087cm` | `4.115±0.166cm` | `0.234±0.015rad/s` | `0.348±0.007rad/s` |
| B_slosh | 3/3 | `36.851±0.231s` | `0.632±0.101mm` | `1.255±0.039mm` | `0.307±0.035mm` | `0.901±0.015mm` | `2.162±0.405mm` | `0.537±0.008mm` | `3.442±0.128cm` | `6.992±1.910cm` | `0.242±0.033rad/s` | `0.341±0.004rad/s` |
| B_ours | 3/3 | `38.073±0.403s` | `0.892±0.076mm` | `1.950±0.414mm` | `0.432±0.018mm` | `0.709±0.098mm` | `1.600±0.486mm` | `0.384±0.032mm` | `3.527±0.204cm` | `5.908±1.120cm` | `0.207±0.007rad/s` | `0.269±0.002rad/s` |

10%~90% 主窗口结论：

```text
internal model: B_slosh 最低，B_ours 次之，B0 最高。
RGB H_vis: B_ours 最低，B_slosh 相比 B0 有轻度下降，B_ours 相比 B0 有明显下降。
command smoothness: B_ours 的 |omega| p95/max 最低。
tracking: 三者 projection p95 均在约 3cm 量级；B_slosh/B_ours 为降低晃动和平滑付出更长到点时间和略高 projection 的代价。
```

---

## 5. 相对 B0 降幅

### 5.1 full window

| method | model p95 | model max | model RMS | RGB p95 | RGB max | RGB RMS |
|---|---:|---:|---:|---:|---:|---:|
| B_slosh vs B0 | `44.7%` | `57.2%` | `44.6%` | `12.2%` | `9.9%` | `4.7%` |
| B_ours vs B0 | `28.4%` | `46.7%` | `28.6%` | `26.8%` | `28.5%` | `27.4%` |

### 5.2 path progress 10%~90%

| method | model p95 | model max | model RMS | RGB p95 | RGB max | RGB RMS |
|---|---:|---:|---:|---:|---:|---:|
| B_slosh vs B0 | `42.5%` | `40.3%` | `43.9%` | `10.7%` | `9.9%` | `0.6%` |
| B_ours vs B0 | `18.9%` | `7.2%` | `21.0%` | `29.8%` | `33.3%` | `29.0%` |

解释：

- 若以 `/spmpc/slosh_height` internal model 为准，`B_slosh` 是本次最强抑晃方法；这和它把优化目标集中在 slosh soft cost 上一致。
- 若以 RGB `H_vis` 作为实物视觉主指标，本次 N=3 中 `B_ours` 的 p95/max/RMS 最低；这说明更平滑的 `omega` 链路对真实液面峰值也有明显收益。
- 两个证据不完全同序，但不是互相否定：internal model 更直接反映控制器内部 observer/预测模型；RGB 反映相机 ROI 内真实红色液面包络。当前应表述为：`B_slosh` 模型侧抑晃最强，`B_ours` 控制更平滑且 RGB 真值最优。

---

## 6. cost share：full window

单位均为百分比。`progress_abs` 使用 `abs(pct_progress)`，因为 progress 是负奖励。

| method | component | p50 | p75 | p95 | max | mean |
|---|---|---:|---:|---:|---:|---:|
| B0 | progress_abs | `67.475` | `71.868` | `82.152` | `94.576` | `64.600` |
| B0 | contour | `5.214` | `9.539` | `18.029` | `33.759` | `6.464` |
| B0 | v | `23.442` | `25.830` | `49.205` | `58.390` | `24.008` |
| B0 | control | `4.958` | `7.362` | `9.512` | `21.930` | `4.816` |
| B0 | smooth | `0.000` | `0.001` | `0.020` | `0.184` | `0.004` |
| B0 | slosh | `0.000` | `0.000` | `0.000` | `0.000` | `0.000` |
| B_slosh | progress_abs | `66.496` | `72.882` | `79.113` | `89.376` | `63.377` |
| B_slosh | contour | `4.651` | `9.681` | `21.407` | `44.988` | `6.727` |
| B_slosh | v | `21.867` | `24.627` | `45.848` | `56.109` | `21.752` |
| B_slosh | control | `4.143` | `6.730` | `9.074` | `15.883` | `4.401` |
| B_slosh | smooth | `0.024` | `0.078` | `0.323` | `1.955` | `0.077` |
| B_slosh | slosh | `1.713` | `3.814` | `12.492` | `37.734` | `3.577` |
| B_ours | progress_abs | `60.988` | `68.826` | `78.817` | `88.898` | `60.112` |
| B_ours | contour | `6.194` | `11.919` | `19.796` | `33.263` | `7.895` |
| B_ours | v | `22.299` | `28.719` | `42.929` | `52.578` | `20.955` |
| B_ours | control | `5.063` | `8.174` | `16.634` | `35.423` | `6.291` |
| B_ours | smooth | `0.074` | `0.253` | `1.206` | `21.041` | `0.401` |
| B_ours | slosh | `2.788` | `5.921` | `12.300` | `34.802` | `4.247` |

Slosh 内部 eta / eta-dot share：

| method | eta share p50 | eta share p95 | eta-dot share p50 | eta-dot share p95 |
|---|---:|---:|---:|---:|
| B0 | `0.000` | `0.000` | `0.000` | `0.000` |
| B_slosh | `90.597` | `99.059` | `9.403` | `35.467` |
| B_ours | `87.104` | `98.296` | `12.896` | `32.240` |

---

## 7. cost share：path progress 10%~90%

| method | component | p50 | p75 | p95 | max | mean |
|---|---|---:|---:|---:|---:|---:|
| B0 | progress_abs | `65.197` | `70.681` | `73.381` | `75.934` | `62.119` |
| B0 | contour | `3.643` | `8.706` | `23.345` | `33.759` | `6.099` |
| B0 | v | `24.658` | `26.139` | `49.063` | `53.637` | `27.510` |
| B0 | control | `4.749` | `6.753` | `8.390` | `9.637` | `4.245` |
| B0 | smooth | `0.000` | `0.001` | `0.021` | `0.099` | `0.003` |
| B0 | slosh | `0.000` | `0.000` | `0.000` | `0.000` | `0.000` |
| B_slosh | progress_abs | `66.086` | `72.878` | `77.202` | `78.307` | `62.905` |
| B_slosh | contour | `2.857` | `7.160` | `24.447` | `44.988` | `5.951` |
| B_slosh | v | `22.841` | `25.395` | `45.506` | `49.899` | `25.184` |
| B_slosh | control | `4.379` | `6.809` | `8.408` | `9.379` | `4.105` |
| B_slosh | smooth | `0.022` | `0.068` | `0.259` | `1.273` | `0.063` |
| B_slosh | slosh | `1.272` | `2.140` | `4.794` | `16.015` | `1.741` |
| B_ours | progress_abs | `60.660` | `69.021` | `77.775` | `81.869` | `60.002` |
| B_ours | contour | `4.977` | `9.375` | `22.194` | `33.263` | `6.810` |
| B_ours | v | `23.928` | `30.163` | `43.062` | `48.642` | `24.806` |
| B_ours | control | `4.353` | `6.613` | `11.011` | `19.154` | `4.857` |
| B_ours | smooth | `0.066` | `0.241` | `1.205` | `21.041` | `0.442` |
| B_ours | slosh | `2.148` | `3.966` | `8.968` | `20.926` | `3.032` |

Slosh 内部 eta / eta-dot share：

| method | eta share p50 | eta share p95 | eta-dot share p50 | eta-dot share p95 |
|---|---:|---:|---:|---:|
| B0 | `0.000` | `0.000` | `0.000` | `0.000` |
| B_slosh | `89.120` | `98.914` | `10.880` | `37.529` |
| B_ours | `85.312` | `97.650` | `14.688` | `32.900` |

cost 结论：

```text
1. B0 的 slosh cost 为 0，符合 baseline 定义。
2. B_slosh 和 B_ours 的 slosh soft cost 都实际进入优化器。
3. progress_abs 仍约 60%~66%，说明优化器没有被 slosh cost 完全压倒，因此三组均能到点。
4. B_ours 的 smooth/control 在局部更强，尤其 smooth max 约 21%，解释了 |omega| max 显著降低。
5. B_slosh 更专注 internal slosh 压低；B_ours 同时承担 smooth/control，因此模型侧抑晃不如 B_slosh，但控制更平滑。
```

---

## 8. 与 20260702 旧结果对比

旧结果在 10%~90% path-progress 指标下大致为：

| old group | N | model p95 | model max_10_90 | model RMS | projection p95 |
|---|---:|---:|---:|---:|---:|
| 20260702 B0 | 3 | `1.317mm` | `2.223mm` | `0.680mm` | `3.04cm` |
| 20260702 B_slosh | 1 | `1.805mm` | `3.949mm` | `0.917mm` | - |
| 20260702 B_ours | 1 | `1.855mm` | `3.266mm` | `0.868mm` | - |

本次 20260705 formal N=3 path 10%~90% 为：

| current group | N | model p95 | model max | model RMS | projection p95 |
|---|---:|---:|---:|---:|---:|
| 20260705 B0 | 3 | `1.100±0.034mm` | `2.102±0.222mm` | `0.547±0.016mm` | `2.947±0.087cm` |
| 20260705 B_slosh | 3 | `0.632±0.101mm` | `1.255±0.039mm` | `0.307±0.035mm` | `3.442±0.128cm` |
| 20260705 B_ours | 3 | `0.892±0.076mm` | `1.950±0.414mm` | `0.432±0.018mm` | `3.527±0.204cm` |

因此，20260702 “B0 反而更好”不能作为方法无效结论，主要原因应拆开看：

1. **delay/tracking baseline 没有先 gate 干净**：20260705 先做了 B0 fixed closed-loop `0.15/0.22` 跟踪 gate，确认 baseline 稳定到点、projection 在约 3cm 量级，再进行三方法正式比较。
2. **hard feasibility 混入**：20260702 hard 组存在 ACADOS fail 和 hard cap namespace/读取疑点；本次 formal N=3 明确 `slosh_constraint_enable=0`，因此是干净 soft cost 对比。
3. **slosh cost 尺度/eta-dot 问题**：旧分析记录指出 runtime 与 codegen 对 `eta_dot_ref` 的尺度可能不一致；本次 cost monitor 显示 eta/eta-dot share 实时可见，且 slosh cost 没有压倒 progress。
4. **实验变量混杂**：20260702 不是先固定 tracking/delay baseline 后只切 variant；20260705 formal N=3 的唯一主要变量是 `B0/B_slosh/B_ours` variant。

---

## 9. 最终结论

1. **安全与配置**：9 个 bag 全部 `GOAL_REACHED`；effective_config 9/9 符合 `V_REF=0.20`、`alpha_max=1.2`、共享加速度/角速度限制、`fixed_closed_loop 0.15/0.22`、hard disabled；`zero_due_to_tracking_safety=0`。
2. **delay compensation**：`delay_compensation_applied_frac=1.0`，相关 debug topics 均存在，说明固定闭环 delay compensation 实际生效。
3. **internal model slosh**：在 10%~90% 主窗口，`B_slosh` 相对 B0 降低 p95 `42.5%`、max `40.3%`、RMS `43.9%`；`B_ours` 相对 B0 降低 p95 `18.9%`、max `7.2%`、RMS `21.0%`。模型侧排序为 `B_slosh < B_ours < B0`。
4. **RGB 真值**：在 10%~90% 主窗口，`B_ours` 的 RGB `H_vis` p95/max/RMS 最低，相对 B0 分别下降 `29.8% / 33.3% / 29.0%`；`B_slosh` 的 RGB p95/max 也低于 B0，但 RMS 基本接近 B0。
5. **控制平滑性**：`B_ours` 的 `|omega|` p95/max 最低，10%~90% 为 `0.207±0.007rad/s` / `0.269±0.002rad/s`，明显低于 B0 和 B_slosh。
6. **方法解释**：`B_slosh` 是模型侧最强抑晃；`B_ours` 则在 smooth/control 局部介入更强，牺牲一点 internal-model slosh 最小化，换来更低角速度尖峰，并在本次 RGB 真值中得到最低真实液面包络。
