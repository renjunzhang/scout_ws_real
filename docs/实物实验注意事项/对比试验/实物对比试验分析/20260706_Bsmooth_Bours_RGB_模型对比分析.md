# 20260706 SPMPC 实物 fixed-path：B_smooth / B_ours / B_slosh RGB 与模型综合分析

> 数据目录：`/media/a/ZRJ/slosh_bags/20260706_fixed_path_compare`  
> RGB 标定：`/media/a/ZRJ/slosh_bags/20260629_calib/red_3ruler.yaml`  
> 分析脚本与 CSV/JSON 输出：`docs/Claude/分析数据/20260706_fixed_path_compare/`  
> 实验计划：`docs/实物实验注意事项/对比试验/实物对比实验/0706_SPMPC实物补充实验矩阵计划.md`  
> 统计窗口：`first ACADOS_OK -> first GOAL_REACHED`，主窗口额外按 `/spmpc/debug/stage0_reference.s0` 取 path progress `10%~90%`。  
> RGB 主指标：`H_vis(t)=abs(rolling_median(max(h_left,h_center,h_right), window=5))`，CSV 字段为 `h_mm_max_lcr_smooth_corr`。

本次只做离线数据分析，没有修改 MPC、GeoRef、OSCRS 或控制链路代码。`lt_dwa_official` 两个 bag 已完成 RGB 提取，但它们缺少 SPMPC internal topics，本报告不把它们混入 SPMPC 模型/代价/保真度主表。

---

## 1. 处理记录与输出文件

本次生成/使用的主要离线分析文件：

```text
docs/Claude/分析数据/20260706_fixed_path_compare/analyze_20260706_fixed_path_compare.py
docs/Claude/分析数据/20260706_fixed_path_compare/analyze_20260706_fidelity.py
docs/Claude/分析数据/20260706_fixed_path_compare/20260706_fixed_path_compare_run_metrics.csv
docs/Claude/分析数据/20260706_fixed_path_compare/20260706_fixed_path_compare_cost_component_stats.csv
docs/Claude/分析数据/20260706_fixed_path_compare/20260706_fixed_path_compare_method_aggregate.csv
docs/Claude/分析数据/20260706_fixed_path_compare/20260706_fixed_path_compare_cost_component_aggregate.csv
docs/Claude/分析数据/20260706_fixed_path_compare/20260706_fixed_path_compare_config_audit.csv
docs/Claude/分析数据/20260706_fixed_path_compare/20260706_fixed_path_compare_reductions_long.csv
docs/Claude/分析数据/20260706_fixed_path_compare/20260706_fixed_path_compare_analysis_summary.json
docs/Claude/分析数据/20260706_fixed_path_compare/20260706_fixed_path_compare_ferrari_fidelity_per_run.csv
docs/Claude/分析数据/20260706_fixed_path_compare/20260706_fixed_path_compare_ferrari_fidelity_aggregate.csv
docs/Claude/分析数据/20260706_fixed_path_compare/20260706_fixed_path_compare_ferrari_gate_threshold_sweep.csv
docs/Claude/分析数据/20260706_fixed_path_compare/20260706_fixed_path_compare_ferrari_gate_threshold_sweep_aggregate.csv
docs/Claude/分析数据/20260706_fixed_path_compare/20260706_fixed_path_compare_ferrari_fidelity_summary.json
```

RGB 推理输出在：

```text
/media/a/ZRJ/slosh_bags/20260706_fixed_path_compare/red_visual_analysis_20260706/red_infer
```

纳入 SPMPC 主分析的 bag：

| method | run | bag | RGB |
|---|---|---|---|
| B0 | gate01 | `B0/B0_fixed_150_220_0706_gate01.bag` | no |
| B_smooth | r1/r2/r3 | `B_smooth/Bsmooth_fixed_150_220_0706_r*.bag` | yes |
| B_ours | r1/r2/r3 | `B_ours/Bours_fixed_150_220_0706_r*.bag` | yes |
| B_slosh | r1/r2/r3 | `B_slosh/Bslosh_fixed_150_220_0706_r*.bag` | yes |

---

## 2. 配置、到点与安全复核

### 2.1 SPMPC bags 均成功到点

| method | runs | GOAL_REACHED |
|---|---:|---:|
| B0 gate | 1 | 1/1 |
| B_smooth | 3 | 3/3 |
| B_ours | 3 | 3/3 |
| B_slosh | 3 | 3/3 |
| total | 10 | 10/10 |

### 2.2 effective_config 全部符合 0706 计划口径

`20260706_fixed_path_compare_config_audit.csv` 复核结果：`all_requested_config_ok = 1` for `10/10`。

| item | target | result |
|---|---:|---:|
| `V_REF` / `v_ref` | `0.20` | 10/10 OK |
| `alpha_max` | `1.2` | 10/10 OK |
| `shared_linear_accel_max` | `0.6` | 10/10 OK |
| `shared_angular_rate_max` | `1.2` | 10/10 OK |
| `shared_angular_accel_max` | `1.2` | 10/10 OK |
| `delay_phase_mode_code` | `3.0` = `fixed_closed_loop` | 10/10 OK |
| `delay_linear_sec` | `0.15` | 10/10 OK |
| `delay_angular_sec` | `0.22` | 10/10 OK |
| `slosh_constraint_enable` | `0` | 10/10 OK |

正式 RGB runs 的 RGB 尺寸均为 `1920x1080`；B0 gate 按计划 `RECORD_RGB=false`，因此 B0 不参与 RGB 降幅比较。

### 2.3 delay compensation 与 command safety

| window | method | delay applied frac | published zero frac | tracking safety zero frac | linear limited frac | angular rate limited frac | angular accel limited frac |
|---|---|---:|---:|---:|---:|---:|---:|
| full | B0 | `1.000` | `0.000` | `0.000` | `0.014` | `0.000` | `0.000` |
| full | B_smooth | `1.000±0.000` | `0.000±0.000` | `0.000±0.000` | `0.009±0.002` | `0.000±0.000` | `0.000±0.000` |
| full | B_slosh | `1.000±0.000` | `0.000±0.000` | `0.000±0.000` | `0.009±0.002` | `0.000±0.000` | `0.005±0.002` |
| full | B_ours | `1.000±0.000` | `0.000±0.000` | `0.000±0.000` | `0.010±0.003` | `0.000±0.000` | `0.002±0.001` |
| path 10%~90% | B0 | `1.000` | `0.000` | `0.000` | `0.005` | `0.000` | `0.000` |
| path 10%~90% | B_smooth | `1.000±0.000` | `0.000±0.000` | `0.000±0.000` | `0.005±0.003` | `0.000±0.000` | `0.000±0.000` |
| path 10%~90% | B_slosh | `1.000±0.000` | `0.000±0.000` | `0.000±0.000` | `0.006±0.003` | `0.000±0.000` | `0.004±0.002` |
| path 10%~90% | B_ours | `1.000±0.000` | `0.000±0.000` | `0.000±0.000` | `0.010±0.005` | `0.000±0.000` | `0.001±0.001` |

结论：本组正式比较没有 tracking safety 停车污染；delay compensation 全程生效。

---

## 3. 主指标：path progress 10%~90%

这是最推荐引用的主窗口。

| date | scenario | method | N/success | model p95 mm | model max mm | model RMS mm | RGB p95 mm | RGB max mm | RGB RMS mm | projection p95 cm | `|omega|` p95 rad/s | `|omega|` max rad/s | goal time s |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 20260706 | fixed-path 10%~90% | B0 | 1/1 | `1.130±0.000` | `2.734±0.000` | `0.582±0.000` | - | - | - | `4.194±0.000` | `0.226±0.000` | `0.365±0.000` | `33.813±0.000` |
| 20260706 | fixed-path 10%~90% | B_smooth | 3/3 | `0.994±0.165` | `2.227±0.240` | `0.491±0.066` | `1.272±0.283` | `2.267±0.309` | `0.639±0.060` | `3.646±1.188` | `0.202±0.022` | `0.248±0.022` | `35.169±0.537` |
| 20260706 | fixed-path 10%~90% | B_slosh | 3/3 | `0.619±0.050` | `1.813±0.628` | `0.314±0.031` | `1.683±0.169` | `2.865±0.466` | `0.776±0.034` | `3.655±0.644` | `0.229±0.015` | `0.319±0.017` | `36.989±0.135` |
| 20260706 | fixed-path 10%~90% | B_ours | 3/3 | `0.924±0.187` | `1.727±0.150` | `0.450±0.063` | `1.309±0.303` | `2.353±0.888` | `0.654±0.097` | `4.687±0.628` | `0.202±0.013` | `0.252±0.010` | `37.592±0.460` |

主窗口事实：

```text
internal model: B_slosh < B_ours < B_smooth < B0 gate。
RGB H_vis: B_smooth ≈ B_ours < B_slosh；B_ours 相比 B_smooth 略高 2%~4%，差值小于组内标准差。
tracking/time: 三个正式方法均 60s 内到点；B_ours 比 B_smooth 慢约 2.42s，projection p95 约 4.69cm。
command smoothness: B_ours 与 B_smooth 的 |omega| p95 基本相同，B_slosh 更高。
```

---

## 4. full active window 主指标

| date | scenario | method | N/success | model p95 mm | model max mm | model RMS mm | RGB p95 mm | RGB max mm | RGB RMS mm | projection p95 cm | `|omega|` p95 rad/s | `|omega|` max rad/s | goal time s |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 20260706 | full active window | B0 | 1/1 | `1.338±0.000` | `4.113±0.000` | `0.676±0.000` | - | - | - | `3.968±0.000` | `0.253±0.000` | `0.447±0.000` | `33.813±0.000` |
| 20260706 | full active window | B_smooth | 3/3 | `1.094±0.139` | `3.592±0.950` | `0.558±0.054` | `1.216±0.140` | `2.267±0.309` | `0.588±0.037` | `3.558±0.431` | `0.224±0.008` | `0.264±0.008` | `35.169±0.537` |
| 20260706 | full active window | B_slosh | 3/3 | `0.696±0.033` | `1.993±0.492` | `0.355±0.023` | `1.268±0.275` | `2.865±0.466` | `0.711±0.042` | `3.365±0.346` | `0.257±0.023` | `0.445±0.016` | `36.989±0.135` |
| 20260706 | full active window | B_ours | 3/3 | `0.942±0.152` | `1.905±0.193` | `0.468±0.053` | `1.239±0.251` | `2.353±0.888` | `0.631±0.086` | `4.588±0.644` | `0.210±0.008` | `0.253±0.008` | `37.592±0.460` |

full window 与主窗口一致：`B_slosh` 模型侧最低但 RGB 更差；`B_ours` 模型侧优于 `B_smooth`，但 RGB 与 `B_smooth` 近似持平。

---

## 5. 降幅表

### 5.1 相对 B_smooth：回答“是否优于单纯平滑控制”

| window | method | model p95 | model max | model RMS | RGB p95 | RGB max | RGB RMS | `|omega|` p95 | goal time |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| path 10%~90% | B_slosh vs B_smooth | `37.7%` | `18.6%` | `36.0%` | `-32.3%` | `-26.4%` | `-21.5%` | `-13.5%` | `-5.2%` |
| path 10%~90% | B_ours vs B_smooth | `7.0%` | `22.4%` | `8.4%` | `-2.9%` | `-3.8%` | `-2.4%` | `-0.2%` | `-6.9%` |

解释：

- `B_ours` 相比 `B_smooth` 的 internal model 指标确实下降，尤其 model max 下降 `22.4%`。
- 但 RGB `H_vis` 没有出现“明显低于 B_smooth”的结果；p95/max/RMS 均为小幅负降幅，即略高于 `B_smooth`。
- 因此 0706 对“显式晃液状态预测 + 平滑控制优于单纯平滑控制”的支持应写成：**模型侧有增益，RGB 侧本日未显示显著优于 smooth-only**。

### 5.2 相对 B0 gate：只用于模型侧和执行侧参考

B0 gate 没录 RGB，因此不做 RGB 相对 B0 降幅。

| window | method | model p95 | model max | model RMS | `|omega|` p95 | goal time |
|---|---|---:|---:|---:|---:|---:|
| path 10%~90% | B_smooth vs B0 | `12.1%` | `18.6%` | `15.6%` | `10.5%` | `-4.0%` |
| path 10%~90% | B_slosh vs B0 | `45.3%` | `33.7%` | `46.0%` | `-1.5%` | `-9.4%` |
| path 10%~90% | B_ours vs B0 | `18.3%` | `36.8%` | `22.7%` | `10.3%` | `-11.2%` |

---

## 6. cost share

单位均为百分比。每格为 `p50 / p95 / mean`。`progress_abs` 使用 `abs(pct_progress)`。

| method | progress_abs | contour | v | control | smooth | slosh |
|---|---:|---:|---:|---:|---:|---:|
| B0 | `66.515 / 73.365 / 61.996` | `3.456 / 25.395 / 6.502` | `24.998 / 48.966 / 27.328` | `4.554 / 8.299 / 4.130` | `0.000 / 0.021 / 0.004` | `0.000 / 0.000 / 0.000` |
| B_smooth | `66.956 / 73.712 / 63.016` | `2.643 / 29.026 / 6.320` | `25.664 / 47.390 / 27.518` | `2.880 / 7.016 / 3.084` | `0.000 / 0.039 / 0.008` | `0.000 / 0.000 / 0.000` |
| B_slosh | `64.640 / 76.108 / 61.923` | `3.656 / 26.301 / 6.804` | `22.672 / 45.680 / 25.049` | `4.502 / 8.637 / 4.253` | `0.023 / 0.272 / 0.067` | `1.283 / 4.998 / 1.866` |
| B_ours | `60.757 / 77.790 / 59.260` | `4.632 / 30.428 / 8.184` | `23.746 / 42.924 / 24.758` | `3.913 / 10.664 / 4.456` | `0.070 / 1.166 / 0.400` | `2.088 / 7.640 / 2.895` |

Slosh 内部 eta / eta-dot share，单位百分比，每格为 `p50 / p95`：

| method | eta share | eta-dot share |
|---|---:|---:|
| B0 | `0.000 / 0.000` | `0.000 / 0.000` |
| B_smooth | `0.000 / 0.000` | `0.000 / 0.000` |
| B_slosh | `88.944 / 98.841` | `11.056 / 39.277` |
| B_ours | `84.055 / 97.412` | `15.945 / 33.573` |

cost 结论：

```text
1. B_smooth 的 slosh cost 为 0，符合 smooth-only 定义。
2. B_slosh / B_ours 的 slosh soft cost 都实际进入优化器。
3. B_ours 的 slosh mean share 与 smooth mean share 都高于 B_slosh，说明完整方法的两个额外目标都在起作用。
4. progress_abs 仍约 59%~63%，未被 slosh/smooth cost 压倒，因此正式 runs 都能稳定到点。
```

---

## 7. Ferrari-style 绝对保真度

视觉真值：`H_vis(t)=abs(h_mm_max_lcr_smooth_corr)`；模型侧：`H_model(t)=abs(/spmpc/slosh_height)`。按 RGB 时间戳插值模型序列，模型样本间隔上限为 `0.15s`。

$$
\gamma_{\text{model\%}} = 100 \cdot \frac{\int \big(H_{\text{model}}(t)-H_{\text{vis}}(t)\big)\,dt}{\int H_{\text{model}}(t)\,dt+\epsilon}
$$

解释：`gamma > 0` 表示模型偏保守，`gamma ≈ 0` 表示接近，`gamma < 0` 表示模型低估视觉包络。

### 7.1 path progress 10%~90%

| method | N | gamma % | RMSE mm | corr | visual p95 | model p95 | U_p95 | under ratio |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| B_smooth | 3 | `-41.146±5.689` | `0.442±0.035` | `0.177±0.074` | `1.272±0.283` | `0.928±0.150` | `0.344±0.145` | `0.701±0.014` |
| B_slosh | 3 | `-168.760±20.084` | `0.587±0.046` | `0.265±0.085` | `1.683±0.169` | `0.552±0.036` | `1.132±0.203` | `0.910±0.021` |
| B_ours | 3 | `-59.181±24.967` | `0.444±0.070` | `0.155±0.209` | `1.309±0.303` | `0.853±0.207` | `0.456±0.347` | `0.712±0.043` |

### 7.2 full active window

| method | N | gamma % | RMSE mm | corr | visual p95 | model p95 | U_p95 | under ratio |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| B_smooth | 3 | `-14.914±8.420` | `0.456±0.031` | `0.169±0.097` | `1.216±0.141` | `1.023±0.146` | `0.193±0.120` | `0.588±0.023` |
| B_slosh | 3 | `-115.836±11.596` | `0.530±0.046` | `0.164±0.046` | `1.270±0.274` | `0.647±0.035` | `0.624±0.299` | `0.814±0.016` |
| B_ours | 3 | `-43.389±23.597` | `0.440±0.068` | `0.183±0.141` | `1.239±0.251` | `0.864±0.149` | `0.375±0.245` | `0.651±0.056` |

保真度结论：

```text
1. 0706 中三组 formal RGB runs 的 gamma 均为负，说明模型整体低估 RGB 包络；不能把 /spmpc/slosh_height 当作真实液面 ground truth。
2. B_slosh 低估最严重：path 10%~90% gamma=-168.8%，under_ratio=0.91。
3. B_smooth 与 B_ours 的 RMSE 接近，B_ours 没有表现出 0705 那种明显更好的绝对保真度。
4. corr 仍偏低，不能 claim 逐帧相位保真；只能使用幅值统计、趋势与安全诊断表述。
```

---

## 8. LT-DWA bags 处理状态

本目录还包含：

```text
lt_dwa_official/LTDWA_fixed_0706_shadow01.bag
lt_dwa_official/LTDWA_fixed_0706_smoke01.bag
```

两者已完成 RGB 提取：

| bag | RGB frames | RGB range note |
|---|---:|---|
| `LTDWA_fixed_0706_shadow01` | 777 | `h_mm_max_lcr` 约 `0.14~0.22mm`，接近静稳/影子记录 |
| `LTDWA_fixed_0706_smoke01` | 1574 | `h_mm_max_lcr` 最高约 `8.00mm`，但缺少 SPMPC internal topics |

当前不把 LT-DWA 放入本报告主表，原因：它没有 `/spmpc/slosh_height`、`/spmpc/cost_breakdown`、`/spmpc/debug/*` 等 SPMPC 诊断，不能和 SPMPC variants 做同字段模型/代价/保真度比较。若后续需要外部 baseline，应单独建立 RGB + tracking + command 的对照报告，并先确认同一路径、同约束、同 goal 判据。

---

## 9. 结论与论文表述建议

### 9.1 事实结论

```text
1. 0706 10 个 SPMPC bags 均 GOAL_REACHED，配置 10/10 OK，delay compensation=1.0，tracking safety zero=0。
2. B_smooth N=3 与 B_ours N=3 同日 bridge 完成，补齐了 smooth-only 实物消融矩阵。
3. B_ours 相比 B_smooth 在 internal model 侧有小到中等降幅：p95 -7.0%，max -22.4%，RMS -8.4%。
4. 但 RGB H_vis 没有显示 B_ours 明显优于 B_smooth：p95/max/RMS 分别约高 2.9%/3.8%/2.4%，且差值小于 run 间波动。
5. B_slosh 模型侧最低，但 RGB 最差、Ferrari-style 低估最严重，因此不应作为最终主方法。
```

### 9.2 推荐写法

保守、符合事实的写法：

```text
20260706 同日 B_smooth/B_ours 消融显示，完整方法的显式晃液预测项确实进一步降低了控制器内部 slosh 指标；但在本日 RGB 真值中，B_ours 与 smooth-only 的视觉液面包络基本持平，未观察到显著优于 B_smooth 的 RGB 降幅。因此论文中不能仅凭 0706 claim “B_ours RGB 明显优于 B_smooth”，应表述为模型侧证据支持显式 slosh objective 的作用，RGB 侧需要结合更多重复或更强激励场景。
```

### 9.3 最终方法选择

如果最终只保留 `B_slosh` 或 `B_ours` 一个方法，本次 0706 结果仍支持保留 `B_ours`：

```text
B_slosh 虽然 internal model 最低，但 RGB 更高且模型严重低估真实视觉包络；B_ours 的 RGB 至少不劣于 B_smooth 太多，明显优于 B_slosh，并且具备 slosh + smooth 两个目标的可解释 cost evidence。因此 B_slosh 更适合作为 slosh-only ablation，而不是最终方法。
```
