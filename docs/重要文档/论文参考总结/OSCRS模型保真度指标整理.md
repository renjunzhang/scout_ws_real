# OSCRS 模型保真度指标整理

来源：`/home/a/下载/液面晃动方案研究/保真度指标.html`

结论：该 HTML 的总体方向是正确的。它把 `/slosh/height` 与视觉真值分开，并从 Ferrari-style 曲线保真度扩展到 OSCRS 候选选择保真度，这是适合当前论文验证需求的。但实际写入流程和报告时需要补充几个约束，避免指标被误用。

## 1. 推荐采用的三层结构

### 1.1 Ferrari-style 绝对保真度

回答问题：模型曲线是否接近实验/视觉曲线。

主指标：

```text
gamma_model_pct =
  100 * integral [H_model(t) - H_vis(t)] dt
      / (integral H_model(t) dt + epsilon)
```

辅助指标：

```text
RMSE
correlation
e_p95  = model_p95  - visual_p95
e_peak = model_peak - visual_peak
e_rms  = model_rms  - visual_rms
```

解释：

```text
gamma_model_pct > 0: 模型整体偏保守
gamma_model_pct ≈ 0: 模型整体吻合
gamma_model_pct < 0: 模型整体低估，安全风险较高
```

注意：`gamma_model_pct` 是积分平均指标，可能掩盖局部峰值低估，所以不能单独支撑 OSCRS hard gate。

### 1.2 OSCRS 选择保真度

回答问题：模型能不能正确指导候选路径排序。

主指标：

```text
A_rank =
  count{pair: sign(model_diff) == sign(visual_diff)}
  / count{all pairs}
```

推荐使用 `p95` 作为主排序指标：

```text
visual_diff = visual_p95_B - visual_p95_A
model_diff  = model_p95_B  - model_p95_A
```

解释：

```text
A_rank >= 0.80: 排序较可信
0.50 <= A_rank < 0.80: 只能作为辅助选择依据
A_rank < 0.50: 不足以指导候选选择
```

注意：如果每个 run 只有 RAW/FIXED/OSCRS 三个条件，则只有 3 个 pair，样本数很小。`A_rank` 应作为强诊断信号，不应过度做统计显著性解释。

### 1.3 安全 gate 保真度

回答问题：模型是否会漏判高晃动或危险片段。

主指标：

```text
U_p95  = max(0, visual_p95  - model_p95)
U_peak = max(0, visual_peak - model_peak)
U_max  = max_t [H_vis(t) - H_model(t)]_+
r_under = count{t: H_model(t) < H_vis(t)} / count{t}
```

若视觉液面存在超过 `eta_lim` 的片段，计算：

```text
violation_recall =
  count{t: H_vis(t) > eta_lim and H_model(t) > eta_lim}
  / count{t: H_vis(t) > eta_lim}
```

若当前视觉幅值远低于 `eta_lim`，固定阈值 recall 没有信息量，应做 threshold sweep：

```text
tau = 0.5, 1.0, 1.5, 2.0, 3.0, 4.0 mm
或 tau = P80, P90, P95 of H_vis
```

## 2. 视觉真值口径

主视觉真值：

```text
h_final = median(h_left, h_center, h_right)
h_corr = h_final - h0
h_smooth_corr = rolling_median(h_corr, window=5)
H_vis(t) = abs(h_smooth_corr(t))
```

安全峰值辅证：

```text
H_vis_max(t) = max(h_left - h0_left, h_center - h0_center, h_right - h0_right)
```

如果只有统一零点 `h0`，可退化为：

```text
H_vis_max(t) = max(h_left, h_center, h_right) - h0
```

注意：`h_max_lcr = max(h_left, h_center, h_right)` 本身不是零点校正后的晃动幅值。用于 safety peak 前必须转成相对初始液位。

## 3. 时间对齐修正

HTML 中使用“最近邻配对”是可以理解的简化，但最终实现建议使用插值：

```text
对每个视觉时间戳 t_vis:
  找到 /slosh/height 中包围 t_vis 的两个样本；
  若相邻模型样本间隔 <= 0.15 s，则线性插值得到 H_model(t_vis)；
  否则该视觉帧记为未配对。
```

配对质量只统计实际参与配对的样本：

```text
paired_samples
pair_dt_median_ms
pair_dt_p95_ms
```

不要把未通过插值窗口的视觉帧计入 `pair_dt_p95_ms`。

## 4. Sign Match 的死区

直接用 `sign(diff)` 容易被很小的数值噪声影响。建议加入死区：

```text
sign_eps = 0.05 mm 或 0.10 mm

if abs(diff) < sign_eps:
    sign = 0
else:
    sign = +1 / -1
```

若视觉差值或模型差值落在死区内，该 pair 应标为 `tie/ambiguous`，不要强行判 yes/no。

## 5. 当前 Phase4 的解释

当前 phase4 已补算：

```text
model_fidelity_summary.csv
model_selection_fidelity.csv
A_rank = 0.500 = 3/6
```

关键结论：

```text
1. /slosh/height 的 gamma_model_pct 全部为正，说明积分意义上整体偏保守。
2. 但 run01 OSCRS 和 run02 RAW 的 e_p95 为负，说明 p95 层面存在低估。
3. U_max 在多个条件下非零，说明局部时刻仍有模型低估视觉液面的风险。
4. A_rank = 0.500，说明模型对候选优劣排序只有一半与视觉一致。
5. 因此 /slosh/height 可作为控制侧辅助指标，但不足以单独支撑 OSCRS hard gate 或候选排序可信性声明。
```

推荐报告表述：

```text
当前模型侧 /slosh/height 与视觉真值在部分条件上趋势一致，但在 run01 OSCRS 和 run02 RAW 中存在 p95 低估；同时候选排序一致率 A_rank=50%。因此，当前模型可作为控制侧辅助量，但尚不能单独作为真实液面 ground truth，也不足以独立支撑 OSCRS hard gate 的实验保真度声明。
```

## 6. 论文中建议保留的指标

主文建议保留：

```text
gamma_model_pct
RMSE
U_p95
U_max
A_rank
```

补充材料或附表建议保留：

```text
correlation
e_p95 / e_peak / e_rms
under_ratio
threshold-sweep recall
pair_dt_median_ms / pair_dt_p95_ms
```

最终判断顺序：

```text
1. 先看视觉结论：真实液面是否降低。
2. 再看 Ferrari-style 保真度：模型曲线是否整体接近视觉。
3. 再看 U_p95 / U_max：是否存在安全低估。
4. 最后看 A_rank：模型是否能指导 OSCRS 候选选择。
```

## 7. 通用分析流程

本节记录从实物 bag 和视觉 CSV 生成模型保真度表的固定流程，避免后续重新熟悉。

### 7.1 输入文件

每个实验组至少需要：

```text
1. 动态 bag
   - /camera/color/image_raw
   - /slosh/height
   - /mpc_status
   - /odom, /cmd_vel 可选，用于 trajectory_analysis

2. 视觉结果 CSV
   - 每帧 stamp_sec 或 t_rel_sec
   - h_mm_left / h_mm_center / h_mm_right
   - h_mm_smooth_corr 或 h_smooth_corr

3. 汇总 CSV
   - visual p95 / peak / RMS
   - model p95 / peak / RMS
   - run / condition / bag

4. 组内差值 CSV
   - visual_p95_diff
   - model_p95_diff
   - comparison
```

phase4 示例：

```text
docs/Claude/分析数据/phase4_visual_20260509/phase4_visual_metric_summary_0424style.csv
docs/Claude/分析数据/phase4_visual_20260509/phase4_runwise_effect_summary_0424style.csv
/data/a/slosh_bags/real/20260508_phase4/phase4_red_visual_debug_20260510/*/*_red_top.csv
```

### 7.2 预处理

固定口径：

```text
H_vis(t) = abs(h_smooth_corr(t))
H_model(t) = /slosh/height(t) * 1000  # m -> mm
```

时间窗口：

```text
t0 = tracking_start
t1 = tracking_end + 2s
```

如果没有可靠 `tracking_end`，就使用 `tracking_start` 后全部有效样本。`tracking_start` 从 `/mpc_status == TRACKING` 的第一个时间戳获取；没有该话题时退化为 `bag_start`，但报告必须注明。

### 7.3 时间配对

对每个视觉时间戳 `t_vis`：

```text
1. 在 /slosh/height 时间序列中找到包围 t_vis 的两个样本；
2. 如果两个模型样本间隔 <= 0.15 s，线性插值得到 H_model(t_vis)；
3. 否则该视觉样本标为 unpaired；
4. pair_dt 取 t_vis 到最近模型样本的时间差。
```

输出：

```text
paired_samples
pair_dt_median_ms
pair_dt_p95_ms
```

验收：

```text
pair_dt_p95_ms <= 80 ms: 时间配对可接受
pair_dt_p95_ms > 80 ms: correlation / gamma_model 需要谨慎解释
```

### 7.4 单 bag Ferrari-style 保真度

对每个 bag 输出一行：

```text
run
condition
paired_samples
pair_dt_median_ms
pair_dt_p95_ms
gamma_model_pct
rmse_mm
corr
visual_p95_mm
model_p95_mm
e_p95_mm
U_p95_mm
visual_peak_mm
model_peak_mm
e_peak_mm
U_peak_mm
U_max_mm
under_ratio
```

计算公式：

```text
gamma_model_pct =
  100 * trapz(H_model - H_vis, t)
      / (trapz(H_model, t) + epsilon)

rmse_mm = sqrt(mean((H_model - H_vis)^2))
corr = pearson_corr(H_model, H_vis)

e_p95 = model_p95 - visual_p95
U_p95 = max(0, visual_p95 - model_p95)

e_peak = model_peak - visual_peak
U_peak = max(0, visual_peak - model_peak)

U_max = max_t [H_vis(t) - H_model(t)]_+
under_ratio = mean(H_model(t) < H_vis(t))
```

建议输出文件名：

```text
model_fidelity_summary.csv
```

### 7.5 组内 OSCRS 选择保真度

只在同一 run/block 内比较，不跨不同初始液位或不同路径混合。

对每个 pair 输出：

```text
run
pair
visual_p95_diff_mm
model_p95_diff_mm
sign_match
```

推荐 pair：

```text
FIXED_MILD - RAW
OSCRS_MEDIUM_ACTIVE - RAW
OSCRS_MEDIUM_ACTIVE - FIXED_MILD
```

死区：

```text
sign_eps = 0.05 mm 或 0.10 mm
```

判定：

```text
if abs(diff) < sign_eps:
    sign = 0   # tie / ambiguous
else:
    sign = sign(diff)

sign_match = sign(model_diff) == sign(visual_diff)
```

`A_rank`：

```text
A_rank = count(sign_match == yes) / count(non-ambiguous pairs)
```

建议输出文件名：

```text
model_selection_fidelity.csv
```

### 7.6 安全 gate 保真度

若真实视觉幅值足够接近或超过 `eta_lim`，输出：

```text
violation_recall at eta_lim
false_safe_count = count{H_vis > eta_lim and H_model <= eta_lim}
```

若视觉幅值远低于 `eta_lim`，不要强行解释 `eta_lim recall`，改做 sweep：

```text
tau_mm = 0.5, 1.0, 1.5, 2.0, 3.0, 4.0
或 tau = P80, P90, P95 of H_vis
```

每个阈值输出：

```text
tau_mm
true_high_count = count{H_vis > tau}
model_detect_count = count{H_vis > tau and H_model > tau}
recall = model_detect_count / true_high_count
false_safe_count = count{H_vis > tau and H_model <= tau}
```

建议输出文件名：

```text
model_gate_fidelity_threshold_sweep.csv
```

### 7.7 报告结构

保真度报告必须和视觉结论分开写：

```text
1. 视觉结论
   - OSCRS 相对 RAW 的 h_smooth_corr p95/peak/RMS 是否降低

2. Ferrari-style 绝对保真度
   - gamma_model_pct / RMSE / corr
   - e_p95 / U_p95 / U_max

3. OSCRS 选择保真度
   - pairwise sign_match
   - A_rank

4. 安全 gate 保真度
   - U_p95 / U_max
   - threshold-sweep recall

5. 最终判断
   - /slosh/height 是可信、部分可信还是不可信
   - 是否足以支撑 OSCRS hard gate 或候选选择
```

### 7.8 结论模板

```text
本批数据中，/slosh/height 与视觉真值的时间配对质量为
pair_dt_p95_ms = ... ms。Ferrari-style gamma_model_pct 显示模型整体
[偏保守 / 接近 / 低估]，但 U_p95 = ... mm、U_max = ... mm 表明
[存在 / 不存在] 安全低估风险。

在 OSCRS 选择保真度上，A_rank = ...，即 .../... 个 run 内 pair
与视觉排序方向一致。因此，当前 /slosh/height
[可以 / 只能部分 / 不能] 用作候选排序依据。

最终判断：/slosh/height 可作为控制侧辅助指标，但
[尚不足以 / 可以] 单独支撑 OSCRS hard gate 的实验保真度声明。
```

### 7.9 常见错误

```text
1. 把 /slosh/height 当视觉真值。
2. 把 run01/run02 的原始液面高度直接混合平均。
3. h_max_lcr 未减零点就当晃动幅值。
4. pair_dt_p95_ms 统计了未成功配对的样本。
5. A_rank 跨不同路径或不同初始液位计算。
6. 不加 sign_eps，导致 0.01 mm 级别差异决定排序方向。
7. gamma_model_pct 为正就宣称 hard gate 安全，忽略 U_max 局部低估。
```
