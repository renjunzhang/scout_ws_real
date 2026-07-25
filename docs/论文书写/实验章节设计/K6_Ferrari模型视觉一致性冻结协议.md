# K6 Ferrari-inspired 模型—视觉一致性与敏感性冻结协议

> 协议编号：K6-FID-v1.0
>
> 冻结日期：2026-07-25
>
> 当前状态：**分析口径已冻结；执行准入尚未通过。** 在正式分析脚本、视觉配置、同步标定和 freeze manifest 完成第 12 节检查前，不得开始 88 次正式采集。
>
> 上位协议：[S-MPCC_experimental_design.md](./S-MPCC_experimental_design.md)
>
> 执行索引：[SPMPC实验矩阵设计.md](./SPMPC实验矩阵设计.md)
>
> 文献依据：[Ferrari et al. 2026 原文](<../../重要文档/论文参考总结/Ferrari 等 - 2026 - Time-Optimal Anti-Sloshing Trajectory Planning for Multiple Liquid-Filled Containers Subject to SCAR.pdf>)，“Time-Optimal Anti-Sloshing Trajectory Planning for Multiple Liquid-Filled Containers Subject to SCARA Motion,” *IEEE Robotics and Automation Letters*, vol. 11, no. 2, pp. 1762–1769, 2026, DOI: 10.1109/LRA.2025.3643281，重点为 Eq. (13)、Eq. (25)、Eq. (26) 及 Section IV。

---

## 1. 协议目的与证据层级

K6 使用正式实物日志检验：

1. S-MPCC 在线使用的非负模态液面包络 \(H_{\mathrm{modal}}\) 与独立视觉量 \(H_{\mathrm{vis}}\) 在幅值和时间趋势上是否一致；
2. 低阶模型在报告路径、容器和激励范围内整体偏高估还是偏低估；
3. 结论对 \(\omega_1,\zeta,c_h\)、液体初态和激励时间偏移是否敏感。

K6 是**支持性模型诊断**，不是 RQ5，不增加 88 次实物实验，也不能替代 RQ1–RQ3 的视觉物理结果。K6 不验证：

- 带符号二维模态状态或液体相位的完整准确性；
- 当前模型的旋转一致性；
- 高阶模态、破波、撞壁、溢出后的非线性流体行为；
- RGB 提取算法本身是无误差真值；
- 严格防溢、稳定性、递归可行性或安全保证。

因此，论文中应使用 “model–vision amplitude-envelope consistency” 或 “Ferrari-inspired model-consistency analysis”，不得概括为“完整液体动力学已被高保真验证”。

---

## 2. Ferrari 原文口径与本文适配

Ferrari 等使用视频提取的实验液面曲线 \(\bar\eta_{\mathrm{exp}}\) 验证模型曲线 \(\bar\eta_{\mathrm{model}}\)，其 Eq. (25) 为

\[
\gamma_{\mathrm{model}}^{\mathrm F}
=
100
\frac{
\int_0^{1.25t_{\mathrm{end}}}
\left(
\bar\eta_{\mathrm{model}}(t)
-
\bar\eta_{\mathrm{exp}}(t)
\right)\,\mathrm dt
}{
\int_0^{1.25t_{\mathrm{end}}}
\bar\eta_{\mathrm{model}}(t)\,\mathrm dt
}.
\]

该式**不带绝对值**。负值表示积分意义上的模型低估，正值表示模型高估；\(1.25t_{\mathrm{end}}\) 用于覆盖运动结束后的残余振荡。Ferrari Eq. (26) 的 \(\gamma_{\mathrm{opt}}\) 衡量优化轨迹相对非优化轨迹的峰值改善，不是模型保真度。

本文冻结以下适配：

| 项目 | Ferrari 原文 | K6-FID-v1.0 |
| --- | --- | --- |
| 外部参考 | GoPro 视频液面曲线 | 冻结 RGB 标定流程得到的 \(H_{\mathrm{vis}}\) |
| 模型量 | 最大液面高度估计 | S-MPCC 实际在线记录的 \(H_{\mathrm{modal}}\) |
| signed bias 分母 | 模型曲线积分 | 保持模型曲线积分，不改成视觉积分 |
| 主窗口 | \(0\) 至 \(1.25t_{\mathrm{end}}\) | \(t_{\mathrm{move}}\) 至 \(t_{\mathrm{arrival}}+5\,\mathrm{s}\) |
| 绝对误差 | 未作为 Eq. (25) | 另定义 \(\gamma_{\mathrm{abs}}\)，不得冒充 Eq. (25) |
| 优化收益 | \(\gamma_{\mathrm{opt}}\) | 不属于 K6；由 RQ1/RQ2 的预注册视觉指标承担 |

固定 5 s 到达后窗口与当前 88 次协议一致，并对所有 trial 使用相同规则。由于时间窗、平台和模型均不同，K6 数值不得与 Ferrari 表中数值作直接性能排名。

---

## 3. 分析总体、分层和统计单位

### 3.1 主要分析总体

K6 的主要总体是 E1–E3 中全部正式 S-MPCC 尝试，共计划 32 次：

| 分层 | 条件 | 计划 trial 数 |
| --- | --- | ---: |
| K6-S1 | \(C_1\)，低风险路径，E1 S-MPCC | 8 |
| K6-S2 | \(C_1\)，高风险路径，E2 S-MPCC | 8 |
| K6-S3 | \(C_2\)，高风险路径，E2 S-MPCC | 8 |
| K6-S4 | \(C_1\)，高风险等时间区组，E3 S-MPCC | 8 |
| 合计 | — | 32 |

所有 32 次正式尝试进入 K6 availability/failure 分母。连续一致性指标只对完成统一到达且通过信号质量检查的 trial 计算，并明确报告“有效数/全部尝试数”。

Baseline、Smooth-only 和 Smooth-match 的论文定义关闭液体状态，因此不得把不存在或为零的在线 \(H_{\mathrm{modal}}\) 当作其模型保真度。若未来用 executed-motion 离线传播同一模型覆盖全部 88 次，该结果必须标为独立的 supplementary replay，不得与本协议的 deployed-model fidelity 混合。

### 3.2 统计单位

一次完整 trial 是一个统计样本。视频帧、控制周期和配对时间点只用于计算 trial-level 指标，不能作为独立样本、不能扩大样本量，也不能直接用于显著性检验。

K6-S1 至 K6-S4 必须分别报告；可以给出 32 次总体的描述性汇总，但不能用总体汇总掩盖路径或容器分层中的系统性低估。

---

## 4. 冻结信号定义

### 4.1 模型侧：\(H_{\mathrm{modal}}\)

主要模型信号固定为 S-MPCC 在线发布的 `/spmpc/slosh_height`，单位为 mm，并满足

\[
H_{\mathrm{modal}}(t)
=c_h\sqrt{\eta_x^2(t)+\eta_y^2(t)}.
\]

正式配置保持 modal-only，关闭抛物面修正。K6 必须使用控制器实际在线记录的该信号：

- 不得用 `/slosh/height` 静默替代；
- 不得对模型曲线进行幅值拟合、缩放或按视觉结果重新标定；
- 不得在看到正式结果后切换模型 topic；
- 若 `/spmpc/slosh_height` 缺失、单位错误或与记录的模态状态不一致，该 trial 标为 K6 model-signal failure。

`/slosh/height` 可作为外部 monitor 的开发诊断，但不属于 K6 主要模型量。

### 4.2 视觉侧：\(H_{\mathrm{vis}}\)

K6 的视觉幅值固定使用 `red_liquid_infer_from_bag.py` 输出的 `h_mm_max_lcr_smooth_corr`：

\[
\begin{aligned}
h_{\max}(t)
&=\max\{h_{\mathrm{left}}(t),h_{\mathrm{center}}(t),h_{\mathrm{right}}(t)\},\\
h_{0,\max}
&=\operatorname{median}\!\left(h_{\max}
\text{ 在运动前 30 个有效帧上的取值}\right),\\
H_{\mathrm{vis}}(t)
&=\left|
\operatorname{rolling\_median}_{5}
\left(h_{\max}(t)-h_{0,\max}\right)
\right|.
\end{aligned}
\]

执行参数固定为 `--zero-correction-frames 30 --smooth-frames 5`，且这 30 个有效帧必须全部早于 \(t_{\mathrm{move}}\)；否则零点检查失败。每帧至少两个标尺有效才进入 K6；仅一个标尺有效的帧记为缺失。出现标尺范围 clipping 的 trial 标为 K6 censored，保留在 availability 表和物理结果中，但不把饱和值当作精确保真度数据。

`h_mm_smooth_corr` 只作为三标尺一致性敏感性和视觉质量检查，不替换 K6 主要视觉曲线。ROI、曝光、HSV、三标尺像素—毫米标定和相机位置必须在第一条正式 trial 前冻结，并对所有方法保持一致。

### 4.3 幅值含义

\(H_{\mathrm{modal}}\) 和 \(H_{\mathrm{vis}}\) 均为非负幅值/包络。K6 可以评价最大壁面爬升代理量的幅值与趋势，但不能由此声称 \(\eta_x,\eta_y\) 的符号、方向或相位得到直接视觉验证。

---

## 5. 时间窗、同步与配对

### 5.1 唯一主要窗口

对第 \(i\) 个 trial，固定

\[
\mathcal W_{\mathrm{K6},i}
=
[t_{\mathrm{move},i},\,
t_{\mathrm{arrival},i}+T_{\mathrm{post}}],
\qquad
T_{\mathrm{post}}=5\,\mathrm{s}.
\]

\(t_{\mathrm{move}}\) 与 \(t_{\mathrm{arrival}}\) 必须复用 RQ1–RQ3 已冻结的定义。不得为 K6 重新选择 tracking start、terminal 前窗口、最佳区段或手工裁剪终点。开始前静止段只用于零点和噪声估计，不进入一致性积分。

### 5.2 时间戳规则

1. 保留 bag 与图像原始时间戳，禁止把模型和视觉序列分别平移到各自的 \(t=0\) 后冒充同步。
2. 主要结果只允许应用独立同步标定得到的单一固定偏移 \(\tau_{\mathrm{cal}}\)；其数值和来源写入 freeze manifest。
3. 以有效视觉帧时间戳为锚点，在相邻模型样本间线性插值。
4. 最近模型样本时间差不得超过 50 ms，相邻模型样本间隔不得超过 100 ms；超过者记为缺失，不跨缺口积分。
5. 视觉序列不跨缺帧插值。

### 5.3 trial-level 时间质量准入

一个 trial 的时间序列指标有效，必须同时满足：

- \(f_{\mathrm{cam}}/f_1\geq 6\)；
- 配对覆盖率不低于 \(\mathcal W_{\mathrm{K6}}\) 内有效视觉帧的 90%；
- \(\operatorname{p95}(|\Delta t_{\mathrm{pair}}|)\leq 50\,\mathrm{ms}\)；
- 有效配对覆盖运动窗口和完整 5 s 到达后窗口；
- 不存在未标记的时钟跳变、重复时间戳或单位错误。

不满足时，该 trial 标为 K6 synchronization/coverage failure；不得放宽阈值、扩大插值间隔或只保留对齐较好的片段。

### 5.4 时滞诊断

主要 \(\gamma_{\mathrm{model}}^{\mathrm F}\)、\(\gamma_{\mathrm{abs}}\)、RMSE、峰值误差和 \(\rho_0\) 均使用固定 \(\tau_{\mathrm{cal}}\)，不使用按 trial 优化的时滞。

补充材料可计算

\[
\tau^\star
=
\arg\max_{\tau\in[-0.10,0.10]\,\mathrm{s}}
\operatorname{corr}
\left(
H_{\mathrm{modal}}(t),
H_{\mathrm{vis}}(t+\tau)
\right)
\]

及 \(\rho^\star\)，仅用于诊断剩余同步/相位偏差。不得用 \(\tau^\star\) 重新计算主要误差指标，也不得只报告时滞校正后的相关性。

---

## 6. trial-level 冻结指标

在 \(\mathcal W_{\mathrm{K6},i}\) 的有效配对片段上采用梯形积分；不跨超过第 5.2 节阈值的缺口。定义

\[
A_{\mathrm m,i}
=\int H_{\mathrm{modal},i}(t)\,\mathrm dt,
\qquad
A_{\mathrm v,i}
=\int H_{\mathrm{vis},i}(t)\,\mathrm dt.
\]

### 6.1 Ferrari-form signed bias：主要 K6 指标

\[
\boxed{
\gamma_{\mathrm{model},i}^{\mathrm F}
=
100
\frac{A_{\mathrm m,i}-A_{\mathrm v,i}}
{A_{\mathrm m,i}}
}
\]

- \(\gamma_{\mathrm{model}}^{\mathrm F}<0\)：模型在积分意义上低估视觉幅值；
- \(\gamma_{\mathrm{model}}^{\mathrm F}>0\)：模型在积分意义上高估视觉幅值；
- 接近 0 只表示积分面积接近，不表示局部峰值或时间相位准确。

### 6.2 防止正负抵消的必报指标

\[
\gamma_{\mathrm{abs},i}
=
100
\frac{
\int
\left|
H_{\mathrm{modal},i}(t)
-
H_{\mathrm{vis},i}(t)
\right|\,\mathrm dt
}{
A_{\mathrm m,i}
}.
\]

\(\gamma_{\mathrm{abs}}\) 越小表示积分绝对偏差越小。它必须与 signed bias 同时报告，不得将其命名为 Ferrari Eq. (25)。

同时报告：

\[
\begin{aligned}
\mathrm{RMSE}_i
&=
\sqrt{
\frac{1}{N_i}
\sum_{k=1}^{N_i}
\left(H_{\mathrm{modal},ik}-H_{\mathrm{vis},ik}\right)^2
},\\
\rho_{0,i}
&=
\operatorname{corr}
\left(H_{\mathrm{modal},i},H_{\mathrm{vis},i}\right),\\
e_{\mathrm{p95},i}
&=
Q_{0.95}(H_{\mathrm{modal},i})
-
Q_{0.95}(H_{\mathrm{vis},i}),\\
e_{\mathrm{peak},i}
&=
\max H_{\mathrm{modal},i}
-
\max H_{\mathrm{vis},i},\\
U_{\mathrm{p95},i}
&=\max(0,-e_{\mathrm{p95},i}),\\
U_{\max,i}
&=
\max_t
\left[
H_{\mathrm{vis},i}(t)
-
H_{\mathrm{modal},i}(t)
\right]_+.
\end{aligned}
\]

\(\rho_0\) 只反映线性趋势，不反映幅值准确性；\(U_{\max}\) 是局部低估诊断，不构成安全证明。

### 6.3 低激励和小分母规则

从独立静止标定数据计算

\[
\epsilon_{A,i}
=
T_{\mathrm{valid},i}
\max\left\{
Q_{0.95}(H_{\mathrm{modal,rest}}),
10^{-6}\,\mathrm{mm}
\right\}.
\]

若 \(A_{\mathrm m,i}\leq\epsilon_{A,i}\)，则
\(\gamma_{\mathrm{model}}^{\mathrm F}\) 和 \(\gamma_{\mathrm{abs}}\) 记为 NA，并标记 low-model-excitation；不得通过更换分母得到有限百分比。RMSE、峰值误差和原始曲线仍报告。

若视觉 p95 未超过独立静止视觉噪声 p95 加 \(3\operatorname{MAD}\)，则相关性和时滞指标记为 low-visual-excitation diagnostic，不作模型趋势结论。

---

## 7. 参数、初态和延迟敏感性

敏感性分析使用冻结的 executed-motion 模型 replay，逐项改变一个因素，其余保持名义值。正式水平固定为：

| 因素 | 水平 |
| --- | --- |
| \(\omega_1\) | \(0.9,1.0,1.1\) 倍冻结值 |
| \(\zeta\) | \(0.5,1.0,1.5\) 倍冻结值 |
| \(c_h\) | \(0.9,1.0,1.1\) 倍冻结值 |
| 激励时间偏移 \(\Delta t_a\) | \(-50,0,+50\) ms |
| 初始模态状态 | 正式记录状态、零状态 |

上述范围是局部敏感性检查，不是参数置信区间。不得增加更多水平后只展示最有利结果。

名义 replay 必须先在独立 smoke 日志上通过与在线 \(H_{\mathrm{modal}}\) 的复现门槛；复现 RMSE、峰值容差、数值积分器、激励计算/滤波和初始状态加载规则写入 freeze manifest。不能复现的正式 trial 不进入参数敏感性汇总，并报告 reproduction-failure rate。

每个变体输出相对名义值的
\(\Delta\gamma_{\mathrm{model}}^{\mathrm F}\)、
\(\Delta\gamma_{\mathrm{abs}}\)、
\(\Delta\mathrm{RMSE}\)、
\(\Delta U_{\max}\) 和
\(\Delta\rho_0\)。

正式数据只能用于描述敏感性，不能据此挑选新的 \(\omega_1,\zeta,c_h\) 后把同一批数据重新标记为“冻结模型结果”。若 K6 表明需要改参数或动力学，应建立新方法版本和独立实验，不得与当前 88 次矩阵混合。

---

## 8. 汇总、区间和图表规则

1. 每个 K6-S1 至 K6-S4 分层展示全部 trial 点。
2. 分层汇总报告 median、IQR、minimum、maximum；95% percentile interval 使用 10,000 次完整 block bootstrap，随机种子固定为 20260725，不按帧重采样。
3. K6 是诊断分析，不预注册“模型通过/失败”的单一阈值，也不对每个指标堆叠 p 值。
4. 任何 \(\gamma_{\mathrm{model}}^{\mathrm F}<0\)、\(U_{\max}>0\)、同步失败、clipping 和 reproduction failure 均保留，不得因不利于模型而删除。
5. 正文若只能展示一条对齐曲线，固定选择各分层中 \(\gamma_{\mathrm{abs}}\) 最接近该分层中位数的 trial；并列时选择 block ID 最小者。完整曲线进入补充材料。
6. 主要表至少包含：stratum、有效数/尝试数、\(\gamma_{\mathrm{model}}^{\mathrm F}\)、\(\gamma_{\mathrm{abs}}\)、RMSE、\(\rho_0\)、\(e_{\mathrm{p95}}\)、\(U_{\max}\) 和质量失败数。
7. 参数敏感性用全分层 small multiples 或紧凑表报告，不得只展示使模型更接近视觉的一组参数。

---

## 9. 失败、缺失和异常处理

以下情况不允许为了 K6 单独补跑或删除正式 trial：

- 模型整体低估、相关性低或局部 \(U_{\max}\) 大；
- solver/fallback/安全终止等方法相关失败；
- 模型进入 Ferrari 原文所述的强非线性失配区域；
- 液面达到标尺上限、撞击边界或发生溢出。

与方法无关且符合上位协议的采集故障仍按统一规则处理。K6 特有状态应分类记录：

| 状态 | 处理 |
| --- | --- |
| valid | 进入全部 K6 指标 |
| low excitation | 百分比或相关性按第 6.3 节记 NA，保留其他量 |
| synchronization/coverage failure | 不计算时间序列保真度，保留 availability |
| visual censored/clipped | 不把饱和值当精确误差，单独报告 |
| model-signal failure | 不允许用其他 topic 静默替代 |
| replay reproduction failure | 保留 nominal 曲线分析，排除敏感性变体并报告 |

K6 无效不自动使 RQ1–RQ3 的视觉物理结果无效；反之，视觉结果有效也不能掩盖 K6 模型失配。

---

## 10. 允许与禁止的论文表述

若数据支持，可以写：

> Across the reported S-MPCC trials, the low-order modal envelope showed [signed bias/absolute disagreement] relative to the calibrated visual amplitude, with the reported path- and container-specific limitations.

可以描述：

- 报告条件下模型总体偏高估或偏低估；
- 幅值趋势、局部低估和参数敏感性；
- 哪些路径、容器或激励范围出现明显失配。

禁止写：

- Ferrari 已证明本文 RGB 是绝对真值；
- signed bias 接近 0 即证明峰值、相位和动力学完全准确；
- 正相关即证明幅值准确；
- \(\gamma_{\mathrm{model}}^{\mathrm F}>0\) 即证明安全或无溢出；
- K6 证明旋转一致性、长期稳定性或任意容器迁移；
- 用内部模型一致性替代 \(H_{\mathrm{vis}}\) 的方法效果比较。

---

## 11. 冻结产物与唯一实现

正式采集前必须归档：

| 产物 | 冻结要求 |
| --- | --- |
| `k6_fidelity_manifest.yaml` | 协议版本、软件 commit、topic、单位、窗口、阈值、\(\tau_{\mathrm{cal}}\)、相机/视觉配置哈希 |
| `analyze_k6_model_visual_consistency.py` | K6 唯一批处理入口；实现本协议全部主指标和质量标志 |
| `k6_trial_metrics.csv` | 每个 trial 一行的质量、保真度和分层指标 |
| `k6_sensitivity_metrics.csv` | 每个 trial × 每个预注册变体一行 |
| `k6_exclusion_log.csv` | 所有 NA、censored、同步和 replay failure 原因 |
| `k6_protocol_smoke_report.md` | 独立 smoke 数据上的公式单元测试、单位测试、同步测试和 nominal replay 复现结果 |

现有 `validate_slosh_monitor_against_visual.py` 可作为实现起点，但其当前版本会搜索 per-trial best lag、未冻结 K6 窗口且没有 Ferrari signed bias，**不能直接作为 K6-FID-v1.0 的正式入口**。

旧 `compute_ferrari_indices.py` 和 `analyze_ferrari_indices.py` 将绝对误差标作 Ferrari Eq. (25)，与原文不一致，也不得直接生成正式 K6 结果。

当前 `spmpc_paper_core/sections/02_experiments.tex` 仍保留旧的 vision-normalized 分母和未明确的 \(T_e\) 窗口。本轮按“先冻结实验协议、暂不修改论文”的顺序保留该下游草稿；后续同步论文时必须改为 K6-FID-v1.0，不得反向用旧 LaTeX 覆盖本协议。旧视觉流程和历史分析报告中的 +2 s 窗口、`h_mm_smooth_corr` 主曲线或 absolute/model “Eq. (25)” 同样只视为开发记录。

---

## 12. 正式采集前 K6 no-go 检查

- [ ] 协议版本固定为 K6-FID-v1.0，并已链接到实验总章和矩阵速查表；
- [ ] 主要总体固定为 32 次正式 S-MPCC 尝试，K6-S1 至 K6-S4 分层已建立；
- [ ] `/spmpc/slosh_height` topic、mm 单位和 modal-only 配置已验证；
- [ ] RGB 使用 `h_mm_max_lcr_smooth_corr`，30 帧零点和 5 帧滚动中位数已验证；
- [ ] 每帧至少两个标尺有效、clipping 和缺帧标志可导出；
- [ ] \(C_1/C_2\) 均满足 \(f_{\mathrm{cam}}/f_1\geq6\)；
- [ ] \(\mathcal W_{\mathrm{K6}}=[t_{\mathrm{move}},t_{\mathrm{arrival}}+5\,\mathrm{s}]\) 可自动复现；
- [ ] 独立同步标定给出冻结的 \(\tau_{\mathrm{cal}}\)，pair-gap 与 coverage 规则已实现；
- [ ] Ferrari signed bias 使用模型积分作分母，absolute disagreement 单独命名；
- [ ] low-excitation、small-denominator、censored 和 failure 规则已通过单元测试；
- [ ] 参数、初态和延迟敏感性水平与第 7 节完全一致；
- [ ] nominal replay 已通过冻结复现门槛；
- [ ] 唯一正式脚本、依赖、随机种子和软件 commit 已写入 manifest；
- [ ] 独立 smoke 报告已生成，未使用正式 RGB 结果调节任何规则；
- [ ] 确认 K6 不增加实物次数、不修改控制器、不构成安全证明。

任一项未完成，K6 执行状态为 **NO-GO**。这不要求改变当前动力学，但意味着在正式采集前仍需完成分析脚本适配和 smoke 验收。

---

## 13. 版本变更规则

第一条正式 trial 开始后，公式、总体、窗口、信号、同步、质量阈值、敏感性水平和统计规则全部锁定。

若之后发现实现错误：

1. 保留原始输出和错误说明；
2. 升级协议/脚本版本；
3. 从原始数据对全部适用 trial 统一重算；
4. 不得只重算或删除对结论不利的 trial；
5. 若改变模型动力学、权重或在线状态传播，则属于新方法版本，旧数据不得与新版本拼接。

---

## 14. 冻结结论

\[
\boxed{
\begin{gathered}
\text{K6-FID-v1.0 冻结 Ferrari-form signed bias + 独立 absolute disagreement；}\\
\text{固定 32 次 S-MPCC 日志、trial-level 统计和到达后 5 s 窗口；}\\
\text{禁止 per-trial 最佳时滞、幅值拟合、topic 回退和正式数据后调参。}
\end{gathered}
}
\]

该协议冻结的是**如何诚实量化当前低阶模型的适用范围**，而不是预先保证模型会得到有利结果。
