# Slosh-risk Reference Governor 论文写法草稿

日期：2026-07-09

适用分支：`paper/spmpc-cn-draft`

目的：把工程模块 `SloshRiskGovernor` 写成论文可用的方法贡献，同时避免把它误写为硬安全控制器、真实液面闭环或新的安全理论。

## 1. Contributions 小节草稿

本文的贡献限定为以下三点。

1. **液体动态记忆增强的在线 MPCC 局部规划。** 构建一种面向移动底盘开口液体运输的晃液感知局部规划问题，将低阶晃液模态状态与底盘状态、路径进度状态共同放入滚动时域 MPCC，在同一优化问题中联合考虑路径误差、路径推进、底盘控制平滑性和预测晃液响应。

2. **有限时域预测参考适配。** 提出一种 soft Slosh-risk Reference Governor，在 MPCC 求解前基于当前晃液状态和短时域液体响应预测选择速度参考缩放因子，仅对 MPCC 的 `v_ref` 进行上限收缩，而不直接修改底盘 `/cmd_vel`，也不作为硬安全锁使用。

3. **实验与验证协议。** 设计内部消融、普通局部规划器对比、移动底盘防晃近邻对比以及模型-外部液面一致性分析，用于评估显式晃液状态预测和预测参考适配相对于仅平滑运动生成的作用，并防止将内部代理量误写为真实液面结论。

推荐范围句：

> 本文不宣称首次提出晃液感知 MPC，也不处理完整动态避障、同伦/走廊推理、高保真流体仿真、形式化防溢出证明或真实液面闭环观测；内部模型预测的晃液代理量只用于优化和诊断，真实液面结论必须由外部视觉或液位观测支持。

## 2. Method 小节草稿：Slosh-risk Reference Governor

Slosh-risk Reference Governor 是 MPCC 求解前的软参考适配层。给定当前晃液状态、底盘速度、角速度和名义速度参考，它在有限时域内预测不同速度缩放候选对晃液高度代理量的影响，并选择满足风险阈值的最大候选。

候选集合可写为

```tex
\mathcal{B}
=
\left\{
1-\frac{i}{M-1}(1-\beta_{\min})
\ \middle|\ i=0,\ldots,M-1
\right\}
\subseteq[\beta_{\min},1].
```

正式稿中为了适配双栏，也可写成等价的分行形式：

```tex
\beta_i
=1-\frac{i}{M-1}(1-\beta_{\min}),
\quad i=0,\ldots,M-1,
\qquad
\mathcal{B}
=\{\beta_i\}_{i=0}^{M-1}\subseteq[\beta_{\min},1].
```

对每个候选 `beta`，令目标速度为

```tex
v_{\beta}=\beta v_{\mathrm{ref}}^{\mathrm{nom}}.
```

使用低阶晃液模型做 `N_g` 步 rollout，并定义

```tex
R(\beta)
=
\max_{0\leq k\leq N_g}
\frac{H_k(\beta)}{H_{\lim}},
```

其中 `H_k(beta)` 是候选 beta 下第 `k` 步预测的模型晃液高度代理量，`H_lim` 是风险归一化高度阈值。原始候选选择为

```tex
\beta_{\mathrm{raw}}
=
\max\{\beta\in\mathcal{B}\mid R(\beta)\leq \rho\}.
```

若上述集合为空，则取 `beta_raw = beta_min`，并标记为 saturated。该 saturated 状态表示软参考适配已经降到最小候选，但不表示形式化安全停止。

为避免参考突变，对 `beta_raw` 做 rate limit：

```tex
\beta_f
=
\operatorname{clip}
\left(
  \beta_{\mathrm{raw}},
  \max\{\beta_{\min},\beta_f^{-}-\dot{\beta}_{\downarrow}\Delta t\},
  \min\{1,\beta_f^{-}+\dot{\beta}_{\uparrow}\Delta t\}
\right).
```

正式稿中也可先定义上下界：

```tex
\underline{\beta}
=\max\{\beta_{\min},\beta_f^{-}-\dot{\beta}_{\downarrow}\Delta t\},
\qquad
\overline{\beta}
=\min\{1,\beta_f^{-}+\dot{\beta}_{\uparrow}\Delta t\},
\qquad
\beta_f
=\operatorname{clip}
  \left(\beta_{\mathrm{raw}},\underline{\beta},\overline{\beta}\right).
```

受管制速度参考为

```tex
\tilde{v}_{\mathrm{ref}}^{g}
=
\beta_f v_{\mathrm{ref}}^{\mathrm{nom}},
\qquad
v_{\mathrm{ref}}^{g}
=
\min\left\{
  v_{\mathrm{ref}}^{\mathrm{nom}},
  \max\{v_{\min},\tilde{v}_{\mathrm{ref}}^{g}\}
\right\}.
```

若不使用最小巡航速度下限，上式退化为

```tex
v_{\mathrm{ref}}^{g}=\beta_f v_{\mathrm{ref}}^{\mathrm{nom}}.
```

边界句：

> 该模块只改变 MPCC 看到的速度参考 `v_ref_current`，不直接修改求解后的 `/cmd_vel`，不接入 RGB 或真实液面反馈，不改变 OCP 状态维度，也不构成 hard safety latch。

## 3. 简短命题/性质和证明草稿

**命题 1（参考不放大与离散候选最优性）。** 假设 `v_ref_nom >= 0`，候选集合 `B` 满足 `B subset [beta_min, 1]`，且 rate limit 后的 `beta_f in [beta_min, 1]`。则 governor 输出的速度参考满足

```tex
v_{\mathrm{ref}}^{g}\leq v_{\mathrm{ref}}^{\mathrm{nom}}.
```

若可行集合

```tex
\{\beta\in\mathcal{B}\mid R(\beta)\leq\rho\}
```

非空，则 `beta_raw` 是离散候选集合中满足预测风险阈值的最大缩放因子。

**证明草稿。** 由 rate limit 与裁剪操作可得 `beta_f <= 1`，因此 `beta_f v_ref_nom <= v_ref_nom`。最终输出又显式使用 `min{v_ref_nom, ...}`，所以不会超过名义速度参考。另一方面，`beta_raw` 的定义就是在所有满足 `R(beta) <= rho` 的候选中取最大元素，因此可行集合非空时它是离散网格上的最大预测可行候选。

**限制说明。** 如果 rate limit 后 `beta_f > beta_raw`，对 `beta_f` 的再次 rollout 可能超过风险阈值，此时只能称为 transient rate-limited，不能声明满足安全约束。若所有候选都不可行，状态为 saturated 到 `beta_min`，仍是软参考适配，不是安全停止。

## 4. 与已有文献的边界说明

本文的第一贡献不是“首次 slosh-aware MPC”。已有液体防晃研究已经覆盖输入整形、路径设计、离线防晃轨迹优化、预测控制、MPC 跟踪控制、特殊机构抑振和移动平台液体运输。本文应强调的是：在标准轮式移动底盘的在线局部规划层，把低阶晃液模态状态作为动态记忆嵌入 MPCC，并以滚动时域方式输出底盘速度命令。

普通移动机器人 local planner、MPCC、MPPI、DWA、TEB 和 jerk-limited trajectory generation 可以在线生成平滑且可执行的运动，但它们通常只传播机器人状态、路径误差、障碍风险或高阶运动学量，不传播容器内液体模态位移和速度。因此，smooth motion 不等于 slosh-aware motion。

移动底盘液体运输近邻文献与本文共享物理任务，但多数落在固定路径、速度剖面、输入整形、离线整段轨迹优化、参考轨迹跟踪、特殊平台或主动抑振机构层级。本文的定位是 online receding-horizon local planner，而不是 fixed profile 或 preplanned trajectory tracking。

Slosh-risk Reference Governor 的边界也要清楚：它不是新的 CBF、安全不变集、chance constraint 或硬参考 governor 理论，而是一个工程上轻量、可诊断、放在 MPCC 前级的有限时域预测参考适配层。

## 5. 不应声称的内容清单

- 不声称“首次 slosh-aware MPC”。
- 不声称 governor 提供 hard safety、formal spill-free guarantee、CBF 级安全证明或闭环稳定性证明。
- 不声称 saturated 到 `beta_min` 等价于安全停止。
- 不声称 transient rate-limited 状态仍满足风险阈值。
- 不声称 governor 直接修改 `/cmd_vel` 或在执行后拦截命令；它只 cap MPCC 的 `v_ref`。
- 不声称 governor 接入 RGB、真实液面反馈或外部液位闭环。
- 不声称 governor 改变 OCP 状态维度；它不把额外 governor 状态加入 MPCC OCP。
- 不把 `/spmpc/slosh_height` 或模型预测高度代理量直接写成真实液面高度。
- 不声称第一版包含完整 obstacle-aware MPCC、homotopy/corridor、动态避障或局部多拓扑规划。
- 不声称高保真 CFD、FEM、VOF 或数字孪生进入在线规划内核。
- 不把机械臂、SCARA、主动抑振机构或罐车悬架控制写成 Scout Mini 同层 baseline。
- 不用“安全保证”“防溢出保证”“严格可行”描述软参考适配；更合适的说法是“降低预测晃液风险”“收缩速度参考”“软性参考调节”。
