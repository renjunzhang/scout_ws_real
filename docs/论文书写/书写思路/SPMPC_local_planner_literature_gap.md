# Agent B：D 组 local planner 文献补强与缺口检查

日期：2026-07-01  
角色：Agent B — D 组 ordinary mobile-robot local planner / MPC / MPCC 谱系检查  
输入优先级：遵循 `SPMPC论文写作路线_文献优先.md`：docx 决定写作组织，Obsidian/原论文决定文献事实，LaTeX 草稿决定当前正文状态。

---

## 1. 结论摘要

当前 D 组文献已经覆盖了若干现代 local planner / MPC / MPPI / trajectory-generation 方向，例如 RPP、LT-DWA、MPPI、UTO、TOPPRA、Ruckig 等；这些足以支撑“普通 local planner 能在线、平滑、可执行，但默认不传播液体动态状态”的主论点。

但如果终稿 Related Work 第一段要承担 docx 要求的“先建立 SPMPC 所在方法层级”这一任务，当前谱系还缺少几类 canonical anchor：

1. **classic DWA**：需要用 Fox--Burgard--Thrun 1997 作为 DWA 家族源头，而不是只引用 LT-DWA。
2. **TEB**：需要 canonical TEB / distinctive-topology trajectory optimization 论文支撑 ROS TEB baseline。
3. **mpc_local_planner / ordinary NMPC local planner**：需要单独引用 ROS `mpc_local_planner` 对应的 NMPC local-planning paper，而不能只泛称 MPC。
4. **MPCC backbone**：需要引用 contouring-error / progress-optimization 的 canonical MPCC 文献，解释 SPMPC 的 `s, v_s` 和 contouring/lag/progress 思路不是凭空出现。
5. **mobile-robot MPCC**：需要引用普通移动机器人/动态环境中的 MPCC local planning，避免把 novelty 误写成“首次把 MPCC 用于 mobile robot local planning”。
6. **local-planner benchmark**：需要 MRPB 作为 baseline 选择与评价维度的 benchmark 参考。

已在 `docs/论文书写/草稿/spmpc_paper_cn/references.bib` 中补入 D15--D21 作为这些 anchor 的 BibTeX 条目。

---

## 2. 已补入 references.bib 的 canonical entries

| 新 key | 文献 | 论文角色 | 在 SPMPC 中的用法 |
|---|---|---|---|
| `D15_Fox1997` | Fox, Burgard, Thrun, *The Dynamic Window Approach to Collision Avoidance*, 1997 | classic DWA source | 支撑 DWA 是 velocity-space online local planner 家族的经典源头；与 LT-DWA 一起组成 DWA 线。 |
| `D16_Rosmann2017` | Rösmann, Hoffmann, Bertram, *Integrated Online Trajectory Planning and Optimization in Distinctive Topologies*, 2017 | TEB canonical paper | 支撑 TEB 作为 optimization-based online local planner baseline；强调其优化机器人轨迹拓扑/动态约束，但无液体模态状态。 |
| `D17_Rosmann2021` | Rösmann, Makarow, Bertram, *Online Motion Planning based on Nonlinear Model Predictive Control with Non-Euclidean Rotation Groups*, 2021 | `mpc_local_planner` / ordinary NMPC local-planner anchor | 支撑 ordinary NMPC local planner baseline；说明 MPC-style local planning 已成熟，但默认状态/代价仍是机器人中心的。 |
| `D18_Lam2013` | Lam, Manzie, Good, *Model Predictive Contouring Control for Biaxial Systems*, 2013 | MPCC contouring-control foundation | 支撑 contouring/lag error 与路径进度优化的 MPCC 基础概念。 |
| `D19_Liniger2015` | Liniger, Domahidi, Morari, *Optimization-Based Autonomous Racing of 1:43 Scale RC Cars*, 2015 | nonlinear MPCC / autonomous-racing backbone | 支撑路径进度、轮式/车辆动力学、receding-horizon contouring control 的成熟性。 |
| `D20_Brito2019` | Brito, Floor, Ferranti, Alonso-Mora, *Model Predictive Contouring Control for Collision Avoidance in Unstructured Dynamic Environments*, 2019 | mobile-robot MPCC / dynamic-environment anchor | 防止过度宣称；可写成“MPCC 已用于 mobile-robot collision avoidance/local planning，但未传播 transported-liquid modal state”。 |
| `D21_Wen2021` | Wen et al., *MRPB 1.0: A Unified Benchmark for the Evaluation of Mobile Robot Local Planning Approaches*, 2021 | local-planner benchmark | 支撑 baseline 选择、评价指标与 local planner benchmark 背景。 |

---

## 3. 当前 D 组覆盖与缺口

### 3.1 已覆盖且可保留

当前 Obsidian D 组已有内容仍有价值：

- `D01_Macenski2023`：RPP，可作为 lightweight tracking / supplementary baseline。
- `D02_Jian2023`：LT-DWA，可作为 modern DWA-family external baseline。
- `D03_Zhang2025`：UTO，可作为差速机器人 trajectory optimization 相关工作。
- `D06_Williams2017`、`D07_Trevisan2025`：MPPI / risk-aware MPPI，可作为 sampling-based MPC local-planning family。
- `D12_Pham2017`、`D13_Berscheid2021`：TOPPRA / Ruckig，可用于 fixed-path speed-profile 或 smooth-only supplementary baseline。
- `D14_Jian2023`：D-CBF-MPC，适合作 narrative reference / dynamic-obstacle MPC guardrail，不应和 D02 LT-DWA 混淆。

### 3.2 主要缺口

| 缺口 | 为什么重要 | 修复状态 |
|---|---|---|
| 只有 LT-DWA、没有 classic DWA | 审稿人通常会把 DWA 识别为 classic local planner family；只 cite LT-DWA 会显得谱系不完整。 | 已补 `D15_Fox1997`。 |
| 没有 TEB canonical entry | TEB 是 ROS local-planner baseline 常见对照；若实验中出现 TEB，Related Work 和 references 必须有正式来源。 | 已补 `D16_Rosmann2017`。 |
| 没有 `mpc_local_planner` 对应普通 NMPC local-planner entry | 论文主线强调 SPMPC 是 slosh-aware MPCC/local MPC；必须有 ordinary MPC local planner 参照物。 | 已补 `D17_Rosmann2021`。 |
| 没有 MPCC foundation | Method 中的 path progress / contouring 需要引用 backbone；否则 SPMPC 的 MPCC 层级缺少事实支点。 | 已补 `D18_Lam2013`、`D19_Liniger2015`。 |
| 没有 mobile-robot MPCC guardrail | 需要避免写成“首次把 MPCC 用到 mobile robot”；novelty 应是 slosh-state augmentation in online local-planning layer。 | 已补 `D20_Brito2019`。 |
| 没有 benchmark reference | 实验章节需要解释为什么选 DWA/TEB/MPC 等 baseline、指标如何设计。 | 已补 `D21_Wen2021`。 |

---

## 4. 推荐写入 Related Work 第一段的逻辑

建议终稿第一段不再只列“DWA / TEB / MPC / MPPI”，而是按方法层级展开：

1. **Velocity-space local planning**：classic DWA → LT-DWA。
2. **Graph / optimization-based local planning**：TEB。
3. **Ordinary MPC / NMPC local planning**：`mpc_local_planner` / NMPC motion planning。
4. **MPPI / sampling-based MPC**：MPPI family。
5. **MPCC / path-progress optimization**：Lam / Liniger / Brito 等 MPCC backbone。
6. **Benchmark / evaluation**：MRPB。

可用的正文骨架如下：

```tex
Classic velocity-space local planners such as DWA \cite{D15_Fox1997} and its long-horizon variants \cite{D02_Jian2023},
optimization-based planners such as TEB \cite{D16_Rosmann2017},
ordinary NMPC local planners \cite{D17_Rosmann2021},
and sampling-based MPC/MPPI methods \cite{D06_Williams2017,D07_Trevisan2025}
can generate online, feasible, and smooth mobile-robot commands.
MPCC formulations further introduce contouring/lag errors and path-progress optimization \cite{D18_Lam2013,D19_Liniger2015},
and have also been used for mobile-robot collision avoidance in dynamic environments \cite{D20_Brito2019}.
Benchmarks such as MRPB summarize common evaluation practice for mobile-robot local planning \cite{D21_Wen2021}.
However, these ordinary local planners usually optimize robot states, path geometry, collision risk, progress, and control smoothness, rather than propagating transported-liquid modal states inside the receding horizon.
```

中文对应逻辑：

> DWA、TEB、ordinary NMPC local planner、MPPI 和 MPCC 已经能在线生成可执行、平滑、避障或路径进度优化的移动机器人运动；但它们的预测状态通常围绕机器人位姿、速度、障碍风险、路径误差和控制平滑性构建，并不包含液体模态位移/速度。因此，它们是 SPMPC 的同层 ordinary local-planner baseline，而不是 slosh-aware planner。

---

## 5. 推荐 baseline 分层

### 5.1 主文 direct external baselines

| Baseline | 建议引用 | 角色 |
|---|---|---|
| DWA | `D15_Fox1997`；若实验用 LT-DWA，再加 `D02_Jian2023` | classic / modern velocity-space local planner。 |
| TEB | `D16_Rosmann2017` | optimization-based ROS local planner。 |
| `mpc_local_planner` / ordinary NMPC | `D17_Rosmann2021` | 最接近“普通 MPC local planner”的同层对照。 |
| LT-DWA | `D02_Jian2023` | 已在 Obsidian D 组中，适合作 modern DWA-family baseline。 |

### 5.2 主文或 appendix 的 method-foundation references

| Reference | 建议引用 | 角色 |
|---|---|---|
| MPCC contouring foundation | `D18_Lam2013` | 解释 contouring/lag error 与 progress variable。 |
| Autonomous-racing MPCC | `D19_Liniger2015` | 支撑 nonlinear receding-horizon MPCC / path-progress optimization。 |
| Mobile-robot MPCC collision avoidance | `D20_Brito2019` | novelty guardrail：MPCC local planning 已存在，SPMPC 的新增点是 liquid modal state。 |

### 5.3 Supplementary / optional references

| Reference | 当前状态 | 用法 |
|---|---|---|
| RPP | 已有 `D01_Macenski2023` | 如果实验加入轻量 tracking baseline，可引用。 |
| MPPI | 已有 `D06_Williams2017` / `D07_Trevisan2025` | 可作为 ordinary MPC/MPPI family，而不一定必须实作 baseline。 |
| TOPPRA / Ruckig | 已有 `D12_Pham2017` / `D13_Berscheid2021` | 用于 smooth-only 或 speed-profile supplementary baseline，证明 smooth-only 不等于 slosh-aware。 |
| UTO | 已有 `D03_Zhang2025` | 可作为差速机器人 trajectory optimization 相关工作。 |
| MRPB | 新增 `D21_Wen2021` | 实验指标和 benchmark 背景。 |

---

## 6. 对当前 LaTeX 草稿的具体建议

当前 `sections/02_related_work.tex` 中普通 local planner 小节已经表达了正确主线：ordinary local planners are online but not liquid-aware。但引用还偏向现有 D01--D13，缺少 classic/canonical anchors。

建议后续修改时至少更新该句：

```tex
路径跟踪器、DWA 类局部规划器、MPC/MPPI、trajectory optimization、risk-aware motion planning 和 jerk-limited trajectory generation 已经能够在线生成可行、平滑且可执行的运动命令\cite{D01_Macenski2023,D02_Jian2023,D03_Zhang2025,D06_Williams2017,D07_Trevisan2025,D13_Berscheid2021}。
```

改为类似：

```tex
Classic velocity-space local planners such as DWA\cite{D15_Fox1997} and modern long-horizon variants\cite{D02_Jian2023}, optimization-based planners such as TEB\cite{D16_Rosmann2017}, ordinary NMPC local planners\cite{D17_Rosmann2021}, MPPI-style predictive control\cite{D06_Williams2017,D07_Trevisan2025}, and jerk-limited online trajectory generation\cite{D13_Berscheid2021} can generate feasible, smooth, and executable robot commands online.
```

如果保留中文稿，可写成：

```tex
经典 DWA 及其长时域变体\cite{D15_Fox1997,D02_Jian2023}、TEB 等优化式局部规划器\cite{D16_Rosmann2017}、普通 NMPC local planner\cite{D17_Rosmann2021}、MPPI 类预测控制方法\cite{D06_Williams2017,D07_Trevisan2025}以及 jerk-limited 在线轨迹生成\cite{D13_Berscheid2021}，已经能够在线生成可行、平滑且可执行的移动机器人运动命令。
```

随后补一句 MPCC backbone：

```tex
MPCC further introduces contouring/lag errors and path-progress optimization as a receding-horizon path-following formulation\cite{D18_Lam2013,D19_Liniger2015}, and has been applied to mobile-robot collision avoidance in dynamic environments\cite{D20_Brito2019}.
```

再接当前 gap 句：

```tex
However, these planners typically reason over robot pose, velocity, path geometry, collision risk, control smoothness, and path progress, rather than propagating transported-liquid modal displacement and velocity states inside the planning horizon.
```

---

## 7. Obsidian D 组建议回填项

为了让 Obsidian 文献事实库与 `references.bib` 同步，建议后续在

```text
/data/a/Obsidian/vaults/StudyVault/30-Projects/MPC/参考论文整理/D_普通移动机器人_local_planner_MPC_轨迹优化/
```

增加或回填以下 note：

1. `D15 The Dynamic Window Approach to Collision Avoidance.md`
2. `D16 Integrated Online Trajectory Planning and Optimization in Distinctive Topologies.md`
3. `D17 Online Motion Planning based on Nonlinear Model Predictive Control with Non-Euclidean Rotation Groups.md`
4. `D18 Model Predictive Contouring Control for Biaxial Systems.md`
5. `D19 Optimization-Based Autonomous Racing of 1-43 Scale RC Cars.md`
6. `D20 Model Predictive Contouring Control for Collision Avoidance in Unstructured Dynamic Environments.md`
7. `D21 MRPB 1.0 A Unified Benchmark for the Evaluation of Mobile Robot Local Planning Approaches.md`

建议每个 note 都按同一矩阵字段标注：

- Planner type；
- Online/offline；
- State/control/objective；
- Liquid model?；
- Slosh state in controller?；
- Online local planner?；
- Path progress optimization?；
- Standard WMR baseline?；
- Role in paper。

---

## 8. 不应过度宣称的边界

补强 D 组后，Related Work 可以更稳地承认以下事实：

- DWA/TEB/MPC/MPPI/MPCC 已经是成熟 online local planning 或 path-following family；
- MPCC 与 path-progress optimization 不是 SPMPC 首创；
- mobile-robot MPCC / collision avoidance 已经存在；
- SPMPC 的 novelty 不在普通 MPCC backbone，而在 **standard WMR online MPCC local-planning layer + low-order transported-liquid modal state propagation + slosh-aware cost/constraints/cmd_vel output**。

推荐保护句：

> 本文不声称首次提出 DWA/TEB/MPC/MPCC 局部规划，也不声称首次将预测控制用于晃液抑制。本文关注的是两条谱系之间的交叉缺口：ordinary mobile-robot local planners are online and executable but generally not liquid-aware, while anti-slosh methods are liquid-aware but often not standard online local planners. SPMPC fills this gap by augmenting an MPCC-style local planner with low-order slosh-state prediction for mobile-base open-liquid transport.

---

## 9. 后续最小改动清单

1. 已完成：在 `references.bib` 中加入 `D15_Fox1997`--`D21_Wen2021`。
2. 建议下一步：更新 `sections/02_related_work.tex` 的普通 local planner 小节 citations。
3. 建议下一步：把 Agent A 的 Related Work rewrite plan 中第一段改成 local planner / MPC / MPCC 先行结构。
4. 建议下一步：回填 Obsidian D 组 note 与 D matrix，避免之后 agent 只看到 D01--D14 而遗漏 canonical anchors。
5. 建议下一步：在实验计划中用 `D21_Wen2021` 支撑 local-planner benchmark/metrics 背景，但不要把尚未完成的结果写成结论。
