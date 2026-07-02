# SPMPC 论文写作路线：以终稿指南为唯一写作总纲

日期：2026-07-01
目标论文：面向移动底盘开口/半开口液体运输的 SPMPC / slosh-aware MPCC 方法论文
当前范围冻结：第一版暂不纳入 Map-vref、terrain-conditioned slosh experience prior、完整 obstacle-aware MPCC、homotopy/corridor 主方法宣称。

---

## 0. 当前最高优先级定义

后续 SPMPC 论文写作以以下文件作为**唯一写作总纲 / 叙事指南 / 终稿组织指南**：

```text
docs/论文书写/书写思路/SPMPC_终稿写作指南_排版美化版.docx
```

该 docx 的作用是决定：

- 论文主线怎么讲；
- Related Work 怎么组织；
- Method 按什么顺序展开；
- Experiments 应该回答哪些审稿人问题；
- 图表、caption、limitations、future work 如何服务 claim。

但必须明确：

> **该 docx 不是事实来源，不是实验结果来源，也不是文献事实来源。**

因此后续写作采用如下边界：

```text
docx 决定“怎么讲”；
Obsidian 文献笔记和原论文决定“文献事实能不能这么讲”；
代码和 LaTeX 方法草稿决定“我们实际实现了什么”；
实验日志和数据决定“结果能不能这么讲”。
```

---

## 1. 工作源优先级

后续所有 agent 或人工写作都按下面优先级执行。

### 1.1 第一优先级：写作总纲

```text
docs/论文书写/书写思路/SPMPC_终稿写作指南_排版美化版.docx
```

用途：统一论文叙事、章节组织、实验闭环和审稿人视角。

若 agent 不能直接读取 docx，可先用 `python zipfile` 或 `pandoc` 提取文本，但提取后的内容仍只作为写作指南。

### 1.2 第二优先级：当前论文 LaTeX 草稿

```text
docs/论文书写/草稿/spmpc_paper_cn/
```

尤其是：

```text
sections/01_intro.tex
sections/02_related_work.tex
sections/03_method.tex
sections/04_experiments.tex
sections/05_conclusion.tex
```

用途：承载正式论文正文。后续修改应让 LaTeX 草稿逐步向 docx 的终稿指南靠拢。

### 1.3 第三优先级：Obsidian 文献事实库

```text
/data/a/Obsidian/vaults/StudyVault/30-Projects/MPC/参考论文整理
```

用途：核对每篇文献到底做了什么。

特别关注：

```text
00_SPMPC参考论文总览.md
参考关系_初步梳理.md
A_液体晃动建模_估计_防晃控制/
B_移动底盘液体运输_防晃路径设计/
C_机械臂_SCARA_机器人操作防晃/
D_普通移动机器人_local_planner_MPC_轨迹优化/
```

### 1.4 第四优先级：代码、实验日志与真实数据

用途：决定哪些实现和实验结果可以写进论文。

当前特别注意：

- `/spmpc/slosh_height` 是 model proxy，不是真实液面高度；
- 没有实验数据时，只能写实验计划、指标和验证问题，不能写结果性判断；
- 如果使用当前仿真而非 fresh simulation，应标注为诊断，不可混入正式对比。

---

## 2. 当前论文主线

终稿指南与当前草稿共同确认的主线是：

> SPMPC 的论文故事不是“又一个液体防晃方法”，而是把低阶晃液状态前移到标准移动底盘的 online MPCC local-planning layer，在滚动时域中联合优化路径跟踪、路径进度、控制平滑性与预测液体响应。

核心交叉点：

```text
mobile base
+ online local planning
+ explicit slosh-state prediction
```

两条对比线：

```text
ordinary local planners:
  online and executable,
  but generally not liquid-aware

anti-slosh methods:
  liquid-aware,
  but often not standard mobile-base online local planners
```

SPMPC 连接二者：

```text
SPMPC:
  standard WMR / Scout-style mobile base
  + online MPCC local planner
  + path progress optimization
  + low-order slosh-state propagation
  + executable /cmd_vel output
```

---

## 3. Related Work 的最终组织法

后续 Related Work 优先采用 docx 建议的“三段式、双主线”结构，而不是把所有液体晃动论文按时间堆在一起。

### 3.1 第一段：普通移动机器人 online local planner / MPC / MPCC

目的：先建立 SPMPC 所在的“方法层级”。

这一段回答：

> 为什么把问题放在 local planner 层是合理的？

应覆盖：

- DWA；
- TEB；
- LT-DWA；
- mpc_local_planner；
- MPPI / MPC local planning；
- MPCC backbone；
- mobile robot MPCC；
- Regulated Pure Pursuit / DWPP / UTO 等可选扩展；
- local planner benchmark / MRPB 等评价框架。

核心转折句：

> 这些方法在线、可执行、通常能生成平滑轨迹或速度命令，但它们一般不传播液体模态状态，因此无法显式处理被运输液体的动态记忆。

### 3.2 第二段：移动底盘液体运输与 anti-slosh 近邻

目的：说明“同平台 / 同物理问题”的已有工作在哪里，以及为什么仍不同层。

应重点整理：

- Hamaguchi / Taniguchi 系列：path design、velocity profile、input shaping、trace control；
- Lim 2024：mobile robot + spherical pendulum + offline trajectory optimization；
- Nguyen Viet / time-optimal anti-sloshing planning and tracking；
- Prabakaran 2026 / special-platform slosh-aware MPC tracking；
- active vibration reducer、mecanum / omnidirectional liquid transfer 等特殊平台或机构路线。

核心转折句：

> 这些方法证明了移动平台液体运输中显式建模和防晃优化的价值，但多数属于固定路径、速度曲线、离线整段轨迹优化、给定轨迹后的 tracking/control，或特殊平台/机构控制，而不是 standard WMR navigation stack 中的 online MPCC local planner。

### 3.3 第三段：跨平台 anti-slosh 背景

目的：说明液体防晃是广泛问题，但不把机械臂/液罐车写成 Scout Mini 同层 baseline。

应覆盖：

- manipulator / SCARA slosh-free trajectory optimization；
- manipulator tracking / flatness / predictive anti-slosh control；
- tank vehicle、ship、hanging tray、nonprehensile tray 等移动载液或结构路线；
- high-fidelity CFD/FEM/SPH 与 measurement / liquid-level sensing 文献可作为背景插入。

核心边界：

> 这些工作在方法原语上重要，证明了液体模型、轨迹优化和预测控制的价值；但其平台、控制输入、自由度和部署接口不同于标准移动底盘 local planner，因此更适合作为 broader related work，而不是本文同层实验 baseline。

### 3.4 最终定位段

Related Work 最后必须落到：

```text
SPMPC = mobile base + online local planning + explicit slosh-state prediction
```

并主动保护 novelty：

> 本文不声称首次将晃液状态放入 MPC。已有工作已经研究了 slosh-aware predictive control、MPC tracking 和 anti-slosh trajectory optimization。本文的贡献在于把低阶晃液状态嵌入标准轮式移动底盘的 online MPCC local-planning layer。

---

## 4. 文献矩阵更新任务

当前文献矩阵仍然有价值，但后续应按 docx 的终稿结构重排和补强。

主要文件：

```text
docs/论文书写/书写思路/SPMPC_related_work_matrix.md
```

矩阵字段建议保持或扩展为：

| 字段                        | 含义                                                                                                                                   |
| --------------------------- | -------------------------------------------------------------------------------------------------------------------------------------- |
| Paper / Method              | 文献或方法名称                                                                                                                         |
| Platform                    | WMR / manipulator / tank vehicle / special mechanism 等                                                                                |
| Planning level              | online local planning / offline trajectory optimization / tracking control / velocity profile / input shaping / modeling / measurement |
| Liquid model used?          | 是否显式使用液体模型                                                                                                                   |
| Slosh state in controller?  | 是否把晃液状态放入控制/优化状态                                                                                                        |
| Online local planner?       | 是否是标准移动机器人在线局部规划层                                                                                                     |
| Path progress optimization? | 是否优化路径进度 / contouring                                                                                                          |
| Standard WMR baseline?      | 是否适合作为 Scout/WMR 同层 baseline                                                                                                   |
| Role in paper               | same-layer baseline / near-neighbor / broader related work / method foundation / novelty guardrail                                     |

判断规则：

> 一篇文献值不值得详细写，不取决于它是不是“也做液体防晃”，而取决于它是否会让审稿人误判我们的 novelty。

凡是可能引发下面质疑的文献，都要详细比较：

```text
这不就是已有 slosh-aware MPC 吗？
这不就是已有 mobile robot liquid trajectory optimization 吗？
这不就是普通 MPC local planner 加个 smooth cost 吗？
```

---

## 5. Method 写作路线

Method 必须服从 docx 的终稿逻辑：先讲边界，再讲状态、动力学、OCP、约束和命令输出。

当前正式 Method 草稿：

```text
docs/论文书写/草稿/spmpc_paper_cn/sections/03_method.tex
```

Obsidian 方法总览：

```text
/data/a/Obsidian/vaults/StudyVault/30-Projects/MPC/Methods/20260630_SPMPC在线防晃局部规划_Method总览.md
```

推荐 Method 结构：

1. Problem formulation and system overview；
2. Reference path and progress parameterization；
3. Alpha-state mobile-robot dynamics；
4. Low-order slosh modal dynamics；
5. Augmented slosh-aware MPCC model；
6. Objective: tracking, progress, smoothness, and slosh cost；
7. Constraints and command generation；
8. Receding-horizon solution / ROS / acados / SQP-RTI implementation；
9. Method variants for ablation。

Method 必须回答：

- 为什么需要 `s, v_s`？因为这是 MPCC / local planning，不是单纯 tracking；
- 为什么需要 `omega` state 和 `alpha` control？因为转向激励与横向晃动相关；
- 为什么需要 `eta, eta_dot`？因为液体有动态记忆；
- 为什么 slosh proxy 不能当真实液面？因为内部模型传播不等于外部观测；
- 为什么不是 smooth-only？因为相同平滑度下，不同液体相位会产生不同响应。

---

## 6. Experiments 写作路线

当前实验章节应继续保持“框架 / 计划 / 占位”口径，直到数据真实完成。

正式实验草稿：

```text
docs/论文书写/草稿/spmpc_paper_cn/sections/04_experiments.tex
```

根据 docx，终稿实验最好覆盖七组证据链：

| 组别           | 至少应做什么                                                                | 回答的问题                                             |
| -------------- | --------------------------------------------------------------------------- | ------------------------------------------------------ |
| 主对比         | SPMPC vs DWA / TEB / mpc_local_planner / LT-DWA                             | ordinary online local planners 是否足够？              |
| 近邻对比       | SPMPC vs Lim / Hamaguchi-style profile 或 conceptual/supplementary baseline | 同物理问题旧方法与 online local planner 的差距在哪里？ |
| 内部消融       | B0 / B_smooth / B_slosh / B_ours                                            | 收益是不是只是更平滑？                                 |
| 初始相位       | 同一路径、不同 slosh state 初值                                             | 规划器是否真正使用 liquid dynamic memory？             |
| 路径类型       | 直线、L/U 弯、S 曲线 / figure-8                                             | 纵向、横向和耦合激励是否覆盖？                         |
| Pareto         | peak/RMS slosh vs completion time                                           | 是否只是开得更慢？                                     |
| 鲁棒性与实时性 | model mismatch、solver time、success rate                                   | 模型错配和实时性是否可接受？                           |

当前没做完实验时，必须写成：

```text
本节计划验证……
该组实验用于回答……
后续将填入……
```

不能写成：

```text
结果表明……
本文证明……
SPMPC 显著优于……
```

---

## 7. Claim–Gap–Evidence 矩阵

建议继续维护：

```text
docs/论文书写/书写思路/SPMPC_claim_gap_evidence_matrix.md
```

推荐结构：

| Claim                                    | 文献支撑                                       | 方法支撑                  | 实验支撑                    | 审稿人可能质疑     | 应对方式                          |
| ---------------------------------------- | ---------------------------------------------- | ------------------------- | --------------------------- | ------------------ | --------------------------------- |
| 普通 local planner 在线但不 liquid-aware | DWA / TEB / MPC / MPPI / MPCC                  | SPMPC 增强状态            | 外部 baseline 对比          | 调平滑是否足够？   | B_ours vs B_smooth + matched time |
| 液体防晃不是 smooth-only                 | 低阶晃液模型 / input shaping / anti-slosh 文献 | slosh modal state         | 消融 + initial phase        | 是否只是更慢？     | Pareto 图                         |
| 移动底盘液体运输已有研究但不同层         | Hamaguchi / Lim / B11 / B12                    | online MPCC local planner | near-neighbor comparison    | 是否重复已有方法？ | 平台/层级/接口差异                |
| 真实评价不能只看 proxy                   | measurement 文献                               | proxy 与外部观测分离      | RGB max-LCR / LCR           | proxy 是否自证？   | 外部 observer + model mismatch    |
| 第一版 scope 克制                        | current code / method boundary                 | 给定安全参考路径          | failure/limitation analysis | 是否缺完整避障？   | 明确 outside scope                |

---

## 8. 给后续 Agent 的统一要求

后续如果重新调用多个 agent，所有 agent 必须先遵守以下统一规则。

### 8.1 统一输入

每个 agent 的 prompt 都应包含或要求优先读取：

```text
docs/论文书写/书写思路/SPMPC_终稿写作指南_排版美化版.docx
```

并说明：

```text
该 docx 是唯一写作总纲，但不是事实来源。
```

### 8.2 统一事实边界

agent 不得：

- 把 docx 中的实验计划写成已完成结果；
- 根据 docx 编造文献书目信息；
- 把机械臂/液罐车方法写成 Scout Mini 同层 baseline；
- 声称 SPMPC 首次将晃液状态放入 MPC；
- 把 `/spmpc/slosh_height` 写成真实液面高度；
- 声称已有完整 obstacle-aware / homotopy / corridor SPMPC 主方法；
- 声称已有 formal spill-free guarantee。

agent 必须：

- 对文献事实回查 Obsidian 笔记或原论文；
- 对方法事实回查 LaTeX `03_method.tex`、代码或方法笔记；
- 对实验结论回查实际实验日志或结果文件；
- 不确定时标注“待核验 / 待实验 / 待补数据”。

### 8.3 建议分工

#### Agent A：Related Work 重排

目标：按 docx 三段式重排 Related Work。

输出候选：

```text
docs/论文书写/书写思路/SPMPC_related_work_rewrite_plan.md
```

要求：

- 第一段 local planner / MPC / MPCC；
- 第二段 mobile-base liquid anti-slosh near-neighbors；
- 第三段 cross-platform anti-slosh background；
- 最后落到 SPMPC positioning；
- 不直接改 LaTeX，先给 rewrite plan。

#### Agent B：D 组 local planner 文献补强

目标：检查普通 local planner 谱系是否足够支撑终稿。

重点：

- DWA；
- TEB；
- mpc_local_planner；
- MPCC backbone；
- mobile robot MPCC；
- MRPB benchmark；
- RPP / DWPP / UTO 可选。

输出：

```text
docs/论文书写/书写思路/SPMPC_local_planner_literature_gap.md
```

#### Agent C：实验计划对齐 docx

目标：把当前 `04_experiments.tex` 与 docx 七组实验闭环对齐。

输出：

```text
docs/论文书写/书写思路/SPMPC_experiment_plan_docx_aligned.md
```

要求：

- 只写计划和指标，不写结果；
- 明确哪些实验已具备基础、哪些待做；
- 明确 fresh-sim、common limits、60s timeout、proxy vs RGB 边界；
- 标出哪些可以主文，哪些适合 appendix。

#### Agent D：Method 一致性检查

目标：检查 Obsidian method note、LaTeX `03_method.tex` 和 docx 是否一致。

输出：

```text
docs/论文书写/书写思路/SPMPC_method_consistency_check.md
```

要求：

- 检查 alpha-state、slosh dynamics、OCP、constraints、cmd_vel 输出是否一致；
- 检查是否有过度宣称；
- 检查是否需要补 ROS/acados/SQP-RTI 实现描述。

#### Agent E：文献事实核验

目标：核验 docx 中建议追加的外部文献是否真实、是否适合正文。

输出：

```text
docs/论文书写/书写思路/SPMPC_docx_extra_references_check.md
```

要求：

- 不要把 docx 建议文献直接加入正文；
- 核验标题、作者、年份、venue、DOI/arXiv；
- 标注：正文必需 / 可选背景 / appendix / 暂不建议。

---

## 9. 不可过度宣称清单

第一版论文不建议宣称：

- 首次将晃液状态放入 MPC；
- 完整闭环稳定性证明；
- 全局最优；
- formal spill-free guarantee；
- 完整 obstacle-aware MPCC / homotopy / corridor 主方法；
- Map-vref / terrain-conditioned slosh prior 已纳入主方法；
- high-fidelity CFD/FEM/SPH 被纳入在线规划内核；
- `/spmpc/slosh_height` 是真实液面高度；
- 机械臂、液罐车或船舶方法是 Scout Mini 同层实验 baseline；
- 尚未完成的 Pareto、model mismatch、initial phase 或实物 RGB 实验已经证明了结论。

建议主方法宣称保持为：

> deterministic slosh-aware alpha-state MPCC local planner with modal slosh-state augmentation and real-time receding-horizon command generation for mobile-base open-liquid transport.

---

## 10. 最终主线一句话

后续所有写作围绕这句话展开：

> 普通移动机器人 local planner 能在线生成可执行运动，但通常不传播液体动态状态；已有 anti-slosh 方法显式考虑液体晃动，但多数工作在离线轨迹、速度曲线、给定轨迹跟踪或特殊平台层。SPMPC 面向标准轮式移动底盘，在 online MPCC local-planning layer 中嵌入低阶晃液状态，在滚动时域内联合优化路径跟踪、路径进度、底盘控制平滑性和预测液体响应，并输出普通 `/cmd_vel` 命令。

---

## 11. 后续执行原则

一句话：

> docx 统一论文怎么讲；Obsidian 和原论文统一文献事实；代码和 Method 草稿统一实现事实；实验日志统一结果事实。

后续重写任一章节时，先对照 docx，再回查事实来源，最后再写入 LaTeX。
