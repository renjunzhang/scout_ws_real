# SPMPC Related Work Rewrite Plan（Agent A）

日期：2026-07-01  
角色：Agent A / Related Work 重排  
目标文件（后续才改）：`docs/论文书写/草稿/spmpc_paper_cn/sections/02_related_work.tex`  
本文件只给 rewrite plan，不直接修改 LaTeX 正文。

---

## 0. 已核对材料与事实边界

本轮已核对：

1. 写作路线总纲：
   - `docs/论文书写/书写思路/SPMPC论文写作路线_文献优先.md`
2. 终稿指南 docx 的 Related Work 要求：
   - `docs/论文书写/书写思路/SPMPC_终稿写作指南_排版美化版.docx`
   - 核心要求：Related Work 采用“三段式”：
     1. 同层 ordinary local planner / MPC / MPCC；
     2. 移动底盘 liquid anti-slosh 近邻；
     3. 跨平台 anti-slosh 背景。
3. 当前 LaTeX 草稿：
   - `docs/论文书写/草稿/spmpc_paper_cn/sections/02_related_work.tex`
4. 当前文献矩阵：
   - `docs/论文书写/书写思路/SPMPC_related_work_matrix.md`
   - `docs/论文书写/书写思路/SPMPC_claim_gap_evidence_matrix.md`
5. 当前 Obsidian 文献库：
   - `/data/a/Obsidian/vaults/StudyVault/30-Projects/MPC/参考论文整理`
6. 当前 BibTeX：
   - `docs/论文书写/草稿/spmpc_paper_cn/references.bib`

事实边界：

- docx 决定“怎么讲”，不是文献事实来源。
- Obsidian 笔记与 `references.bib` 决定当前可引用的文献事实。
- 不宣称 SPMPC 首次把晃液状态放入 MPC。
- 不把机械臂、液罐车、船舶或特殊机构方法写成 Scout/WMR 同层 baseline。
- 不把 `/spmpc/slosh_height` 写成真实液面高度。
- 第一版不宣称完整 obstacle-aware / homotopy / corridor MPCC，也不宣称 formal spill-free guarantee。

---

## 1. 当前 `02_related_work.tex` 的诊断

当前草稿已经有较完整的事实基础，尤其是：

- 已经明确区分 model-predicted slosh proxy 与真实液面观测。
- 已经承认 slosh-aware MPC / predictive control 已存在。
- 已经把 Hamaguchi 系列、Lim 2024、B11、B12 放在移动底盘近邻层级比较。
- 已经指出 ordinary local planners 是 `online but not liquid-aware`。
- 已有移动底盘近邻层级表和文献定位图占位。

但它与终稿指南的最终组织法仍有三个结构差异：

### 1.1 开头顺序不符合 docx 的“三段式”

当前草稿开头先写：

1. 晃液建模、估计与测量；
2. 输入整形、防晃轨迹生成与预测控制；
3. 移动底盘液体运输；
4. 普通 local planner。

终稿指南要求优先建立本文所在的“方法层级”：

```text
ordinary online local planner / MPC / MPCC
→ mobile-base liquid anti-slosh near-neighbors
→ cross-platform anti-slosh background
→ SPMPC positioning
```

因此建议把 ordinary local planner 提到第一段/第一小节，而不是放到最后。

### 1.2 高保真建模、测量、机械臂文献目前展开较多

这些文献有用，但在终稿 Related Work 中不宜压过主线。建议：

- A01/A02：保留，因为直接支撑低阶晃液状态。
- A12/C05：保留，因为它们是 novelty guardrail，说明 slosh-aware MPC 已存在。
- A03/A04/A05/A06：压缩成一句“高保真模型存在但不适合在线 local planner 内核”。
- A08/A09/A10/A11：主要移到实验指标/评价边界中；Related Work 中最多一句。
- C01-C07：不再逐篇展开；选 C02/C03/C05/C06 或 docx 额外建议的 GSFT/非抓取托盘线作代表，强调“方法原语接近但平台/接口不同”。

### 1.3 local planner 文献谱系需要由 Agent B 补强

当前 `references.bib` 已有：

- `D01_Macenski2023`：Regulated Pure Pursuit；
- `D02_Jian2023`：LT-DWA；
- `D03_Zhang2025`：UTO / differential-drive trajectory optimization；
- `D06_Williams2017`：MPPI；
- `D13_Berscheid2021`：Ruckig / jerk-limited trajectory generation。

但终稿指南还希望覆盖：

- classic DWA；
- TEB；
- `mpc_local_planner`；
- MPCC backbone；
- mobile robot MPCC；
- local planner benchmark / MRPB 等。

这些应交给 Agent B 或文献事实核验后再加入，不应由 Agent A 在未核验 bib metadata 时凭空写入 citation key。

特别注意：

- `D02_Jian2023` 是 LT-DWA。
- `D14_Jian2023` 是 ICRA 2023 D-CBF-MPC 叙事参考。
- 两者不可混同；Related Work 若引用 LT-DWA 用 `D02_Jian2023`，若只借鉴叙事结构则一般不必在正文 Related Work 中引用 `D14_Jian2023`。

---

## 2. 推荐终稿结构

建议将 Related Work 重写为 3 个主小节 + 1 个定位小结。

```latex
\section{相关工作}

\subsection{Online local planning and trajectory optimization for mobile robots}
...

\subsection{Mobile-base liquid transport and anti-slosh planning}
...

\subsection{Cross-platform slosh modeling and anti-slosh control}
...

\subsection{Positioning of SPMPC}
...
```

如果版面紧张，最后一节也可以不是 `\subsection`，而是在第三小节末尾用一段 `In summary` 收束。

---

## 3. 第一段：ordinary local planner / MPC / MPCC

### 3.1 目的

先建立 SPMPC 的方法层级：本文不是一个离线轨迹规划器，也不是专用 transfer controller，而是一个标准移动底盘 local-planning layer 中的 online receding-horizon planner。

这一段应回答：

> 为什么把问题放在 local planner 层是合理的？

### 3.2 应覆盖文献

当前可直接用的 citation keys：

- `D01_Macenski2023`：Regulated Pure Pursuit；
- `D02_Jian2023`：LT-DWA；
- `D03_Zhang2025`：differential-drive trajectory optimization / UTO；
- `D06_Williams2017`：MPPI；
- `D13_Berscheid2021`：jerk-limited real-time trajectory generation。

待 Agent B 补强/核验后加入：

- classic DWA；
- TEB；
- `mpc_local_planner`；
- MPCC backbone / mobile robot MPCC；
- local planner benchmark / MRPB。

### 3.3 推荐写法要点

应写：

- local planner 的任务是在滚动时域内生成短时可执行轨迹或速度命令；
- DWA/RPP/LT-DWA/MPC/MPPI/trajectory optimization 等方法已经能处理平滑性、kinodynamic feasibility、障碍、安全或风险；
- 它们适合作为外部 baseline，因为部署层级相同或接近；
- 但它们通常不把 transported liquid 的 modal state 作为预测状态，也不显式建模液体动态记忆。

关键转折句：

> These planners are online and executable, but their prediction states and objectives are typically robot-centric or obstacle-centric. Without propagating liquid modal states, they cannot distinguish control sequences that are similarly smooth but interact differently with the current sloshing phase.

### 3.4 建议保留的 claim

中文主线：

> 普通 local planner 解决的是“机器人怎么在线、可执行、平滑、安全地走”；SPMPC 额外解决的是“机器人携带的液体在当前动态相位下会如何响应”。

---

## 4. 第二段：mobile-base liquid anti-slosh near-neighbors

### 4.1 目的

这是 Related Work 的核心 gap 段。它应说明：已有研究确实处理了“移动底盘/移动平台 + 液体运输”，但多数不在 standard WMR online local planner 层。

### 4.2 应覆盖文献

核心必写：

- `B04_Hamaguchi2002`：curved path design / transfer control；
- `B03_Hamaguchi2004`：velocity control and path design；
- `B02_Hamaguchi2005`：path design and trace control；
- `B01_Lim2024`：mobile robot + spherical pendulum + offline trajectory optimization；
- `B11_Viet2025`：time-optimal planning + anti-sloshing control + CBF/tracking；
- `B12_Prabakaran2026`：special-platform slosh-aware MPC tracking/control。

可选短提：

- `B05_Hamaguchi2019`、`B06_Hamaguchi2018`：active vibration reducer / special mechanism；
- `B07_Nguyen2024`：mecanum platform liquid transfer；
- `B09_Pang2026`：tank vehicle active suspension / special vehicle control。

### 4.3 推荐分类方式

不要按年份堆叠，应按 planning/control layer 分：

| 类别 | 代表文献 | 与 SPMPC 的差异 |
|---|---|---|
| Fixed path / velocity profile / input shaping | `B04`, `B03`, `B02` | 预先设计路径或速度剖面，不在每个周期求解 slosh-aware local OCP |
| Offline slosh-constrained trajectory optimization | `B01` | 显式液体模型，但整段离线优化，不是 online local planner |
| Time-optimal planning + robust tracking / CBF | `B11` | 更偏 reference planning + tracking/control，不是 navigation stack local planner |
| Special-platform MPC tracking/control | `B12` | 证明 slosh-aware MPC tracking 已存在，但不是 standard WMR online MPCC |
| Active vibration reducer / special mechanism | `B05`, `B06` | 依赖额外机构或特殊平台，不是普通底盘软件规划层 |

### 4.4 推荐核心句

> These mobile-base and mobile-container studies demonstrate the importance of liquid-aware modeling and anti-slosh optimization. However, they are mostly formulated as fixed path/profile design, offline full-trajectory optimization, reference planning followed by tracking, or special-platform/mechanism control. They do not directly provide an online MPCC-style local planner for a standard wheeled mobile robot that repeatedly optimizes path progress, chassis commands, and predicted liquid response in the same horizon.

### 4.5 表格建议

保留当前 `table` 的想法，但把表格标题收紧为：

```latex
\caption{Planning/control layers of mobile-base liquid-transport studies relative to SPMPC.}
```

表格列建议为：

| Work family | Representative papers | Planning/control layer | Main gap relative to SPMPC |
|---|---|---|---|

不建议在主文表格中过度展开平台细节；详细矩阵留在写作笔记或 appendix。

---

## 5. 第三段：cross-platform anti-slosh background

### 5.1 目的

这一段不是为了找 baseline，而是为了保护 novelty 边界：

- 液体低阶建模已有；
- anti-slosh trajectory optimization 已有；
- slosh-aware predictive/MPC control 已有；
- 但这些工作的平台、控制输入、自由度和部署接口不同于 standard WMR local planner。

### 5.2 应覆盖文献

低阶模型 / 方法基础：

- `A01_Leva2021`：2D excitation modal slosh estimation；
- `A02_Guagliumi2021`：simple model-based sloshing estimation。

predictive/MPC anti-slosh guardrail：

- `A12_Okatsuka2011`：MPC in liquid container transfer system；
- `C05_Okatsuka2012`：BEM + predictive control。

机械臂 / 操作防晃代表：

- `C02_Muchacho2022`：slosh-free robot trajectory optimization；
- `C03_Moriello2018`：manipulating liquids with robots；
- `C06_Viet2024`：flatness-based trajectory planning + tracking control。

传统 input shaping / smooth-only 对照：

- `A07_Singer1990`。

可选压缩背景：

- `A03/A04/A05/A06`：高保真模型存在但不适合 online planner 内核；
- `A08/A09/A10/A11`：测量/视觉评价存在，但更适合实验章节。

### 5.3 推荐写法

此段应短，不要写成第二篇综述。建议按三句话展开：

1. 低阶模态/等效模型为 online horizon 内传播液体状态提供基础。
2. predictive/MPC anti-slosh 与机械臂/专用 transfer system 证明了显式液体模型的价值，因此本文不宣称首次提出 slosh-aware MPC。
3. 这些工作多数输出关节/末端/专用机构命令，或处理给定轨迹 tracking；其接口不同于标准轮式底盘 local planner 的 `/cmd_vel` 输出。

关键句：

> These works are important methodological foundations, but their actuation spaces and deployment interfaces differ from standard mobile-base local planning. Therefore, they serve as broader related work and novelty guardrails rather than same-layer baselines.

---

## 6. 最终定位段

Related Work 最后应明确落到：

```text
SPMPC = standard WMR / mobile base
      + online MPCC local planning
      + explicit low-order slosh-state propagation
      + path-progress optimization
      + executable chassis command output
```

建议中文收束：

> 综上，已有文献分别建立了低阶晃液建模、防晃轨迹优化、slosh-aware predictive control、移动底盘液体运输以及普通移动机器人在线局部规划的基础。本文不声称首次研究液体防晃，也不声称首次将晃液状态纳入 MPC。本文的定位是在标准轮式移动底盘的 online MPCC local-planning layer 中嵌入低阶液体模态状态，使规划器在滚动时域内联合权衡路径跟踪、路径进度、底盘控制平滑性与预测液体响应，并输出普通底盘可执行命令。

建议英文收束：

> Existing studies separately establish low-order slosh modeling, anti-slosh predictive control, mobile-base liquid transport, and advanced mobile-robot local planning. The intersection targeted in this work is different: an online receding-horizon MPCC local planner for a standard wheeled mobile robot that explicitly propagates liquid modal states and jointly optimizes path progress, tracking performance, chassis commands, smoothness, and predicted slosh response.

---

## 7. 从当前草稿到终稿结构的具体搬运方案

### 7.1 当前 `晃液建模、估计与测量`

当前位置：`02_related_work.tex` 第一个小节。  
建议处理：拆分并压缩到第三段。

- A01/A02：保留到第三段开头。
- A03/A04/A05/A06：合并成一句高保真背景。
- A08/A09/A10/A11：移出主线，放到实验评价或仅一句。

### 7.2 当前 `输入整形、防晃轨迹生成与预测控制`

建议处理：拆入第三段。

- A07：作为 input shaping / smooth-only 背景。
- C01-C07：选 2-3 篇代表，不逐篇长列。
- A12/C05：保留为 novelty guardrail。

### 7.3 当前 `移动底盘液体运输与防晃路径设计`

建议处理：基本保留，提前成为第二段。

- Hamaguchi 系列历史线保留，但压缩成一个谱系句。
- Lim / B11 / B12 作为强近邻重点比较。
- special mechanism 短提，不要展开。
- 当前表格可保留，但标题与列名按 planning/control layer 收紧。

### 7.4 当前 `普通移动机器人局部规划与轨迹优化`

建议处理：提到第一段，并补强谱系。

- 先写 local planner 社区共识。
- 再写 DWA/TEB/RPP/LT-DWA/MPC/MPPI/MPCC 等。
- 最后转折到 liquid modal state 缺失。

### 7.5 当前 `小结：SPMPC 的定位`

建议处理：保留并加强。

- 明确三不宣称：不是首个 anti-slosh；不是首个 slosh-aware MPC；不是首个移动底盘液体运输。
- 明确一个贡献层级：standard WMR online MPCC local planner with slosh-state augmentation。

---

## 8. Citation 优先级

### 8.1 主文必引

- `D02_Jian2023`：LT-DWA / ordinary planner baseline。
- `D01_Macenski2023`：RPP / ordinary tracking baseline。
- `D06_Williams2017`：MPPI / online model-based planning family。
- `B01_Lim2024`：mobile robot + spherical pendulum + offline trajectory optimization。
- `B02_Hamaguchi2005`、`B03_Hamaguchi2004`、`B04_Hamaguchi2002`：WMR/mobile container path/profile/input shaping historical line。
- `B11_Viet2025`：time-optimal anti-slosh planning/control + CBF/tracking近邻。
- `B12_Prabakaran2026`：special-platform slosh-aware MPC tracking/control。
- `A01_Leva2021`、`A02_Guagliumi2021`：低阶 slosh model foundation。
- `A12_Okatsuka2011`、`C05_Okatsuka2012`：slosh-aware predictive/MPC control guardrail。

### 8.2 主文可选 / 压缩引用

- `A07_Singer1990`：input shaping / smooth-only 背景。
- `C02_Muchacho2022`、`C03_Moriello2018`、`C06_Viet2024`：机械臂/操作防晃代表。
- `D03_Zhang2025`、`D13_Berscheid2021`：trajectory optimization / jerk-limited smoothness。
- `A08_Weaver2024`：液面测量，可放实验评价边界。

### 8.3 建议暂不在主文展开

- `A03/A04/A05/A06`：高保真模型，除非需要一句对照。
- `A09/A10/A11`：测量方法细节，除非实验章节需要。
- `B05/B06/B07/B09/B10`：只在表格或一句类别短提。
- `D04/D05/D07/D08/D09/D10/D11/D12`：除非 Agent B 认为 local planner 谱系需要。

---

## 9. 待补强 / 待核验清单

### 9.1 local planner bib gap

若终稿明确写 DWA / TEB / mpc_local_planner / MPCC backbone，需要补充真实 BibTeX 条目。当前 `references.bib` 尚未看到这些 canonical entries。

建议交给 Agent B：

- Fox, Burgard, Thrun, Dynamic Window Approach；
- Rösmann et al., Timed Elastic Band；
- `mpc_local_planner` 对应论文/仓库引用；
- MPCC backbone / contouring-control reference；
- mobile robot MPCC 或 local planner benchmark。

### 9.2 docx 额外文献

docx 中提到的部分外部文献，如 GSFT、nonprehensile tray feedforward/plugin 等，若要加入正文，需要 Agent E 核验：

- title / author / year / venue / DOI / arXiv；
- 是否真的适合主文；
- 是否与当前 scope 冲突。

### 9.3 D14 Jian 叙事参考

`D14_Jian2023` 是 D-CBF-MPC / ICRA 2023 叙事参考，不应写成 LT-DWA，也不应作为 SPMPC ordinary local planner baseline。若在论文正文中引用，需说明它属于 safety-critical MPC / D-CBF-MPC 背景，而不是本文核心 related work 主线。

---

## 10. 可执行 rewrite 步骤

建议后续真正改 `02_related_work.tex` 时按以下顺序：

1. 保留开头总述，但把问题句改成：
   - local planner 已成熟；
   - liquid anti-slosh 已成熟；
   - gap 在二者交叉处。
2. 将当前 `普通移动机器人局部规划与轨迹优化` 提到第一小节。
3. 将当前 `移动底盘液体运输与防晃路径设计` 提到第二小节，保留表格但压缩类别。
4. 将当前 `晃液建模...` 与 `输入整形...` 合并为第三小节。
5. 将 sensing/high-fidelity 细节压缩，避免 Related Work 过重。
6. 保留并加强定位小结，明确 novelty guardrail。
7. 若新增 DWA/TEB/mpc_local_planner citations，先补 `references.bib` 并核验编译。

---

## 11. 建议的最终章节骨架（可直接照此改 LaTeX）

```latex
\section{相关工作}

本节按论文论证链组织相关工作：普通移动机器人局部规划提供了在线、可执行的运动生成层；移动底盘液体运输研究证明了运输液体时显式考虑晃动的重要性；跨平台防晃研究进一步提供了低阶建模、轨迹优化和预测控制基础。本文关注这些方向的交叉处：标准轮式移动底盘上的在线晃液感知 MPCC 局部规划。

\subsection{Online local planning and trajectory optimization for mobile robots}
% DWA/TEB/RPP/LT-DWA/MPC/MPPI/MPCC
% transition: online but not liquid-aware

\subsection{Mobile-base liquid transport and anti-slosh planning}
% Hamaguchi, Lim, B11, B12, special mechanism
% table: planning/control layers

\subsection{Cross-platform slosh modeling and anti-slosh control}
% A01/A02, A12/C05, C02/C03/C06, A07
% boundary: methodological foundation, not same-layer baseline

\subsection{Positioning of SPMPC}
% not first anti-slosh, not first slosh-aware MPC
% contribution: online MPCC local planner + slosh-state propagation
```

---

## 12. 一句话结论

当前 Related Work 的事实材料已经足够，但终稿应从“四类文献综述”改成“三段式 claim-driven positioning”：

> local planners are online but not liquid-aware; mobile-base liquid anti-slosh methods are liquid-aware but mostly fixed-profile/offline/tracking/special-platform; cross-platform anti-slosh work provides modeling and predictive-control foundations but not a standard WMR local-planning interface. SPMPC fills the intersection by embedding low-order slosh states into an online MPCC local planner.
