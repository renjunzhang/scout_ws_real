# SPMPC 论文写作路线：文献定位优先

日期：2026-06-29
目标论文：面向化学实验室开口/半开口液体运输的 SPMPC / slosh-aware MPCC 方法论文
当前范围冻结：第一版暂不纳入 Map-vref、terrain-conditioned slosh experience prior、完整 obstacle-aware MPCC、homotopy/corridor 主方法宣称。

---

## 0. 总体判断

论文写作应先从“相关论文与方法定位”开始，而不是直接写 Method 或实验结果。

核心原因：这篇论文需要向审稿人解释清楚 SPMPC 的位置：

1. 相对于机械臂/操作臂液体防晃方法，SPMPC 针对的是轮式移动底盘和局部规划控制问题；
2. 相对于普通移动底盘 local planner，SPMPC 对液体晃动更加友好，因为它显式建模并预测液体动态记忆；
3. 相对于已有移动底盘液体转运方法，SPMPC 不是单纯 fixed profile、input shaping 或离线 trajectory optimization，而是在线 receding-horizon slosh-aware MPCC local planner/controller；
4. 实验 baseline 的选择必须由文献定位推出，不能只随机选几个算法比较。

因此，推荐写作路线是：

> 先做文献矩阵与 claim-gap 定位，再写方法与理论说明，最后设计实验和结果组织。

---

## 1. 第一阶段：相关论文与定位矩阵

这是最重要的一步。目标不是简单堆参考文献，而是建立论文的论证框架：

> 现有方法分别解决了什么问题？它们为什么不能完全覆盖化学实验室移动底盘开口液体运输？SPMPC 的新增价值在哪里？

建议优先参考以下已有材料：

- `docs/实物实验注意事项/对比论文的寻找/对比方法候选池_现代local_planner与液体防晃方法.docx`
- `docs/重要文档/论文参考总结/`
- `docs/论文书写/参考文档/SPMPC_RAL论文组织与Method初稿_第一版.docx`

建议生成一个文献矩阵文件，例如：

- `docs/论文书写/书写思路/SPMPC_related_work_matrix.md`

矩阵字段建议如下：

| Paper / Method            | Platform    | Online / Offline           | Method type                               | Liquid model | Local planner?               | Can be baseline?                | Role in our paper                                    |
| ------------------------- | ----------- | -------------------------- | ----------------------------------------- | ------------ | ---------------------------- | ------------------------------- | ---------------------------------------------------- |
| DWA                       | WMR         | Online                     | velocity sampling                         | No           | Yes                          | Yes                             | classic local-planner baseline                       |
| TEB                       | WMR         | Online                     | timed trajectory optimization             | No           | Yes                          | Yes                             | classic optimization-based local-planner baseline    |
| LT-DWA                    | WMR         | Online                     | long-horizon DWA + graph optimization     | No           | Yes                          | Yes if integration passes gate  | modern local-planner baseline                        |
| mpc_local_planner         | WMR         | Online                     | NMPC local planner                        | No           | Yes                          | strong backup baseline          | closest ordinary MPC local planner                   |
| Hamaguchi-style           | WMR         | mostly offline/profile     | input shaping / path-speed design         | Yes          | Not a standard local planner | inspired baseline               | classic mobile-base anti-slosh reference             |
| Lim-style                 | WMR         | Offline                    | slosh-constrained trajectory optimization | Yes          | No                           | inspired/supplementary baseline | mobile-base liquid trajectory optimization reference |
| Muchacho / GSFT / Ferrari | Manipulator | offline or online tracking | slosh-free manipulation                   | Yes          | No                           | no direct Scout baseline        | high-level anti-slosh related work                   |
| SPMPC                     | WMR         | Online                     | slosh-aware alpha-state MPCC              | Yes          | Yes                          | proposed method                 | online liquid-aware local planning/control           |

---

## 2. 文献分类逻辑

### 2.1 机械臂/机器人操作中的液体防晃方法

代表方向：

- Muchacho et al., slosh-free robot trajectory optimization；
- GSFT / geometric slosh-free tracking；
- Ferrari et al., time-optimal anti-sloshing trajectory planning；
- Moriello / Biagiotti feed-forward slosh suppression；
- SCARA、多容器、机械臂末端轨迹防晃方法。

这些文献的作用：

1. 证明显式液体模型和液体状态预测是有价值的；
2. 说明液体防晃不是简单速度平滑可以完全替代的；
3. 提供液体模态建模、slosh-free trajectory optimization、模型验证等 related work 支撑。

但它们不应作为主要实车 baseline，因为：

| 维度     | 机械臂防晃方法                          | SPMPC                              |
| -------- | --------------------------------------- | ---------------------------------- |
| 平台     | 机械臂、SCARA、末端执行器               | 差速/轮式移动底盘                  |
| 控制对象 | end-effector pose / joint motion        | chassis`v, omega, a, alpha`      |
| 任务     | 点到点操作或末端轨迹跟踪                | 化学实验室移动液体运输             |
| 规划层级 | 多为离线轨迹优化或 manipulator tracking | 在线 local planner / MPCC          |
| 约束     | 机械臂关节和末端轨迹                    | 非完整底盘、路径进度、局部规划频率 |
| 论文角色 | related work / inspiration              | proposed online WMR local planner  |

论文中的表述重点：

> Existing robotic manipulation studies show the importance of explicit sloshing models, but they are not designed for wheeled mobile-base local planning with nonholonomic motion, path-progress optimization, and ROS local-planner execution.

---

### 2.2 普通移动底盘 local planner

代表方向：

- DWA；
- TEB；
- LT-DWA；
- mpc_local_planner；
- Nav2 MPPI；
- Regulated Pure Pursuit；
- T-MPC++ / topology-driven MPC / MPCC local planning。

这些文献的作用：

1. 提供同层 local planner baseline；
2. 说明移动机器人局部规划已经成熟；
3. 帮助界定 SPMPC 不是“重新发明 local planner”，而是在 local planner 中加入液体动态预测。

这些方法的不足：

| 维度                                   | 普通底盘 local planner         | SPMPC                                      |
| -------------------------------------- | ------------------------------ | ------------------------------------------ |
| 主要目标                               | 到点、避障、路径跟踪、控制平滑 | 到点 + 跟踪 + 降低液体晃动                 |
| 状态                                   | robot pose / velocity          | robot + path progress + slosh modal states |
| 液体动态记忆                           | 无                             | 有，`eta, eta_dot`                       |
| 是否对残振相位敏感                     | 否                             | 是                                         |
| 是否能区分“同样平滑但相位不同”的动作 | 通常不能                       | 可以                                       |
| 论文角色                               | 外部 local-planner baseline    | proposed method                            |

关键论点：

> Smooth motion is not necessarily slosh-aware motion. Two commands with similar velocity smoothness may interact differently with the residual liquid oscillation depending on the sloshing phase. SPMPC can distinguish these cases because it propagates the liquid modal states inside the prediction horizon.

这个论点直接支撑内部消融中的：

- `B_ours vs B_smooth`：证明显式液体状态预测不只是普通平滑；
- `B_ours vs B_slosh`：证明 smooth shaping 对最终方法也必要。

---

### 2.3 移动底盘液体运输方法

这是最接近 SPMPC 的 related work，需要重点整理。

候选方法包括：

- Hamaguchi-style input shaping / velocity control / path design；
- Lim-style mobile robot 2D slosh trajectory optimization；
- Choi RAS 2024 food-serving robot sloshing suppression；
- mecanum / omnidirectional mobile platform liquid transfer；
- time-optimal anti-sloshing control under disturbances；
- 其他 WMR / OMR liquid transfer 方法。

建议分类：

| 类别                     | 代表方法                       | 与 SPMPC 的关系                                                    |
| ------------------------ | ------------------------------ | ------------------------------------------------------------------ |
| 经典移动底盘防晃         | Hamaguchi / Taniguchi          | 可做 inspired profile baseline，说明传统 input shaping/path design |
| 移动底盘离线液体轨迹优化 | Lim-style                      | 可做补充或仿真 baseline，说明 offline optimization                 |
| 服务机器人液体抑制       | Choi RAS 2024                  | 强 related work，需要精读并说明场景差异                            |
| 麦轮/全向平台液体运输    | Mecanum liquid transfer papers | related work，不一定同层 baseline                                  |
| 普通 local planner       | DWA / TEB / LT-DWA / MPC       | 同层 baseline，但无液体模型                                        |
| SPMPC                    | 本文                           | online slosh-aware MPCC local planner/controller                   |

论文中的表述重点：

> Compared with traditional mobile-base liquid-transport methods that rely on fixed velocity profiles, input shaping, or offline trajectory optimization, SPMPC performs online receding-horizon local planning and control while jointly optimizing path progress, chassis controls, and predicted liquid modal response.

---

## 3. 第二阶段：Claim–Gap–Evidence 矩阵

在正式写 Introduction 和 Related Work 前，建议建立 claim 矩阵。

建议输出文件：

- `docs/论文书写/书写思路/SPMPC_claim_gap_evidence_matrix.md`

推荐结构：

| 我们的 claim                              | 文献支撑                               | 实验支撑                     | 审稿人可能质疑             | 应对方式                                               |
| ----------------------------------------- | -------------------------------------- | ---------------------------- | -------------------------- | ------------------------------------------------------ |
| 普通 local planner 不考虑液体动态         | DWA, TEB, LT-DWA, mpc_local_planner    | external baseline comparison | 它们调得更平滑是否也可以？ | B_ours vs B_smooth，速度/时间匹配                      |
| 机械臂防晃不等同于移动底盘 local planning | Muchacho, GSFT, Ferrari                | task/platform distinction    | 为什么不直接用机械臂方法？ | 强调非完整底盘、在线 local planner、path progress      |
| 显式 slosh state 在 smooth 之外有贡献     | slosh dynamics / anti-slosh literature | B_ours vs B_smooth           | 效果是否只是更慢？         | matched-speed/time protocol                            |
| alpha-state 有助于液体友好转向            | local planner + dynamics discussion    | omega-rate / alpha metrics   | 是否只是参数调小？         | common limits + alpha-state ablation/diagnostics       |
| SPMPC 适合化学实验室液体运输              | lab scenario motivation + real robot   | RGB liquid-height validation | RGB 是否是真值？           | 明确`/spmpc/slosh_height` 只是 proxy，RGB 是实物评价 |

---

## 4. 第三阶段：方法与理论说明

文献定位完成后，再写 Method。Method 不应过早展开 Map-vref 或未完成模块。

建议 Method 结构：

1. Problem Formulation and System Overview；
2. Reference Path and Progress Parameterization；
3. Alpha-State Mobile-Robot Dynamics；
4. Slosh Dynamics and Markov Augmentation；
5. Augmented Nonlinear Dynamics；
6. Slosh-Aware MPCC Objective；
7. State and Input Constraints；
8. Online Slosh-State Update；
9. Receding-Horizon Solution and Command Generation；
10. Method Variants for Same-Framework Ablation。

理论说明建议聚焦以下几点，不强行做过度证明：

### 4.1 Slosh Markov augmentation

通过引入液体状态：

\[
x_s = [\eta_x, \dot{\eta}_x, \eta_y, \dot{\eta}_y]^T
\]

液体的历史激励被压缩到当前模态位移和模态速度中。给定当前增广状态和未来控制，未来液体响应可以在 MPC horizon 内传播。

这说明 SPMPC 能感知 residual sloshing，而 smooth-only planner 不能。

### 4.2 Alpha-state angular dynamics

将角速度作为状态、角加速度作为控制：

\[
\dot{\omega} = \alpha
\]

并施加：

\[
|\alpha| \leq \alpha_{\max}
\]

从而在 OCP 中显式限制转向变化率，减少横向晃动激励。

### 4.3 Smooth-only 不等于 slosh-aware

smooth cost 只惩罚控制幅值或控制变化；slosh-aware cost 惩罚预测的液体模态响应。当前液体存在残振时，即使两个控制序列一样平滑，也可能因为与残振相位不同而导致不同液面响应。

这是 SPMPC 相对 `B_smooth` 的核心理论解释。

### 4.4 不建议过度宣称

不要声称：

- 全局最优；
- 完整闭环稳定性证明；
- 完整 obstacle-aware MPCC；
- homotopy/corridor 已作为主方法；
- Map-vref / terrain-conditioned prior 已纳入第一版；
- `/spmpc/slosh_height` 等同于真实液面高度。

建议写法：

> The nonconvex OCP is solved in real time using SQP-RTI to obtain a local solution. Formal closed-loop stability under all operating conditions and full obstacle-aware MPCC are outside the scope of this work.

---

## 5. 第四阶段：实验设计

实验应从 claim 推出，而不是单纯列结果。

### 5.1 内部消融：证明显式 slosh prediction 与 smooth shaping 的作用

主表 A：

| Method   | Slosh dynamics/cost | Smooth shaping | Role                      |
| -------- | ------------------- | -------------- | ------------------------- |
| B0       | No                  | Weak           | ordinary alpha-state MPCC |
| B_slosh  | Yes                 | Weak           | slosh-only ablation       |
| B_smooth | No                  | Strong         | smooth-only ablation      |
| B_ours   | Yes                 | Strong         | final method              |

关键比较：

- `B_ours vs B_smooth`：证明 slosh-aware 不是普通平滑；
- `B_ours vs B_slosh`：证明 smooth shaping 对最终效果必要；
- `B_smooth vs B0`：证明平滑本身的收益；
- `B_slosh vs B0`：证明液体代价本身的作用。

### 5.2 外部 local planner 对比：证明 SPMPC 相对普通底盘规划更液体友好

主表 B：

| Method                      | Type                          | Slosh-aware? | Role                        |
| --------------------------- | ----------------------------- | ------------ | --------------------------- |
| DWA                         | velocity-space sampling       | No           | classic baseline            |
| TEB                         | timed trajectory optimization | No           | optimization-based baseline |
| LT-DWA or mpc_local_planner | modern local planner / NMPC   | No           | modern or MPC baseline      |
| B_ours                      | slosh-aware MPCC              | Yes          | proposed method             |

公平性协议：

- 相同 fixed global path；
- 相同起点/终点；
- 相同地图/重置流程；
- 统一 `v_max`、`omega_max`、`a_max`、`alpha_max` 或最接近配置；
- 统一 goal tolerance 和 timeout；
- 失败计入 success rate；
- 使用 RGB 或统一外部 observer 评价液面；
- 不用 `/spmpc/slosh_height` 直接评价 DWA/TEB。

### 5.3 移动底盘液体防晃方法补充对比

补充表 C：

| Method             | Role                                              |
| ------------------ | ------------------------------------------------- |
| Hamaguchi-inspired | classical input-shaped fixed-path profile         |
| Lim-inspired       | offline slosh-constrained trajectory optimization |
| B_smooth           | online smooth-only SPMPC                          |
| B_ours             | online slosh-aware MPCC                           |

注意：Hamaguchi/Lim 与 DWA/TEB 不是完全同层方法，不建议混成一张总排名表。

---

## 6. 化学实验室场景叙事

论文场景应聚焦化学实验室，而不是泛泛 food serving 或 terrain transport。

推荐问题表述：

> Given a fixed laboratory reference path, how can a wheeled mobile robot locally plan and control its motion while explicitly accounting for the dynamic memory of open-liquid sloshing, so that it can transport chemical liquid samples safely with reduced free-surface oscillation?

中文表述：

> 给定化学实验室中的固定参考路径，轮式移动机器人如何在局部滚动优化中显式考虑开口液体的动态记忆，使其在保持到点和路径跟踪性能的同时降低液面晃动与潜在飞溅风险？

应用动机关键词：

- chemical samples；
- reagents；
- open or partially filled containers；
- spillage；
- contamination；
- safety risk；
- laboratory benches / instruments / storage areas；
- predefined safe routes。

---

## 7. 推荐写作顺序

虽然正文中 Related Work 在 Method 前后都可以，但实际工作建议顺序是：

1. 文献矩阵：整理机械臂防晃、普通底盘 local planner、移动底盘液体运输；
2. Claim–Gap–Evidence 矩阵：明确每个 claim 由哪些文献和实验支撑；
3. Method 公式：写 alpha-state、path progress、slosh augmentation、OCP；
4. 实验协议：写 baseline、公平性、指标、RGB 评价；
5. Results 结构：按 claim 排列结果；
6. Related Work 初稿；
7. Introduction 初稿；
8. Abstract、Conclusion、标题最后精修。

一句话原则：

> 文献决定论文定位，定位决定方法叙事，claim 决定实验设计。

---

## 8. 给后续 Agent 的建议任务

可以让 agent 按下面几个任务并行或顺序执行。

### Agent 任务 A：整理文献矩阵

输入：

- `docs/实物实验注意事项/对比论文的寻找/对比方法候选池_现代local_planner与液体防晃方法.docx`
- `docs/重要文档/论文参考总结/`

输出：

- `docs/论文书写/书写思路/SPMPC_related_work_matrix.md`

要求：

- 按机械臂防晃、普通底盘 local planner、移动底盘液体运输、MPCC/MPC 工具四类整理；
- 标明每篇论文是 direct baseline、supplementary baseline、related work，还是 inspiration-only；
- 不要把机械臂方法误写成同层 Scout 实车 baseline；
- 不要把 MPPI 写成已完成 baseline，除非后续确实实现。

### Agent 任务 B：整理 Claim–Gap–Evidence 矩阵

输出：

- `docs/论文书写/书写思路/SPMPC_claim_gap_evidence_matrix.md`

要求：

- 每个 claim 对应至少一组文献支撑和一组实验支撑；
- 明确审稿人可能质疑点；
- 明确需要哪些实验图表支撑；
- 突出 `B_ours vs B_smooth` 的核心因果意义。

### Agent 任务 C：写 Related Work 草稿

输出：

- `docs/论文书写/书写思路/SPMPC_related_work_draft.md`

建议小节：

1. Slosh suppression in robotic manipulation；
2. Local planning for wheeled mobile robots；
3. Liquid transportation by mobile robots；
4. Positioning of the proposed SPMPC。

### Agent 任务 D：写 Method 初稿

参考：

- `docs/论文书写/参考文档/SPMPC_RAL论文组织与Method初稿_第一版.docx`
- `src/scout_apps/control/spmpc_local_planner/README.md`

输出：

- `docs/论文书写/书写思路/SPMPC_method_draft_no_map_vref.md`

要求：

- 不写 Map-vref；
- 不写 terrain-conditioned prior；
- 不宣称完整 obstacle-aware MPCC；
- 场景改成 chemical laboratory liquid transport；
- 保留 alpha-state、slosh augmentation、MPCC objective、solver、ablation variants。

### Agent 任务 E：写实验设计草稿

输出：

- `docs/论文书写/书写思路/SPMPC_experiment_plan.md`

要求：

- 包括内部消融、外部 local planner、移动底盘液体防晃补充对比；
- 写明 fairness protocol；
- 写明 RGB max-LCR 与 `/spmpc/slosh_height` 的语义边界；
- 写明 60s timeout、fresh-sim、common limits；
- 化学实验室 fixed-path 场景化表述。

---

## 9. 最终主线一句话

这篇论文的主线建议固定为：

> 机械臂防晃方法证明了显式液体建模的重要性，但不直接适用于轮式移动底盘局部规划；普通移动底盘 local planner 能跟踪路径和生成平滑运动，但不考虑液体动态记忆；已有移动底盘液体运输方法多依赖固定 profile、input shaping 或离线优化。本文提出面向化学实验室开口液体运输的在线 slosh-aware alpha-state MPCC，在同一滚动优化中联合考虑路径进度、底盘控制和液体模态响应，从而在保持到点与路径跟踪性能的同时降低液面晃动。

