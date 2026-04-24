# 2026-04-24 Risk-Adaptive Scheduling 改进计划

## 当前状态

当前 `Risk-Adaptive Scheduling` 不是从零开始设计，而是在

- [2026-04-19_Risk-Adaptive Scheduling Layer实现方案.md](/home/a/scout_ws/docs/Claude/修改方案-时间-简介/2026-04-19_Risk-Adaptive%20Scheduling%20Layer实现方案.md)

这份方案基础上，已经完成了第一版代码落地。

换句话说，当前状态是：

- **代码主体已完成**
- **outer loop 已集成**
- **参数已进 yaml**
- **bag 中已能看到 `rho_k / r_k / u_k / Q_eta_k / fallback_active`**
- **主缺口不再是“有没有实现”，而是“是否有足够数据证明它值得作为论文主贡献”**

### 已完成的部分

按 04-19 方案，当前已经实现：

1. `r_k` 物理风险
   - 由 `h_risk / e_risk / t_risk` 组成
2. `u_k` 激励一致性不确定性
   - 来自 `|a_y_imu - v*omega|`
3. `rho_k`
   - 由 `r_k` 与 `u_k` 合成
4. sigmoid 调度
   - `gamma / rho_0`
5. 三路调度输出
   - `Q_eta`
   - `eta_bar`
   - `v_ref_eff`
6. rate limit
7. fallback
8. outer loop 单步延迟集成

因此，当前 `risk_scheduler` 的问题不是“没做出来”，而是：

- 还没有通过实物重复实验证明其优势是否稳定
- 当前收益方向更偏 `p95 / RMS`，而不是 `peak`
- 与论文主指标的对齐还不够强

### 当前实验层面的判断

基于现有预实验，当前最合理的口径是：

- `FAS` 更像**峰值抑制器**
- `PROP` 更像**持续风险整形器**

所以当前不能写成：

- `PROP` 已经全面优于 `FAS`

更稳的写法是：

- `risk_scheduler` 已有初步效果，但其主要收益方向目前更像 **risk redistribution**

### 当前改进计划的出发点

因此，这份 04-24 计划不是“重新实现 Risk-Adaptive Scheduling”，而是：

1. 在 04-19 已实现版本基础上，补足实验与论证
2. 优先修正最可疑的结构问题
3. 最后再决定是否需要第二版调度律

---

## 执行摘要

1. **当前判断**  
   `risk_scheduler` 已经能工作，但当前更像“持续风险整形器”，不是“峰值最小化器”。

2. **当前证据**  
   预实验里 `PROP` 对 `p95 / RMS` 有潜在收益，但没有稳定压过 `FAS` 的 `peak`。

3. **当前不做的事**  
   不立刻重写调度器，不同时重调 `gamma / rho_0 / beta / fallback`。

4. **P0：先补数据**  
   固定路径下补足 `NOM / FAS_Q5 / PROP_Q5` 重复实验，优先 `P3_mixed`、`P2_s_curve`，每条件 `>=5` 次。

5. **P1：最小代码改动**  
   先只改一件事：把 `risk_scheduler` 的 `v_ref` 输入从固定巡航速度改成真实参考速度源。

6. **P2：论文级消融**  
   拆开验证 `adaptive_Q_eta / adaptive_eta_bar / adaptive_v_ref`，明确真正起主要作用的是哪一路。

## 成功判据

当前将 `PROP` 的成功定义为：

- `visual p95` 或 `RMS` 相比 `FAS` 改善 `>= 10%`
- `visual peak` 相比 `FAS` 恶化 `<= 5%`
- `task time` 相比 `FAS` 增加 `<= 10%`

论文写法上，不把 `PROP` 定义成“peak 必须最低”，而定义成：

- **优先降低持续风险暴露**
- **同时不显著恶化峰值与任务效率**

因此，后续统计报告至少要同时给出：

- paired difference
- effect size
- bootstrap CI
- Wilcoxon / permutation test（辅助）

## 背景

当前 `risk_scheduler` 已完成工程集成，能够在线调度：

- `Q_eta`
- `eta_bar`
- `v_ref_eff`

并且在实物预实验中已经观察到：

- `PROP` 相比 `FAS`，并未稳定降低 `peak`
- 但对 `p95 / RMS` 有潜在改善

因此当前问题不是“调度器完全无效”，而是：

1. **主指标对齐不够**
   - 当前行为更像“持续风险整形器”，不是“峰值抑制器”
2. **外层影响太多**
   - 虽已关掉 `curvature cap / speed governor / input shaping`
   - 但 `risk_scheduler` 本身仍同时调三路，难以解释贡献来源
3. **论文化证据不足**
   - 缺重复实验
   - 缺对现有方法的定位
   - 缺对调度律设计选择的论证

---

## 目标

后续改进不追求立刻重写调度器，而是按以下顺序推进：

### 目标 1：先把当前版本的实验结论做扎实

回答两个问题：

1. `FAS_Q5 vs NOM`
   - `Q_slosh` 是否稳定降低液面 `peak / p95 / RMS`
2. `PROP_Q5 vs FAS_Q5`
   - `risk_scheduler` 是否在 `peak` 不明显恶化的前提下，稳定降低 `p95 / RMS`

这是当前最优先的验证目标。

### 目标 2：把 `risk_scheduler` 的输入和论文主指标对齐

当前最值得怀疑的点不是权重数值本身，而是：

- 调度器接收的 `v_ref` 基准仍偏向固定 `v_nominal`
- 这更容易改变整体保守性，而不一定正中危险弯段的真实参考速度

因此下一版优先考虑：

- 将 `risk_scheduler.update(..., v_ref)` 的输入，从固定巡航速度改为**当前真实路径/速度剖面的参考速度源**

推荐第一版控制输入为：

- `v_sched = min(ref_points[0:3].v_ref)`

同时记录以下候选量，便于离线比较：

- `min(ref[0:3])`
- `mean(ref[0:3])`
- `mean(ref[0:H])`

### 目标 3：把三路调度拆开做消融

当前 `risk_scheduler` 同时调：

- `Q_eta`
- `eta_bar`
- `v_ref_eff`

后续应拆成独立开关：

- `adaptive_Q_eta`
- `adaptive_eta_bar`
- `adaptive_v_ref`

这样才能回答：

- 真正有效的是自适应惩罚？
- 自适应软约束？
- 还是自适应降速？

当前预期的主次顺序是：

1. `adaptive_v_ref`
2. `adaptive_Q_eta`
3. `adaptive_eta_bar`

因此 `eta_bar` 更适合作为辅助 safety-like 通道，不宜在论文第一层就写成主贡献。

### 目标 4：补足论文论证

需要把当前工程实现提升到论文可解释口径：

1. 为什么使用 sigmoid 调度，而不是 hard switch
2. 为什么需要 `rate_limit_per_step`
3. 为什么 `r_k` 与 `u_k` 要分开
4. 为什么在线调三个量，而不是只调一个

这部分需要结合文献整理，不一定要求代码先改。

---

## 分阶段实施

## P0：不改算法，只补数据

先完成固定路径重复实验：

- `NOM`
- `FAS_Q5`
- `PROP_Q5`

路径优先：

- `P3_mixed`
- `P2_s_curve`

每条件至少：

- `>= 5` 次

输出：

- `peak`
- `p95`
- `RMS`
- `task time`
- `v_mean / jerk_rms`

判定：

- 若 `PROP` 只在 `p95 / RMS` 优于 `FAS`，则论文口径写成“risk redistribution”，不写成“全面更优”

## P0.5：加入同速度 baseline

增加：

- `FAS_Q5_slow`

目的：

- 排除“`PROP` 只是因为整体跑慢了”这一解释

优先匹配量：

- `task time`

辅助报告：

- `v_mean`
- `jerk_rms`

## P1-A：reference-consistent scheduling

只做一个最小改动：

- 将 `risk_scheduler` 的 `v_ref` 输入改为**真实参考速度源**
- 第一版先使用 `min(ref_points[0:3].v_ref)`

暂不同时修改：

- `gamma`
- `rho_0`
- `beta`
- fallback 逻辑

原因：

- 先解决最可疑的结构问题
- 避免一次改太多后无法解释结果变化

## P1-B：physical normalization correction

单独修正 `h_max` 的物理量纲代理问题：

- 从当前代理写法修正为 `h_max_physical = h_coeff * eta_bar_max`

这一步与 `P1-A` 分开进行，不与 `v_ref` 输入修正同时上线。

## P2：拆分调度通道

新增实验开关：

- 仅调 `v_ref`
- 仅调 `Q_eta`
- `Q_eta + v_ref`
- `Q_eta + eta_bar + v_ref`
- `eta_bar only`（最后做）

目标：

- 找到真正贡献主要来自哪一条通道

## P3：fallback 与终点项整理

在主实验口径稳定后，再考虑：

1. `u_k` 的分档处理
   - 低：正常调度
   - 中高：更保守
   - 持续高：fallback
2. fallback 是否应同步保守降速
3. `t_risk` 是否单独用于 terminal / settling

当前主实验建议：

- `w_t = 0` 或显著降权

原因：

- 当前主问题是弯道与连续转向阶段的 slosh risk
- `t_risk` 会把 near-goal 效应混入主实验结论

因此这几项都不应插到当前主实验之前。

---

## 当前建议

当前不建议立刻重写 `risk_scheduler`。

更合理的顺序是：

1. 先按当前冻结口径录足够多的 bag
2. 先确认 `PROP vs FAS` 的真实收益方向
3. 加入 `FAS_Q5_slow` baseline
4. 再做 `P1-A / P1-B`

---

## 搜索论文可用 prompt

### Prompt 1：找与本工作最接近的 adaptive MPC / anti-slosh 文献

```text
请帮我检索与“mobile robot liquid slosh suppression using MPC”最相关的论文，重点关注：
1. 是否使用 MPC 或 predictive control
2. 是否有 online/adaptive scheduling of weights, constraints, or reference speed
3. 是否有 real-world experiments on curved paths
4. 是否比较 fixed-weight MPC vs adaptive MPC
请按“问题设置 / 控制结构 / 是否实物验证 / 可作为我论文 baseline 或 related work 的价值”整理。
```

### Prompt 2：找 risk scheduling / gain scheduling 的论证方式

```text
请检索使用 sigmoid scheduling, gain scheduling, or risk-adaptive parameter tuning 的控制论文，重点看：
1. 为什么用 sigmoid 而不是 hard switch
2. 如何解释 rate limiter / smoothing on scheduled parameters
3. 如何设计 risk score combining model state and uncertainty
4. 审稿中常见的质疑点是什么
请给我可直接用于论文写作的论证结构，而不是只列论文标题。
```

### Prompt 3：找 anti-slosh baseline

```text
请检索 liquid slosh suppression 的经典 baseline，重点看：
1. input shaping / ZV / ZVD
2. fixed-weight MPC
3. feedforward or trajectory shaping methods
4. 这些方法在转弯路径、变曲率路径下的局限性
请总结哪些 baseline 最适合与“risk-adaptive slosh-aware MPC”对比。
```

### Prompt 4：找 adaptive speed planning / reference shaping 文献

```text
请检索会根据风险、曲率、载荷或液体状态在线调整 reference speed 的轨迹跟踪/移动机器人论文，重点看：
1. 速度调度依据是什么
2. 调整的是 nominal speed, local reference, 还是 MPC constraints
3. 是否区分 peak suppression 与 sustained risk reduction
4. 是否有能支撑我把 risk scheduler 写成“risk redistribution layer”而不是“peak minimizer”的文献表述
```

---

## 一句话

当前 `risk_scheduler` 不需要立刻重写；下一步最值得做的是：

- **先补重复实验**
- **再把调度输入改成真实参考速度**
- **最后拆分三路调度做消融**
