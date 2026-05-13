# de Groot 2025 T-MPC 方法剖析

论文：de Groot, Ferranti, Gavrila, Alonso-Mora, "Topology-Driven Parallel Trajectory Optimization in Dynamic Environments", IEEE T-RO Vol.41, 2025 (在线发表 2024-10-04). DOI:10.1109/TRO.2024.3475047. 代码: <https://github.com/tud-amr/mpc_planner>

剖析日期：2026-05-12
剖析对象：Guidance→Planning→Decision 三层架构与 OSCRS G→F→R→S 的结构对应关系；同伦类（homotopy class）概念在候选生成中的借鉴意义。

## 1. 论文定位与场景

```text
机器人:   差速移动底盘（与 Scout Mini 同类）
环境:     动态障碍物（行人等），自由空间随时间变化
任务:     在动态环境中规划无碰、动力学可行的轨迹并执行
模式:     receding horizon — 每个控制周期重新规划并执行第一步
```

核心问题：非凸轨迹优化在动态障碍环境中易陷入局部最优——障碍物把自由空间切成多个"通道"（左绕/右绕），单一初值的局部优化器无法探索所有通道。

与 Scout 的关系：

```text
T-MPC 动态避障                 Scout OSCRS
─────────────────────────────────────────────────
障碍物切割自由空间 → homotopy   路径曲率引发 slosh → candidate set
Guidance Planner 生成多同伦类    GeoRef 生成多平滑候选
P 个 MPC 并行局部优化            R 层 rollout 预测 slosh
Decision Making 选最小代价       S 层 hard gate + score 选最优
执行最优轨迹                    发布 /scout/global_path_anti_slosh
```

## 2. 三层架构

```text
x₀ (当前状态), O (动态障碍预测), γ (参考路径)
        │
        ▼
  构造动态自由空间: X = R² × [0,T], C = X \ O
        │
        ▼
┌─ Step 1: Guidance Planner ────────────────────────────┐
│  生成 P 条同伦类不同的引导轨迹                          │
│  Visibility-PRM → DFS → homotopy comparison → 去重      │
└───────────────────────────────────────────────────────┘
        │
        ▼
┌─ Step 2: P 个 Local Planner / MPC 并行优化 ───────────┐
│  每个局部规划器被限制在对应同伦类内                      │
│  优化目标: min Σ J(x_k, u_k)                            │
│  s.t. 动力学 + 避障 + 拓扑保持约束                       │
└───────────────────────────────────────────────────────┘
        │
        ▼
┌─ Decision Making ────────────────────────────────────┐
│  比较各条优化轨迹的代价 → 选最小代价执行                 │
│  一致性决策: 上一轮选中者加权优先                        │
└───────────────────────────────────────────────────────┘
        │
        ▼
  执行最优轨迹前一段 → 下一周期重复
```

## 3. Guidance Planner 详解

### 3.1 Visibility-PRM 建图

在动态自由空间中采样节点，构建路标图。节点类型：

| 节点 | 作用 |
|------|------|
| Guard | 表示新的可见区域边界 |
| Connector | 连接两个 Guard，形成拓扑通道 |
| Goal | 布置在参考路径附近的多目标节点 |

### 3.2 候选生成与去重

在 Visibility-PRM 图上用 DFS 搜索多条候选路径 → 输出 piecewise linear trajectories → 平滑为 cubic splines → 得到 differentiable guidance trajectories。通过 homotopy comparison 过滤等价路径，保证送入后续优化的 P 条 guidance 是拓扑不同的。

### 3.3 与 OSCRS G 层的对应

```text
T-MPC Guidance Planner            OSCRS G 层
─────────────────────────────────────────────────
Visibility-PRM 采样 + DFS         GeoRef smoothing 参数网格
homotopy comparison 去重           candidate_specs 顺序保证
piecewise linear → cubic spline   resample + sanitize
输出 P 条 guidance trajectories   输出 5 条候选 (original/mild/.../strong)
```

关键差异：T-MPC 的候选差异来自**空间拓扑**（绕障碍物的方式），OSCRS 的候选差异来自**曲率强度**（smoothing 程度）。两者都是"生成多条候选 → 后续层选择"的范式。

## 4. Local Planner / MPC 并行优化

每条 guidance trajectory 启动一个 local planner 进一步优化：

$$
J_i^* = \min_{u,x} \sum_{k=0}^{N} J(x_k, u_k)
$$

约束：
1. 动力学：$x_{k+1} = f(x_k, u_k)$
2. 初始状态：$x_0 = x_{\text{init}}$
3. 避障：$g(x_k, o_k^j) \leq 0$
4. **拓扑保持**：$g_H(x_k, o_k^j, \tau_{i,k}) \leq 0$

拓扑保持约束是关键创新：以 guidance trajectory 为初值并不能保证优化结果仍属不同同伦类，需要显式约束轨迹在每个时间步相对于每个障碍物保持在 guidance 所在的那一侧。

### T-MPC++

在 P 个 guided planner 之外额外加入一个**无引导的普通 local planner**（不带拓扑约束）。保证最终结果不劣于单独使用普通 local planner。

## 5. Decision Making

最小代价决策：

$$
\tau_{i^*}^*, \quad i^* = \arg\min_i J_i^*
$$

一致性决策（减少连续周期中的策略切换）：

$$
\tau_{i^*}^*, \quad i^* = \arg\min_i w_i J_i^*, \quad w_i = \begin{cases} c_i & \text{上一轮被选过} \\ 1 & \text{否则} \end{cases}
$$

其中 $0 \leq c_i \leq 1$。

### 与 OSCRS S 层的对应

```text
T-MPC Decision Making             OSCRS S 层
─────────────────────────────────────────────────
min J_i* (统一代价)               hard gate (oh/or/os) + batch-norm score
一致性决策 (c_i 加权)             同分时 candidate_specs 顺序优先
无引导 fallback (T-MPC++)         original 永远作为 fallback
```

## 6. 关键概念

### homotopy class (同伦类)

两条轨迹如果在不碰障碍的情况下可以连续变形为彼此，属于同一类。左绕和右绕通常是不同同伦类。

### HGO (Homotopy Globally Optimal)

T-MPC 不能保证非凸问题的全局最优。HGO 是较弱的性质：如果每个可行同伦类都有候选、拓扑约束最终不激活、执行最低代价轨迹，则最终轨迹代价不高于每个可行同伦类中最差局部最优的代价。

### 计算复杂度

Guidance Planner: $O((n^2 + P^2)M)$，实际近似 $O(n^2 M)$（n < 100 通常足够）。

## 7. 对 Scout / OSCRS 的可借鉴点

### 已借鉴

- **G→F→R→S 架构**: `mpc_planner` 的 `Guidance→Planner→Decision` 三层结构直接启发了 OSCRS 的模块化边界设计。`oscrs/pipeline.py` 中的 `run_pipeline()` 对应 `Planner::update()` 的编排角色
- **多候选并行评估**: T-MPC 的 P 个 planner 并行优化 → OSCRS 对 5 个候选独立做 F→R→S 评估链
- **Fallback 保证**: T-MPC++ 的无引导 planner → OSCRS 的 `original` 永远保留且不进 S_full

### 可借鉴但未迁移

- **拓扑保持约束**: 如果未来 OSCRS 的候选生成扩展到 homotopy 维度（如 collision 导致的不同绕行），需要类似 $g_H$ 的机制确保候选不被后续优化"拉回"同一同伦类。当前 collision gate 仅是 binary accept/reject，不保持拓扑
- **一致性决策**: 当前 OSCRS 没有跨周期的选择一致性机制。如果连续周期频繁在 mild/medium 之间切换，可借鉴 $w_i$ 加权减少抖动
- **Visibility-PRM**: 当前 GeoRef 只在曲率维度生成候选。如果加入空间拓扑维度（左绕/右绕），需要类似 PRM 的采样机制

### 不可直接迁移

- **MPC solver 体系**: T-MPC 的 local planner 是完整的非线性 MPC solver，OSCRS 作为 reference-layer post-processor 不承担求解器角色
- **动态障碍预测**: OSCRS 不处理动态障碍，碰撞检查是静态 costmap point-cost
- **并行计算**: T-MPC 的 P 个 MPC 并行运行依赖多线程，OSCRS 当前是串行评估

## 8. 术语速查

| 术语 | 含义 |
|------|------|
| homotopy class | 轨迹在不碰障碍物前提下的连续变形等价类 |
| homotopy comparison | 判断两条候选是否属于同一同伦类 |
| homotopy constraint | MPC 优化中的约束，防止轨迹跨到障碍物另一侧 |
| HGO | Homotopy Globally Optimal — 同伦类覆盖充分条件下的弱全局最优 |
| Visibility-PRM | 基于可见性采样的概率路标图，用于在自由空间中生成多条拓扑不同的路径 |
| T-MPC++ | T-MPC 变体，额外加入无引导 local planner 作为 fallback |
| receding horizon | 每个控制周期求解有限时域优化问题，只执行第一步，下一周期重新求解 |
