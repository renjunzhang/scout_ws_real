# Dobrevski 2024 DADWA 方法剖析

论文：Dobrevski, Skočaj, "Dynamic Adaptive Dynamic Window Approach", IEEE T-RO Vol.40, 2024, pp.3068–3081 (在线发表 2024-05-14). DOI:10.1109/TRO.2024.3400932. 单位: University of Ljubljana.

剖析日期：2026-05-12
剖析对象：深度强化学习 + 经典 local planner 的混合架构；对 Scout 本地规划层参数自适应的潜在借鉴意义。

## 1. 论文定位与场景

```text
机器人:   差速移动底盘（与 Scout Mini 同类）
传感器:   2D 激光雷达
环境:     静态 + 动态障碍物，多种几何布局
任务:     无碰导航到目标点
方法:     DWA 作为安全执行器，深度强化学习 (PPO) 作为动态参数调节器
```

核心洞察：**不要用神经网络直接输出速度命令，而是让网络预测 DWA 代价函数的权重**。DWA 保证安全（速度空间搜索 + 运动学约束），网络保证适应性（根据激光观测动态调参）。

与 Scout 的关系：

```text
DADWA                            Scout
─────────────────────────────────────────────────
DWA 生成候选速度                  GeoRef 生成候选路径
网络预测 DWA 权重                 hand-tuned 参数 (ay_ratio_limit, etc.)
PPO 在仿真中训练                  bag 录制 + 离线分析调参
扩展代价函数 (4 项)               OSCRS score (多指标加权)
```

## 2. DWA 基础

DWA 在每个控制周期在速度空间中搜索短期最优命令。搜索空间是安全速度与动力学可达速度的交集：

$$
V_{\text{search}} = V_{\text{admissible}} \cap V_{\text{dynamic}}
$$

经典 DWA 代价函数（最大化）：

$$
G(\nu, \omega) = \sigma\big( \alpha \cdot h(\nu, \omega) + \beta \cdot c(\nu, \omega) + \gamma \cdot v(\nu, \omega) \big)
$$

| 项 | 含义 |
|----|------|
| $h(\nu, \omega)$ | heading — 朝向目标的程度 |
| $c(\nu, \omega)$ | clearance — 离障碍物的距离 |
| $v(\nu, \omega)$ | velocity — 线速度大小 |
| $\alpha, \beta, \gamma$ | 权重系数（传统 DWA 中手工固定） |

DWA 的核心局限：**没有一组通用的最优权重**。开阔走廊需要高 velocity 权重，狭窄通道需要高 clearance 权重，复杂障碍需要高 heading 权重。手工固定权重必然在某些场景中性能退化。

## 3. DADWA 方法

### 3.1 结构流程

```text
历史激光观测 + 目标位置 + 当前速度
        │
        ▼
  神经网络 (actor) 预测 DWA 权重 (α, β, γ, δ)
        │
        ▼
  扩展 DWA 采样候选速度 + rollout 候选轨迹
        │
        ▼
  按加权代价函数评分 → 选最优 (v, ω)
        │
        ▼
  机器人执行
```

### 3.2 MDP 建模

| 元素 | 内容 |
|------|------|
| 状态 $s_t$ | $(l_t^t, l_{t-3}^t, l_{t-7}^t, d_t, \phi_t, \nu_t, \omega_t)$ — 3 帧激光 + 目标距离/方向 + 当前速度 |
| 动作 $a_t$ | $(\alpha_t, \beta_t, \gamma_t, \delta_t)$ — **DWA 权重，不是机器人速度** |
| 转移 | model-free RL，只与仿真环境交互 |
| 奖励 | 稀疏事件奖励：到达目标 +1，碰撞 -1，超时 0 |
| 折扣 | $\gamma = 0.99$ |
| 训练算法 | PPO (Proximal Policy Optimization) |

### 3.3 振荡问题的解决：distance-to-curvature

传统 DWA 在障碍物附近容易因 heading 权重过高而原地振荡——机器人不愿暂时偏离目标方向来绕障。

DADWA 新增第四项 **distance-to-curvature** $d(\nu, \omega)$：评价候选圆弧轨迹是否靠近目标，帮助机器人绕开障碍物后重新朝目标前进。

扩展后的 DADWA 代价函数：

$$
G(\nu, \omega) = \alpha \cdot h(\nu, \omega) + \beta \cdot c(\nu, \omega) + \gamma \cdot v(\nu, \omega) + \delta \cdot d(\nu, \omega)
$$

网络同时调节 $\alpha$（heading）和 $\delta$（distance-to-curvature），在"直冲目标"和"绕行接近"之间动态平衡。

### 3.4 观测历史

三通道一维输入（每通道 90 个 beam），让网络感知障碍物运动趋势：

- 通道 1：当前 laser scan
- 通道 2：3 个控制周期前的观测（对齐到当前坐标系）
- 通道 3：7 个控制周期前的观测

历史帧先转笛卡尔坐标 → 坐标变换对齐到当前机器人系 → 重投影为距离 scan。

### 3.5 网络结构

```text
3×90 range scans
        │
        ▼
  3 层 Conv1D (提取障碍物空间特征)
        │
        ▼
  FC 256 + ReLU (压缩为特征向量)
        │
        ▼
  拼接 [目标距离, 目标方向, 当前 v, 当前 ω]
        │
        ▼
  FC 256 + ReLU
        │
        ▼
  输出层: α, β, γ, δ (经 softplus 保证 > 0)
```

## 4. 训练与泛化

- 训练环境：多种障碍物布局（走廊、迷宫、开放空间、动态障碍）
- 训练策略：curriculum learning — 从简单场景逐步过渡到复杂场景
- 泛化测试：训练时未见的障碍物布局、动态障碍配置、不同的目标位置
- 对比基线：经典 DWA（固定权重）、ADWA（自适应权重但非学习）

结果表明 DADWA 在导航成功率、路径效率、振荡抑制上均优于手工调参的 DWA/ADWA。

## 5. 对 Scout / OSCRS 的可借鉴点

### 架构层面的对应

```text
DADWA                            OSCRS
─────────────────────────────────────────────────
DWA 生成候选速度 → rollout 轨迹    G 生成候选路径
4 项加权代价函数                   G→F→R→S 多层评估
权重由网络动态预测                  权重由 launch/yaml 手工固定
PPO 在仿真中端到端训练               bag 录制 + 离线分析迭代调参
```

### 可借鉴

- **学习调参 vs 手工调参**: OSCRS 当前有大量参数（`ay_ratio_limit`, `residual_ratio`, score weights 等），这些参数的手工调优成本高且场景泛化不确定。DADWA 的"网络预测权重"思路提示：理论上可以训练一个轻量网络，根据路径几何特征（长度、曲率分布、速度剖面）预测 OSCRS gate/score 的最佳参数
- **distance-to-curvature 概念**: OSCRS 的 S 层目前没有显式考虑"候选轨迹曲率分布是否有利于接近目标"——当前只看 slosh 指标和几何平滑度。如果导航场景需要同时优化"低晃"和"朝向目标"，可借鉴 $d(\nu, \omega)$ 的思路在 score 中加入目标导向项
- **多帧历史作为输入**: 当前 OSCRS 只对单条 global_path 做决策，没有利用历史路径信息判断"这个候选的几何变化是否合理"。如果未来引入时间一致性，历史帧可作为 pipeline 输入

### 不可直接迁移

- **训练依赖大量仿真交互**: PPO 需要百万级仿真 episode，而 OSCRS 的 slosh rollout 模型（2 阶 ODE）本身保真度还在验证中，不适合作为 RL 训练环境
- **动作空间不同**: DADWA 输出 DWA 权重（4 个连续标量），OSCRS 的选择是 discrete（5 选 1 + fallback）。直接套用连续动作 RL 框架需要大幅改造
- **安全边界**: DADWA 的 DWA 本身保证运动学安全（速度空间搜索），但网络权重如果出错可能导致绕障失败。OSCRS 的 hard gate 是确定性安全边界，RL 引入的概率性决策会削弱这一保证
- **实时性差距**: DADWA 的网络推理 < 1ms，但 OSCRS 的 slosh rollout (ODE 积分) 已占主要计算。额外加入网络推理不会显著增加延迟，但训练和维护成本高

### 当前阶段的建议

DADWA 的思路对 OSCRS 的最大启发不是"加网络"，而是**把参数调优问题本身当作可优化的对象**。短期更实际的路径：
1. 先用 bag 数据系统性地分析当前参数在不同路径类型下的敏感度
2. 如果发现某些参数（如 score weights）在不同场景下最优值差异大，再考虑是否用 lookup table 或简单 heuristic 做场景自适应
3. RL 方向保留为长期探索，等待 slosh rollout 模型保真度验证通过后再评估

## 6. 术语速查

| 术语 | 含义 |
|------|------|
| DWA | Dynamic Window Approach — 在速度空间中搜索短期最优命令的经典 local planner |
| ADWA | Adaptive DWA — 根据环境自适应调整权重，但非学习方法 |
| PPO | Proximal Policy Optimization — 限制策略更新幅度的 policy gradient RL 算法 |
| model-free RL | 不显式学习环境转移模型，通过与仿真交互学习策略 |
| clipped objective | PPO 中限制策略更新幅度的目标函数 |
| beam | 激光雷达单个角度方向的测距点 |
| Conv1D | 一维卷积，在激光 scan 序列上提取局部空间模式 |
| softplus | 平滑的 ReLU 变体 $\log(1+e^x)$，保证输出为正 |
| curriculum learning | 从简单任务逐步过渡到复杂任务的训练策略 |
| sparse reward | 只在关键事件（到达/碰撞）给奖励，中间步骤无奖励信号 |
