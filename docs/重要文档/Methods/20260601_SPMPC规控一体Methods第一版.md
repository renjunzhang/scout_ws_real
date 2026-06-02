# SPMPC 规控一体 Methods（第一版，对齐当前代码）

> 日期：2026-06-01
> 范围：`src/scout_apps/control/spmpc_local_planner` 当前实现。
> 原则：本文每个公式都对应代码中的实际逻辑，不写代码里没有的机制。
> 求解器口径：确定性运动基元库 + 择优（motion-primitive MPC），**不是** MPPI，**不是** SQP/NLP。
> corridor / obstacle / guidance 属可扩展项，放附录，不进主线。

---

## 1. 概述

本文方法面向移动底盘液体运输任务，在规控一体局部 MPC 框架内显式引入液体晃动模态。
与"先规划几何路径、再用限速/限加速度后处理减晃"的两段式方案不同，本方法在同一个滚动优化中
同时考虑路径跟踪、控制平滑、终点收敛以及**预测液体模态能量**，并据此选择局部控制序列。

方法的核心有两点：

1. 将液体容器晃动建模为由底盘加速度激励的二阶模态系统，并入 MPC 的预测模型；
2. 在控制序列空间用一组**有物理含义的运动基元（motion primitives）**做参数化，
   使优化器能够选出"激励整形（excitation-shaping）"型动作（过弯前减速、horizon 中段速度谷、
   出弯后柔和恢复），而不仅是单调的整体快慢。

---

## 2. 状态与控制

机器人状态（代码 `RobotState`）：

$$
x^r_k = [\,p_{x,k},\ p_{y,k},\ \psi_k,\ v_k\,]^\top
$$

液体模态状态（代码 `SloshState`）：

$$
x^\eta_k = [\,\eta_{x,k},\ \dot\eta_{x,k},\ \eta_{y,k},\ \dot\eta_{y,k}\,]^\top
$$

联合状态 $x_k = [x^r_k;\ x^\eta_k]$，控制输入 $u_k = [a_k,\ \omega_k]^\top$。

**路径进度 `s` 不是优化状态。** 每个预测步将预测位置 $p_k$ 投影到全局参考路径上得到进度与
横向误差（代码 `ProgressProjector`，采用线段投影）。这一点与 MPCC 把 spline 进度并入优化状态的做法不同，
需在论文中明确区分。

---

## 3. 联合预测模型

### 3.1 底盘运动学

第一版采用 unicycle + 加速度输入，Euler 离散（代码 solver rollout）：

$$
\begin{aligned}
p_{x,k+1} &= p_{x,k} + \Delta t\, v_k \cos\psi_k \\
p_{y,k+1} &= p_{y,k} + \Delta t\, v_k \sin\psi_k \\
\psi_{k+1} &= \psi_k + \Delta t\, \omega_k \\
v_{k+1} &= v_k + \Delta t\, a_k
\end{aligned}
$$

Scout Mini 为四轮滑移转向（skid-steer）底盘，但其左右两侧轮速分别绑定，对外仅暴露线速度与角速度
两个控制自由度，故平面运动在运动学层面等价于单轮车模型；本文不显式建模轮胎滑移与左右轮速分配，
转向滑移视为有界扰动，由 MPC 每周期的闭环重规划吸收。

预测步内的等效加速度激励：纵向 $a_x = (v_{k}-v_{k-1})/\Delta t$，横向 $a_y = v_k\,\omega_k$（向心加速度）。

### 3.2 液体模态动力学

每个方向 $i\in\{x,y\}$ 使用二阶模态模型：

$$
\ddot\eta_i + 2\zeta\omega_n\dot\eta_i + \omega_n^2\eta_i = -k_a\, a_i
$$

其中 $\omega_n$ 为液体一阶模态自然频率、$\zeta$ 为阻尼比、$k_a$ 为加速度到模态的等效增益。
三者来自容器物理辨识（代码 `SloshDynamics::configure` 从 `slosh_models::LiquidSloshModel` 取
`omega_n` 与 `height_coeff`）。

代码以离散状态空间形式积分（`SloshDynamics::step`）：

$$
x^\eta_{k+1} = A_d\, x^\eta_k + B_d\, [\,a_{x,k},\ a_{y,k}\,]^\top
$$

$A_d, B_d$ 为上述连续二阶系统的离散化矩阵。该模型具有记忆性：当前模态状态由初始残余晃动与
过去加速度激励累积决定，因此降晃需要的是合适的**加速度时序**，而非瞬时低速。

预测液面高度（代码 `SloshDynamics::height`）：

$$
h_k = \text{height\_coeff}\cdot \|\eta_k\|,\qquad \|\eta_k\| = \sqrt{\eta_{x,k}^2 + \eta_{y,k}^2}
$$

---

## 4. 目标函数

总代价为各项加权和（代码 `CostBreakdown`，主线相关项）：

$$
J = J_{\text{contour}} + J_{\text{lag}} + J_{\text{progress}} + J_{v}
  + J_{\text{control}} + J_{\text{smooth}} + J_{\text{terminal}}
  + \lambda\,(J_{\eta} + J_{\dot\eta})
$$

（corridor / obstacle 两项见附录 A，主因果实验中关闭。）

### 4.1 路径跟踪项

给定参考路径切向 $t(s)$、法向 $n(s)$，横向 contour 误差 $e_{\text{contour}}$ 与切向 lag 误差 $e_{\text{lag}}$
由投影得到（代码 `ProgressProjector` 的 `signed_distance` / 进度差）：

$$
J_{\text{contour}}+J_{\text{lag}}
= \frac{1}{N}\sum_{k}\Big(Q_c\big(\tfrac{e_{\text{contour},k}}{e_{c,\text{ref}}}\big)^2
                       + Q_l\big(\tfrac{e_{\text{lag},k}}{e_{l,\text{ref}}}\big)^2\Big)
$$

进度奖励（鼓励前进，**为负值**，不参与误差类归一化）：

$$
J_{\text{progress}} = -Q_s\,(s_N - s_0)
$$

### 4.2 控制 / 平滑 / 终端

$$
J_{\text{control}} = \frac{1}{N}\sum_k\Big(R_a\big(\tfrac{a_k}{a_{\max}}\big)^2 + R_\omega\big(\tfrac{\omega_k}{\omega_{\max}}\big)^2\Big)
$$

$$
J_{\text{smooth}} = \frac{1}{N}\sum_k R_{\Delta a}\big(\tfrac{a_k - a_{k-1}}{a_{\max}}\big)^2
$$

终端项约束末端接近终点与低速（代码 terminal 处理）。

### 4.3 液体模态代价

$$
J_\eta = \frac{1}{N}\sum_k Q_{\text{slosh}}\Big(\frac{h_k}{h_{\text{ref}}}\Big)^2,
\qquad
J_{\dot\eta} = \frac{1}{N}\sum_k Q_{\text{slosh}}\,\rho_{\dot\eta}\Big(\frac{\|\dot\eta_k\|}{\omega_n h_{\text{ref}}}\Big)^2
$$

归一化口径与控制层 SPMPC（Route A）一致：$h_{\text{ref}}=\text{slosh\_height\_ref}$，
$\rho_{\dot\eta}=\text{slosh\_eta\_dot\_ratio}$，保证两套方法物理一致。

**所有 stage cost 在 horizon 内累加后除以步数 $N$**，与只算一次的 progress/control 同尺度，
使各权重 $Q_i$ 成为可解释、可跨路径/容器复现的相对重要性系数。

---

## 5. 数学性质与有效性证明

本节证明的目标不是宣称当前运动基元 solver 达到全局最优，而是说明：

```text
只要液体模态模型稳定且候选控制序列中存在不同激励时序，
把 J_slosh 加入同一 horizon 代价，会在候选集内偏向更低预测液面能量的控制序列；
该预测液面能量同时给出 RMS、peak 和残余晃动的上界。
```

### 5.1 模态系统的稳定性

对单个方向 $i\in\{x,y\}$，无外部激励时：

$$
\ddot\eta_i + 2\zeta\omega_n\dot\eta_i + \omega_n^2\eta_i = 0
$$

当：

$$
\omega_n>0,\qquad \zeta>0
$$

该二阶系统渐近稳定。写成状态空间形式：

$$
\dot x^\eta_i =
A_c x^\eta_i + B_c a_i
$$

其中：

$$
x^\eta_i =
\begin{bmatrix}
\eta_i \\
\dot\eta_i
\end{bmatrix},
\qquad
A_c =
\begin{bmatrix}
0 & 1 \\
-\omega_n^2 & -2\zeta\omega_n
\end{bmatrix}
$$

$A_c$ 的特征值为：

$$
\lambda_{1,2}
=
-\zeta\omega_n
\pm
j\omega_n\sqrt{1-\zeta^2}
$$

在常见欠阻尼情形 $0<\zeta<1$ 下，特征值实部均为 $-\zeta\omega_n<0$，所以系统稳定。离散化后：

$$
x^\eta_{k+1} = A_d x^\eta_k + B_d a_k
$$

其中 $A_d=e^{A_c\Delta t}$，其特征值模长小于 1，因此 $A_d$ 为 Schur 稳定矩阵。

这说明：液体晃动具有记忆性，但在无进一步激励时会自然衰减；控制器需要做的是减少 horizon 内的激励叠加和末端残余模态能量。

### 5.2 加速度序列到液面高度的映射

离散模态状态满足：

$$
x^\eta_k
=
A_d^k x^\eta_0
+
\sum_{j=0}^{k-1}
A_d^{k-1-j}B_d a_j
$$

因此预测液面高度：

$$
h_k = c_h \left\| C_\eta x^\eta_k \right\|
$$

其中 $c_h=\text{height\_coeff}$，$C_\eta$ 取出 $\eta_x,\eta_y$ 两个位置模态。

代入可得：

$$
h_k
=
c_h
\left\|
C_\eta A_d^k x^\eta_0
+
C_\eta\sum_{j=0}^{k-1}A_d^{k-1-j}B_d a_j
\right\|
$$

这说明 $h_k$ 不是当前瞬时加速度的函数，而是：

```text
初始残余晃动 x_eta_0
+ 过去/未来加速度序列经过稳定二阶系统卷积
```

所以只惩罚瞬时 $a_k$ 或 jerk，只能间接减少激励；而惩罚 $h_k$ 或 $x^\eta_k$ 是直接压低被液体模态滤波后的晃动响应。

### 5.3 slosh cost 对 RMS 和 peak 的上界

定义：

$$
J_\eta
=
\frac{Q_{\text{slosh}}}{N}
\sum_{k=0}^{N}
\left(\frac{h_k}{h_{\text{ref}}}\right)^2
$$

则有：

$$
\frac{1}{N}\sum_{k=0}^{N}h_k^2
=
\frac{h_{\text{ref}}^2}{Q_{\text{slosh}}}J_\eta
$$

因此液面 RMS 满足：

$$
h_{\mathrm{rms}}
=
\sqrt{\frac{1}{N}\sum_{k=0}^{N}h_k^2}
=
h_{\text{ref}}\sqrt{\frac{J_\eta}{Q_{\text{slosh}}}}
$$

峰值满足：

$$
h_{\mathrm{peak}}^2
=
\max_k h_k^2
\le
\sum_{k=0}^{N}h_k^2
=
\frac{N h_{\text{ref}}^2}{Q_{\text{slosh}}}J_\eta
$$

即：

$$
h_{\mathrm{peak}}
\le
h_{\text{ref}}
\sqrt{\frac{N J_\eta}{Q_{\text{slosh}}}}
$$

所以降低 $J_\eta$ 会直接降低液面 RMS，并降低液面峰值的理论上界。实际实验中 RGB `max(left, center, right)` 的 peak/p95/RMS 正是这个预测指标的外部验证。

### 5.4 残余晃动能量的上界

取模态能量型 Lyapunov 函数：

$$
V_k = (x^\eta_k)^\top P x^\eta_k,\qquad P\succ0
$$

对稳定离散系统 $A_d$，任取 $Q\succ0$，存在唯一 $P\succ0$ 满足离散 Lyapunov 方程：

$$
A_d^\top P A_d - P = -Q
$$

无后续激励时：

$$
V_{k+1}-V_k = -(x^\eta_k)^\top Q x^\eta_k < 0
$$

因此末端模态能量越低，任务结束后的残余晃动上界越低。由于 $h_k=c_h\|C_\eta x^\eta_k\|$，存在常数 $\alpha>0$ 使：

$$
h_k^2 \le \alpha V_k
$$

所以降低 horizon 末端附近的 $J_\eta+J_{\dot\eta}$，等价于降低任务结束后残余液面振荡的上界。

### 5.5 候选集内的择优保证

当前 solver 使用有限运动基元库 $\mathcal{P}$。每个基元 $p\in\mathcal{P}$ 对应一条确定性控制序列：

$$
U(p)=\{u_0(p),u_1(p),\ldots,u_{N-1}(p)\}
$$

rollout 后得到总代价：

$$
J(p)=J_{\mathrm{base}}(p)+\lambda J_{\mathrm{slosh}}(p)
$$

其中：

$$
J_{\mathrm{base}}
=
J_{\text{contour}}+J_{\text{lag}}+J_{\text{progress}}+J_v
+J_{\text{control}}+J_{\text{smooth}}+J_{\text{terminal}}
$$

solver 选择：

$$
p^\star = \arg\min_{p\in\mathcal{P}}J(p)
$$

因此对任意候选 $p$：

$$
J_{\mathrm{base}}(p^\star)+\lambda J_{\mathrm{slosh}}(p^\star)
\le
J_{\mathrm{base}}(p)+\lambda J_{\mathrm{slosh}}(p)
$$

若两个候选的路径跟踪、进度和控制代价相同，即：

$$
J_{\mathrm{base}}(p_1)=J_{\mathrm{base}}(p_2)
$$

则：

$$
J(p_1)<J(p_2)
\Longleftrightarrow
J_{\mathrm{slosh}}(p_1)<J_{\mathrm{slosh}}(p_2)
$$

也就是说，在相同运动任务代价下，加入 slosh cost 会严格选择预测晃动更低的候选。

若基础代价不同，$p_1$ 会优于 $p_2$ 的充分条件为：

$$
\lambda
\left[
J_{\mathrm{slosh}}(p_2)-J_{\mathrm{slosh}}(p_1)
\right]
>
J_{\mathrm{base}}(p_1)-J_{\mathrm{base}}(p_2)
$$

该不等式给出一个清楚解释：

```text
slosh cost 的收益必须足够大，才能抵消路径/进度/控制代价的损失。
```

这也是为什么实验需要 B0 / B_slosh / B_smooth / B_ours 消融：它们分别检验普通规控一体、模态代价、普通平滑和组合策略的贡献。

### 5.6 anti-slosh primitive 的必要性

由 5.2 可知，液面响应取决于加速度序列经过二阶系统后的卷积。对频率接近 $\omega_n$ 的激励，液面响应会被放大；对相位相反或提前减速的激励，响应可被部分抵消。

如果候选库只有线性 start/end 模板，则控制序列只能表达：

```text
整体更快 / 更慢
整体更左 / 更右
```

它不一定包含使下面卷积项变小的时序：

$$
\sum_{j=0}^{k-1}A_d^{k-1-j}B_d a_j
$$

anti-slosh primitives 通过 mid-valley、pre-turn-brake、brake-then-recover 等模板显式加入中段速度谷和提前减速，使候选集 $\mathcal{P}$ 包含更可能降低模态响应的控制序列。数学上，它不是改变 cost，而是扩展可行候选集：

$$
\mathcal{P}_{\mathrm{anti}}
=
\mathcal{P}_{\mathrm{linear}}
\cup
\mathcal{P}_{\mathrm{shape}}
$$

由于最小化是在更大的集合上进行：

$$
\min_{p\in\mathcal{P}_{\mathrm{anti}}} J(p)
\le
\min_{p\in\mathcal{P}_{\mathrm{linear}}} J(p)
$$

因此 anti-slosh primitive 至少不会使候选集内最优总代价变差；若新增模板提供了更低的 slosh 响应且基础代价损失不超过 5.5 的阈值，则会被 solver 选中。

---

## 6. 求解：激励整形运动基元 MPC（核心）

本方法不直接对每个时刻的控制量做梯度优化，而是在控制序列空间用一组**确定性运动基元**做参数化，
逐个在联合机器人–液体模型上 rollout、评分，**择优（argmin）**输出第一步控制：

$$
u^\star = \arg\min_{p\in\mathcal{P}}\ J\big(\mathrm{rollout}(x_k,\ p)\big)
$$

### 6.1 基元参数化

每个基元 $p$ 由三段速度/角速度尺度 $(\text{start}, \text{mid}, \text{end})$ 描述（代码 `makePiecewiseControls`），
在 horizon 上分两段线性插值生成完整控制序列。关键在于 **mid 段可低于两端**，
因此能表达 linear start–end 参数化无法表达的激励整形时序：

```text
mid-valley        过弯/高风险段中部降速谷
pre-turn-brake    入弯前提前减速
brake-then-recover 减速后柔和恢复
jerk-limited recovery 限加加速度恢复
```

### 6.2 基元库

代码 `makePrimitiveDescs` 按 variant 给出基元库 $\mathcal{P}$：

```text
linear 模式:     start/end 两端尺度的网格组合 (144 个候选, mid = 两端均值, 等价单调过渡)
anti_slosh 模式: 在 linear 基础上额外加入 8 个激励整形模板 (见 5.1)
```

### 6.3 与 MPPI / SQP 的区别（论文须写清）

- **不是 MPPI**：MPPI 在控制序列上加随机噪声采样并按 cost 指数加权平均；本方法是
  **确定性、有物理含义的基元库 + argmin 择优**，无随机扰动、无加权平均。
- **不是 SQP/NLP**：不构造可微 QP、不做梯度迭代；每个基元独立 rollout 后直接比较 cost。

这一参数化属于"控制参数化 / 基函数 MPC（input parameterization / move blocking）"谱系，
其优点是无需可微求解器、动作可解释；代价是解空间受基元库覆盖限制（见 §7 局限）。

---

## 7. 滚动执行与闭环

每个控制周期（代码 `controlTimerCallback`）：

```text
1. 读机器人状态 (odom) 与液体模态观测 (slosh observer);
2. 对参考路径做投影, 得到当前进度与横向误差;
3. 以观测到的液体状态作为预测初值 x^eta_0;
4. 遍历基元库, 每个基元在联合模型上 rollout, 计算 J;
5. 取 J 最小的基元, 输出其第一步控制 (a, omega);
6. 下周期用新观测重新求解。
```

液体初值由 **slosh observer**（odom/IMU 加速度推进的模态估计，代码 `SloshDynamics::step`
+ ROS 层差分）每周期重置。因此整体为闭环：观测器用实测加速度更新当前模态 → 注入预测初值 →
基元择优 → 执行 → 新观测。需区分两类积分：

- **观测器更新（observer update）**：用已发生的实测加速度，将模态推进到当前周期（面向过去，作初值）；
- **horizon 预测（prediction）**：用基元假设的未来控制，预测未来模态（面向未来，进 cost）。

二者用同一个模态模型，但输入与用途不同；闭环体现在每周期预测起点都被实测刷新。

液面真值（RGB 视觉 max(left, center, right)）仅用于离线评价，**不进控制环**；控制环内的 $h_k$
是模型预测值。

### 7.1 实现时间尺度

当前 SPMPC 与既有 `scout_local_planner` 实物主线对齐：

```text
控制频率: 30 Hz
控制周期: 1/30 s ≈ 0.0333 s
预测步长: dt = 1/30 s
horizon_steps: 60
预测时长: 约 2.0 s
```

即每 33 ms 重新求解一次，并只执行最优候选控制序列的第一步。该设置与旧控制器 MPC 的
`control_rate=30.0`、`mpc/dt=0.0333333333`、`N=60` 对齐，方便 Route A 与 Route B 的实物数据比较。

---

## 8. 局限（诚实写入论文）

基元库参数化的表达力受库覆盖限制：优化器只能在给定基元中择优，
若最优的激励整形时序不在库内则无法生成。当实验表明基元库不足以产生显著降晃时，
可将求解器升级为对 $u_{0:N-1}$ 的连续梯度优化（SQP/OSQP），作为后续工作。

---

## 9. 对比实验设计

实验目标不是证明本方法在所有局部规划任务上优于所有 planner，而是验证两个明确问题：

```text
Q1: 在同一固定几何路径上，把液体模态状态纳入局部规控一体目标，是否能降低液面晃动？
Q2: 该收益是否超过普通平滑控制，且不是由更慢、更绕或终点停车策略造成？
```

因此实验分为固定路径主实验、内部消融、外部 planner 对比和点到点泛化。

### 9.1 固定路径主实验：无障碍 P2 S 曲线

主实验使用固定路径，场景无障碍：

```text
路径: P2_s_curve 固定几何路径
起点: 每次回到同一实物起点附近
终点: 固定 goal pose
环境: 无障碍或障碍不参与局部决策
视觉: RGB 液面高度, max(left, center, right)
```

选择无障碍固定路径的原因是：

```text
1. 锁定几何路径，避免“绕路少晃”污染结论；
2. 不让避障能力成为变量；
3. 直接考察 slosh-aware local planning 对速度/角速度时序的影响；
4. 方便与已有 Route A fixed-path 数据对齐。
```

主评价窗口：

```text
TRACKING main window = tracking start -> terminal 前 1 s
```

terminal approach 单独分析，不混入主效果统计。原因是 terminal 停车存在独立 jerk/制动问题，
如果混入主窗口，会把“终点停车干净与否”和“正常跟踪阶段是否抑晃”混在一起。

### 9.2 内部消融：证明贡献来自哪里

内部消融使用同一个 `spmpc_local_planner`、同一条路径、同一套输入输出接口，只改变 cost/primitive 配置。

| 组别 | 配置 | 作用 |
|---|---|---|
| B0 | 无 slosh cost，普通 integrated MPC 原型 | 基准 |
| B_slosh | B0 + slosh horizon cost，linear primitive | 证明模态代价单独是否有效 |
| B_smooth | B0 + 更强 control/smooth cost，无 slosh | 证明普通平滑能降多少 |
| B_ours | B_slosh + B_smooth | 证明模态代价和平滑组合 |
| B_slosh_linear | slosh cost + linear primitive | 检查只有线性候选时 slosh cost 的作用 |
| B_slosh_anti | slosh cost + anti-slosh primitive | 证明激励整形基元是否进一步有效 |
| B_ours_anti | slosh + smooth + anti-slosh primitive | 当前完整方法候选 |

关键对比关系：

```text
B_slosh vs B0:
  证明 slosh modal cost 是否改变行为并降低预测/视觉晃动。

B_smooth vs B0:
  证明普通平滑项能带来多少收益。

B_ours vs B_smooth:
  证明 slosh-aware 是否优于 smooth-only。

B_slosh_anti vs B_slosh_linear:
  证明不是只靠增加 Q_slosh，而是需要能表达减晃时序的候选库。

B_ours_anti vs B0/B_smooth:
  证明当前完整方法相对普通 integrated MPC 和 smooth-only 的收益。
```

### 9.3 外部 planner baseline

外部 baseline 用于工程泛化，不作为固定路径主因果证明的唯一依据。当前计划：

| 方法 | 来源 | 运行方式 | 论文角色 |
|---|---|---|---|
| TEB | ROS1 `teb_local_planner` | 通过 `baseline_local_planner_runner` 脱离 `move_base` | 经典优化型 local planner |
| DWA | ROS1 `dwa_local_planner` | 通过同一 runner 脱离 `move_base` | 传统速度采样下界 |
| mpc_local_planner | rst-tu-dortmund `mpc_local_planner` | isolated build + runner | 非晃液 MPC local planner baseline |
| SPMPC | 本文方法 | `spmpc_local_planner` | slosh-aware integrated local planner |

TEB / DWA / `mpc_local_planner` 本体仍是 `nav_core::BaseLocalPlanner` 插件，但实验中不启动 `move_base`。
统一由 `baseline_local_planner_runner` 调用：

```text
输入:
  /scout/global_path 或 /scout/global_path_fixed
  /scout/goal
  /odom
  /map
  /scan_front
  /tf

输出:
  /cmd_vel
  /baseline/<name>/status
  /baseline/<name>/global_plan
```

这样保证外部 planner 和 SPMPC 在实验接口上尽量一致。

### 9.4 公平性约束

固定路径主实验中必须固定：

```text
同一 P2_s_curve 几何路径
同一 goal pose
同一容器、液面高度、相机参数
同一 RGB 标定和 HSV 阈值
同一主评价窗口
同一速度/角速度/加速度安全上限
同一 rosbag 话题集合
```

对外部 planner，额外要求：

```text
1. 不允许用更宽路径偏离换取低晃；
2. completion time 若相差超过 ±10%，结果必须按 trade-off 解释；
3. path deviation 必须报告，不能只报告液面指标；
4. 若 planner 因避障或路径形状改变导致几何不同，该结果只能作为工程泛化，不进入主因果表。
```

### 9.5 评价指标

主指标使用视觉液面高度，采用 RGB 三标尺流程得到：

$$
h_{\mathrm{rgb}}(t)
=
\max
\left(
h_{\mathrm{left}}(t),
h_{\mathrm{center}}(t),
h_{\mathrm{right}}(t)
\right)
$$

主窗口内统计：

```text
RGB p95
RGB RMS
RGB peak
RGB AUC_tau
model h_peak_pred / h_p95_pred
cmd_v / odom_v
ax / ay / jerk
completion time
path deviation
solver time
```

其中：

$$
\mathrm{AUC}_{\tau}
=
\int
\max(0,h_{\mathrm{rgb}}(t)-\tau)\,dt
$$

若按路径进度对齐，则写成：

$$
\mathrm{AUC}_{\tau}^{s}
=
\int
\max(0,h_{\mathrm{rgb}}(s)-\tau)\,ds
$$

`model h_peak_pred / h_p95_pred` 只作为模型侧解释指标，不替代 RGB 真值。

### 9.6 统计与判定

每组建议至少：

```text
pilot: n = 3
正式: n >= 5
```

正式实验采用 randomized block：

```text
每个 block 内包含所有方法各一次；
方法顺序随机；
block 前后录 static bag 检查视觉漂移。
```

判定逻辑：

```text
若 B_ours_anti 相对 B0 同时降低 RGB p95/RMS/peak 中至少两个指标，
且 completion time 没有超过 ±10% 公平性带，
则认为 slosh-aware integrated planning 在主窗口有效。

若液面下降但 completion time 明显增加，
则写成晃动-时间 trade-off 改善，不写成无代价优越。

若 model 指标下降但 RGB 不下降，
只能说明模型侧预测改善，不能宣称真实液面改善。
```

统计检验：

```text
4 组以上 matched block:
  Friedman test + paired Wilcoxon post-hoc + Holm correction

两组对比:
  paired Wilcoxon signed-rank test
```

曲线图只做描述性展示，不逐点做显著性声明；显著性只对 per-run 标量指标做。

### 9.7 terminal approach 单独诊断

terminal 前 1 s 单独统计：

```text
v_ref / cmd_v / odom_v
odom_ax / odom_jerk
model slosh peak
terminal mode
capture / reached 时刻
```

该窗口只用于诊断终点停车是否干净，不用于证明 slosh cost 主效果。若 terminal 处出现 jerk 脉冲，
应单独修 terminal envelope 或停止逻辑，而不是把它归因到 SPMPC 主方法。

---

## 附录 A：corridor / obstacle / guidance（可扩展项，非主线）

代码已实现以下 Phase 3 能力，但在固定路径主因果实验中关闭，仅用于点到点工程泛化：

- **corridor**：将横向误差约束在管道内 $|e_{\text{contour}}| \le w_{\text{corridor}}/2$，
  违反进 $J_{\text{corridor}}$ 软惩罚，可选硬剔除越界基元（代码 `CORRIDOR_REJECT`）。
- **obstacle**：静态 OccupancyGrid 代价，沿预测轨迹取邻域最大代价归一化进 $J_{\text{obstacle}}$
  （代码 `CostmapGrid::maxCostInRadius`）。避障由全局规划器负责，本方法只在其无碰撞路径附近做局部优化。
- **guidance**：center/left/right 三路横向偏置候选（代码 `makeGuidanceBiases`），
  属轻量 guidance，不实现动态障碍预测 / H-signature 分类 / 多 solver topology。

这些在主线 method 中不展开，避免把绕行能力混入液体模态代价的贡献归因。
