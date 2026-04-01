# slosh_models 模型整理与 MPC 接入说明

本文档基于当前代码实现整理 `slosh_models` 的液体晃动模型，并给出将其接入
`/home/a/scout_ws/src/scout_apps/control/scout_local_planner` 时需要注意的事项。
重点不是泛泛讲液体晃动原理，而是严格按照当前仓库代码说明：

- `slosh_models` 当前到底实现了什么
- “线性/非线性”在当前代码里的真实含义
- 融入 `scout_local_planner` 时最容易踩的坑
- 针对 `local_planner_ros.cpp`、`cost_function.cpp`、`constraint_manager.cpp`
  的函数级改造清单

文中参考标注说明：

- `[R1]` 液体晃动主参考论文：`"Sloshing Dynamics Estimation for Liquid-filled Containers under 2-Dimensional Excitation"`（ECCOMAS 2021），当前仓库 `config/slosh_params.yaml` 已直接标注该来源
- `[R2]` 模态根数值表来源：`Abramowitz & Stegun`，当前代码用其表值近似 `J'_1(x)=0` 的前几阶根
- `[R3]` OSQP 标准 QP 形式：`0.5 * z^T P z + q^T z,  l <= A z <= u`，本文用于说明当前 `scout_local_planner`/`mpc_vel_tracker` 的矩阵系数写法
- `[C1]` 当前仓库已有的工程实现参考，不是论文：`slosh_models/src/mpc_vel_tracker.cpp`

---

## 1. 当前 slosh_models 包结构

代码入口：

- `include/slosh_models/liquid_slosh_model.h`
- `src/liquid_slosh_model.cpp`
- `include/slosh_models/mpc_vel_tracker.h`
- `src/mpc_vel_tracker.cpp`
- `config/slosh_params.yaml`

角色分工：

- `LiquidSloshModel`：液体晃动动力学模型本体
- `MPCVelTracker`：一个独立的“速度 MPC + 晃动约束”示例，不是当前 `scout_local_planner` 直接调用的主控制器

对当前工程最有价值的是 `LiquidSloshModel`，因为 `scout_local_planner` 已经有：

- 增广状态索引 `ETA_X ... ETA_Y_DOT`
- `SloshIntegration`
- `DiffDriveModel::predict()/linearize()` 中的晃动耦合接口

当前真正缺的是闭环接线和代价/约束落地。

---

## 2. LiquidSloshModel 当前实现整理

### 2.1 状态、输入、输出

在 `LiquidSloshModel` 中：

- 状态：`[xn, vxn, yn, vyn]`
- 输入：`[ax, ay]`
- 输出：液面晃动高度 `eta`

对应代码：

- 头文件：`include/slosh_models/liquid_slosh_model.h`
- 实现：`src/liquid_slosh_model.cpp`

物理语义：

- `xn, yn`：主模态位移
- `vxn, vyn`：主模态速度
- `ax, ay`：容器在底盘坐标系中的平面加速度输入

### 2.2 configure() 做了什么

`configure(const Params&)` 的工作：

1. 读取容器几何参数 `R, h, rho`
2. 根据 `mode_index` 选取模态根 `xi_1n` `[R2]`
3. 计算液体总质量 `m_F`
4. 计算模态固有频率 `omega_n`（代码注释对应论文式(3)）`[R1]`
5. 计算模态质量 `m_n`（代码注释对应论文式(4)）`[R1]`
6. 计算等效刚度 `k_n` 与阻尼 `c_n`
7. 计算液面高度系数 `height_coeff`（代码注释对应 L/NL 输出式）`[R1]`
8. 构建连续状态矩阵
9. 做 ZOH 离散化，得到 `A_discrete_`, `B_discrete_`

这里最关键的是：当前模型的状态演化始终是线性 MSD 模型，离散化后仍然是线性离散系统。

### 2.3 update() 的真实含义

`update(const Eigen::Vector2d& accel_base, double omega_z, double alpha_z)`
在当前代码里做了两件事：

1. 先根据容器偏心距做旋转修正：

```text
a_cx = a_x - alpha_z * r_y - omega_z^2 * r_x
a_cy = a_y + alpha_z * r_x - omega_z^2 * r_y
```

2. 再用线性离散模型更新：

```text
x_{k+1} = A_d * x_k + B_d * u_k
```

所以要注意：旋转修正发生在输入层，不是在状态矩阵里引入非线性项。
这里的旋转修正写法来自当前代码对论文旋转项说明的工程化实现，可视为
`[R1]` 的输入层补偿，而不是新的状态方程推导。

### 2.4 getSloshHeight() 的真实含义

当前 `getSloshHeight()` 返回的是：

```text
eta_total = eta_slosh + eta_parabola
eta_slosh = height_coeff * sqrt(xn^2 + yn^2)
eta_parabola = R^2 * omega_z^2 / (4g)   // 仅当 use_parabola_term=true
```

其中：

- `eta_slosh` 对应当前代码中基于主模态位移恢复液面高度的部分，来源于 `L/NL` 高度映射 `[R1]`
- `eta_parabola` 对应旋转抛物面静态抬升项，当前代码按 `Lp` 解释使用 `[R1]`

这意味着：

- `eta_slosh` 来自模态位移，是状态的非线性输出映射
- `eta_parabola` 是纯角速度项，不属于模态状态本身

这个区别在接入 MPC 代价和约束时非常重要。

---

## 3. “线性/非线性”在当前代码中的真实含义

### 3.1 当前 use_linear_model 不会把 MPC 变成非线性 MPC

`use_linear_model` 的作用是切换 `height_coeff` 的计算公式：

- 线性模型：

```text
height_coeff = 4 * h * m_n / (m_F * R)
```

- 非线性模型：

```text
height_coeff = xi_1n^2 * h * m_n / (m_F * R)
```

这两个输出系数公式在当前代码注释里分别对应线性 `L` 模型和非线性 `NL`
模型 `[R1]`。但注意这里切换的是输出映射系数，不是状态转移矩阵的结构。

但无论 `use_linear_model` 是 `true` 还是 `false`：

- `A_discrete_`
- `B_discrete_`
- `update()`

仍然是线性 MSD 离散状态更新。

结论：

- 当前“非线性”主要体现在高度输出映射更换
- 不是“状态方程变成非线性”
- 不是“当前 OSQP/QP 会自动变成 NMPC”

### 3.2 use_parabola_term 是输出修正，不是预测模型修正

`use_parabola_term=true` 时，额外给液面高度加：

```text
R^2 * omega_z^2 / (4g)
```

这也是输出层的非线性修正，不会自动反映到 `scout_local_planner`
里的预测模型和线性化矩阵。

因此：

- 可以用它做离线指标、在线监控、论文图表
- 不应该在第一轮就直接作为 QP 里的硬约束表达式

---

## 4. 融入 scout_local_planner 时最需要注意的事情

### 4.1 先区分“预测模型”与“指标输出”

建议分两层处理：

- 预测和优化层：继续保持线性增广模型
- 指标和物理解释层：允许用 `eta_total` 这种非线性输出

如果一上来就试图把 `sqrt(xn^2+yn^2)` 或 `omega^2` 直接塞进 OSQP，
会立刻脱离当前 QP 架构。

### 4.2 对四轮差速底盘，`ay = v * omega` 只是低速近似

当前 `scout_local_planner` 里，slosh 侧向激励的核心近似是：

```text
ay = v * omega
```

这对理想差速/自行车模型是常见近似，但你的实物是四轮差速（skid-steer）：

- 低速时可用
- 高速急弯时会有滑移和侧偏
- 实际容器横向激励不一定等于 `v * omega`

因此建议：

- slosh 第一轮实验先限制在低速
- 实物先不要追求“高速大角速度下绝对准确”
- 后续如果有 IMU，可优先用 IMU 的横向加速度替代 `v * omega`

### 4.3 偏心项先不要急着开

`LiquidSloshModel::update()` 里已经支持：

- `offset_x`
- `offset_y`
- `alpha_z * r`
- `omega_z^2 * r`

但当前 `DiffDriveModel` 里的晃动预测只显式用了：

- `ax = a`
- `ay = v * omega`

没有把偏心旋转项完整带入预测线性化。

如果估计器更新时用了偏心项，而 MPC 预测里没用，会出现：

- 估计模型和预测模型不一致
- 闭环效果变差，难以判断问题来源

建议第一轮：

- `offset_x = 0`
- `offset_y = 0`
- 先把主链路跑通

### 4.4 加速度估计必须滤波

`local_planner_ros` 如果直接用 `odom` 差分估计 `ax` 和 `alpha_z`：

- 实物噪声会很大
- 晃动状态会被虚假高频激励
- 代价/约束效果会被污染

建议：

- 至少加入一阶低通滤波
- 或者先只用 `ay = v * omega`，`alpha_z` 先置 0
- 先保证估计稳定，再逐渐加复杂项

### 4.5 当前 readFromAugmentedState() 还是空壳

`SloshIntegration::readFromAugmentedState()` 现在没有真正把增广状态写回
`LiquidSloshModel` 内部状态，因为 `LiquidSloshModel` 还没有 `setState()`。

这意味着：

- 只能从内部模型写到增广状态
- 不能反向把优化/估计状态同步回内部模型

如果后面要做：

- warm-start reset
- 路径切换后的状态重建
- 外部观测器回写

就需要先给 `LiquidSloshModel` 增加 `setState()`

### 4.6 软代价与硬约束不要一起上

建议顺序：

1. 先接线 + 状态注入
2. 再加 slosh soft cost
3. 最后再尝试硬约束

原因：

- 如果一开始 soft/hard 一起上，一旦不可行或效果不明显，很难定位问题
- 第一篇实验更需要先证明“Q_slosh 改变后指标会动”

---

## 5. 关于线性/非线性的接入建议

### 5.1 第一版推荐配置

对当前 `scout_local_planner`，建议第一版使用：

- `use_linear_model: true`
- `use_parabola_term: true`
- `offset_x: 0.0`
- `offset_y: 0.0`

理由：

- 预测模型仍是线性 QP 友好
- 高度输出保留转弯时的物理修正
- 避免偏心项导致估计/预测不一致

### 5.2 代价函数该惩罚什么

第一版建议惩罚：

- `ETA_X`
- `ETA_Y`

并用：

```text
Q_eta = Q_slosh * height_coeff^2
```

把 YAML 中的 `Q_slosh` 解释成“惩罚液面高度平方”的物理权重。
这个写法和 `[C1]` 中 `MPCVelTracker::buildAugmentedMatrices()` 的 Hessian 构造一致，
也与 `eta^2 = height_coeff^2 * (xn^2 + yn^2)` 的展开一致 `[R1]`。

不建议第一版直接惩罚：

- `getSloshHeight()` 的非线性表达式
- 包含 `omega^2` 的总高度

因为这会破坏当前 QP 结构。

### 5.3 硬约束该怎么理解

如果未来要做 `slosh_height_max` 的保守代理约束，建议先约束模态位移盒：

```text
|eta_x| <= eta_bar
|eta_y| <= eta_bar
```

如果目标是约束“总液面高度”，又启用了 `use_parabola_term=true`，
则要注意：

```text
eta_total = eta_modal + eta_parabola
eta_parabola = R^2 * omega^2 / (4g)
```

这意味着给模态位移分配的高度预算应该是：

```text
eta_modal_budget = slosh_height_max - eta_parabola_budget
```

如果直接用 `slosh_height_max / height_coeff` 做盒约束，会忽略转弯抛物面项，
得到的并不是“总高度约束”。

---

## 6. 当前代码下的函数级改造清单

以下清单按当前仓库状态整理，目标是把 slosh 真正接进 `scout_local_planner`。

### 6.1 local_planner_ros.cpp

目标：把 `SloshIntegration` 真正接入闭环，保证 `x0` 的晃动状态非零且随运动更新。

#### loadParameters()

需要补的内容：

- 读取 `mpc/slosh_height_max`
- 读取 `slosh/*` 参数：
  - `container_radius`
  - `liquid_height`
  - `liquid_density`
  - `mode_index`
  - `damping_ratio`
  - `offset_x`
  - `offset_y`
  - `use_linear_model`
  - `use_parabola_term`
- 读取加速度估计滤波参数，例如：
  - `slosh_estimator/alpha_ax`
  - `slosh_estimator/alpha_alpha`

注意：

- `slosh_params.dt` 必须与 `mpc_params_.dt` 一致
- `slosh_height_max` 当前只在 `types.h` 和 yaml 里定义，运行时还没有真正读入

#### initialize()

需要补的内容：

- 实例化并配置 `SloshIntegration`
- 创建 `std::shared_ptr<DiffDriveModel>`
- 如果 slosh 启用，则调用 `model->setSloshIntegration(&slosh_integration_)`
- 用 `mpc_solver_.setDynamicsModel(model)` 替换默认模型
- 新增调试发布器：
  - `/slosh/state`
  - `/slosh/height`
  - 可选 `/slosh/ax_est`, `/slosh/ay_est`

注意：

- `control_rate` 与 `mpc/dt` 最好保持一致或接近
- 如果 `Q_slosh <= 0`，可以仍然配置 slosh 模型做观测，但不要在 QP 中启用 slosh 代价

#### odomCallback()

建议补充：

- 缓存上一时刻 `v`, `omega`, `stamp`
- 为 `controlLoop()` 的加速度估计准备数据

注意：

- 不建议在 `odomCallback()` 内直接更新 slosh 状态，最好仍在控制周期里统一做

#### controlLoop()

需要补的内容：

1. 用 `odom` 差分估计：
   - `ax_est`
   - `alpha_z_est`
2. 对估计量做低通滤波
3. 计算 `ay_est`
   - 第一版继续用 `current_v_ * current_omega_`
4. 调用：
   - `slosh_integration_.update(ax_est, ay_est, current_omega_, alpha_z_est)`
5. 构造 `current_state` 后调用：
   - `slosh_integration_.writeToAugmentedState(current_state)`
6. 发布调试 topic：
   - 当前 slosh 状态
   - 当前 slosh 高度

注意：

- 如果控制器进入 `REACHED` 或 `ERROR`，要考虑是否 reset slosh 状态
- 建议第一版只在显式 stop/reset 时清零 slosh，不要在每次新路径时清零

#### resetWarmStart()

建议补充策略说明：

- `resetWarmStart(false)` 是否同时 reset slosh 状态，需要显式定义
- 推荐：
  - 到达终点可 reset slosh
  - 相似路径更新不要 reset slosh
  - 显著重规划可保留或清零，取决于你希望“物理连续”还是“优化问题重置”

---

### 6.2 cost_function.cpp

目标：把 slosh 作为 QP 中真正生效的软代价接进去。

#### buildQPCost()

第一版建议直接在这里加，不必先抽象新类。

需要补的内容：

- 当 `params_.Q_slosh > 0` 且 slosh 模型已配置时：
  - 获取 `height_coeff`
  - 构造 `Q_eta = Q_slosh * height_coeff^2`
  - 在 `Q_total` 的 slosh 状态子块上添加二次项

推荐第一版只加：

```text
ETA_X
ETA_Y
```

也就是：

```text
Q_total(ETA_X, ETA_X) += Q_eta
Q_total(ETA_Y, ETA_Y) += Q_eta
```

不建议第一版就加：

- `ETA_X_DOT`
- `ETA_Y_DOT`

因为先把“位移抑制是否有效”跑通更重要。

注意 1：OSQP 系数约定

当前项目目标函数形式是：

```text
0.5 * z^T H z + g^T z
```

因此 slosh 的 Hessian 写法必须与当前仓库已有写法一致，不能单独乱乘 2。
这里的系数约定应统一按 OSQP 标准形式理解 `[R3]`。

注意 2：不要直接把 `getSloshHeight()` 放进 QP

因为：

- `sqrt(eta_x^2 + eta_y^2)` 非线性
- `use_parabola_term` 还会引入 `omega^2`

当前 QP 第一版只做二次近似。

#### initialize()

如果后面你决定把 slosh 抽象成独立 cost term 类，那么：

- 在 `initialize()` 中按条件挂入 `SloshCost`

但从当前工程进度看，第一版不需要为了抽象层次增加复杂度。

---

### 6.3 constraint_manager.cpp

目标：把 `slosh_height_max` 在后续阶段落地为保守线性约束代理。

当前建议：这一块放在 `P1`，不要作为第一轮接入动作。

#### initialize()

后续如果启用 slosh 约束，需要：

- 注册新的 `SloshDispBoundsConstraint`

#### totalConstraints()

需要补：

- 把 slosh 状态约束行数计入总约束数
- 保证 `(N+1)` 个状态步都被正确计数

#### buildQPConstraints()

需要补的内容：

- 对每个 `k = 0..N`，增加 `ETA_X`、`ETA_Y` 两行约束
- 约束形式先做盒约束：

```text
-eta_bar <= ETA_X <= eta_bar
-eta_bar <= ETA_Y <= eta_bar
```

其中 `eta_bar` 的推荐定义不是直接：

```text
slosh_height_max / height_coeff
```

而是更保守地考虑 `L_inf` 近似：

```text
eta_bar = eta_modal_budget / (height_coeff * sqrt(2))
```

如果启用了 `use_parabola_term=true`，则总高度预算还要预留抛物面项：

```text
eta_modal_budget = slosh_height_max - eta_parabola_budget
eta_parabola_budget = R^2 * omega_budget^2 / (4g)
```

这个预算拆分不是论文直接给出的 QP 约束形式，而是基于 `[R1]` 总高度表达式
对当前线性 QP 架构做的工程化保守代理。

注意：

- 如果 `eta_modal_budget <= 0`，说明这个硬约束在当前角速度预算下根本不可行
- 这就是为什么第一轮不建议直接上硬约束

---

## 7. 推荐接入顺序

### 阶段 A：先让状态活起来

改：

- `local_planner_ros.cpp`

目标：

- `/slosh/state` 非零
- `/slosh/height` 随加减速和转弯变化

### 阶段 B：再让代价生效

改：

- `cost_function.cpp`

目标：

- `Q_slosh=0` 与 `Q_slosh>0` 时，`eta_max` 或 `eta_rms` 出现可解释差异

### 阶段 C：最后再试约束

改：

- `constraint_manager.cpp`

目标：

- 限制晃动峰值
- 同时不可行率可接受

---

## 8. 建议的第一版实验口径

对当前四轮差速实物，建议第一版实验这样做：

- 低速
- `use_linear_model: true`
- `use_parabola_term: true`
- `offset_x = 0`
- `offset_y = 0`
- 只做 soft cost
- 不做 hard constraint

先回答两个最关键的问题：

1. slosh 状态有没有真实进入闭环
2. `Q_slosh` 改变后，晃动指标有没有变化

如果这两点都不能回答清楚，就不应该继续往更复杂的约束和论文图表推进。

---

## 9. 参考文献与来源对照

### [R1] 液体晃动主参考

- `Sloshing Dynamics Estimation for Liquid-filled Containers under 2-Dimensional Excitation`
- 当前仓库明确出处：
  - `slosh_models/config/slosh_params.yaml`
  - `slosh_models/src/liquid_slosh_model.cpp` 中关于式(3)、式(4)、式(23)、式(24) 与 `Lp` 的注释
- 本文档中引用它的内容：
  - 固有频率 `omega_n`
  - 模态质量 `m_n`
  - 高度系数 `height_coeff`
  - `eta_slosh`
  - `eta_parabola`

### [R2] 模态根数值表

- `Abramowitz & Stegun`
- 当前仓库使用位置：
  - `slosh_models/src/liquid_slosh_model.cpp`
  - 用于 `J'_1(x)=0` 的前几阶模态根常数表 `MODAL_ROOTS`

### [R3] OSQP 标准 QP 形式

- 标准形式：`0.5 * z^T P z + q^T z,  l <= A z <= u`
- 当前仓库对应位置：
  - `slosh_models/src/mpc_vel_tracker.cpp`
  - `scout_local_planner` 的 `cost_function.cpp` / `constraint_manager.cpp` 采用同一写法
- 本文档中引用它的内容：
  - 为什么 Hessian/gradient 系数要严格对齐
  - 为什么不能把非线性 `sqrt()` / `omega^2` 直接原样塞进当前 QP

### [C1] 仓库内实现参考

- `slosh_models/src/mpc_vel_tracker.cpp`
- 作用：
  - 给出当前仓库里已经跑通过的 `Q_slosh * h_coeff^2` Hessian 写法
  - 可作为 `scout_local_planner` 接入 slosh 软代价时的实现对照
- 注意：
  - 这是工程实现参考，不是论文来源

---

## 10. 面向当前项目联调的“液体建模速览”（2026-03）

这一节用于把本文与近期项目文档对齐，便于快速回答两个问题：

1. 当前到底在用什么液体模型；
2. 这些模型/公式分别来自哪篇论文，哪些属于工程化近似。

### 10.1 当前液体建模的最小闭环结论

基于当前仓库状态（`docs/change_log2.md`、`docs/mpc_数学化架构总结.md`、
`docs/总结1.md`）可以把“anti-slosh MPC”概括为：

- **模型层**：4 维二维 MSD 晃动子系统（`ETA_X, ETA_X_DOT, ETA_Y, ETA_Y_DOT`）
  与 Frenet 跟踪状态增广，形成 8 维状态 QP 预测模型；
- **代价层**：通过 `Q_slosh_eta = Q_slosh * height_coeff^2` 惩罚
  `ETA_X^2 + ETA_Y^2`，属于软抑制；
- **约束层（可选）**：对 `ETA_X/ETA_Y` 施加盒约束（保守代理，不是直接总液面约束）；
- **外环治理（可选）**：speed governor 基于液面风险调低 `v_des`，不改变 QP 结构。

这也解释了“晃动是加在代价还是模型”这个常见问题：

- 不只是代价，也不只是模型；
- 当前是“**模型 + 代价 + 可选约束 + 可选外环治理**”的协同方案。

### 10.2 当前“线性/非线性”边界（再次强调）

当前工程里，优化器主问题仍是 OSQP 的线性二次规划（QP）：

- 状态更新是线性离散模型；
- 软代价是二次型；
- 约束是线性等式/不等式（含盒约束代理）。

“非线性”主要体现在：

- 输出层的高度映射（`sqrt(xn^2+yn^2)`）；
- 可选旋转抛物面项（`omega_z^2`）。

因此当前不是 NMPC，而是“带非线性物理解释输出的线性 QP MPC”。

### 10.3 参考论文与工程化近似的对应关系

#### A) 核心论文（主模型来源）

- **[R1]** `Sloshing Dynamics Estimation for Liquid-filled Containers under 2-Dimensional Excitation`
  （ECCOMAS 2021）

当前直接采用/对应的内容：

- 一阶模态频率 `omega_n` 与模态质量 `m_n` 的计算；
- 线性/非线性高度映射系数（`height_coeff` 的两种写法）；
- 旋转抛物面项 `Lp` 对应的高度修正解释。

#### B) 数值常数来源（不是晃动理论主文）

- **[R2]** Abramowitz & Stegun（表值）

当前用途：

- 近似 `J'_1(x)=0` 的前几阶根（`MODAL_ROOTS`），用于模态参数计算。

#### C) 优化求解形式来源（不是液体动力学论文）

- **[R3]** OSQP 标准 QP 形式

当前用途：

- 统一目标函数 Hessian/gradient 系数写法；
- 约束矩阵 `l <= A z <= u` 的工程组织方式；
- 解释为何第一版不能把 `sqrt()` 和 `omega^2` 原样当作硬约束塞入 QP。

#### D) 工程参考（仓库内）

- **[C1]** `slosh_models/src/mpc_vel_tracker.cpp`

当前用途：

- 作为 `Q_slosh * height_coeff^2` 写法的“已跑通实现模板”；
- 用于对照 `scout_local_planner` 的接入方式。

### 10.4 当前阶段的模型可信边界（实物 37 mm 试管）

当前实物主参数按 37 mm 水试管采集阶段初值设置（半径 0.0185 m、液高 0.058 m），
阻尼比先采用中性初值（`damping_ratio=0.05`）。

现阶段结论是：

- 该模型可作为“低阶风险代理”用于控制抑制；
- 但尚未完成基于视频/传感器的 `omega_eff`、`zeta_eff` 精细辨识闭环；
- 因此更适合做“趋势控制与相对对比”（如 `Q_slosh=0/5/10`），
  而不是宣称“绝对液面高度高精度重建”。
