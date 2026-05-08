# Muchacho 2022 方法剖析

论文：Muchacho, Laha, Figueredo, Haddadin, "A Solution to Slosh-free Robot Trajectory Optimization", arXiv:2210.12614v1, 2022（TUM Munich Institute of Robotics & ML, IROS 2022 接收 [Ref-Muchacho2022]）。

剖析日期：2026-05-07
剖析对象：与 Ferrari 2026 形成对照——同样是"用模型生成 slosh-free 轨迹"，但 Muchacho 用 QP 而不是 NLP，方法结构对实时性的影响 / 对 Scout 的可迁移性边界。

## 1. 论文定位与场景

```text
机器人:   7-DoF Franka Emika Panda（torque controlled）
载具:     EE 上夹持一个立方容器（0.1m 立方，10g 装载）
液体:     水 / 红酒（实验）
执行模式: prehensile（容器夹在 EE 上，可任意倾斜）
任务:     极端时间最优 slosh-free 输送
```

灵感来源：Spillnot 机械装置（市售防洒杯托）。其核心是把杯子用钩子悬挂成被动单摆，杯子在外力作用下自然摆动，使液面始终与重力对齐——本质上是物理实现的"被动 slosh-free 跟随"。本文把这个机制**反过来用作机器人 EE 的运动设计模型**：把 EE 视为悬挂点（pivot），把容器+液体视为虚拟摆下端的点质量，主动控制 pivot 让虚拟摆自然倾斜，即可使容器始终保持 slosh-free。

## 2. 核心思想：slosh-free condition

来自 [Ref-Dang2004]，文中作为模型构建出发点：

```text
(a_z + g) tan θ = a_x                                     式(1)

含义:
  当容器 vertical axis 与"重力 + EE 反作用力"合力线方向对齐时，
  液面相对容器内壁不产生侧向偏移，slosh-free 成立。
  其中 θ 是容器相对竖直方向的倾角，
       a_x 是容器质心的水平加速度（外力分量），
       a_z 是垂直加速度。
```

这条几何关系给出了"容器要怎么倾才能不洒"的几何约束。Muchacho 把这个约束**装进运动学模型**：用球面摆描述容器（pendulum mass）相对 pivot（EE）的运动，pendulum 的自然摆动恰好满足式(1)。

**与 MSD 模型的本质区别**：

```text
MSD (Ferrari / Scout 当前用):
  描述液体内部 modal mass 的振动响应。
  状态量 = 液体相对容器的位移 (x_n, y_n)。
  目标 = 限制 |x| + |y|（即 sloshing height η̄）。

Muchacho:
  描述容器自身（含液体当作刚体点质量）相对 EE pivot 的摆动。
  状态量 = 容器倾角 (θ, φ)。
  目标 = 让 EE 路径与重力合力线匹配，从而维持式(1) → 液面不动。
```

两者根本差别：MSD 把"晃"当作要被压制的振动，Muchacho 把"晃"当作要被主动跟随的摆动。前者要求容器相对世界保持稳定，后者允许容器相对世界倾斜。**Muchacho 的方法只在 prehensile + 可主动倾斜 的硬件上成立**。

## 3. 球面摆参数化与运动方程

### 3.1 参数化（§III-B）

球面摆没有全局非奇异最小坐标。论文采用**两个正交平面摆角**（θ 绕 ŷ，φ 绕 x̂），围绕稳定平衡点（底部）非奇异：

```text
x_m = [ x − l sin θ                     ]
      [ y + l cos θ sin φ                ]
      [ z_p − l cos θ cos φ              ]                   式(2)
```

l = pendulum rod length，h = container 高度，x_p = pivot 位置。

### 3.2 EOM（§III-C）

Euler-Lagrange 推导，杆无质量、无摩擦：

```text
l θ̈ = −sin θ (g + z̈_p) cos φ + ẍ_p cos θ + ÿ_p sin φ sin θ
      − l cos θ sin θ φ̇²                                   式(4)
l cos θ φ̈ = −sin φ (g + z̈_p) − ÿ_p cos φ + 2 l θ̇ φ̇ sin θ   式(5)
```

文章证明这条非线性 EOM 自动满足 §III-A 的 slosh-free 条件——这是论文方法**理论上**正确的核心。

### 3.3 围绕稳定平衡的一阶线性化

围绕 θ = φ = θ̇ = φ̇ = ẋ_p = ẏ_p = ż_p = 0 做泰勒展开，得到两条解耦的线性二阶方程：

```text
l θ̈ = −g θ + u_1,        u_1 = ẍ_p                           式(11)
l φ̈ = −g φ − u_2,        u_2 = ÿ_p                           式(12)
```

观察：

```text
1. z̈_p（垂直加速度）在一阶展开里被消掉了 ——
   论文 §VI 的限制段明确写到这是模型损失：高 a_z 工况补偿不准。

2. 两个方向解耦，等同于两个独立平面摆，可叠加。

3. 摆的固有频率 ω_n = sqrt(g/l) 完全由 l 决定，
   与液体物性无关 —— 这正是 Muchacho 把"摆"当作几何抽象、
   不再依赖液体参数的根源。
```

## 4. 点质量近似有效性（§III-D）

这一节是论文最值得借鉴的方法学贡献，也是 Muchacho 在工程上凌驾于纯 MSD 的地方。

对一个真实容器，转动惯量除了 m·l²（围绕 pivot），还包括 J_c（围绕容器 CG）。点质量假设忽略 J_c，引入近似误差 p：

```text
(1 + p) l² m = l² m + J_c                                   式(6)
p l² m       = J_c                                          式(7)
```

对边长 h 的立方体（J_c = m h² / 6 是上界）：

```text
l = h / sqrt(6 p)         或          r = l / h = 1 / sqrt(6 p)   式(8)
```

经验值：

```text
r = l / h = 3     →   p < 1.8%
r = l / h = 6     →   p < 0.46%
r = l / h = 9     →   p < 0.21%
```

**含义**：l（虚拟摆长）越长，点质量近似越准；但 l 越长，达到同一倾角所需 EE 加速度越小 → 轨迹越保守。这是模型保真度 vs 轨迹激进度的显式取舍参数。

## 5. QP 形式（§IV）

Muchacho 把 §III 线性模型直接装进 quadratic program。这是和 Ferrari NLP 的关键差异。

### 5.1 状态、输入、离散化

```text
连续状态: x = [x_p, y_p, z_p, θ, φ, u_1, u_2, u_3, θ̇, φ̇]   维度 10
输入:     u = [ẍ_p, ÿ_p, z̈_p]                                维度 3
输出:     y = [x_m^T, ẋ_m^T]                                  维度 6（含一阶近似 (x − lθ) 等）
```

零阶保持 ZOH 在 T_s 离散化得到 (A, B, C, D)。

### 5.2 优化变量与约束

```text
χ = [x_0^T, ..., x_N^T, u_0^T, ..., u_{N-1}^T]^T
```

目标：让 mass 实际位置 y_k 跟随期望 y_{d,k}：

```text
J = (1/2) Σ ||C x_k − y_{d,k}||²                            式(15)
```

H = block-diag( {C^T C}_{i=0..N}, 0_{3N×3N} )；展开后是
χ 的二次型 (1/2) χ^T H χ − g^T χ。

约束（全是线性）：

```text
等式: A x_k + B u_k − x_{k+1} = 0,   k = 0..N−1            式(23)/(29)
不等式 (状态/输入 box):
  χ ≤ ub_χ,    −χ ≤ lb_χ                                    式(24)/(30,31)
不等式 (jerk box, 通过有限差分定义 pivot jerk):
  (1/T_s)(u_{k+1} − u_k) ≤ ub_u                              式(25)/(32)
  −(1/T_s)(u_{k+1} − u_k) ≤ lb_u                             式(26)/(33)
可选等式: 端点 mass 速度位置精确匹配                         式(27)/(34)
```

整体写成：

```text
χ* = argmin (1/2) χ^T H χ − g^T χ
     subject to {dynamics, box, jerk, optional endpoint}     式(28)
```

### 5.3 求解器与时间复杂度

论文未直接报告解 QP 时间，但反复强调：

```text
QP "可以在机器人 1 kHz inner control loop 里实时求解"。
对照 Ferrari 的 IPOPT NLP（85–471 s offline），
这是数量级差异。
```

QP 之所以快：所有约束线性，所有动力学线性，目标二次。即便节点数 N 较大，OSQP / qpOASES 等都能在毫秒级完成。

### 5.4 关节空间映射

QP 输出的是 pendulum state（即虚拟轨迹），不是机器人关节。落到 Panda 上的链路：

```text
1. 由 pendulum state 重建 mass pose 的 dual quaternion；
2. 用 damped pseudo-inverse Jacobian J^† 求关节速度 q̇;
3. 积分得到 q(t)；
4. 有限差分得到 q̈, q⃛；
5. 反推关节扭矩 + 校验机器人动力学限位。
```

这是 Muchacho 区别于 Scout 的另一关键点：**他们的"轨迹"是 EE 7-DoF 关节级，不是 base 平面级**。

## 6. 实物实验

```text
机器人:   Franka Emika Panda 7-DoF
容器:     0.1 m 立方，10 g
T_s:      33 ms
轨迹 1:   step 0.3m in x，r ∈ {3, 6, 9}
轨迹 2:   demonstrated square-like path（人示教）
最大速度:  0.63 m/s（轨迹 2），0.48 m/s（轨迹 1）
最大加速度: 1.22 m/s²
```

slosh-free 验证（Table I）：

```text
r       Force Alignment Error      Kinematic Error
3       2.51e-2                    7.66e-4
6       1.13e-2                    4.54e-4
9       0.75e-2                    3.15e-4
```

```text
含义:
  Force Alignment Error = 实测侧向力 / 总外力的 worst-case 比例。
  即"合力是否真的与 z 轴对齐"的实测误差。
  较 r 越大 → 模型保真度越高 → 误差越小 → 越严格 slosh-free。

  Kinematic Error = (a_z + g) tan θ − a_x 的 worst case，即式(1) 的误差。
```

红酒杯实物：optimized 轨迹无溢出；简单 P 控制器作为对照直接洒。

## 7. 与 Scout 系统的对应关系

Muchacho 是 prehensile + tiltable + 7-DoF 机械臂；Scout 是 non-prehensile + non-tiltable + 差速底盘。三者交叉点很少，必须逐项分析。

| 论文设定 | Scout 设定 | 是否可迁移 |
|---|---|---|
| EE 主动倾斜容器 | 容器固定在托盘上，无主动倾斜自由度 | **否** —— 这是根本差异 |
| Spillnot 物理类比（pendulum 跟随重力） | 杯子静止放置，无悬挂结构 | 否 |
| slosh-free condition (a_z+g) tan θ = a_x | θ 强制 = 0 → 退化为 a_x = 0 | 不可作为约束 |
| 7-DoF 关节空间映射 | 差速底盘 (v, ω) | 否 |
| QP + 1kHz 实时求解 | 在线导航循环 ~50 Hz | **结构可借鉴**（OSQP 我们也用） |
| 点质量近似 + r = l/h 准则 | 容器是开口杯，liquid CG 与 rigid-body CG 不同 | 部分启发 |
| 一阶线性化 + 解耦平面摆 | 我们的 MSD 也是线性二阶 + xy 解耦 | 概念同源 |

最关键的事实是：**式(1) 的 slosh-free 几何条件在 Scout 上是无法满足的硬性事实**。Scout 没有任何机制把容器倾过去。这意味着任何加速度 a_x ≠ 0 都会引起液面相对容器的偏移，无法通过倾斜抵消。

由此推导：**对 non-prehensile 平台，slosh-free 不是"几何条件能否满足"的问题，而是"激励能压到多低"的问题**。这正好回到 MSD / 模态阻尼框架，Muchacho 的方法学不能直接搬过来。

## 8. 可借鉴 / 不可迁移 / 反向启发

### 8.1 可借鉴

```text
1. 一阶线性化 + 解耦平面 + ZOH 离散 + QP 的实现链路
   是任何"在线 slosh-aware 优化"的标准模板。
   即便我们走 candidate scoring 路线，
   未来若要回到"在线优化 v_profile"也应当套这套结构而不是 NLP。

2. 点质量近似 r = l/h 阈值（p < 1.8% @ r=3）的方法学
   可以反向用到 MSD 的"第一模态足够"判据上：
   给我们的 ξ_{1n}=1.841 一阶模态权重 m_1 / m_F 一个量化容差，
   作为 docs/重要文档/Slosh Dynamics论文中建模总结.md 的补充。

3. 端点等式约束（式27）配合 box + jerk box 的写法
   是写 QP 时"必到点 + 平滑"的标准句式，
   可以用在我们 candidate rollout 起终点 v_init / v_end 的强约束设定上。
```

### 8.2 不可直接迁移

```text
1. slosh-free condition (a_z+g) tan θ = a_x
   非 prehensile + 无主动 tilt 的 Scout 上根本不成立。

2. 球面摆模型替换 MSD
   Muchacho 模型只描述容器整体倾斜，不描述液体相对容器的振动；
   对 Scout 这种"容器不能倾、必须看液体相对运动"的场景信息量不够。

3. Spillnot 物理装置
   Scout 不安装挂钩；如果安装，问题就退化为 Muchacho 的硬件版本，
   失去 Scout 课题"non-prehensile 移动底盘"的差异化贡献。

4. 7-DoF 关节空间反解 + dual quaternion
   差速底盘没有 6D 末端工作空间。
```

### 8.3 反向启发

```text
1. Muchacho 方法的"硬件假设"决定了它的成功域。
   论文 §VI 明确列出局限：高 a_z 不准、yaw 固定。
   这给我们的 limitation 段一个对照样本：
   prehensile 方法 fail 在 a_z + yaw；
   non-prehensile 方法 fail 在哪？
   答：fail 在 lateral acceleration 本身——必须把 ax 压低到模态噪声以下。
   这强化了 Online GeoRef 的理论合理性：
     "non-prehensile 平台的 slosh-free 等价于 ax 极小化，
      路径几何是 ax 的主导因子，因此 path-first 是合适抽象层。"

2. Muchacho 是"过摆"思路（actively tilt），
   Scout 是"不摆"思路（must hold container vertical）。
   论文写法可以对仗:
   Muchacho:   "make the container follow gravity vector"
   Ours:        "minimize lateral excitation since the container cannot follow"
   一句话就把 contribution 立住了。

3. Muchacho 的 r = l/h 取舍（保真 vs 激进）
   反映"模型简化等级"是一个可调旋钮。
   Scout 当前没有这个旋钮：
   slosh model 被默认为 ω_n=31.25, ζ=0.05 一组固定参数。
   实物阶段如果模型精度不够，可参考这种"一个参数控制保真度"的工程思路，
   引入 candidate scoring 的"模型保真档位"——
   high fidelity (r 大): 保守地 reject candidate
   low fidelity (r 小): 容忍更多 candidate
   作为离线 ablation 工具。
```

## 9. 对当前 slosh-model-guided GeoRef 方案的影响

针对 `docs/Claude/修改方案-时间-简介/2026-05-07_Slosh模型引导GeoRef候选评分方案.md`，本剖析建议的修订点（与 Ferrari 剖析的 M-A..M-E 不重复，编号 N-A..N-D）：

```text
N-A. §11 论文定位与 Muchacho / Ferrari 的对照写法:
  在论文 introduction / related work 段建议加入显式对仗:
    Muchacho 2022:  prehensile + active tilt  → slosh-free via pendulum follow
    Ferrari 2026:   prehensile + offline NLP → slosh-bounded via η_lim
    Ours:            non-prehensile + online selection → low-excitation reference
  这能把课题差异化讲清楚，而不是只说"我们方法更轻"。

N-B. §3 第一版不改 MPC 的依据强化:
  借 Muchacho §III-A 的 slosh-free condition 显式说明:
    Scout 缺主动 tilt → (a_z+g) tan θ = a_x 中 θ ≡ 0 →
    任意 a_x ≠ 0 必然产生模态激励 →
    只能在路径生成层把 a_x 压低，控制器层不可能实现 "几何 slosh-free"。
  这条物理推导能彻底切断"未来要不要做 slosh-aware MPC"的反复争论。

N-C. §6 Step 0 验收新增模型一致性指标:
  借 Muchacho 式(1) 的思路定义 Scout 上的"slosh-quality proxy":
    proxy_align(t) = | a_x(t) | / (g + a_z(t))
  对每条 candidate rollout 计算:
    proxy_align_p95 = p95 over t of proxy_align(t)
  含义: 假设 Scout 有 Spillnot 那样的虚拟 tilt，
        所需 tilt 角度的统计上界。
  虽然 Scout 实际不 tilt，但这个量直接反映"路径的 slosh-friendliness"，
  与 path_terminal_eta_norm / h_p95_pred 互补，作为 score 一项软分量。

N-D. §10 不要做的事 增加一条:
  显式禁止"复活 slosh-aware MPC（参考 Muchacho QP 形式）"。
  原因: Muchacho QP 形式之所以工作，
        前提是 EE 能任意 tilt + 7-DoF 关节冗余空间，
        Scout 两者都没有，搬过来必然 degenerate 成
        "约束 ax + 限制 a 上界"，正是 GEOREF_CONSTRAINED 已 FAIL 的方向。
        因此 Muchacho QP 的优雅形式是 prehensile-only 优雅，
        non-prehensile 复用没有同构对照物，禁止再走该方向。
```

## 10. 一句话总结

```text
Muchacho 2022 是 "prehensile + active-tilt + 实时 QP" 框架下做 slosh-free；
其 slosh-free 几何条件在 non-prehensile 移动底盘上恒不成立。
方法不能直接迁移；
但 (i) QP 实时结构、(ii) 点质量近似的 r = l/h 准则、(iii) 一阶线性化解耦的写法、
(iv) prehensile 方法 fail 模式与 non-prehensile 形成互补的论文论证逻辑
都是值得在 docs/Claude/修改方案-... slosh评分方案 与论文 limitation 段
反向利用的素材。
```
