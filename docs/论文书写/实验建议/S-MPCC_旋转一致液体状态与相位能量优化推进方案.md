# S-MPCC 旋转一致液体状态与相位能量优化推进方案

> 文档状态：方法升级与 novelty audit 工作稿
>
> 检索日期：2026-07-22
>
> 适用阶段：后续独立方法版本的开发评估；不适用于当前已冻结的 88 次正式实验协议
>
> 当前决策（2026-07-25）：本轮不修改液体动力学、代价或控制代码，先按当前 release 完成 88 次正式实验。本文所述 rotation/power 改动不构成本轮采集的前置条件，也不得与本轮数据混合。
>
> 目标：判断二维液体状态的旋转一致性是否已有先例，明确 S-MPCC 可保留的创新边界，并给出从公式、代码、回放、仿真到实物 pilot 的推进门槛。

## 1. 执行结论

本次检索得到四个直接结论。

1. **液体状态的旋转一致性不是空白，也不能作为“首次提出”的贡献。**
   - Hamaguchi 和 Taniguchi 已在轮式移动机器人曲线路径运输中采用球摆型晃液模型，非线性方程显式包含机器人转弯、向心激励以及纵横模态耦合。
   - Di Leva 等 2026 年进一步针对三维平移叠加固定轴旋转，推导了低阶 MSD/PEN 模型；其 SCARA 方程显式包含旋转矩阵、Coriolis、centrifugal 和 Euler-acceleration 项。
   - 更高保真度的浅水方程和机器人球摆/几何方法也早已处理完整刚体姿态或容器倾转。

2. **当前 S-MPCC 的论文公式和主线代码没有完整实现这种旋转一致性。**
   - 液体状态被定义在车体纵向/横向轴上，但当前 OCP 中液体微分方程只使用 `a_x=a`、`a_y=v*omega`。
   - 运行时 `SloshDynamics::step(...)` 接收 `omega_z`，但当前实现实际忽略该变量。
   - 公共 `LiquidSloshModel` 中所谓“旋转修正”只处理容器安装偏心导致的 `alpha*r` 和 `omega^2*r` 点加速度，不等于对残余液体模态方向进行旋转一致传播。

3. **值得推进的技术主线不是重新发明旋转模型，而是：**

   > 采用有文献依据的旋转一致二维模态动力学，将其作为跨控制周期保留的液体动态记忆嵌入在线 path-progress MPCC，并进一步构造包含平移、Euler 和 centrifugal 作用的有符号模态能量注入代价。

4. **在未来独立方法版本立项时应执行 stop/go，但不打断当前 88 次实验。**
   - 先做旋转一致模型的独立消融，不要同时修改模型、代价、horizon 和执行层。
   - 如果现有工况下新增旋转项对 replay、预测动作和真实液面均无可辨识影响，则不应为了公式新颖强行纳入主方法。
   - 如果旋转模型有效，再单独增加有符号能量注入项，形成第二层消融。

因此，推荐顺序为：

```text
坐标/方程审计
→ rotation-only 模型实现
→ 离线 replay 与数值门槛
→ 仿真和实物 pilot
→ power-cost 独立增量
→ 冻结新的方法 release
→ 为新 release 另立正式实验协议
```

---

## 2. 当前 S-MPCC 实现审计

本次审计以 Git `HEAD=d0dcea2`（`docs: format S-MPCC paper organization guide`）及 2026-07-22 工作区为基线。近期提交已经冻结三方法、随机区组和方法主线；当前 `spmpc_paper_core/sections/01_method.tex` 的未提交 diff 只把架构 TikZ 图替换为外部 PDF，没有改变液体动力学、状态维数或代价公式。因此，下述公式与代码差距不是由该排版 diff 造成的。

### 2.1 论文中的当前定义

当前论文将二维模态状态定义为

\[
\boldsymbol x_\ell=
[\eta_x,\dot\eta_x,\eta_y,\dot\eta_y]^\top,
\]

其中 \(x_b\) 为车体前向，\(y_b\) 为车体横向，激励为

\[
\boldsymbol a_b=
\begin{bmatrix}
a\\
v\omega
\end{bmatrix}.
\]

液体方程采用两个相互解耦的二阶振子：

\[
\ddot\eta_i+2\zeta\omega_1\dot\eta_i+\omega_1^2\eta_i=-a_i,
\qquad i\in\{x,y\}.
\]

对应位置：

- `docs/论文书写/草稿/spmpc_paper_core/sections/01_method.tex`，当前核心稿的液体模型、增广动力学和液体代价；
- `docs/论文书写/草稿/spmpc_paper/sections/03_method.tex`，液体状态与激励定义；
- 两份方法稿中的 `eq:liquid_dynamics` 和 `eq:augmented_dynamics`；
- 当前液体代价为模态位移与速度的标准二次惩罚。

### 2.2 主线代码中的当前行为

| 位置 | 当前行为 | 对旋转一致性的含义 |
|---|---|---|
| `scripts/acados/spmpc_acados_model.py` | `a_x=a`、`a_y=v*omega`，液体微分方程不含 `theta`、`alpha*eta`、`omega*eta_dot` 或 `omega^2*eta` | OCP 预测中没有显式的模态坐标旋转输运 |
| `src/dynamics/slosh_dynamics.cpp` | `step(..., omega_z)` 中 `omega_z` 被注释为未使用，状态按固定 LTI `Ad/Bd` 更新 | 在线 observer 没有传播 yaw 对已有残余模态方向的影响 |
| `slosh_models/liquid_slosh_model.cpp` | 使用 `alpha*r` 和 `omega^2*r` 修正偏心容器处的点加速度 | 这是安装偏心修正，不是模态状态坐标系旋转 |
| `execution_state_predictor.cpp` | delay replay 使用 `ax=(v-v_prev)/dt`、`ay=v*omega`，没有 `alpha` 进入液体方程 | 延迟补偿分支与当前简化模型一致，但同样缺少旋转耦合 |
| `spmpc_acados_cost.py` | 仅惩罚 `eta^2` 和 `eta_dot^2` | 能看到液体状态大小，但没有区分候选动作是在注入还是抽取模态能量 |

### 2.3 必须区分的两个“旋转效应”

#### A. 容器安装偏心的点加速度

若容器相对底盘旋转中心的车体系偏移为 \(\boldsymbol r_b\)，其中心加速度为

\[
\boldsymbol a_{c,b}
=
\begin{bmatrix}a\\v\omega\end{bmatrix}
+\alpha\boldsymbol J\boldsymbol r_b
-\omega^2\boldsymbol r_b,
\qquad
\boldsymbol J=
\begin{bmatrix}0&-1\\1&0\end{bmatrix}.
\]

当前公共液体模型已经包含这一类修正。核心实验令 \(\boldsymbol r_b=0\)，所以这些项消失。

#### B. 已有液体模态状态在旋转车体系中的传播

即使 \(\boldsymbol r_b=0\)，只要底盘存在 yaw rate，已有的 \(\boldsymbol\eta_b\) 和 \(\dot{\boldsymbol\eta}_b\) 仍处在随底盘旋转的坐标系中。其动力学一般会出现 Coriolis、centrifugal 和 Euler 项。

当前实现缺少的是 B，而不是 A。

---

## 3. 文献检索方案

### 3.1 检索问题

本次 novelty audit 回答三个问题：

1. 是否已有液体低阶状态在旋转容器坐标系中的一致动力学？
2. 是否已有轮式移动机器人在转弯时使用这种液体状态？
3. 是否已有工作把旋转一致、跨周期保留的液体状态嵌入在线 path-progress MPCC，并使用有符号能量注入进行动作选择？

### 3.2 数据源

- Crossref：DOI、作者、题名、卷期页和出版社核验；
- OpenAlex：题名检索、摘要、开放全文位置和引用元数据；
- Semantic Scholar：移动机器人最近邻摘要和开放版本核查；
- Springer、arXiv、Shimane University Repository：公开全文；
- IEEE Xplore/Elsevier：公开元数据与摘要；无法访问全文的文献单独标记。

### 3.3 主要检索式

```text
liquid sloshing rotating coordinate frame
sloshing non-inertial frame translating rotating tank
two dimensional slosh model yaw rotation container
mobile robot liquid slosh turning dynamics
spherical pendulum slosh robot manipulation SE(3)
sloshing tank arbitrary translation rotation low order model
slosh passivity control energy
slosh energy model predictive control
phase aware slosh suppression mobile robot
```

### 3.4 纳入与排除原则

纳入：

- 明确包含平移与旋转耦合的液体动力学；
- 移动机器人、机械臂、SCARA 或 spacecraft 中的低阶晃液模型；
- 与在线规划、MPC、轨迹优化、能量/passivity 控制相关的工作；
- 能帮助判断 S-MPCC novelty boundary 的基础模型论文。

排除：

- 只研究固定容器自然晃动且没有运动输入；
- 只讨论 CFD 网格技巧、结构强度或海洋载荷而不涉及控制/状态建模；
- 只与液体传感、倾倒或液位检测相关而没有运输动力学。

### 3.5 检索限制

- 这是有针对性的 novelty audit，不是 PRISMA 系统综述。
- 本次无法阅读全文核查 Lim 2024 和 Chen 2024 的全部方程，因此对这两篇论文只使用可核验摘要/元数据，不推断其未公开细节。Ferrari 2026 已通过仓库内全文核查。
- 没有订阅式 Scopus/Web of Science 全库检索，不能据此作绝对“first”声明。
- “未发现”只表示在本次可核实公开全文与摘要范围内没有发现。

---

## 4. 相关论文与旋转一致性证据

### 4.1 文献矩阵

| 工作 | 平台/运动 | 旋转处理 | 在线性 | 与 S-MPCC 的关系 | 核验状态 |
|---|---|---|---|---|---|
| Hamaguchi & Taniguchi, 2005 | 轮式移动机器人，直线与曲线路径 | 球摆型非线性模型显式包含转弯角速度、向心激励和纵横模态耦合 | 离线路径/速度设计 + 跟踪 | 直接证明移动机器人转弯晃液和旋转耦合不是新问题；但不是每周期 path-progress MPCC，也不保留在线液体估计用于重规划 | 公开全文核验 |
| Ardakani & Bridges, 2011 | 任意刚体运动容器 | 浅水方程精确建模 roll/pitch/yaw 与 surge/sway/heave | 否，高保真数值模型 | 建立完整刚体旋转晃液的理论先例，但计算层和模型层与 S-MPCC 不同 | 摘要与 DOI 核验 |
| Reyhanoglu & Rubio Hervas, 2012 | 航天器平面运动、多燃料晃动模态 | 文献范围为二维平移与一维姿态旋转耦合；本文未据摘要反推具体坐标项 | 非线性闭环控制 | 说明“旋转平台 + 晃液状态”的耦合在航天控制中早有先例，但不是移动底盘 path-progress 规划 | DOI/元数据核验；旋转范围由 Di Leva 2026 的综述性描述交叉确认，全文方程未核实 |
| Di Leva et al., 2022 | 圆柱容器二维平移及增加竖直加速度 | 仅空间平移；其“3D”指 \(x,y,z\) 平移，不含姿态旋转 | 估计/离线优化基础 | 与当前两个解耦 MSD 模态最接近，也清楚划定了 translation-only 模型边界 | 公开全文核验 |
| Muchacho et al., 2022 | 机械臂、球摆、容器倾转 | 通过三维球摆和 end-effector orientation 耦合平移与倾转；论文明确说明实验中 yaw 固定 | 离线/快速 QP 轨迹优化 | 有姿态一致性，但目标是 tilt compensation，不是移动底盘跨周期液体状态记忆 | arXiv 全文核验 |
| Arrizabalaga et al., 2024 | 机械臂、SE(3) 实时跟踪 | 用旋转矩阵和虚拟四旋翼生成与合加速度对齐的容器姿态 | 在线实时跟踪 | 姿态处理覆盖 SO(3)，但不传播液体动态状态，也不是 path-progress MPCC | arXiv 全文核验 |
| Lim et al., 2024 | 轮式移动机器人、二维轨迹优化 | 摘要确认使用 spherical-pendulum dynamic constraints，并联合优化速度与角速度；具体坐标方程未核实 | 整段轨迹优化、仿真 | 是最接近的移动底盘旋转/球摆近邻之一，不能声称首次在移动机器人转弯中考虑晃液 | 摘要与 DOI 核验，全文未核实 |
| Chen & Lian, 2024 | 履带底盘 + 6-DoF 机械臂 | spherical pendulum MPC；完整旋转方程未核实 | MPC fast replanning、仿真 | 削弱“首次 MPC/在线重规划”的宽泛主张；决策层仍不同于标准底盘 path-progress MPCC | 摘要与 DOI 核验，全文未核实 |
| Di Leva et al., 2026 | 三维平移 + 固定方向单轴旋转，含 SCARA yaw | 位置通过 \(R_z(\theta_c)\) 变换；EOM 显式含 \(2\dot\theta_c\dot\eta\)、\(\dot\theta_c^2\eta\)、\(\ddot\theta_c J\eta\) | 模型估计，非在线规划 | 当前旋转一致低阶模型最直接的先例，应作为方法升级的主要理论来源 | Springer 全文核验，Sec. 6.1、Eqs. 54–57 |
| Ferrari et al., 2026 | 多容器 SCARA 运动 | 通过 \(R_z(\theta)\) 和 EOM 显式包含 Coriolis、centrifugal、Euler 及偏心容器点加速度，并约束预测液面高度 | 离线 time-optimal planning，实物验证 | 说明旋转感知低阶模型已经进入 RA-L 级轨迹优化；但不是在线底盘局部规划 | 仓库内全文、DOI 和出版元数据核验，关键为 Eqs. 6–13、19–24 |
| Gogte et al., 2013 | 横向晃液控制 | passivity-based control | 闭环控制 | 说明 energy/passivity 本身不是新概念；S-MPCC 不能声称首次能量型防晃 | DOI/元数据核验，全文未核实 |

航天器文献在平台、激励和控制目标上与本文差异很大，但它进一步收紧了基础动力学的 novelty boundary：不能把“旋转刚体与内部晃液状态耦合”本身写成新概念。本文档不依赖该论文的未核实方程；具体 Coriolis、centrifugal 和 Euler 符号仍以已阅读全文的 Di Leva et al. 2026 为直接依据。

### 4.2 最关键的直接先例：Di Leva et al. 2026

该文考虑容器底部中心 \(O_c\) 的三维平移与绕竖直轴旋转。第 \(n\) 个晃液质量在惯性坐标系中的位置写成

\[
{}^0\boldsymbol s_n
=
\boldsymbol S_c
+R_z(\theta_c)
\begin{bmatrix}
x_n\\y_n\\h/2+h_n+f(x_n,y_n)
\end{bmatrix}.
\]

将该位置和速度代入 Lagrange 方程后，SCARA 模型出现：

- \(2\dot\theta_c\dot y_n\)、\(-2\dot\theta_c\dot x_n\)：Coriolis 耦合；
- \(\dot\theta_c^2x_n\)、\(\dot\theta_c^2y_n\)：centrifugal 项；
- \(\ddot\theta_c y_n\)、\(-\ddot\theta_c x_n\)：Euler/angular-acceleration 项；
- 世界坐标平移加速度经 \(R_z^\top(\theta_c)\) 投影到容器轴。

这与当前 S-MPCC 的两个独立车体系二阶振子有实质差异。

### 4.3 轮式移动机器人先例：Hamaguchi & Taniguchi 2005

该文已经针对 WMR 曲线运输构造球摆型晃液模型。其完整非线性方程包含：

- \(v/r\) 或 yaw-rate 相关项；
- \(v^2/r\) 向心激励；
- 两个摆角及其速度的交叉耦合；
- 曲线路径、速度和加速图形的输入整形设计。

因此，新的论文表述不能写成：

> Existing mobile-robot slosh models ignore turning or modal-direction coupling.

更准确的差异是：

> 既有 WMR 方法主要预先设计路径与速度图形；S-MPCC 希望把旋转一致的液体动态记忆放入每周期在线 path-progress OCP，使当前残余相位改变下一段局部动作。

### 4.4 机器人姿态补偿不等同于液体状态传播

Muchacho 2022 和 Arrizabalaga 2024 都对容器姿态进行了几何一致处理，但它们的核心目标是调节容器倾角，使液面法向与合加速度一致。二者并不等价于：

- 从 odometry 持续传播 \((\eta_x,\dot\eta_x,\eta_y,\dot\eta_y)\)；
- 在 successive MPC cycles 之间保留残余液体相位；
- 在固定水平容器、标准轮式底盘上通过速度/角速度分配消晃。

这仍为 S-MPCC 留下了清晰的决策层差异。

---

## 5. 更新后的创新边界

### 5.1 不能再主张的内容

- 首次建立旋转容器中的二维液体模型；
- 首次在轮式移动机器人转弯中考虑液体晃动；
- 首次使用球摆或 MSD 模型处理二维/三维晃液；
- 首次使用 MPC 对移动机器人液体运输进行重规划；
- 首次使用 energy/passivity 思想抑制晃液；
- 首次在 SCARA motion 中加入液面约束。

### 5.2 仍可能成立、但必须保持保守的贡献

在本次可核实文献范围内，尚未发现以下完整组合：

1. 标准轮式移动底盘；
2. 给定几何路径上的在线 path-progress MPCC；
3. 跨控制周期保留、由实际执行运动传播的二维液体模态状态；
4. 模态状态在 yawing body frame 中旋转一致传播；
5. 路径进度、底盘加速度、角加速度和未来液体响应在同一 OCP 联合优化；
6. 利用 matched smooth、time-match、actual/zero replay 和实际视觉液面验证该机制。

可采用的保守表述为：

> Building on rotation-aware low-order slosh models, we integrate a carried, rotation-consistent liquid modal state into an online path-progress MPCC loop for a standard wheeled base. The current liquid phase is propagated across control cycles and jointly optimized with path progress and base motion.

若后续有符号能量注入项也通过验证，可以追加：

> A signed generalized-power term distinguishes candidate motions that inject energy into the current liquid mode from those that dissipate or avoid excitation.

不建议在没有更完整全文检索前写 “the first”。可使用：

> To the best of our knowledge, no prior experimentally validated study has demonstrated this complete online decision-layer combination.

---

## 6. 建议的旋转一致低阶动力学

### 6.1 变量定义

令

\[
\boldsymbol\eta_b=
\begin{bmatrix}\eta_x\\\eta_y\end{bmatrix},
\qquad
\boldsymbol\nu_b=
\left(\frac{\mathrm d\boldsymbol\eta_b}{\mathrm dt}\right)_b,
\qquad
\boldsymbol J=
\begin{bmatrix}0&-1\\1&0\end{bmatrix}.
\]

这里下标 \(b\) 和导数符号必须同时冻结：\(\boldsymbol\eta_b\) 是在随底盘 yaw 旋转的车体/容器轴中表达的模态位移，\(\boldsymbol\nu_b\) 是这些车体系分量的时间导数，而不是“惯性系速度向量旋转到车体系后的分量”。两种速度定义相差 \(\omega\boldsymbol J\boldsymbol\eta_b\)，混用会直接改变 Coriolis、Euler 和能量项的符号。

底盘中心处车体系平动加速度为

\[
\boldsymbol a_b=
\begin{bmatrix}a\\v\omega\end{bmatrix}.
\]

如果未来考虑偏心安装，则使用

\[
\boldsymbol a_{c,b}
=\boldsymbol a_b
+\alpha\boldsymbol J\boldsymbol r_b
-\omega^2\boldsymbol r_b.
\]

核心实验仍令 \(\boldsymbol r_b=0\)，以隔离模态坐标旋转效应。

### 6.2 旋转一致线性一阶模态候选式

由 Di Leva 等 2026 的 SCARA EOM 在小振幅、第一模态、水平圆柱容器条件下可整理为：

\[
\boxed{
\ddot{\boldsymbol\eta}_b
+\left(2\zeta\omega_1\boldsymbol I+2\omega\boldsymbol J\right)
\dot{\boldsymbol\eta}_b
+\left[
(\omega_1^2-\omega^2)\boldsymbol I
+\alpha\boldsymbol J
\right]\boldsymbol\eta_b
=-\boldsymbol K_a\boldsymbol a_{c,b}
}
\]

其中 \(\boldsymbol K_a=\operatorname{diag}(\kappa_x,\kappa_y)\)。若沿用当前论文的单位输入增益，则 \(\boldsymbol K_a=\boldsymbol I\)。

分量形式为

\[
\begin{aligned}
\ddot\eta_x={}&
-2\zeta\omega_1\dot\eta_x
+2\omega\dot\eta_y
-(\omega_1^2-\omega^2)\eta_x
+\alpha\eta_y
-\kappa_x a_{c,x},\\
\ddot\eta_y={}&
-2\zeta\omega_1\dot\eta_y
-2\omega\dot\eta_x
-(\omega_1^2-\omega^2)\eta_y
-\alpha\eta_x
-\kappa_y a_{c,y}.
\end{aligned}
\]

其重要性质是：

- 当 \(\omega=0\)、\(\alpha=0\) 时，严格退化为当前两个独立二阶振子；
- Coriolis 项 \(2\omega\boldsymbol J\dot{\boldsymbol\eta}\) 只改变模态速度方向，不直接做功；
- centrifugal 项改变有效刚度；
- Euler 项使角加速度与当前液体位移方向耦合；
- 状态维数仍为 4，增广 OCP 仍为 10 维。

### 6.3 坐标约定与适用性门槛

上述式子不是可以脱离状态定义直接粘贴的“通用修正项”。纳入代码前必须满足以下条件。

1. **成对简并模态。** 对水平圆柱容器，第一阶横向模态在 \(x/y\) 方向具有相同的 \(\omega_1\) 和近似相同的阻尼，因此可组成二维向量。若未来容器明显非轴对称，必须改用完整的二维刚度、阻尼和输入矩阵，不能继续假设两个方向等频。
2. **只选择一种状态坐标。** 若状态改为惯性系模态坐标，则应在惯性系传播并旋转输入/输出，不能再叠加同一组车体系虚拟力项。当前实现最小改动路线是保留车体系状态并采用第 6.2 节方程。
3. **yaw 与 \(\boldsymbol J\) 符号冻结。** 本文采用正 yaw 为逆时针、\(R_z(\theta)\) 从车体系映射到惯性系、\(\boldsymbol J\) 如上。若代码坐标约定不同，应整体变换，不能逐项凭经验改符号。

建议增加一个独立的世界系重构 oracle。令

\[
\boldsymbol\eta_I=R_z(\theta)\boldsymbol\eta_b,
\qquad
\dot{\boldsymbol\eta}_I
=R_z(\theta)
\left(\boldsymbol\nu_b+\omega\boldsymbol J\boldsymbol\eta_b\right).
\]

数值积分后的 \(\boldsymbol\eta_I\) 有限差分必须与右式一致；其二阶导数还应满足坐标运动学恒等式

\[
\ddot{\boldsymbol\eta}_I
=R_z(\theta)
\left(
\ddot{\boldsymbol\eta}_b
+2\omega\boldsymbol J\boldsymbol\nu_b
+\alpha\boldsymbol J\boldsymbol\eta_b
-\omega^2\boldsymbol\eta_b
\right).
\]

该测试不依赖液体参数，适合专门捕获符号、坐标和导数定义错误。

### 6.4 名义工况的量级筛查

对当前 \(C_1\)：

```text
R = 18.5 mm
h = 58 mm
omega_1 ≈ 31.25 rad/s ≈ 4.97 Hz
omega_max = 1.2 rad/s
alpha_max = 1.2 rad/s²
zeta = 0.05（当前配置）
```

无量纲比例约为

\[
\frac{\omega_{\max}}{\omega_1}\approx0.038,
\qquad
\frac{2\omega_{\max}}{\omega_1}\approx0.077,
\]

\[
\frac{\omega_{\max}^2}{\omega_1^2}\approx1.5\times10^{-3},
\qquad
\frac{\alpha_{\max}}{\omega_1^2}\approx1.2\times10^{-3}.
\]

表面上 centrifugal 和 Euler 刚度项较小，但 Coriolis 系数相对当前阻尼系数的最大比例为

\[
\frac{2\omega_{\max}}{2\zeta\omega_1}
=\frac{\omega_{\max}}{\zeta\omega_1}
\approx0.77.
\]

这不代表实际液面一定改善 77%，只说明在低阻尼模型中，yaw-induced velocity coupling 不能仅凭 \(\omega/\omega_1\) 小就直接忽略。是否对规划有实际意义必须由 replay 和物理 pilot 判断。

---

## 7. 有符号模态能量注入候选

### 7.1 为什么不能只把现有二次项换个名字

当前代价

\[
w_\eta\|\boldsymbol\eta\|^2
+w_{\dot\eta}\|\dot{\boldsymbol\eta}\|^2
\]

已经与模态能量相似。把它重写成 \(E\) 并不能形成新贡献。

真正新增的信息应当是：给定当前液体相位，一个候选底盘动作究竟在向模态注入能量，还是避免/抽取能量。

### 7.2 名义模态能量平衡

在单位模态质量归一化下，定义非旋转参考的名义一阶模态能量（严格说是 specific/modal energy）

\[
E_0
=\frac12\boldsymbol\nu_b^\top\boldsymbol\nu_b
+\frac12\omega_1^2
\boldsymbol\eta_b^\top\boldsymbol\eta_b.
\]

将第 6 节的旋转一致动力学代入，可得

\[
\dot E_0
=-2\zeta\omega_1\|\boldsymbol\nu_b\|^2
+P_{\mathrm{ext}},
\]

其中

\[
\boxed{
P_{\mathrm{ext}}
=
\omega^2\boldsymbol\nu_b^\top\boldsymbol\eta_b
-\alpha\boldsymbol\nu_b^\top\boldsymbol J\boldsymbol\eta_b
-\boldsymbol\nu_b^\top\boldsymbol K_a\boldsymbol a_{c,b}
}
\]

而 Coriolis 项满足

\[
\boldsymbol\nu_b^\top\boldsymbol J\boldsymbol\nu_b=0,
\]

因此不直接改变 \(E_0\)。

这里的 \(P_{\mathrm{ext}}\) 应称为“相对于名义模态能量定义的 generalized external power”，不要在尚未完成 Lagrange/数值交叉核验前称为真实液体总机械功率。

### 7.3 候选代价与可微实现

为了只识别 signed-power 的增量作用，主推荐版本应保留当前二次液体代价，只新增一项：

\[
J_{\mathrm{slosh},k}
=J_{\mathrm{quad,current},k}
+w_P
\left(
\frac{[P_{\mathrm{ext},k}]_{+,\epsilon}}{P_{\mathrm{ref}}}
\right)^2,
\]

其中可使用 SQP 更友好的平滑正部

\[
[z]_{+,\epsilon}
=\frac12\left(z+\sqrt{z^2+\epsilon_P^2}\right),
\qquad \epsilon_P>0,
\]

并在独立开发数据上冻结 \(\epsilon_P\)。若希望把现有两个权重改写成物理归一化的 energy-only 形式，则应作为单独的 R2 消融：

\[
J_{\mathrm{slosh},k}^{E+P}
=w_E\frac{E_{0,k}}{E_{\mathrm{ref}}}
+w_P
\left(
\frac{[P_{\mathrm{ext},k}]_{+,\epsilon}}{P_{\mathrm{ref}}}
\right)^2.
\]

不能在同一个“power ablation”里同时替换原二次权重又加入 \(P_{\mathrm{ext}}\)，否则收益来源不可识别。

为减少任意尺度，建议从当前 \(\eta_{\mathrm{ref}}\) 派生

\[
E_{\mathrm{ref}}
=\frac12\omega_1^2\eta_{\mathrm{ref}}^2,
\qquad
P_{\mathrm{ref}}=\omega_1E_{\mathrm{ref}},
\]

而不是为每个容器单独手调两个新的归一化常数。正功率项只加在 stage cost；terminal slosh cost 第一轮保持 current 定义，以免同时改变阶段与终端机制。

含义：

- current quadratic，或单独验证后的 \(E_0\) 版本，约束预测液体状态的总体幅度；
- \([P_{\mathrm{ext}}]_{+,\epsilon}\) 只惩罚增加名义模态能量的候选动作，并避免在零点使用不可微的 `max`；
- 不奖励无限大的负功率，避免优化器为“抽取能量”产生过激动作；
- 控制幅值、tracking、progress 和 actuator bounds 继续限制动作。

### 7.4 可选软能量包络

若 cost-only 仍存在强权重敏感性，可以测试

\[
E_{0,k}\leq E_{\max}+\epsilon_k,
\qquad
\epsilon_k\geq0,
\]

并惩罚 \(w_\epsilon\epsilon_k^2\)。

不建议直接将内部低阶模态高度硬约束宣传为 spill-free guarantee。软能量包络只代表内部模型风险预算。

### 7.5 新颖性限制

Passivity-based lateral slosh control 已有先例，因此不能声称首次使用能量/passivity。可争取的差异应限定为：

> 在旋转一致、跨周期传播的二维液体状态基础上，将包含平移和 yaw-induced terms 的有符号 generalized-power term 嵌入标准轮式底盘的在线 path-progress MPCC。

---

## 8. 实现推进计划

### 8.1 原则：先 rotation-only，后 power-cost

必须保留三个可独立复现的版本：

| 版本 | 旋转一致动力学 | 正功率代价 | 用途 |
|---|---:|---:|---|
| `S-MPCC-current` | 否 | 否 | 当前方法基准 |
| `S-MPCC-rot` | 是 | 否 | 证明旋转模型本身是否必要 |
| `S-MPCC-rot-power` | 是 | 是 | 在保留 current quadratic 的前提下，证明有符号动作选择是否带来额外收益 |

不要直接从 current 跳到 rot-power，否则无法判断收益来自模型还是代价。
`energy-only` 只作为 replay/权重归一化诊断，不自动升级为第四个正式物理方法。

### 8.2 代码改动清单

#### A. CasADi/acados 动力学

文件：

```text
src/scout_apps/control/spmpc_local_planner/scripts/acados/spmpc_acados_model.py
```

动作：

- 保持 10 维状态和 3 维控制不变；
- 在 \(\dot\eta_x,\dot\eta_y\) 方程中加入 `omega`、`alpha` 交叉项；
- 保留 `a_x=a`、`a_y=v*omega`；
- 如未来启用偏心容器，再单独加入 `r_x/r_y`，不要和本轮 rotation-only 混合；
- 重新生成 acados artifacts，并核对 Python/C++ 参数布局。

#### B. 运行时液体 observer

文件：

```text
src/scout_apps/control/spmpc_local_planner/include/spmpc_local_planner/dynamics/slosh_dynamics.h
src/scout_apps/control/spmpc_local_planner/src/dynamics/slosh_dynamics.cpp
src/scout_apps/control/spmpc_local_planner/src/ros/spmpc_local_planner_ros.cpp
```

动作：

- 将 `step(state, ax, ay, omega)` 扩展为至少 `step(state, ax, ay, omega, alpha)`；
- 对冻结的 \(\omega,\alpha\)，observer 子系统仍是线性的；但在增广 OCP 中它是输入依赖的双线性/非线性动力学，不能继续无条件复用固定 `Ad/Bd`；
- 推荐使用与 OCP 一致的 RK4/ERK 小步积分，或对每个采样冻结 \(\omega,\alpha\) 后离散化；
- 从连续 odometry 估计 \(\alpha=(\omega_q-\omega_{q-1})/\Delta t_q\)，采用冻结滤波器并记录 raw/filtered 值；
- 新旧 observer 并行 shadow 一段时间，不立即改变正式 solver 输入。

#### C. delay predictor 与 warm start

文件：

```text
src/scout_apps/control/spmpc_local_planner/src/ros/execution_state_predictor.cpp
src/scout_apps/control/spmpc_local_planner/src/warm_start/diff_drive_flatness_warm_start.cpp
```

动作：

- delay replay 中由相邻 command 样本计算 \(\alpha\)；
- warm-start rollout 使用预测 \(\omega_k\) 和 \(\alpha_k\)，不能继续调用旧 LTI step；
- actual branch 和 replay branch 必须使用同一模型版本，避免比较被模型实现差异污染。

#### D. 能量代价与诊断

文件：

```text
src/scout_apps/control/spmpc_local_planner/scripts/acados/spmpc_acados_cost.py
src/scout_apps/control/spmpc_local_planner/src/ros/diagnostics_publisher.cpp
src/scout_apps/control/spmpc_local_planner/include/spmpc_local_planner/core/types.h
```

新增日志至少包括：

```text
E0
P_translation = -nu^T K_a a_c
P_euler = -alpha * nu^T J eta
P_centrifugal = omega^2 * nu^T eta
P_ext
positive_power_cost
rotation_model_enabled
power_cost_enabled
alpha_raw
alpha_filtered
```

### 8.3 不应同时修改的内容

在 rotation-only 消融阶段冻结：

- 所有 tracking/progress/control/smooth 权重；
- 原有 \(w_\eta,w_{\dot\eta}\)；
- horizon、步长和 solver 选项；
- 路径、容器、参考速度和 actuator bounds；
- terminal、gate、rate limiter 和 fallback；
- RGB 处理、相机标定和 trial window。

---

## 9. 数学和软件验收门槛

### 9.1 单元测试

1. **零旋转退化测试**

   令 \(\omega=\alpha=0\)，新模型的连续导数和离散一步结果必须与 current 模型在数值容差内一致。

2. **零状态纯旋转测试**

   令 \(\boldsymbol\eta=\boldsymbol\nu=0\)、\(\boldsymbol a_c=0\)，任意 \(\omega,\alpha\) 下状态应保持零。

3. **分量式/矩阵式一致性测试**

   对随机有限状态比较 compact form 与 component form。

4. **车体系/世界系重构一致性测试**

   用第 6.3 节的 \(R_z(\theta)\) 关系比较 body-frame 积分、世界系有限差分速度和二阶运动学恒等式；分别覆盖恒定 yaw、变 yaw、正负 yaw 和正负 \(\alpha\)。

5. **能量平衡有限差分测试**

   验证

   \[
   \frac{E_0(t+\Delta t)-E_0(t)}{\Delta t}
   \approx
   -2\zeta\omega_1\|\nu\|^2+P_{\mathrm{ext}}.
   \]

6. **Python/C++ 一致性测试**

   相同状态、输入和参数下，CasADi continuous derivative、C++ observer derivative 与离散 rollout 必须一致。

7. **旧日志 replay 确定性测试**

   同一 snapshot 重复求解，模型版本和随机种子不变时第一动作应可重复。

### 9.2 数值准入

- acados codegen、编译和现有测试全部通过；
- SQP-RTI 状态和 residual 不出现系统性恶化；
- solve-time p95 仍低于控制周期预算；
- solver failure、min-step、fallback 和 deadline miss 不高于 current；
- 新增 \(\alpha\) 估计不存在明显噪声爆炸或符号错误；
- actual/zero replay 的 actual branch 能复现在线输出。

---

## 10. 离线 replay 与仿真方案

### 10.1 旧 bag 双模型 shadow replay

对已有高曲率路径 bag 同时运行：

```text
current observer
rotation-aware observer
```

保持实际执行输入完全相同，比较：

- \(\|\eta\|\)、\(\|\dot\eta\|\) 和模态相位；
- 高 yaw-rate/曲率反转区间的状态差；
- 预测 \(H_{\mathrm{modal}}\) 与 \(H_{\mathrm{vis}}\) 的时间对齐、相关性和峰值误差；
- current/rot 两个模型下的 optimized first action；
- \(P_{\mathrm{ext}}\) 正值积分

\[
I_+=\int[P_{\mathrm{ext}}(t)]_+\,dt.
\]

### 10.2 replay 分层

| Replay | Dynamics | Cost | 回答的问题 |
|---|---|---|---|
| R0 | current | current quadratic | 当前基准 |
| R1 | rotation-aware | current quadratic | 只改变动力学是否改变计划？ |
| R2 | rotation-aware | energy-only | 可选：物理归一化是否改善权重稳定性？ |
| R3 | rotation-aware | current quadratic + positive power | 主检验：只增加有符号相位项是否产生额外变化？ |
| R3E | rotation-aware | energy + positive power | 可选：仅在 R2 被接受后检查 energy 参数化与 power 的组合 |
| R4 | rotation-aware, zero liquid state | same as accepted power variant | 动态记忆是否是变化来源？ |

### 10.3 受控状态测试

在相同机器人、路径、容器和模态能量下构造：

- 纵向四相位；
- 横向四相位；
- yaw rate 取 \(-\omega_0,0,+\omega_0\)；
- angular acceleration 取 \(-\alpha_0,0,+\alpha_0\)。

记录：

- \(a_0^\star,\alpha_0^\star,v_{s,0}^\star\)；
- horizon 内 \(E_0\)、\(P_{\mathrm{ext}}\)、\([P_{\mathrm{ext}}]_+\)；
- 计划速度和角速度分配；
- cost breakdown。

### 10.4 旋转一致性专用机制场景

推荐构造一个不进入正式 88 次的机制 pilot：

```text
1. 沿原车体 x 方向激起可控残余晃动；
2. 在保持容器中心位置基本不变时改变底盘 yaw；
3. 随后沿新车体 x 方向施加相同平移动作；
4. 比较 current 与 rotation-aware 对残余晃动方向和下一动作的预测。
```

单纯原地 yaw 且液体初态为零并不能验证该问题；必须先产生非零、有方向的残余液体状态，再改变车体朝向。

---

## 11. 实物 pilot 与正式实验关系

### 11.1 rotation-only pilot

在正式实验开始前，先使用独立开发路径或正式高风险路径的非正式 pilot：

| 方法 | 建议重复 | 目的 |
|---|---:|---|
| `S-MPCC-current` | 3–5 | 当前工作点 |
| `S-MPCC-rot` | 3–5 | 旋转动力学增量 |

必须同时报告：

- 全窗口与 10%–90% progress 的 \(H_{\mathrm{vis}}\) p95/RMS；
- completion time、tracking p95 和 success；
- 高曲率区段的动作和液体状态差；
- solve-time p95、failure 和 command intervention；
- trial 原始点，不只报告均值。

### 11.2 power-cost pilot

只有 rotation-only 通过后，才比较：

| 方法 | 建议重复 | 目的 |
|---|---:|---|
| `S-MPCC-rot` | 3–5 | 旋转一致但无 signed-power |
| `S-MPCC-rot-power` | 3–5 | signed-power 增量 |

主分支只允许扫描 \(w_P\) 的小型预定义候选集；原有 \(w_\eta,w_{\dot\eta}\) 和非液体权重应冻结。只有进入可选 energy 分支时才引入并冻结 \(w_E\)。

### 11.3 与 88 次正式实验的关系

- 以上 trial 全部是 development/pilot，不进入正式样本。
- 当前决定是先用现有冻结 release 完成 \(24+48+16=88\) 次正式实验：\(C_1\) 低风险路径三方法 \(3\times8=24\)，高风险路径两容器三方法 \(2\times3\times8=48\)，以及 \(C_1\) 高风险路径 Smooth-match--S-MPCC \(2\times8=16\)。本文的 rotation/power pilot 在本轮之后另行评估，不计入也不改变这 88 次。
- 未来若 rotation/power 通过 stop/go，它们构成新的方法 release，必须另行冻结并建立独立证据包；不能回写本轮方法，也不能把两个 release 的数据混合成同一组正式实验。
- 不建议为了新增方法消融把正式方法扩成四个核心物理组；rotation/current 和 power/no-power 可以优先通过 replay、受控 phase study 和少量独立 pilot 证明。

---

## 12. Stop/Go 决策规则

### 12.1 Rotation-aware GO

同时满足以下条件才进入 power-cost 阶段：

- 数值测试全部通过；
- 高 yaw/曲率反转区间 current 与 rot 状态/计划差异可重复；
- rot 模型对实际视觉液面的相位或峰值预测不劣于 current；
- 实物 pilot 中 rot 的视觉指标改善方向稳定，或至少显著提高 actual/zero/phase mechanism identifiability；
- completion、tracking、success 和实时性没有不可接受退化；
- 改善不是执行层 limiter 或统一减速造成。

### 12.2 Rotation-aware NO-GO

出现以下任一情况，应保留 current 模型并把旋转一致性写入 limitation/future work：

- 新增项在实际 \(\omega,\alpha\) 分布下几乎不改变状态和第一动作；
- 视觉液面预测没有改善，甚至因 \(\alpha\) 噪声系统性恶化；
- 求解稳定性或实时性明显下降；
- 只有通过大幅改动其他权重才能看到收益；
- 机制差异仅存在于不可能达到的人工极端状态。

### 12.3 Power-cost GO

- 在相同完成时间附近，\(I_+\) 和 \(H_{\mathrm{vis}}\) 均下降；
- 相位翻转时第一动作变化方向符合能量平衡；
- 相比 energy-only，不是简单降低全程速度；
- 权重附近存在稳定工作区，不再出现严重非单调停滞；
- 结果可以用一个新增权重 \(w_P\) 解释，而不是多个联动参数。

---

## 13. 论文结构的对应修改

### 13.1 Related Work 必须新增或加强

1. **Hamaguchi & Taniguchi 2005**：不要只把其概括为速度曲线/路径设计，还应承认其 WMR 球摆模型已经包含曲线转弯耦合。
2. **Di Leva et al. 2026**：必须新增，这是旋转一致低阶模型最直接的近期先例。
3. **Ardakani & Bridges 2011**：作为完整刚体旋转液体模型的高保真边界。
4. **Muchacho et al. 2022、Arrizabalaga et al. 2024**：说明姿态—平移耦合和 SO(3) 几何处理已有充分基础。
5. **Lim 2024、Chen 2024、Ferrari 2026**：分别作为移动底盘球摆优化、MPC fast replanning 和 SCARA time-optimal 近邻。
6. **Passivity-based lateral slosh control**：如果使用 signed-power/energy 叙事，需要承认 energy/passivity 先例。

### 13.2 方法贡献建议

若 rotation-aware 和 power-cost 都通过，贡献可改成：

1. **Rotation-consistent liquid-memory MPCC.** 采用固定轴旋转下的一阶二维模态动力学，并将实际执行运动传播得到的状态跨控制周期保留在 path-progress MPCC 中。
2. **Phase-aware generalized-power objective.** 通过有符号外部功率区分增加和避免名义模态能量的候选动作，而不是只惩罚速度、加速度或无符号状态幅值。
3. **Matched physical evidence.** 通过视觉液面、Smooth-only、Smooth-match、跨容器、phase/replay 和执行链审计证明改进来自动态液体记忆，而不是普通平滑、整体减速或下游限幅。

若仅 rotation-aware 通过，则不要强行加入 power contribution，贡献写成在线集成和实物机制证据即可。

### 13.3 不能把 adopted model 写成自创公式

推荐写法：

> We adopt and specialize the fixed-axis rotation-aware first-mode dynamics from Di Leva et al. to the planar motion of a differential-drive base. Our contribution is not the underlying SCARA slosh equation itself, but its carried-state integration into an online path-progress MPCC loop and the resulting state-dependent motion decisions.

---

## 14. 主要风险与反方审稿意见

### 风险 1：旋转模型是已有工作，创新反而被削弱

应对：主动引用，不声称模型原创；把创新放在在线决策层、跨周期状态和实物因果证据。

### 风险 2：实际 yaw 相对液体固有频率过小

应对：先用实际日志分布而不是 actuator upper bound 计算无量纲项；若影响不可辨识，NO-GO。

### 风险 3：\(\alpha\) 数值微分噪声污染 observer

应对：独立冻结滤波、记录 raw/filtered、做延迟分析，不允许按方法分别调滤波器。

### 风险 4：低阶模型对真实液面方向不可观

当前视觉主指标是最大液面高度，可能无法直接验证晃动平面的方位。应利用：

- 两相机或可辨方向的视觉测量；或
- 受控残余波 + yaw + 后续平移实验；或
- 以实际液面幅值为物理结果，以 rotation replay/phase study 为方向机制证据。

### 风险 5：signed-power 只是另一种调权方式

应对：主比较必须是 `S-MPCC-rot` 与保持同一 quadratic 的 `S-MPCC-rot-power`；若启用 energy 参数化，再追加 energy-only 与 energy+positive-power。两条分支都应展示相同能量、相反相位下动作方向不同，否则该项不能作为贡献。

### 风险 6：方法和实验继续膨胀，无法压入 RA-L 8 页

应对：正文只保留最终动力学、能量平衡、一个机制图、主物理表和关键 replay。详细推导、搜索记录、pilot 和测试放代码/数据仓库引用，而不是试图塞入正文附录。

---

## 15. 推荐执行清单

### P0：文献与公式冻结

- [ ] 将 Di Leva 2026、Ardakani 2011、Hamaguchi 2005 补入 Related Work 候选库；
- [ ] 手工复核第 6 节 compact/component signs；
- [ ] 用符号计算或有限差分复核第 7 节能量平衡；
- [ ] 明确 \(\eta_x,\eta_y\) 是容器/车体系坐标，并冻结 \(\boldsymbol\nu_b\) 的导数定义；
- [ ] 通过圆柱容器成对简并模态与世界系重构测试；
- [ ] 冻结 rotation-only 的数学版本号。

### P1：软件实现

- [ ] CasADi/acados rotation-aware dynamics；
- [ ] C++ observer 使用 \(\omega,\alpha\)；
- [ ] delay predictor 和 warm start 同步；
- [ ] 新增模型版本与功率诊断；
- [ ] 全部单元测试和 replay 确定性测试。

### P2：Shadow/replay

- [ ] 旧 bag current/rot 双模型 shadow；
- [ ] 高曲率区段差异报告；
- [ ] actual/zero/phase replay；
- [ ] 实际 solve-time 与 failure audit；
- [ ] Rotation GO/NO-GO 决策记录。

### P3：独立 pilot

- [ ] rotation-only paired pilot；
- [ ] 若通过，再做 power/no-power pilot；
- [ ] 只用独立 pilot 选方法和权重；
- [ ] 不把 pilot 混入正式样本。

### P4：正式冻结

- [ ] 最终 controller config、codegen、commit 和 hash；
- [ ] Smooth-only、Smooth-match 重新确认公平性；
- [ ] 更新实验协议中的方法定义；
- [ ] 正式随机区组开始后不再修改动力学或代价。

---

## 16. 参考文献与核验链接

1. Hamaguchi, M., & Taniguchi, T. (2005). Damping and Transfer Control of Liquid in a Cylindrical Container Using a Wheeled Mobile Robot. *Journal of Robotics and Mechatronics, 17*(5), 546–552.

   DOI: <https://doi.org/10.20965/jrm.2005.p0546>

   开放全文：<https://ir.lib.shimane-u.ac.jp/34733/files/11957>

2. Alemi Ardakani, H., & Bridges, T. J. (2011). Shallow-water sloshing in vessels undergoing prescribed rigid-body motion in three dimensions. *Journal of Fluid Mechanics, 667*, 474–519.

   DOI: <https://doi.org/10.1017/S0022112010004477>

3. Di Leva, R., Carricato, M., Gattringer, H., & Müller, A. (2022). Sloshing dynamics estimation for liquid-filled containers performing 3-dimensional motions: Modeling and experimental validation. *Multibody System Dynamics, 56*(2), 153–171.

   DOI: <https://doi.org/10.1007/s11044-022-09841-0>

4. Cabral Muchacho, R. I., Laha, R., Figueredo, L. F. C., & Haddadin, S. (2022). A Solution to Slosh-free Robot Trajectory Optimization. *IROS 2022*, 223–230.

   DOI: <https://doi.org/10.1109/IROS47612.2022.9981173>

   arXiv: <https://arxiv.org/abs/2210.12614>

5. Arrizabalaga, J., Pries, L., Laha, R., Li, R., Haddadin, S., & Ryll, M. (2024). Geometric Slosh-Free Tracking for Robotic Manipulators. *ICRA 2024*, 1226–1232.

   DOI: <https://doi.org/10.1109/ICRA57147.2024.10610813>

   arXiv: <https://arxiv.org/abs/2402.05197>

6. Lim, W., Jung, J., & Han, S. (2024). 2D Trajectory Optimization Using Spherical Pendulum Dynamic Constraints for Reducing Liquid Sloshing on Mobile Robots. *ICCAS 2024*, 487–492.

   DOI: <https://doi.org/10.23919/ICCAS63016.2024.10773274>

7. Chen, F.-W., & Lian, F.-L. (2024). Fast Replanning Slosh-Free Trajectory for Mobile Manipulation Using Model Predictive Control. *ARIS 2024*, 1–5.

   DOI: <https://doi.org/10.1109/ARIS62416.2024.10679970>

8. Di Leva, R., Soprani, S., Palli, G., Biagiotti, L., & Carricato, M. (2026). Sloshing-height estimation for liquid-filled containers under four-dimensional motions including spatial translation and rotation about a fixed direction: Modelling and experimental validation. *Nonlinear Dynamics, 114*(8), Article 618.

   DOI: <https://doi.org/10.1007/s11071-026-12443-6>

   关键位置：Sec. 6.1，Eqs. 54–57。

9. Ferrari, A., Di Leva, R., Soprani, S., Biagiotti, L., Palli, G., & Carricato, M. (2026). Time-Optimal Anti-Sloshing Trajectory Planning for Multiple Liquid-Filled Containers Subject to SCARA Motion. *IEEE Robotics and Automation Letters, 11*(2), 1762–1769.

   DOI: <https://doi.org/10.1109/LRA.2025.3643281>

10. Gogte, G., Venkatesh, C., Tiwari, D., & Singh, N. M. (2013). Passivity Based Control for Lateral Slosh. *Communications in Computer and Information Science*, 673–681.

    DOI: <https://doi.org/10.1007/978-3-642-36321-4_62>

11. Reyhanoglu, M., & Rubio Hervas, J. (2012). Nonlinear dynamics and control of space vehicles with multiple fuel slosh modes. *Control Engineering Practice, 20*(9), 912–918.

    DOI: <https://doi.org/10.1016/j.conengprac.2012.05.011>

---

## 17. 最终建议

本轮优化不应包装成“发现过去没人考虑旋转”。相关工作已经存在，而且 2026 年出现了非常直接的低阶 SCARA 模型和 RA-L 轨迹优化先例。

对后续独立方法版本，更稳健的路线是：

> 把旋转一致性作为候选的物理模型升级；把新增贡献放在标准轮式底盘的在线 path-progress MPCC、跨周期液体动态记忆、有符号相位能量决策，以及 matched physical evidence 上。

只有当 rotation-only 和 power-cost 分别通过独立 stop/go 后，才将它们写入未来版本的 S-MPCC。它们不是当前 88 次实验或当前投稿版本成立的必要条件。若它们未产生可辨识收益，宁可保留当前较简单的方法并诚实说明旋转项在本工况下被忽略，也不要为了公式数量引入未经证实的复杂性。

---

检索与写作说明：本文档由 AI 辅助完成定向文献检索、公开全文核查、公式整理和实施规划。正式投稿前，作者应亲自阅读全文核对所有拟引用论文，并对旋转方程、符号约定和能量平衡进行独立推导确认。
