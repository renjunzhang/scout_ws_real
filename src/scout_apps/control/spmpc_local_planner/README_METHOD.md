# SPMPC 论文方法章节参考

> 文档用途：记录当前代码仍保留的 full-horizon online S-MPCC 基座，为历史论文 Introduction、Method、Ablation 和 Discussion 提供方法口径；当前论文目标已经转向 Phase-Rejoining，见 [`README.md`](README.md)。
> 核对版本：2026-08-21，分支 `offline-slosh-plan-online-tracking`，重构验收提交 `92cd2eac`。
> 数学形式仍是历史 continuous MPCC；代码路径、求解接口和运行时模块边界已按当前实现同步。代码仍是最终事实来源。
> 实物边界：当前只能称为“实物执行感知骨架已经建立”，不能称为“已经适配实物”；正式实物闭环仍是 **G0 NO-GO**。

---

## 0. 一句话方法定义

SPMPC（晃液感知模型预测轮廓控制）是一个面向标准轮式移动底盘开口液体运输的在线局部规划与控制方法。它在 alpha-state MPCC 中同时传播机器人状态、路径进度和低阶液体模态状态，并联合优化路径跟踪、路径推进、底盘可执行性、控制平滑性和预测晃液响应；在此基础上，可选的 Slosh-risk Reference Governor 根据短时预测风险在线调节进入 MPCC 的参考速度。

核心思想不是“让轨迹更平滑”这么简单，而是：

> 液体晃动具有动态记忆。未来液体响应不仅取决于未来加速度，还取决于当前液体模态位移、模态速度和残余振荡相位。因此，液体状态应作为在线规划状态进入预测模型，而不能只用速度、加速度或 jerk 的静态平滑指标间接替代。

---

## 1. 建议写进论文的贡献点

### 贡献 1：液体动态记忆增强的在线 MPCC 局部规划

将标准轮式移动底盘的开口液体运输表述为在线 slosh-aware local planning 问题，并提出液体模态状态增强的 alpha-state MPCC。该方法在同一滚动时域最优控制问题中联合优化：

1. 参考路径的 contour/lag 误差；
2. 路径进度状态和虚拟进度速度；
3. 可执行的底盘线速度、角速度、线加速度和角加速度；
4. 跨控制周期的命令连续性；
5. 预测液体模态位移、模态速度和残余晃液响应。

该贡献的关键区别是“显式传播液体动态状态”，而不是仅提高控制平滑权重。建议用 `B0 / B_smooth / B_slosh / B_ours` 消融证明这一点。

### 贡献 2：有限时域预测晃液风险参考调节

在内层 SPMPC 外增加轻量 Slosh-risk Reference Governor。它对离散速度比例候选进行短时液体状态 rollout，选择满足预测风险阈值的最大参考速度比例，并对比例变化进行速率限制与最终风险复核：

$$
\beta^\star
=
\max_{\beta\in\mathcal B}\ \beta,
\qquad
\text{s.t.}\quad
\max_k \frac{H_k(\beta)}{H_{\mathrm{lim}}}
\le \rho_{\mathrm{th}}.
$$

该 governor 只调节内层 MPCC 的参考速度，不直接裁剪 `/cmd_vel`，也不改变 acados OCP 的状态维度和求解结构。因此它形成“外层风险自适应参考 + 内层连续 MPCC”的分层结构。

该贡献必须通过 `B_ours` 与 `B_ours_gov` 的 governor off/on 对照验证；如果 governor 显著降低平均速度，还应增加 `B_ours_uniform_slow`，排除“只是整体变慢”的解释。

### 贡献 3：区分模型预测与真实液面的受控证据链

通过同一求解器下的模块拆解消融、普通非液体感知局部规划器对比，以及独立 RGB 液面指标，验证液体状态预测相对于 smooth-only 运动生成的独立价值。

该贡献主要属于实验与证据链设计：

- 内部模型量用于优化和在线诊断；
- RGB 或其他外部液面观测用于论文主评价；
- 不使用内部模型预测量自我证明模型有效。

### 不建议单独宣称为核心创新的内容

以下内容重要，但更适合作为方法实现、实时部署或安全保障，而不是独立的核心算法贡献：

- 将 `omega` 设为状态、将 `alpha` 设为控制的 alpha-state 实现；
- acados SQP-RTI、差速平坦性热启动和保守 warm-start fallback；
- terminal slowdown、capture stop、spin-fail 和 tracking safety gate；
- 命令历史记录、延迟诊断和 fixed closed-loop 状态补偿；
- Map-vref 历史接口；
- ROS diagnostics、录包和实验脚本。

其中 alpha-state 是完整 SPMPC 能够输出平滑、受约束底盘命令的重要组成部分，应在 Method 中写清楚，但不宜表述为“首次提出 alpha-state MPCC”。

---

## 2. 问题设置与适用范围

考虑一个标准轮式移动底盘携带开口或半开口液体容器，沿给定安全参考路径运动。在线局部规划器在每个控制周期接收：

- 当前机器人位姿、线速度和角速度；
- 全局或固定安全参考路径；
- 当前内部液体模态状态；
- 上一控制周期的控制量；
- 可选的运行时参考速度和命令历史。

规划器输出普通移动底盘可直接执行的速度命令：

$$
\boldsymbol u_{\mathrm{cmd}}
=
\begin{bmatrix}
v_{\mathrm{cmd}} & \omega_{\mathrm{cmd}}
\end{bmatrix}^{\mathsf T}.
$$

本文第一版方法的合理范围是：

- 给定安全参考路径附近的在线局部轨迹规划与控制；
- 差速或等效非完整约束轮式移动底盘；
- 低阶模型驱动的晃液状态预测；
- 实时滚动时域优化与底盘速度命令输出。

当前不应宣称：

- 完整 obstacle-aware MPCC 已经实现；
- 车体多边形碰撞约束已经进入当前 continuous MPCC OCP；
- 已实现随机 MPC、协方差传播或机会约束；
- 模型硬约束等价于真实液面无溢出保证；
- 内部液体模态状态是由真实液面传感器闭环估计得到的；
- Map-vref 或 terrain-conditioned experience map 属于第一版主方法。

---

## 3. 在线闭环总体结构

当前控制链可概括为：

```text
/odom + TF + reference path
        │
        ├─ robot state: [px, py, theta, v, omega]
        ├─ odom-driven slosh propagation
        └─ path projection and local cubic reference fitting
        │
        ▼
optional runtime/profile v_ref
        │
        ▼
optional Slosh-risk Reference Governor
        │   caps v_ref_current only
        ▼
optional delay-phase execution-state prediction
        │
        ▼
alpha-state slosh-aware MPCC
        │
        ├─ contour / lag tracking
        ├─ path progress s / v_s
        ├─ chassis dynamics and bounds
        ├─ smoothness / anti-creep
        ├─ slosh modal dynamics and cost
        └─ optional modal hard cap
        │
        ▼
first optimal control [a0, alpha0, v_s0]
        │
        ▼
[v_cmd, omega_cmd] + terminal/safety handling
```

需要注意当前代码顺序：`SpeedReferenceController` 先基于 `ControlCycleInputResult.raw_input` 中的机器人/液体状态调节 `v_ref_current`，随后 `ControlCycleInputPreparer::completePrediction()` 才应用可选的 delay-phase 状态预测并形成 `solver_input`。底层 execution predictor 自身只接触 domain state/command/time，不依赖 `SolverInput`。因此在 `fixed_closed_loop` 模式下，governor 与内层 MPCC 仍可能分别使用原始状态和预测执行状态；论文若将 governor 与延迟补偿同时作为主实验设置，应明确说明或进一步统一状态口径。

---

## 4. 机器人、路径进度与控制变量

### 4.1 Alpha-state 机器人模型

当前主线将角速度 `omega` 放入状态，将角加速度 `alpha` 作为控制。

非晃液基础状态为：

$$
\boldsymbol x_r
=
\begin{bmatrix}
p_x & p_y & \theta & v & s & \omega
\end{bmatrix}^{\mathsf T}.
$$

控制输入为：

$$
\boldsymbol u
=
\begin{bmatrix}
a & \alpha & v_s
\end{bmatrix}^{\mathsf T},
$$

其中：

- $a=\dot v$：底盘切向线加速度；
- $\alpha=\dot\omega$：底盘角加速度；
- $v_s=\dot s$：参考路径上的虚拟进度速度；
- $s$：参考路径弧长或路径进度参数。

连续时间运动学为：

$$
\begin{aligned}
\dot p_x &= v\cos\theta,\\
\dot p_y &= v\sin\theta,\\
\dot\theta &= \omega,\\
\dot v &= a,\\
\dot s &= v_s,\\
\dot\omega &= \alpha.
\end{aligned}
$$

该结构使角速度连续性成为动力学的一部分，并允许在 OCP 内直接约束角加速度。相比将 $\omega$ 作为每一步可瞬时改变的直接控制量，它更符合底盘的实际执行能力。

### 4.2 机器人和控制边界

当前主线 OCP 使用以下 box constraints：

$$
0\le v_k\le v_{\max},
\qquad
-\omega_{\max}\le\omega_k\le\omega_{\max},
$$

$$
-a_{\max}\le a_k\le a_{\max},
\qquad
-\alpha_{\max}\le\alpha_k\le\alpha_{\max},
$$

$$
0\le v_{s,k}\le v_{s,\max}.
$$

当前实现取：

$$
v_{s,\max}=v_{\max}.
$$

默认平台参数为：

$$
v_{\max}=0.8\ \mathrm{m/s},\quad
\omega_{\max}=1.2\ \mathrm{rad/s},
$$

$$
a_{\max}=0.6\ \mathrm{m/s^2},\quad
\alpha_{\max}=1.2\ \mathrm{rad/s^2}.
$$

---

## 5. 参考路径、进度投影与轮廓误差

### 5.1 局部参考曲线

机器人当前位置首先投影到参考路径上，得到当前进度 $s_0$。代码对进度使用非回退保护：

$$
s_0\ge s_{\min},
$$

其中 $s_{\min}$ 是已接受的上一周期路径进度，用于减少路径投影跳回造成的局部规划不稳定。

在当前预测窗口内，参考路径被拟合为关于 $s$ 的三次多项式：

$$
r_x(s)=c_{x,0}+c_{x,1}s+c_{x,2}s^2+c_{x,3}s^3,
$$

$$
r_y(s)=c_{y,0}+c_{y,1}s+c_{y,2}s^2+c_{y,3}s^3.
$$

参考切向角为：

$$
\phi(s)=\operatorname{atan2}\left(r_y'(s),r_x'(s)\right).
$$

局部多项式覆盖区间近似为：

$$
[s_0,s_{\mathrm{end}}],
\qquad
s_{\mathrm{end}}
=
\min\left(s_{\mathrm{path}},s_0+v_{\max}T_f\right).
$$

### 5.2 Contour 和 lag 误差

定义位置误差：

$$
\Delta p_x=p_x-r_x(s),
\qquad
\Delta p_y=p_y-r_y(s).
$$

当前代码中的 contour error 为：

$$
e_c
=
\sin\phi(s)\,\Delta p_x
-
\cos\phi(s)\,\Delta p_y.
$$

lag error 为：

$$
e_l
=
-\cos\phi(s)\,\Delta p_x
-
\sin\phi(s)\,\Delta p_y.
$$

$e_c$ 表示相对参考路径的法向偏差，$e_l$ 表示沿切向的进度误差。lag 的符号不影响当前二次代价，但论文公式应与实现保持一致。

### 5.3 路径曲率

由局部参考多项式可得到曲率：

$$
\kappa(s)
=
\frac{r_x'(s)r_y''(s)-r_y'(s)r_x''(s)}
{\left(r_x'(s)^2+r_y'(s)^2+\varepsilon\right)^{3/2}}.
$$

当前 cost 中使用 $\varepsilon=10^{-6}$ 避免分母退化。

---

## 6. 低阶液体模态模型

### 6.1 二维主导模态状态

两个平面方向的液体状态写为：

$$
\boldsymbol x_l
=
\begin{bmatrix}
\eta_x & \dot\eta_x & \eta_y & \dot\eta_y
\end{bmatrix}^{\mathsf T}.
$$

单个方向采用二阶阻尼模态模型：

$$
\ddot\eta_i
+2\zeta\omega_n\dot\eta_i
+\omega_n^2\eta_i
=-\kappa_i a_i,
\qquad i\in\{x,y\}.
$$

当前轮式底盘近似使用：

$$
a_x=a,
\qquad
a_y=v\omega.
$$

因此连续液体动力学为：

$$
\begin{aligned}
\frac{\mathrm d}{\mathrm dt}\eta_x
&=\dot\eta_x,\\
\frac{\mathrm d}{\mathrm dt}\dot\eta_x
&=-2\zeta\omega_n\dot\eta_x
-\omega_n^2\eta_x
-\kappa_x a,\\
\frac{\mathrm d}{\mathrm dt}\eta_y
&=\dot\eta_y,\\
\frac{\mathrm d}{\mathrm dt}\dot\eta_y
&=-2\zeta\omega_n\dot\eta_y
-\omega_n^2\eta_y
-\kappa_y v\omega.
\end{aligned}
$$

当前 acados wrapper 运行时注入：

$$
\kappa_x=\kappa_y=1.
$$

$\omega_n$ 和液面高度系数 $c_h$ 由纯 C++ `SloshDynamics` 直接按已冻结的模态根、公式和 ZOH 运算顺序计算，阻尼比 $\zeta$ 来自同一份 typed config。该实现保留与历史 `slosh_models::LiquidSloshModel` 合同的逐值回归，但本包已不再对 `slosh_models` 建立编译或运行依赖。论文不应另写一套与代码不一致的经验参数。

### 6.2 增强状态

完整 SPMPC 状态为：

$$
\boldsymbol x
=
\begin{bmatrix}
p_x & p_y & \theta & v & s & \omega &
\eta_x & \dot\eta_x & \eta_y & \dot\eta_y
\end{bmatrix}^{\mathsf T}.
$$

即：

- `B0/B_smooth` 使用 6 维机器人与进度状态；
- `B_slosh/B_ours` 使用 10 维机器人、进度与液体增强状态。

液体状态进入 OCP 的意义不只是增加一个晃液惩罚量，而是使预测模型具有液体动态记忆：

$$
\boldsymbol x_{k+1}=f_d(\boldsymbol x_k,\boldsymbol u_k).
$$

在给定相同未来控制的情况下，不同的当前 $[\eta,\dot\eta]$ 会产生不同的未来晃液预测，因此可能改变规划器对局部运动的选择。

### 6.3 在线内部液体状态传播

当前系统没有用 RGB 液面测量闭环更新液体模态状态。ROS 层根据相邻 odometry 样本估计激励：

$$
\hat a_{x,k}
=
\frac{v_k-v_{k-1}}{\Delta t_{\mathrm{odom}}},
\qquad
\hat a_{y,k}=v_k\omega_k.
$$

然后用共享离散液体模型传播：

$$
\boldsymbol x_{l,k+1}
=
A_d\boldsymbol x_{l,k}
+B_d
\begin{bmatrix}
\hat a_{x,k}\\
\hat a_{y,k}
\end{bmatrix}.
$$

因此更准确的称呼是 model-driven slosh state propagation，而不是 real liquid-surface observer。

### 6.4 模型液面高度代理量

内层 solver 的代价、硬约束和主要预测诊断统一采用 modal-only 高度：

$$
H_{\mathrm{modal}}
=
c_h\sqrt{\eta_x^2+\eta_y^2}.
$$

系统中还保留可选的转弯准静态抛物面修正：

$$
H_{\mathrm{para}}
=
\frac{R^2\omega^2}{4g},
$$

$$
H_{\mathrm{proxy}}
=
H_{\mathrm{modal}}+H_{\mathrm{para}}.
$$

但当前内层 MPCC 的 slosh cost 和 hard constraint 不包含 $H_{\mathrm{para}}$。该项只可用于可选 observer/governor 高度口径或可视化，不能把不同口径混写为同一个“液面高度约束”。

论文真实液面评价应使用离线 RGB max-LCR 或其他外部测量，不能把 $H_{\mathrm{modal}}$ 直接称为真实液面高度。

---

## 7. Slosh-aware MPCC 最优控制问题

### 7.1 有限时域 OCP

设预测步数为 $N$，采样时间为 $\Delta t$。每个控制周期求解：

$$
\begin{aligned}
\min_{\boldsymbol x_{0:N},\boldsymbol u_{0:N-1}}
\quad &
\sum_{k=0}^{N-1}\ell_k(\boldsymbol x_k,\boldsymbol u_k)
+\ell_N(\boldsymbol x_N)\\
\text{s.t.}\quad
&\boldsymbol x_0=\hat{\boldsymbol x}(t),\\
&\boldsymbol x_{k+1}=f_d(\boldsymbol x_k,\boldsymbol u_k),\\
&\boldsymbol x_k\in\mathcal X,\\
&\boldsymbol u_k\in\mathcal U,\\
&k=0,\ldots,N-1.
\end{aligned}
$$

当前默认：

$$
N=60,
\qquad
\Delta t=\frac{1}{30}\ \mathrm{s},
\qquad
T_f=N\Delta t=2.0\ \mathrm{s}.
$$

### 7.2 曲率自适应参考速度

进入 cost 的基础参考速度记为 $v_{\mathrm{cruise}}$。它可能来自：

1. variant 的固定 `v_ref`；
2. 可选 runtime/profile override；
3. 可选 slosh-risk governor 调节后的参考速度。

内层 cost 根据路径曲率进一步构造：

$$
v_{\kappa}(s)
=
\frac{v_{\mathrm{cruise}}}
{\sqrt{1+\left(\frac{v_{\mathrm{cruise}}|\kappa(s)|}
{\omega_{\max}}\right)^2}}.
$$

最终阶段参考速度为：

$$
v_{\mathrm{ref}}(s)
=
\max\left(0.3v_{\mathrm{cruise}},v_{\kappa}(s)\right).
$$

其作用是使高曲率区域的参考速度自然降低，同时保留正的参考速度下限，避免弯道中参考速度退化为零。

### 7.3 路径跟踪代价

当前误差项做无量纲归一化：

$$
J_{\mathrm{track},k}
=
w_c\left(\frac{e_{c,k}}{e_{c,\mathrm{ref}}}\right)^2
+
w_l\left(\frac{e_{l,k}}{e_{l,\mathrm{ref}}}\right)^2.
$$

当前运行时取：

$$
e_{c,\mathrm{ref}}
=
\max\left(10^{-3},\frac{W_{\mathrm{corridor}}}{2}\right),
$$

$$
e_{l,\mathrm{ref}}
=
\max\left(0.1,v_{\max}\Delta t\right).
$$

这里的 corridor width 主要用于误差归一化。当前 continuous MPCC 并没有因此自动获得完整的 corridor hard constraint。

### 7.4 路径推进与 anti-creep 代价

线性路径推进奖励为：

$$
J_{\mathrm{progress},k}
=
-w_p\frac{v_{s,k}}{v_{s,\max}}.
$$

物理速度和虚拟进度速度跟踪为：

$$
J_{v,k}
=
w_v
\left(
\frac{v_k-v_{\mathrm{ref}}(s_k)}{v_{\max}}
\right)^2,
$$

$$
J_{v_s,k}
=
w_{v_s}
\left(
\frac{v_{s,k}-v_{\mathrm{ref}}(s_k)}{v_{s,\max}}
\right)^2.
$$

为避免优化器在弯道中通过把 $v$ 或 $v_s$ 压得很低而得到停滞解，当前主线还增加只惩罚“低于参考速度”的非对称 anti-creep 项：

$$
\delta_v
=
\frac{\max(0,v_{\mathrm{ref}}(s_k)-v_k)}{v_{\max}},
$$

$$
\delta_{v_s}
=
\frac{\max(0,v_{\mathrm{ref}}(s_k)-v_{s,k})}{v_{s,\max}},
$$

$$
J_{\mathrm{ac},k}
=
\gamma_{\mathrm{ac}}w_v
\left(\delta_v^2+\delta_{v_s}^2\right).
$$

当前 codegen 默认：

$$
\gamma_{\mathrm{ac}}=8.
$$

### 7.5 控制幅值和实际平滑项

控制幅值代价为：

$$
J_{\mathrm{ctrl},k}
=
w_a\left(\frac{a_k}{a_{\max}}\right)^2
+w_\omega\left(\frac{\omega_k}{\omega_{\max}}\right)^2
+w_\alpha\left(\frac{\alpha_k}{\alpha_{\max}}\right)^2.
$$

其中 $\omega_k$ 是状态，$\alpha_k$ 是控制。全时域的 $\alpha$ 惩罚相当于直接抑制角速度变化率，从而提高转向命令平滑性。

当前 wrapper 中的权重映射为：

$$
w_a=w_{\mathrm{control}}+w_{\mathrm{accel}},
\qquad
w_\omega=w_{\mathrm{control}}.
$$

如果变体没有单独配置分裂后的平滑权重，则：

$$
w_\alpha=w_{\Delta a}=w_{\Delta v_s}=w_{\mathrm{smooth}}.
$$

当前代码并没有在所有预测步显式加入完整的
$\|\boldsymbol u_k-\boldsymbol u_{k-1}\|_S^2$。实际的跨周期连续性项只在 stage 0 生效：

$$
J_{\Delta u,0}
=
w_{\Delta a}
\left(\frac{a_0-a_{\mathrm{prev}}}{a_{\max}}\right)^2
+
w_{\Delta v_s}
\left(\frac{v_{s,0}-v_{s,\mathrm{prev}}}{v_{s,\max}}\right)^2.
$$

对 $k>0$：

$$
J_{\Delta u,k}=0.
$$

如果尚无上一控制周期的有效控制量，例如节点刚启动后的第一次求解，则 stage-0 连续性权重也置零。

角速度跨周期连续性由 $\omega$ 状态初值保证，预测域内的角速度变化由 $\dot\omega=\alpha$、$|\alpha|\le\alpha_{\max}$ 和 $w_\alpha\alpha^2$ 共同塑造。

因此论文不应直接把当前实现写成“所有 stage 都具有完整 $\Delta a/\Delta\omega/\Delta v_s$ 代价”。

### 7.6 晃液状态代价

定义：

$$
\|\boldsymbol\eta_k\|^2
=
\eta_{x,k}^2+\eta_{y,k}^2,
$$

$$
\|\dot{\boldsymbol\eta}_k\|^2
=
\dot\eta_{x,k}^2+\dot\eta_{y,k}^2.
$$

模态位移参考尺度由模型高度参考量确定：

$$
\eta_{\mathrm{ref}}
=
\frac{H_{\mathrm{ref}}}{c_h}.
$$

模态速度参考尺度为：

$$
\dot\eta_{\mathrm{ref}}
=
\omega_n\eta_{\mathrm{ref}}.
$$

晃液代价为：

$$
J_{\mathrm{slosh},k}
=
w_\eta
\frac{\|\boldsymbol\eta_k\|^2}{\eta_{\mathrm{ref}}^2}
+
w_{\dot\eta}
\frac{\|\dot{\boldsymbol\eta}_k\|^2}
{\dot\eta_{\mathrm{ref}}^2}.
$$

当前参数关系为：

$$
w_\eta=w_{\mathrm{slosh}},
$$

$$
w_{\dot\eta}
=
w_{\mathrm{slosh}}r_{\dot\eta},
$$

其中 $r_{\dot\eta}$ 对应配置 `slosh_eta_dot_ratio`。

该代价使规划器能够区分“控制曲线同样平滑，但与当前液体相位耦合不同”的候选运动。

### 7.7 完整阶段代价

当前 alpha-state 主线的阶段代价可写为：

$$
\ell_k
=
\frac{1}{N}
\left(
J_{\mathrm{track},k}
+J_{\mathrm{progress},k}
+J_{v,k}
+J_{v_s,k}
+J_{\mathrm{ac},k}
+J_{\mathrm{ctrl},k}
+\mathbf 1_{k=0}J_{\Delta u,0}
+J_{\mathrm{slosh},k}
\right).
$$

对于不启用晃液模型的变体：

$$
J_{\mathrm{slosh},k}=0.
$$

除以 $N$ 的目的，是减小不同预测步数下阶段累计权重的尺度漂移。

### 7.8 终端代价

当前终端代价只包含终端路径误差和可选的终端晃液状态：

$$
\ell_N
=
J_{\mathrm{track},N}
+J_{\mathrm{slosh},N}.
$$

终端代价当前没有除以 $N$，也不包含控制项、进度奖励或 anti-creep 项。

### 7.9 可选模态晃液硬约束

对于 `B_slosh_hard` 和 `B_ours_hard`，当前 acados OCP 增加：

$$
\eta_{x,k}^2+\eta_{y,k}^2
\le
\eta_{\max}^2,
$$

其中：

$$
\eta_{\max}
=
\frac{H_{\max}}{c_h}.
$$

等价地：

$$
H_{\mathrm{modal},k}
=
c_h\sqrt{\eta_{x,k}^2+\eta_{y,k}^2}
\le H_{\max}.
$$

实现细节：

- stage 0 的该约束被放宽；
- stage $1,\ldots,N$ 和 terminal 节点受约束；
- 当前没有 slack，是无松弛非线性硬约束；
- soft-only 变体通过注入极大的 $\eta_{\max}^2$ 关闭约束；
- 约束只对应 modal-only 模型代理量，不包括 RGB 真值和抛物面修正项；
- 因模型误差、状态传播误差和执行延迟存在，该约束不能表述为严格无溢出保证。

主实验不应只用 `B_ours_hard` 代表全部方法，否则无法区分 slosh soft cost、smooth shaping 和 hard cap 各自的作用。

### 7.10 诊断代价与真实 OCP 代价的区别

`/spmpc/cost_breakdown` 用于在线解释各项趋势，但当前 C++ 诊断重算并没有完整复制 CasADi EXTERNAL cost：

- 诊断中的 `J_v` 使用单一运行时 `v_ref`，没有完整重算曲率自适应 $v_{\mathrm{ref}}(s_k)$；
- 诊断没有单独计入非对称 anti-creep 附加项；
- 终端 cost 的诊断分解也不是 acados 内部目标值的逐项严格复现。

因此论文若报告“优化目标占比”，应先补齐一致的 objective reconstruction，不能直接把现有 `cost_breakdown` 当作精确 OCP objective。

---

## 8. Slosh-risk Reference Governor

### 8.1 分层作用位置

Governor 位于 MPCC 之前。令未调节参考速度为：

$$
v_{\mathrm{nom}}.
$$

它输出：

$$
v_{\mathrm{gov}}
=
\beta_f v_{\mathrm{nom}},
\qquad
\beta_f\in[\beta_{\min},1].
$$

然后将 $v_{\mathrm{gov}}$ 作为内层 MPCC 的 `v_ref_current`。Governor 不直接修改最终速度命令，也不向 acados 增加状态或约束。

### 8.2 候选集合

当前使用从 1 递减到 $\beta_{\min}$ 的均匀离散网格：

$$
\mathcal B
=
\left\{
1-\frac{i}{M-1}(1-\beta_{\min})
\ \middle|\
i=0,\ldots,M-1
\right\}.
$$

若 $M=1$，只评估 $\beta=1$。

候选按从大到小顺序检查，因此第一个满足风险条件的候选就是网格内最大的可接受速度比例。

### 8.3 短时 surrogate rollout

对每个候选 $\beta$，定义目标速度：

$$
v_{\mathrm{target}}
=
\operatorname{clip}
\left(\beta v_{\mathrm{nom}},0,v_{\mathrm{nom}}\right).
$$

仿真初始速度为：

$$
v_0^{g}
=
\operatorname{clip}
\left(v_{\mathrm{robot}},0,v_{\mathrm{nom}}\right).
$$

Governor 不求解第二个 MPC，而使用受加速度限制的简单追踪模型：

$$
a_{x,k}^{g}
=
\operatorname{clip}
\left(
\frac{v_{\mathrm{target}}-v_k^g}{\Delta t},
-a_g,
a_g
\right).
$$

角速度采用当前角速度指数衰减的 surrogate：

$$
\omega_k^g
=
\omega_0
\exp\left(-\frac{k\Delta t}{\tau_\omega}\right).
$$

当 $\tau_\omega\le0$ 或参数无效时，代码保持当前角速度，不执行指数衰减。

横向激励为：

$$
a_{y,k}^{g}=v_k^g\omega_k^g.
$$

液体状态通过共享 `SloshDynamics` 离散模型传播：

$$
\boldsymbol x_{l,k+1}^{g}
=
f_l^d
\left(
\boldsymbol x_{l,k}^{g},
a_{x,k}^{g},
a_{y,k}^{g}
\right).
$$

速度更新为：

$$
v_{k+1}^{g}
=
\operatorname{clip}
\left(
v_k^g+a_{x,k}^{g}\Delta t,
0,
v_{\mathrm{nom}}
\right).
$$

Governor 的 rollout 是轻量短时风险代理，不使用参考路径未来曲率，也不复用内层 MPCC 的最优控制序列。因此论文应称其为 predictive reference adaptation 或 lightweight slosh-risk governor，而不是第二个完整 MPC。

### 8.4 风险定义与候选选择

对候选 $\beta$，预测峰值高度为：

$$
H_{\mathrm{peak}}(\beta)
=
\max_{k=0,\ldots,N_g}H_k(\beta).
$$

归一化风险为：

$$
\rho_{\mathrm{peak}}(\beta)
=
\frac{H_{\mathrm{peak}}(\beta)}{H_{\mathrm{lim}}}.
$$

候选可接受条件为：

$$
\rho_{\mathrm{peak}}(\beta)
\le
\rho_{\mathrm{th}}.
$$

原始最优比例为：

$$
\beta^\star
=
\max
\left\{
\beta\in\mathcal B
\mid
\rho_{\mathrm{peak}}(\beta)
\le\rho_{\mathrm{th}}
\right\}.
$$

如果网格中没有可接受候选：

$$
\beta^\star=\beta_{\min},
$$

并报告 `SATURATED`。这表示 governor 已达到最保守的允许比例，但不代表预测风险已经满足阈值。

### 8.5 比例变化率限制

设上一周期比例为 $\beta_{f,k-1}$。当前实现分别限制恢复速度和降速速度：

当 $\beta^\star>\beta_{f,k-1}$ 时：

$$
\beta_{f,k}
=
\min
\left(
\beta^\star,
\beta_{f,k-1}+r_{\uparrow}\Delta t
\right).
$$

当 $\beta^\star<\beta_{f,k-1}$ 时：

$$
\beta_{f,k}
=
\max
\left(
\beta^\star,
\beta_{f,k-1}-r_{\downarrow}\Delta t
\right).
$$

最后：

$$
\beta_{f,k}
\leftarrow
\operatorname{clip}
\left(\beta_{f,k},\beta_{\min},1\right).
$$

实际输出参考速度为：

$$
v_{\mathrm{gov}}
=
\operatorname{clip}
\left(
\beta_{f,k}v_{\mathrm{nom}},
v_{\min},
v_{\mathrm{nom}}
\right).
$$

### 8.6 最终风险复核

由于速率限制后的 $\beta_f$ 可能不同于网格选择的 $\beta^\star$，代码会对最终 $\beta_f$ 再做一次 rollout，并报告：

$$
m_\rho
=
\rho_{\mathrm{th}}
-
\rho_{\mathrm{peak}}(\beta_f).
$$

若：

$$
m_\rho\ge0,
$$

则 `predicted_risk_admissible=true`。

若网格存在可行候选，但下降速率限制使最终比例暂时仍过高，则报告：

```text
TRANSIENT_RATE_LIMITED
```

因此 governor 是风险自适应参考调节器，而不是任何时刻都严格保证风险约束的安全过滤器。

配置中的 release_threshold 当前只参与 active 诊断状态判断，不参与候选可行性条件，也没有直接改变 $\beta^\star$ 的选择。论文不应把它描述成已经实现的严格滞回可行域。

### 8.7 Governor 高度口径

Governor 可通过 `include_parabola_height` 选择：

$$
H^g=H_{\mathrm{modal}}
$$

或：

$$
H^g=H_{\mathrm{modal}}+H_{\mathrm{para}}.
$$

当前 governor 配置默认允许包含 parabola term，而内层 OCP hard cap 固定为 modal-only。若 governor 成为论文正式贡献，必须在实验配置中固定并报告该开关，避免内外层风险口径不一致。

---

## 9. 滚动时域执行

每个控制周期只执行最优控制序列的第一项：

$$
\boldsymbol u_0^\star
=
\begin{bmatrix}
a_0^\star & \alpha_0^\star & v_{s,0}^\star
\end{bmatrix}^{\mathsf T}.
$$

底盘命令由本周期 solver 输入状态中的速度单步积分得到。默认模式下该状态来自当前 odometry；启用并通过 fixed_closed_loop 守卫时，它可能是延迟补偿后的预测执行状态：

$$
v_{\mathrm{cmd}}^{\mathrm{pre}}
=
v_{\mathrm{in}}
+a_0^\star\Delta t,
$$

$$
\omega_{\mathrm{cmd}}^{\mathrm{pre}}
=
\omega_{\mathrm{in}}
+\alpha_0^\star\Delta t.
$$

经过物理边界裁剪：

$$
v_{\mathrm{cmd}}
=
\operatorname{clip}
\left(v_{\mathrm{cmd}}^{\mathrm{pre}},0,v_{\max}\right),
$$

$$
\omega_{\mathrm{cmd}}
=
\operatorname{clip}
\left(
\omega_{\mathrm{cmd}}^{\mathrm{pre}},
-\omega_{\max},
\omega_{\max}
\right).
$$

随后命令还可能受到 terminal controller、Phase-Rejoin supervisor、发布前 limiter 和安全门控影响。因此上述裁剪只描述 solver 原始输出到候选命令的主要映射，不能替代最终发布边界的安全合同。当前 `ControlCycleEngine::step()` 已通过 `CommandPipeline + PublicationTransaction + ICommandSink` 原子完成一次 finalization 和一次 sink 调用；只有成功且命令一致的 receipt 才允许 Phase-Rejoin commit，成功交付的 receipt 命令才写入 limiter state 和 history。ROS 层只实现 `/cmd_vel` sink 和消息/诊断转换，不再二次改写命令。`ControlCycleAudit` schema v4 分开保存 proposed、finalized 和 published 三层命令、提交状态以及预计/实际发布时间。这里的 receipt 只表示 ROS publisher 接受交付，不是 Scout CAN/底盘 ACK；此外最终 `CommandPipeline` 虽已无条件拒绝非有限值，但仍没有无条件线速度/角速度绝对硬包络，角速度/加速度 limiter 仍是配置式的，独立 freshness/deadline/driver watchdog 也尚未闭合。

---

## 10. 可选执行延迟与相位补偿

该模块不是 SPMPC 核心创新，但会影响实物闭环中 solver 初始状态与命令真正执行时刻的一致性。

命令历史缓存保存：

$$
\mathcal H_u(t)
=
\{(t_i,v_{\mathrm{cmd},i},\omega_{\mathrm{cmd},i})\}.
$$

给定分通道纯延迟和可选一阶惯性：

$$
d_f=\max(d_v,d_\omega),
\qquad
(\tau_v,\tau_\omega)\ge 0,
$$

当前 predictor 使用 $d_f$ 形成共同积分窗口，但在每个积分时刻分别采样：

$$
v^\star(t)=\mathcal H_v(t-d_v),
\qquad
\omega^\star(t)=\mathcal H_\omega(t-d_\omega),
$$

再按各自的 $\tau_v,\tau_\omega$ 更新执行速度。因此代码已经有双通道历史采样和可选一阶执行惯性骨架，而不是单一 delay；但增益、死区、饱和、方向不对称、滑移和电量影响均未进入模型，时间常数也尚未辨识冻结。

执行状态预测器从 $t-d$ 到 $t$ 对历史命令积分：

$$
\begin{aligned}
p_{x,j+1}&=p_{x,j}+v_j\cos\theta_j\Delta t_j,\\
p_{y,j+1}&=p_{y,j}+v_j\sin\theta_j\Delta t_j,\\
\theta_{j+1}&=\theta_j+\omega_j\Delta t_j.
\end{aligned}
$$

同时根据历史速度变化传播液体状态：

$$
a_{x,j}
=
\frac{v_j-v_{j-1}}{\Delta t_j},
\qquad
a_{y,j}=v_j\omega_j.
$$

在 `fixed_closed_loop` 模式下，只有当命令历史完整、预测有效且 odometry 新鲜时，runtime 层返回的 `DelayPhaseApplication` 才同时采用预测机器人状态和预测液体状态；`ControlCycleInputPreparer` 随后把它们写入 `solver_input`。否则保持 `raw_input` 中的原始状态。这个边界保证 execution predictor 不依赖求解接口。

当前 predictor 已改为通过纯 C++ `ExecutionModel` 回放 published-command history。`ExecutionModelContract` 统一双通道整步/fractional delay、`tau`、方向增益、死区和饱和语义，`ExecutionAugmentedState` 显式保存两路 pending buffer 与 actuator output；同一个模型再传播机器人和液体状态。这个兼容路径仍然是 history-only，不包含求解中的本周期新决策。现在明确区分：

```text
t_c       控制周期开始时刻
t_e       求解前调用 predictor 的 evaluation epoch
t_pub     本周期经过 solver、安全链和 limiter 后交给 ROS publisher 的实际时间
d_c       t_pub - t_c
t_hat_pub t_c + d_hat_c
```

当 `PublishEpochEstimate` 有效时，代码现在以 $\widehat t_{\mathrm{pub}}$ 作为 history predictor 的 evaluation epoch，用旧的 published-command history 先把 source-stamped robot/liquid state 对齐到预计发布时间，再预测到：

$$
\widehat t_{\mathrm{pub}}+\max(d_v,d_\omega).
$$

同一个 typed estimate 也进入 PhaseClock、`SolverInput.publish_epoch_estimate` 和 `SolverInput.cycle_timing`；任一字段与当前周期不可重算一致时，在状态查询或 solver 调用前 fail closed。`PublishLatencyModel` 从 $t_c$ 生成：

$$
\widehat t_{\mathrm{pub}}=t_c+\widehat d_c,
$$

并在 `ControlCycleAudit` 中记录实际 $d_c$、误差和 deadline miss。默认 `publish_timing.enabled=false`，此时 estimate 无效，predictor 和 PhaseClock 保持使用显式求解前 evaluation epoch $t_e$，因此默认运行行为没有改变。$\widehat d_c$ 仍需由 Scout held-out 标定 artifact 冻结。

此外，当 $d_v=150\,\mathrm{ms}$、$d_\omega=220\,\mathrm{ms}$ 时，本周期新线速度命令会在共同角速度前沿前约 $70\,\mathrm{ms}$ 开始作用。当前在线 predictor 在求解前仍只使用旧命令，固定的前沿状态没有保留这段对新决策的依赖。预计发布时间的在线 typed 接线已经完成；WP3A 增加纯 C++ `DelayAugmentedPhaseDynamics`，让 $q=[a,\alpha,v_s]$ 从上一发布命令形成新 $u^{\mathrm{pub}}$、进入双通道 buffer，并按 $N_e=n_f+N_\ell$ 传播；WP3B 生成了 `nx=22,nu=3,N_e=10` 的 CasADi C 离散转移核，完成 128 组随机单步、第一拍 Jacobian 和 terminal Jacobian 与 C++ 参考转移的一致性；WP3C 又以同一转移生成和编译独立 DISCRETE acados capsule，加入 published-command、robot/pending speed 和 rate 硬约束。但新 capsule 尚未进入在线 factory/history context，也没有 formal nominal-relative cost/parameters、terminal 9D gate 和 $\mathcal B^{\mathrm{exec}}$；capability gate 会明确拒绝 formal 请求。因此执行增广模型已进入独立 optimizer，但尚未进入在线正式闭环。

默认配置也没有启用该功能：`delay_phase.mode=off`、`phase_rejoin.mode=off`、两个时间常数为 0、`require_complete_history=false`。官方实物 runner 的非 pilot 默认关闭 delay；pilot 只显式传两个纯延迟，未传时间常数、完整历史和 Phase-Rejoin 合同参数。因此当前 runner 不能建立 formal `phase_rejoin=enforce` 实物合同。

现有三组正向仿真也全部显式使用 $d_v=d_\omega=\tau_v=\tau_\omega=0$，对应的 Gazebo plant 收到命令后基本直接设置速度。它们只证明零延迟理想执行 proxy 下指标正向，不能证明 Scout 150–220 ms 延迟下仍然正向。

论文建议把该模块写成 real-system state alignment 或 deployment compensation，并在 Discussion 中说明，而不是用它替代 SPMPC 的核心方法贡献。

---

## 11. 实时求解实现

当前 acados 设置为：

```text
integrator                  ERK
ERK stages                 4
ERK steps per interval     1
NLP solver                 SQP_RTI
QP solver                  PARTIAL_CONDENSING_HPIPM
Hessian                    EXACT
regularization             PROJECT
Levenberg-Marquardt        1e-3
```

阶段和终端代价使用 CasADi EXTERNAL cost。B0 和 slosh-aware 模型分别生成独立 solver：

```text
spmpc_b0       6 states, 3 controls
spmpc_slosh   10 states, 3 controls
```

主线后端为：

```text
continuous_mpcc_acados
```

`continuous_mpcc_direct_omega_legacy` 只用于 RouteB 结构诊断，`primitive` 只作为早期 rollout、fallback 或附录对照，不应与当前主线方法混写。

当前求解边界已经拆分为：

```text
solver/api/solver_input.h   权威 SolverInput
solver/api/solver_output.h  权威 SolverOutput
solver/api/solver.h         权威 SpmpcSolver backend 接口
runtime/control_cycle_timing.h
core/start_lock_recovery_diagnostics.h
```

`solver/api/solver_io.h` 和 `core/spmpc_solver.h` 只保留为兼容 facade。Phase-Rejoin coordinator 也不再接收整个 `SolverOutput`，而是由 controller adapter 转成只含命令和终端状态的 `PhaseSolveView`。因此 solver telemetry、执行预测和 phase 决策之间不再通过总 DTO 形成隐式依赖。

当前 `/spmpc/solver_time_ms` 和周期审计会记录求解时间，但没有在求解超过一个 30 Hz 周期（`33.3 ms`）时阻止本周期命令发布的运行期 deadline gate。实物实时性不能仅凭离线日志均值放行。

实物 runner 与路径版本也是部署方法的一部分。当前 1235 行 runner 仍在 Shell 中拼接大量参数，却没有传入两个执行时间常数、完整历史和 Phase-Rejoin formal 合同；目标应是 typed `ExperimentSessionConfig` 经 C++ preflight 生成 immutable manifest，Shell 只做薄启动。路径更新方面，ROS 只比较 frame、点数和首尾位置，`SpmpcProblem` 则比较处理后路径的长度和所有点；两套判定可能导致同首尾、同点数但中间形状变化时，只复位 problem 内部 progress/terminal，而遗留 controller 的 phase、goal latch、speed-reference 和 shifted-plan/warm-start 状态。正式版本必须以唯一 `reference_id/reference_epoch` 驱动整条链原子复位。

---

## 12. 主实验变体和因果问题

| 变体 | 液体状态/代价 | 强平滑权重 | 模态硬约束 | 主要回答的问题 |
|---|---:|---:|---:|---|
| `B0` | 否 | 否 | 否 | 基础 alpha-state MPCC 的表现 |
| `B_smooth` | 否 | 是 | 否 | 只增强平滑性是否足够降低晃液 |
| `B_slosh` | 是 | 否 | 否 | 显式液体动态状态是否有独立价值 |
| `B_ours` | 是 | 是 | 否 | 完整 soft-cost 方法的综合表现 |
| `B_slosh_hard` | 是 | 否 | 是 | 模态 hard cap 在弱平滑条件下的作用 |
| `B_ours_hard` | 是 | 是 | 是 | hard cap 对完整方法的附加影响 |

核心对比：

$$
\texttt{B\_slosh}-\texttt{B0}
\quad\Rightarrow\quad
\text{液体状态/代价的独立作用},
$$

$$
\texttt{B\_smooth}-\texttt{B0}
\quad\Rightarrow\quad
\text{普通平滑的作用},
$$

$$
\texttt{B\_ours}-\texttt{B\_smooth}
\quad\Rightarrow\quad
\text{slosh-aware 是否超过 smooth-only},
$$

$$
\texttt{B\_ours}-\texttt{B\_slosh}
\quad\Rightarrow\quad
\text{强平滑对实物可执行性的附加作用}.
$$

Governor 对比：

下列 B_ours_gov 和 B_ours_uniform_slow 是实验方法标签，不是 variants.yaml 中已经存在的独立 variant；前者表示在 B_ours 上打开 governor，后者表示关闭 governor 并使用匹配的固定低参考速度。

$$
\texttt{B\_ours\_gov}-\texttt{B\_ours}
\quad\Rightarrow\quad
\text{预测风险参考调节的附加收益}.
$$

若完成时间差异明显，还应比较：

$$
\texttt{B\_ours\_gov}
\quad\text{vs}\quad
\texttt{B\_ours\_uniform\_slow}.
$$

---

## 13. 建议的论文 Methods 章节结构

### 13.1 Problem Formulation

说明：

- 标准轮式移动底盘沿给定安全路径运输开口液体；
- 目标是在保持路径跟踪和任务进度的同时降低预测液体响应；
- 液体晃动是带状态记忆的动态问题，而非单纯平滑问题。

### 13.2 Slosh-Augmented Alpha-State Model

依次给出：

1. $\boldsymbol x_r=[p_x,p_y,\theta,v,s,\omega]$；
2. $\boldsymbol u=[a,\alpha,v_s]$；
3. 机器人动力学；
4. $\boldsymbol x_l=[\eta_x,\dot\eta_x,\eta_y,\dot\eta_y]$；
5. 液体模态动力学和 $a_x=a,a_y=v\omega$；
6. 完整增强状态。

### 13.3 Slosh-Aware MPCC Objective and Constraints

依次给出：

1. 局部参考曲线；
2. contour/lag error；
3. 路径进度奖励；
4. 曲率参考速度和 anti-creep；
5. 控制幅值与精确平滑项；
6. 液体模态代价；
7. 机器人/控制 bounds；
8. 可选 modal hard cap。

### 13.4 Predictive Slosh-Risk Reference Governor

依次给出：

1. 候选比例集合；
2. surrogate rollout；
3. 峰值风险；
4. 最大可接受比例选择；
5. 比例 rate limiting；
6. 最终风险复核；
7. 与内层 MPCC 的接口。

### 13.5 Receding-Horizon Execution

说明：

- acados SQP-RTI 实时求解；
- 只执行第一帧控制；
- $a_0/\alpha_0$ 积分为速度命令；
- 可选 delay alignment、terminal controller 和 safety gates 属于实物部署层。

### 13.6 Ablation Design

说明 `B0/B_smooth/B_slosh/B_ours` 如何分离：

- 普通平滑；
- 液体动态状态；
- 完整方法；
- 可选 hard cap；
- 可选 governor。

---

## 14. 推荐的 Introduction 贡献表述

可将论文贡献收敛为以下三点：

1. **Online slosh-aware local planning formulation.** We formulate open-liquid transport by a standard wheeled mobile robot as an online local planning problem with liquid dynamic memory, rather than treating slosh suppression as trajectory smoothing alone.
2. **Slosh-augmented MPCC with predictive reference adaptation.** We develop an alpha-state MPCC that jointly optimizes path progress, executable chassis motion, command smoothness, and low-order liquid modal response, together with a lightweight finite-horizon slosh-risk governor that adapts the reference speed without modifying the inner OCP structure.
3. **Target controlled real-system evidence.** After G0 release, we will separate liquid-state prediction from smooth-only motion generation through structured ablations and evaluate the actual liquid surface using external RGB measurements instead of relying solely on the controller's internal model proxy.

第 3 点是目标证据表述，不是当前完成状态；在带执行模型仿真、G0 held-out 和独立 RGB 实物试验完成前，不能改为完成时或现在时。若 governor 实验尚未完成或结果不足，也应把第 2 点拆开：正文主贡献只保留 slosh-augmented MPCC，governor 降级为 extension 或 future work，避免代码存在但证据不足时过度宣称。

---

## 15. 论文中必须保持的诚实边界

建议明确写出：

1. 低阶模态模型是控制友好的近似，不是在线 CFD/FEM/SPH；
2. 内部液体状态由模型和 odometry 激励传播，不是真实液面闭环观测；
3. modal hard cap 不等价于真实液面无溢出保证；
4. governor 使用简化短时 rollout，不是完整鲁棒 MPC 或形式化 reference governor 安全证明；
5. 当前主线关注给定安全路径附近的在线规划控制，不把完整动态避障和同伦推理作为贡献；
6. 当前已有 `d_c` 预计/实测审计、统一执行参考模型、预计发布时间 typed 接线，以及本周期新命令进入双通道 buffer 的 C++ 参考动力学、数值一致的 CasADi C 转移核和独立可求解 acados capsule；但默认估计仍关闭，新 capsule 尚未进入在线 factory/context，formal cost/parameters 和 terminal gate 尚未完成，Scout 执行参数也未冻结，不能宣称已经适配实物；
7. 当前缺少独立 odom/TF watchdog、solver deadline、最终命令无条件硬包络、driver 命令超时/确认和急停制动动态合同，正式实物闭环仍是 G0 NO-GO；
8. 当前唯一 ROS 命令事务已闭合，但 receipt 仍不是 CAN/底盘 ACK；runner 配置和路径版本也尚未形成单一 typed/epoch 合同；
9. 实物评价必须同时报告任务时间、路径误差、命令平滑性、求解耗时和外部液面指标，避免通过停车或全程低速获得表面上的降晃结果。

---

## 16. 公式与代码对应表

| 方法内容 | 当前代码 |
|---|---|
| alpha-state / slosh 增强动力学 | `tools/codegen/acados/spmpc_acados_model.py` |
| contour/lag、anti-creep、控制和 slosh cost | `tools/codegen/acados/spmpc_acados_cost.py` |
| box constraints 和 modal hard cap | `tools/codegen/acados/spmpc_acados_constraints.py` |
| SQP-RTI、ERK、HPIPM codegen | `tools/codegen/acados/generate_spmpc_acados.py` |
| acados ABI/capsule | `src/solver/acados/generated_solver.cpp` |
| acados 参数注入与结果解码 | `src/solver/acados/stage_parameter_builder.cpp`, `src/solver/acados/solution_decoder.cpp` |
| warm-start 回退与连续 MPCC backend | `src/warm_start/warm_start_policy.cpp`, `src/solvers/continuous_mpcc_solver_acados.cpp` |
| 液体离散模型和高度代理 | `src/dynamics/slosh_dynamics.cpp` |
| odometry/IMU 驱动的液体状态传播 | `src/estimation/slosh_observer_bank.cpp` |
| 预测晃液风险 governor 与周期编排 | `src/core/slosh_risk_governor.cpp`, `src/controller/speed_reference_controller.cpp` |
| 双通道执行合同、增广状态和 history prediction | `src/runtime/execution_prediction/execution_model.cpp`, `command_history_buffer.cpp`, `execution_state_predictor.cpp` |
| delay-augmented solver horizon 参考转移 | `src/solver/delay_augmented/phase_rejoin_dynamics.cpp`, `include/spmpc_local_planner/solver/api/execution_horizon_context.h` |
| delay-augmented CasADi 离散转移与一致性生成 | `tools/codegen/acados/spmpc_delay_augmented_phase_model.py`, `tools/codegen/acados/generate_delay_augmented_phase_transition.py`, `generated/casadi/` |
| 独立 delay-augmented DISCRETE acados capsule 与 capability gate | `tools/codegen/acados/generate_delay_augmented_phase_acados.py`, `src/solver/acados/delay_augmented_phase_solver.cpp`, `generated/acados/spmpc_delay_augmented_phase_solver_manifest.h` |
| 预计发布时间、实际 `d_c` 和 deadline 审计 | `src/runtime/timing/publish_latency_model.cpp`, `src/controller/control_cycle_engine.cpp` |
| 最终命令事务、receipt 与提交时序 | `src/controller/command/command_pipeline.cpp`, `src/controller/command/publication_transaction.cpp`, `src/controller/control_cycle_engine.cpp` |
| solver I/O 与 backend 权威接口 | `include/spmpc_local_planner/solver/api/solver_input.h`, `solver_output.h`, `solver.h` |
| Phase solver 结果窄适配 | `src/controller/phase_solve_adapter.cpp`, `include/spmpc_local_planner/phase_rejoin/types.h` |
| 主消融参数 | `config/planner/variants.yaml` |
| 容器和液体参数 | `config/containers/tube_default.yaml` |
| 底盘运动约束 | `config/platforms/scout_mini.yaml` |

---

## 17. 后续修改检查清单

每次修改方法或论文公式后至少检查：

- [ ] 状态维度是否仍为 B0 6D、slosh 10D；
- [ ] 控制是否仍为 `[a, alpha, v_s]`；
- [ ] `omega` 是否仍是状态、`alpha` 是否仍是控制；
- [ ] contour/lag 符号是否与 CasADi 实现一致；
- [ ] cost 是否仍包含曲率参考速度和 anti-creep；
- [ ] 是否错误地把 stage-0 连续性写成全 horizon 的完整 `Delta u`；
- [ ] `eta_dot_ref` 是否仍为 `omega_n * eta_ref`；
- [ ] hard constraint 是否仍为 modal-only 且 stage 0 放宽；
- [ ] governor 是否仍只修改 `v_ref_current`；
- [ ] governor 最终 rate-limited beta 是否仍会重新 rollout；
- [ ] governor 与 inner OCP 是否使用同一高度口径；
- [ ] delay compensation 是否默认关闭、正式实验是否明确报告模式；
- [ ] 是否把零延迟理想执行 proxy 误写成 Scout 执行延迟验收；
- [ ] `d_c` 和本周期新命令的双通道因果传播是否已进入正式模型；
- [ ] odom/TF watchdog、solver deadline、最终硬包络和 driver 停车/确认合同是否已独立验收；
- [ ] `ControlCycleEngine::step()` 是否原子返回唯一 `u_pub`、审计和 history event，ROS 是否只做发布；
- [ ] 实物 session 是否来自 typed config、C++ preflight 和 immutable manifest，而不是 Shell 参数拼接；
- [ ] 全链是否使用同一个 `reference_id/reference_epoch` 并原子复位 phase/goal/progress/warm-start；
- [ ] RGB 是否仍只用于外部评价而未进入闭环；
- [ ] 当前实验是否足以支撑 governor 作为正式贡献；
- [ ] 是否误把 Map-vref、obstacle、homotopy 或工程安全门控写成第一版核心方法。
