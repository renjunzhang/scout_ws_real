# 当前 MPC 数学化架构总结（含通用 MPC 对比）

本文基于当前代码实现，给出可直接对应到工程的数学表达。

## 1) 当前项目 MPC 的优化问题

设预测步长为 \(N\)，状态维度 \(n_x=8\)，控制维度 \(n_u=2\)。

- 状态：
\[
\mathbf x_k=
\begin{bmatrix}
 e_{l,k}&e_{c,k}&e_{\theta,k}&v_k&\eta_{x,k}&\dot\eta_{x,k}&\eta_{y,k}&\dot\eta_{y,k}
\end{bmatrix}^\top
\]
- 控制：
\[
\mathbf u_k=
\begin{bmatrix}a_k&\omega_k\end{bmatrix}^\top
\]

决策变量采用 OSQP 结构化堆叠：
\[
\mathbf z=[\mathbf x_0,\mathbf u_0,\mathbf x_1,\mathbf u_1,\dots,\mathbf x_{N-1},\mathbf u_{N-1},\mathbf x_N]
\]

QP 形式：
\[
\min_{\mathbf z}\ \frac12\mathbf z^\top \mathbf P\mathbf z+\mathbf q^\top\mathbf z
\quad
\text{s.t.}\quad
\mathbf l\le \mathbf A\mathbf z\le \mathbf u
\]

## 2) 运动学/动力学模型（项目特化）

### 2.1 基础 Frenet 跟踪子系统（4 维）

离散更新：
\[
\begin{aligned}
e_{l,k+1}&=e_{l,k}+\Delta t\,(v_k-v_{\text{path},k})\\
e_{c,k+1}&=e_{c,k}+\Delta t\,v_k e_{\theta,k}\\
e_{\theta,k+1}&=e_{\theta,k}+\Delta t\,(\omega_k-\kappa_k v_k)\\
v_{k+1}&=v_k+\Delta t\,a_k
\end{aligned}
\]

### 2.2 液体晃动增广子系统（4 维）

晃动状态定义：
\[
\mathbf x^{\text{slosh}}_k=
\begin{bmatrix}
\eta_{x,k}&\dot\eta_{x,k}&\eta_{y,k}&\dot\eta_{y,k}
\end{bmatrix}^\top
\]
其中：
- \(\eta_x,\eta_y\)：主模态位移（不是最终液面高度）
- \(\dot\eta_x,\dot\eta_y\)：对应模态速度

离散模型：
\[
\mathbf x^{\text{slosh}}_{k+1}=\mathbf A_s\mathbf x^{\text{slosh}}_k+\mathbf B_s
\begin{bmatrix}
a_k\\a_{y,k}
\end{bmatrix},
\quad a_{y,k}\approx v_k\omega_k
\]

因此该项目是“Frenet 跟踪 + 晃动线性子系统”增广模型，并在 \(a_y=v\omega\) 处发生耦合。

## 3) 代价函数（项目特化）

可写为：
\[
J=\sum_{k=0}^{N}\left(
\mathbf x_k^\top\mathbf Q_k\mathbf x_k+\mathbf q_k^\top\mathbf x_k
\right)
+\sum_{k=0}^{N-1}\left(
\mathbf u_k^\top\mathbf R\mathbf u_k+\mathbf r_k^\top\mathbf u_k
\right)
+\sum_{k=0}^{N-1}\Delta\mathbf u_k^\top\mathbf R_\Delta\Delta\mathbf u_k
\]

其核心项包括：
- 跟踪项：\(e_l,e_c,e_\theta,v\)；
- 控制项：\(a,\omega\)；
- 控制变化率项：\(\Delta a,\Delta\omega\)；
- 液体软代价项：
\[
J_{\text{slosh}}=Q_{\text{slosh},\eta}(\eta_x^2+\eta_y^2),
\quad
Q_{\text{slosh},\eta}=Q_{\text{slosh}}\cdot h_{\text{coeff}}^2
\]

## 4) 约束系统（项目特化）

### 4.1 等式约束

- 初值约束：\(\mathbf x_0=\mathbf x_{\text{meas}}\)
- 动力学约束：\(\mathbf x_{k+1}=\mathbf A_k\mathbf x_k+\mathbf B_k\mathbf u_k+\mathbf c_k\)

### 4.2 不等式约束

- 状态边界：\(v_{\min}\le v_k\le v_{\max}\)
- 控制边界：\(|a_k|\le a_{\max},\ |\omega_k|\le \omega_{\max}\)
- 变化率边界（可开关）：
\[
|\omega_k-\omega_{k-1}|\le \alpha_{\max}\Delta t,
\quad
|a_k-a_{k-1}|\le j_{\max}\Delta t
\]
- 液体盒约束（可开关）：
\[
-\bar\eta\le \eta_{x,k}\le \bar\eta,
\quad
-\bar\eta\le \eta_{y,k}\le \bar\eta
\]

## 5) 回答核心问题：没有“液面约束层”会怎样？

“液面约束层”可分两层看：

1. **硬约束层**（box constraint）关闭：
   - 仅移除 \(\eta_x,\eta_y\) 的不等式边界；
   - 若软代价仍在，抑制变软，不保证峰值不越界。

2. **软代价层**（\(Q_{\text{slosh},\eta}\)）也关闭：
   - 优化器不再主动惩罚晃动；
   - 晃动状态仅作为“随动动态变量”存在，对目标函数无直接影响；
   - 此时系统行为近似回到“纯路径跟踪 MPC”（只管跟踪与控制平滑/边界）。

3. 若进一步不注入 slosh 模型（回到 4 维状态）：
   - 即恢复无增广的标准 Frenet 跟踪结构。

## 6) 通用 MPC 与本项目 MPC 的数学化对比

### 6.1 通用 MPC（抽象）

\[
\begin{aligned}
\min_{x_{0:N},u_{0:N-1}}\ &\sum_{k=0}^{N-1}\ell(x_k,u_k)+\ell_f(x_N)\\
\text{s.t. }&x_{k+1}=f(x_k,u_k),\\
&g(x_k,u_k)\le 0,\ h(x_N)\le 0
\end{aligned}
\]

### 6.2 本项目 MPC（工程落地）

- 采用**线性化/离散化后 QP**（OSQP）；
- 状态是 **8 维增广 Frenet+slosh**；
- 约束是“动力学等式 + 速度/角速度/加速度/变化率边界 + 可选 slosh 盒约束”；
- 目标是“路径跟踪 + 控制平滑 + 可选晃动软抑制”。

即：本项目是通用 MPC 在移动机器人路径跟踪场景下的一个“结构化 QP 特例”，而非自由空间 NMPC。
