# Phase-Rejoining Residual MPC：Methods 章节组织思路

> **建议方法名：**Phase-Rejoining Residual MPC（PR-RMPC）。
>
> **一句话核心思想：**在线控制可以在冻结的离线防晃名义序列附近做有界修正，但候选相位必须与当前双通道执行状态兼容，预测轨迹必须在整个延迟增广时域内保持执行兼容，并在终端进入经验重接 gate；否则只允许使用冻结的有界恢复反馈或 fail-closed 零命令。
>
> **命名边界：**当前 22 维在线问题采用相位索引的名义位置、航向、进度、速度、液体和命令队列跟踪，没有 contour/lag 几何误差。因此正文不得再称 S-MPCC、residual MPCC 或 online local planning。

## 0. 论文应该主张什么

本文的方法贡献不是重新提出一般 MPCC，而是把“在线纠偏是否仍能接回离线防晃尾段”写成一个可执行、可审计的闭环：

```text
冻结路径
→ 基于延迟动力学 rollout 的参数化速度剖面优化
→ 完整名义尾段＋逐相位经验 gate／B_exec／恢复反馈合同
→ source-time 状态对齐与最终发布命令历史
→ 邻近相位选择＋完整预测域可达 witness
→ 22 维 delay-augmented residual MPC
→ residual／recovery／fail-closed 监督
→ 唯一最终命令事务与发布回执
```

Methods 应围绕以下四点展开：

1. 线速度和角速度命令具有不同延迟，在线决策必须从预计发布时刻、由最终发布历史构造的 pending-command 状态开始传播；
2. 相位只能在时钟邻域内单调选择，并且每周期只选择一次、求解一次；
3. 9 维经验 gate 与 14 维执行兼容集共同限制重接，但它们只是冻结数据上的经验 admission rule，不是安全证书；
4. 求解器结果、冻结恢复反馈和 fail-closed 零命令共用同一个最终发布事务，预测历史只记录真正交给 transport 的命令。

本文只处理冻结路径、冻结容器/模型和小幅偏离。当前方法没有在线避障、局部重规划或自由时间缩放；新障碍或合同失效时应终止 trial，而不是宣称控制器在线绕障。

![Phase-Rejoining Residual MPC 方法结构](assets/figures/phase_rejoining_method_structure_zh.svg)

**图 1　方法结构。**图中的 22 维在线 OCP、完整预测域执行兼容约束、经验 gate、恢复反馈和最终发布事务必须与下文使用同一套符号与边界。

## 1. III-A：问题定义与符号

### 1.1 机器人—液体状态

令控制器内部使用的 9 维机器人—液体状态为

$$
\chi_k=
[p_{x,k},p_{y,k},\psi_k,v^r_k,\omega^r_k,
\eta_{x,k},\dot\eta_{x,k},\eta_{y,k},\dot\eta_{y,k}]^\top
\in\mathbb R^9,
$$

其中 $v^r,\omega^r$ 是延迟执行模型的 realized actuator outputs，$z_k^\ell=[\eta_x,\dot\eta_x,\eta_y,\dot\eta_y]^\top$ 是控制器内部的一阶模态液体状态。外部仿真的 liquid truth、RGB 液面或实物液面测量均不进入在线控制律，只用于独立评价或离线 recovery 标签。

给定冻结路径 $\gamma(s)$ 和离线名义序列 $\bar{\mathcal A}$，在线目标不是重新生成路径，而是在邻近相位 $j$ 上求一个有限 residual，使机器人纠偏后仍满足冻结尾段的经验重接条件。

### 1.2 “可重接”的准确含义

本文使用的“可重接”只表示：在冻结的数据生成合同下，经验分类规则根据完整尾段 recovery rollout 标签，将某一相位误差判为可以接回。该判断允许出现 held-out false accept（gate 接受、但独立 rollout 标签为未恢复），并不证明该次在线状态必然恢复。它不等价于：

- 当前液面已经很低；
- 对所有扰动都能恢复；
- 鲁棒控制不变集、funnel 或 viability kernel；
- recursive feasibility 或形式化安全证书。

论文中统一使用 **empirical phase-indexed recovery admission rule** 或 **empirical rejoin gate**，避免使用 certified recoverable set。

## 2. III-B：离线名义序列与经验恢复合同

### 2.1 参数化路径速度剖面优化

代码合同名仍可保留 `OfflineSloshOCP`，但论文必须将其准确描述为：

> a low-dimensional, path-indexed speed-profile optimization using delay-aware dynamic rollout

当前生成器默认使用 $R=8$ 个路径索引速度缩放节点

$$
\lambda=[\lambda_1,\ldots,\lambda_R],
\qquad 0.55\le\lambda_r\le1.05,
$$

沿路径对 $\lambda(s)$ 线性插值，并乘到由最大速度、曲率横向加速度限制和剩余制动距离共同给出的基准速度上。角速度由冻结的路径曲率前馈、航向反馈和横向误差反馈生成，进度率由 realized 线速度在路径切向上的投影给出。也就是说，角速度序列和完整控制序列不是独立优化变量。

每个候选 $\lambda$ 都通过与在线控制器同语义的 delay-augmented dynamics rollout。下式中的液面指标来自该离线生成器内部的低阶模型，不是独立 plant 或实物液面真值。默认使用有界 Powell 搜索，目标为

$$
\begin{aligned}
J_{\mathrm{off}}(\lambda)=
&\;6\!\times\!10^5 h_{\mathrm{rms}}^2
+2\!\times\!10^5 h_{95}^2
+40d_{\mathrm{path,max}}^2\\
&+60\|e_{p,f}\|^2+4e_{\psi,f}^2+0.015T
+0.01\sum_{r=1}^{R-1}(\lambda_{r+1}-\lambda_r)^2.
\end{aligned}
$$

路径偏离、终点位置和终点航向条件在优化后由完整 rollout 验收。随后执行受发布速率限制的减速，并保留真实的零发布命令保持段，使命令队列、底盘输出和液体状态继续演化至停稳。

这是一种低维参数化 OCP/轨迹优化实现，而不是全状态—全控制序列的直接配点。正文应描述决策变量、固定跟踪律、Powell 搜索和后验验收，不应仅写“求解 OfflineSloshOCP”让读者误解为 direct transcription。

### 2.2 冻结 artifact

离线和 recovery 流水线最终生成逐相位 artifact

$$
\bar{\mathcal A}=\left\{
(\bar t_i,\bar x^{\mathrm{aug}}_i,\bar q_i,
\bar u_i^{\mathrm{pub}},\bar\kappa_i,
\rho_i,\beta_i)
\right\}_{i=0}^{M},
$$

其中 $\bar x_i^{\mathrm{aug}}\in\mathbb R^{22}$ 是完整延迟增广名义状态，$\bar q_i$ 是 published-command-rate 输入，$\bar u_i^{\mathrm{pub}}$ 是名义发布命令，$\bar\kappa_i$ 提供恢复反馈的名义命令中心，$\rho_i\in\mathbb R^9_{>0}$ 是经验 gate 半径，$\beta_i\in\mathbb R^{14}_{>0}$ 是执行兼容边界。

artifact 身份固定路径与坐标系、$\Delta t$、执行与低阶液体模型、状态/参数 schema、完整减速—沉降—零命令尾段、recovery policy 以及 $\rho_i$、$\beta_i$ 的定义。容器与 fill、生成/运行软件、六个 condition 配置和可执行文件在正式实验前另行冻结。任一方法定义改变都必须重新生成 artifact 并重新冻结实验；Methods 不展开逐文件 hash 或 session manifest 的工程细节。

### 2.3 冻结的有界恢复反馈

V3 artifact 使用的是冻结机器人状态反馈，而不是逐相位固定动作。令名义坐标系下的误差为

$$
\begin{bmatrix}e_\parallel\\e_\perp\end{bmatrix}
=
\begin{bmatrix}
\cos\bar\psi_j&\sin\bar\psi_j\\
-\sin\bar\psi_j&\cos\bar\psi_j
\end{bmatrix}
\begin{bmatrix}\bar p_{x,j}-p_x\\\bar p_{y,j}-p_y\end{bmatrix},
\qquad
e_\psi=\operatorname{wrap}(\bar\psi_j-\psi).
$$

冻结反馈为

$$
\delta v=\operatorname{clip}
(k_\parallel e_\parallel+k_v(\bar v_j-v^r),
-\delta v_{\max},\delta v_{\max}),
$$

$$
\delta\omega=\operatorname{clip}
(k_\perp e_\perp+k_\psi e_\psi
+k_\omega(\bar\omega_j-\omega^r),
-\delta\omega_{\max},\delta\omega_{\max}),
$$

$$
u_{\mathrm{rec}}^{\mathrm{des}}=
\operatorname{clip}_{\mathcal U}
\left(\bar\kappa_j+[\delta v,\delta\omega]^\top\right).
$$

当前冻结数值为

$$
(k_\parallel,k_\perp,k_\psi,k_v,k_\omega)
=(0.80,1.20,1.50,0.40,0.40),
$$

$$
(\delta v_{\max},\delta\omega_{\max})=(0.08,0.20).
$$

命令 envelope 为

$$
\mathcal U=[0,0.80]~\mathrm{m/s}\times[-1.20,1.20]~\mathrm{rad/s}.
$$

恢复命令还要相对于当前可信 pending queue 的尾项执行与 OCP 相同的发布速率事务：

$$
|u^{\mathrm{rec}}_{v}-b^v_{4}|\le a_{\max}\Delta t,
\qquad
|u^{\mathrm{rec}}_{\omega}-b^\omega_{6}|
\le\alpha_{\max}\Delta t.
$$

该反馈不使用液体状态；液体信息只参与经验 gate 和离线 recovery 成败标签。论文可称其为 **frozen bounded recovery feedback policy** $\kappa_j(e_r)$，不可再写“保存的固定动作”。

### 2.4 经验 gate 与执行兼容集

9 维误差按 wrapped yaw 构造为

$$
e_i^{(9)}=\big[
p_x-\bar p_{x,i},p_y-\bar p_{y,i},
\operatorname{wrap}(\psi-\bar\psi_i),
v^r-\bar v_i^r,\omega^r-\bar\omega_i^r,
z^\ell-\bar z_i^\ell
\big]^\top,
\qquad
m_i(e)=\sum_{r=1}^{9}\left(\frac{e_r}{\rho_{r,i}}\right)^2.
$$

经验 gate 定义为

$$
\widehat{\mathcal R}^{\mathrm{emp}}_i
=\{e^{(9)}:m_i(e)\le1\}.
$$

执行误差只包含 realized outputs 和两路 pending queues：

$$
e_i^{\mathrm{exec}}=
[v^r-\bar v_i^r,\omega^r-\bar\omega_i^r,
(b^v-\bar b_i^v)^\top,(b^\omega-\bar b_i^\omega)^\top]^\top
\in\mathbb R^{14},
$$

$$
\mathcal B_i^{\mathrm{exec}}
=\{e^{\mathrm{exec}}:
|e^{\mathrm{exec}}_r|\le\beta_{r,i},\;r=1,\ldots,14\}.
$$

每条 recovery 样本都从被扰动相位开始，在闭环中用同一冻结反馈和发布速率事务依次跟踪后续每个名义尾段相位，并在最后相位继续完成固定 settling tail；不能写成“执行一次 recovery action 后开环恢复 nominal tail”。recovered 标签由独立 plant rollout 的以下八项条件共同定义：

$$
\begin{gathered}
d_{p,\max}\le0.25\,\mathrm{m},\qquad
d_{\psi,\max}\le0.35\,\mathrm{rad},\qquad
h_{\max}^{\mathrm{true}}\le2.3\,\mathrm{mm},\\
d_{p,f}\le0.15\,\mathrm{m},\qquad
d_{\psi,f}\le0.15\,\mathrm{rad},\qquad
|v_f|\le0.03\,\mathrm{m/s},\\
|\omega_f|\le0.05\,\mathrm{rad/s},\qquad
h_f^{\mathrm{true}}\le1.5\,\mathrm{mm}.
\end{gathered}
$$

这里的 independent-plant liquid truth 只用于成败标签，不可成为 gate feature 或恢复反馈输入；拟合后才得到的 $\mathcal B_i^{\mathrm{exec}}$ 也不参与 recovered 标签，否则会形成循环定义。

数据以完整 recovery rollout 为统计单位，并按 seed 和 rollout ID 分成互斥的 fit、tune 和 held-out。当前拟合过程仅用 recovered fit rollout：9 维基础半径取 $\sqrt 9$ 乘各坐标最大绝对误差，14 维基础边界取各坐标最大绝对误差；tune 数据只能选择一个全局 shrinkage，held-out 只评价一次。当前工具分别报告 recovered 样本 coverage、未恢复样本中的 false-accept rate $\mathrm{FA}/(\mathrm{FA}+\mathrm{TR})$ 及二者的 Wilson 区间，同时以自己的 accepted-sample 分母报告 conditional false-safe $\mathrm{FA}/(\mathrm{TA}+\mathrm{FA})$ 及其独立 Wilson 区间；不得混用两个分母或套用另一比率的区间。若 $\mathrm{TA}+\mathrm{FA}=0$，conditional false-safe 及其区间必须显式记为未定义，而不是写成 0。

## 3. III-C：source-time 对齐与 22 维执行增广模型

### 3.1 共同 publication epoch

令机器人/观察状态的 source stamp 为 $t_s$，控制周期开始为 $t_c$。控制器用冻结的发布延迟估计 $\widehat d_c$ 定义预计发布时刻

$$
t_p=t_c+\widehat d_c.
$$

机器人、内部液体和两路 pending-command 状态先用最终发布命令历史传播到同一个 $t_p$；随后用对齐后的机器人位置在冻结路径上做 monotone guarded projection，重新计算 progress 并写入 OCP 初态。progress 不是从 source epoch 作为独立状态随历史传播。若 source stamp、命令历史、预计发布 epoch 或执行合同不一致，则不允许复用另一 epoch 的增广状态，也不调用 recovery，而是 fail closed。

命令历史只记录最终 publication receipt 声明已经交给 transport 的 $u^{\mathrm{pub}}$，不记录原始求解器候选 $u^{\mathrm{sol}}$。因此所有延迟均从最终发布边界定义。

### 3.2 精确的 22 维状态和 3 维输入

当前冻结执行合同为 $\Delta t=1/30\,\mathrm{s}$、线速度延迟 $d_v=0.15\,\mathrm{s}$、角速度延迟 $d_\omega=0.22\,\mathrm{s}$。线/角 pending queue 分别有 5 和 7 个元素。在线求解器的精确状态顺序为

$$
\begin{aligned}
x_k^{\mathrm{aug}}=[
&p_x,p_y,\psi,v^r,s,\omega^r,
\eta_x,\dot\eta_x,\eta_y,\dot\eta_y,\\
&b^v_0,b^v_1,b^v_2,b^v_3,b^v_4,
b^\omega_0,b^\omega_1,b^\omega_2,b^\omega_3,
b^\omega_4,b^\omega_5,b^\omega_6]_k^\top
\in\mathbb R^{22}.
\end{aligned}
$$

这里没有额外的 $x^a$ 状态块；$v^r,\omega^r$ 本身就是执行器输出状态。输入为

$$
q_k=[a_k^{\mathrm{pub}},\alpha_k^{\mathrm{pub}},\nu_{s,k}]^\top
\in\mathbb R^3,
$$

其中 $a^{\mathrm{pub}},\alpha^{\mathrm{pub}}$ 是**新发布速度命令的变化率**，不是延迟后物理底盘的瞬时加速度。代码/artifact 中沿用字段名 `a`、`alpha`，论文符号必须保留 `pub` 上标以避免歧义。

第一拍和后续各拍的发布命令由 pending queue 尾项生成：

$$
u^{\mathrm{pub}}_{v,k}=b^v_{4,k}+a_k^{\mathrm{pub}}\Delta t,
\qquad
u^{\mathrm{pub}}_{\omega,k}=b^\omega_{6,k}
+\alpha_k^{\mathrm{pub}}\Delta t,
$$

$$
b^v_{k+1}=[b^v_{1,k},b^v_{2,k},b^v_{3,k},b^v_{4,k},
u^{\mathrm{pub}}_{v,k}],
$$

$$
b^\omega_{k+1}=[b^\omega_{1,k},\ldots,b^\omega_{6,k},
u^{\mathrm{pub}}_{\omega,k}].
$$

因此第一拍 residual 约束必须作用在 $b^v_{4,0}+a_0^{\mathrm{pub}}\Delta t$ 和 $b^\omega_{6,0}+\alpha_0^{\mathrm{pub}}\Delta t$ 上，不能写成 $v^r_0+a_0\Delta t$、$\omega^r_0+\alpha_0\Delta t$。

两路命令分别按 integer/fractional delay 在每个离散步内分段作用。设线、角 fractional event 与步长终点的排序并集为

$$
0=\tau_0<\tau_1<\cdots<\tau_L=\Delta t,
\qquad \delta_h=\tau_{h+1}-\tau_h.
$$

对于分段 $h$ 和通道 $r\in\{v,\omega\}$，令 $y_v=v^r$、$y_\omega=\omega^r$，$\mu_r(\widetilde u_{r,h})$ 表示冻结的增益—死区—饱和映射。代码使用如下分段末端输出：

$$
y_{r,h+1}=
\begin{cases}
\mu_r(\widetilde u_{r,h}), & \tau_r^{a}=0,\\
\mu_r(\widetilde u_{r,h})+
e^{-\delta_h/\tau_r^{a}}
\bigl(y_{r,h}-\mu_r(\widetilde u_{r,h})\bigr),
& \tau_r^{a}>0.
\end{cases}
$$

随后以该末端输出执行位姿更新，并以分段平均纵向加速度和末端横向加速度作为 ZOH 液体输入：

$$
\begin{aligned}
p_{x,h+1}&=p_{x,h}+\delta_h v^r_{h+1}\cos\psi_h,\\
p_{y,h+1}&=p_{y,h}+\delta_h v^r_{h+1}\sin\psi_h,\\
\psi_{h+1}&=\operatorname{wrap}(\psi_h+\delta_h\omega^r_{h+1}),\\
a_{x,h}&=\frac{v^r_{h+1}-v^r_h}{\delta_h},
\qquad a_{y,h}=v^r_{h+1}\omega^r_{h+1},\\
z^\ell_{h+1}&=A_{\ell,d}(\delta_h)z^\ell_h+
B_{\ell,d}(\delta_h)
\begin{bmatrix}a_{x,h}\\a_{y,h}\end{bmatrix}.
\end{aligned}
$$

$A_{\ell,d},B_{\ell,d}$ 由低阶液体连续模型的矩阵指数精确 ZOH 离散化得到；位姿则是实现中明确的分段末端 Euler 更新。所有 event segment 依次复合后再移动 pending queue，得到 $F_{\mathrm{exec}\text{-}\ell}$。因此文稿不将该转移表述为对连续位姿—执行器方程的精确积分；当 $\tau_r^a=0$ 时，$a_{x,h}$ 是代码中的有限差分分段平均量，不声称命令跳变处存在经典导数。

当前冻结 controller contract 的两路 $\tau_r^a$ 均为 0、正负方向增益均为 1、deadzone 均为 0，因此在线模型实际退化为带输出边界的双通道纯运输延迟。实现保留了一般一阶执行通道 schema，但 Methods 不应暗示当前在线求解器使用了非零执行器惯性；正式仿真的 independent plant 可保留非零 time constant，作为刻意的模型失配。

求解器内部 yaw 使用 continuous lift 保持动力学光滑，代价和 gate 的 yaw 误差仍使用 wrapped difference。

### 3.3 执行前沿与预测长度

当前合同对应线/角 integer delay steps 为 4/6，fractional delay 为约 $16.67/20.00\,\mathrm{ms}$；共同栅格化执行前沿为

$$
n_f=\max\left(
\left\lceil\frac{d_v}{\Delta t}\right\rceil,
\left\lceil\frac{d_\omega}{\Delta t}\right\rceil
\right)=7.
$$

液体重接窗口为 $N_\ell=3$，所以

$$
N=n_f+N_\ell=10,
$$

终端位于预计发布时刻后约 $333.3\,\mathrm{ms}$，也就是共同栅格化执行前沿后约 $100\,\mathrm{ms}$。这 10 步全部由决策相关的 22 维离散转移传播；前 7 步是显式执行延迟传播，不是 history-only 固定前沿，也不是额外的自由液体 preview。

## 4. III-D：相位候选选择与完整时域资格检查

### 4.1 时钟邻域与单调性

绝对 PhaseClock 给出 $i_{\mathrm{clock}}$。候选只来自局部窗口，并满足

$$
j\ge j_{\mathrm{prev}},
\qquad
j-i_{\mathrm{clock}}\le r_{\mathrm{lead}}.
$$

当前 $r_{\mathrm{lead}}=1$；常规候选窗口的 backward/forward radius 为 1/3，但 forward radius 仍受最多领先时钟一步的硬限制。允许候选暂时保持在已提交相位，禁止回退到已提交相位之前，也禁止累计超前、全局跳相或自由时间缩放。

### 4.2 先检查执行可行性，再做 9 维评分

对每个候选 $j$，先要求当前执行状态满足 $\mathcal B_j^{\mathrm{exec}}$。为了使“完整时域可执行”是闭合定义，令 $u^{\mathrm{pub}}_{r,-1}=b^r_{\mathrm{tail}}$ 为当前 pending queue 尾项，并定义发布命令 witness 集

$$
\begin{aligned}
\mathcal W_j(\widehat X)=\bigl\{U={}&(u^{\mathrm{pub}}_0,\ldots,u^{\mathrm{pub}}_{N-1}):\\
&u^{\min}_r\le u^{\mathrm{pub}}_{r,k}\le u^{\max}_r,\\
&|u^{\mathrm{pub}}_{r,k}-\bar u^{\mathrm{pub}}_{r,j+k}|\le\Delta u_r,\\
&|u^{\mathrm{pub}}_{r,k}-u^{\mathrm{pub}}_{r,k-1}|
\le \dot u_r^{\max}\Delta t,\\
&X^w_{0|j}=\widehat X,\quad
X^w_{k+1|j}=F_{\mathrm{pub}}(X^w_{k|j},u^{\mathrm{pub}}_k),\\
&x^{\mathrm{exec}}(X^w_{k|j})-
\bar x^{\mathrm{exec}}_{j+k}\in\mathcal B^{\mathrm{exec}}_{j+k},
\quad k=0{:}N,\\
&r\in\{v,\omega\},\quad k=0{:}N-1
\text{ for the command and transition constraints}
\bigr\}.
\end{aligned}
$$

$F_{\mathrm{pub}}$ 是将命令直接追加到两路 queue 后执行上述分段转移与 queue shift 的同一模型。因此当命令在后续步中移入 pending queue 的每一位置时，它都必须落入对应相位的 $\mathcal B^{\mathrm{exec}}$。实现分通道求解一维区间链，再对合成的二维 witness 做因果 rollout 复核。候选资格集为

$$
\mathcal I_{\mathrm{eligible}}=
\{j\in\mathcal I_{\mathrm{clock}}:\exists U\in\mathcal W_j(\widehat X)\}.
$$

只有通过上述 current＋full-horizon 资格过滤的候选，才用 9 维归一化误差评分

$$
S_j=w_pS_{p,j}+w_\psi S_{\psi,j}
+w_vS_{v,j}+w_\ell S_{\ell,j},
$$

并选择

$$
j^*=\arg\min_{j\in\mathcal I_{\mathrm{eligible}}} S_j.
$$

9 维量在这一步用于候选排序，而不是把全局最相似相位任意跳进来。相位选择完成后只构造一张参数图像并求解一次 OCP；不得求解多个候选再挑最好结果。

## 5. III-E：Delay-Augmented Residual MPC

### 5.1 名义相对代价，而非 contour/lag MPCC

给定已选相位 $j$，定义 wrapped yaw error

$$
e_{\psi,k}=\operatorname{atan2}
(\sin(\psi_k-\bar\psi_{j+k}),
\cos(\psi_k-\bar\psi_{j+k})).
$$

在线问题的 stage state cost 为

$$
\begin{aligned}
\ell_x=&\;
w_p\frac{(p_x-\bar p_x)^2+(p_y-\bar p_y)^2}{\sigma_p^2}
+w_\psi\frac{e_\psi^2}{\sigma_\psi^2}
+w_s\frac{(s-\bar s)^2}{\sigma_s^2}\\
&+w_v\frac{(v^r-\bar v^r)^2}{\sigma_v^2}
+w_\omega\frac{(\omega^r-\bar\omega^r)^2}{\sigma_\omega^2}\\
&+w_\eta\frac{(\eta_x-\bar\eta_x)^2
+(\eta_y-\bar\eta_y)^2}{\sigma_\eta^2}
+w_{\dot\eta}\frac{(\dot\eta_x-\dot{\bar\eta}_x)^2
+(\dot\eta_y-\dot{\bar\eta}_y)^2}{\sigma_{\dot\eta}^2}\\
&+w_{b_v}\frac{\|b^v-\bar b^v\|_2^2}{\sigma_v^2}
+w_{b_\omega}\frac{\|b^\omega-\bar b^\omega\|_2^2}{\sigma_\omega^2},
\end{aligned}
$$

输入 cost 为

$$
\ell_q=
w_a\frac{(a^{\mathrm{pub}}-\bar a^{\mathrm{pub}})^2}{\sigma_a^2}
+w_\alpha\frac{(\alpha^{\mathrm{pub}}-\bar\alpha^{\mathrm{pub}})^2}{\sigma_\alpha^2}
+w_{\nu_s}\frac{(\nu_s-\bar\nu_s)^2}{\sigma_{\nu_s}^2}.
$$

当前冻结归一化尺度为

$$
(\sigma_p,\sigma_\psi,\sigma_s,\sigma_v,\sigma_\omega)
=(0.15,1.0,0.10,0.80,1.20),
$$

$$
(\sigma_\eta,\sigma_{\dot\eta},
\sigma_a,\sigma_\alpha,\sigma_{\nu_s})
=(0.00275036585,0.0859380278,0.60,1.20,0.80).
$$

总目标为

$$
J=\sum_{k=0}^{N-1}(\ell_{x,k}+\ell_{q,k})+\ell_{x,N}.
$$

这里的位置项是对相位索引名义 $x/y$ 的欧氏距离，航向项是 wrapped yaw tracking，代码中不存在路径切/法向 contour–lag 分解。配置层残留的 `w_contour`、`w_lag` 只是历史参数别名，分别映射到 position/yaw weight，不得在论文中解释为 contour/lag cost。

### 5.2 硬约束

初态固定为对齐到 $t_p$ 的完整 22 维状态：

$$
x_{0|t}^{\mathrm{aug}}=\widehat x^{\mathrm{aug}}(t_p).
$$

每个 stage 满足离散动力学、输入范围和发布命令范围：

$$
x_{k+1|t}^{\mathrm{aug}}=
F_{\mathrm{exec-\ell}}(x_{k|t}^{\mathrm{aug}},q_{k|t}),
$$

$$
|a_k^{\mathrm{pub}}|\le0.60,
\qquad |\alpha_k^{\mathrm{pub}}|\le1.20,
\qquad 0\le\nu_{s,k}\le0.80,
$$

$$
0\le u^{\mathrm{pub}}_{v,k}\le0.80,
\qquad |u^{\mathrm{pub}}_{\omega,k}|\le1.20,
$$

$$
|u^{\mathrm{pub}}_{v,k}-\bar u^{\mathrm{pub}}_{v,j+k}|
\le\delta v_{\max},
\qquad
|u^{\mathrm{pub}}_{\omega,k}-\bar u^{\mathrm{pub}}_{\omega,j+k}|
\le\delta\omega_{\max}.
$$

完整 14 维 execution compatibility 不是只在 terminal 检查，而是在整个预测域内作为硬约束：

$$
e_{k|t}^{\mathrm{exec}}
\in\mathcal B_{j+k}^{\mathrm{exec}},
\qquad k=0,\ldots,N.
$$

对完整方法 C4，终端还满足 9 维经验 gate：

$$
m_{j+N}(e_{N|t}^{(9)})
\le1-\varepsilon_{\mathrm{num}},
\qquad \varepsilon_{\mathrm{num}}=10^{-5}.
$$

该 inner margin 只为给独立复核保留数值余量，不改变冻结的经验 gate 定义。当前 22 维问题没有在线路径 corridor hard constraint，也没有独立的全时域液面高度 hard constraint；液体误差进入 nominal-relative cost，9 维状态中的液体分量只在 terminal empirical gate 中作为硬 admission 条件。论文不得添加代码中不存在的约束。

### 5.3 求解与独立验收

当前 backend 使用 acados Full SQP、Gauss–Newton Hessian、`FUNNEL_L1PEN_LINESEARCH` 和最多 20 次 SQP 迭代。内层 QP 四项容差为 $10^{-9}$，外层 stationarity/equality/inequality/complementarity 验收阈值均为 $10^{-6}$。

acados 返回后还要独立执行：

1. 完整 stage/terminal 硬约束复算；
2. 用冻结的 C++ 22 维模型对求解控制序列做 causal rollout；
3. 检查求解状态与 causal rollout 的最大差异不超过 $10^{-6}$；
4. 从 causal rollout 取第一条 published command，而不是直接信任未经复算的 raw NLP 状态。

求解器在调用数值优化器之前发现的合同、参数图像或 warm-start 因果性错误，以及求解后的独立硬约束/causal audit 错误，归为 integrity failure。数值优化器确实被调用后因状态码、收敛或 KKT 验收返回失败，归为 optimization failure。只有后一类允许进入 recovery 资格判断。

## 6. III-F：监督器、恢复分支与最终发布事务

### 6.1 三类运行结果

监督器必须区分“优化失败”和“证据/合同失效”，不能把所有失败都交给 recovery：

| 条件 | 输出候选 | 相位提交 |
| --- | --- | --- |
| solver 成功，terminal empirical gate（C4）及 current/terminal $\mathcal B^{\mathrm{exec}}$ 通过，第一拍 residual 合同一致 | causal rollout 的 residual 第一拍 | 最终命令按原值成功发布后才允许 |
| solver 发生可恢复的 optimization failure，或 solver 成功但 terminal empirical/$\mathcal B^{\mathrm{exec}}$ admission 拒绝；同时 current empirical gate（C4）、current $\mathcal B^{\mathrm{exec}}$ 和 recovery rate contract 有效 | 冻结有界恢复反馈 $\kappa_j(e_r)$ | 不提交 residual 相位 |
| 状态/epoch/artifact/schema/hash/独立约束或 causal audit 等 integrity failure，或当前 gate／执行兼容／恢复事务无效 | 请求 $(0,0)$ fail-closed | 不提交 |

零命令是异常情况下的 fail-closed 请求，不应表述为已验证的防晃制动策略。若后续安全层改写已由预测验证的 residual 命令，该周期也不能提交相位。

### 6.2 C3/C4 的唯一消融差异

C3 与 C4 共用同一 22 维 Full SQP、同一 V3 artifact、同一 residual authority、同一全时域 $\mathcal B^{\mathrm{exec}}$、同一 recovery policy 和同一最终发布事务。唯一差异是对同一 9 维 empirical gate metric 采用 **enforce** 还是 **monitor-only**，且该差异同时作用于两个 admission 位置：

- C4：在 terminal residual admission 中将 gate 作为 NLP 硬约束并在求解后复核；在 recovery admission 中还必须通过 current empirical gate；
- C3：在 terminal 和 current 两处都计算、记录同一 metric，但不用它拒绝 residual 或 recovery。

因此 C3 在论文中的显示名统一为 **GateMonitorPR-RMPC**（residual MPC with empirical-gate monitoring only）。`residual_no_gate` 只是为兼容已冻结 runner/config 保留的 legacy mode 字符串；它不表示 $\mathcal B^{\mathrm{exec}}$、recovery 或整个 empirical metric 被关闭。C3 不应称为“terminal-gate-only ablation”，也不能写成“完全没有 recovery/$\mathcal B^{\mathrm{exec}}$ 的普通 residual controller”。

### 6.3 唯一最终命令出口

每周期的完整算法为：

```text
1. 读取 source-stamped 机器人/内部液体状态和最终发布命令历史。
2. 估计预计发布时刻 t_p，并把 22 维执行状态对齐到共同 epoch。
3. 在 PhaseClock 邻域内用 current B_exec 和 full-horizon witness 过滤候选。
4. 用 9 维归一化误差选定一个相位 j，只构造一次参数图像。
5. 求解 N=10 的 delay-augmented residual MPC，并独立复核 KKT、约束和 causal rollout。
6. 监督器在 residual、冻结 recovery feedback 和 fail-closed 零命令之间选择。
7. 先执行 safety supervisor，再完成 safety／terminal／phase／solver 的冻结优先级 arbitration；随后由唯一 `PublicationTransaction` 内的 execution-contract finalizer／limiter 生成最终命令并交给 sink。
8. delivered receipt 的时间戳和命令有限时，把 receipt 声明的实际命令和时间写回 history；只有 receipt 完全一致且命令未被改写时才提交相位。
```

当前 publication receipt 只证明命令已经交给配置的 ROS/transport 边界，不是 Scout CAN 或底盘真实执行回执。论文实物章节必须将此限制写清楚，不能把 ROS transport receipt 称为 actuator acknowledgement。

## 7. 写作与证据边界

### 7.1 截至 2026-08-24 的状态

| 层级 | 当前可说 | 当前不可说 |
| --- | --- | --- |
| 方法实现 | 22 维 decision-dependent delay-augmented OCP、完整预测域 $\mathcal B^{\mathrm{exec}}$、held-out empirical gate、冻结 recovery feedback、Full SQP/KKT/causal audit 和唯一发布事务已形成 development 闭环 | 不得继续写旧 B0/history-only、缺失 $\mathcal B^{\mathrm{exec}}$ 或固定 recovery action |
| 正式仿真资格 | 路径、Plant、六条件、seeds、运行顺序和统计口径均已有冻结入口 | 正式 trials 执行数仍为 0/96；本轮必要的控制逻辑与统计修改完成后需重新冻结一次 |
| 实物资格 | 代码接口和标定清单存在 | Scout/Nokov/IMU/RGB 的 G0、shadow、真实参数冻结和正式实物实验均未开始，real-robot enforce 仍为 NO-GO |

本轮修改完成后只做一次新的 clean build、配置冻结和 readiness 检查，然后直接进入正式仿真；不再继续扩展平台合同，也不根据正式结果回调参数。

### 7.2 论文中的术语替换表

| 旧写法 | 统一替换为 |
| --- | --- |
| Phase-Rejoining Residual S-MPCC | Phase-Rejoining Residual MPC |
| contour/lag tracking | phase-indexed nominal $x/y$ and yaw tracking |
| OfflineSloshOCP（不解释变量） | parameterized path-indexed speed-profile optimization with delay-aware rollout |
| 固定保存动作 $u_{\mathrm{rec}}(i)$ | frozen bounded recovery feedback $\kappa_i(e_r)$ |
| $a,\alpha$ 是底盘加速度 | $a^{\mathrm{pub}},\alpha^{\mathrm{pub}}$ 是 published-command rates |
| terminal-only $\mathcal B^{\mathrm{exec}}$ | current＋full-horizon prefilter＋all-stage/terminal $\mathcal B^{\mathrm{exec}}$ |
| recovery funnel/certificate | empirical recovery admission rule |
| receipt 等于实车执行回执 | ROS/transport publication receipt |

### 7.3 配图合同

方法图采用本文件第 0 节的八段流程。图中文字必须保留“参数化速度剖面优化”的限定，并明确 22D OCP、full-horizon $\mathcal B^{\mathrm{exec}}$、bounded recovery feedback 和 publication receipt 边界；图本身不能被用作 formal/real-robot 资格证据。

## 8. 实现核对索引（不进入论文正文）

以下路径均相对于 `src/scout_apps/control/spmpc_local_planner/`：

- 22D state/control/parameter layout：`tools/codegen/acados/spmpc_delay_augmented_phase_model.py`
- 当前生成 solver identity 与 $N=10=7+3$：`generated/acados/spmpc_delay_augmented_phase_solver_manifest.h`
- published-command-rate 语义和 queue-tail 第一拍：`include/spmpc_local_planner/solver/delay_augmented/phase_rejoin_dynamics.h`、`src/solver/delay_augmented/phase_rejoin_dynamics.cpp`
- phase window、current $\mathcal B^{\mathrm{exec}}$ 和候选排序：`src/phase_rejoin/phase_candidate_selector.cpp`
- full-horizon causal witness：`src/phase_rejoin/execution_horizon_compatibility_gate.cpp`
- 9D gate / 14D box：`src/phase_rejoin/empirical_recovery_gate.cpp`、`src/phase_rejoin/execution_compatibility_gate.cpp`
- frozen recovery feedback：`src/phase_rejoin/bounded_tracking_recovery_policy.cpp`
- residual/recovery/stop 分支：`src/phase_rejoin/phase_rejoin_coordinator.cpp`
- KKT、约束和 causal admission：`src/solvers/delay_augmented_phase_online_solver.cpp`
- 唯一发布事务和 receipt history：`src/controller/command/publication_transaction.cpp`
- 参数化离线速度剖面生成：`tools/simulation/generate_offline_slosh_ocp_plan.py`
- fit/tune/held-out gate：`tools/simulation/fit_phase_rejoin_recovery.py`
