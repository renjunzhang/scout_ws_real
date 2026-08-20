# Phase-Rejoining Residual S-MPCC：Methods 章节组织思路

> **一句话核心 idea：**在线 residual 可以纠偏，但只有在真实执行延迟后仍能接回离线防晃尾段的残差修正才被接受。

> **实现载体：**在共同前沿后约 100 ms 的联合终端，同时检查相位索引 9 维经验 gate 和执行状态兼容集。

> **当前判断：**这是论文目标方法；双通道前沿 B0、正式 OfflineSloshOCP artifact、held-out gate 和执行兼容集尚未闭合，当前实物 **enforce** 仍为 NO-GO。

## 1. III-A：问题与核心思路

离线防晃依靠“前段激励—后段抵消”。普通 MPCC 为减小当前路径误差而临时转向或变速时，可能改变液体相位，破坏后续抵消动作。因此本文真正检查的不是“100 ms 后液面是否最低”，而是：

> 这次修正真实作用到 Scout 后，是否还保留接回离线尾段并完成液体抵消的可能？

~~~text
完整离线序列＋逐相位 gate/执行兼容集
→ source-time 对齐＋邻近相位选择
→ 双通道增广传播＋前沿后约 100 ms 联合终端
→ 终端 gate
→ residual 第一拍 / 保存动作 / 零命令统一发布
~~~

![执行前沿对齐的相位重接残差 S-MPCC 方法结构](assets/figures/phase_rejoining_method_structure_zh.svg)

**图 1　方法结构。**核心创新只有“用尾段重接条件约束 residual 修正”；双通道执行对齐和 fail-closed 监督器是它成立所必需的支撑机制。

**范围：**只处理冻结路径、冻结容器和小幅偏离。当前没有在线避障；MBF 只能在 trial 前生成并冻结路径，新障碍出现时终止 trial。

## 2. III-B：共同模型与离线 artifact

机器人—液体状态、基础 OCP 状态和执行增广状态分别为

$$
\chi=[p_x,p_y,\psi,v,\omega,\eta_x,\dot\eta_x,\eta_y,\dot\eta_y]\in\mathbb R^9,
$$

$$
X=[p_x,p_y,\psi,v,s,\omega,z^\ell],\qquad
X^{\mathrm{aug}}=[X,b^v,b^\omega,x^a].
$$

$q=[a,\alpha,v_s]$ 是 OCP 输入；$u^{\mathrm{sol}}$ 是候选速度命令；$u^{\mathrm{pub}}$ 是经过监督、限幅和安全层后真正发给 Scout 的命令。离线与在线共用

$$
q\rightarrow u^{\mathrm{sol}}
\xrightarrow{\mathcal G_{\mathrm{phase,pub}}}u^{\mathrm{pub}}
\xrightarrow{(d_v,\tau_v),(d_\omega,\tau_\omega)}(v^r,\omega^r)
\rightarrow(\dot v^r,v^r\omega^r)\rightarrow z^\ell.
$$

delay buffer 记录真实 $u^{\mathrm{pub}}$，而不是未经过安全链的 $u^{\mathrm{sol}}$。RGB 液面只用于独立评价。

OfflineSloshOCP 输出

$$
\bar{\mathcal A}=\left(
\{\bar X_i^{\mathrm{aug}},\bar t_i\}_{i=0}^{M},
\{\bar q_i,\bar u_i^{\mathrm{pub}}\}_{i=0}^{M-1}
\right),
$$

并保留完整尾段：

~~~text
路径运动 → 减速 → 液体沉降 → 零命令保持
~~~

每个相位 $i$ 绑定 9 维经验 gate $\widehat{\mathcal R}^{\mathrm{emp}}_i$、执行状态兼容集 $\mathcal B_i^{\mathrm{exec}}$ 和保存动作 $u_{\mathrm{rec}}(i)$。兼容集采用逐相位硬边界

$$
\mathcal B_i^{\mathrm{exec}}=
\{e^{\mathrm{exec}}:\ |e_r^{\mathrm{exec}}|\le\beta_{r,i}^{\mathrm{exec}},\ \forall r\}.
$$

$\beta_{r,i}^{\mathrm{exec}}$ 由恢复 rollout 保守构造并在 held-out 前冻结。从相位 $i$ 周围采样偏离，只对执行状态满足 $\mathcal B_i^{\mathrm{exec}}$ 的样本执行保存动作并继续名义尾段；满足约束且按规定沉降的样本标为“可重接”，再拟合 gate 并用独立 trial 统计 false-accept。

artifact 至少冻结路径 hash、坐标系、$\Delta t$、执行/液体模型、容器、约束、完整尾段、gate、$\mathcal B_i^{\mathrm{exec}}$ 和 schema；任一项变化都重新生成。

## 3. III-C：双通道执行对齐与相位重接

延迟 $d_v,d_\omega$ 是从最终 $u^{\mathrm{pub}}$ 发布时刻辨识的。controller 在 $t_c$ 开始计算时，用冻结或在线估计的 $\widehat d_c$ 定义预计发布时刻 $t=t_c+\widehat d_c$；实际计算/发布延迟 $d_c$ 与估计误差 $d_c-\widehat d_c$ 进入 G0 统计和扰动边界。令

$$
n_v=\lceil d_v/\Delta t\rceil,\qquad
n_\omega=\lceil d_\omega/\Delta t\rceil,\qquad
n_f=\max(n_v,n_\omega),\qquad N_e=n_f+N_\ell.
$$

当前候选参数给出一条很具体的时间线：

| 相对预计发布时刻 $t$ | 发生什么 |
| --- | --- |
| $t$ | 用已发布历史和计算期间的已知保持命令，将状态从 $t_s$ 对齐到预计发布时刻 |
| $t+150\,\mathrm{ms}$ 左右 | 新线速度命令可能开始作用 |
| $t+220\,\mathrm{ms}$ 左右 | 新角速度命令才开始作用 |
| $t+7\Delta t\approx t+233.3\,\mathrm{ms}$ | 共同栅格化执行前沿 |
| $t+10\Delta t\approx t+333.3\,\mathrm{ms}$ | 联合终端位于前沿后 $N_\ell=3$、约 100 ms，检查重接条件 |

所以 100 ms 从共同执行前沿后计算，完整预测 lead 是

$$
(t-t_s)+N_e\Delta t.
$$

**B0 的具体原因：**新线速度命令在共同前沿前已作用约 83 ms。当前 history-only predictor 却用旧命令生成固定前沿状态，然后才启动短窗 OCP，漏掉了这段对新决策的依赖；同时 $0.22\,\mathrm{s}$ 与 $7\Delta t\approx0.2333\,\mathrm{s}$ 还相差约 $13.3\,\mathrm{ms}$。

正式方法必须从当前增广状态连续求解

$$
X_{k+1}^{\mathrm{aug}}
=F_{\mathrm{exec\mbox{-}\ell}}(X_k^{\mathrm{aug}},q_k),
\quad k=0,\ldots,N_e-1,
$$

或使用严格保留相同决策依赖的凝聚 bridge。前 $n_f$ 步只是执行延迟所需的因果传播，不是自由液体预览。

相位只在运行时钟附近选择：

$$
j\in[i_{\mathrm{clock}}-r_-,\,i_{\mathrm{clock}}+r_+],\qquad
j\ge j_{\mathrm{prev}},\qquad
j_f=j+n_f,\quad j_e=j+N_e.
$$

先用对齐到 $t$ 的 9 维状态选定一个 $j$，再只求解一次 OCP。禁止自由时间缩放、任意后退和全局跳相位；错位过大时判为不可重接。

## 4. III-D：Residual OCP 与经验 terminal gate

令 $\xi=[v,\omega]$。在线 OCP 保留 contour/lag 跟踪，并限制对名义速度、输入和液体状态的偏离：

$$
J=\sum_{k=0}^{N_e-1}\left(
J_{\mathrm{track},k}
+\|\xi_k-\bar\xi_{j+k}\|_{R_\xi}^2
+\|q_k-\bar q_{j+k}\|_{R_q}^2
+\|z_k^\ell-\bar z_{j+k}^\ell\|_{R_\ell}^2
\right)+J_f.
$$

核心约束是

$$
|v_t+a_0\Delta t-\bar v_j^{\mathrm{pub}}|\le\Delta v_{\max},\qquad
|\omega_t+\alpha_0\Delta t-\bar\omega_j^{\mathrm{pub}}|\le\Delta\omega_{\max},
$$

$$
e_{N_e|t}^{(9)}\in\widehat{\mathcal R}^{\mathrm{emp}}_{j_e},\qquad
e_{N_e|t}^{\mathrm{exec}}
=\left[b^v-\bar b^v,b^\omega-\bar b^\omega,x^a-\bar x^a\right]_{N_e|t}
\in\mathcal B_{j_e}^{\mathrm{exec}}.
$$

9 维 gate 和执行兼容集共同构成硬终端约束，不是可被软权重换掉的 $J_{\mathrm{rejoin}}$。每周期只发布第一拍，再用真实 $u^{\mathrm{pub}}$ 更新 buffer。

当前 gate 使用相位 $i$ 的 9 维位置、速度和液体相对误差，并以对角椭球表示：

$$
m_i(e)=\sum_r\left(\frac{e_r}{\rho_{r,i}}\right)^2,\qquad
m_i(e)\le1\Longleftrightarrow e\in\widehat{\mathcal R}^{\mathrm{emp}}_i.
$$

$m_i\le1$ 不表示“当前液面很低”，而表示“该 9 维误差在经验数据中有接回尾段的可能”；只有再满足 $\mathcal B_i^{\mathrm{exec}}$ 才放行。最重要的 held-out 指标是 false-accept：判断可重接，但实际恢复失败。

这里明确选择“9 维经验 gate＋执行状态硬兼容集”，避免直接拟合高维椭球的样本爆炸。但 $u_{\mathrm{rec}}(i)$ 仍是固定动作而非反馈 $\kappa_i(e)$，因此只能称 **phase-indexed empirical recovery gate/set**，不能称 funnel、certificate、feedback policy 或 recursive feasibility。

## 5. III-E：监督器与逐周期算法

| 在线判断 | 候选输出 |
| --- | --- |
| OCP 成功且 terminal gate/执行兼容约束通过 | residual-bounded $u^{\mathrm{sol}}$ 第一拍 |
| OCP 失败或 terminal gate 拒绝，但当前 $\widehat{\mathcal R}^{\mathrm{emp}}_j$、$\mathcal B_j^{\mathrm{exec}}$、保存动作和合同有效 | $u_{\mathrm{rec}}(j)$ |
| 状态过旧、当前 gate 拒绝、执行状态不兼容或对象无效 | 请求 $(0,0)$；这是 fail-closed 停车，不是防晃减速 |
| 启动期 artifact/solver/合同失效 | 拒绝进入 enforce |

所有运行期候选都经过同一 $\mathcal G_{\mathrm{phase,pub}}$、limiter 和 safety override；只有最终 $u^{\mathrm{pub}}$ 能进入 Scout 和下一周期 buffer。

~~~text
1. 读取 source-stamped 状态和已发布命令历史。
2. 将状态从 t_s 对齐到预计发布时刻 t。
3. 在时钟邻域内选定相位 j。
4. 从 X_t_aug 求解 N_e=n_f+N_ℓ 步 delay-augmented OCP。
5. 检查终端 9 维 gate 和执行状态兼容集。
6. 监督器选择 u_sol、u_rec(j) 或 (0,0) 候选。
7. 经统一安全链得到 u_pub，发布并写回历史。
~~~

## 6. 实现边界

| 当前缺口 | 判断 |
| --- | --- |
| B0 决策依赖与 $13.3\,\mathrm{ms}$ epoch/index 偏差 | 必须改成 delay-augmented OCP/等价凝聚 bridge 后再做 G0 |
| metadata 为 **offline_slosh_ocp=false** | 尚无正式完整尾段 artifact |
| 9 维 gate 半径仅用于 development smoke，也无 $\mathcal B_i^{\mathrm{exec}}$ | 补齐执行兼容约束及 held-out false-accept 证据 |
| S0–S4 只有分支接线结果 | 防晃改善仍须由 C0–C4 和独立 RGB 决定 |

**Go/No-Go：**G0 必须证明名义完整 lead $(t-t_s)+333.3\,\mathrm{ms}$ 内的预测可信，同时给出实际 $d_c$ 和 $d_c-\widehat d_c$ 的分布/边界，且新命令对联合终端有可检测作用；不通过就停止在线相位修正，退回执行感知的离线前馈＋低频纠偏。

若最终不保留 contour/lag/progress，名称改为 **Phase-Rejoining Residual MPC**。
