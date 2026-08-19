# Phase-Rejoining Residual S-MPCC：Methods 章节组织思路

> 文档用途：统一方法逻辑、符号和论文 Methods 结构。
>
> 当前状态：`RESEARCH_PROPOSAL`。尚未完成的理论性质和实验结论不能使用完成时表述。
>
> 推荐方法名：**Phase-Rejoining Residual S-MPCC**。

## 1. 方法总览

一句话概括：

> 先用双通道执行模型预测新命令何时真正作用于 Scout，再用短时域残差 S-MPCC 修正离线防晃序列；修正终端必须进入相位索引的尾段恢复对象，从而保留接回完整防晃尾段的能力。

![Phase-Rejoining Residual S-MPCC 方法结构](assets/figures/phase_rejoining_method_structure_zh.svg)

> 图 1：方法由“离线准备—在线重接—恢复与降级”三部分组成。SVG 源图位于 `assets/figures/phase_rejoining_method_structure_zh.svg`，PNG 预览位于同目录。

整体流程：

```text
冻结几何路径与任务合同
→ 执行—液体增广模型
→ OfflineSloshOCP 完整防晃名义序列
→ 相位索引尾段恢复对象与恢复策略
→ 在线读取真实状态和最终发布命令历史
→ 双通道执行前沿对齐
→ 枚举有限候选名义索引
→ 短时域残差 S-MPCC
→ 终端重接判据
→ 发布第一拍 / 执行恢复策略 / 防晃降级
```

恢复对象的名称必须与证据等级一致：

| 已完成证据 | 允许名称 | 允许结论 |
|---|---|---|
| 逐拍鲁棒包含、终端正不变、在线策略可移位 | recovery funnel / certificate | 标定模型与有界扰动内的尾段可行性和递归可行性 |
| Monte Carlo 或数据 rollout + 独立 held-out 验证 | empirical recovery set / gate | 报告接受率、重接成功率和 false-accept |

## 2. 研究问题与创新边界

### 2.1 要解决的三个断点

旧方案存在三个关键问题：

1. 求解器命令不等于真实运动，Scout 的线速度和角速度具有不同延迟与惯性；
2. MPC 每周期只执行第一拍，后续用于液体相位抵消的动作可能被下一次重规划覆盖；
3. 液体模型的远期预测不够可信，不能简单把完整剩余尾段全部放进在线 NMPC。

本文的核心问题是：

> 如何只依赖经数据验证的短期液体预测，判断一次真实底盘修正是否仍能重接离线完整防晃尾段？

### 2.2 真正的增量

以下组件已有直接近邻，不能单独作为创新：

| 组件 | 代表近邻 | 本文的处理 |
|---|---|---|
| 低阶晃液模型与液高映射 | A01、A02 | 作为模型基础 |
| 离线防晃 OCP | B01、B11、C01、C04 | 生成要保护的名义尾段 |
| 离线参考 + 在线跟踪 | B10、B11、C06、C07 | 不是主创新 |
| 在线 slosh-aware MPC | B12、C11、AR03、AR04 | 不能声称首次 |
| phase/progress MPC | D19、D20 | 名义索引本身不是创新 |
| reference governor | AR07 | 只做门控仍不够 |

主创新应收敛为：

> 用相位索引的短时鲁棒前驱或经验恢复 gate，将许多局部可信转移组合成对完整防晃尾段的恢复条件。

贡献按三层组织：

1. **算法贡献**：相位索引尾段恢复对象约束短时域残差 S-MPCC；
2. **必要模型闭合**：最终发布命令经过双通道执行动态形成真实运动，再激励液体；
3. **实证贡献**：真实 Scout、真实液体和独立视觉评价下验证预测、重接判断与执行效果。

### 2.3 第一篇论文的范围

- 输入是一条已平滑、已碰撞复核并冻结的几何路径；
- 在线只处理执行误差和预注册的小幅偏离；
- 动态障碍不是核心实验，最多加入小幅静态绕行作为扩展；
- 大幅路径变化、容器变化或模型版本变化时，原名义序列和恢复对象失效；
- 若不实现执行期路径重规划，论文定位为 prescribed-route anti-slosh execution，而不是完整 local planner。

## 3. 执行前沿与索引语义

### 3.1 最重要的 G0 放行门

当前平台的代表时间尺度为：

$$
\omega_n\approx31.246\ \mathrm{rad/s},
\qquad
T_1=\frac{2\pi}{\omega_n}\approx201\ \mathrm{ms},
$$

$$
d_v\approx150\ \mathrm{ms},
\qquad
d_\omega\approx220\ \mathrm{ms}.
$$

候选 $100\ \mathrm{ms}$ 液体窗口约为半个第一模态周期，但新命令可能在该窗口内尚未充分作用。因此液体窗口必须从执行前沿之后计算：

$$
d_v+T_\ell\approx250\ \mathrm{ms},
\qquad
d_\omega+T_\ell\approx320\ \mathrm{ms}.
$$

在实现恢复对象前，必须同时验证：

1. 从当前时刻到总 lead 末端的液体预测误差；
2. 终端液体状态对当前第一拍命令的灵敏度；
3. processed IMU 在液体主频附近的幅值与相位误差；
4. 延迟和惯性参数跨 trial、电量与地面是否稳定。

若总 lead 预测或控制灵敏度不通过，停止恢复漏斗路线，退回：

> 执行感知的离线防晃前馈 + 不破坏主要节奏的低频路径纠偏。

### 3.2 时域符号

离线 artifact 与在线控制统一使用周期 $\Delta t$。

| 符号 | 含义 |
|---|---|
| $d_v,d_\omega$ | 线速度与角速度通道延迟 |
| $d_f=\max(d_v,d_\omega)$ | 共同叙事前沿，内部仍保留双通道动态 |
| $n_f=\lceil d_f/\Delta t\rceil$ | 执行前沿对应步数 |
| $T_\ell$ | G0 数据支持的前沿后液体窗口 |
| $N_\ell=\lceil T_\ell/\Delta t\rceil$ | 液体窗口步数，实际离散窗口必须重新验证 |
| $N_e=n_f+N_\ell$ | 当前时刻到恢复检查终端的耦合步数 |
| $N_r\ge N_e$ | 机器人几何参考长预览 |
| $N_b$ | 离线恢复集合计算块长 |

实际恢复检查时刻为：

$$
t_e=t+N_e\Delta t.
$$

$t_e$ 与连续叙事位置 $t+d_f+T_\ell$ 可能存在离散取整差，实验必须使用真实的 $t_e$。

### 3.3 当前、前沿和终端索引

选中当前名义索引后：

$$
i_t=\text{当前接受的名义索引},
\qquad
i_f=i_t+n_f,
\qquad
i_e=i_t+N_e=i_f+N_\ell.
$$

在线候选 $j$ 始终表示“候选当前索引”，不是前沿索引：

$$
j_f=j+n_f,
\qquad
j_e=j+N_e.
$$

因此：

- 当前残差命令相对 $\bar u_j$ 构造；
- 执行模型从当前状态逐步传播，不能把延迟计算两遍；
- terminal membership 在 $j_e$ 对应的恢复对象上检查。

### 3.4 有限相位重接

第一版不优化连续 $\tau$ 或 $\dot\tau$，只枚举上一索引附近的小规模整数候选：

$$
i_t^{\mathrm{sh}}=\min(i_{t-1}+1,M),
$$

$$
\mathcal I_t=
\left(
\{i_t^{\mathrm{pred}}-r_b,\ldots,i_t^{\mathrm{pred}}+r_f\}
\cup\{i_t^{\mathrm{sh}}\}
\right)
\cap\{i_{t-1},\ldots,M\}.
$$

对每个 $j\in\mathcal I_t$ 单独求连续 QP/NLP，再选择代价最低的可行解。这样避免在线 MINLP，也禁止：

- 在整条序列上任意最近邻跳转；
- 通过自由时间伸缩伪造可恢复；
- 只按几何位置或液体幅值重接；
- 长期相位错位后继续假设原尾段有效。

robust 分支若要证明递归可行，候选裁剪不得删除正常移位候选 $i_t^{\mathrm{sh}}$。

## 4. 执行感知的机器人—液体模型

### 4.1 命令链必须分开

至少区分：

- $u^{\mathrm{opt}}$：求解器原始命令；
- $u^{\mathrm{pub}}$：经过正常限幅和整形后最终发布的命令；
- $x^a$：delay buffer、rate limiter 和惯性内部状态；
- $v^r,\omega^r$：Scout 真实线速度和角速度；
- $z_\ell$：低阶液体状态；
- $y_\ell$：独立液面评价量。

正常发布链写为：

$$
u_k^{\mathrm{pub}}
=
\mathcal G_{\mathrm{pub}}
\left(u_k^{\mathrm{opt}},x_k^a\right).
$$

delay buffer 必须由 $u^{\mathrm{pub}}$ 驱动。确定性的限幅或 rate limiter 应进入 $\mathcal G_{\mathrm{pub}}$ 和 $x^a$；外部紧急安全覆盖会使当前恢复论证失效。

### 4.2 双通道执行模型

第一版候选模型为：

$$
\tau_v\dot v^r+v^r
=\mathcal S_v\!\left(v^c(t-d_v)\right),
$$

$$
\tau_\omega\dot\omega^r+\omega^r
=\mathcal S_\omega\!\left(\omega^c(t-d_\omega)\right).
$$

$\mathcal S_v,\mathcal S_\omega$ 只表示无记忆饱和或死区；rate limiter 具有内部状态，必须放入 $x^a$。最终结构需要与纯延迟、ARX/状态空间等候选在 held-out trial 上比较。

### 4.3 容器激励与液体状态

容器参考点的基础激励为：

$$
a_x^C=\dot v^r,
\qquad
a_y^C=v^r\omega^r.
$$

双轴第一模态可统一写为：

$$
\ddot\eta_q
+2\zeta_q\omega_{n,q}\dot\eta_q
+\omega_{n,q}^2\eta_q
=-\kappa_q a_q^C,
\qquad q\in\{x,y\}.
$$

保留幅值和速度符号的液体状态为：

$$
z_\ell=
[\omega_{n,x}\eta_x,\dot\eta_x,
 \omega_{n,y}\eta_y,\dot\eta_y]^\mathsf T.
$$

位姿和液体都由 predicted realized motion 驱动，不能由理想命令直接驱动。

### 4.4 在线状态与冻结合同

- odom/Nokov：机器人位姿和 realized motion；
- 最终发布命令历史：双通道 delay buffer；
- processed IMU：容器激励与液体状态传播；
- RGB：第一版只做独立真实液面评价；
- 所有状态必须使用 source timestamp 对齐。

离线名义序列与恢复对象绑定路径、执行模型、容器、液位、约束和配置 hash。任一合同变化后必须重新生成。

## 5. OfflineSloshOCP 名义序列

### 5.1 离线问题

OfflineSloshOCP 优化 $u^{\mathrm{opt}}$，通过 $\mathcal G_{\mathrm{pub}}$ 和增广模型得到真实运动与液体响应：

$$
\min
J_{\mathrm{time}}
+J_{\mathrm{path}}
+J_{\mathrm{command}}
+J_{\mathrm{liquid}}
+J_{\mathrm{terminal}},
$$

满足：

- 冻结路径与走廊约束；
- Scout 输入、速度、转速和变化率边界；
- 液体模型标定域和液高 surrogate；
- 终点机器人、执行器、命令历史和液体残余合同。

防晃收益不能仅来自整体减速；主对比必须匹配或显式报告任务时间。

### 5.2 冻结 artifact

状态节点和控制区间分别为：

$$
\{\bar x_i,\bar x_i^a,\bar z_{\ell,i}\}_{i=0}^{M},
\qquad
\{\bar u_i^{\mathrm{opt}},\bar u_i^{\mathrm{pub}}\}_{i=0}^{M-1}.
$$

同时保存节点物理时间、路径位置、约束余量、终端合同和所有 hash。

临近终点的在线索引需要显式 hold 延拓。只有当：

1. $\bar x_M,\bar x_M^a,\bar z_{\ell,M}$ 是保持命令下的名义平衡点；
2. 第 6 节终端集合在保持策略下正不变；

才能对 $i\ge M$ 延拓名义状态、保持命令和鲁棒恢复对象。

### 5.3 名义序列先要证明值得保护

构造恢复对象前必须证明：

- actual-input replay 能解释 held-out 液面主频、幅值和相位；
- identified-actuator replay 能验证“命令→运动→液体”的完整预测链；
- OfflineSloshOCP 在同任务时间下优于 smooth/jerk-limited baseline；
- 改善超过独立 RGB 测量分辨率；
- 终端残余和 settling time 确实改善。

若离线序列没有稳定的实液收益，停止 phase-rejoining 主线。

## 6. 相位索引尾段恢复对象

### 6.1 增广误差与约束域

恢复对象必须使用满足 Markov 性的误差状态：

$$
e_i=
[e_s,e_y,e_\psi,e_v,e_\omega,
 e_a^\mathsf T,e_z^\mathsf T,
 e_{u,\mathrm{hist}}^\mathsf T]^\mathsf T.
$$

其中 $e_z=z_\ell-\bar z_{\ell,i}$。离线名义中允许非零液体状态，在线惩罚的是相对名义偏差，而不是强制每拍液体静止。

每个索引定义约束域：

$$
\mathcal Z_i=
\left\{(e,\delta u):
\begin{array}{l}
\text{路径误差在复核走廊内},\\
\text{Scout 状态和命令满足边界},\\
\bar u_i^{\mathrm{opt}}+\delta u\in\mathcal U_i,\\
\text{液体状态在标定域和 surrogate 余量内}
\end{array}
\right\}.
$$

### 6.2 鲁棒恢复漏斗

局部误差动力学为：

$$
e_{i+1}=f_i^e(e_i,\delta u_i,w_i),
\qquad
w_i\in\mathcal W_i.
$$

$\mathcal W_i$ 包含执行模型残差、液体模型残差、参数偏差和状态估计误差。

终点先定义可保持集合与策略：

$$
\left(e,\kappa_M(e)\right)\in\mathcal Z_M,
$$

$$
f_M^e
\left(e,\kappa_M(e),w\right)
\in\widehat{\mathcal F}^{\mathrm{rob}}_M,
\quad
\forall e\in\widehat{\mathcal F}^{\mathrm{rob}}_M,\
\forall w\in\mathcal W_M.
$$

随后从 $M-1$ 向前计算集合和保存策略，使每一拍满足：

$$
\boxed{
\widehat{\mathcal F}^{\mathrm{rob}}_i
\subseteq
\left\{
e:
\begin{array}{l}
\left(e,\kappa_i(e)\right)\in\mathcal Z_i,\\
f_i^e\left(e,\kappa_i(e),w\right)
\in\widehat{\mathcal F}^{\mathrm{rob}}_{i+1},\
\forall w\in\mathcal W_i
\end{array}
\right\}.
}
$$

这条逐拍包含关系才是 recovery funnel/certificate 的核心。$N_b$ 只用于离线分块加速；块内仍需保存中间集合和策略，并验证逐拍包含。

### 6.3 经验恢复 gate

若只能通过 rollout 拟合恢复范围，则使用：

$$
\widehat{\mathcal R}^{\mathrm{emp}}_i,
\qquad
\kappa_i^{\mathrm{emp}}.
$$

生成流程：

1. 预注册参数、扰动、初始相位和成功条件；
2. 用 development rollout 生成标签并拟合保守内域；
3. 在完全隔离的 held-out rollout 和实物 trial 上验证；
4. 报告 coverage、false-accept、false-reject 和 accepted-but-failed rejoin。

经验数据只能支持有限保持区间 $M{:}M+H_{\mathrm{emp}}$，不能外推无限不变性。

经验分支不能使用 invariant、funnel、certificate 或 recursive feasibility 等表述。

## 7. 在线 Phase-Rejoining Residual S-MPCC

### 7.1 候选传播

对每个 $j\in\mathcal I_t$，从当前 source-stamped 增广状态开始：

$$
x_{0|t}=x_t,
$$

$$
x_{k+1|t}
=f_{\mathrm{aug}}
\left(x_{k|t},u_{k|t}^{\mathrm{pub}},w_{k|t}\right),
\quad
k=0,\ldots,N_e-1.
$$

模型显式保留两路 delay buffer、惯性、容器激励和液体状态。不能先把状态搬到共同执行前沿后再重复施加延迟。

### 7.2 残差命令

经验/确定性分支使用：

$$
u_{k|t}^{\mathrm{opt}}
=\bar u_{j+k}^{\mathrm{opt}}+\delta u_{k|t},
$$

$$
u_{k|t}^{\mathrm{pub}}
=\mathcal G_{\mathrm{pub}}
\left(u_{k|t}^{\mathrm{opt}},x_{k|t}^a\right).
$$

robust 分支使用 tube policy：

$$
\pi_{k|t}(e)
=\bar{\delta u}_{k|t}
+K_{j+k}^{\mathrm{pre}}
\left(e-\check e_{k|t}^{(j)}\right),
$$

$$
\mathcal T_{k|t}^{(j)}
=\check e_{k|t}^{(j)}
\oplus\mathcal D_{k|t}^{(j)}.
$$

tube 由候选 $j$ 坐标下的当前状态估计集合初始化：

$$
\check e_{0|t}^{(j)}
=\operatorname{ctr}\!\left(\mathcal E_t^{(j)}\right),
\qquad
\mathcal E_t^{(j)}
\subseteq\mathcal T_{0|t}^{(j)}.
$$

tube 中心按名义动力学传播，不能作为自由游标：

$$
\check e_{k+1|t}^{(j)}
=f_{j+k}^e
\left(\check e_{k|t}^{(j)},\bar{\delta u}_{k|t},0\right).
$$

并要求：

$$
\mathcal T_{k+1|t}^{(j)}
\supseteq
\left\{
f_{j+k}^e(e,\pi_{k|t}(e),w):
e\in\mathcal T_{k|t}^{(j)},\
w\in\mathcal W_{j+k}
\right\},
$$

$$
\left(e,\pi_{k|t}(e)\right)\in\mathcal Z_{j+k},
\qquad
\forall e\in\mathcal T_{k|t}^{(j)}.
$$

### 7.3 几何 MPCC 与名义相位必须分开

令冻结路径 $p:[0,L]\rightarrow\mathbb R^2$，切向和法向为 $t(s),n(s)$。定义：

$$
\varepsilon_{k|t}^{\perp}
=n(s_{k|t})^\mathsf T
\left(p_{k|t}^r-p(s_{k|t})\right),
$$

$$
\varepsilon_{k|t}^{\parallel}
=t(s_{k|t})^\mathsf T
\left(p_{k|t}^r-p(s_{k|t})\right).
$$

几何进度满足：

$$
s_{k+1|t}=s_{k|t}+\Delta t\,\nu_{k|t},
\qquad
0\le s_{k|t}\le L,
$$

$$
\left|s_{k|t}-\bar s_{j+k}\right|
\le\epsilon_s^{\max}.
$$

进度变化量定义为：

$$
\Delta\nu_{0|t}
=\nu_{0|t}-\nu_{t-1}^{\mathrm{applied}},
\qquad
\Delta\nu_{k|t}
=\nu_{k|t}-\nu_{k-1|t}\quad(k\ge1).
$$

$s$ 是几何路径进度，$j+k$ 是防晃名义相位；优化 $s$ 不能改变液体参考索引。

在线目标为：

$$
\min
J_{\mathrm{MPCC}}
+J_{\mathrm{nominal}}
+J_{\mathrm{residual}}
+J_{\mathrm{liquid,rel}}
+J_{\mathrm{rejoin}},
$$

其中：

$$
J_{\mathrm{MPCC}}
=
\sum_{k=0}^{N_e-1}
\left[
q_\perp(\varepsilon_{k|t}^{\perp})^2
+q_\parallel(\varepsilon_{k|t}^{\parallel})^2
-q_p\Delta t\,\nu_{k|t}
+q_\nu(\Delta\nu_{k|t})^2
\right].
$$

若最终实现没有 $s,\nu$、contouring/lag error 和 progress term，方法应改名为 **Phase-Rejoining Residual MPC**。

### 7.4 核心终端判据

robust 分支：

$$
\boxed{
\mathcal T_{N_e|t}^{(j)}
\subseteq
\widehat{\mathcal F}^{\mathrm{rob}}_{j_e}.
}
$$

empirical 分支：

$$
\boxed{
e_{N_e|t}^{(j)}
\in
\widehat{\mathcal R}^{\mathrm{emp}}_{j_e}.
}
$$

第二式只能称 empirical rejoining gate，不能称安全证书。

### 7.5 长预览与第一拍执行

$N_r$ 只用于读取更远的几何参考、曲率和走廊信息：

- 自由残差控制只到 $N_e-1$；
- $N_e{:}N_r$ 不得引入液体模型看不到的自由控制；
- 若需要动态长尾，只能接入名义尾段或保存的恢复策略；
- 长预览不能放宽 $N_e$ 处的 terminal membership。

选中 $j_t$ 后只发布第一拍：

$$
\delta u_t=
\begin{cases}
\pi_{0|t}(\hat e_t), & \text{robust 分支},\\
\delta u_{0|t}, & \text{empirical 分支},
\end{cases}
$$

$$
u_t^{\mathrm{opt}}
=\bar u_{j_t}^{\mathrm{opt}}+\delta u_t,
$$

$$
u_t^{\mathrm{pub}}
=\mathcal G_{\mathrm{pub}}
\left(u_t^{\mathrm{opt}},x_t^a\right).
$$

下一周期重新读取真实 $u^{\mathrm{pub}}$ 历史和 realized motion。

## 8. 求解失败、恢复与降级

### 8.1 robust 分支

每次成功求解后保存：

$$
\Pi_t=
\left\{
\left(\mathcal T_{k|t},\pi_{k|t}\right)_{k=1}^{N_e-1},
\left(\widehat{\mathcal F}_i^{\mathrm{rob}},\kappa_i\right)_{i=j_e}^{M}
\right\}.
$$

下一周期求解失败时：

1. 检查 hash、时间戳、最终发布命令和状态估计集合仍在保存 tube 内；
2. 未到 terminal set 时执行移位后的 $\pi$ 前缀；
3. 到达 $\widehat{\mathcal F}_i^{\mathrm{rob}}$ 后依次调用 $\kappa_i$；
4. 第 $r$ 次移位依次附接 $\kappa_{j_e+r-1}$，不能重复附接固定索引。

终端正不变、逐拍恢复包含和在线 tube 可移位三者同时成立，才能论证递归可行。

### 8.2 empirical 分支

只有当前实际误差本身满足：

$$
e_t\in\widehat{\mathcal R}_{i_t}^{\mathrm{emp}},
$$

且仍在验证合同内时，才能调用 $\kappa_{i_t}^{\mathrm{emp}}$。前一周期预测未来会进入 gate，不能推出当前求解失败时已经可恢复。

### 8.3 集合外与硬安全

若所有候选均不可重接，或保存策略因命令覆盖、状态偏离、传感器异常或合同变化失效：

1. 不允许放宽索引或任意时间伸缩；
2. 执行独立验证的 slosh-aware slowdown / controlled stop；
3. 人车安全需要时立即急停并记录液体风险；
4. 停稳后重置液体状态，必要时重新生成路径、名义序列和恢复对象。

## 9. 实施放行路线

| 阶段 | 必须证明 | 不通过时 |
|---|---|---|
| G0：总 lead | 预测可信且第一拍对终端液体有可检测作用 | 只保留离线前馈与低频纠偏 |
| G1：离线名义 | 同任务时间下实液指标优于 smooth baseline | 停止 phase-rejoining |
| G2：恢复对象 | held-out 上重接成功，robust 分支满足逐拍包含 | 降级为 empirical gate 或停止 |
| G3：在线闭环 | residual controller 与恢复条件均有独立收益 | 收缩在线修正权限 |

优先实验偏离：

- 人为增加命令延迟；
- 临时限幅或摩擦变化；
- 短时速度门或暂停；
- 小初始位姿误差；
- 安全范围内的不同初始液体相位。

## 10. 正式 Methods 章节结构

论文 Methods 建议只保留七个小节：

| 小节 | 核心内容 |
|---|---|
| III-A. Problem Formulation and Architecture | prescribed-route 范围、图 1、输入输出和保证边界 |
| III-B. Realization-Aware Robot–Liquid Dynamics | 发布命令、双通道执行模型、容器激励与液体状态 |
| III-C. Offline Anti-Slosh Nominal Generation | OfflineSloshOCP、终端合同和冻结 artifact |
| III-D. Discrete Phase Rejoining | 当前/前沿/终端索引、有限候选和禁止自由时间伸缩 |
| III-E. Phase-Indexed Tail-Recovery Objects | robust funnel 或 empirical gate、保存策略 |
| III-F. Online Residual S-MPCC | tube/point 预测、MPCC 目标、terminal membership 和第一拍 |
| III-G. Recovery Logic and Scope | contingency shift、slowdown/stop/replan 和适用域 |

ROS topic、路径预处理参数、完整辨识结果、控制权重、集合参数和求解器配置放补充材料，不进入 Methods 主线。

## 11. 最低实验与指标

### 11.1 主对比

1. matched-time smooth / jerk-limited；
2. OfflineSloshOCP 纯回放；
3. residual S-MPCC，无恢复终端；
4. residual S-MPCC + empirical recovery gate；
5. 完整 robust phase-rejoining 方法；
6. 可选 full-horizon liquid NMPC oracle。

### 11.2 关键消融

- 无执行前沿对齐 vs 双通道执行模型；
- 相位无关终端球 vs 相位索引恢复对象；
- $e_z$-only vs 完整 Markov 增广误差；
- nominal point terminal vs robust tube inclusion；
- 无保存策略 vs 保存 $(\mathcal T,\pi)$ 和 $\kappa_i$；
- 自由时间伸缩 vs 有限离散重接；
- 不同 $N_\ell$ 和 total-lead 终端位置。

### 11.3 必须报告

- 液面：peak、P95、RMS、spill margin、terminal tail、settling time；
- 预测：lead-dependent 幅值/相位误差与 first-action sensitivity；
- 恢复：coverage、rejoin success、false-accept、false-reject；
- 任务：路径误差、完成时间、相位索引偏移；
- 执行链：$u^{\mathrm{opt}}$、$u^{\mathrm{pub}}$、realized motion 和容器激励；
- 实时性：solver p50/p95/p99/max、deadline miss、fallback 次数。

统计单位是完整 trial 或配对 block，不能把 MPC cycle、IMU sample 或 RGB frame 当独立样本。

## 12. 写作边界与最终表述

### 12.1 完整 robust 条件成立时

必须同时完成：

1. 有效扰动集合 $\mathcal W_i$；
2. 终端 hold set 正不变；
3. 每个索引的逐拍鲁棒包含；
4. 在线 tube policy 与 contingency 可移位。

此时核心贡献可以写为：

> 本文通过执行前沿对齐和相位索引的尾段恢复漏斗，将离线防晃序列的剩余抵消能力编码为短时域残差 S-MPCC 的终端条件，使在线修正在声明的模型与扰动范围内保留接回完整防晃尾段的可行策略。

### 12.2 只完成 empirical 分支时

应写为：

> 本文通过执行前沿对齐和独立数据验证的相位索引恢复 gate，筛选短时域残差修正，并报告 gate 接受后实际重接成功与失败的统计结果。

不能出现 funnel、certificate、invariant、guaranteed safe 或 recursive feasibility。

### 12.3 无论哪条分支都不能声称

- 首次移动机器人防晃或首次 slosh-aware MPC；
- 名义索引就是传感器测得的真实液体相位；
- $100\ \mathrm{ms}$ 是跨平台天然可信时域；
- 低阶模型液高约束等于现实液体绝对不洒；
- 平均求解时间证明硬实时；
- 固定路径实验等于完整在线 local planning。
