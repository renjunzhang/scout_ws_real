# Phase-Rejoining Residual S-MPCC：Methods 章节组织

> 文档定位：固定 Methods 的因果主线、核心公式和证据边界；实现参数、ROS 配置及完整实验清单放入实验节或补充材料。
>
> 当前证据等级：`RESEARCH_PROPOSAL`。第一阶段只称 **phase-indexed empirical recovery gate**；满足鲁棒逐拍包含等条件后，才升级为 recovery funnel/certificate。

## 1. 方法主线

Scout 的线、角速度命令具有不同延迟和惯性。在线修正若只追求瞬时液面较低，可能破坏离线序列后续的相位抵消。因此本文回答的问题是：

> 当前修正经过真实执行链产生作用后，系统是否仍能在约束内重新接回离线防晃序列的剩余尾段？

方法采用“**离线完整防晃 + 在线短时残差修正**”：OfflineSloshOCP 生成完整名义动作及相位索引恢复对象；在线控制器利用命令历史对齐执行前沿，只在邻近相位中寻找重接点，并把尾段可恢复性作为残差 S-MPCC 的终端 gate。

![执行前沿对齐的相位重接残差 S-MPCC 方法结构](assets/figures/phase_rejoining_method_structure_zh.svg)

**图 1　方法结构。** 上层离线生成名义序列与恢复对象，中层在线完成执行前沿对齐、有限相位重接和短时域优化，下层执行第一拍并闭环反馈；不可重接或合同失效时转入防晃减速、受控停车或重新规划。

第一篇论文限定于**冻结几何路径及预注册的小幅偏离**。大幅改道、动态障碍和容器条件变化不作恢复承诺，而是触发重新规划。

## 2. Methods 正文结构

正文压缩为五节，每节只承担一个论证任务。

### III-A. 问题定义与总体结构

定义冻结路径、机器人—液体增广状态、控制输入、平台/液体约束和最终发布命令。结合图 1 说明离线 artifact、在线控制器和真实系统之间的接口，并给出评价量：路径误差、任务时间和独立测得的液面响应。

### III-B. 执行感知模型与离线名义序列

区分求解器命令 $u^{\mathrm{opt}}$、最终发布命令 $u^{\mathrm{pub}}$ 与真实运动 $(v^r,\omega^r)$，采用唯一的执行—液体因果链：

$$
u_k^{\mathrm{pub}}
=\mathcal G_{\mathrm{pub}}(u_k^{\mathrm{opt}},x_k^a),
\qquad
u^{\mathrm{pub}}\rightarrow x^a\rightarrow(v^r,\omega^r)
\rightarrow(\dot v^r,v^r\omega^r)\rightarrow z_\ell .
$$

$x^a$ 包含线/角双通道 delay buffer、限速和惯性状态；$z_\ell$ 是由纵向加速度与横向向心加速度驱动的双轴低阶液体状态。离线和在线使用同一离散模型，buffer 记录 $u^{\mathrm{pub}}$，所有状态按源时间戳对齐；RGB 液面只用于独立评价。

OfflineSloshOCP 首先生成名义序列

$$
\bar{\mathcal A}
=\{\bar x_i,\bar x_i^a,\bar z_{\ell,i},
\bar u_i^{\mathrm{opt}},\bar u_i^{\mathrm{pub}},t_i\}_{i=0}^{M},
$$

随后为各相位构造经验恢复对象 $(\widehat{\mathcal R}^{\mathrm{emp}}_i,\kappa_i^{\mathrm{emp}})$，共同形成运行 artifact。路径、模型、执行链、容器条件、约束或终端合同变化时，artifact 立即失效并在运动前重新生成。

### III-C. 执行前沿对齐与有限相位重接

双通道延迟分别保留在增广状态中，共同前沿只用于统一索引：

$$
d_f=\max(d_v,d_\omega),\qquad
n_f=\left\lceil\frac{d_f}{\Delta t}\right\rceil,\qquad
N_e=n_f+N_\ell,
$$

$$
j_f=j+n_f,\qquad j_e=j+n_f+N_\ell.
$$

在线 OCP 从当前时刻 $t$ 的增广状态出发，利用命令历史和双通道 buffer 统一预测 $N_e$ 步；执行前沿位于第 $n_f$ 步，前沿后可信液体窗口为 $N_\ell$ 步。延迟只能在这次增广预测中出现一次，不能先外推到前沿后再重复加入。

相位 $j$ 只在上一接受索引附近单调枚举。几何进度 $s$ 与防晃相位分开，并通过

$$
|s_{k|t}-\bar s_{j+k}|\le\epsilon_s^{\max}
$$

限制错位，禁止任意时间伸缩和全局跳相位。

### III-D. 相位索引的经验尾段恢复门

恢复误差必须是 Markov 状态，至少覆盖路径误差、真实速度、执行器状态、液体状态和命令历史：

$$
e_i=\big[e_i^{\mathrm{path}},\ e_i^{v,\omega},\ e_i^a,\ e_i^\ell,\ e_i^{\mathrm{hist}}\big].
$$

对每个相位，用扰动 rollout 构造 $\widehat{\mathcal R}^{\mathrm{emp}}_i$，在 held-out trial 上验证 coverage、false-accept、false-reject 和实际重接率，并同时保存可执行策略 $\kappa_i^{\mathrm{emp}}$。经验 gate 只支持统计性结论，不宣称递归可行性。

只有进一步证明

$$
(e,\kappa_i(e))\in\mathcal Z_i,
\qquad
f_i^e(e,\kappa_i(e),w)\in\widehat{\mathcal F}_{i+1}^{\mathrm{rob}},
\quad \forall w\in\mathcal W_i,
$$

并满足终端正不变与策略可移位，才可把恢复对象升级为 recovery funnel/certificate。

### III-E. 在线残差 S-MPCC 与失效安全监督器

对每个候选相位 $j$，在线控制量写为

$$
u_{k|t}^{\mathrm{opt}}
=\bar u_{j+k}^{\mathrm{opt}}+\delta u_{k|t},
$$

并最小化

$$
J=J_{\mathrm{MPCC}}+J_{\mathrm{nominal}}+J_{\mathrm{residual}}
+J_{\mathrm{liquid,rel}}+J_{\mathrm{rejoin}},
$$

满足平台硬约束、发布增量约束和经验终端 gate：

$$
u^{\mathrm{pub}}\in\mathcal U,\qquad
\Delta u^{\mathrm{pub}}\in\Delta\mathcal U,\qquad
e_{N_e|t}^{(j)}\in\widehat{\mathcal R}^{\mathrm{emp}}_{j_e}.
$$

控制器选择代价最低的可行候选，只发布第一拍，并在下一周期根据真实 $u^{\mathrm{pub}}$ 历史重新求解。几何参考可以长预览，但 $N_e$ 之后不得引入液体模型未覆盖的自由控制。

| 在线结果 | 动作 |
| --- | --- |
| 候选可行且终端 gate 通过 | 发布所选候选第一拍 |
| 求解失败，但当前误差已在 gate 内，且合同、时间戳、状态和策略硬约束均有效 | 调用 $\kappa_i^{\mathrm{emp}}$，命令仍经过同一发布约束 |
| 候选不可重接、出现非有限值或任一合同失效 | 防晃减速或受控停车，必要时重新规划 |
| 人员安全触发急停 | 无条件覆盖上述液体恢复逻辑 |

若最终实现没有 contouring error、lag error 和 progress term，方法名称应改为 **Phase-Rejoining Residual MPC**。

## 3. 唯一前置放行门 G0

若 $d_v\approx150\,\mathrm{ms}$、$d_\omega\approx220\,\mathrm{ms}$，且前沿后液体窗口 $T_\ell\approx100\,\mathrm{ms}$，则终端实际位于当前时刻约 $250$–$320\,\mathrm{ms}$ 后。实施恢复 gate 前必须在 held-out 数据上确认：

- 总 lead 上的液体状态预测仍可信；
- 当前新命令对恢复终端具有可检测的控制作用；
- IMU 在液体主频附近的幅值与相位误差可接受；
- 延迟参数跨 trial、电量和地面条件足够稳定。

G0 不通过时，停止在线相位修正，退回执行感知的离线前馈与低频纠偏；不能仅把“$100\,\mathrm{ms}$ 窗口”向后平移后继续宣称有效。

## 4. 证据与写作边界

- 当前只称 **phase-indexed empirical recovery gate/set**，报告 false-accept（false-safe）及实际重接统计。
- 只有完成逐拍鲁棒包含、终端正不变和策略移位证明后，才使用 **recovery funnel/certificate** 与递归可行性表述。
- 不把低阶液高 surrogate 等同于现实绝对不洒，也不把平均求解时间等同于硬实时保证。
- ROS topic、模型辨识流程、控制权重、求解器配置和实验参数移至实验节或补充材料。
