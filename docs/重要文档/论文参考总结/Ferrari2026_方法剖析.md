# Ferrari 2026 RA-L 方法剖析

论文：Ferrari et al., "Time-Optimal Anti-Sloshing Trajectory Planning for Multiple Liquid-Filled Containers Subject to SCARA Motion", IEEE RA-L Vol.11 No.2, Feb 2026, pp.1762–1769.

剖析日期：2026-05-07
剖析对象：方法学层面的可借鉴点 / 不可直接迁移的边界 / 与当前 Scout Online GeoRef 主线的对应关系。

## 1. 论文定位与场景

论文研究的不是控制器层面的"在线压晃"，而是**离线轨迹规划阶段把液面高度模型作为硬约束**。

```text
任务: pick-and-place
机器人: 6-DOF Comau SMART-SiX 工业机械臂（固定基座）
末端动作: SCARA-type 4D 运动
  3D 平动 (r∈R³)
  + 绕竖直轴 1D yaw rotation (θ)
载具: 末端 EE 上挂 tray，tray 上摆 NC 个圆柱容器（各容器距旋转轴距离 d_i 不同）
液体: 水（低粘度，最难抑制晃动）
执行模式: prehensile（容器与 tray 刚性约束，不离开 tray）
```

与 Scout 的根本差异（在借鉴前必须明确）：

```text
机械臂 SCARA              Scout Mini 移动底盘
─────────────────────────────────────────────────
6-DOF 关节 + 工作空间盒    差速底盘 (v, ω)，工作空间是导航地图
EE 加速度可任意指定       底盘加速度受地面/牵引/质心约束
3D 平动 + θ_z (4D)         平面 (x, y, θ_z)，z 固定
固定基座，无避障          全局路径来自 MBF，含障碍/clearance
prehensile（夹持）        non-prehensile（开口杯放在托盘上）
执行前完整生成轨迹        在线 plan + 在线 track
GoPro 视觉提取真液面       /slosh/height 模型估计 + 实物视觉 TBD
```

特别注意：论文 Table I 明确分类 prehensile / non-prehensile，**Ferrari 的方法属于 prehensile**，而 Scout 的杯子是放在托盘上（non-prehensile）。论文 §V 把 non-prehensile 的"waiter motion"列为 future work，Scout 的设定本质上更接近 future work 的方向。

## 2. Sloshing 建模剖析

### 2.1 MSD 模型选择依据

论文采用的是 mass-spring-damper (MSD) 离散模型，配抛物面约束（[15]、[16]、[17] 路线），不是 pendulum 模型 [6]、[7]、[9]。原因：

```text
MSD 优势:
  η̄ 估计有闭式 / 半闭式表达，比 pendulum 数值积分更便宜；
  与 [16][17] 的实验校准在 1D/3D 平动场景下已验证。

本论文新增贡献:
  把 MSD 从 [16][17] 的 3D 平动扩展到 4D (含 θ_z)，
  显式纳入容器在 tray 上的偏置 P_i，
  给出 EOM 中 θ̈ / θ̇² / 离心耦合项。
```

### 2.2 关键参数与公式

每模态 n 的固有频率 ω_n、模态质量 m_n（Bessel 函数根 ξ_{1n}=1.841）：

```text
ω_n  = sqrt(g · ξ_{1n}/R · tanh(ξ_{1n} · h/R))           式(1)
m_n  = m_F · 2R / (ξ_{1n} h (ξ_{1n}² − 1)) · tanh(ξ_{1n} h/R)   式(2)
```

阻尼比 ζ_n（半经验，含液体粘度 ν 与密度 μ）：

```text
ζ_n = 0.92 · sqrt(ν/μ) / sqrt(g R³)
      · [1 + 0.318/sinh(ξ_{1n} h/R) · (1 + (1−h/R)/cosh(ξ_{1n} h/R))]   式(3)
```

最大液面高度（仅取一阶模态）：

```text
η̄_i ≈ ξ_{1,1}² · h_i · m_{i,1} / (m_{i,F} · R_i) · sqrt(x_{i,1}² + y_{i,1}²)   式(13)
```

**这条式子是全篇最值得借鉴的工程要点**：sqrt(x²+y²) 是模态质量的径向位移，η̄ 与该位移近似线性。这意味着即使不解 MSD ODE，只要在仿真后处理里跟踪模态平面位移，就能得到 η̄ 的可比较代理量。

### 2.3 抛物面约束选择

论文显式说明：[15] 提出非线性弹簧（参数 w=2, α∈[1/2, 2/3]）让模态质量沿抛物面运动，但 [19] 的实验表明 α=0（不加非线性弹簧，仅约束在抛物面上滑动）反而对 η̄ 估计更**保守且准确**。本文采用 α=0。

```text
含义:
  论文不通过更复杂的非线性弹簧拟合，反而用更简单的几何约束得到更好估计。
  这是反复实验后做出的反 Occam 选择 —— 简化反而更稳。
```

## 3. 运动方程（EOM）

LAGrange 方程在 (x_{i,n}, y_{i,n}) 上求导，得到含抛物面 P_i² · x²、x·y 耦合的 2x2 矩阵 ODE：

```text
[1 + P²x²    P²xy   ] [ẍ]   [a]
[P²xy        1 + P²y²][ÿ] = [b]                                  式(11)
```

其中 a, b 是关于：
- 容器加速度 r̈_i,x / r̈_i,y / r̈_i,z（来自 r̈ + 旋转耦合）
- 旋转项 θ̇² · x、θ̈ · y、θ̇·ẏ
- 模态自身刚度阻尼 −ω² x − 2ωζ ẋ

可以看出激励来源有三类，**这点对借鉴非常关键**：

```text
1. 容器质心平动加速度（r̈ + ω̇ × d + ω × (ω × d)）
   → 这是 Scout 平面运动的主要类比，但 Scout 的 ω̇ × d 项小（杯子直接装托盘上方）。

2. tray 角加速度 θ̈ / 角速度平方 θ̇²
   → Scout 的 yaw 是底盘 yaw，θ̇ 即 odom_yaw_rate；
     Scout 容器距 yaw 旋转轴的偏置 = 托盘位置 d，
     这意味着 Scout 也存在 ω² · d 的离心激励，
     当前 GeoRef 计算 ay = v² · κ 其实只覆盖了"路径切向曲率离心"那一项，
     未显式包含 ω̇ × d 这种"角加速度 × 偏置"贡献。

3. 重力项 −g · P · x（来自抛物面下凹）
   → 等价于把模态系统中的恢复力来源吸收到刚度里，对 Scout 同样适用。
```

## 4. 两套优化形式

论文提出两条平行路线，**对 Scout 的研究分支选择有直接对应关系**。

### 4.1 Assigned Path（§III-A）

```text
输入: 路径 r(s) 用 B-spline 控制点 p_j 给定，θ(s) 取 s 的线性插值（式17）
变量: 仅 motion law s(t)（控制输入 u=⃛s）
状态: x = [s, ṡ, s̈, x_1, y_1, ẋ_1, ẏ_1, ..., x_NC, y_NC, ẋ_NC, ẏ_NC]   式(18)
代价:
  min ∫₀^{t_end} (1 + k u²) dt                                     式(19a)
  k = 1e-2，时间最优 + jerk 平滑
约束:
  η̄_i(t) ≤ η_lim         t ∈ [0, t_end]                           式(19e)
  η̄_i(t) ≤ 0.2 η_lim     t > t_end（残振）                         式(19f)
  |u| ≤ u_max                                                      式(19g)
  q̇_j ∈ [q̇_min, q̇_max]                                            式(19h)
```

**关键点**：θ(s) 强制取 s 的线性函数（式17），不让 θ̇ 独立优化。论文给出原因：作 control input 实验下，θ̈ 与 ⃛s 同时优化只能边际改善 t_end，但显著增加 NLP 复杂度。这是工程化简取舍。

### 4.2 Point-to-Point（§III-B）

```text
输入: 起点、终点 + 若干 way-volume（半径 δ 的球）
变量: r(t) 与 θ(t) 完全自由，控制 u = [⃛r, θ̈̇]
约束新增:
  way-volume 通过约束:
    δ² ≥ (r(t_j) − r_j)ᵀ (r(t_j) − r_j),   j = 1, ..., n_v
  workspace box: r_min ≤ r(t) ≤ r_max
  Jacobian condition number κ(J)⁻¹ ≥ k_cond（避免奇异）
```

**与 Scout 的对应**：

```text
论文 way-volumes 概念:
  人为指定路径必须经过的若干球形区域。
  优化器在球内自由选择实际通过点。

Scout Online GeoRef 的对应:
  MBF 给的 raw global path 提供"必经走廊"，
  geometry candidates (mild/medium/strong) 在该走廊内偏离生成，
  collision/drift gate + endpoint check 起 way-volume 约束作用。
```

差别在于：

```text
论文: 把 way-volumes + slosh hard constraint 一起进 NLP，offline 一次性求最优。
Scout: 把 candidate 已经预先生成（候选有限），slosh 模型只用于 score。
论文走的是"continuous optimization in trajectory space"，
Scout 走的是"discrete selection in candidate space"。
```

## 5. 简化策略：仅约束最外侧容器（§III-C）

```text
论点:
  在 NC 个相同容器、相同液位的工况下，
  仅对外侧 IC / EC（距 yaw 轴距离最远的两个）加 η_lim 约束，
  内侧容器自动满足。

证据:
  Fig.3(a): 8 容器全约束 vs 仅 2 外侧约束，⃛s 轨迹"几乎重合"；
  Fig.3(b): 仅约束外侧，内侧 η̄ 全部低于 η_lim。

收益:
  优化复杂度与容器数无关 → t_comp 从 471s 降到 85s。
```

对 Scout 的可借鉴性：

```text
Scout 当前是单容器场景，不直接受益于"外侧最严"。
但若将来扩展到多容器/多排杯子托盘，可以套用此简化。
反向启发更重要:
  Scout 当前 ay = v²·κ 是单点估计，
  若考虑 yaw 旋转的离心 ω²·d 贡献，
  外侧偏置最大处也是激励最严点 —— 单容器情况就是 d = 容器距底盘 yaw 轴距离。
  因此 ay 估计应改为:
    ay_eff = v²·κ + (offset 在车辆 yaw 轴坐标系下的 ω̇ 与 ω² 项)
  这是当前 GeoRef predicted ay 公式潜在的口径偏低问题。
```

## 6. 求解器与计算成本

```text
框架: CasADi + IPOPT
方法: multiple-shooting + RK4 积分
时间离散: 151 sub-intervals
最大迭代: 3000
NLP 误差: 1e-8

实测计算时间（Intel i7 8th gen, 16 GB RAM）:
  1A assigned path: 85 s
  2A assigned path: 307 s
  1B point-to-point: 97 s
  2B point-to-point: 149 s
  8 容器全约束 2A: 471.4 s
  8 容器仅外侧: 85.1 s

Sloshing 单条轨迹仿真（不优化，只前向积分）: ≈ 1 s（MATLAB 实现）
```

**这是把 Ferrari 方法直接移植到 Scout 的最大障碍**：85 秒级 NLP 不能跑在导航循环里。要在 Scout 上启用类似框架，必须：

```text
方案 A: 完全离线规划 + 在线纯执行
  适合工业流水线 / 固定起终点。
  对 Scout 在 maze / open 这种导航场景不可用。

方案 B: 短 horizon 在线 NLP（MPC 形式）
  我们曾在 MPC 里塞 Q_slosh 走过这条路，已被证伪。

方案 C: 离线模型用作"评分器"
  正是当前 docs/Claude/修改方案-时间-简介/2026-05-07_Slosh模型引导GeoRef候选评分方案.md
  的路线 —— 模型只用于已有 candidate 的相对排序，不参与连续优化。
  与 Ferrari 方法是"信息复用，框架不复制"的关系。
```

## 7. 实验验证方法

模型保真度指标：

```text
γ_model = 100 · ∫₀^{1.25 t_end} |η̄_model(t) − η̄_exp(t)| dt
              / ∫₀^{1.25 t_end} η̄_model(t) dt          式(25)

意义:
  在含残振的 1.25·t_end 窗内，模型与视觉提取真液面的归一化偏差。
  γ_model 越小越准；负值表示模型低估，可能漏检超限。
```

优化收益指标：

```text
γ_opt = 100 · (η̄_max,Nopt − η̄_max,Opt) / η̄_max,Nopt   式(26)

意义:
  相对未优化轨迹，最大液面高度的下降百分比。
```

实验值（h=40 mm, η_lim=15 mm）：

```text
Motion 1A IC γ_opt = 52.8%（assigned path opt）
Motion 1B IC γ_opt 接近 1A（path 也优化，但本身较平缓）
Motion 2A IC γ_opt = 64%（最复杂场景，opt 把溢出 η̄≈60 mm 压回 17 mm）
Motion 2B IC γ_opt = 72%（同时优化 path + motion law）
```

**对 Scout 的方法论启示**：

```text
1. γ_model 这种"积分归一化偏差"是论文级 slosh-height 模型验证的合适指标，
   Scout 实物阶段补 RealSense 真液面后应该套这套口径，而不是只看 h_p95 偏差。

2. γ_opt 给出了 baseline / optimized 对比的量化报告口径，
   Scout 当前的"GEOREF vs RAW 三包均值百分比"已经是同一思路，
   写论文时可以引用 Ferrari 这个 γ_opt 形式作为标准化参照。

3. Motion 2A 的 nopt → opt：原本视频里液体直接溢出，模型在该非线性区失效（γ_model 不可信）。
   这说明 MSD 在极端激励下不准 —— 与 Scout 在 maze 场景 raw 自身碰墙
   导致基线不可用是同一类问题：模型/方法都需要先把场景压到线性区。
```

## 8. 与 Scout 系统的对应关系（精确版）

| 论文阶段 | 论文实现 | Scout 当前对应 | 是否可直接迁移 |
|---|---|---|---|
| 任务定义 | 起终点 + way-volumes + η_lim | MBF goal + global path + 容器参数 | 部分 |
| Sloshing 模型 | MSD 4D + 第一模态 + 抛物面 | linear modal model（同源） | 是 |
| η_lim 概念 | 硬约束 (式 19e/f, 23e/f) | 当前无；只用 score | 关键缺失，可借鉴为 hard gate |
| 残振约束 | η ≤ 0.2·η_lim, t > t_end | 当前 SLOSH_SETTLING 已弃用 | 概念可借，实现需新机制 |
| 优化变量 | s(t) 或 r(t),θ(t) | candidate 集合（离散） | 否 —— 框架不同 |
| 求解器 | CasADi+IPOPT, 151 节点, 85–471s | 在线 selector，几 ms | 否 |
| 简化（仅外侧） | 多容器优化加速 | 单容器，不适用；但启发 ay 含 ω²·d | 反向启发 |
| 验证指标 | γ_model, γ_opt + GoPro | h_rms/p95, 当前无视觉真液面 | 是 |

## 9. 可借鉴 vs 不可直接迁移

### 9.1 应当借鉴

```text
1. η̄ 闭式（式13）作为 score 中的 path_terminal_eta_norm 的物理标定。
   当前方案 §4 的 height_coeff 映射可以直接用 ξ²·h·m_n/(m_F·R) 的形式落地。

2. ζ_n 的半经验公式（式3）。
   当前 launch 的 zeta=0.05 是经验拍脑袋；
   有了式(3) 可以从容器 R/h 与水的 ν, μ 物理给出 ζ_n 估计，
   交叉校验或替换 GeoRef rollout 的 ζ 默认值。

3. ω̇ × d / ω² × d 离心耦合的存在性。
   当前 ay = v²·κ 缺少 yaw 加速度 + 偏置贡献，
   建议在 slosh-rollout 里把容器到底盘 yaw 轴的水平偏置 d_offset 显式引入，
   ax/ay 公式扩展为含 ω̇·d 与 ω²·d 项。

4. 残振约束（η ≤ 0.2 η_lim）作为 candidate 评分的额外维度。
   当前 §4.1 的 path_terminal_E 已经反映 path 末端瞬态能量，
   但论文的 0.2·η_lim 是更强的"settling 必须收回到20%以内"定量门槛。
   可以引入 score 的一个软分量：path_terminal_eta / η_lim ≤ 0.2 才不扣分。

5. γ_model / γ_opt 验证指标。
   实物阶段补 RealSense 视觉液面后，论文 §IV-B 这套对比应当复刻，
   是 reviewer 期望的标准报告范式。
```

### 9.2 不能直接迁移

```text
1. CasADi+IPOPT 在线优化 → 不可行（85s 级，导航循环承受不起）。
2. 完整 NLP 形式（式 19、23）→ 不可行（同上）。
3. 仅约束最外侧容器 → 单容器场景不适用；多容器是 future work。
4. way-volume + workspace box → Scout 用 MBF + obstacle map，几何形式不同。
5. Jacobian 条件数约束 κ(J)⁻¹ ≥ k_cond → Scout 是 nonholonomic 底盘，无对应。
6. SCARA θ(s) 线性化（式17）→ Scout yaw 由路径切线方向决定，不是 free 变量。
```

### 9.3 反向启发

```text
1. 论文用 NLP 离线得到时间最优；
   Scout 当前的 vehicle_v_max 是手工拍的固定上限。
   不需要复制 NLP，但可以借鉴"η_lim 反推 v_max"思路：
     给定容器参数 + η_lim，对当前 path 段算出允许的 v_upper。
   这其实是 GEOREF_CONSTRAINED 已尝试的方向，已经走过并 FAIL，
   失败原因恰好是 v_upper hard bound 把激励转到 modal velocity / energy（§13 known risk）。
   论文方法在 prehensile + offline 设定下不会暴露这一失败模式 ——
   它的失败仅在"模型本身在非线性区不准"（motion 2A nopt 工况）。
   对比之下 Scout 的失败是"控制器执行 vs 模型预测的分裂"，
   这是 prehensile→non-prehensile + offline→online 两个跨度共同放大的问题。

2. 论文 motion 2A nopt 视频里液体冲到容器盖上是结构性失效；
   Scout maze 场景 raw 碰墙也是结构性失效。
   两者共同点：方法/模型都要求场景先在"基线可成立"区。
   写论文时这是 limitation 段落的天然类比，
   Scout 应当显式承认 maze 是 out-of-scope，不去硬碰。
```

## 10. 对当前 slosh-model-guided GeoRef 方案的影响

针对 `docs/Claude/修改方案-时间-简介/2026-05-07_Slosh模型引导GeoRef候选评分方案.md`，本剖析建议的具体修订：

```text
M-A. §4 ax/ay 公式补 yaw-offset 项:
  ay_i = v_i² · kappa_i + omega_dot_i · d_offset
  alpha_i 中的 v² · dκ/ds 即 omega_dot 的几何来源，
  需把容器到 yaw 轴的水平偏置 d_offset 作为 prediction 参数显式引入。

M-B. §4 zeta 默认值附加物理标定路径:
  prediction/slosh_zeta_from_physics: bool, default false
  若启用，ζ_n 用式(3) 由容器 R, h 与液体 ν, μ 计算。
  默认仍用 0.05 经验值以保持当前实验复现性。

M-C. §4.1 path_terminal_E 增加配套硬性诊断:
  同时计算 path_terminal_eta_ratio = sqrt(eta_x²+eta_y²) / eta_norm_path_max,
  若 > 0.2 则视为"末端未充分收敛"，在 Step 0 PASS 条件 (e) 上加一条
  saturation floor 不作为唯一退出阈，path_terminal_eta_ratio < 0.2 也作为质量门槛。

M-D. §11 论文定位:
  写法可以从 "Online geometry smoothing" 升级为
  "Online sloshing-model-guided geometric reference selection,
   bridging Ferrari-style offline NLP and real-time mobile navigation"。
  借助 Ferrari 在 prehensile 工业场景的成果作为出发点，
  Scout 工作的差异化贡献是 non-prehensile + 在线 + 移动底盘。

M-E. Step 2 验证表新增 γ_model 列:
  实物阶段必加。
  仿真阶段可用 /slosh/height（模型估计）vs MSD rollout 计算"模型自洽 γ_model_sim"，
  作为 Step 0 模型一致性的额外检查项。
```

## 11. 一句话总结

```text
Ferrari 是"prehensile + offline NLP + 工业机械臂"框架下做时间最优防晃；
Scout 是"non-prehensile + online selection + 移动底盘"框架。
方法论上不可直接迁移；
但 MSD 闭式 η̄、ζ_n 物理公式、ω²·d 离心耦合、残振 0.2·η_lim 阈、γ_model/γ_opt 评测口径
都是可以"信息复用，框架不复制"的零件。
```
