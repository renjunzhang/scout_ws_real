# 风险自适应 MPC 方案分析（v3）

> 初版 2026-04-14；v2：收紧三处关键口径；v3：补齐六处投稿级实现细节

---

## 0. 总体判断

方向正确，比"固定 Q_slosh"强一个档次，比重构 MPCC 稳定得多。
核心洞察是对的：**把现有的三块松散启发式（soft cost、box constraint、speed governor）用同一个风险指标 ρ_k 统一驱动，从工程拼接变成方法学上闭合的 co-design**。

下面逐节分析，包括真实隐患和需要补的内容。

---

## 1. 贡献 1：风险自适应调度（ρ_k 框架）

### 1.1 好在哪里

ρ_k 的统一作用是这个方案最有力的地方：

```
同一个 ρ_k 同时驱动：
  Q_{η,k}       ← slosh 惩罚强度
  η̄_k           ← 约束上界
  v_ref_eff_k   ← 参考速度折扣
```

审稿人看到的不再是"三个独立补丁"，而是"一个风险度量驱动三个控制参数的分层架构"。
这个包装本身就是一个可以在方法节立住的贡献。

### 1.2 关键隐患：ρ_k 的反馈环与解法

**问题**：ρ_k 的公式里包含 ĥ^pred_max（MPC 预测的最大液面峰值）。但 MPC 预测本身用的是 Q_{η,k}，而 Q_{η,k} 又由 ρ_k 决定。

```
ρ_k → Q_{η,k} → MPC 预测 → ĥ^pred_max → ρ_k  （隐式循环）
```

**解法（必须写进算法框架，不能只在文字里提）**：采用冻结 monitor rollout + 单步延迟的双层结构：

```
每个控制周期 k 的执行顺序：

Step 1（outer loop，低频可 ≤ 控制频率）：
  从上一周期冻结的 monitor rollout 读取：
    ĥ^pred_max_{k-1}，E^slosh_{k-1}
  计算 r_k，u_k，得到 ρ_k
  更新三个调度参数：Q_{η,k}，η̄_k，v_ref_eff_k

Step 2（inner loop，控制频率 40Hz）：
  用 Step 1 输出的参数构建 QP
  求解，输出 cmd_vel
  保存本周期 monitor rollout 供下周期 Step 1 使用
```

这不是缺陷，是分层调度的自然结构。
**论文里需要给出这个 two-step algorithm box**，否则审稿人仍会把它看成"经验调权"。

### 1.3 ρ_k 的形式化（v3 收紧：明确单调有界调度函数）

ρ_k 分两类子项，显式分开：

```
ρ_k = w_r · r_k + w_u · u_k  ∈ [0, 1]

r_k（physical risk，来自冻结 monitor rollout）：
  r_k = w_h · clip(ĥ^pred_max_{k-1} / h_max, 0, 1)
      + w_e · clip(E^slosh_{k-1} / E_max, 0, 1)
      + w_t · clip(Φ_goal(k) / Φ_max, 0, 1)

u_k（excitation uncertainty，来自独立 IMU 观测）：
  u_k = clip(|a_{y,imu} - v·ω| / a_uncert_max, 0, 1)
```

三个调度参数由同一个单调有界函数 s(ρ) 驱动：

```
s(ρ) = σ( γ·(ρ - ρ_0) )     # sigmoid，γ 控制斜率，ρ_0 控制激活阈值

Q_{η,k}      = Q_η^min + (Q_η^max - Q_η^min) · s(ρ_k)
η̄_k          = η̄^max - Δη̄ · s(ρ_k)
v_ref_eff_k  = v_ref_k · (1 - β · s(ρ_k))
```

**为什么要明确 s(ρ) 的单调有界性**：

- 单调性保证"高风险 → 更保守控制"，是控制器安全性的基本属性，审稿人会验证
- 有界性保证 QP 参数始终在物理有意义范围内（Q_η ≥ 0，η̄ > 0，v_ref > 0）
- 用 sigmoid 而不是线性：避免 ρ_k 小扰动导致 QP 参数剧烈跳变（防 chattering）
- 只要 s(·) 是固定的已知函数，整个调度层就是**参数化调度而非在线优化**，QP 结构不变

如果审稿人问"为什么不直接把 ρ_k 放进 QP 作为优化变量"，答案是：
**放进 QP 会让问题变成 bilinear，破坏凸性；当前方案在外层用 ρ_k 参数化 QP，内层仍是标准二次规划。**

### 1.3.1 防 chattering：变化率限制（v3 补）

sigmoid 本身只保证输出连续，但 ρ_k 本身可能在相邻周期间跳变（急转弯入弯时 u_k 突增），
导致 Q_{η,k} 和 η̄_k 在一个控制周期内大幅跳变，反而把底盘激励出去。

**必须加变化率限制**，对三个调度参数分别设置每周期最大变化量：

```
ΔQ_η_max    = (Q_η^max - Q_η^min) × rate_limit_per_step
Δη̄_max      = Δη̄ × rate_limit_per_step
Δv_ref_max  = β × v_ref × rate_limit_per_step

Q_{η,k}    = clip(Q_{η,k}^desired,  Q_{η,k-1} ± ΔQ_η_max)
η̄_k         = clip(η̄_k^desired,     η̄_{k-1} ± Δη̄_max)
v_ref_eff_k = clip(v_ref_eff^desired, v_ref_eff_{k-1} ± Δv_ref_max)
```

`rate_limit_per_step` 建议初始取 0.05~0.10（即每周期最多变化 5%~10% 的满量程）。

**为什么这等价于 jerk-bounded smoothing**：
对 v_ref 加变化率限制，本质上是对参考速度的变化率施加上界，
与轨迹规划里 jerk-bounded reference 的逻辑一致——目的相同：
防止参考信号本身的突变把系统重新激起来。
这个处理必须写进 Algorithm box，否则实车会有 chattering，审稿人也会问。

**r_k vs u_k 的语义必须写清楚**：

`|a_{y,imu} - vω|` 的物理含义是激励输入的不确定性代理，不是液面风险本身。
当它大时，说明机器人处于非稳态运动、IMU 零偏未完全补偿或轮速计滑移。
把它放在 u_k（excitation uncertainty）而不是 r_k（physical risk）里，
保证 r_k 只来自冻结 monitor（不循环），u_k 只来自独立 IMU 观测（不依赖 QP 求解结果）。
两者都不是当前求解器的直接输出，消除了"优化器自己给自己打分"的循环。

### 1.4 权重整定口径

每一项先归一化到 [0,1]（已在公式里用 clip 做）。
初始等权 w=1，通过 ablation 给出最优比例。既有物理量纲依据，又有实验支撑。
γ 和 ρ_0 通过两点标定：定义"低风险基线状态"对应 s(ρ)≈0，"高风险阈值状态"对应 s(ρ)≈1。

---

## 2. 贡献 2：IMU-calibrated excitation estimation

### 2.1 好在哪里

"我们不是估液面，我们是在校准激励输入"——这个定位非常稳。
不需要承诺完整状态观测器闭环，与现有代码接口清晰（只改 `/slosh/omega_est_used` 和 `/slosh/ay_est` 的来源）。

### 2.2 需要补的对比实验

| 配置 | ω 来源 | ay 来源 |
|---|---|---|
| A | odom | odom (v·ω) |
| B | IMU | odom (v·ω) |
| C | IMU | IMU (bias-compensated) |

对每种配置报告：slosh 模型与外部液面测量的相关系数、急转弯段激励估计偏差、对 MPC 最终效果的影响。
这个 A/B/C 对比是贡献 2 的实验主干。

### 2.3 零偏估计必须正式化

当前零偏只到"粗测值 + 静态 TF 入口"，不能以这个状态投稿。
最小可接受的正式方法：

```
静止 N 秒 → 取 IMU ay 均值 → 作为静态零偏 → 运行时实时扣除
温漂 / 姿态引起的动态零偏 → 在 limitation 里说明
```

---

## 3. 贡献 3：终点残余晃动抑制（投稿版：T2 + 参数辨识）

### 3.1 当前最脆弱的地方

当前 near-goal 是外层状态机 `GOAL_STOP_PENDING` → 硬发 0 速，不是 MPC 连续优化到停。
硬停激发液体自由振荡，残余晃动最大。这是论文里的一个真实 gap，也是最容易被审稿人抓住的地方。

**投稿版必须用 T2**，因为"残余晃动被压下去了"这个结论需要可量化的 settling time 数据，
而 T1（只加 terminal cost）输出的数据不够干净，无法单独作为一个贡献点的实验支撑。

### 3.2 T2：settling MPC 模式（投稿版标准）

```
状态机: TRACKING → [进入 goal capture 区] → SETTLING → REACHED

SETTLING 模式规格：
  时域：N_settle 覆盖至少 2 个液体振荡周期，即 N_settle · dt ≥ 4π/ω₀
  代价：
    Q_tracking = 0（不再跟踪路径参考）
    Q_eta 设为 TRACKING 阶段的 Q_eta^max
    Q_etadot > 0（速率收敛）
    Q_v 大（驱动 v→0）
  终止条件：
    |η_x| < ε_η AND |η_y| < ε_η AND |v| < ε_v
    timeout：若超过 T_settle_max 强制进 REACHED
  输出：settling_time（从进入 SETTLING 到满足终止条件的时长）
```

`settling_time` 是这个贡献点的核心指标，必须在论文 Table 里报告：

| 条件 | settling_time 均值 ± 标准差 |
|---|---|
| Q_slosh=0（无抑制） | X.X ± X.X s |
| Q_slosh 固定 | X.X ± X.X s |
| ρ_k 自适应（本方法） | X.X ± X.X s |

### 3.3 为什么 T2 必须和 ω₀/ζ 辨识绑定

T2 的时域设计（N_settle · dt ≥ 4π/ω₀）直接依赖 ω₀。
如果 ω₀ 不准，审稿人会问：你的 settling 窗口是怎么设计的？
因此 ω₀ 和 ζ 的辨识实验不是可选项，是 T2 贡献的前提条件。

---

## 4. 外部液面观测：近似硬门槛（v2 修订）

> **v2 修订**：这一节从"可选增强"改成"近似硬门槛"。原因见下。

### 4.1 为什么不能只靠 IMU

**IMU 是 excitation / platform motion 指标，不是 liquid response 指标。**

如果控制器只是更早降速、更保守转弯，IMU ay RMS 当然会变好；但这不自动等价于"液面峰值真的更小"。
液体控制论文的评审标准恰好卡在这里：你必须说明液体本身更稳，而不只是底盘动作更平滑。

已有移动机器人工作走"仅用 IMU、不直接观测 slosh"路线的也有。
你这篇如果主张"模型增广 MPC + 风险自适应 + 真实液面改善"，却没有外部液面观测，审稿人会问：**你比 IMU-only 路线多出来的建模和优化，到底证明了什么？**

### 4.2 "外部锚定测量"的最低要求

**ToF 阵列不是必须**，但"至少一种经外部标定的 liquid-response 测量"是近似硬门槛。

最低成本方案（已经足够），协议需一次性固定，不能边跑实验边改：

```
硬件：
  容器侧面贴竖向刻度尺，分辨率 1mm，长度覆盖预期液面变化范围（建议 ≥20mm）
  侧视相机：RealSense 彩色通道 或 USB 单目相机，≥ 30fps，固定安装
  曝光：固定曝光时间（不开自动曝光），避免快速液面运动时模糊

同步：
  相机时间戳与 ROS 时钟同步（/camera/color/image_raw 带 header.stamp）
  与 /slosh/height、/imu/data 的时间对齐误差 < 1 帧（< 33ms）

提取脚本（后处理，不要求实时）：
  检测刻度尺区域 ROI（固定像素区域，一次标定）
  在 ROI 内找液面边缘（Canny 或灰度梯度阈值）
  液面高度 = 液面边缘像素位置 × (刻度尺 mm/像素比例)
  静止液面高度作为基准零点，输出相对高度 h_ext_mm

误差标定：
  静止时记录 50 帧，计算 h_ext_mm 的 std，这就是测量噪声基底
  目标：静止噪声 std < 0.2mm（若大于此值，需改善光照或固定安装）
```

这比深度学习估计器更直接，精度更容易向审稿人解释。
现有的 GRU+CNN 模型仍然报告，作为"learning-based estimator"与刻度尺方案的对比。

**一旦协议固定，在任何实验跑之前先验证静止噪声是否达标。** 否则所有后续 peak_mm 数据都是没有可信误差边界的。

### 4.3 指标分层（v2 修订）

**主指标（liquid response）**：
- 外部测量的液面峰值（刻度尺 / 视觉模型）
- 残余晃动 settling time
- 超阈值时间比例 / near-spill / spill violation rate

**辅指标（因果链证据）**：
- IMU ay RMS（说明底盘激励确实更温和）
- IMU 角冲量
- v/ω 平滑度、任务完成时间、tracking error、solve time

**一句话原则**：
> IMU 指标可以当"为什么控制更温和"的因果链证据，不能当"液体本身更稳"的最终效果证据。

---

## 5. 主模态频率与有效阻尼的实验辨识（v2 扩展）

> **v2 修订**：从"ω₀ 标定"扩展为"频率 + 阻尼"联合辨识。

整个 slosh 模型驱动的不只是 ω₀，还有有效阻尼比 ζ。
残余晃动 settling time 对 ζ 尤其敏感，而贡献 3（终点残余晃动抑制）的可信度取决于 ζ 是否准确。

**推荐辨识方法**：sudden-stop 实验 + 时域拟合

```
1. 固定液量，机器人以恒定速度直行，突然停止
2. 用侧视相机记录液面自由振荡
3. 对振荡曲线拟合衰减正弦：A·exp(-ζω₀t)·cos(ωd·t + φ)
4. 得到 ω₀ 和 ζ
5. 重复不同液量，得 ω₀(fill_level), ζ(fill_level)
```

**必须在跑主实验之前做完辨识，T2 的时域设计依赖 ω₀。**

### 液位策略决策（v3：必须明确二选一，不能模糊处理）

**Option 1（推荐）：单液位，全文固定**
- 选定一个液量（例如容器 60% 满），全程使用
- 辨识一组 (ω₀, ζ)，写进论文参数表
- 优点：最稳，没有"多液位如何切换参数"的问题
- 适合：投稿版，scope 收紧

**Option 2：多液位，查表或保守选取**
- 对 2~3 个液量分别辨识 (ω₀, ζ)，给出 ω₀(fill_level) 曲线
- 控制器使用最保守值（最低 ω₀，即最宽 settling 窗口）
- 优点：泛化性更好，更有说服力
- 代价：实验量增加，论文篇幅压力更大

**结论：除非有充分时间，强烈建议选 Option 1。**
论文里写明"单液位固定实验条件"，作为 limitation 而不是缺陷——
"multi-fill-level adaptation"可以放到 future work。

---

## 6. 实验矩阵与实现优先级（v3 修订）

### 6.1 主表：与经典方法对比

主表回答"比已有路线强在哪里"，条件不超过 4 个，每条运行 ≥5 次：

| 条件 | 缩写 | 描述 |
|---|---|---|
| Nominal tracking | NOM | 无任何 slosh 处理，Q_η=0 |
| Input-shaped reference | ISR | 将 v_ref 卷积 ZV shaper（经典 input shaping，作为 baseline） |
| Fixed anti-slosh MPC | FAS | 固定 Q_η，当前已实现的方案 |
| **Proposed full method** | **PROP** | ρ_k 自适应 + IMU 校准 + T2 settling |

这四条覆盖"无抑制 / 经典 input shaping / 固定 MPC / 本方法"，是对已有文献路线的直接对比。

### 6.2 消融表：贡献归因

消融表回答"强出来的部分分别来自哪里"，在 PROP 的基础上逐项去掉：

| 消融条件 | ρ_k 自适应 | IMU 校准 | T2 Settling |
|---|---|---|---|
| FAS（已有基线） | ✗ | ✗ | ✗ |
| +adaptive only | ✓ | ✗ | ✗ |
| +IMU only | ✗ | ✓ | ✗ |
| +settling only | ✗ | ✗ | ✓ |
| +adaptive+IMU | ✓ | ✓ | ✗ |
| **PROP（完整）** | **✓** | **✓** | **✓** |

消融表在 supplementary 或 appendix 里报告，正文只放 1~2 行核心数据。

### 6.3 实现优先级（v3 修订）

| 优先级 | 事项 | 工作量 | 阻塞什么 |
|---|---|---|---|
| **P0** | 固定评价协议（侧视相机 + 刻度尺），验证静止噪声 < 0.2mm | 0.5 天 | 所有 peak_mm 主指标 |
| **P1** | sudden-stop 辨识 ω₀ + ζ，确定单液位策略 | 0.5 天 | T2 时域设计 |
| **P2** | 形式化 ρ_k（r_k+u_k, single-step delay, sigmoid, rate limit） | 1 天 | 方法主贡献骨架 |
| **P3** | 将 Q_η、η̄、v_ref 全部绑定到 ρ_k + 实现 fallback 回退 | 1 天 | C3/C4/PROP 条件 |
| **P4** | 实现 T2 settling MPC 模式 | 1 天 | settling_time 指标 |
| **P5** | IMU ay 零偏正式标定 + A/B/C 实验 | 0.5 天代码 + 1 天实验 | 贡献 2 支撑 |
| **P6** | 跑主表（NOM/ISR/FAS/PROP × ≥5次） + 统计检验 | 2~3 天 | 投稿核心数据 |
| **P7** | 跑消融表 | 1~2 天 | 贡献归因证据 |

---

## 7. Fallback 回退策略（v3 新增）

### 7.1 为什么必须有 fallback

系统承认 `u_k`（excitation uncertainty）可能很大——也就是 IMU 与运动学激励估计存在较大偏差。
既然承认了这一点，就必须定义"偏差过大时系统怎么办"，否则论文里的"输入可信度进入风险调度"这句话就是空话。

没有 fallback，实车上出现下面任何一种情况时系统行为不确定：
- IMU 话题丢帧或频率骤降
- 零偏未锁定时强行进入调度（`/slosh/imu_ay_bias_ready = false`）
- `u_k` 在某段连续 M 周期都超过阈值（可能是 IMU 本身异常）

### 7.2 Fallback 规则（最小实现）

```
触发条件（满足任意一条）：
  (a) IMU 话题超过 T_imu_timeout（建议 100ms）未收到新数据
  (b) /slosh/imu_ay_bias_ready == false
  (c) u_k > u_threshold_high 连续 M 个周期（建议 M=10，u_threshold_high=0.8）

Fallback 行为：
  冻结 ρ_k 调度（Q_η、η̄、v_ref 保持上一个有效值不变）
  OR 回退到固定的 FAS 参数（Q_η = Q_η^fix，η̄ = η̄^fix）
  发布 /risk_scheduler/fallback_active = true（用于 bag 分析时排除异常段）

恢复条件：
  IMU 恢复正常 AND 连续 K 周期 u_k < u_threshold_low（建议 K=20，迟滞）
```

### 7.3 这在论文里的作用

- 方法描述节里只需一段话：说明存在 IMU 监控和安全回退机制，不让异常 IMU 数据破坏调度
- 实验节里用 `/risk_scheduler/fallback_active` 话题标记异常帧，报告"主实验中 fallback 触发率 < X%"
- 向审稿人证明：**系统对 IMU 失效有明确的处理策略，不是在实验室条件下才能跑的**

---

## 8. 论文最终故事线（v3 更新）

> 我们提出了一种用于差速移动机器人液体搬运的**风险自适应防溢控制框架**。
> 在不改变 tracking MPC 主体结构的前提下，定义了一个显式分离物理液面风险（r_k）与激励不确定性（u_k）的统一风险指标 ρ_k，通过带变化率限制的单调有界调度函数协同驱动 slosh 代价权重、防溢约束边界和参考速度折扣；当 IMU 不可信时系统自动回退到固定参数模式。
> IMU 偏差补偿用于校准 slosh 子模型的横向激励输入，而非直接估计液面状态。
> 终点阶段引入 settling MPC 模式，基于辨识的 ω₀/ζ 参数设计专用时域和代价，量化残余晃动消散时间。
> 方法在真实 Scout Mini 平台上以单液位固定条件验证，以侧视外部液面测量作为主指标，IMU 作为因果链辅证；对比 Nominal / Input-shaped / Fixed anti-slosh 三条经典路线，消融实验给出各增量的独立贡献。

---

## 9. 哪些事不要做

- 做 MPCC（路径进度变量）：scope 太大，动摇控制主线
- 把避障 costmap 耦合写成本文贡献
- 声称实现了完整 nonlinear slosh model
- 把监督学习塞进闭环
- 把 IMU ay RMS 当做主验证指标
- 把"外部液面观测"当可选增强（它是近似硬门槛）
