# RA-L 投稿方案：Ferrari/Muchacho 整合设计

撰写日期：2026-05-07
基础剖析：`Ferrari2026_方法剖析.md` + `Muchacho2022_方法剖析.md`
当前进展基础：`docs/Claude/总结/2026-05-06_Online_GeoRef阶段性总结.md`、`2026-05-07.md`
当前在审方案：`docs/Claude/修改方案-时间-简介/2026-05-07_Slosh模型引导GeoRef候选评分方案.md`

本文不是修改方案的再一版细化，而是**站在两篇论文剖析之上，重新审视当前主线，给出一个面向 RA-L 投稿的完整设计**。重点参考 Ferrari 2026 RA-L，因为它是同期同顶刊，方法学语言和 reviewer 期待范式都有直接对照。

---

## 1. 现状清盘（必须先承认的事实）

### 1.1 已成功的部分（可直接进 paper）

```text
Online GeoRef（geometry-only）
  在 open_user_goal 三包 vs RAW 三包：
    h_p95     -18.6%
    eta_dot   -11.5%
    ay_p95    -38.5%
    active_s  +5.5%
  Baseline 切片证明:
    简单降速 RAW_SLOW 不能复现该收益（h_max 反而 +22%, eta_dot +40%）；
    走相同 topic chain 的 GEOREF_ORIGINAL fallback 也不能复现；
    收益归因到 geometry smoothing candidate selection。
```

### 1.2 已 FAIL 的部分（必须写进 limitation 或 negative result）

```text
Q_slosh / Q_eta_dot / terminal slosh cost     无法稳定跨路径压低 /slosh/height
OUTPUT_GUARD / PMG                            output cap 触发闭环补偿与 eta_dot 上升
GEOREF_CONSTRAINED Step 2a x3                 v/a/jerk 一致性改善但 eta_dot/energy/tracking 同时变差
Maze 场景                                     RAW 自身碰墙，不存在干净基线，scope 之外
open_goal_b 单包                              selected=original，无候选可对比，泛化未成立
```

### 1.3 目前手上唯一仍在推进的设计

```text
Slosh 模型引导 GeoRef 候选评分方案（M1–M6 修订版已完成）:
  在 candidate 已通过 hard gate（collision/drift/length/endpoint）后，
  追加线性模态 slosh rollout，按 h_p95_pred / energy / eta_dot / path_terminal_E 排序。
  默认关闭，Step 0 离线 gate 三分支判据（PASS / SATURATED / FAIL）。
```

这条线本身是合理的演进，但存在两个**RA-L 投稿层面**的硬伤——下一节专门讨论。

---

## 2. 当前方案的 RA-L 硬伤

把当前方案放在 Ferrari 2026 旁边并问"reviewer 会问什么"，立刻能列出三条致命缺口：

### 2.1 没有硬安全保证

```text
Ferrari 有: η̄_i(t) ≤ η_lim 是 NLP 显式不等式约束。
            可以写 "the proposed method guarantees η̄ ≤ η_lim by construction"。
我们当前: 只有 score 加权排序。
         无法回答 "What is the safety guarantee that the selected candidate keeps η below threshold?"。

这是 reviewer 期待的标准答案缺失，
不是文字润色能补的，是方法本身缺一条结构。
```

### 2.2 没有残振约束

```text
Ferrari 有: η̄_i(t > t_end) ≤ 0.2 η_lim
我们当前: SLOSH_SETTLING 已弃用，candidate rollout 也不覆盖 settling。

后果: 所有结果都说 "tracking phase 期间 h 下降"，
      但 reviewer 一定会问 "What about residual oscillation after stop?"。
      我们当前数据只能含糊地引用 eta_dot_rms 全程平均，
      无法独立量化 settling 残振。
```

### 2.3 没有 / 难有 visual ground truth

```text
Ferrari 有: GoPro 提取 η_exp(t)，γ_model 公式量化模型保真度。
我们当前: /slosh/height 是模型估计，validation 自洽但没真液面对照。

RA-L reviewer 在 fluid manipulation 类稿件上几乎一定会要求实测视觉，
没有视觉的话方法层面再漂亮，结果章节会被打回 "needs experimental ground truth"。
```

---

## 3. 借两篇论文的"杠杆点"

不是抄方法，是抄它们提供的**方法学话语权**（reviewer 已经熟悉的范式）。

### 3.1 Ferrari 提供的杠杆

```text
LV1. η_lim 硬约束概念（最关键）
     在我们方案里把 candidate 的 hard-gate 集合从纯几何扩展为
     "几何 hard-gate + slosh hard-gate"，
     直接借用 Ferrari 的 η_lim 不等式，作为 candidate 通过条件之一。

LV2. 残振 0.2·η_lim 约束
     在 candidate rollout 末端之后，追加固定时长（如 2s）的零输入自由响应段，
     约束自由响应衰减后的 η̄ 必须 ≤ 0.2·η_lim。
     这一段不需要 NLP，只需要 ODE 自由衰减。

LV3. η̄ 闭式（式13）
     给我们的 score 一个有物理量纲的 mm 级输出，
     而不是抽象的 norm 数。reviewer 看 mm 比看 norm 信任度高得多。

LV4. ζ_n 半经验公式（式3）
     Scout 当前 ζ=0.05 拍脑袋；
     用论文式(3) 由 R, h, ν, μ 给出可复算的 ζ。
     这是直接削弱 reviewer 一票"参数怎么选的"质疑的最低成本动作。

LV5. γ_model / γ_opt 评测口径
     直接套用，paper 里的 Table 复刻 Ferrari Table III 的形式。
     这本身就是论文 SOTA 对照的 buy-in。

LV6. Two-tier optimization 概念
     Ferrari 比较 assigned-path 与 path+motion-law 两版。
     对应到我们就是:
       Tier-A: candidate 来自 GeoRef，slosh 评分只调度 selection；
       Tier-B: candidate 来自 GeoRef，slosh 评分 + ζ/η_lim 一起反馈到 candidate generator
                让其重新生成更好候选（一次反馈循环，不进 NLP）。
     第一版做 Tier-A 即可，Tier-B 留作 future work / discussion。
```

### 3.2 Muchacho 提供的杠杆

```text
LV-M1. slosh-free condition (a_z+g) tan θ = a_x 作为论证逻辑
       用来"证明" Scout 这种 non-prehensile 平台不可能通过几何倾斜消振，
       因此必须从激励源（路径加速度）入手。
       这是引言 / related work 切割本研究价值的关键论证。

LV-M2. r = l/h 模型保真度准则
       给我们 ω_n / ζ 的"模型档位"概念背书：
       低保真档（ω 大、ζ 小）会过滤更多 candidate；
       高保真档（ω 小、ζ 大）容忍更多。
       作为离线 ablation 工具，让 reviewer 看到模型选择敏感度。

LV-M3. QP 实时性论证
       Muchacho 强调他的 QP 1kHz 可解。
       我们应当显式对比:
         Ferrari NLP    85–471 s/trajectory   offline only
         Muchacho QP    < 1 ms/iter           1 kHz
         Ours selection 几 ms/cycle           50 Hz nav loop
       三者在 compute / safety / setup 维度 trade-off，
       这张表能直接立住差异化贡献。
```

---

## 4. 改进框架：Online Slosh-Constrained Reference Selection (OSCRS)

把上面 LV1–LV6 + LV-M1–LV-M3 整合，得到本文最终推荐的框架。命名为 OSCRS（**O**nline **S**losh-**C**onstrained **R**eference **S**election）。

### 4.1 整体架构

```text
MBF raw global path
        │
        ▼
┌─────────────────────────────────────┐
│ GeoRef Candidate Generator          │
│ (existing, geometry-based)          │
│ 输出 N 条 candidate paths C_1..C_N  │
└────────────────┬────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────┐
│ Per-Candidate Slosh Rollout (NEW)  │
│ Inputs: r(s), κ(s), v(s) per ℓ_i  │
│ Model: linear MSD α=0 (Ferrari[19])│
│        ζ from physics (Ferrari [3])│
│ Time domain dt=0.05s, RK2 / 半隐式 │
│ Append 2s zero-input free response │
│ for residual oscillation             │
│ Output η̄_pred(t) per candidate (mm)│
└────────────────┬────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────┐
│ Two-Layer Evaluation (NEW)         │
│ Layer 1 — Hard gate:               │
│   η̄_pred(t) ≤ η_lim   ∀t∈[0,t_end]│
│   η̄_pred(t) ≤ 0.2 η_lim ∀t>t_end │
│   (集合外 candidate 直接淘汰)        │
│ Layer 2 — Score:                    │
│   weighted sum of                   │
│   {h_p95_pred, energy_rms_pred,     │
│    eta_dot_rms_pred,                │
│    path_terminal_E,                 │
│    geometric penalties}             │
│   batch-normalized                  │
└────────────────┬────────────────────┘
                 │
                 ▼ (best-scored that passed hard gate)
┌─────────────────────────────────────┐
│ Fallback Policy (NEW)              │
│ 若 Layer 1 全部淘汰:                 │
│   退回 geometry-only selected,       │
│   并发布 hard_gate_failed=true       │
│   作为运行期 SAFETY ALARM topic     │
└────────────────┬────────────────────┘
                 │
                 ▼
/scout/global_path_anti_slosh
        │
        ▼
Normal MPC tracking (Q_slosh = 0, MPC 不改)
        │
        ▼
/cmd_vel
```

**关键约束**：MPC 部分**绝不再动**。所有创新放在 reference 层的"hard gate + score"上。这是从 GEOREF_CONSTRAINED 失败学到的最重要的纪律。

### 4.2 关键技术决策

#### D1. Candidate generator 是否要改？

```text
不改。
当前 mild/medium/strong 三档 + original 已经能覆盖大多数 open 场景。
本方案的研究问题是"如何更好地从已有 candidate 集合里选"，
而不是"如何生成更好 candidate"。
后者是独立的 future work，本论文不混在一起。
```

#### D2. η_lim 怎么定？

```text
分两步:
  第一步（容器物理层）:
    使用 Ferrari 式(13) η̄ ≈ ξ²·h·m_n/(m_F·R)·sqrt(x²+y²)
    把 modal coords 映射回 mm 高度。
    ξ_{1,1}=1.841，h/R 由当前杯子参数算得。
    /slosh/height 当前观测器口径必须与该 closed form 对齐，
    否则验证不可比。

  第二步（任务层）:
    η_lim 取容器最高点到静止液面距离的 60%-80%（Ferrari 用 15 mm 对应 h=40 mm 容器，37.5%）。
    Scout 当前杯子是 100 mm 高、~50% 装载，η_lim 建议从 25 mm 起，
    然后做 ablation: η_lim ∈ {15, 20, 25, 30} 看 candidate pass-rate 变化。
```

#### D3. 残振段怎么实现？

```text
candidate rollout 末端 t_end 之后追加 t_settle = 2s 的零输入自由响应。
ODE 离散化：
  ax(t) = ay(t) = 0 for t > t_end
  η̄_pred(t > t_end) 由模态自由振荡 + ζ_n 阻尼自然衰减得到。
不引入新参数（除 t_settle 一项）。
作为 hard gate Layer 1 的第二条（残振 ≤ 0.2 η_lim）。
```

#### D4. 是否要 Ferrari-NLP 离线 oracle 作 baseline？

```text
强烈推荐做。这是 RA-L 评审区分"工程 trick"和"方法贡献"的最有力证据。

实现路径:
  把 Ferrari §III-A 的 assigned-path 优化对 Scout 设置改写:
    输入: MBF raw path (s, r(s))
    变量: s(t) 单一 motion law (跟 Ferrari 完全同构)
    模型: 同一 MSD α=0 + 同一 ζ_n
    约束: η̄(t) ≤ η_lim, residual ≤ 0.2 η_lim, |v| ≤ v_max, |a| ≤ a_max, |jerk| ≤ jerk_max
    求解: CasADi + IPOPT, 离线

  期望产出:
    每条 path 对应一条"理论上 time-optimal 且满足 η_lim"的 v(t),
    作为 OSCRS 离散选择的"上界 oracle"。

  报告口径:
    Ours OSCRS vs Oracle:
      η-compliance 一致（都满足 η_lim, residual）
      tracking_time 差距 X% （Oracle 必然更快，因为连续优化）
      compute time: Oracle 数十秒-数百秒, OSCRS 几 ms
    立得住的论点:
      "OSCRS gives near-oracle slosh-compliance at orders-of-magnitude lower compute."

  工作量评估:
    CasADi MATLAB/Python 端 NLP 写起来 1-2 周，
    + 解析积分链 1 周，
    + Scout 几何/动力学约束 0.5 周，
    Total ~ 3-4 周。
    这是 RA-L 论文级方法对照的标准投入。
```

#### D5. 真液面视觉怎么补？

```text
问题: 没有视觉 ground truth 是 RA-L 致命短板。

可选方案:
  V1. RealSense D435i 顶视容器内液面
      投影 + Hough/边缘检测 提取液面线
      转换为 η_exp(t)
      工作量: 标定 + 处理管线 ~ 2-3 周。
      仿真不需要这个;但实物不可绕过。

  V2. 高速 USB camera + ArUco / 印色带 + 计算机视觉
      Ferrari 用 GoPro 离线后处理。
      Scout 板载需要在线/近实时, 处理可放到笔记本。
      工作量与 V1 类似但更易上手。

推荐 V2 优先, V1 备用。
```

#### D6. 验证场景能跨多少个？

```text
当前 open_user_goal 单一目标已成立。
open_goal_b 单包正向但 selected=original，不能算真泛化。
maze 场景 RAW 不安全，不可作为对照基线。

新增推荐:
  S2. open_goal_c   选与 open_user_goal 起终点不同的另一条 open path。
       要求 selected 非 original，否则 candidate 集合空。
  S3. open_long_path  大于 12 m 的较长 path, 测试方法在长 path 上 saturation 表现。
  S4. open_with_obstacle_pair  开阔区一对障碍夹住的 path,
       要求 candidate 能在保持 collision gate 的同时找到低激励选择。

最少需要 4 个 open 场景目标 + RAW/GeoRef-only/OSCRS 三 condition,
每条 5 包 ⇒ 60 包仿真录包。
2-3 周可完成。
```

---

## 5. 论文论证骨架

按 RA-L 标准长度（6-8 页）规划章节。

### 5.1 Title 候选

```text
T1. "Online Slosh-Constrained Reference Selection for Non-Prehensile
     Liquid Transport on Mobile Robots"

T2. "Bridging Offline NLP and Real-Time Path Selection for
     Anti-Sloshing Mobile Manipulation"

T3. "Discrete Approximation of Slosh-Constrained Trajectory Optimization
     for Online Mobile Liquid Transport"
```

T1 最直接表达贡献，推荐主选；T3 把"近似"含义点明，更学术。

### 5.2 Contributions（RA-L 鼓励 ≤3 条）

```text
C1. An online reference-layer framework that enforces
    sloshing-height and residual-oscillation hard constraints
    on a discrete candidate space, providing a non-prehensile
    counterpart to Ferrari et al. (2026)'s offline NLP
    approach for the prehensile, fixed-base setting.

C2. A unified slosh-height predictor that combines
    Ferrari's MSD-α=0 closed form (η̄ in mm) and
    physics-derived damping ratio with per-candidate
    forward simulation including a residual-oscillation
    settling segment. This makes the proposed evaluation
    directly comparable to state-of-the-art prehensile methods.

C3. Comprehensive evaluation on the Scout Mini platform
    in simulation and real-world settings, including
    visual ground truth via overhead camera, comparison
    against geometry-only and offline-NLP-oracle baselines,
    and explicit failure-mode disclosure.
```

### 5.3 章节布局

```text
I.   Introduction
       - 问题背景: liquid transport on mobile robots
       - 与 Ferrari/Muchacho 的差异化（prehensile vs non-prehensile,
         offline vs online, fixed-base vs mobile）
       - 本研究的三条贡献

II.  Related Work
       - Filter-based / shaping methods（Singer 1990 等）
       - Constrained optimization (Ferrari 2026, Muchacho 2022, Reinhold 2019)
       - Geometry-only path post-processing (我们之前的 GeoRef 工作)
       - Negative results landscape:
           Q_slosh-as-cost, OUTPUT_GUARD, GEOREF_CONSTRAINED
         (这一段可作为引言之外的独立 subsection,
          也可压缩为 introduction 末尾的 paragraph)

III. Slosh Model and Sloshing-Height Predictor
     A. MSD α=0 model recap (Ferrari [19])
     B. Sloshing-height closed form
     C. Physics-derived damping ratio
     D. Per-candidate rollout: time-domain dt=0.05s, RK2,
        residual-oscillation settling segment

IV.  Online Slosh-Constrained Reference Selection (OSCRS)
     A. Architecture
     B. Two-layer evaluation
     C. Hard gate: η_lim and 0.2 η_lim residual
     D. Score: weighted geometric + slosh terms
     E. Fallback policy when hard gate filters out everything

V.   Experimental Setup
     A. Scout Mini platform, container, liquid
     B. Visual ground truth via overhead camera
     C. Baselines: RAW, GeoRef-only, OSCRS, Ferrari-NLP-oracle
     D. Metrics: γ_model, γ_opt (Ferrari Table III 复刻),
        plus h_rms/p95/max, eta_dot_rms, modal_energy_rms,
        ay_p95/ax_p95, tracking_time, candidate distribution

VI.  Results
     A. Simulation: 4 open scenarios × 4 conditions × 5 bags
     B. Real-world: 2-3 scenarios × 4 conditions × 3 bags
     C. Comparison with Ferrari-NLP oracle
     D. Sensitivity: η_lim ∈ {15,20,25,30} mm, ζ from physics vs default
     E. Failure modes: maze (out of scope), saturation regime,
        candidate-empty fallback

VII. Discussion
     A. Why hard gate works where MPC cost did not
     B. Discrete-vs-continuous gap (limit of candidate density)
     C. Limitations: open scenarios only, simple containers,
        non-prehensile assumption
     D. Future work: candidate generator co-design (Tier-B),
        full 6D motion (waiter motion 风格), 多容器扩展

VIII. Conclusion
```

### 5.4 与 Ferrari 论文的对仗写法

可以在 Introduction 直接写：

```text
"We address an under-explored setting in fluid manipulation:
non-prehensile liquid transport on mobile platforms, where
the container cannot be tilted by the robot. This setting
breaks the slosh-free condition (a_z+g) tan θ = a_x exploited
by recent prehensile, fixed-base approaches [Ferrari 2026,
Muchacho 2022], and renders offline NLP-based time-optimal
trajectory planning [Ferrari 2026] inapplicable due to the
online replanning required by mobile navigation."

"Our key insight is that, while continuous trajectory
optimization is intractable online, the same η_lim hard
constraint that drives Ferrari's NLP can be projected onto
a discrete candidate space generated by an online geometric
path post-processor. The resulting selection problem is
solvable in milliseconds while preserving the safety
guarantee on predicted sloshing height."
```

这是把两篇论文同时引用进来、把 Scout 工作的位置一次性钉死的标准写法。

---

## 6. 可行性评估

### 6.1 时间预算（假设 2026-08 投稿）

```text
现在: 2026-05-07
投稿目标: 2026-08-01（保守留 1 个月修改与提交）

可用净时间: ~12 周

里程碑（粗排）:
  W1-W2:   OSCRS 架构实现（candidate slosh rollout + two-layer evaluation + fallback）
  W3-W4:   仿真验证矩阵 4 场景 × 4 条件 × 5 包 = 80 包 + 分析
  W3-W4:   并行: Ferrari-NLP oracle 实现（CasADi）
  W5-W6:   Ferrari-oracle vs OSCRS 仿真对照
  W4-W6:   并行: 视觉 ground truth pipeline 搭建
  W7-W8:   实物部署 + 实物录包
  W9-W10:  数据整理 + 论文初稿
  W11-W12: 修改 + 投稿
```

最大不确定性在 W3-W4 的 Ferrari-oracle 和 W4-W6 的视觉管线，
这两条任意一条延期就会压缩论文写作时间。

### 6.2 风险清单

```text
R1. open 场景可挖空间已饱和（Step 0 SATURATED 分支）
    后果: OSCRS 仿真上不显著优于 geometry-only
    应对: 论文可改为 "η_lim 安全保证" 角度而非 "更优收益" 角度,
         即使数值收益小, hard gate 提供的 by-construction 安全也是贡献。
         对应 contribution 重排 C1 主、C3 次。

R2. Ferrari-oracle 在 Scout differential drive 上不收敛
    后果: 没有 oracle 对照
    应对: 退回到与 geometry-only / RAW 三方对照,
         在 discussion 段说明 Scout 非完整约束使 Ferrari NLP 形式直接迁移困难,
         反而成为本研究 motivation 的另一条支撑。

R3. 视觉管线延期
    后果: 实物没有 ground truth
    应对: 实物只报模型估计 /slosh/height,
         在 discussion 段承认 visual validation 是 limitation,
         并提供仿真侧的 γ_model 形式自洽性指标作为 partial substitute。

R4. 实物 Scout 平台不可用 / 安全问题
    后果: 没有实物验证
    应对: 仿真扩到 8 个场景以上,
         加 sensitivity ablation, 把方法本身的 robustness 立住,
         投稿 letter / RAL+ICRA Option 而非纯 RAL。

R5. saturation floor 触发（Step 0 SATURATED）
    后果: 不进入在线阶段
    应对: 把 OSCRS 的离线分析章节单独写成 "offline analysis paper",
         搭配 Online GeoRef 的现有结果作为应用论文,
         或转向 "non-prehensile slosh-aware planning study" 这一 broader scope。
```

### 6.3 最低投稿标准（即使所有风险都触发）

```text
仿真 4 场景 × 3 条件（RAW / GeoRef-only / OSCRS）× 5 包
+ η_lim sensitivity ablation
+ Step 0 离线分析自洽性
+ 详细 limitation 段（明确 open scenario only, 不宣称泛化）
=> 即使没有 oracle 对照、没有视觉 ground truth, 仍可作为
   IROS / ICRA submission 投出。
   IROS 接受率高于 RA-L, 是合理 fallback。
```

---

## 7. 与"现有 slosh评分方案"的差异

`docs/Claude/修改方案-时间-简介/2026-05-07_Slosh模型引导GeoRef候选评分方案.md` 是一份 **engineering plan**（默认关闭、Step 0 三分支验收、不动 MPC）。

OSCRS 是它的 **paper-grade 升级**，关键 delta：

```text
1. 增加 hard gate Layer 1 (η_lim + 残振 0.2 η_lim)
   现方案只有 score, 不构成 by-construction safety claim.
   OSCRS 把这条加上, RA-L 才能写 contribution。

2. 增加 settling 阶段 rollout
   现方案 §4.1 path_terminal_E 只覆盖 path 末端瞬态,
   OSCRS 追加 t_settle=2s 自由响应段,
   把残振统一进 hard gate 第二条。

3. 显式纳入 Ferrari NLP oracle 对照
   现方案没提 oracle.
   OSCRS 把 Ferrari §III-A 改写到 Scout setting 作为离线上界,
   RA-L 评审需要这个对照来区分 "工程优化" vs "方法贡献"。

4. 视觉 ground truth 必须有
   现方案 §9 提到"如果 RealSense 可用",
   OSCRS 把它升级为强必选项。

5. 论文论证逻辑明确
   现方案 §11 论文定位过于宽泛,
   OSCRS 给出三条 contribution 与对仗写法,
   并在 paper 章节布局上预先排好位置。

6. 风险与 fallback 全列
   现方案没有 fallback paper venue 讨论,
   OSCRS 把 SATURATED / oracle 不收敛 / 视觉延期 / 实物不可用
   四条主要风险全部对应到 paper 形态调整。
```

实操层面：

```text
现方案 = OSCRS 的 Step 0 离线诊断 + Step 1 在线接入。
OSCRS = 现方案 + (Layer 1 hard gate) + (settling rollout) + (Ferrari oracle baseline) + (visual GT)。

并不互相替代, 现方案是 OSCRS 实现路径的前两步。
现方案如 PASS 即可向 OSCRS 推进; 如 SATURATED 则按 OSCRS §6 R5 回退到离线分析 paper。
```

---

## 8. 立刻可执行的下一步

按 OSCRS 路径，三件事最先动：

```text
A. Step 0 离线诊断脚本（与现方案 Step 0 同口径，但额外算 η̄ in mm）
   预计 2-3 天。
   交付物: 复用现有 GEOREF 五包 + open_goal_b 三包,
          算每条 candidate 的 η̄_pred(t)、residual η̄_pred(t>t_end),
          检查在 η_lim ∈ {15, 20, 25, 30} mm 各档下:
            (i)  能否找到非空 candidate 集合
            (ii) 当前 geometry-only selected 是否在该集合内
            (iii) Step 0 PASS / SATURATED / FAIL 三分支判据值

B. Ferrari §III-A 在 Scout setting 的离线 NLP oracle 写起
   预计 3-4 周, 与 A 并行。
   交付物: open_user_goal 上一条 v(t) oracle 轨迹,
          可与 OSCRS 选择结果做 tracking_time / η_max / γ_opt 对比。

C. 视觉 GT 管线方案选定 (V1/V2/其他) 并下硬件单
   预计 1 周决策, 4-6 周硬件到货+标定。
   交付物: η_exp(t) 提取脚本 + 一条 demo bag 上的 model vs exp 对比图。
```

A 是必做且最小代价；B 是 RA-L 加分项；C 是从 IROS 升 RA-L 的关键。

---

## 9. 一段总结

```text
当前 Online GeoRef 主线在仿真 open 场景已经成立, 但缺三件事使其无法进 RA-L:
hard 安全保证、settling 阶段约束、视觉 ground truth。
Ferrari 2026 给出了这三件的 prehensile/offline 模板, Muchacho 2022 给出了实时性论证。
本方案 OSCRS = 在 reference 层把 Ferrari 的 η_lim 与残振硬约束用 candidate hard gate 形式落地,
保留 Online GeoRef 的离散 / 在线 / 移动 / non-prehensile 差异化 contribution,
并以 Ferrari NLP 离线 oracle 作为方法对照、视觉相机作为实测 ground truth。
12 周时间表里 A/B/C 三条立刻可启的工作覆盖了从 SATURATED 到 RA-L 的全部分支。
```
