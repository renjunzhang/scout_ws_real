# Macenski 2023 Regulated Pure Pursuit 方法剖析

论文：Macenski, Singh, Martín, Ginés, "Regulated Pure Pursuit for Robot Path Tracking", arXiv:2305.20026v1, 2023（Samsung Research America / Manipal Institute / Rey Juan Carlos University，ROS 2 Nav2 标准实现 [Ref-Macenski2023]）。

剖析日期：2026-05-27
剖析对象：方法学层面的可借鉴点 / 不可直接迁移的边界 / 与当前 Scout SloshPriorityMPC 主线和外部 baseline 选型的对应关系。

## 1. 论文定位与场景

论文不是新提路径跟踪范式，而是在 Pure Pursuit (PP) 家族上做工程化增量改良，目标是 **service robot / industrial mobile robot 在受限、部分可观环境下的安全可执行性**。

```text
任务: 室内导航 / 走廊 / 货架间 / 盲弯
机器人: 差速底盘（Tiago 实验平台）
路径来源: 全局规划器（kinematically-feasible 或 holonomic 搜索路径）
跟踪目标: 已知路径 P(s) 上的 lookahead 点
执行模式: 在 ROS 2 Nav2 框架下作为 controller plugin
```

与 Ferrari / Muchacho 的根本差异：

```text
Ferrari/Muchacho             Macenski RPP
─────────────────────────────────────────────────
离线 NLP / QP                在线纯几何 + 启发式
固定基座机械臂 / SCARA       差速底盘
最小化液面/振动              最小化跟踪误差 + 避撞 + 安全减速
模型物理参数（液体动力学）   无动力学模型，纯几何 + 速度启发式
工业固定路径                 受限环境多路径
```

特别注意：RPP **完全不考虑车辆动力学**（论文 §3 明确说 "Pure Pursuit nor its variations account for dynamic effects of the vehicle"）。这是它的工程取舍：用更简单的算法 + 启发式 cap 换取广泛部署。

## 2. Pure Pursuit 基础（§2）

### 2.1 经典 PP 几何

给定路径 P 上当前距离 robot 最近的点 p_r，沿路径前方取 lookahead 点 p_l，满足：

```text
dist(p_l) = sqrt((x_r - x_l)^2 + (y_r - y_l)^2) >= L          式(2)
```

其中 L 是 lookahead 距离。把 p_l 转到 robot base 坐标系下记为 p_l' = (x_l', y_l')，曲率：

```text
kappa = 2 * y_l' / L^2                                          式(3)
```

这条式子是 PP 算法的核心：**用一段圆弧把 robot 从当前位姿连到 lookahead 点，圆弧曲率给出 omega = v * kappa**。

### 2.2 经典 PP 的硬性短板

```text
1. 固定速度 v_t：算法本身不规定怎么选 v，default 实现普遍用常数。
   service robot 场景下不可接受（盲弯/走廊/接近障碍都该减速）。
2. 高曲率下 overshoot / undershoot：圆弧近似在急转弯时与真实路径偏离。
3. 不建模车辆动力学：无加速度/jerk 约束。
4. 无避撞：纯路径跟踪，靠外部 collision checker 兜底。
```

### 2.3 Adaptive Pure Pursuit (APP, [Ref-7])

APP 的唯一改进：**lookahead 距离与速度线性绑定**

```text
L_t = v_t * l_t                                                 式(4)
```

其中 l_t 是 lookahead 时间增益。意义：高速时往前看得远（避免追不上），低速时近距离精跟踪（避免摆动）。**APP 仍然不规定 v_t 怎么选**，但解决了 PP 在变速场景下的稳定性。

## 3. Regulated Pure Pursuit (RPP) 方法（§3）

RPP 的核心贡献：在 APP 基础上加 **两条速度启发式**，对算法选出的 v_t 做后处理 cap。

### 3.1 整体流程

```text
输入: 全局路径 P, 当前位姿, 当前距离障碍 d_O
1. 路径变换到 base frame: 修剪、转换坐标
2. 选 lookahead 点 p_l, 计算 L_t = v_t * l_t (APP)
3. 计算 kappa = 2 * y_l' / L_t^2 (PP)
4. 两条启发式 cap v_t:
   a) curvature heuristic: 高曲率时降速
   b) proximity heuristic: 近障碍时降速
   v_t' = min(curvature_cap, proximity_cap)
5. omega = v_t' * kappa
6. 时域 collision check（新增）: 把 (v_t', omega) 向前投影 N 秒，检查碰撞
```

### 3.2 启发式 1: 曲率减速（§3 curvature heuristic）

```text
v_t' = { v_t,                kappa > T_kappa
       { v_t / (r_min * kappa),  kappa <= T_kappa            式(5)
```

物理意义：

```text
T_kappa = 1 / r_min        曲率阈值（对应"最小可不减速的转弯半径"）
当 kappa > T_kappa（弯太急）时，按 1/(r_min*kappa) 线性缩放 v
等价于: 急转弯时把切向速度压到 v_t * (r_min / current_radius)
```

**对 Scout 主线的启发**: 这就是一个 path-curvature-based 横向加速度软约束。当前 Scout MPC 的 `tracking_curvature_speed_cap_enable` 默认 false（见 mpc_params.yaml），但内部 PathHandler 的 speed profile 是按类似思路算的（max_lat_accel 约束 v <= sqrt(a_lat_max / kappa)）。**两者的物理本质相同**，只是 RPP 写成线性 cap、PathHandler 写成 sqrt 包络。

### 3.3 启发式 2: 近障碍减速（§3 proximity heuristic）

```text
v_t' = { v_t * alpha * d_O / d_prox,    d_O <= d_prox
       { v_t,                           d_O > d_prox       式(6)
```

物理意义：

```text
d_O    : 当前到最近障碍的距离
d_prox : 触发减速的距离阈值
alpha  : 减速增益（alpha <= 1）

线性减速：距离 = d_prox 时无影响，距离 = 0 时按 alpha 比例降速
```

论文测试过指数和二次形式，结论是 **线性最实用** —— 调参范围宽，不会出现"距离稍小就急刹"。

### 3.4 启发式合成

```text
v_t' = min(curvature_cap, proximity_cap)                     §3 paragraph
omega = v_t' * kappa                                          式(7)
```

注意 omega 用的是 **regulated v_t' 而不是原 v_t**。论文说这样避免 undershoot：原 v 算 omega 而实际用 v_t' 跑，会导致曲率反应不及时。

### 3.5 时域 collision check（§3 最后）

```text
新增: 把 (v_t', omega) 在时间维向前投影 N 秒，圆弧上采点检查 collision
不再像 PP 那样只检查 lookahead 点
理由: 低速时 lookahead 距离很远（几十米/几百秒），不合理；
      时间窗口反而是固定的物理约束。
```

## 4. Nav2 实现细节（§4）

```text
1. PP / APP / RPP 共享同一份实现，只是参数开关不同（节省维护成本）
2. ROS 2 plugin 形式，可热插拔
3. 92% unit test 覆盖率
4. 支持 reversing（cusp 路径自动正反向切换）
5. 提供 "approach goal" 平滑停车（按距离比例缩 v）
6. 提供 minimum velocity threshold（防止 cap 把速度压到完全卡住）
```

**Nav2 工程化痕迹很重**：作者明确说 RPP 不是学术创新而是 "incremental improvement on state of the art with focus on real-world deployed robots"。

## 5. 实验验证（§5）

四个实验：

### 5.1 Path Tracking Experiment（仿真，TurtleBot3）

```text
阶跃路径跟踪:
  PP   tracking error: 0.19 m
  APP  tracking error: 0.10 m
  RPP  tracking error: 0.03 m      （好一个数量级）
```

**关键观察**: RPP 跟踪误差好不是因为算法更"准"，而是因为 **急转弯被 curvature heuristic 压慢了，所以 lookahead 更短、跟踪更紧**。这是 emergent property。

### 5.2 Blind Turning Experiment（实物 Tiago）

```text
盲弯出现障碍物，测平均停车距离:
  PP   0.15 m
  APP  0.16 m
  RPP  0.24 m      （33% 增加，留余量）
```

RPP 因为 proximity heuristic 提前减速 + 时域 collision check 反应更快，避免"刚转过来就撞"。

### 5.3 Confined Corridor Experiment（实物 Tiago）

```text
1.5m 宽走廊 + 0.7m 障碍，slalom 路径:
  PP   tracking error 0.100 m  (路径短切)
  APP  tracking error 0.059 m
  RPP  tracking error 0.052 m  (14% better than APP)
```

RPP 在 final sharp turn 处的 proximity + curvature 双 cap 让机器人减速 → 更远离障碍 → 更紧贴路径。

### 5.4 Full-system Experiment（70m 校园路线）

```text
PP   85.6 s, avg speed 0.661 m/s, tracking error 0.062 m
APP  94.3 s, avg speed 0.675 m/s, tracking error 0.049 m
RPP  88.6 s, avg speed 0.646 m/s, tracking error 0.043 m
```

**结论引人深思**: RPP 在系统级跟 APP 几乎没区别（avg speed 略低 4%）。但作者认为这正说明 RPP 的额外安全特性 "come at little disadvantage" —— 可以在不损失系统效率的前提下提高安全。

## 6. 论文核心局限（§6 + 我的判断）

```text
论文自己承认:
  1. 无车辆动力学建模 - 假设速度命令立刻生效
  2. 路径必须 kinematically feasible（无 Ackermann 转弯半径约束保障）
  3. 高曲率下仍有 short-cut（虽然比 PP 少）

我额外的观察:
  4. 启发式参数 (r_min, alpha, d_prox) 没有理论指导,
     只能靠系统集成商手工调
  5. 两条启发式之间没有耦合 - 走廊 + 急转弯叠加时 cap 可能过度保守
  6. 完全不考虑搭载物（液体/易碎品） - proximity 是对障碍物的，
     不是对车上货物的
```

## 7. 与 Scout SloshPriorityMPC 系统的对应关系

| 维度 | RPP | Scout SloshPriorityMPC |
|---|---|---|
| 范式 | 几何 PP + 启发式 cap | online MPC + slosh modal cost |
| 速度选择 | 后处理 cap（curvature/proximity） | MPC 内部联合优化 |
| 横向 acc 约束 | 通过 curvature heuristic 间接 | 显式 path_handler max_lat_accel + slosh cost |
| 障碍响应 | proximity heuristic 减速 | 不处理（依赖外部全局规划器） |
| 液体感知 | 无 | modal slosh state + cost |
| 终点行为 | 距离线性减速 + min v | 两段式 envelope + capture stop |
| 跟踪误差量级 | 0.03-0.05 m | 路径跟踪不是核心 KPI |
| 计算成本 | O(N) 几何运算，< 1 ms | OSQP QP 求解，~10-30 ms |

## 8. 可借鉴 vs 不可直接迁移

### 8.1 应当借鉴

```text
1. proximity heuristic 的线性形式（式 6）
   Scout 当前没有"近障碍减速"机制（全靠全局规划器避障）。
   如果未来需要补 reactive 层（如 anti-slosh + 动态行人），
   可直接套这条线性减速公式。Ferrari 系不会教你这个。

2. 时域 collision check（N 秒前向投影）替代距离 collision check
   Scout 当前 terminal capture stop 用的是距离阈值（terminal_capture_stop_distance）。
   时域阈值更鲁棒（不依赖 lookahead 远近）。
   可作为未来 terminal recovery 改写的方向（先不做）。

3. 同一份实现支持 PP/APP/RPP 参数化（§4）
   工程上很对：算法变体共享代码，参数开关切换。
   Scout 的 C/D/E/F + TOPPRA/Ruckig 也是这种结构，验证了这条工程范式。

4. minimum velocity threshold
   RPP 提到 cap 可能把速度压到完全卡住，必须有 v_min。
   Scout 的 terminal_capture_v = 0.18 就是同样的角色。
   未来如果加 slosh-aware reactive cap，必须遵循这条。
```

### 8.2 不能直接迁移

```text
1. 把 RPP 当 Scout 的主控制器
   RPP 没有动力学模型，slosh 模态项进不去。
   把 slosh cost 加进 RPP 等于"圆周运动几何 + 液体物理"两套语言搅在一起。

2. 把 RPP 当 paper baseline（控制器对比）
   RPP 是几何 + 启发式，不在 cost-based 框架内。
   要做 "MPC vs 几何控制器" 对比，本身就是不公平比较；
   且 reviewer 会问"你在比方法还是比框架"。
   建议放 related work 一句话：classical pure pursuit lacks dynamics modeling
   and is unsuitable for liquid transport tasks.

3. 时域 collision check 替代 terminal capture stop
   Scout 当前 terminal 行为已经在两段式 envelope + d200 实物上验证通过。
   动它就要重跑 terminal smoke。
   先不动，未来 Nav2 迁移时再考虑。

4. curvature heuristic 替代 PathHandler 的 sqrt 包络
   两者物理等价但参数语义不同。
   切换会影响所有 v_ref，所有 baseline 都要重新调参。
   不值得。
```

### 8.3 反向启发

```text
1. RPP 在 service robot 领域被 ROS 2 Nav2 选为默认 controller，
   说明工业部署看重的不是"最优"，而是"参数好理解 + 启发式行为可解释"。
   Scout SloshPriorityMPC 的论文叙事可以借这一点：
     "We propose an MPC formulation that retains the interpretability of
      heuristic-based controllers like RPP, while providing principled
      handling of liquid dynamics that pure geometric methods cannot model."

2. RPP 的核心 selling point 是 "incremental improvement on APP"
   不是新框架，而是"加两条启发式让 APP 在真实场景能用"。
   Scout 的论文也可以类似定位：
     "incremental improvement on standard MPC for liquid transport,
      by adding modal liquid cost and excitation smoothing penalties"
   反而比 "we propose a new controller" 更稳。

3. RPP 强调 "comes at little disadvantage"（§5.4）—— 不强调"远超所有"
   论文表 4: RPP 跟 APP 系统级几乎相同。作者写成优势:
   "the additional benefits of RPP come at little disadvantage".
   Scout 的论文如果 F 组在 time-matched 下没赢 TOPPRA，可以用同样语调:
     "comparable sloshing reduction at no significant time cost, with the
      added benefit of real-time liquid risk estimation."
```

## 9. 在当前论文实验中的定位

按 `docs/Claude/修改方案-时间-简介/2026-05-25_SloshPriorityMPC论文baseline对比实验计划.md`：

```text
TOPPRA-style / Ruckig-style: 主表 fixed-path retiming baseline（已实施）
Smooth MPC / E:              主表 same-framework baseline（已实施）
SloshPriorityMPC / F:        ours（主线）
TEB / DWA:                   appendix engineering baseline

RPP 的合理定位:
  related work 一句话提到，说明 service robot 主流路径跟踪算法存在但
  不建模液体动力学，因此不适合作为液体运输任务的主 baseline。
  不进正文主表。

可能的引用写法:
  "Geometric path tracking methods such as Pure Pursuit and its variants
   (e.g., Regulated Pure Pursuit [Macenski2023]) are widely deployed on
   service robots, but they lack any vehicle dynamics or payload modeling,
   making them unsuitable for tasks where dynamic excitation must be
   actively controlled, such as liquid transport."
```

## 10. 对 Scout 当前主线的具体影响

针对 `docs/Claude/修改方案-时间-简介/2026-05-25_SloshPriorityMPC对比实验设计.md`：

```text
N-A. §6 主要比较关系
  当前已有 D-C / E-C / F-E / F-D / F-C 五组对比。
  RPP 不进主表，故不需要额外组别。

N-B. §9 引用口径
  Smooth MPC / E 段已经引用 comfort MPC 文献。
  RPP 可作为 related work 段落里"非 MPC 几何跟踪"的代表，
  说明"为什么不选 RPP 做 baseline"，而不是"我们超过了 RPP"。

N-C. 论文结构建议
  Related Work 章节里建议这样组织:
    - Path tracking controllers
      - Geometric: Pure Pursuit family (PP, APP, RPP [Macenski2023])
      - Optimization-based: MPC (LPV, NMPC, GMPC)
    - Smooth motion planning
      - Path retiming: TOPPRA [Pham2018], Ruckig [Berscheid2021]
    - Anti-sloshing
      - Prehensile: Ferrari 2026 (offline NLP), Muchacho 2022 (pendulum QP)
      - Non-prehensile mobile: this work
```

## 11. 一句话总结

```text
Macenski RPP 是 "用两条简单启发式给经典 Pure Pursuit 套上服务机器人安全外衣"
的工程化典范，是 ROS 2 Nav2 主流跟踪 controller 的标准实现。
方法学上简单到不可能与 MPC 直接对比；
但其 "无动力学、无 payload 建模" 的根本短板恰好衬托 Scout
SloshPriorityMPC 在液体运输任务上的差异化贡献。
RPP 应放 related work 而非主表，作为 "service robot 主流但不适用本任务"
的对照说明。
```
