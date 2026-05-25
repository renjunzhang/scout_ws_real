# 2026-05-25 SloshPriorityMPC 论文 baseline 对比实验计划

## 0. 目标

当前论文不需要证明“我们的导航 planner 比所有 planner 强”，而是证明：

```text
普通平滑/限加速度/限 jerk 方法只能间接降低液体激励；
SloshPriorityMPC 显式把液体模态响应放进目标函数，因此在同等固定路径任务中更适合液体搬运。
```

因此 baseline 应优先选择“会平滑运动、降低加速度或 jerk，但不建模晃液”的方法。

## 1. 最终 baseline 定位

正文主表采用四个方法：

```text
TOPPRA-style
Ruckig-style
Smooth MPC / E
SloshPriorityMPC / F
```

其中 TOPPRA-style 和 Ruckig-style 负责“外部论文方法对比”，Smooth MPC/E
负责“同框架公平对比”，SloshPriorityMPC/F 是最终方法。

C/D 不进正文主表，作为 ablation：

```text
C: normal tracking MPC reference
D: slosh modal cost only
```

TEB/DWA 不进正文主表，只放 appendix 或 related work。原因是它们会改变几何路径，
不适合和 fixed-path retiming 方法混在同一个因果结论里。

## 2. 方法分层

| 层级 | 方法 | 建议定位 | 原因 |
|---|---|---|---|
| 正文主 baseline 1 | TOPPRA-style retiming | 给定路径速度重定时 baseline | 外部论文方法；路径不变，只改速度、加速度约束 |
| 正文主 baseline 2 | Ruckig-style retiming | jerk-limited smooth-motion baseline | 外部论文方法；明确限制速度、加速度、jerk |
| 正文主 baseline 3 | Smooth MPC / E | 同框架公平 baseline | 同一 MPC、同一路径、同一 terminal，只关闭 slosh cost |
| 正文主方法 | SloshPriorityMPC / F | ours | slosh-aware + excitation smoothing |
| ablation | C / D | 内部消融 | C 是 normal tracking reference；D 证明 slosh modal cost only |
| 工程 baseline | TEB | ROS1 smooth local planner baseline | ROS1 Noetic 下标准工程对照，但会改变几何路径，归因不如 fixed-path retiming 干净 |
| 可选下界 | DWA | 传统 ROS navigation baseline | 经典但不够 smooth，只适合作为传统下界或 appendix |
| related work | RTEB | 近期 TEB 扩展 | 2024/2025 新工作，重点是鲁棒 replanning，不适合作为当前固定路径主实验 |
| related work / future baseline | TOPP-DWR | 差速机器人 TOPP 新方向 | 很贴题，但若没有成熟实现，本轮不强行复现 |

## 3. 双任务实验结构：学习 Ferrari 的 Assigned Path / Point-to-Point

参考 Ferrari 2026 的两类问题设置：

```text
Assigned Path:
  几何路径已给定，只优化/比较 motion law。

Point-to-Point:
  只给起点、终点和通行约束，轨迹几何有一定自由度。
```

对应到 Scout：

| Ferrari 结构 | Scout 实验任务 | 作用 |
|---|---|---|
| Assigned Path | 固定 P2_s_curve 路径跟踪 | 主消融；几何路径不变，最容易归因到速度、加速度、jerk、slosh cost |
| Point-to-Point | 同起点/终点导航任务 | 工程泛化；允许 TEB/DWA/普通导航策略发挥，但归因不如固定路径干净 |

### 3.1 Task A：固定轨迹跟踪（Assigned Path）

这是论文主结果。

```text
输入:
  同一条 P2_s_curve 几何路径。

比较:
  TOPPRA-style acceleration-limited retiming
  Ruckig-style jerk-limited retiming
  Smooth MPC
  SloshPriorityMPC

评价:
  RGB p95 / peak / RMS
  ax_p95 / ay_p95 / jerk_p95
  duration
  tracking error

优点:
  路径不变，变量少，能清楚回答“运动激励策略是否降低晃动”。
```

### 3.2 Task B：点到点任务（Point-to-Point）

这是工程泛化或 supplementary。

```text
输入:
  同一起点、终点；
  同一地图与定位；
  同一最大速度/加速度/安全参数；
  不启用 OSCRS / GeoRef。

比较:
  TEB
  DWA（可选传统下界）
  Smooth MPC / normal MPC
  SloshPriorityMPC

评价:
  RGB p95 / peak / RMS
  path length
  duration
  tracking / final error
  ax/ay/jerk

注意:
  因为几何路径会变化，Task B 不用于证明 slosh cost 的干净因果贡献；
  它只说明在真实导航任务中，方法是否仍有工程价值。
```

### 3.3 论文中两个任务的角色

```text
Task A:
  主表、主图、主结论。
  用来证明 explicit slosh awareness beyond generic smoothing。

Task B:
  工程验证 / supplementary。
  用来说明方法不是只在人工固定路径上有效。
```

不建议倒过来。点到点任务看起来更接近真实应用，但路径几何差异会让审稿人质疑“液面下降是不是因为路径更简单”。

## 4. 正文主实验组合

正文主表固定为这 4 类：

```text
B1: TOPPRA-style acceleration-limited retiming
B2: Ruckig-style jerk-limited retiming
B3: Smooth MPC
Ours: SloshPriorityMPC
```

其中：

```text
TOPPRA-style 和 Ruckig-style:
  外部论文 baseline，提供权威对比。
  代表”不同范式”：速度规划层而非控制器目标函数层。
  与 Ours 共享同一 MPC tracking controller，隔离 speed planning effect。

Smooth MPC / E (= Comfort-oriented MPC):
  同框架 baseline，排除”只是 MPC 平滑调参”的质疑。
  有明确文献支撑：该配置对应 comfort-oriented / smooth-tracking MPC paradigm，
  即通过惩罚加速度及其变化率（R_a / R_da）降低运动激励，但不建模液体。
  论文中不称”内部调参”，而是引用该范式的代表论文后写成：
    “Following the comfort-oriented MPC paradigm [cite], we configure a
     non-slosh-aware variant that penalizes acceleration (R_a) and control
     rate (R_da) to reduce excitation, without explicit liquid modeling.”

SloshPriorityMPC / F:
  最终方法。
```

### 4.1 Smooth MPC / E 组的文献支撑

E 组配置（`Q_slosh=0, R_a=1.0, R_da=2.0`）并非随意拍参数，而是遵循
comfort-oriented / smooth-tracking MPC 范式。以下论文可用于引用支撑：

| 论文 | 发表 | 引用要点 |
|---|---|---|
| GMPC: Geometric Model Predictive Control for Wheeled Mobile Robot Trajectory Tracking | 2024 | wheeled mobile robot MPC tracking controller，强调比普通 NMPC 更平滑 |
| MPC Based Car-Following Control for Electric Vehicles Considering Comfort | SAE 2023 | 目标函数优化 acceleration 和 acceleration change rate，本质是 comfort / low-jerk MPC |
| MPC-Based Routing and Tracking Architecture for Safe Autonomous Driving in Urban Traffic | 2024 | 多目标 MPC 含 tracking、comfort、lateral acceleration safety constraint |
| Convergent wheeled robot navigation based on an interpolated potential function and gradient | Robotics and Autonomous Systems 2024 | 非完整轮式机器人 MPC，显式考虑 speed and acceleration constraints |
| MPC-Based Dynamic Velocity Adaptation in Nonlinear Vehicle Systems | 2024 | 控制器层面动态调整速度，强调车辆动态约束和真实系统 |

论文中推荐引用 1-2 篇最贴近的（优先选 wheeled mobile robot + comfort / acceleration 惩罚的），
不需要全引。写法示例：

```text
正文 Methods 或 Experimental Setup:
  “As a non-slosh-aware baseline, we configure a comfort-oriented MPC variant
   (Group E) that follows the paradigm of penalizing acceleration and
   control-rate variations to reduce motion excitation [cite GMPC 2024,
   cite Comfort MPC 2023]. This variant uses the same tracking controller,
   path, and terminal configuration as SloshPriorityMPC but removes the
   modal slosh-state penalty (Q_slosh = 0).”

Related Work:
  “Comfort-oriented MPC has been applied to vehicle and mobile robot tracking
   [cite 2-3 papers], aiming to reduce acceleration and jerk for passenger
   comfort or payload safety. These approaches indirectly reduce sloshing
   excitation but do not model liquid dynamics explicitly.”
```

注意：不要写成”我们实现了 GMPC 并做了对比”。E 组是你自己的 MPC 调参，
文献只提供**范式依据**（证明 R_a/R_da 配置不是随意拍的，是有理论基础的）。

可选补充：

```text
C: normal tracking MPC reference
D: slosh modal cost only ablation
TEB/DWA: appendix 或 related work
```

## 5. 与当前 C/D/E/F 的关系

当前内部 ablation：

| 内部组 | 论文定位 | 是否进主表 |
|---|---|---|
| C | Normal tracking MPC reference | 否，放 ablation/reference |
| D | Slosh modal cost only | 否，放 ablation |
| E | Smooth MPC / non-slosh-aware smooth-motion baseline | 是 |
| F | SloshPriorityMPC | 是 |

外部 baseline 映射：

| 论文 baseline | 实验实现建议 | 是否需要新代码 |
|---|---|---|
| TOPPRA-style | 固定 P2 路径，生成满足 `v/a` 约束的 `v_ref(s)` | 可能需要离线路径速度剖面脚本 |
| Ruckig-style | 对 P2 路径速度序列做 jerk-limited retiming，输出平滑 `v_ref(s)` | 可能需要离线速度剖面脚本或调用 Ruckig |
| Smooth MPC | 使用 E 组：`Q_slosh=0`，提高 `R_a/R_da` | 已有 |
| SloshPriorityMPC | 使用 F 组：`Q_slosh>0` + 提高 `R_a/R_da` | 已有 |
| TEB | 独立 ROS local planner 跑同起终点 | 需要额外启动/配置，且路径不完全一致 |

## 6. 推荐论文表格结构

正文主表：

| Method | Slosh-aware | Path fixed | Acc limited | Jerk limited | RGB p95 | RGB peak | Duration | Tracking error |
|---|---|---|---|---|---:|---:|---:|---:|
| TOPPRA-style | No | Yes | Yes | No |  |  |  |  |
| Ruckig-style | No | Yes | Yes | Yes |  |  |  |  |
| Smooth MPC / E | No | Yes | Soft | Soft |  |  |  |  |
| SloshPriorityMPC / F | Yes | Yes | Soft | Soft |  |  |  |  |

消融表：

| Method | Role | 目的 |
|---|---|---|
| C | Normal tracking MPC | 普通 MPC 参考 |
| D | Slosh cost only | 证明 slosh modal cost 单独作用 |
| E | Smooth MPC | 平滑控制项单独作用 |
| F | SloshPriorityMPC | slosh + smooth 的组合效果 |

appendix / related work：

| Method | Role |
|---|---|
| TEB | ROS1 engineering baseline，不进正文主表 |
| DWA | traditional navigation lower bound，不进正文主表 |

## 7. 公平性约束

所有 fixed-path baseline 必须保持：

```text
同一 P2_s_curve 几何路径；
同一起点/终点；
同一 terminal d200 配置；
同一 RGB 标定；
同一 HSV；
同一主窗口：TRACKING start -> first terminal - 1s；
OSCRS / GeoRef 全部关闭。
```

TEB / DWA 由于会自己生成局部轨迹，必须单独标注：

```text
engineering baseline, not fixed-path retiming baseline
```

不要把 TEB/DWA 的结果和 fixed-path 方法混成同一因果解释。

## 8. 指标

主指标：

```text
RGB p95
RGB peak
RGB RMS
duration
tracking error
```

解释指标：

```text
model /slosh/height p95 / peak
ax_p95
ay_p95
jerk_p95
mean velocity
path length
cost contribution
```

Ferrari-style 指标：

```text
gamma_opt vs baseline
gamma_model
RMSE
corr
U_p95 / U_max
```

## 9. 引用口径

### Ruckig

定位：

```text
jerk-limited real-time trajectory generation baseline
```

可写：

```text
Ruckig-style retiming constrains velocity, acceleration, and jerk but does not model liquid dynamics.
```

### TOPPRA / TOPP-DWR

定位：

```text
time-optimal path parameterization / fixed-path velocity retiming baseline
```

可写：

```text
TOPPRA-style retiming preserves the geometric path and computes a feasible timing profile under velocity and acceleration constraints.
```

TOPP-DWR 可在 related work 中说明：

```text
Recent TOPP-DWR extends time-optimal path parameterization to differential-drive wheeled robots with angular-velocity and wheel constraints.
```

### TEB

定位：

```text
ROS1 smooth local planner engineering baseline
```

可写：

```text
TEB optimizes local trajectories with respect to execution time, obstacle distance, and kinodynamic constraints, but it is not liquid-aware and may change the geometric path.
```

### Smooth MPC / Comfort-oriented MPC

定位：

```text
same-controller comfort-oriented MPC baseline, with literature-backed R_a/R_da configuration
```

可写：

```text
Following the comfort-oriented MPC paradigm [cite GMPC 2024, cite Comfort MPC 2023],
the Smooth MPC baseline uses the same tracking controller and path as ours but
removes the modal slosh-state penalty (Q_slosh = 0), relying solely on acceleration
(R_a) and control-rate (R_da) penalization to reduce motion excitation.
This isolates the contribution of explicit slosh awareness from generic
comfort-oriented smoothing.
```

不要写成：

```text
"We implement GMPC [cite] as a baseline."
→ 审稿人会查你用的是不是 GMPC 的代码，答案是你用的是自己的 MPC。
```

正确定位是"范式引用"不是"方法复现"：

```text
E 组的 R_a/R_da 配置遵循 comfort MPC 文献里"惩罚加速度及其变化率"的设计范式，
但实现载体是我们自己的 MPC 框架（同 SloshPriorityMPC 共享 solver、model、horizon）。
```

## 10. 实验执行顺序

先把正文主表做干净：

```text
Step 1: Task A 固定 P2_s_curve，完成 E/F n=3 或 n=5
Step 2: Task A 增加 TOPPRA-style acceleration-limited velocity profile
Step 3: Task A 增加 Ruckig-style jerk-limited velocity profile
Step 4: 生成正文主表：TOPPRA-style / Ruckig-style / E / F
Step 5: 补 C/D ablation，用于解释内部机制
Step 6: 如时间允许，Task B 点到点跑 TEB/DWA，放 appendix
```

## 11. 成功标准

最理想：

```text
SloshPriorityMPC / F 相比 TOPPRA-style / Ruckig-style / Smooth MPC / E：
  RGB p95 更低；
  RGB peak 更低；
  completion time 不显著劣化，或劣化可解释；
  tracking error 仍可接受。
```

如果 Ruckig/TOPPRA 已经接近 Ours：

```text
论文主张改成：
  显式 slosh cost 与 generic smooth-motion baseline 相当或更优；
  但 Ours 还能提供可解释的液体模态风险指标和 cost attribution。
```

如果 Ours 只比 Smooth MPC 好，但不比 Ruckig/TOPPRA 好：

```text
保留为 limitation；
强调当前工作证明了 slosh-aware objective 的可行性，后续需要更强的 retiming + slosh 联合优化。
```

## 12. 不建议

```text
1. 不把 TEB 作为唯一主 baseline；
2. 不把 DWA 作为 smooth baseline；
3. 不说“复现 Ruckig/TOPPRA 原论文全部方法”，只说 Ruckig-style / TOPPRA-style baseline；
4. 不把会改变路径的 TEB 与 fixed-path retiming 混在同一个因果结论里；
5. 不再引入 P1/P2 新模型结构。
```
