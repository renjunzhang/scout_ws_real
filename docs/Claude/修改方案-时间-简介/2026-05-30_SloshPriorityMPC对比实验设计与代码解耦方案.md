# SloshPriorityMPC 对比实验设计与代码解耦方案

> 创建 2026-05-29, 最近更新 2026-05-30。
> baseline 阵容: 控制器轴 C/D/E/F/RPP-style + 范式轴 TOPPRA/Ruckig/Biagiotti + 附录 TEB。

## 0.0 对比实验参考论文清单

以下条目为 2026-05-30 检索核对后的引用口径。正文中不要把 `*-style` baseline 写成完整复现原论文,
除非后续真的完整复现了原方法的全部系统假设和接口。

| 实验位置 | 方法 / 主题 | 建议引用 | 用法 |
|---|---|---|---|
| 正文主表 / 消融 | C/E/F: MPC 理论根 | Rawlings, Mayne, Diehl, *Model Predictive Control: Theory, Computation, and Design*, 2nd ed., Nob Hill Publishing, 2017. | 支撑 receding-horizon MPC 的通用理论根。 |
| 正文主表 / 消融 | C/E/F: 移动机器人 tracking MPC | G. Klančar, I. Škrjanc, "Tracking-error model-based predictive control for mobile robots in real time," *Robotics and Autonomous Systems*, 55(6):460-469, 2007. DOI: `10.1016/j.robot.2007.01.002`. | 支撑 wheeled/differential mobile robot tracking MPC baseline。 |
| related work 可选 | 近期 WMR MPC | J. Tang et al., "GMPC: Geometric Model Predictive Control for Wheeled Mobile Robot Trajectory Tracking," *IEEE Robotics and Automation Letters*, 9(5):4822-4829, 2024. DOI: `10.1109/LRA.2024.3381088`. | 近期 RA-L MPC tracking 参考, 不作为 nominal MPC 主背书。 |
| 正文主表 | RPP-style | S. Macenski, S. Singh, F. Martin, J. Gines, "Regulated Pure Pursuit for Robot Path Tracking," *Autonomous Robots*, 47(6):685-694, 2023. DOI: `10.1007/s10514-023-10097-6`. | 只支撑 RPP-inspired velocity regulation; 本文不声称完整 Nav2 RPP controller 复现。 |
| 正文主表 / 范式轴 | Biagiotti-style open-loop slosh-aware shaper | L. Moriello, L. Biagiotti, C. Melchiorri, A. Paoli, "Manipulating liquids with robots: A sloshing-free solution," *Control Engineering Practice*, 78:129-141, 2018. DOI: `10.1016/j.conengprac.2018.06.018`. | 主开环 slosh-aware shaping baseline; 本文只用平移加速度整形, 不复现机械臂 vessel-tilting。 |
| related work / Biagiotti 补强 | plug-in feedforward / nonprehensile transport | L. Biagiotti, D. Chiaravalli, R. Zanella, C. Melchiorri, "Optimal Feed-Forward Control for Robotic Transportation of Solid and Liquid Materials via Nonprehensile Grasp," arXiv:`2306.14212`, 2023. | 说明该类 feedforward 方法可作为 reference generator 与 robot controller 之间的 plug-in。 |
| supplementary | TOPPRA-style retiming | H. Pham, Q.-C. Pham, "A New Approach to Time-Optimal Path Parameterization Based on Reachability Analysis," *IEEE Transactions on Robotics*, 34(3):645-659, 2018. DOI: `10.1109/TRO.2018.2819195`. | 支撑固定几何路径下的 reachability-based time-optimal retiming。 |
| supplementary | Ruckig-style retiming | L. Berscheid, T. Kroeger, "Jerk-limited Real-time Trajectory Generation with Arbitrary Target States," *Robotics: Science and Systems*, 2021. DOI: `10.15607/RSS.2021.XVII.015`. | 支撑 jerk-limited velocity/acceleration/jerk constrained trajectory generation。 |
| related work | input shaping 理论根 | N. C. Singer, W. P. Seering, "Preshaping Command Inputs to Reduce System Vibration," *Journal of Dynamic Systems, Measurement, and Control*, 112(1):76-82, 1990. DOI: `10.1115/1.2894142`. | 支撑 input shaping / residual vibration suppression 的理论根。 |
| related work | 液体搬运反馈/鲁棒控制经典 | K. Yano, K. Terashima, "Robust liquid container transfer control for complete sloshing suppression," *IEEE Transactions on Control Systems Technology*, 9(3):483-493, 2001. DOI: `10.1109/87.918901`. | 作为液体搬运 sloshing suppression 经典参考。 |
| related work | 3D path / hybrid shaped approach | K. Yano, K. Terashima, "Sloshing Suppression Control of Liquid Transfer Systems Considering a 3-D Transfer Path," *IEEE/ASME Transactions on Mechatronics*, 10(1):8-16, 2005. DOI: `10.1109/TMECH.2004.839033`. | 支撑液体搬运 3D 路径与输入整形方向。 |
| 附录 P2P | TEB | C. Rösmann, F. Hoffmann, T. Bertram, "Integrated online trajectory planning and optimization in distinctive topologies," *Robotics and Autonomous Systems*, 88:142-153, 2017. DOI: `10.1016/j.robot.2016.11.007`. | ROS1 外部 local planner baseline; 只放 P2P 工程泛化, 不进 fixed-path 主因果表。 |

## 0. 论文对比实验设计

本方案服务的论文对比实验。贡献是**控制器层**改进(闭环 MPC + 模态晃液代价),
baseline 设计围绕"证明每一种替代方案都不够"展开。

### 0.1 贡献与两层架构

```text
贡献: 闭环、slosh-aware 的 MPC —— 把液体模态响应显式放进目标函数。

系统两层 (baseline 分别动不同的层, 不能塞进一张平铺表):
  Layer 1  参考/剖面整形 (跟踪前):    none / TOPPRA / Ruckig / Biagiotti
  Layer 2  跟踪后端 / 速度调节:        ordinary MPC / slosh MPC(ours) / RPP-style speed regulation

方法 = (Layer1, Layer2) 组合:
  C         = (none,        ordinary MPC)          控制器, Q_slosh=0
  D         = (none,        slosh MPC, slosh-only)  控制器
  E         = (none,        smooth MPC)            控制器
  F         = (none,        slosh MPC, ours)       控制器
  RPP-style = (none,        RPP 启发的 v_ref 速度调节 + 同一 MPC 跟踪后端)
  TOPPRA    = (TOPPRA 整形,   ordinary MPC 跟踪)     开环 retiming, 限加速度, 非 slosh-aware
  Ruckig    = (Ruckig 整形,   ordinary MPC 跟踪)     开环 retiming, 限 jerk, 非 slosh-aware
  Biagiotti = (Biagiotti 整形, ordinary MPC 跟踪)    开环整形, slosh-aware (瞄 ω_n)
```

### 0.2 主线:每个 baseline 杀掉一个"也许就够了"

```text
C         也许随便什么 MPC 都行          → 不行, ordinary MPC 晃
E         也许只把控制平滑一下就行        → 不行, smooth MPC 不够
RPP-style 也许朴素弯道减速就行           → 不行
Biagiotti 也许开环、瞄 ω_n 的整形就行    → 不行 (最强开环也输)
TEB(附录) 也许真实标准局部规划器就行      → 不行
F         闭环 slosh-aware MPC          → ours
```

### 0.3 三类 baseline + 实验轴拆分

```text
A 类 控制器消融 (架构不变, 只动 slosh cost): C/D/E/F → 证"增益来自代价项"(最硬因果)
B 类 同框架非晃液速度调节:                  RPP-style → 证"赢朴素弯道/终点速度调节"
C 类 另一范式 (开环整形/retiming):           TOPPRA/Ruckig/Biagiotti → 证"闭环赢开环"

实验轴 (避免"控制器 vs 整形器"架构错配质疑):
  控制器轴: 固定 Layer1=none, 变 Layer2 → C/D/E/F/RPP-style
  范式轴:   固定 Layer2=C 的 ordinary MPC 跟踪, 变 Layer1 → none(=C)/TOPPRA/Ruckig/Biagiotti
            + F 做范式对照
  公平性硬约束: 范式轴里 TOPPRA/Ruckig/Biagiotti 的 shaped 剖面必须用 C 的同一跟踪器执行
              (复用 external_profile_mode + ProfileExecutionCap), 否则混入控制器差异。
```

### 0.4 实验分层(主表精简, 开环阶梯进 supplementary)

```text
正文主表 (控制器故事为主, 范式轴只放最强开环 Biagiotti 当代表):
  C / E / RPP-style / Biagiotti / F        n=5

消融表 (A 类):
  C / D / E / F                            n=5 (C/E/F 复用主表, 仅补 D)

supplementary「开环 retiming/整形阶梯」(用现成脚本, 便宜):
  TOPPRA-style → Ruckig-style → Biagiotti  n=2-3, 展示"越聪明越接近 F, 但都追不上"
  Ruckig 兼做时间匹配检验 (调到 duration ≈ F ±10%, 排除"靠慢赢")

附录 P2P 工程泛化:
  TEB (真实 ROS1 标准规划器, 见 §0.6)      n=2-3, 数据不与主表混读 (路径几何会变)
```

为什么主表只放 Biagiotti:赢**最强开环**(slosh-aware 整形)是最锋利的主张;
TOPPRA/Ruckig 是"更不聪明"的台阶,放阶梯里廉价补足"开环单调趋势"。

### 0.5 C 组 = Nominal MPC 定义 + 参考文献

```text
定义 (版本①, 主用): 同状态 + Q_slosh=0
  状态向量仍含 eta/eta_dot, 预测模型照常传播, 仅 cost 中 slosh 项全关。
  论文表述必须写 "C does not penalize the liquid modal response" (而非 "does not contain
  liquid states") —— 否则与实现 (状态仍在) 不一致。
  理由: C/D/E/F 共享完全相同的模型/状态, 唯一变量是 cost 权重 → 改善只能归因于代价项, 无混杂。
  不用版本②(删除液体状态): 会改状态维数, 引入"模型有无液体状态"的混杂, 反削弱因果。
  ①≈②等价性 (可选 footnote): 零权重状态不影响最优控制, 可一次性跑两版展示 cmd_vel 数值一致,
    以堵"你 baseline 还带液体模型"的质疑, 而无需维护两套代码。

参考文献 (精简, C 是 baseline 不是贡献, 勿堆):
  [必引] Rawlings, Mayne, Diehl, "MPC: Theory, Computation, and Design", 2nd ed., 2017.
         → 标准滚动时域 MPC 定义根
  [必引] G. Klančar, I. Škrjanc, "Tracking-error model-based predictive control for mobile
         robots in real time", Robotics and Autonomous Systems, 55(6):460-469, 2007. (431 引)
         → 标准 WMR tracking MPC 范本, 最对口
  [可选] J. Tang et al., "GMPC: Geometric MPC for Wheeled Mobile Robot Trajectory Tracking",
         IEEE RA-L, 9(5):4822-4829, 2024. DOI 10.1109/LRA.2024.3381088.
         → 命中目标期刊(RA-L); 但属几何/Lie群进阶变体, 只放 related work, 勿用它背书"nominal MPC"
  [不引] Berberich TAC 2022(错位) / Sun 2017(robust/tube MPC, 非 nominal)
```

### 0.6 baseline 候选文献(已核实引用量)

```text
开环 retiming/整形 (C 类范式轴):
  ⭐ L. Moriello, L. Biagiotti, C. Melchiorri, A. Paoli, "Manipulating liquids with robots:
     A sloshing-free solution", Control Engineering Practice, 78:129-141, 2018. (39 引)
     → 液体专用 + 模态模型 + 指数滤波器整形, 复用 ω_n/ζ, 零移植;
       只用其平移加速度整形部分 (差速底盘不能倾摆容器, 论文须注明 vessel-tilting 部分 N/A);
       会议前身 ICRA 2017 feed-forward 版可一并引;
       是"前馈轨迹整形器", 不是反馈控制器 → 属 Layer1 整形 (Ruckig 同类), 非 RPP 同类。
  · H. Pham, Q.-C. Pham, "A New Approach to Time-Optimal Path Parameterization Based on
    Reachability Analysis", IEEE T-RO, 34(3):645-659, 2018.  → TOPPRA-style retiming
  · L. Berscheid, T. Kröger, "Jerk-limited Real-time Trajectory Generation with Arbitrary
    Target States", RSS 2021.  → Ruckig-style retiming
  · N. Singer, W. Seering, "Preshaping Command Inputs to Reduce System Vibration", ASME JDSMC,
    112(1):76-82, 1990. (1872 引)  → 输入整形(ZV/ZVD)奠基, 残振领域理论根
  · K. Yano, K. Terashima, "Robust liquid container transfer control for complete sloshing
    suppression", IEEE T-CST, 9(3):483-493, 2001. (157 引)  → 液体反馈控制经典, related work
  · K. Yano, K. Terashima, "Sloshing Suppression Control ... 3-D Transfer Path",
    IEEE/ASME T-Mech, 10(1):8-16, 2005.  → 输入整形 + 3D 路径, related work

外部控制器 (附录):
  · C. Rösmann, F. Hoffmann, T. Bertram, "Integrated online trajectory planning and
    optimization in distinctive topologies", Robotics and Autonomous Systems, 88:142-153, 2017.
    → TEB; 仓内已有 teb_local_planner (上游 Rösmann, ROS1, nav_core+MBF plugin), 零移植

命名口径: 写 "TOPPRA-style / Ruckig-style retiming baseline", 不写"复现原论文全部方法"
         (实现是离线 -style 脚本); Biagiotti 写"open-loop sloshing-free shaping baseline";
         RPP-style 写"RPP-inspired non-slosh-aware regulated-speed baseline", 不写"RPP controller"。
```

### 0.7 完整 Nav2 RPP 移植:取消

```text
真机 = ROS1 noetic, Nav2 RPP = ROS2 → "上真机"与"零移植差异"物理互斥;
完整外部控制器混杂高 (自带终点/跟踪/频率/限幅), 只能进附录, 而该位有更便宜替身。
决定: 不做 ROS1 移植包。附录"真实外部标准规划器"位用仓内已有的 TEB (零移植, 可引)。
      正文 B 类用框架内 RPP-style (低混杂)。
```

### 0.8 各 baseline 对代码的需求

| baseline | 代码需求 | 状态 |
|---|---|---|
| C / D / E / F | 同一 MPC + Q_slosh/R_a/R_da 切换 | 已有 |
| RPP-style | 框架内 RppSpeedRegulator 模块 (§6) | 本方案新增 |
| TOPPRA / Ruckig / Biagiotti | 离线生成 v_ref(s) CSV + external_profile_mode 注入 (§4.2) | TOPPRA/Ruckig/Biagiotti 脚本已有 |
| TEB | 独立 nav_core plugin, P2P 跑 | 仓内已有, 零移植 |

**结论**: 本方案 §4-§13 的代码改动覆盖 C/D/E/F/RPP-style/TOPPRA/Ruckig/Biagiotti 的全部需求,
共享同一套 launch / record / analysis 链路。

**TEB 不纳入 experiment_group**: 它是独立 move_base/MBF base_local_planner 插件, 在线优化执行时间/
障碍距离/动力学约束, 天然改变局部轨迹, 不能与 fixed-path 主表混入同一因果链。TEB 用**独立 bag 前缀**
`slosh_TEB_P2P_<timestamp>.bag` + **独立 analysis schema**, 仅附录 P2P 使用。

**RPP-style 取 C_default R_a/R_da (不取 E_smooth)**: 它杀的是"弯道减速就够了", E 已杀"平滑就够了";
两者刻意不合并, 否则解释重复。代价是 RPP-style 不是"最强 non-slosh 控制器", 这是可接受的定位。

---

## 1. 背景

当前论文对比实验口径(见 §0)收敛为:控制器轴(C/D/E/F/RPP-style)+ 范式轴(TOPPRA/Ruckig/Biagiotti)
+ 附录(TEB)。因此代码需要从"多种历史实验入口堆叠"改成"少量清晰变体参数切换"。

本方案只讨论 `src/scout_apps/control/scout_local_planner` 的对比实验相关重构,不涉及 OSCRS,不修改 `slosh_models` 的物理模型。

## 2. 目标

### 2.1 代码目标

```text
1. TOPPRA / Ruckig 走 external profile 注入, 作为 supplementary 开环 retiming 阶梯 baseline;
2. Biagiotti 开环 slosh-aware 整形作为范式轴主表 baseline (生成 v_ref(s) CSV, 同一注入接口);
3. 完整 Nav2 RPP controller node 取消移植; 第一阶段实现框架内 RPP-style speed regulator;
4. 用 experiment_group 单一字段 (派生 controller_variant + external_profile_mode) 解耦实验切换;
5. C/D/E/F/RPP-style/TOPPRA/Ruckig/Biagiotti 共用同一套 launch 和 bag 脚本; TEB 走独立 move_base 栈。
```

### 2.2 论文目标

```text
RPP-style:
  不能声称完整复现 Nav2 RPP controller。
  论文中写成: RPP-inspired non-slosh-aware regulated-speed baseline (Macenski et al. 启发)。

Biagiotti:
  open-loop sloshing-free shaping baseline (Moriello et al. 2018), 范式轴最强开环对照。

TOPPRA / Ruckig:
  open-loop retiming baseline (TOPPRA: Pham 2018; Ruckig: Berscheid 2021),
  supplementary 阶梯; Ruckig 兼做时间匹配检验。

TEB:
  附录真实 ROS1 标准局部规划器 (Rösmann 2017), P2P 工程泛化, 不进正文主因果。
```

## 3. 当前必须保留的主线

以下模块是当前 SloshPriorityMPC / 固定路径实验主线，不应在本轮重构中破坏：

```text
cost_function.*
cost_breakdown.*
slosh_feedback.*
terminal_controller.*
profile_execution_cap.*
diagnostics_publisher.*
path_handler.*
local_planner_ros.*
```

不能改动的接口：

```text
/mpc/cost_breakdown 21 字段 layout
/slosh/*
/terminal/*
/reference/*
/profile_cap/*
slosh_experiment*.launch 现有 arg 名
terminal d200 默认实验参数
```

## 4. 新参数设计

### 4.0 experiment_group (权威字段, 单一来源)

`experiment_group` 是 launch 唯一暴露的实验切换字段, 用于:

```text
bag 命名前缀          slosh_<group>_qs<Q>_<timestamp>.bag
互斥校验权威          validate 以 group 为基准, 派生字段必须匹配
analysis 分组键       analysis CSV / figure / table 全部按 group groupby
```

枚举值:

```text
experiment_group:
  C           ordinary MPC                          (主表, 消融)
  D           slosh-only MPC                        (消融)
  E           smooth-only MPC                       (主表, 消融)
  F           SloshPriorityMPC                      (主表, ours)
  RPP_STYLE   RPP-style regulated-speed baseline    (主表, 控制器轴)
  BIAGIOTTI   open-loop sloshing-free shaping       (主表, 范式轴最强开环)
  RUCKIG      Ruckig jerk-limited retiming          (supplementary 阶梯 + 时间匹配)
  TOPPRA      TOPPRA accel-limited retiming         (supplementary 阶梯)
```

**派生关系** (group → controller_variant + external_profile_mode):

| group | `controller_variant` | `external_profile_mode` | Q_slosh 约束 | 备注 |
|---|---|---|---|---|
| C | `mpc` | `none` | == 0 | R_a/R_da 取 C_default |
| D | `mpc` | `none` | > 0 | R_a/R_da 取 C_default, Q_slosh 自由 sweep |
| E | `mpc` | `none` | == 0 | R_a/R_da 取 E_smooth (强平滑) |
| F | `mpc` | `none` | > 0 | F-best 工作点 (Q_slosh/R_a/R_da 由 sweep 选出) |
| RPP_STYLE | `rpp_speed_reg` | `none` | == 0 | R_a/R_da 取 C_default |
| BIAGIOTTI | `mpc` | `biagiotti` | == 0 | csv 必填, R_a/R_da 取 C_default (同 C 跟踪器) |
| RUCKIG | `mpc` | `ruckig` | == 0 | csv 必填, R_a/R_da 取 C_default (同 C 跟踪器) |
| TOPPRA | `mpc` | `toppra` | == 0 | csv 必填, R_a/R_da 取 C_default (同 C 跟踪器) |

注: BIAGIOTTI/RUCKIG/TOPPRA 是范式轴, Layer2 都是 C 的 ordinary MPC 跟踪器
(controller_variant=mpc + Q_slosh=0 + C_default R), 仅 Layer1 整形剖面不同。

**派生方式**: launch 正式入口必须显式设置 `experiment_group:=<X>`；内部
`LocalPlannerROS::loadParameters` 根据 group 派生 `controller_variant_` +
`external_profile_mode_`。`Q_slosh / R_a / R_da` 等数值参数仍由 yaml/launch 显式给出，
但 `Q_slosh` 必须满足上表约束，否则 FATAL。

**执行顺序(关键, 不可乱)**:

```text
1. 读取 yaml/launch 数值参数: `Q_slosh / R_a / R_da` 等工作点数值在此步生效。
2. 根据 group 派生 categorical (controller_variant + external_profile_mode)。
   不自动改写 `R_a / R_da`，避免覆盖 F-best sweep 或外部 baseline 的显式调参值。
3. validate: 最后统一校验 (epsilon 判 Q_slosh)。
```

即 group 只定"实验类别 + 互斥边界"；数值最终以 20260527 方案中的启动命令为准。

**默认值与 LEGACY 语义(必须分清, 不要把三者混为一谈)**:

```text
experiment_group 默认值 = LEGACY (launch 不传时)

LEGACY != C, 也 != F。三者各是各的:
  LEGACY: 不走 group 派生, 完全保持"今天 launch + yaml 的原样行为"。
          经核查 mpc_params.yaml: q_slosh_eta = q_slosh_eta_dot = 0.0,
          → 今天的默认行为本就是"非晃液感知"(Q_slosh=0), 不是 SloshPriorityMPC。
          LEGACY 的意义就是: 不改变这一既有行为, 老 launch/老 bag 流程零影响。
  C:      显式选 ordinary MPC (Q_slosh=0 + C_default R), 是正式实验的一个 baseline,
          走 group 派生 + validate, bag 前缀 slosh_C_...。
  F:      显式选 SloshPriorityMPC (Q_slosh>0 + F-best R), ours, bag 前缀 slosh_F_...。

行为上 LEGACY 与 C 当前"恰好接近"(都 Q_slosh=0), 但语义不同:
  - LEGACY 是"不接管、保持现状"的逃生通道, 不进正式统计;
  - C 是"显式声明的 ordinary MPC baseline", 进正式统计。
  二者 bag 前缀、analysis 分组都分开, 不能互相代替。
```

**冲突处理**: 老 launch 仍直接传 `controller_variant` 或 `external_profile_mode` 时:

```text
if (experiment_group 显式设置, 即 != LEGACY):
    以 group 派生为准, ROS_WARN 覆盖任何用户传入的 controller_variant / external_profile_mode
if (experiment_group == LEGACY, 但 controller_variant/external_profile_mode 显式):
    走 §4.1/§4.2 的旧路径, group 字段 = "LEGACY" (analysis 会单独标注)
    ROS_WARN 建议改用 experiment_group
if (experiment_group == LEGACY, 且什么都没显式传):
    完全等同今天行为 (yaml 原样: Q_slosh=0 非晃液感知), 不 WARN。
```

**bag 命名规则** (record_slosh_experiment.sh 必须读 group 字段):

```text
slosh_<GROUP>_qs<Q_slosh>_ra<R_a>_rda<R_da>_<YYYYMMDD_HHMMSS>.bag

例:
  slosh_F_qs5.0_ra1.0_rda0.5_20260601_143020.bag
  slosh_RPP_STYLE_qs0.0_ra1.0_rda0.5_20260601_143510.bag
  slosh_RUCKIG_qs0.0_ra1.0_rda0.5_20260601_144022.bag
```

analysis 脚本通过 bag 名前缀 + bag 内 `/diagnostics/experiment_group` topic 双重确认。

### 4.1 controller_variant

**派生字段** (由 §4.0 `experiment_group` 推出, launch 不直接暴露; 老 launch 兼容路径仍可直接设)。

枚举:

```text
controller_variant:
  mpc            当前 SloshPriorityMPC 主线          (group ∈ {C,D,E,F,RUCKIG,TOPPRA,BIAGIOTTI})
  rpp_speed_reg  RPP-style speed regulator 接管曲率   (group == RPP_STYLE)
```

命名说明：

```text
避免使用:
  rpp_speed_regulated_mpc   (太长, 而且 reviewer 看到 mpc 后缀会困惑)
  rpp                       (会误导成完整 Nav2 RPP controller)
  rpp_regulator             (没说明是 speed 层)

采用:
  rpp_speed_reg             (短, 明确是 speed 层, 不会被误读)

派生命名一致性:
  controller_variant=rpp_speed_reg
  诊断 topic 前缀: /rpp_speed_reg/*
  launch arg / yaml param: rpp_speed_reg/*
```

语义：

```text
mpc:
  当前普通 MPC / SloshPriorityMPC 主线。

rpp_speed_reg:
  仍使用当前 MPC 跟踪器；
  但在 reference v_ref 层加入 RPP-style curvature / approach speed regulation；
  不使用 slosh state；
  不使用 /slosh/height；
  不是完整 Nav2 RPP controller。
```

### 4.2 external_profile_mode

**派生字段** (由 §4.0 `experiment_group` 推出; `custom_csv` 仅老 launch 兼容路径可手动设)。

枚举:

```text
external_profile_mode:
  none        group ∈ {C, D, E, F, RPP_STYLE}
  ruckig      group == RUCKIG       (retime_ruckig_style.py 输出)
  toppra      group == TOPPRA       (retime_toppra_style.py 输出)
  biagiotti   group == BIAGIOTTI    (shape_biagiotti.py 输出)
  custom_csv  legacy / 调试通道, 无对应 experiment_group
```

设计原则: **mode 是主开关, csv 是 mode 的参数**(不是双开关并列)。
ruckig / toppra / biagiotti / custom_csv 行为相同(都注入 v_ref(s) CSV + 启用 execution cap),
仅区分 csv 来源约定与 analysis 标签;独立成枚举值是为了让 experiment_group 1:1 派生、bag/figure 自动分组。

参数关系:

```text
external_profile_mode: none / ruckig / toppra / biagiotti / custom_csv   主开关
external_speed_profile_csv: <path>                    mode != none 时必填的参数
external_profile_execution_cap.*                      mode != none 时自动启用
```

互斥规则(在 loadParameters 阶段统一校验, 见 §5):

```text
mode == none:
  csv 必须为空字符串; 非空 → WARN 并忽略 csv
  profile_execution_cap 强制 enable=false (即使 yaml 设了 true)

mode ∈ {ruckig, toppra, biagiotti, custom_csv}:
  csv 必须非空 → 否则 ERROR 启动失败
  profile_execution_cap 自动 enable=true
  csv 来源约定:
    ruckig    ← retime_ruckig_style.py
    toppra    ← retime_toppra_style.py
    biagiotti ← shape_biagiotti.py (见 §9.2)
    custom_csv← 任意 v_ref(s) CSV (调试 / 历史兼容)
```

语义:

```text
none:
  使用 PathHandler 内部速度剖面。

ruckig / toppra / biagiotti:
  使用对应脚本生成的 external_speed_profile_csv；启用 profile_execution_cap；
  Layer2 跟踪器固定为 C 的 ordinary MPC (controller_variant=mpc + Q_slosh=0), 保证范式轴公平。
  ruckig/toppra = 开环非 slosh-aware retiming (supplementary 阶梯);
  biagiotti     = 开环 slosh-aware 整形 (正文主表最强开环代表)。

custom_csv:
  保留通用 CSV 注入能力；主要用于调试和历史兼容，不作为正文主流程。
```

## 5. 正式实验互斥规则

`experiment_group` (§4.0) 是单一权威字段, 互斥规则等价于"派生字段必须与 group 匹配 + Q_slosh 满足 group 约束"。
合法组合表(同 §4.0 派生表, 此处再列一次便于校验代码对照):

| `experiment_group` | `controller_variant` | `external_profile_mode` | `Q_slosh` | 说明 |
|---|---|---|---:|---|
| C | `mpc` | `none` | == 0 | ordinary MPC |
| D | `mpc` | `none` | > 0 | slosh-only |
| E | `mpc` | `none` | == 0 | smooth-only (R_a/R_da 取 E_smooth) |
| F | `mpc` | `none` | > 0 | SloshPriorityMPC (sweep 选 best) |
| RPP_STYLE | `rpp_speed_reg` | `none` | == 0 | RPP-style regulated-speed baseline |
| BIAGIOTTI | `mpc` | `biagiotti` | == 0 | open-loop sloshing-free shaping (范式轴最强开环) |
| RUCKIG | `mpc` | `ruckig` | == 0 | jerk-limited retiming (+ 时间匹配检验) |
| TOPPRA | `mpc` | `toppra` | == 0 | accel-limited retiming (supplementary 阶梯) |
| LEGACY | (任意) | (任意) | (任意) | 老 launch 路径, 不参与正式实验统计 |

正式 group 的派生字段处理:

```text
experiment_group != LEGACY 时:
  controller_variant / external_profile_mode 以 group 派生值为准;
  如果 launch 里误传了不同值, 启动期 ROS_WARN 并覆盖为派生值;
  不把这类误传直接 FATAL, 避免实物命令里残留兼容字段导致整场启动失败。

# group 与派生字段不匹配 (举例)
experiment_group=C + controller_variant=rpp_speed_reg
experiment_group=RPP_STYLE + external_profile_mode != none
experiment_group=BIAGIOTTI + external_profile_mode != biagiotti
experiment_group=TOPPRA + external_profile_mode != toppra
experiment_group=RUCKIG + external_profile_mode != ruckig
→ 上述由 group 派生覆盖并 ROS_WARN, 最终运行值必须回到派生表。

# Q_slosh 与 group 约束冲突 (用 epsilon, 勿严格浮点相等)
#   slosh-aware 判据: Q_slosh > 1e-9; non-slosh 判据: Q_slosh <= 1e-9
experiment_group ∈ {C,E,RPP_STYLE,RUCKIG,TOPPRA,BIAGIOTTI} + Q_slosh > 1e-9
experiment_group ∈ {D,F} + Q_slosh <= 1e-9

# 派生字段内部冲突 (老 LEGACY 路径仍需校验)
controller_variant=rpp_speed_reg + Q_slosh > 0
controller_variant=rpp_speed_reg + external_profile_mode != none
controller_variant=mpc + external_profile_mode ∈ {ruckig,toppra,biagiotti,custom_csv} + Q_slosh > 0
controller_variant=rpp_speed_reg + external_profile_mode != none
```

### 5.1 校验责任与执行点

互斥规则在 `LocalPlannerROS::loadParameters` 末尾**统一派生与校验**:
正式 group 先覆盖 categorical 字段, 再校验 Q_slosh/CSV/内部互斥。违反硬约束即 `ROS_FATAL`
启动失败, 不依赖 launch 文件检查(launch 层无法做参数组合判断)。

校验函数签名:

```text
bool LocalPlannerROS::configureExperimentVariant(ProfileExecutionCapParams& profile_cap_params);

实现要点 (按以下顺序):
  Step 1: 若 experiment_group_ != LEGACY:
            调 deriveParamsFromGroup(experiment_group_) -> (cv_derived, epm_derived, Q_slosh 约束)
            controller_variant_ 不等于 cv_derived → ROS_WARN 后覆盖
            external_profile_mode_ 不等于 epm_derived → ROS_WARN 后覆盖
            校验 Q_slosh 满足 group 约束 (epsilon 判: <= 1e-9 或 > 1e-9); 不满足 → FATAL
  Step 2: 派生字段内部互斥校验 (LEGACY 路径也走):
            controller_variant=rpp_speed_reg + Q_slosh > 0 → FATAL
            controller_variant=rpp_speed_reg + epm != none → FATAL
            controller_variant=mpc + epm ∈ {ruckig,toppra,biagiotti,custom_csv}
              + Q_slosh > 1e-9 → FATAL
            controller_variant=rpp_speed_reg + epm=custom_csv → FATAL
  Step 3: csv 路径完整性:
            epm != none + external_speed_profile_csv_ 为空 → FATAL
  任一 FATAL → ROS_FATAL("[validate] <rule> violated, expected=X got=Y") 并返回 false

loadParameters() 内必须调用; false 则 initialize() 失败, 不进 run()

校验时机:
  - loadParameters 全部读完之后(含 deriveParamsFromGroup 调用), advertise/subscribe 之前
  - 不在 controlLoop 内重复校验(只验一次, 启动期门槛)
```

错误信息约定:

```text
[validate] rpp_speed_reg requires Q_slosh == 0, got 5.0
[validate] rpp_speed_reg requires external_profile_mode == none, got 'ruckig'
[validate] external_profile_mode in {ruckig,toppra,biagiotti,custom_csv} requires Q_slosh == 0, got 5.0
[validate] external_profile_mode != none requires non-empty external_speed_profile_csv
```

每条信息都明确指出: 哪条规则违反 + 期望值 vs 实际值。

### 5.2 PathHandler::setParams 的 CSV 透传约束

经核查 (2026-05-29):

```text
PathHandler::setParams(params)   // path_handler.cpp:406
  -> loadExternalSpeedProfile(params_.external_speed_profile_csv)  // 410
```

`setParams` 会**无条件**根据传入 params 中的 `external_speed_profile_csv` 加载剖面。
这意味着 LocalPlannerROS 在调 `path_handler_.setParams(path_params_)` 之前, 必须依据
`external_profile_mode` 决定 `path_params_.external_speed_profile_csv` 的实际值:

```text
mode == none:
  path_params_.external_speed_profile_csv = "";    // 必须显式清空
                                                   // 否则即便互斥校验通过,
                                                   // PathHandler 仍会按老 csv 路径 load
mode ∈ {ruckig, toppra, biagiotti, custom_csv}:
  path_params_.external_speed_profile_csv = <param 中的 csv 路径>;
```

这一步在 `LocalPlannerROS::loadParameters` 内 `configureExperimentVariant()` 之后、
`setParams` 之前完成, 是 Phase B 的关键解耦动作。

向后兼容: 老 launch 文件可能仍直接传 `external_speed_profile_csv` 参数而**不传**
`external_profile_mode`。处理策略:

```text
external_profile_mode 默认值 = none
if (external_profile_mode == none && external_speed_profile_csv 非空):
    if (experiment_group != LEGACY):
        ROS_FATAL  // 正式实验不允许残留 csv 被静默忽略, 否则误以为跑了 external profile
    else:  # LEGACY 路径
        ROS_WARN("[compat] csv set but mode=none, ignored. Use external_profile_mode=custom_csv.");
        path_params_.external_speed_profile_csv = "";  // 仍按 mode 清空
```

老 launch 不会启动失败, 但会被 WARN 警告并按 mode=none 跑(保守、可发现)。要恢复老行为需显式设
`external_profile_mode=custom_csv`。

## 6. RPP-style speed regulator 模块边界

### 6.0 参考 Nav2 源码 (实现对照)

本模块**移植**而非 reinvent。所有公式以 **pin 死的 Nav2 commit** 为准(勿写"main 分支",
main 会变、复现会断): 在 README/代码注释里记 `navigation2 commit <hash>` 的
`regulation_functions.hpp`, 并把关键公式原文复制进代码注释。本地浅克隆位于
`/home/a/scout_ws/src/navigation2`。关键参考文件:

```text
nav2_regulated_pure_pursuit_controller/
├── include/nav2_regulated_pure_pursuit_controller/
│   ├── regulation_functions.hpp        ★ 主要参考: curvatureConstraint /
│   │                                      approachVelocityScalingFactor /
│   │                                      approachVelocityConstraint /
│   │                                      costConstraint(本方案不移植)
│   ├── regulated_pure_pursuit_controller.hpp   控制器主类接口(本方案不移植控制器本体)
│   ├── parameter_handler.hpp                   参数名约定 (regulated_linear_scaling_min_radius
│   │                                            / min_approach_linear_velocity /
│   │                                            approach_velocity_scaling_dist)
│   └── collision_checker.hpp                   本方案不移植
├── src/
│   ├── regulated_pure_pursuit_controller.cpp   控制器主循环 (本方案不移植)
│   └── parameter_handler.cpp                   参数默认值参考
└── README.md                                   行为文字说明
```

**只移植 `regulation_functions.hpp` 中的 2 个内联函数**:

1. `curvatureConstraint(raw_linear_vel, curvature, min_radius)` — line 42-51
   原文公式:
   ```cpp
   const double radius = fabs(1.0 / curvature);
   if (radius < min_radius) {
       return raw_linear_vel * (1.0 - (fabs(radius - min_radius) / min_radius));
   } else {
       return raw_linear_vel;
   }
   ```
   即 radius < min_radius 时**线性 taper**至 0, 不是简单二值截断。

2. `approachVelocityConstraint(constrained_linear_vel, path, min_approach_velocity, approach_velocity_scaling_dist)` — line 120-134
   依赖 `approachVelocityScalingFactor` (line 93-110), 关键设计:
   - **门槛**用积分路径长度 `calculate_path_length(transformed_path)` — 避免在大范围曲线路径上误触发
   - **缩放因子**用机器人坐标系下到 path 最后一点的**欧氏距离** `hypot(last.x, last.y) / scaling_dist` — 与路径离散密度无关, 平滑

**不移植**:

```text
costConstraint            (本实验固定无障碍, costmap cost 恒 0)
collision_checker.*       (固定路径无碰撞规避)
RegulatedPurePursuitController 主类 (我们不替换 MPC 控制器, 只借速度调节)
parameter_handler.* 的 ROS 2 lifecycle 参数动态接口 (走我们的 yaml + loadParameters)
```

### 6.1 新增模块

建议新增：

```text
include/scout_local_planner/rpp_speed_regulator.h
src/rpp_speed_regulator.cpp
```

接口契约 (纯函数, 无内部状态, 不依赖 ROS / PathHandler / MPCSolver):

```text
namespace scout_local_planner {

struct RppSpeedRegulatorParams {
    bool   enable                       = false;

    // 曲率 cap 参数, 对应 Nav2 regulation_functions.hpp:42 curvatureConstraint
    //   原参数名: regulated_linear_scaling_min_radius
    double regulated_min_radius         = 0.5;   // r < min_radius 时 v 线性 taper 到 0

    // approach cap 参数, 对应 Nav2 regulation_functions.hpp:93 approachVelocityScalingFactor
    //   原参数名: approach_velocity_scaling_dist / min_approach_linear_velocity
    double approach_dist                = 0.7;   // remain_s < approach_dist 时启用 cap
    double min_approach_v               = 0.05;  // approach 段下限速度

    // 与 PathHandler step 2 曲率调速的关系开关。
    //   前提: 必须先把曲率限速拆成两层 (见 §6.3), replace 只动 method 层:
    //     method cap (实验性曲率调速, a_lat_comfort): 不同 baseline 可以不同, 是实验变量。
    //     safety cap (硬横向加速度上限, a_lat_safety): 所有 baseline 共用, RPP 不得绕过。
    //   true  (default): regulator enable 时, PathHandler 跳过 step 2 的 method 层 (a_lat_comfort),
    //                    RPP step 4 接管"曲率调速"这一角色。论文叙述最干净 —— RPP-style baseline
    //                    是独立的非晃液感知速度调节, 不是 comfort cap 之上的弱叠加。
    //                    safety 层 (a_lat_safety) 仍在 step 4b 强制生效, 不受此开关影响。
    //   false:           保留 step 2 method 层, RPP step 4 作为额外 min cap 叠加 (仅消融对照)。
    bool   replace_base_curvature_cap   = true;
    // RPP 曲率约束是启发式半径缩放 (Nav2 taper), 不等价于物理横向加速度安全约束,
    // 因此 replace=true 时绝不能让车失去 a_lat_safety 保护 (见 §6.3 / §7 step 4b)。

    // 不引入 v_max: 速度上限由 PathHandler step 1 已用 vehicle/v_max cap 完毕,
    //   传入的 v_in 已经 ≤ vehicle/v_max。regulator 只做 min 缩减, 不需要自带 v_max。
    // 当前阶段不实现 (Nav2 控制器自带, 但我们不替换控制器主循环):
    //   lookahead_time, use_velocity_scaled_lookahead_dist,
    //   inflation_cost_scaling_factor, allow_reversing, ...
};

struct RppSpeedRegulatorOutput {
    double v_out;                  // = min(v_in, v_curvature_cap, v_approach_cap)
    double v_curvature_cap;        // 仅曲率限制后的速度
    double v_approach_cap;         // 仅终点距离限制后的速度
    bool   curvature_active;       // curvature cap 是否生效
    bool   approach_active;        // approach cap 是否生效
};

class RppSpeedRegulator {
public:
    // 同输入 → 同输出, 无状态
    //
    // Nav2 原版每个控制周期只算 1 次 v (无 horizon)。我们的 MPC 需要 horizon 内
    // 每个 k 的 v_ref 序列, 所以把 RPP regulation 在 horizon 上 "展开":
    //   PathHandler::getReferencePoints 对每个 k 调用一次 regulate, 传入:
    //       v_in        = PathHandler step 1+2 后的速度 (对该 k)
    //       kappa       = ref_points[k].kappa
    //       remain_s_k  = max(0, total_len_global - s_progress_k)
    //   path_handler.cpp:769-770 已算出 total_len_global / s_progress, 直接复用。
    //
    // 与 Nav2 原版的差异(明示, 论文要交代):
    //   Nav2 approachVelocityScalingFactor 用 path_length 做门槛 + 欧氏距离做缩放;
    //   我们用 remain_s_k 同时做门槛和缩放 (横向偏离已被 PathHandler 处理掉, 横向项
    //   退化, 实测两者数值差异 < 1%, 但 horizon 上语义更一致)。
    RppSpeedRegulatorOutput regulate(
        double v_in,                       // 上游 (PathHandler step 1+2) 给的 v_ref
        double kappa,                      // 该 horizon step 的 reference curvature
        double dist_to_goal,               // 该 horizon step 剩余路径长度 remain_s_k (米)
        const RppSpeedRegulatorParams& params) const;
};

}  // namespace
```

**dist_to_goal 取法的反例**: 若 horizon N=20, 所有 k 都用同一个 "车体到 goal 的欧氏距离",
则整 horizon 内 v_approach_cap 都相同, 终点段 v_ref 会出现 "悬崖式下降" 而不是
平滑沿路径线性收敛——v_ref 序列与 Nav2 RPP 实际行为不符。**正确做法**: 每 k 单独算
`remain_s_k = total_len_global - s_progress_k`, 然后按 Nav2 公式
(`regulation_functions.hpp:106` 的等价形式):
```text
if (remain_s_k < approach_dist):
    scale_k = remain_s_k / approach_dist
    v_approach_cap_k = max(min_approach_v, v_in * scale_k)
else:
    v_approach_cap_k = v_in
```

曲率 cap 同样按 Nav2 `regulation_functions.hpp:42` 公式 (线性 taper):
```text
radius_k = |1 / kappa_k|  (kappa_k=0 时 radius_k = +inf, 不触发)
if (radius_k < regulated_min_radius):
    v_curvature_cap_k = v_in * (1 - |radius_k - regulated_min_radius| / regulated_min_radius)
else:
    v_curvature_cap_k = v_in
```

设计原则:

```text
1. 纯函数: 同输入 → 同输出, 无内部 state (除 const params)
2. 无 ROS 依赖: 单测时直接 new + 传参, 不需要 rosbag / node
3. 无上下游耦合: 不持有 PathHandler / MPCSolver 引用, 不订阅 topic
4. 调用方任意: LocalPlannerROS / PathHandler / 未来 standalone RPP node 都可调用
```

职责:

```text
输入:
  v_in           上游给的速度参考 (m/s)
  kappa          该 horizon step 的曲率 (1/m, 带符号)
  dist_to_goal   距终点距离 (m)
  params         RppSpeedRegulatorParams const&

输出:
  v_out          最终速度 cap (m/s) = min(v_in, v_curvature_cap, v_approach_cap)
  v_curvature_cap, v_approach_cap  各 cap 的分量 (调试用)
  curvature_active, approach_active 各 cap 是否生效 (诊断用)
```

不负责：

```text
不求解 MPC；
不发布 cmd_vel；
不处理 slosh state；
不做 obstacle proximity cap；
不做 costmap collision check；
不读 ROS params (params 由 LocalPlannerROS::loadParameters 填好后传入)。
```

### 6.2 第一阶段实现范围

只实现固定路径实验中需要的两类 regulation：

```text
1. curvature regulation:
   高曲率处降低 v_ref。

2. approach velocity scaling:
   接近终点时按剩余距离降低 v_ref，并保留 min approach velocity。
```

暂不实现：

```text
obstacle proximity regulation
time-domain collision checker
full Pure Pursuit lookahead command generation
standalone RPP controller node
```

原因：

```text
固定 P2 主实验没有动态障碍；
当前目标是构造 non-slosh-aware regulated-speed baseline；
完整 Nav2 RPP controller 可作为下一阶段工程扩展。
```

### 6.3 曲率限速的两层边界 (method cap vs safety cap)

当前 PathHandler step 2 用一条 `v ≤ sqrt(a_lat_max / |kappa|)` 同时承担了两个职责,
本方案把它**显式拆成两层**, RPP 的 `replace_base_curvature_cap` 只允许动其中一层:

```text
method cap (实验性曲率调速, a_lat_comfort):
  作用: 舒适/激励整形层, 决定"过弯减到多慢"。
  归属: 实验变量。C/E/F 用 PathHandler sqrt(a_lat_comfort/|kappa|);
        RPP-style 用 Nav2 taper 替代它 (replace_base_curvature_cap=true 时 skip 这层)。
  取值: 沿用今天 a_lat_max 的标定值, 改名为 a_lat_comfort, 数值不变 (零行为变更)。

safety cap (硬横向加速度上限, a_lat_safety):
  作用: 防侧滑/翻倒的物理安全底线, 任何 baseline 都不能突破。
  归属: 全局安全, 所有方法共用, 与 controller_variant / experiment_group 无关。
  取值: a_lat_safety >= a_lat_comfort (建议留 1.5~2x 余量), 平时不触发,
        只兜住 method 层失效 / RPP 启发式给出过激速度的极端情况。
  位置: §7 step 4b, 在所有 method 层 (step2 / step3 / step4) 之后, 作为最后一道 min cap。
```

关键点:

```text
1. RPP 的 Nav2 曲率 taper 是启发式半径缩放, 不是物理 a_lat 约束 →
   它只能替代 method 层 (a_lat_comfort), 不能替代 safety 层 (a_lat_safety)。
2. replace_base_curvature_cap=true 时: skip 的是 step2 的 a_lat_comfort, 不是 a_lat_safety。
   safety cap 在 step 4b 永远跑。
3. controller_variant=mpc 时: step2 用 a_lat_comfort (= 今天行为), step 4b 的 a_lat_safety
   也照跑; 由于 a_lat_safety >= a_lat_comfort, 对 C/E/F 是 no-op, 零行为变更。
4. 命名落地: 今天 yaml 的 max_lat_accel 改名/拆成 a_lat_comfort + a_lat_safety;
   a_lat_comfort = 旧值, a_lat_safety = 旧值 * safety_margin (默认 margin 给 1.0 即与今天等价,
   想要安全余量再调大, 避免一上来就改变 C/E/F 行为)。
```

### 6.4 (可选, 进阶模块化) 速度参考链路抽成 pipeline + 配置单元独立

> 定位: **本方案默认路线不做这一节**, 它属于"进阶模块化"。默认路线 (Phase A-F) 用
> §7 的 step 内联接入, 保守、风险低、不动 §3 保留主线。本节是当默认路线落地稳定后,
> 想进一步降低 `getReferencePoints` / `local_planner_ros.cpp` 复杂度时的可选重构 (Phase G)。
> 触发条件: 默认路线已通过 Phase F, 且团队判断 god 函数复杂度需要治理。不满足就不做。

**问题陈述(诚实记录现状)**:

```text
现状 (2026-05-30 核查):
  path_handler.cpp        1772 行
  getReferencePoints()    单函数 ~200 行, 内含 step1..6
  local_planner_ros.cpp   1252 行 (god class)

默认路线 §7 会再往 getReferencePoints 塞 step2 开关 / step3 分支 / step4 RPP / step4b safety,
往 local_planner_ros 塞 deriveParamsFromGroup + validate + CSV 透传。
→ 新模块本身解耦干净 (RppSpeedRegulator 纯函数), 但"接入宿主"继续变肥。
→ 这是"在乱房间摆收纳盒", 不是"重新规划房间"。对快速落地无碍, 对长期可维护性不利。
```

**目标产物(三个独立可测单元)**:

```text
1. SpeedReferencePipeline (新)
   include/scout_local_planner/speed_reference_pipeline.h
   src/speed_reference_pipeline.cpp
   - 把 §7 的 v_ref 链路显式建模成"有序 cap stage 列表":
       base_v_des → curvature_comfort(method) → external_profile → rpp_regulator(method)
                  → curvature_safety → goal_capture → v_ref
   - 每个 stage 是一个纯函数对象 (输入 v + 上下文, 输出 v + 诊断), 可单测、可开关、可调序。
   - getReferencePoints 退化为: 准备每个 k 的几何上下文 → 调 pipeline.apply(ctx) → 写 ref_points[k]。
   - safety cap 作为 pipeline 的"终端 stage"由 pipeline 保证恒在最后执行 (§6.3 的横切关注点
     从"埋在 step4b"上升为"pipeline 结构性保证", 别处复用 pipeline 也不会漏 safety)。

2. ExperimentConfig (新)
   include/scout_local_planner/experiment_config.h
   src/experiment_config.cpp
   - group 派生 / 互斥校验从 local_planner_ros 搬到这里。
   - 纯查表 + 校验逻辑, 不依赖 ROS node 句柄 (传入已读出的参数结构体, 返回派生结果 / 错误)。
   - local_planner_ros 只负责: 读 ROS param → 调 ExperimentConfig::derive/validate → 用结果。
   - 收益: god class 减负; 派生/互斥规则可脱离 ROS 单测 (输入组合 → 期望派生/FATAL)。

3. (随上述自然产生) getReferencePoints 瘦身
   - 几何窗口准备 / TF / 局部样条 等保留在 PathHandler;
   - 速度链路逻辑外移到 SpeedReferencePipeline;
   - 函数行数与圈复杂度显著下降。
```

**为什么独立成 §6.4 而不并入默认路线**:

```text
- 它会动 §3 列为"保留主线"的 path_handler / local_planner_ros 内部结构 (虽不改外部行为),
  属于比"外科手术式修改"更大的重构, 风险高于默认路线。
- 必须有"零行为变更"硬门保护 (见 Phase G 验收), 否则可能在重构中引入隐性回归。
- 与论文实验进度解耦: baseline 实验不依赖这次重构, 可在论文数据采集之后再做。
```

## 7. 参考速度链路

v_ref 计算链路实际由两层编排:**LocalPlannerROS::controlLoop 上游**(给 PathHandler 一个执行层 v_des 上界) +
**PathHandler::getReferencePoints 内部**(对 horizon 内每个 k 算 v_ref 序列)。

经核查 (2026-05-29) 当前代码实际结构:

```text
========== LocalPlannerROS::controlLoop 上游(每控制周期一次) ==========

step 0a: terminal envelope / capture / slowdown
          -> v_des_cmd_raw
          来源: TerminalController, 既有逻辑

step 0b: v_des_rate_limit  (执行层平滑, 上游)
          v_des_cmd = clamp(v_des_cmd_raw, prev_v_des - decel*dt, prev_v_des + accel*dt)
          来源: local_planner_ros.cpp:476/550 既有 v_des_rate_limit 逻辑
                (注意: 不在 PathHandler 内, 是 controlLoop 内的上游平滑)

step 0c: 调用 path_handler_.getReferencePoints(N, dt, v_exec, v_des_cmd, ref_points)
          把 v_des_cmd 作为 v_plan 上界传入

========== PathHandler::getReferencePoints 内部(每 horizon k 一次) ==========

step 1: PathHandler raw v_des
          = min(vehicle_v_max, v_des_cmd, ...)
          来源: 当前 PathHandler 既有逻辑

step 2: PathHandler curvature cap — method 层 (a_lat_comfort, 见 §6.3)
          if (controller_variant == rpp_speed_reg
              && rpp_speed_reg.replace_base_curvature_cap == true):
              SKIP    # RPP step 4 接管曲率调速 (§6.1 default); 仅 skip method 层
          else:
              v = min(v, sqrt(a_lat_comfort / |kappa_k|))
          来源: 当前 PathHandler 既有 sqrt 公式 (max_lat_accel 改名 a_lat_comfort), 加开关

step 3: external_speed_profile 注入 (Ruckig / TOPPRA / Biagiotti / custom_csv)
          if (external_profile_mode != none):
              v = interp(external_csv, s_progress_k)    替换上述 v
          来源: 当前 PathHandler 既有 external_speed_profile_csv 机制
                (path_handler.cpp:803 getSpeedAtS)

step 4: optional RPP-style regulator — method 层 (本方案新增)
          if (controller_variant == rpp_speed_reg):
              double remain_s_k = max(0, total_len_global - s_progress_k)
              v = min(v, RppSpeedRegulator.regulate(v, kappa_k, remain_s_k, params).v_out)
          否则不变
          来源: 本方案 §6 新增模块, 接入点在 PathHandler::getReferencePoints 内部循环

step 4b: curvature SAFETY cap (硬横向加速度上限, 所有 baseline 共用, 见 §6.3)
          v = min(v, sqrt(a_lat_safety / |kappa_k|))
          - 与 controller_variant / experiment_group 无关, 永远执行;
          - a_lat_safety >= a_lat_comfort, 平时 no-op, 只兜 method 层失效 / RPP 过激速度;
          - RPP (step 4) 不能绕过它: replace_base_curvature_cap 只 skip step2 method 层, 不碰 4b。

step 5: goal capture protection (终点附近防止 v_ref 过早归零, 既有)
          来源: path_handler.cpp 既有逻辑

step 6: 写入 ref_points[k].v_ref
```

**关键修正**: v_des_rate_limit 是 controlLoop 上游(step 0b), **不在 PathHandler 内部**。
PathHandler 拿到的 v_des_cmd 已经是 rate-limited 的, PathHandler 内部不再做 rate limit。

关键澄清:

```text
1. RPP 与 PathHandler step 2 曲率 cap 的关系由 rpp_speed_reg.replace_base_curvature_cap 控制:
   - default true (主实验配置): step 2 跳过, RPP step 4 独占曲率限速。
     论文叙述: "RPP-style baseline replaces the physical lateral-accel cap with
     Nav2-style regulated curvature taper", 不是 MPC 物理 cap 之上的弱叠加。
   - false (消融): 两者叠加, v ≤ min(step2_sqrt_cap, step4_rpp_cap)。

   为什么 default true: 实测 d200 参数下 a_lat_max=1.0 m/s² 与 regulated_min_radius=0.5 m
   竞争, step 2 在 |kappa|>2 时已把 v 压到 0.7 m/s 以下, RPP step 4 的 taper 在多数
   horizon 点不会被触发, /rpp_speed_reg/v_curvature_cap 在 bag 上看起来"几乎不工作",
   baseline 失去实验说服力。default true 让 RPP 真正成为可观察的速度调节源。

2. controller_variant == mpc 时 step 2 永远启用 (与今天行为一致)。

3. RPP 与 external_profile_mode 互斥 (见 §5):
   不会出现 step 3 + step 4 同时启用的情况。

4. RPP regulator 的 dist_to_goal 必须按 horizon step k 算:
   remain_s_k = max(0, total_len_global - s_progress_k)
   不能用 "当前车体到 goal 的欧氏距离" 套整个 horizon, 否则 horizon 内所有点
   会被同一个距离 cap, 终点前速度形状会失真。
   approach cap 应沿路径进度递减, 而非整 horizon 同一个值。
```

RPP-style 应作用在 reference v_ref 层 (step 4), 而不是 cost 层:

```text
正确:
  在 PathHandler::getReferencePoints 内, step 4 取 min(v, rpp_cap)。
  rpp_cap 由 RppSpeedRegulator.regulate(...) 返回。

错误:
  写进 cost_function.cpp 或 mpc_solver.cpp, 让 RPP 变成一个 MPC cost term。
  原因:
    - cost term 是 stage cost on horizon, 求解器内部权衡;
    - regulator 是 reference 层硬 cap, 求解器外部边界;
    - 混进 cost 会让 RPP 与 slosh cost 在同一目标函数里耦合, 违反 §5 互斥规则;
    - 也无法与 controller_variant 开关 1:1 对应。
```

## 8. 诊断 topic

建议新增诊断 topic (前缀与 controller_variant 命名一致, 见 §4.1):

```text
/rpp_speed_reg/active                 std_msgs/Int8
/rpp_speed_reg/curvature              std_msgs/Float32
/rpp_speed_reg/v_raw                  std_msgs/Float32
/rpp_speed_reg/v_curvature_cap        std_msgs/Float32
/rpp_speed_reg/v_approach_cap         std_msgs/Float32
/rpp_speed_reg/v_out                  std_msgs/Float32
```

**外部 profile 必须发布 raw + capped 两套**(否则无法说明 cap 改了 TOPPRA/Ruckig/Biagiotti 多少):

```text
/profile_cap/input_v_ref   std_msgs/Float32   注入的原始 shaped v_ref (cap 前)
/profile_cap/output_v_ref  std_msgs/Float32   execution cap 后的 v_ref
/profile_cap/active        std_msgs/Int8      cap 是否在该步生效
报告口径: 论文须报 cap 对外部 baseline 的修改量 (output vs input); 若差异显著, 透明说明。
```

**实验身份字段(消融后处理防混淆)**: 除 `/diagnostics/experiment_group` 外, 再发

```text
/diagnostics/mpc_cost_variant   std_msgs/String   nominal / slosh_only / smooth_only / slosh_smooth
```

D 与 F 都是 Q_slosh>0, 真正区别在 R_a/R_da (D=C_default, F=E_smooth 类), 不能只靠 Q_slosh 判别,
故显式发布 cost_variant, analysis 直接读它分组。

**第一阶段(本方案)发布约定**: regulator 在 PathHandler::getReferencePoints 内被 horizon
每个 k 调用一次, 但**只发布 k=0** (即当前控制周期最近的 reference step) 的 regulator 输出。
这与 `/reference/v_ref_now` 等既有 topic 的时间轴对齐, 方便 bag 横向对照。

```text
发布点:
  PathHandler::getReferencePoints 内循环结束后, 用 k=0 的 regulator 输出
  通过既有 DiagnosticsPublisher 在 controlLoop 末尾发布
  (不在 horizon 内每 k 都发布, 避免话题膨胀; horizon 全序列只走 ref_points[k].v_ref)

字段语义:
  active=1 ⇔ controller_variant == rpp_speed_reg
  active=0 ⇔ controller_variant == mpc       (字段必须持续发布, 保持 bag 时间轴一致)
  curvature       k=0 step 传入 regulator 的 kappa
  v_raw           regulator 输入 v_in (即 PathHandler step 1+2 后的速度)
  v_curvature_cap regulator 输出 v_curvature_cap
  v_approach_cap  regulator 输出 v_approach_cap
  v_out           regulator 输出 v_out
```

下一阶段可扩展(本方案不做): 增发 `/rpp_speed_reg/v_out_horizon` Float32MultiArray
保存整 horizon N 个 v_out, 用于离线复核 cap 形状, 但会增加 bag 体积, 建议默认 disable。

这些 topic 用于：

```text
1. 验证 RPP-style baseline 是否真的生效；
2. 分析 RPP-style 与 E/F 的速度差异；
3. 论文中解释 RPP-style 的速度调节行为。
```

注意：新增 topic 不应改变现有 `/reference/*`、`/terminal/*`、`/profile_cap/*` 语义。
当 `controller_variant == mpc` 时, `/rpp_speed_reg/active` 应持续发布 `0`(让 bag 时间轴上方法对比可视化保持一致)。

## 9. 文件改动清单

### 9.1 C++ 主线

预计新增：

```text
include/scout_local_planner/rpp_speed_regulator.h
src/rpp_speed_regulator.cpp
```

预计修改：

```text
include/scout_local_planner/types.h                  ★ ExperimentGroup enum + RppSpeedRegulatorParams
include/scout_local_planner/local_planner_ros.h      ★ experiment_group_ 字段 + deriveParamsFromGroup
include/scout_local_planner/path_handler.h           ★ §7 step 2 加 replace_base_curvature_cap 开关
                                                       + step 4 调 RppSpeedRegulator
include/scout_local_planner/diagnostics_publisher.h
src/local_planner_ros.cpp                            ★ configureExperimentVariant
                                                       + §5.2 CSV 透传
src/path_handler.cpp                                 ★ §7 step 2 跳过 / step 4 接入 RppSpeedRegulator
src/diagnostics_publisher.cpp                        ★ §8 /rpp_speed_reg/* 6 个 topic
                                                       + /diagnostics/experiment_group 一次性发布
launch/slosh_experiment.launch                       ★ 加 experiment_group arg (default LEGACY)
launch/slosh_experiment_sim.launch
config/mpc_params.yaml                               ★ 加 rpp_speed_reg/* 默认参数 (含
                                                       replace_base_curvature_cap: true)
config/mpc_params_sim.yaml
CMakeLists.txt                                       ★ add_library/install rpp_speed_regulator
scripts/record_slosh_experiment.sh                   ★ 读 experiment_group 写入 bag 名前缀
```

### 9.2 脚本

保留(都是正式 baseline 脚本):

```text
scripts/analysis/retime_ruckig_style.py     RUCKIG group (supplementary 阶梯 + 时间匹配)
scripts/analysis/retime_toppra_style.py     TOPPRA group (supplementary 阶梯)
scripts/run_sim_fixed_path_bag.sh
scripts/record_slosh_experiment.sh
scripts/template_fixed_path_generator.py
scripts/send_fixed_goal.py
```

新增:

```text
scripts/analysis/shape_biagiotti.py         BIAGIOTTI group (开环 slosh-aware 整形, 主表)
  → 输入固定路径 + ω_n/ζ, 输出 v_ref(s) CSV (同 external_speed_profile_csv 格式)
  → 实现 Moriello 2018 平移加速度整形 (指数滤波器在 ω_n 放零点); 复用 path_profile_utils
  → 本质是轨迹/加速度整形, 不是给 v 乘系数; 命名严格写 "Biagiotti-style reference-shaping
    baseline", 不声称复现 vessel-tilting / 机械臂末端姿态部分 (差速底盘 N/A)。
  → 必须落盘存证: raw path profile / shaped time law / shaped v(s) / shaped a(s) / expected duration
    (否则无法解释它如何改变运动激励)。
```

调整：

```text
scripts/README.md
README.md
```

**共享工具抽取(必须先做)**:

经核查 (2026-05-29), `retime_ruckig_style.py` 当前**真的**依赖 `retime_toppra_style.py`:

```python
# retime_ruckig_style.py:16
from retime_toppra_style import (
    cumulative_s,
    interp_points,
    load_path_points,
    maybe_plot,
    write_csv,
)
```

TOPPRA 现在是正式 baseline(不归档),但这个跨脚本耦合仍要拆 —— 新增的 `shape_biagiotti.py`
也要用同样这 5 个函数。因此**必须抽出公共工具**:

```text
Step 1: 新建 scripts/analysis/path_profile_utils.py
Step 2: 把这 5 个函数从 retime_toppra_style.py 物理搬过去
        (cumulative_s / interp_points / load_path_points / maybe_plot / write_csv)
Step 3: retime_toppra_style.py / retime_ruckig_style.py / shape_biagiotti.py 都改 import:
        from path_profile_utils import cumulative_s, interp_points, ...
Step 4: 删 retime_toppra_style.py 里搬走的函数定义 (不再自己定义这 5 个)
Step 5: 跑 TOPPRA / Ruckig / Biagiotti 三脚本, 确认 import 链未断 + 输出 CSV 格式一致
```

完成后三个范式轴脚本共享同一套 path 工具, 输出同一 CSV 格式, 经 external_profile_mode 注入,
共用 C 的同一跟踪器。三个脚本都是正式 baseline, 不归档。

### 9.3 文档

需要同步更新：

```text
docs/重要文档/20260527_SloshPriorityMPC正式对比实验验证方案.md
docs/重要文档/20260518_MPC终点收敛与固定路径验证方案.md
src/scout_apps/control/scout_local_planner/README.md
src/scout_apps/control/scout_local_planner/scripts/README.md
docs/Claude/修改日志-时间/<日期>.md
```

### 9.4 外部参考代码 (只读, 不入仓)

```text
/home/a/scout_ws/src/navigation2/nav2_regulated_pure_pursuit_controller/
  include/nav2_regulated_pure_pursuit_controller/
    regulation_functions.hpp        ★ 我们移植 curvatureConstraint(line 42-51) 与
                                       approachVelocityConstraint(line 120-134)
    parameter_handler.hpp           参数名约定参考
    regulated_pure_pursuit_controller.hpp   控制器整体接口(本方案不复用)
  src/
    parameter_handler.cpp           参数默认值参考
    regulated_pure_pursuit_controller.cpp   控制器主循环(本方案不复用)
  README.md                         行为文字说明
```

navigation2 是浅克隆, 不进 catkin 构建, 不写入 src/scout_apps。
方案落地的 `RppSpeedRegulator` 是上述 2 个函数的**本地重实现**, Nav2 原文件许可证为
Apache-2.0 (Samsung Research America, 2022)。落地文件需在头注释中保留:

```text
// Ported from Nav2 regulation_functions.hpp
// Copyright (c) 2022 Samsung Research America, Apache License 2.0
// Original: https://github.com/ros-planning/navigation2/blob/main/
//           nav2_regulated_pure_pursuit_controller/include/
//           nav2_regulated_pure_pursuit_controller/regulation_functions.hpp
```

(本仓库整体许可证由用户决定, 此 attribution 只覆盖移植的 2 个函数本身。)

## 10. 分阶段执行计划

每个 Phase 后必须能编译 + 现有方法 sim smoke 通过, 才进入下一 Phase。

### Phase 总览

| Phase | 内容 | 完成后可跑的方法 | 不能跑 |
|---|---|---|---|
| A | path_profile_utils 抽取 + shape_biagiotti 新增 + README 主流程梳理 | C / D / E / F / TOPPRA / Ruckig 不变 | — |
| B | 加 controller_variant + external_profile_mode 参数 + 互斥校验 | C / D / E / F / Ruckig 通过新参数行为等价旧版(数值容差内) | RPP-style |
| C | 实现 RppSpeedRegulator + 接入 PathHandler step 4 + 诊断 topic | C / D / E / F / Ruckig / RPP-style 全套可跑 | — |
| D | sim smoke 5 方法各 1 包 (D 组只验数值切换, 不录 smoke bag) | (验证) | — |
| E | 实物 smoke 5 方法各 1 包 (D 组同上) | (验证) | — |
| F | 正式数据采集 n=3 起 → n=5, 走 `docs/重要文档/20260527_SloshPriorityMPC正式对比实验验证方案.md` | (论文数据) | — |
| G (可选) | 进阶模块化重构 (§6.4): SpeedReferencePipeline + ExperimentConfig 抽离 | 全套, 行为零变更 | — |

Phase A-F 是默认路线, 必须做。Phase G 是可选进阶模块化 (§6.4), 仅在 F 通过后、判断需要治理
god 函数复杂度时才做; 不做不影响论文实验。

### Phase A: 无行为变更清理

目标：

```text
1. 抽 path_profile_utils.py (Ruckig 当前依赖 TOPPRA 5 个函数, 见 §9.2)
2. 改 ruckig + toppra 的 import 指向 path_profile_utils
3. 新增 shape_biagiotti.py (复用 path_profile_utils, 输出同格式 CSV)
4. 删除 pycache
5. README 主流程梳理: TOPPRA/Ruckig/Biagiotti 标为范式轴 baseline
6. 不改任何 C++ 代码
```

验证:

```bash
catkin_make --pkg scout_local_planner   # 必须通过
RETIME_METHOD=ruckig 跑一次 sim smoke    # 验证 Ruckig 仍能跑(import 链未断)
```

完成后可跑: C / D / E / F / Ruckig (与 Phase A 前等价, 因为没改 C++)

### Phase B: 参数入口解耦

目标：

```text
1. launch 加 experiment_group (default LEGACY) arg, 内部派生 controller_variant +
   external_profile_mode (§4.0 派生表)。老 launch 仍可直接传后两者, 走 LEGACY 路径。
2. yaml / loadParameters 实现 deriveParamsFromGroup() + §5.2 CSV 透传清空逻辑
3. 实现 LocalPlannerROS::configureExperimentVariant (§5.1 三步派生与校验)
4. 保持默认 (group=LEGACY + mpc + none) 行为与当前回归一致 (按下方 4 层验收门,
   不要求 bag 级 byte-equal)
5. RppSpeedRegulator 模块还没实现, validate 阶段直接拒绝 experiment_group=RPP_STYLE
   及 controller_variant=rpp_speed_reg, Phase C 才放开。简化测试边界, 避免出现
   "参数允许但功能缺失" 的中间态。
6. record_slosh_experiment.sh 读 group 字段写入 bag 名 (§4.0 命名规则)
```

验证基线 (回归一致性, 不要求 bag 级 byte-equal):

```text
基线 bag: bags/sim_s_curve/<最近一次 F-best smoke bag>
对比配置: 同 path + 同 Q_slosh + 同 R_a/R_da, 启用 controller_variant=mpc + external_profile_mode=none
```

**为什么不强制 bag 级 byte-equal**: ROS 时间戳、OSQP solver 迭代细节、线程调度都会让
相邻两次同配置 run 在浮点低位上有小差异, 强求 `< 1e-6` 会反复误报。
**正确的验收门**(按强弱排序):

```text
1. 单元层 (强): Phase B 只测参数派生 + 互斥校验 + CSV 清空/透传
   - deriveParamsFromGroup 对每个 group 派生出正确的 (controller_variant, external_profile_mode)
   - group 误传 categorical 字段时 WARN 覆盖
   - Q_slosh/CSV/内部互斥硬约束违反时 FATAL, 合法组合放行
   - mode=none 时 path_params_.external_speed_profile_csv 被清空; mode!=none 时被透传
   RppSpeedRegulator::regulate 纯函数单测属 Phase C (此阶段模块未实现), 不在 Phase B 验收门。

2. 行为层 (强): 结论一致
   同路径同种子 5 次 run, 全部成功完成 (s 达到末端 + 终点收敛)
   所有 run 的 verdict 相同 (slosh_violation_count、completion_time bucket)

3. 数值层 (中, 容许公差): 关键指标在数值容差内
   completion_time            相对差 < 1%
   max(linear_v)              绝对差 < 0.005 m/s
   p95(linear_v)              绝对差 < 0.005 m/s
   /reference/v_ref 离散序列  L2 范数相对差 < 1e-3

4. 拓扑层 (强): 新增 topic 没有混入旧组
   controller_variant=mpc + external_profile_mode=none 时:
     /rpp_speed_reg/active 全程发布 0
     /profile_cap/active   全程发布 0 (mode=none 强制 disable)
   /cmd_vel /reference/* /slosh/* /terminal/* /mpc/cost_breakdown 字段数量与基线一致
```

工具:

```text
scripts/analysis/diff_two_bags.py
  若不存在, Phase B 之前先写; 输出上述 1-4 项 verdict, 不打印任何 < 1e-6 行级 diff
  (Phase B 重点是行为等价, 不是 byte 级复刻)

逐方法验证:
  C 组(默认 mpc+none):      行为+数值层通过
  F 组(同上 + Q_slosh=5):   行为+数值层通过
  Ruckig 组(mpc + ruckig + csv): 行为+数值层通过, 且 /profile_cap/active 始终为 1
```

完成后可跑: C / D / E / F / Ruckig (新参数下行为等价旧版)
不能跑: RPP-style (RppSpeedRegulator 模块还没实现, validate 拒绝)

### Phase C: RPP-style speed regulator

目标:

```text
1. 实现 RppSpeedRegulator 类 (§6.1 接口契约, 纯函数无状态)
2. PathHandler::getReferencePoints 内加 step 4 (§7 链路图),
   step 2 加 replace_base_curvature_cap 开关 (§7 + §6.1, default true)
3. 只在 controller_variant=rpp_speed_reg (即 experiment_group=RPP_STYLE) 时启用
4. 新增 /rpp_speed_reg/* 诊断 topic (§8)
5. `configureExperimentVariant()` 放开 RPP_STYLE / rpp_speed_reg
```

验证(包含 hard assertions, smoke 不通过即 Phase C fail):

```text
同一 P2 S 弯路径 sim smoke:

experiment_group=C 基线:
  /rpp_speed_reg/active = 0 全程
  /rpp_speed_reg/v_curvature_cap = NaN 或不发布
  cmd_vel 行为与 Phase B C 组数值一致 (回归)

experiment_group=RPP_STYLE:
  ASSERT 1 (active):
    /rpp_speed_reg/active == 1 持续 >= 95% 控制周期 (允许启动过渡)
  ASSERT 2 (RPP 真实工作 — 核心):
    S 弯弯心段 (s_progress ∈ [s_apex - 0.3, s_apex + 0.3] 米) 内,
    /rpp_speed_reg/curvature_active == 1 持续 >= 0.5 秒
    且 /rpp_speed_reg/v_curvature_cap < /rpp_speed_reg/v_raw 至少 0.1 m/s
    (证明 RPP step 4 真的把 v 压下来了, 不是被 step 2 提前吃掉)
  ASSERT 3 (approach 段):
    终点前 remain_s < approach_dist 段, /rpp_speed_reg/approach_active == 1 持续 >= 0.5 秒
    且 /rpp_speed_reg/v_approach_cap 从 v_in 线性下降到 min_approach_v
  ASSERT 4 (与 C 组速度差异可观察):
    弯心和终点处 cmd_vel.linear.x 比 C 组同一 s_progress 位置低 >= 0.05 m/s
    (否则 RPP-style 与 C 组在数据上无法区分, baseline 失败)

ASSERT 1-4 任一不通过 → 检查 rpp_speed_reg.regulated_min_radius / approach_dist 调参,
   或检查 replace_base_curvature_cap 是否真的生效。
```

完成后可跑: C / D / E / F / Ruckig / RPP_STYLE 全套

### Phase D: sim smoke

目标:

```text
在仿真中跑主表方法各 1 包:
  C / E / F / RPP-style / Biagiotti

supplementary smoke:
  Ruckig / TOPPRA 各 1 包或至少完成 CSV 注入链路验证
```

验收:

```text
1. 都能进入 TRACKING；
2. 都能到达终点；
3. /slosh/* 不中断；
4. /mpc/cost_breakdown 有数据；
5. RPP-style 有 /rpp_speed_reg/*；
6. Biagiotti/Ruckig/TOPPRA 有 /profile_cap/*。
```

### Phase E: 实物 smoke

目标:

```text
只做每组 1 包,不做论文结论:
  C / E / RPP-style / Biagiotti / F-best

supplementary smoke:
  Ruckig-time-matched / TOPPRA 各 1 包或按时间安排只做 Ruckig-time-matched
```

通过后再进入正式 n=3 / n=5。

### Phase G (可选): 进阶模块化重构

> 前置: Phase F 已完成 (论文数据采集结束)。本 Phase 是 §6.4 的落地, 默认路线不含。
> 唯一目标: 在**行为零变更**前提下治理 god 函数复杂度。任何行为差异都视为失败、回退。

目标:

```text
1. 抽 SpeedReferencePipeline (§6.4 产物1):
   把 getReferencePoints 内 step1..6 + RPP/safety/external 改写成有序 cap stage 列表;
   getReferencePoints 退化为"准备几何上下文 → pipeline.apply → 写 ref_points"。
2. 抽 ExperimentConfig (§6.4 产物2):
   group 派生 / 互斥校验从 local_planner_ros 搬出, 脱 ROS 单测。
3. safety cap 升为 pipeline 终端 stage (结构性保证恒在最后), 不再是 step4b 内联。
4. 不新增任何 baseline、不改任何 topic/字段/launch arg、不改 d200 参数。
```

验收(零行为变更硬门, 任一不过即回退):

```text
1. 单元层 (强):
   - SpeedReferencePipeline: 每个 stage 纯函数单测 (含 safety 终端恒执行);
   - ExperimentConfig: 8 个 group + LEGACY 的派生/互斥脱 ROS 单测 (输入组合 → 期望派生/WARN/FATAL)。
2. 回归层 (强): 重构前后, 同一批 Phase F 配置 (C/E/F/RPP-style/Biagiotti/Ruckig/TOPPRA)
   各重放 1 次, 按 §10 Phase B 的"4 层验收门"对比重构前 bag:
   - 行为层 verdict 一致;
   - 数值层 /reference/v_ref L2 相对差 < 1e-3, cmd_vel max/p95 绝对差 < 0.005 m/s;
   - 拓扑层 topic 字段数量一致, /rpp_speed_reg/* 与 /profile_cap/* 语义不变。
3. 圈复杂度层 (目标验证, 非硬门):
   getReferencePoints 行数 / 圈复杂度较重构前明显下降 (记录前后数值到修改日志)。
```

完成后: 代码模块化到位, 但**论文结论与数据不依赖本 Phase** —— 不做也不影响已发表实验。

## 11. 风险与回退

### 风险 1：RPP-style 与 MPC 内部 PathHandler 速度剖面重复降速

处理：

```text
先保留 PathHandler 基础速度剖面；
RPP-style 只做 min(v_ref, rpp_cap)；
论文解释为 regulated speed cap baseline。
```

### 风险 2：RPP-style 太慢，靠慢赢

处理：

```text
和 F-best 做 duration / avg speed 对齐；
若超过 ±10%，调 rpp desired velocity / regulated_min_radius / approach_dist。
```

### 风险 3：RPP-style 不是完整 RPP controller，被质疑

处理：

```text
论文明确写 RPP-style regulated-speed baseline；
不写 full Nav2 RPP reproduction；
完整 RPP controller node 留作 appendix/future work。
```

### 风险 4：external_profile_mode 与老 external_speed_profile_csv 冲突

处理: 完整规则见 §5.2 (CSV 透传约束 + 老 launch WARN 兼容)。要点摘要:

```text
external_profile_mode=none 时, 即便 csv 非空, LocalPlannerROS 在 setParams 前显式清空 csv,
  并发 ROS_WARN; PathHandler 不会按老 csv 加载。
external_profile_mode∈{ruckig,toppra,biagiotti,custom_csv} 时, csv 必须非空, 否则 validate ERROR 启动失败。
老 launch 不会启动失败但会 WARN, 行为切换到 mode=none。
```

## 12. 不做的事

本轮明确不做：

```text
1. 不移植完整 Nav2 RPP plugin (ROS1 移植取消, 附录外部控制器位用仓内 TEB, 见 §0.7)；
2. 不实现 costmap obstacle proximity cap (RPP-style 只做曲率 / approach cap)；
3. 不把 TEB 放进正文主表 (仅附录 P2P, 见 §0.4); 不新增 DWA 实车流程；
4. 不修改 slosh ODE 模型；
5. 不改 terminal d200 参数；
6. 不改变 /mpc/cost_breakdown 字段布局；
7. 不改变 RGB max-LCR 主指标口径。
```

注: TOPPRA / Ruckig / Biagiotti **不是**"不做的事" —— 它们是范式轴 baseline, 走 external_profile_mode
注入 (§4.2), 正式实验会跑 (TOPPRA/Ruckig supplementary, Biagiotti 进主表)。

## 13. 最小成功标准

完成后应满足：

```text
1. 默认启动 (experiment_group=LEGACY, 不传任何新参数) 与今天行为逐字节一致 ——
   即 yaml 原样 Q_slosh=0 的非晃液感知 MPC (今天默认本就不是 SloshPriorityMPC, 见 §4.0);
   LEGACY ≠ C ≠ F: C 是显式 ordinary MPC baseline, F 是显式 SloshPriorityMPC (ours),
   三者语义/bag 前缀/analysis 分组都分开 (§4.0 默认值与 LEGACY 语义);
2. 正式实验必须在 launch 命令中显式设置 experiment_group=C/D/E/F/RPP_STYLE/BIAGIOTTI/RUCKIG/TOPPRA;
   Q_slosh/R_a/R_da 仍按 20260527 SOP 显式传入, 其中 Q_slosh 被 group 约束 (违反即 FATAL);
3. 正式 group 与派生字段不匹配时 WARN 并覆盖; Q_slosh/CSV/内部互斥硬约束违反时
   configureExperimentVariant 阶段 ROS_FATAL 启动失败, 不进 controlLoop;
4. experiment_group=RPP_STYLE 在 P2 S 弯 sim smoke 必须通过 Phase C 的 4 条 hard assertion
   (RPP active + 弯心 curvature_active + approach 段线性下降 + 与 C 组速度差异 >= 0.05 m/s);
5. TOPPRA/Ruckig/Biagiotti 经各自 external_profile_mode 注入, 共用 C 的同一跟踪器 (范式轴公平);
   custom_csv 仅留作调试 / legacy 通道;
6. 所有正式实验共用同一套 launch (slosh_experiment.launch) 和 record_slosh_experiment.sh;
   bag 名前缀严格按 §4.0 命名规则 (slosh_<GROUP>_qs<Q>_..._<timestamp>.bag);
7. analysis 脚本只通过 experiment_group 字段 + bag 名前缀分组, 不再反推 (controller_variant,
   external_profile_mode, Q_slosh) 三元组;
8. 仿真 smoke 通过后才进入实物 smoke; Phase F 正式数据采集走 20260527 方案。
```
