# spmpc_local_planner

> 论文 Methods、公式、贡献点与写作边界参考：[`README_METHOD.md`](README_METHOD.md)。

`spmpc_local_planner` 是 Scout 液体运输实验中的 **SPMPC 规控一体局部规划器**。当前主线不是早期 Phase3 的 primitive smoke，也不是单纯的路径跟踪控制器，而是：

```text
slosh-aware continuous MPCC + acados SQP-RTI
```

它每个控制周期在同一个滚动时域优化问题中同时处理：

```text
局部轨迹几何
参考路径进度 s / v_s
底盘速度与角速度动态
contour / lag 路径误差
控制约束与平滑
液体模态状态预测
液面晃动代价
ROS 闭环执行与诊断发布
```

因此本文档中的 SPMPC 仍按“规划控制一体”口径理解：它不是“先给一条局部路径、再单独跟踪”，而是在 horizon 内同时优化局部运动、路径进度和第一帧控制命令。

当前 continuous MPCC 有两条需要区分的结构：

```text
continuous_mpcc_acados
  当前主线 alpha-state MPCC：omega 是状态，alpha=d(omega)/dt 是控制。

continuous_mpcc_direct_omega_legacy
  RouteB / direct-omega 诊断后端：omega 是控制，alpha_max 用作输出 cmd_omega rate clamp。
```

早期 `primitive` / `rollout` 后端仍保留为工程 fallback、smoke test 或附录 baseline，但不再是当前论文和实物主实验的核心叙事。

---

## 1. 规划器总体结构

当前包的核心数据流如下：

```text
/odom + /scout/global_path_fixed + optional /map
        │
        ▼
ROS Adapter: SpmpcLocalPlannerROS
        │
        ├─ 读取参数、订阅状态、接收路径、维护 TF / odom / costmap
        ├─ 将 ROS Path / OccupancyGrid 转为内部 ReferencePath / CostmapGrid
        └─ 将求解结果发布为 /cmd_vel、/spmpc/local_trajectory 和 /spmpc/* diagnostics
        │
        ▼
SpmpcProblem
        │
        ├─ 保存 ReferencePath
        ├─ 维护 progress / costmap / solver input
        └─ 通过 SolverFactory 创建具体后端
        │
        ▼
SpmpcSolver interface
        │
        ├─ continuous_mpcc_acados                 当前主线：alpha-state continuous MPCC + acados
        ├─ continuous_mpcc_direct_omega_legacy    RouteB 诊断：direct-omega continuous MPCC
        └─ primitive                              保留后端：rollout / primitive argmin fallback
        │
        ▼
SolverOutput
        │
        ├─ 第一帧 cmd_v / cmd_omega
        ├─ 预测轨迹
        ├─ slosh horizon summary
        ├─ solver status / backend / solve time
        └─ cost breakdown
```

其中 `/cmd_vel.linear.x` 由当前速度和最优加速度积分得到；`v_s` 是 MPCC 中的虚拟路径进度速度，不等同于底盘线速度。

---

## 2. 目录职责

```text
include/spmpc_local_planner/
├── core/          通用数据结构、Solver 抽象、SpmpcProblem、costmap、primitive fallback
├── reference/     ReferencePath、路径投影、spline / MPCC 参考路径工具
├── dynamics/      SloshDynamics，对接 slosh_models 的液体模态动力学
├── solvers/       SolverFactory 与 continuous MPCC acados backends
├── ros/           ROS node wrapper、diagnostics publisher
├── costs/         预留：后续独立代价模块
├── constraints/   预留：后续独立约束模块
└── terminal/      预留：后续终端项模块

src/
├── core/
├── reference/
├── dynamics/
├── solvers/
└── ros/

config/
├── planner/       通用规划器参数与 variant 配置
├── platforms/     Scout Mini 平台限制
├── containers/    容器和液体参数
└── experiments/   fixed_path / point_to_point 实验配置

launch/            ROS 启动入口
scripts/           实物实验脚本、smoke、录包、分析与 acados codegen
scripts/acados/    生成 alpha-state 与 direct-omega acados solver
generated/acados/  acados generated solver artifacts
```

设计边界：

```text
core / reference / dynamics / solvers  尽量保持算法逻辑
ros                                    负责 ROS I/O 和消息转换
scripts/acados                         负责模型生成
generated/acados                       负责 generated solver
```

当前 CMake 仍以一个主要 planner library 组织 core、solver 和 ROS 代码；目录层面已经分层，但 target 级别未来可进一步拆为 `spmpc_core` 与 `spmpc_ros`。

### 2.1 Scout Mini 实物尺寸 / footprint 口径

2026-06-25 已按 `docs/实物实验注意事项/代码移植/实物说明书.md` 复核 Scout Mini 实物尺寸。说明书关键参数：

```text
长×宽×高: 612 × 580 × 245 mm
轴距:     451 mm
前/后轮距: 490 mm
最高速度: 1.5 m/s
最小转弯半径: 0 m
```

SPMPC 平台配置文件为：

```text
config/platforms/scout_mini.yaml
```

当前写入的 package-local 尺寸源如下：

```yaml
robot:
  v_max: 0.8
  omega_max: 1.2
  a_max: 0.6
  alpha_max: 1.2
  geometry:
    length: 0.612
    width: 0.580
    height: 0.245
    wheelbase: 0.451
    track_width: 0.490
    footprint:
      type: polygon
      vertices: [[0.31, 0.2925], [0.31, -0.2925], [-0.31, -0.2925], [-0.31, 0.2925]]
    circumscribed_radius: 0.426
    inscribed_radius: 0.2925
```

解释：

- `v_max=0.8 m/s` 小于说明书最高速度 `1.5 m/s`，是 formal comparison 的保守速度上限。
- `platform/kinematics=differential` 与四轮差速转向、最小转弯半径 `0 m` 一致。
- footprint 使用 `0.620 m × 0.585 m` 保守矩形，覆盖说明书 `0.612 m × 0.580 m`，并与 TEB / MPC local planner baseline 里的 footprint 一致。
- 当前主线 `continuous_mpcc_acados` 直接使用速度/角速度/加速度约束；footprint 目前作为实物尺寸和后续 collision/costmap 口径的 source-of-truth，不代表 OCP 内已经加入完整车体多边形碰撞约束。
- `config/experiments/fixed_path.yaml` 与 `config/experiments/point_to_point.yaml` 中 `obstacle_enable` 仍默认关闭；如果后续打开 obstacle cost，`obstacle_influence_radius` 已改为 `0.45 m`，覆盖上述 conservative circumscribed footprint。

因此，当前固定路径无障碍仿真矩阵在运动能力层面符合实物且偏保守；若进入带障碍物/狭窄通道/实物安全验证，必须继续确认 costmap inflation、footprint collision 与现场安全边界，不得只依赖质点 MPCC。

尺寸参数写入后的 no-regression 验证：2026-06-25 使用 strict fresh-sim S 曲线 N=3 复跑 `spmpc/B_ours`，显式地图 `/data/a/scout_sim_replacement/maps/proxy_world_manual_saved_20260611_154348.pbstream`，结果 `3/3 GOAL_REACHED`，strict freshness invalid `0`。结果目录：`/data/a/scout_sim_replacement/results/strict_fresh_fair_n3_20260625_234232_spmpc_after_size_params_n3`；tracking RMS mean `0.024 m`，tracking max mean `0.042 m`，final error mean `0.182 m`。这说明新增的 `robot/geometry` 和未启用 obstacle radius 调整没有破坏当前 fixed-path 主线行为。

---

## 3. 方法主线：alpha-state continuous MPCC

### 3.1 状态与控制

当前主线 `continuous_mpcc_acados` 将 `omega` 提升为 OCP 状态，将角加速度 `alpha` 作为控制输入。

B0 / non-slosh 模型为 6D：

```text
x_b0 = [px, py, theta, v, s, omega]
u    = [a, alpha, v_s]
```

slosh-aware 模型为 10D：

```text
x_slosh = [px, py, theta, v, s, omega, eta_x, eta_x_dot, eta_y, eta_y_dot]
u       = [a, alpha, v_s]
```

含义：

```text
a       底盘切向加速度
alpha   底盘角加速度，即 omega_dot
v_s     虚拟路径进度速度，用于推进 MPCC 路径参数 s
omega   底盘角速度状态
```

对应输出边界为：

```text
cmd_vel.linear.x  = clamp(v_current + a_0 * dt, 0, v_max)
cmd_vel.angular.z = clamp(omega_current + alpha_0 * dt, -omega_max, omega_max)
```

### 3.2 路径误差与进度

MPCC 不跟踪固定时间索引轨迹，而是在参考路径上优化路径进度 `s`，并通过：

```text
contour error       法向轮廓误差
lag error           切向滞后误差
progress reward     路径进度奖励
v / v_s tracking    物理速度和虚拟进度速度 anti-creep
```

共同决定局部运动。

当前 anti-creep 相关参数来自 variant 配置：

```text
w_v     物理速度 v 对 v_ref 的 tracking penalty
w_vs    虚拟进度速度 v_s 对 v_ref 的 tracking penalty
v_ref   参考速度，当前统一初值 0.25 m/s
```

`/spmpc/cost_breakdown` 的字段布局保持不变；当前 `J_v` 口径表示物理速度 `v` 与虚拟进度速度 `v_s` tracking penalty 的合计。

### 3.3 液体模态

`dynamics/SloshDynamics` 将底盘加速度映射到二维液体模态：

```text
eta_x, eta_x_dot, eta_y, eta_y_dot
```

模型预测液面高度为：

```text
h_model = c_h * sqrt(eta_x^2 + eta_y^2)
```

该量用于控制器内部优化和 `/spmpc/*` 诊断，是模型 proxy；论文和实物报告中的真实液面指标以离线 RGB max-LCR 为准。

在 alpha-state 主线里，slosh 动力学使用预测状态中的 `omega`；在 RouteB direct-omega 里，slosh 动力学使用控制输入中的 `omega`。

---

## 4. RouteB / direct-omega 诊断结构

`continuous_mpcc_direct_omega_legacy` 是为定位 alpha-state stall / anti-chatter 问题保留的诊断后端。它仍是 continuous MPCC，因为局部轨迹、路径进度和第一帧控制仍在同一个 OCP 中优化；区别是角速度处理方式不同。

B0 direct-omega 模型为 5D：

```text
x_b0_direct = [px, py, theta, v, s]
u           = [a, omega, v_s]
```

slosh direct-omega 生成入口为 9D：

```text
x_slosh_direct = [px, py, theta, v, s, eta_x, eta_x_dot, eta_y, eta_y_dot]
u              = [a, omega, v_s]
```

输出边界为：

```text
cmd_omega_raw = clamp(omega_0, -omega_max, omega_max)
cmd_omega     = clamp(cmd_omega_raw, prev_cmd_omega ± alpha_max * dt)
cmd_v         = clamp(v_current + a_0 * dt, 0, v_max)
```

注意事项：

```text
1. RouteB 中 alpha_max 不是 OCP 内部控制约束，而是输出 cmd_omega rate limit。
2. B0 direct-omega 已用于 RouteB 诊断；slosh direct-omega 有 codegen / link 入口，但 formal 使用仍需单独验证。
3. RouteB 与 alpha-state 改变了 OCP 状态/控制结构，不能混进同一主表作为普通 slosh 消融。
```

当前 P2 RouteB B0 诊断结论：

```text
alpha_max = 20 / 4 / 3.5 均能到达终点。
alpha_max = 3.0 虽能到达，但 solve fail 多、路径明显变长。
alpha_max = 8.0 更快，但出现左摇右晃。
当前 B0 候选工作点倾向 3.5~4.0 rad/s^2，其中 3.5 表现最好。
```

---

## 5. Solver 后端

### 5.1 continuous_mpcc_acados：当前主线

`continuous_mpcc_acados` 是当前主线后端。它通过 acados generated solver 求解 alpha-state continuous MPCC OCP。

生成模型：

```text
spmpc_b0      6D alpha-state baseline continuous MPCC
spmpc_slosh   10D alpha-state slosh-aware continuous MPCC
```

编译宏：

```text
SPMPC_WITH_ACADOS
SPMPC_WITH_ACADOS_SLOSH
```

若 acados 或 generated solver 不可用，后端会退化为 stub，并通过 `/spmpc/status` 报告类似：

```text
ACADOS_NOT_IMPLEMENTED
ACADOS_NOT_CREATED
ACADOS_SOLVE_FAILED_*
```

### 5.2 continuous_mpcc_direct_omega_legacy：RouteB 诊断后端

生成模型：

```text
spmpc_b0_direct_omega_legacy  5D direct-omega B0
spmpc_slosh_direct_omega      9D direct-omega slosh，formal 使用需验证
```

编译宏：

```text
SPMPC_WITH_ACADOS_B0_DIRECT_OMEGA_LEGACY
SPMPC_WITH_ACADOS_SLOSH_DIRECT_OMEGA
```

典型状态：

```text
B0_ACADOS_DIRECT_OMEGA_LEGACY_OK
ACADOS_DIRECT_OMEGA_SOLVE_FAILED_*
```

### 5.3 primitive：保留 fallback / baseline

`primitive` 后端基于候选控制序列 rollout 和 argmin 评分。它仍可用于：

```text
早期 smoke
工程 fallback
primitive / anti-primitive 附录消融
与 continuous MPCC 的工程对照
```

但当前论文 Methods 和实物主实验不再把它作为核心方法。

---

## 6. 主实验变体

当前 alpha-state 主实验应在同一个 `continuous_mpcc_acados` 后端下比较：

| variant | backend | generated model | 状态维度 | slosh 状态/代价 | smooth | 用途 |
|---|---|---|---:|---|---|---|
| `B0` | `continuous_mpcc_acados` | `spmpc_b0` | 6D | 否 | 否 | 基础 alpha-state continuous MPCC baseline |
| `B_smooth` | `continuous_mpcc_acados` | `spmpc_b0` | 6D | 否 | 是 | 只看控制平滑是否降晃 |
| `B_slosh` | `continuous_mpcc_acados` | `spmpc_slosh` | 10D | 是 | 否 | 只看 slosh-aware 模型/代价是否有效 |
| `B_ours` | `continuous_mpcc_acados` | `spmpc_slosh` | 10D | 是 | 是 | 完整方法 |

核心对照关系：

```text
B_slosh vs B0        slosh 模型/代价是否有效
B_smooth vs B0       仅靠控制平滑是否有效
B_ours  vs B_smooth  slosh-aware 是否优于 smooth-only
B_ours  vs B0        最终方法总体收益
```

可选附录组：

```text
B_accel
B_slosh_linear
B_slosh_anti
B_ours_anti
primitive backend
RouteB direct-omega backend
external planner baselines
```

实验公平性注意：

```text
1. SPMPC 内部主表必须固定同一个 solver backend、同一个 alpha_max、同一组 warm-start / terminal / reference 设置。
2. alpha-state vs RouteB 是 backend / OCP 结构消融，不是普通 slosh-aware 消融。
3. 若 RouteB 成为正式候选，所有 SPMPC variants 必须使用同一个 RouteB backend 与同一个 alpha_max。
4. TEB/DWA common-limit 对比需要同步普通动力学限制；若 RouteB 用 alpha_max=3.5，而 TEB/DWA acc_lim_theta 仍为 1.2，不能称为完全 common-limit。
```

---

## 7. 与 RGB 液面测量的边界

`spmpc_local_planner` 不依赖在线 RGB 识别包，也不打开相机。

边界如下：

```text
spmpc_local_planner
  负责 continuous MPCC、slosh 模型预测、/cmd_vel、/spmpc/* diagnostics

realsense_liquid_measurement
  负责在线 RGB 液面识别、debug image、monitor dashboard、离线 bag RGB 推断
```

重要区分：

```text
/spmpc/slosh_height      模型预测液面 proxy，发布边界单位为 mm
/liquid/*                在线 RGB 调试 proxy，不进入控制回路
离线 RGB max-LCR          论文/报告主评价真值
```

实物脚本默认要求录制 RGB 原始图像用于离线真值，但默认不录 `/liquid/*` 在线 proxy。只有显式设置：

```bash
RECORD_ONLINE_LIQUID=true
```

才会把在线 `/liquid/*` 同步进 bag。

---

## 8. 配置文件

```text
config/planner/common.yaml             通用 MPCC / solver / warm-start / terminal 参数
config/planner/variants.yaml           B0 / B_smooth / B_slosh / B_ours 等变体
config/platforms/scout_mini.yaml       Scout Mini 速度、加速度、角速度、角加速度限制
config/containers/tube_default.yaml    容器和液体模态参数
config/experiments/fixed_path.yaml     固定路径实验模板
config/experiments/point_to_point.yaml 点到点 smoke / 工程测试模板
```

原则：

```text
平台限制放 platforms/
液体和容器参数放 containers/
方法变体差异放 planner/variants.yaml
实验入口参数放 experiments/
```

当前关键默认值：

```text
robot/v_max      = 0.8
robot/omega_max  = 1.2
robot/a_max      = 0.6
robot/alpha_max  = 1.2

acados/warm_start/enable                        = true
acados/warm_start/fallback_to_previous_solution = false
acados/warm_start/fallback_to_primitive         = true

terminal/goal_tolerance = 0.20
```

`launch` 中的 `alpha_max:=-1.0` 是哨兵值，表示使用 platform yaml 默认值；非负值才会覆盖 `robot/alpha_max`。

---

## 9. 构建与 generated solver

### 9.1 生成 acados solver

alpha-state 主线需要生成两个 solver：

```bash
source /opt/ros/noetic/setup.bash
source /home/a/acados_venv/bin/activate
export ACADOS_SOURCE_DIR=/home/a/acados
export LD_LIBRARY_PATH=/home/a/acados/lib:${LD_LIBRARY_PATH:-}
cd /home/a/scout_ws/src/scout_apps/control/spmpc_local_planner/scripts/acados

python3 generate_spmpc_acados.py --model b0
python3 generate_spmpc_acados.py --model slosh
```

RouteB / direct-omega 诊断需要额外生成：

```bash
python3 generate_spmpc_acados.py --model b0_direct_omega_legacy
python3 generate_spmpc_acados.py --model slosh_direct_omega
```

可先做装配检查：

```bash
python3 generate_spmpc_acados.py --check --model b0
python3 generate_spmpc_acados.py --check --model slosh
python3 generate_spmpc_acados.py --check --model b0_direct_omega_legacy
python3 generate_spmpc_acados.py --check --model slosh_direct_omega
```

### 9.2 编译 planner

```bash
cd /home/a/scout_ws
source /opt/ros/noetic/setup.bash
source /home/a/acados_venv/bin/activate
export ACADOS_SOURCE_DIR=/home/a/acados
export LD_LIBRARY_PATH=/home/a/acados/lib:${LD_LIBRARY_PATH:-}
catkin_make --force-cmake -DCATKIN_WHITELIST_PACKAGES="spmpc_local_planner" --pkg spmpc_local_planner
source devel/setup.bash
```

构建时应看到 continuous/acados 后端相关信息。若运行时 `/spmpc/status=ACADOS_NOT_IMPLEMENTED`，通常说明当前构建没有找到 acados 或 generated solver。

---

## 10. 启动入口

### 10.1 固定路径 alpha-state continuous MPCC

```bash
roslaunch spmpc_local_planner spmpc_fixed_path.launch \
  planner_variant:=B_ours \
  solver_backend:=continuous_mpcc_acados
```

### 10.2 固定路径 RouteB / direct-omega 诊断

```bash
roslaunch spmpc_local_planner spmpc_fixed_path.launch \
  planner_variant:=B0 \
  solver_backend:=continuous_mpcc_direct_omega_legacy \
  alpha_max:=3.5
```

### 10.3 点到点工程测试

```bash
roslaunch spmpc_local_planner spmpc_point_to_point.launch \
  planner_variant:=B0 \
  solver_backend:=continuous_mpcc_acados
```

### 10.4 实物固定路径主脚本

正式实物对比实验推荐使用封装脚本：

```bash
DATE=<DATE> \
GOAL_X=<x> GOAL_Y=<y> GOAL_YAW=<yaw> \
VARIANT=B_ours \
SOLVER_BACKEND=continuous_mpcc_acados \
bash src/scout_apps/control/spmpc_local_planner/scripts/run_continuous_real.sh
```

该脚本负责固定路径生成、planner 启动、录包、metadata 和基本 preflight。详细实验顺序见：

```text
docs/实物实验注意事项/对比试验/20260603_SPMPC连续MPCC实物对比实验SOP.md
```

当前注意：`run_continuous_real.sh` 已支持 `SOLVER_BACKEND`，但尚未把 `alpha_max` 运行时覆盖透传到 `spmpc_fixed_path.launch`。如果 RouteB 实物实验需要使用仿真诊断得到的 `alpha_max=3.5`，正式跑前需先补脚本透传，或手动 roslaunch 传 `alpha_max:=3.5`。

---

## 11. 诊断话题

主要诊断输出：

```text
/spmpc/status                    求解器状态
/spmpc/solver_backend            当前后端，例如 continuous_mpcc_acados
/spmpc/controller_variant         当前 variant
/spmpc/experiment_mode            fixed_path / point_to_point 等
/spmpc/local_trajectory           预测局部轨迹
/spmpc/debug/progress_s           当前路径进度
/spmpc/debug/slosh_state          模态状态 proxy
/spmpc/debug/predicted_horizon    完整 N+1 状态、N 控制预测时域
/spmpc/debug/pre_solve_snapshot   solve() 前 OCP 参数、边界和 warm-start 原始变量快照
/spmpc/slosh_height               模型预测液面高度 proxy，发布单位 mm
/spmpc/slosh_horizon_summary      预测时域晃动摘要
/spmpc/solver_time_ms             求解耗时
/spmpc/cost_breakdown             代价分解
/spmpc/corridor                   corridor 诊断
/spmpc/guidance                   guidance 诊断，主要用于 fallback / legacy
/spmpc/primitive                  primitive 诊断，主要用于 fallback / legacy
```

其中两个 replay 诊断使用自定义消息：

- `PredictedHorizon`：记录 `t,x,y,yaw,v,omega,s`、四个液体模态状态、`h_modal`，以及每阶段的 `a,alpha_or_omega,v_s`。默认连续 MPCC 时域为 60，因此应有 61 个状态和 60 个控制。
- `PreSolveSnapshot`：记录本周期真实 solver 输入、参考曲线三次多项式系数、有效 `v_ref`、上一控制/上一解、运行时边界、全部阶段参数，以及紧邻 `solve()` 前写入 acados 的完整原始变量初值。B0 的阶段参数应为 `61 x 23`，含液体模型的 variant 应为 `61 x 32`。

这两个消息当前只由论文主线后端 `continuous_mpcc_acados` 完整填充；其他后端会发布 `valid=false`。`PreSolveSnapshot.primal_guess_only=true` 表示尚未记录对偶变量和 acados 内部 SQP memory。因此它提供了 actual/zero replay 所需的完整显式原始变量上下文，但不能单凭消息存在就宣称逐位精确复现。正式采集前必须验证离线 actual 分支能够在冻结容差内复现在线 solver status、第一控制量和 raw command；若失败，再补录对偶变量或内部求解器状态。

不要复用 `/mpc/cost_breakdown`，避免污染 `scout_local_planner` 既有分析链路。

---

## 12. 录包口径

实物主实验建议使用 `run_continuous_real.sh`，默认记录控制、状态、路径、诊断和 RGB 原始图像：

```text
/spmpc/*
/slosh/*
/odom
/cmd_vel
/tf
/camera/color/image_raw
/camera/color/camera_info
```

`run_continuous_real.sh` 和 `record_spmpc_full_rgb_bag.sh` 均已显式包含 `/spmpc/debug/predicted_horizon` 与 `/spmpc/debug/pre_solve_snapshot`。正式 run 前按 `scripts/README.md` 的 replay 话题 smoke 检查确认消息有效且数组尺寸正确。

其中 RGB 原始图像用于离线推断真实液面高度。在线 `/liquid/*` 仅是调试 proxy，默认不录；需要时显式设置：

```bash
RECORD_ONLINE_LIQUID=true
```

也可使用较底层录包脚本：

```bash
OUT_DIR=/tmp/spmpc_bags NAME=spmpc_debug \
  src/scout_apps/control/spmpc_local_planner/scripts/record_spmpc_experiment.sh
```

fixed-path 仿真对比、fresh-sim 规则和 metrics 提取见：

```text
src/scout_apps/control/spmpc_experiments/scripts/README.md
docs/实物实验注意事项/实物实验前的仿真/
```

---

## 13. 维护说明

当前包的长期结构目标是：

```text
算法核心可独立于 ROS 测试
ROS adapter 只做 I/O 和消息转换
continuous MPCC 是主线 backend
primitive 后端只做 fallback / baseline
RouteB direct-omega 作为诊断/结构消融，不默认混入主表
RGB 在线分析保持在 sensors/realsense_liquid_measurement
```

若后续继续重构，优先级建议为：

```text
1. CMake target 拆分为 spmpc_core / spmpc_ros
2. SolverParams 按 robot/reference/slosh/acados/primitive 分组
3. 将 rollout solver 内部的 cost / constraints / terminal 逻辑迁入对应目录
4. 保持 acados generated code 只影响 solvers/*_acados.cpp 与 scripts/acados
5. 若 RouteB 进入 formal 主线，先补齐 slosh direct-omega 验证与 run_continuous_real.sh 的 alpha_max 透传
```
