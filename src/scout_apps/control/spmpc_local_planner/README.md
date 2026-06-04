# spmpc_local_planner

`spmpc_local_planner` 是 Scout 液体运输实验中的 **SPMPC 规控一体局部规划器**。当前主线不是早期 Phase3 的 primitive smoke，也不是单纯的路径跟踪控制器，而是：

```text
slosh-aware continuous MPCC + acados SQP-RTI
```

它在同一个滚动时域优化问题中同时处理：

```text
底盘运动预测
参考路径进度推进
contour / lag 路径误差
控制约束与平滑
液体模态状态预测
液面晃动代价
ROS 闭环执行与诊断发布
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
        ├─ continuous_mpcc_acados   当前主线：连续 MPCC + acados SQP-RTI
        └─ primitive                保留后端：rollout / primitive argmin fallback
        │
        ▼
SolverOutput
        │
        ├─ 首步控制 a, omega, v_s
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
├── solvers/       SolverFactory 与 continuous MPCC acados backend
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
scripts/acados/    生成 spmpc_b0 / spmpc_slosh acados solver
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

---

## 3. 方法主线：continuous MPCC

### 3.1 状态与控制

基础连续 MPCC 使用 5D 状态：

```text
x_b0 = [px, py, theta, v, s]
```

slosh-aware 变体使用 9D 状态：

```text
x_slosh = [px, py, theta, v, s, eta_x, eta_x_dot, eta_y, eta_y_dot]
```

控制输入为：

```text
u = [a, omega, v_s]
```

含义：

```text
a       底盘切向加速度
omega   底盘角速度
v_s     虚拟路径进度速度，用于推进 MPCC 路径参数 s
```

### 3.2 路径误差

MPCC 不跟踪固定时间索引轨迹，而是在参考路径上优化路径进度 `s`，并通过：

```text
contour error   法向轮廓误差
lag error       切向滞后误差
progress reward 路径进度奖励
```

共同决定局部运动。

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

---

## 4. Solver 后端

### 4.1 continuous_mpcc_acados：当前主线

`continuous_mpcc_acados` 是当前实物主线后端。它通过 acados generated solver 求解连续 MPCC OCP。

生成模型：

```text
spmpc_b0      5D baseline continuous MPCC
spmpc_slosh   9D slosh-aware continuous MPCC
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

### 4.2 primitive：保留 fallback / baseline

`primitive` 后端基于候选控制序列 rollout 和 argmin 评分。它仍可用于：

```text
早期 smoke
工程 fallback
primitive / anti-primitive 附录消融
与 continuous MPCC 的工程对照
```

但当前论文 Methods 和实物主实验不再把它作为核心方法。

---

## 5. 主实验变体

当前主实验在同一个 `continuous_mpcc_acados` 后端下比较：

| variant | backend | generated model | 状态维度 | slosh 状态/代价 | smooth | 用途 |
|---|---|---|---:|---|---|---|
| `B0` | `continuous_mpcc_acados` | `spmpc_b0` | 5D | 否 | 否 | 基础 continuous MPCC baseline |
| `B_smooth` | `continuous_mpcc_acados` | `spmpc_b0` | 5D | 否 | 是 | 只看控制平滑是否降晃 |
| `B_slosh` | `continuous_mpcc_acados` | `spmpc_slosh` | 9D | 是 | 否 | 只看 slosh-aware 模型/代价是否有效 |
| `B_ours` | `continuous_mpcc_acados` | `spmpc_slosh` | 9D | 是 | 是 | 完整方法 |

核心对照关系：

```text
B_slosh vs B0        slosh 模型/代价是否有效
B_smooth vs B0       仅靠控制平滑是否有效
B_ours  vs B_smooth  slosh-aware 是否优于 smooth-only
B_ours  vs B0        最终方法总体收益
```

可选附录组：

```text
B_slosh_linear
B_slosh_anti
B_ours_anti
primitive backend
external planner baselines
```

这些不混入主表，避免同时改变“求解器形式”和“是否 slosh-aware”。

---

## 6. 与 RGB 液面测量的边界

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
/spmpc/slosh_height      模型预测液面 proxy，不是真值
/liquid/*                在线 RGB 调试 proxy，不进入控制回路
离线 RGB max-LCR          论文/报告主评价真值
```

实物脚本默认要求录制 RGB 原始图像用于离线真值，但默认不录 `/liquid/*` 在线 proxy。只有显式设置：

```bash
RECORD_ONLINE_LIQUID=true
```

才会把在线 `/liquid/*` 同步进 bag。

---

## 7. 配置文件

```text
config/planner/common.yaml       通用 MPCC / solver / cost 参数
config/planner/variants.yaml     B0 / B_smooth / B_slosh / B_ours 等变体
config/platforms/scout_mini.yaml Scout Mini 速度、加速度、角速度限制
config/containers/tube_default.yaml 容器和液体模态参数
config/experiments/fixed_path.yaml  固定路径实验模板
config/experiments/point_to_point.yaml 点到点 smoke / 工程测试模板
```

原则：

```text
平台限制放 platforms/
液体和容器参数放 containers/
方法变体差异放 planner/variants.yaml
实验入口参数放 experiments/
```

---

## 8. 构建与 generated solver

### 8.1 生成 acados solver

实物主线需要生成两个 solver：

```bash
source /opt/ros/noetic/setup.bash
source ~/acados_venv/bin/activate
cd /home/geist/scout_ws/src/scout_apps/control/spmpc_local_planner/scripts/acados

python generate_spmpc_acados.py --model b0
python generate_spmpc_acados.py --model slosh

deactivate
```

### 8.2 编译 planner

```bash
cd /home/geist/scout_ws
source /opt/ros/noetic/setup.bash
source ~/.bashrc        # ACADOS_SOURCE_DIR / LD_LIBRARY_PATH
catkin_make --pkg spmpc_local_planner --force-cmake
source devel/setup.bash
```

构建时应看到 continuous/acados 后端相关信息。若运行时 `/spmpc/status=ACADOS_NOT_IMPLEMENTED`，通常说明当前构建没有找到 acados 或 generated solver。

---

## 9. 启动入口

### 9.1 固定路径 continuous MPCC

```bash
roslaunch spmpc_local_planner spmpc_fixed_path.launch \
  planner_variant:=B_ours \
  solver_backend:=continuous_mpcc_acados
```

### 9.2 点到点工程测试

```bash
roslaunch spmpc_local_planner spmpc_point_to_point.launch \
  planner_variant:=B0 \
  solver_backend:=continuous_mpcc_acados
```

### 9.3 实物固定路径主脚本

正式实物对比实验推荐使用封装脚本：

```bash
DATE=<DATE> \
GOAL_X=<x> GOAL_Y=<y> GOAL_YAW=<yaw> \
VARIANT=B_ours \
bash src/scout_apps/control/spmpc_local_planner/scripts/run_continuous_real.sh
```

该脚本负责固定路径生成、planner 启动、录包、metadata 和基本 preflight。详细实验顺序见：

```text
docs/实物实验注意事项/对比试验/20260603_SPMPC连续MPCC实物对比实验SOP.md
```

---

## 10. 诊断话题

主要诊断输出：

```text
/spmpc/status                    求解器状态
/spmpc/solver_backend            当前后端，例如 continuous_mpcc_acados
/spmpc/controller_variant         当前 variant
/spmpc/experiment_mode            fixed_path / point_to_point 等
/spmpc/local_trajectory           预测局部轨迹
/spmpc/debug/progress_s           当前路径进度
/spmpc/debug/slosh_state          模态状态 proxy
/spmpc/slosh_height               模型预测液面高度 proxy
/spmpc/slosh_horizon_summary      预测时域晃动摘要
/spmpc/solver_time_ms             求解耗时
/spmpc/cost_breakdown             代价分解
/spmpc/corridor                   corridor 诊断
/spmpc/guidance                   guidance 诊断，主要用于 fallback / legacy
/spmpc/primitive                  primitive 诊断，主要用于 fallback / legacy
```

不要复用 `/mpc/cost_breakdown`，避免污染 `scout_local_planner` 既有分析链路。

---

## 11. 录包口径

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

其中 RGB 原始图像用于离线推断真实液面高度。在线 `/liquid/*` 仅是调试 proxy，默认不录；需要时显式设置：

```bash
RECORD_ONLINE_LIQUID=true
```

也可使用较底层录包脚本：

```bash
OUT_DIR=/tmp/spmpc_bags NAME=spmpc_debug \
  src/scout_apps/control/spmpc_local_planner/scripts/record_spmpc_experiment.sh
```

---

## 12. 维护说明

当前包的长期结构目标是：

```text
算法核心可独立于 ROS 测试
ROS adapter 只做 I/O 和消息转换
continuous MPCC 是主线 backend
primitive 后端只做 fallback / baseline
RGB 在线分析保持在 sensors/realsense_liquid_measurement
```

若后续继续重构，优先级建议为：

```text
1. CMake target 拆分为 spmpc_core / spmpc_ros
2. SolverParams 按 robot/reference/slosh/acados/primitive 分组
3. 将 rollout solver 内部的 cost / constraints / terminal 逻辑迁入对应目录
4. 保持 acados generated code 只影响 solvers/continuous_mpcc_solver_acados.cpp 与 scripts/acados
```
