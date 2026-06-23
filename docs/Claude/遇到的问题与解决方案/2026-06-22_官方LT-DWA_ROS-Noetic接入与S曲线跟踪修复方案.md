# LT-DWA 官方 ROS Noetic 接入与 S 曲线跟踪修复方案

**项目：** 张仁军 / SPMPC 对比实验
**日期：** 2026-06-22
**目标：** 用作者公开的 LT-DWA ROS Noetic 实现替换当前不完整的 `LT-DWA-v2-inspired` 适配器，建立可复现、公平、可进入论文对比表的现代 local-planner baseline。

---

## 0. 最终决策

当前建议是：

```text
停止继续盲调现有 LT-DWA-v2-inspired 的 path/progress 权重；
保留它作为问题诊断和附录资产；
主线转向作者公开的完整 LT-DWA ROS Noetic 实现。
```

原因不是“官方代码一定优于 DWA”，而是当前自研适配器只复用了：

```text
DWA 工程框架
+ 长视域候选生成/评分思想
```

并未完整实现原论文的：

```text
Reference Navigation Path
+ Time-Varying Distance Fields
+ Long-Term DWA state-cost tree
+ voxel sampling
+ EB-MPC graph optimization
```

原论文明确采用 LT-DWA 生成长时域初始状态序列，再通过 Elastic-Band Model Predictive Control（EB-MPC）进行图优化，以提高安全性并降低控制抖动 [1]。作者仓库提供 Ubuntu 20.04 / ROS Noetic 实现，并包含 `seed_policy.cpp` 和 `eb_mpc_trajectory_optimizer.cpp` 等完整模块 [2]。

> **注意：** 官方代码不是一个可直接替换 `dwa_local_planner` 的标准 `nav_core` 插件。其公开主程序带有作者仿真循环和 `getchar()` 启动逻辑，因此仍需要接口适配，但应尽量保持核心 LT-DWA + EB-MPC 算法不变。

---

# 1. 当前问题结论

根据当前 `LT-DWA-v2` S 曲线问题记录，失败现象为：

```text
match_progress_s: 0.00 → 5.90 m
match_dist:       0.00 → 1.59 m
heading error:    最高约 81°
final status:     TRACKING_DIVERGED
```

与此同时：

```text
cmd_w max ≈ 0.54 rad/s
odom_w max ≈ 0.54 rad/s
omega_max = 1.2 rad/s
```

因此当前证据不支持“角速度上限不足”是主要原因，而更支持：

```text
1. 单一 lookahead target 不能表达完整 S 弯；
2. 候选切弯/偏离后仍能获得 progress 奖励；
3. 缺少完整 reference-navigation-path stage cost；
4. 缺少 EB-MPC 对 seed trajectory 的二次优化；
5. 大误差后缺少明确 recovery；
6. 实际重新规划频率可能低于 DWA。
```

当前适配器应正式命名为：

```text
LT-DWA-inspired experimental adapter
```

不要在论文中称为完整 LT-DWA 复现。

---

# 2. 官方 LT-DWA 与当前适配器的差异

| 组成 | 官方 LT-DWA | 当前 LT-DWA-v2-inspired | 风险 |
|---|---|---|---|
| 参考构造 | 随预测帧变化的 reference navigation path | 单一/有限 lookahead local target | S 弯易切弯 |
| 长视域搜索 | 分层 state-cost tree | 简化长视域 rollout/评分 | 候选覆盖不足 |
| 状态降采样 | voxel sampling | 自定义候选筛选 | 搜索分布不同 |
| 障碍表示 | time-varying distance fields | 当前工程 costmap/简化评分 | 非原论文结构 |
| 二次优化 | EB-MPC + g2o | 无完整 EB-MPC | seed 抖动、误差无法修正 |
| 控制连续性 | 优化阶段显式降低 jitter | 主要依赖评分和平滑项 | 转向换向质量差 |
| 公开实现 | Ubuntu 20.04 + ROS Noetic | Scout 自研包 | 当前不能算官方复现 |
| 论文口径 | 完整 LT-DWA | inspired adapter | 必须分开命名 |

原论文将 LT-DWA 定位为**初始状态序列生成器**，随后再用 EB-MPC 优化；论文消融也显示优化阶段可以显著降低线加速度和角加速度抖动 [1]。

---

# 3. 总体接入架构

推荐保持官方代码为 vendor 核心，在其外围增加 Scout 接口层：

```text
/scout/global_path_fixed ─┐
/odom                     ├──> lt_dwa_official_bridge
/tf                       │       ├── reference path adapter
/static map / costmap     │       ├── robot-state adapter
                          │       └── environment adapter
                          ▼
                Official LT-DWA Core
          Reference Path + LT-DWA Tree + EB-MPC
                          ▼
                official command / trajectory
                          ▼
              benchmark command adapter
                          ▼
              /benchmark/cmd_vel_raw
                          ▼
                 common command gate
                          ▼
                        Scout
```

## 3.1 三个包的建议边界

```text
third_party/lt_dwa_official/
    作者代码，固定 commit，尽量不改算法

lt_dwa_official_bridge/
    ROS topic / frame / path / map / command 接口适配

lt_dwa_official_benchmark/
    launch、参数冻结、诊断、rosbag、结果分析
```

### 核心原则

```text
1. 不把 Scout 业务逻辑塞进官方 LT-DWA 核心；
2. 不让官方代码直接控制实物底盘；
3. 所有输出先进入 /benchmark/cmd_vel_raw；
4. 统一通过现有 common command gate；
5. 任何核心算法修改都记录 patch 和 config hash。
```

---

# 4. 分阶段实施方案

## Phase 0：冻结当前证据

### 工作

保存当前 `LT-DWA-v2-inspired`：

```text
源代码 commit
launch 参数
失败 bag
S 曲线路径 JSON
分析文档
RViz 截图
```

### 目的

后续可以清楚区分：

```text
LT-DWA-inspired
vs
Official LT-DWA port
```

### Gate

```text
所有证据有路径、时间、git SHA、config hash；
当前适配器不再无记录地继续修改。
```

---

## Phase 1：原样复现官方仓库

作者 README 声明的环境包括 [2]：

```text
Ubuntu 20.04
Python 3.9
ROS Noetic
ros-navigation
move_base
pcl-ros
OpenCV
g2o
SuiteSparse
yaml-cpp
python-rvo2
```

官方构建命令为：

```bash
catkin_make -DCMAKE_BUILD_TYPE=Release
```

### 第一阶段只做

```text
1. 单独 catkin workspace；
2. 固定官方 commit；
3. 原样编译；
4. 原样运行 ORCA demo；
5. 原样运行 static demo；
6. 保存官方 demo bag / log / CPU 占用。
```

### 暂时禁止

```text
改 Scout topic
改路径算法
删 EB-MPC
改 cost
改采样树
```

### Gate A

```text
官方 demo 可重复启动；
LT-DWA seed 和 EB-MPC optimized trajectory 可视化正常；
输出无 NaN；
运行频率和耗时可记录。
```

---

## Phase 2：官方代码接口审计

官方仓库虽然是 ROS Noetic 工程，但公开 `local_planner_node.cpp` 主要面向作者自己的测试环境，包含：

```text
getchar() 启动
planOrcaOnce()
planCrowdOnce()
planStaticOnce()
固定次数测试循环
```

因此它不是直接可用的 `nav_core::BaseLocalPlanner` 插件。

### 需要审计

```text
1. 机器人状态从哪里进入；
2. navigation path 的数据结构；
3. static occupancy map 的接口；
4. dynamic agents 的接口；
5. LT-DWA seed 输出结构；
6. EB-MPC 输入/输出结构；
7. 最终控制命令如何生成；
8. 所有时间步、horizon 和频率定义；
9. frame 约定；
10. 是否假设 holonomic / differential drive。
```

### 交付物

```text
docs/lt_dwa_official_interface_audit.md
```

其中必须画出：

```text
输入 → reference path → LT-DWA tree → EB-MPC → command
```

### Gate B

在不修改核心算法前，能够用一个离线单元测试调用：

```text
current robot state
+ synthetic reference path
+ empty/static environment
→ official optimized state sequence
```

---

## Phase 3：建立官方核心 library 接口

由于原主程序是测试驱动循环，建议最小改造为 library API，而不是直接重写算法。

推荐接口：

```cpp
struct LtDwaInput {
    RobotState robot;
    ReferencePath reference;
    StaticDistanceField static_field;
    DynamicObstaclePrediction dynamic_obstacles;
    double stamp;
};

struct LtDwaOutput {
    bool success;
    std::vector<RobotState> seed_trajectory;
    std::vector<RobotState> optimized_trajectory;
    double cmd_v;
    double cmd_w;
    Diagnostics diagnostics;
};

class OfficialLtDwaPlanner {
public:
    bool initialize(const OfficialLtDwaConfig& config);
    LtDwaOutput plan(const LtDwaInput& input);
    void reset();
};
```

### 修改边界

允许修改：

```text
main/test loop
数据输入接口
结果输出接口
诊断接口
命名空间
```

暂时不允许修改：

```text
state-cost tree 数学逻辑
voxel sampling
EB-MPC edge/cost
g2o 优化结构
reference path 公式
```

### Gate C

使用官方 demo 输入时：

```text
library 版本和原程序的 seed / optimized trajectory 基本一致。
```

---

## Phase 4：Scout ROS Bridge

新增：

```text
src/scout_apps/control/lt_dwa_official_bridge/
```

## 4.1 输入接口

建议订阅：

```text
/scout/global_path_fixed
/odom
/tf
/tf_static
/map 或统一 static costmap
可选动态障碍 topic
/scout/goal
```

## 4.2 路径适配

必须保留官方“reference navigation path”逻辑，不应重新退化成单一 lookahead target。

桥接层只负责：

```text
frame 统一到 map
路径去重和弧长计算
必要的等弧长重采样
起点 bounded projection
将完整 navigation path 交给官方参考生成模块
```

禁止：

```text
桥接层预先生成一个 local target，替代官方 reference path。
```

## 4.3 状态适配

从 `/odom` 和 TF 提供：

```text
x, y, theta
v, omega
```

加速度可由：

```text
官方状态递推
或
统一 command history
```

得到，但必须和官方动力学语义一致。

## 4.4 环境适配

你的第一批 fixed-path benchmark 可先采用：

```text
static / empty obstacle environment
```

但不能简单传全零且不记录。必须明确：

```text
obstacle mode = disabled-for-fixed-path-benchmark
```

之后再接：

```text
static occupancy grid → distance field
```

动态人群不是当前液体运输实验的必要条件，可以留到后续。

## 4.5 输出适配

输出统一为：

```text
/benchmark/cmd_vel_raw
/baseline/lt_dwa_official/local_trajectory
/baseline/lt_dwa_official/seed_trajectory
/baseline/lt_dwa_official/diagnostics
/baseline/lt_dwa_official/status
```

所有 `/cmd_vel` 必须通过现有 common gate。

### Gate D

```text
直线 fresh-sim N=5：
100% 到点；
无 TRACKING_DIVERGED；
cmd 与 odom 符号/尺度一致；
seed 与 optimized trajectory 均可视化。
```

---

# 5. 固定路径跟踪验证顺序

不要直接从完整 S 曲线开始。

## Test 1：直线

目的：

```text
验证 frame、速度、终点和 command pipeline。
```

## Test 2：单左弯

目的：

```text
验证正角速度和 reference orientation。
```

## Test 3：单右弯

目的：

```text
排除符号、坐标系和角度 wrap 问题。
```

## Test 4：低曲率 S 弯

目的：

```text
验证 reference path 换向和 EB-MPC 连续性。
```

## Test 5：正式高曲率 S 弯

目的：

```text
验证和 DWA / TEB / SPMPC 的共同 benchmark。
```

每一级通过后才能进入下一级。

---

# 6. 必须增加的诊断

## 6.1 Seed 与优化轨迹对比

每个周期记录：

```text
LT-DWA seed RMS cross-track
EB-MPC optimized RMS cross-track
seed max lateral error
optimized max lateral error
seed / optimized heading RMS
seed / optimized control variation
```

这可以直接验证官方方法的 EB-MPC 是否真正改善 seed。

## 6.2 Reference 诊断

记录前若干 stage：

```text
reference x_i, y_i, yaw_i
reference arc length
candidate/optimized projected s_i
lateral error e_y_i
heading error e_psi_i
```

## 6.3 Tree 诊断

```text
expanded node count
valid node count
voxel-pruned node count
tree depth
best seed cost
failure layer
```

## 6.4 优化诊断

```text
g2o iterations
initial cost
final cost
optimizer status
optimization time
```

## 6.5 执行诊断

```text
actual replanning frequency
command publish frequency
command source plan age
cmd_v / cmd_w
odom_v / odom_w
deadline miss
```

---

# 7. Candidate Oracle 仍然保留，但角色改变

在官方完整方法接入后，Oracle 诊断用于回答：

```text
LT-DWA tree 是否产生了贴路径 seed？
EB-MPC 是否把 seed 优化得更好？
最终控制是否忠实执行 optimized trajectory？
```

推荐定义：

```text
best_seed_tracking_rank
selected_seed_tracking_rank
optimized_vs_seed_improvement
selected_vs_tracking_oracle_gap
```

判断：

| 结果 | 根因 |
|---|---|
| 所有 seed 都差 | reference / tree expansion / limits 问题 |
| 好 seed 未被选 | LT-DWA cost 问题 |
| seed 尚可但 EB-MPC 变差 | graph cost / parameter 问题 |
| optimized trajectory 好但执行差 | command / frequency / robot interface 问题 |

---

# 8. 参数对齐原则

## 8.1 必须共同对齐

```text
v_max
omega_max
a_max
alpha_max
robot radius
global path
start pose / goal
controller command gate
goal tolerance
slosh evaluator
```

## 8.2 不能机械复制的参数

官方默认配置中有：

```text
max_v = 1.0
max_w = 1.0
max_acc = 1.0
max_angular_acc = 1.0
time_step = 0.2
policy = seed
sampler = voxel
optimizer = eb_mpc
```

这些参数需先按官方 demo复现，再映射到 Scout common-limit。不能直接把你当前 inspired adapter 的：

```text
path_lateral
heading
progress
lookahead_distance
```

套到官方算法，因为二者 cost 结构不同。

## 8.3 规划频率公平性

必须区分：

```text
replanning frequency
command publish frequency
internal prediction dt
```

正式比较至少同时报告：

```text
mean / p95 plan time
actual replanning Hz
deadline miss rate
command plan age
```

如果官方 LT-DWA 受计算量限制只能低频运行，应如实报告，而不是把 25 Hz 重复发布当成 25 Hz 规划。

---

# 9. 官方方法的最小参数调试顺序

只在官方完整管线运行后进行：

```text
1. 机器人几何和动力学上限；
2. reference path resampling；
3. prediction dt / horizon；
4. tree expansion and voxel sampling；
5. reference tracking weights；
6. EB-MPC smooth / acceleration weights；
7. static obstacle distance-field weights；
8. dynamic crowd parameters。
```

每次只修改一个参数族，并固定：

```text
git SHA
config hash
path hash
sim seed
```

---

# 10. 正式论文 baseline 的进入 Gate

官方 LT-DWA 至少满足：

```text
1. 直线、左弯、右弯、S 弯均通过；
2. 每条路径 fresh-sim N≥5；
3. success rate = 100%；
4. 无 TRACKING_DIVERGED；
5. 无 projection jump；
6. tracking RMS 不显著差于 DWA；
7. 完成时间 / mean velocity 与 DWA 可比；
8. command gate clamp ratio ≤ 1%；
9. |dv/dt|、|domega/dt| 不明显恶化；
10. 实际规划耗时满足控制周期。
```

如果不能满足，仍应保留失败率，而不能只删除失败 run。

---

# 11. 与 DWA / TEB / SPMPC 的最终矩阵

| 方法 | 角色 | 主文建议 |
|---|---|---:|
| ROS DWA | 经典采样 local planner | 保留 |
| TEB | 经典优化 local planner | 保留 |
| official LT-DWA | 现代长视域 local planner | 通过 Gate 后加入 |
| `mpc_local_planner` | 普通 NMPC local planner | 强烈建议保留 |
| SPMPC B_ours | 本文方法 | 核心 |
| LT-DWA-inspired v2 | 自研不完整适配器 | 附录 / 不进主排名 |

在无障碍 fixed-path 场景中，官方 LT-DWA 不一定必然优于 DWA，因为其主要优势还包括静态障碍、crowd、长视域和轨迹平滑 [1]。如果完整官方实现仍在你的高曲率 fixed-path benchmark 中明显落后，应如实说明场景适配性，而不是继续无限修改直到它“看起来更好”。

---

# 12. 是否保留当前 LT-DWA-v2 修复方案

保留，但用途调整为：

```text
1. 验证 progress gating / path tube / recovery 的通用诊断；
2. 用作官方接入时的接口对照；
3. 写入 appendix，说明为何不把 inspired adapter 当正式 LT-DWA；
4. 不再作为主文现代 baseline。
```

若后续仍要修，它的顺序是：

```text
Candidate Oracle
→ progress gating
→ whole-trajectory path cost
→ curvature-seeded candidates
→ recovery mode
→ frequency test
```

但优先级低于官方代码接入。

---

# 13. 项目文件建议

```text
src/third_party/LT_DWA/
src/scout_apps/control/lt_dwa_official_bridge/
src/scout_apps/control/lt_dwa_official_benchmark/

docs/LT_DWA/
├── 00_upstream_commit_and_license.md
├── 01_official_demo_reproduction.md
├── 02_interface_audit.md
├── 03_scout_bridge_design.md
├── 04_parameter_mapping.md
├── 05_validation_gates.md
└── 06_deviation_from_upstream.md
```

任何对 upstream 的修改以 patch 记录：

```text
patches/
├── 0001-library-interface.patch
├── 0002-ros-bridge-hooks.patch
└── 0003-diagnostics-only.patch
```

---

# 14. 推荐立即执行的任务清单

```text
[ ] 固定 current LT-DWA-v2-inspired commit 和失败证据
[ ] 新建隔离 catkin workspace
[ ] clone 官方 LT_DWA 并记录 commit
[ ] 安装 g2o / SuiteSparse / ROS Noetic dependencies
[ ] 原样编译官方仓库
[ ] 原样跑 ORCA demo
[ ] 原样跑 static demo
[ ] 完成 interface audit
[ ] 将核心提取成 library API
[ ] 建立 lt_dwa_official_bridge
[ ] 先跑直线 N=5
[ ] 再跑左右单弯
[ ] 再跑低曲率 S 弯
[ ] 最后跑正式 S 弯
[ ] 通过 Gate 后进入 common-limit 对比
```

---

# 15. 风险与止损条件

## 风险 1：官方代码强绑定作者仿真

应对：

```text
只抽取 planner core；
不要把 crowd simulator 和 static demo 整体塞进 Scout launch。
```

## 风险 2：代码不是标准插件

应对：

```text
做 bridge node / library wrapper；
第一阶段不强行改成 nav_core 插件。
```

## 风险 3：官方实现计算量过高

应对：

```text
记录真实 plan time；
优先降低树宽或 horizon，但必须保留 EB-MPC；
不能用重复发布伪装规划频率。
```

## 风险 4：完整方法仍不适合 fixed-path 高精度跟踪

止损条件：

```text
在完成接口正确性、官方管线完整性和合理参数调试后，
若 S 弯 tracking RMS 仍显著差于 DWA，
则将 LT-DWA 移至附录或只用于障碍场景，
主文现代 baseline 改用 mpc_local_planner。
```

---

# 16. 最终建议

当前最合理的路线是：

> **使用官方 ROS Noetic LT-DWA 作为正式现代 local-planner baseline；保留完整 LT-DWA tree + EB-MPC，只在外围增加 Scout bridge、统一 command gate 和诊断模块。**

当前自研 `LT-DWA-v2-inspired` 的失败更像是不完整方法结构和适配评分导致，不能据此得出“LT-DWA 比 DWA 差”的结论。

但同样要保持边界：

> 官方代码接入后仍需通过 fixed-path tracking Gate；如果完整 LT-DWA 在你的场景下仍不占优，应真实报告，而不是通过大幅改算法把它变成另一种方法。

---

# 参考文献与官方资源

[1] Z. Jian, S. Zhang, L. Sun, W. Zhan, N. Zheng, and M. Tomizuka, “Long-Term Dynamic Window Approach for Kinodynamic Local Planning in Static and Crowd Environments,” *IEEE Robotics and Automation Letters*, 2023. DOI: **10.1109/LRA.2023.3266664**.
论文公开版本：https://arxiv.org/abs/2310.02648

[2] Z. Jian et al., **Official LT_DWA ROS Noetic Repository**, MIT License.
https://github.com/flztiii/LT_DWA
仓库 README 指定 Ubuntu 20.04、ROS Noetic、g2o、SuiteSparse 等依赖，并包含 `local_planner`、`local_map_generation`、`crowd_simulator`、`static_map` 等包。

[3] D. Fox, W. Burgard, and S. Thrun, “The Dynamic Window Approach to Collision Avoidance,” *IEEE Robotics & Automation Magazine*, vol. 4, no. 1, pp. 23–33, 1997. DOI: **10.1109/100.580977**.
**关联：** DWA 经典基线和 LT-DWA 的方法起点。
