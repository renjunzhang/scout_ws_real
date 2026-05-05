# 低激励几何参考生成 + MPC 跟踪方案

> 当前文档记录 2026-05-05 后的新主线。旧 `README.md` 仍描述完整的增广 slosh MPC / risk scheduler 框架，但该框架不是当前实验中最好的成功来源。

## 1. 与旧方案的框架对比

| 项目 | 旧 README 风险自适应 MPC | 当前新方案 |
|---|---|---|
| 防晃位置 | MPC 内部代价、约束、risk scheduler、output/reference shaping | MPC 前端的几何参考生成 |
| MPC 角色 | 同时尝试 tracking 与 anti-slosh | 只作为 constrained tracker |
| 成功机制 | 期望 MPC 通过 `Q_slosh / Q_eta_dot / eta_bar / v_ref_eff` 主动压晃 | 给 MPC 一条本身低激励、低曲率变化的路径 |
| `Q_slosh` | 可启用 | 当前成功配置中关闭 |
| `risk_scheduler` | 可启用 | 当前成功配置中关闭 |
| `PROFILE_REF_V2` | 速度剖面/参考修正候选 | 在 geo56 上变差，不作为最终方法 |
| 输出层 guard / PMG | 历史消融入口 | 已否定，不作为主线 |
| 论文定位 | 风险自适应 anti-slosh MPC | low-excitation geometric reference generation for MPC tracking |

一句话：

```text
旧方案把防晃放进 MPC 或 cmd_vel 后处理；
新方案把防晃前移到几何参考生成层，MPC 只负责跟踪。
```

## 2. 当前最好结果

当前最好的闭环配置是：

```text
PATH_ID=P3_geo56
CONDITION=CUSTOM
PATH_FILE=docs/Claude/分析数据/generated_paths/P3_mixed_geo_i56_g55_d18.json
MPC_CMD_VEL_LEAD_TIME=0.05
Q_slosh=0
Q_slosh_eta_dot=0
risk_scheduler=false
input_shaping=false
energy_profile=false
```

两包 `P3_geo56 CUSTOM` 相对旧 `P3_mixed_radius CUSTOM` 均值：

```text
tracking_time -17.1%
h_rms         -8.5%
h_p95         -28.1%
h_max         -7.7%
eta_dot       -8.2%
energy        -8.4%
odom_ax       -29.6%
odom_jerk     -15.9%
solve_success 1.0
```

仍未完全通过的指标：

```text
ay_p95    +12.0%
track_p95 +20.8%
```

解释：

```text
该结果证明低激励几何参考能显著降低模型估计液面晃动；
但当前 geo56 候选仍存在横向加速度和跟踪误差 trade-off。
```

## 3. 为什么新方案有效

旧 P3 几何：

```text
P3_mixed_radius
kappa_p95  = 1.271
kappa_max  = 1.451
dkappa_p95 = 3.084
```

新 P3 几何：

```text
P3_mixed_geo_i56_g55_d18
kappa_p95  = 0.824
kappa_max  = 0.916
dkappa_p95 = 1.804
```

液体晃动由车辆激励触发：

```text
纵向: ax
横向: ay ≈ v * omega
转向变化: alpha ≈ ax * kappa + v^2 * dkappa
```

因此降低 `kappa` 和 `dkappa` 会直接降低横向/角向激励风险，也让 MPC 更容易执行平滑轨迹。

## 4. 当前系统结构

```text
目标/原始路径
        ↓
离线低激励几何候选生成
        ↓
离线 rollout / geometry / time gate 筛选
        ↓
固定 PATH_FILE 回放
        ↓
PathHandler 生成局部 reference path
        ↓
MPC 跟踪 reference
        ↓
cmd_vel
        ↓
底盘 / Gazebo
```

MPC 本体仍是原 tracking MPC：

```text
状态: e_l, e_c, e_theta, v, eta_x, eta_x_dot, eta_y, eta_y_dot
控制: a, omega
约束: v, a, omega, da, domega
求解器: OSQP
```

但当前成功配置不依赖 slosh cost：

```text
Q_slosh = 0
Q_slosh_eta_dot = 0
terminal slosh cost = 0
risk_scheduler = false
PROFILE_REF_V2 = false
```

## 5. 候选生成与筛选

当前新增/使用的离线脚本：

```text
scripts/sweep_p3_geometry_candidates.py
scripts/optimize_anti_slosh_reference.py
scripts/extract_slosh_metrics.py
scripts/diagnose_reference_execution_chain.py
```

筛选流程：

```text
1. 从原始 P3_mixed.json 生成多组平滑几何候选。
2. 计算 length / kappa / dkappa / drift。
3. 对候选路径做 signed slosh rollout 和 time/ay gate。
4. 只把离线 pass_all 的候选放入 ROS 闭环仿真。
```

当前 `geo56` 的离线来源：

```text
candidate = geo_i56_g55_d18
iters = 56
gain = 0.55
max_drift ≈ 0.18m
actual_drift = 0.161m
```

## 6. 实验命令

当前最好配置：

```bash
PATH_ID=P3_geo56 CONDITION=CUSTOM RUN_ID=geo56_customXX START_DELAY=30 APPROACH_START_ENABLE=false \
PATH_FILE=/home/a/scout_ws/docs/Claude/分析数据/generated_paths/P3_mixed_geo_i56_g55_d18.json \
PATH_PUBLISH_ONCE_KEEPALIVE=false PATH_WARMUP_S=3 \
VEHICLE_V_MAX=3.0 \
MPC_CMD_VEL_LEAD_TIME=0.05 \
rosrun scout_local_planner run_sim_fixed_path_bag.sh
```

对照配置，旧 radius baseline：

```bash
PATH_ID=P3_mixed_radius CONDITION=CUSTOM RUN_ID=baselineXX START_DELAY=30 APPROACH_START_ENABLE=false \
PATH_FILE=/data/a/fixed_paths/candidates/P3_mixed_radius.json \
PATH_PUBLISH_ONCE_KEEPALIVE=false PATH_WARMUP_S=3 \
VEHICLE_V_MAX=3.0 \
MPC_CMD_VEL_LEAD_TIME=0.05 \
rosrun scout_local_planner run_sim_fixed_path_bag.sh
```

## 7. 论文表述建议

推荐主线：

```text
Low-excitation geometric reference generation for slosh-aware MPC tracking.
```

中文：

```text
面向液体搬运移动机器人的低激励几何参考生成与 MPC 跟踪。
```

不要写：

```text
风险自适应 MPC 成功主动抑制液体晃动。
MPC 内 slosh cost 是主要贡献。
PROFILE_REF_V2 是最终方法。
```

可以写：

```text
实验表明，将防晃逻辑前移到几何参考生成层，比在 MPC 代价函数中加入晃动软惩罚或在输出层裁剪命令更有效。
```

## 8. 当前边界

已成立：

```text
geo56 CUSTOM 能稳定降低 /slosh/height p95、eta_dot、energy。
该降低不是靠大幅减速，因为 tracking_time 反而缩短。
```

更新后的 open fixed-path 主实验中，`P3_geo56 CUSTOM x3` 相对 `P3_mixed_radius CUSTOM x2` 均值：

```text
tracking_time -10.7%
h_rms         -11.4%
h_p95         -18.2%
h_max         -11.8%
eta_dot       -9.8%
energy        -11.2%
ay_p95        -4.2%
track_p95     -0.6%
odom_ax       -22.3%
odom_jerk     -14.7%
solve_success 1.0
```

因此当前论文可以写成：

```text
低激励几何参考在相同 MPC tracker 下同时降低模型估计晃动、车辆激励和任务时间。
```

当前边界：

```text
geo56 仍是离线 PATH_FILE 结果；
尚未接入 scout_global_planner 的任意在线全局路径。
```

## 9. 在线 path post-processor 工程方案

当前 `geo56` 是离线生成的固定路径：

```text
P3_mixed.json
    ↓
sweep_p3_geometry_candidates.py
    ↓
P3_mixed_geo_i56_g55_d18.json
    ↓
fixed_global_path_runner.py 回放
```

要支持任意 `scout_global_planner` 生成的路径，需要把离线步骤升级为在线路径后处理器：

```text
scout_global_planner / mbf_global_sim
        ↓
/scout/global_path_raw
        ↓
anti_slosh_path_post_processor
        ↓
/scout/global_path_anti_slosh
        ↓
scout_local_planner MPC tracker
```

### 9.1 ROS 接口

建议新增节点：

```text
scripts/anti_slosh_path_post_processor.py
```

订阅：

```text
/scout/global_path_raw    nav_msgs/Path
```

发布：

```text
/scout/global_path_anti_slosh           nav_msgs/Path
/anti_slosh_path/debug/original         nav_msgs/Path
/anti_slosh_path/debug/mild             nav_msgs/Path
/anti_slosh_path/debug/medium           nav_msgs/Path
/anti_slosh_path/debug/strong           nav_msgs/Path
/anti_slosh_path/metrics                std_msgs/Float32MultiArray 或 DiagnosticArray
```

`scout_local_planner` 启动时改为订阅：

```text
global_path_topic:=/scout/global_path_anti_slosh
```

全局规划器输出 remap 为：

```text
/scout/global_path -> /scout/global_path_raw
```

### 9.2 最小在线算法

第一版不要做复杂全局优化，只做有限候选生成与打分：

```text
输入 raw path
    ↓
sanitize: 去重复点 / 极短段
    ↓
resample: 固定 ds，例如 0.05~0.10 m
    ↓
生成候选:
  C0 original
  C1 mild smoothing
  C2 medium smoothing
  C3 strong smoothing
    ↓
对每个候选计算:
  length_ratio
  max_drift
  endpoint_error
  kappa_p95 / kappa_max
  dkappa_p95 / dkappa_max
  ay_proxy = v_nom^2 * kappa_p95
  alpha_proxy = v_nom^2 * dkappa_p95
    ↓
过滤不可行候选
    ↓
选择最低 score
    ↓
发布 anti_slosh path
```

建议 score：

```text
score =
  w_k  * normalized(kappa_p95)
+ w_dk * normalized(dkappa_p95)
+ w_l  * max(0, length_ratio - 1)
+ w_x  * max_drift
```

第一版权重可固定，不要做在线学习：

```text
w_k=1.0
w_dk=0.5
w_l=0.3
w_x=0.5
```

### 9.3 安全约束

在线任意路径不能无约束平滑，否则可能穿墙或偏离可行走廊。

第一版必须有以下硬门槛：

```text
endpoint_error <= 0.05 m
max_drift <= 0.15~0.20 m
length_ratio <= 1.15
min_segment_length >= 0.02 m
candidate must preserve path direction
```

如果运行在 maze 或有障碍环境，还必须增加 collision check：

```text
查询 costmap
机器人 footprint 沿 candidate 采样
任一点 collision -> reject candidate
```

没有 collision check 时，只能声明适用于：

```text
open field
known safe corridor
fixed-path evaluation
```

不能宣称对任意复杂地图安全。

### 9.4 节点参数

建议参数：

```yaml
anti_slosh_path_post_processor:
  input_topic: /scout/global_path_raw
  output_topic: /scout/global_path_anti_slosh
  ds: 0.10
  publish_debug: true

  candidates:
    mild:
      iters: 18
      gain: 0.35
      max_drift: 0.08
    medium:
      iters: 40
      gain: 0.45
      max_drift: 0.12
    strong:
      iters: 56
      gain: 0.55
      max_drift: 0.18

  gates:
    max_drift: 0.18
    max_length_ratio: 1.15
    max_endpoint_error: 0.05
    enable_collision_check: false

  score:
    w_kappa: 1.0
    w_dkappa: 0.5
    w_length: 0.3
    w_drift: 0.5
```

### 9.5 与 `mbf_global_sim.launch` 的集成

目标不是让 MPC 直接吃原始全局路径：

```text
mbf_global_sim -> /scout/global_path -> scout_local_planner
```

而是：

```text
mbf_global_sim -> /scout/global_path_raw
anti_slosh_path_post_processor -> /scout/global_path_anti_slosh
scout_local_planner subscribes /scout/global_path_anti_slosh
```

启动示例：

```bash
roslaunch scout_global_planner mbf_global_sim.launch \
  global_path_topic:=/scout/global_path_raw

rosrun scout_local_planner anti_slosh_path_post_processor.py \
  _input_topic:=/scout/global_path_raw \
  _output_topic:=/scout/global_path_anti_slosh \
  _publish_debug:=true

roslaunch scout_local_planner slosh_experiment_sim.launch \
  global_path_topic:=/scout/global_path_anti_slosh \
  Q_slosh:=0 \
  Q_slosh_eta_dot:=0 \
  risk_scheduler_enable:=false
```

具体 launch 参数名需要以 `mbf_global_sim.launch` 实际接口为准。

### 9.6 在线版本验收标准

先在 open 场地验证：

```text
same start / same goal
raw global path + MPC
anti_slosh processed path + same MPC
```

通过标准：

```text
h_rms / h_p95 / h_max 下降
eta_dot / energy 下降
tracking_time <= raw +15%
solve_success_ratio >= 0.97
processed path max_drift <= configured gate
```

再进入 maze：

```text
必须启用 collision check；
否则 maze 只能作为定性展示，不作为安全性结论。
```

## 10. 下一步

若目标是尽快完成论文：

```text
1. 冻结 geo56 CUSTOM 作为 fixed-path positive result。
2. 实现在线 anti_slosh_path_post_processor，证明方法可接入任意 global path。
3. 将 PROFILE_REF_V2、PMG、slosh-cost 作为消融或失败分析。
```

若目标是继续冲全指标通过：

```text
1. 生成更保守候选，例如 geo_i40_g55_d12。
2. 优先降低 ay_p95 和 track_p95，而不是继续压速度。
3. 不再回到 MPC slosh cost 或输出 guard 路线。
```
