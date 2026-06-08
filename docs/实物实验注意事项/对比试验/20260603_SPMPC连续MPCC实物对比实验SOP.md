# 20260603 SPMPC 连续 MPCC 实物对比实验 SOP

> 状态：2026-06-04 整理版。对象：`spmpc_local_planner` 的 `continuous_mpcc_acados` 后端，即规控一体连续 MPCC 实物主线。
> 主评价真值：**离线从 bag 推断的 RGB max(left, center, right) 液面高度**。`/spmpc/*`、observer、在线 RGB 只作调试/工程辅助。
> 仿真只用于集成联调，**抑晃效果只在实物上以离线 RGB 真值评定**。
> 运行脚本：`src/scout_apps/control/spmpc_local_planner/scripts/run_continuous_real.sh`
> 工控机迁移与 acados 安装：`docs/实物实验注意事项/代码移植/20260602_实物端代码拉取与子模块注意事项.md`
> 论文实验设计、证据链和正式方法矩阵以 `docs/实物实验注意事项/对比试验/20260605_SPMPC论文对比实验设计建议.md` 为准；本文只作为连续 MPCC 仿真/实物操作 SOP。
>
> **2026-06-08 更新（转向角加速度约束）**：连续 MPCC 模型把 `omega` 提为状态、`alpha = d(omega)/dt` 作为控制，并在 OCP 内硬约束 `|alpha| <= alpha_max`（默认 1.2 rad/s²，对齐 TEB/DWA 的 `acc_lim_theta`）。状态维度 B0 5D→**6D**、slosh 9D→**10D**；下发角速度改为 `cmd_omega = clamp(实测omega + alpha·dt)`。该约束在 OCP 内消除旧模型"直道高速甩舵"chattering，使"降晃"靠平滑预判而非高频打舵。**改动后必须重新 generate 两个求解器并 `--force-cmake` 重编**（见 §3.2），否则 C++ `static_assert` 会在编译期直接报维度不一致。
>
> **2026-06-08 追加（RouteB / direct-omega 诊断后端）**：当前代码同时保留 `continuous_mpcc_direct_omega_legacy`，其 OCP 仍为 direct-omega：`x=[px,py,theta,v,s]`、`u=[a,omega,v_s]`，anti-chatter 由出口 `cmd_omega` rate clamp 实现。仿真 RouteB B0 结果显示 `alpha_max=3.5` 可无 solve fail 到达，`3.0` 太紧、`8.0` 开始左摇右晃。若把 RouteB 升为实物候选/正式主线，必须整组固定同一 `solver_backend` 与同一 `alpha_max`，并把外部 TEB/DWA 的 `acc_lim_theta` 同步到同一口径或明确标注为非 common-limit 对比。

---

## 0. 实验设计总览

### 0.1 主实验问题

本实验验证：在同一 Scout 实物平台、同一固定路径模板、同一 RGB 标定条件下，连续 MPCC 中加入 slosh-aware 模型/代价是否能降低真实液面晃动。

规划控制一体口径仍成立：SPMPC 每周期在 horizon 内同时优化局部几何轨迹、路径进度 `s/v_s` 与第一帧控制命令，不是只对给定速度/路径做跟踪。当前差别在于角速度处理有两条 continuous 结构：alpha-state 主线在 OCP 内约束 `alpha=d(omega)/dt`；RouteB/direct-omega 后端在 OCP 外对下发 `cmd_omega` 做 rate clamp。

主线只比较同一 continuous backend 下的变体。若使用 `continuous_mpcc_acados`，主表固定 alpha-state 后端；若使用 `continuous_mpcc_direct_omega_legacy` 作为 RouteB 候选，主表也必须整组固定该后端和同一 `alpha_max`。`primitive` 后端和外部 planner 只作为附录或工程 baseline，不能混入 SPMPC 内部主表，否则会同时改变“求解器形式”和“是否 slosh-aware”。

### 0.2 论文主文实验块

主文建议收敛为五个实验块，避免把诊断性 RouteB、权重扫和工程 smoke 混进主线叙事：

| 主文实验块 | 包含内容 | 一句话目的 |
|---|---|---|
| Experiment 1: Internal Ablation | `B0 / B_smooth / B_slosh / B_ours` | 在同一 SPMPC backend、同一 `alpha_max`、同一路径和同一约束下，证明 slosh-aware 与 smooth 各自对完成率、跟踪和液面晃动的贡献。 |
| Experiment 2: External Baselines | `B_ours` vs TEB vs DWA | 在 common-limit 或明确标注的 tuned-limit 条件下，比较最终 SPMPC 与传统局部规划器的 success / duration / tracking / slosh Pareto 表现。 |
| Experiment 3: Metric Analysis | full-window vs post-start + Pareto | 同一批 bag 同时报告全程液面风险、去掉起步瞬态后的行进阶段抑晃，以及速度/完成率/跟踪/晃动之间的 trade-off。 |
| Experiment 4: Real-Robot Validation | 实物 fixed-path；最低 `B0/B_ours`，时间够再加 TEB/DWA | 在真实 Scout、真实液体和同一 RGB 标定下验证仿真趋势能否迁移到实物平台。 |
| Experiment 5: Slosh Proxy Validation | 模型 slosh 输出 vs RGB max-LCR | 验证 `/spmpc/slosh_height` 或 `/slosh/height` 作为在线模型 proxy 与离线 RGB 真值之间的相关性、滞后和误差边界。 |

RouteB alpha-state vs direct-omega、`alpha_max` 扫描、`w_slosh` 扫描、路径迁移和起点扰动更适合放在诊断/附录，除非后续决定把其中某项升为论文主实验。

### 0.3 SPMPC 内部主实验组

alpha-state 主线（`continuous_mpcc_acados`）如下：

| variant | solver backend | generated model | 状态维度 | slosh 状态/代价 | smooth | 用途 |
|---|---|---|---:|---|---|---|
| `B0` | `continuous_mpcc_acados` | `spmpc_b0` | 6D | 否 | 否 | 基础连续 MPCC baseline |
| `B_smooth` | `continuous_mpcc_acados` | `spmpc_b0` | 6D | 否 | 是 | 只看控制平滑是否降晃 |
| `B_slosh` | `continuous_mpcc_acados` | `spmpc_slosh` | 10D | 是 | 否 | 只看 slosh-aware 是否有效 |
| `B_ours` | `continuous_mpcc_acados` | `spmpc_slosh` | 10D | 是 | 是 | 我们最终方法 |

> 状态维度含 `omega`（2026-06-08 起）：B0 `[px,py,θ,v,s,omega]`，slosh 再追加 `[η_x,η̇_x,η_y,η̇_y]`；控制 `u=[a, alpha, v_s]`，`alpha=d(omega)/dt` 硬约束 `|alpha|<=1.2 rad/s²`。

RouteB / direct-omega 候选结构如下（当前 B0 仿真诊断已验证，slosh direct-omega 是否进入 formal 主线需另做同口径验证）：

| variant | solver backend | generated model | 状态维度 | slosh 状态/代价 | `alpha_max` 语义 | 用途 |
|---|---|---|---:|---|---|---|
| `B0` | `continuous_mpcc_direct_omega_legacy` | `spmpc_b0_direct_omega_legacy` | 5D | 否 | 出口 `cmd_omega` rate clamp | RouteB B0 诊断/候选 |
| `B_slosh`/`B_ours` | `continuous_mpcc_direct_omega_legacy` | `spmpc_slosh_direct_omega` | 9D | 是 | 出口 `cmd_omega` rate clamp | 需额外验证后才能进 formal 主表 |

> 若 RouteB 被选为正式主线，论文表述应写成“direct-omega continuous MPCC + output omega-rate limiting”，不能继续写成“OCP 内硬约束 `alpha` 的 alpha-state MPCC”。

核心对照关系：

```text
B_slosh vs B0        slosh 模型/代价是否有效
B_smooth vs B0       仅靠平滑控制是否有效
B_ours  vs B_smooth  slosh-aware 是否优于 smooth-only
B_ours  vs B0        最终方法总体收益
```

可选附录组：`B_slosh_linear`、`B_slosh_anti`、`B_ours_anti`、`primitive` 后端、`scout_local_planner`/`mpc_planner` 外部 baseline。

仿真侧论文主矩阵可先用统一 runner 做 smoke / 半正式统计：

```bash
OUT_ROOT=/data/a/spmpc_paper_compare/fixed_path_matrix_$(date +%Y%m%d_%H%M%S) \
PATH_FILE=/data/a/fixed_paths/sim/P2_s_curve.json \
PATH_ID=P2_s_curve \
SLOSH_MONITOR_ENABLE=true \
RUNS=1 \
RECORD_SEC=60 \
PRE_PATH_WAIT_SEC=30 \
SPMPC_SOLVER_BACKEND=continuous_mpcc_acados \
bash src/scout_apps/control/spmpc_experiments/scripts/run_fixed_path_paper_matrix.sh
```

正式数据仍建议一个方法/一组方法 fresh 启动仿真；runner 只负责统一目录、meta 和指标提取。

### 0.4 真值和辅助量边界

```text
主真值：离线 RGB max-LCR 液面高度(mm)
在线 RGB：现场调试/故障发现/实时观察，不作为最终论文指标
/spmpc/*：模型预测、代价、求解状态和诊断，不作为真实液面高度
observer：执行过程的模型状态 proxy，可解释趋势，但不替代 RGB 真值
```

---

## 1. 公平性控制

每个有效 run 必须满足：

```text
同一地面起点标记
同一目标点 GOAL_X / GOAL_Y / GOAL_YAW
同一 P2_s_curve 模板参数
同一 solver backend（SPMPC 内部主表）
同一 v_max / omega_max / a_max / alpha_max 或同一外部 planner 动态限制口径
同一 warm-start / terminal / reference preprocess 设置
同一 RealSense RGB 手动曝光/增益/白平衡
同一天同一 calibration / ROI / HSV
同一容器、液位、安装姿态
同一录包 topic 口径
组间回到同一起点并等待液体静稳
```

实物端不强求 bit-level replay 同一条 JSON 路径。每次回位和定位存在厘米级误差；正式公平性口径是“同一起点标记 + 同目标点 + 同模板生成规则 + 同参数”，同时在分析中报告跟踪误差与路径进度，用于排除明显异常 run。

当前公平性判断：

```text
SPMPC 内部消融：
  公平，前提是 B0/B_smooth/B_slosh/B_ours 全部使用同一 solver_backend、同一 alpha_max、同一 warm-start/terminal/reference 设置，
  只改变 slosh_enable / smooth 权重 / w_slosh 等预先定义的消融因素。

alpha-state vs RouteB：
  不是同一主表内的公平消融，因为同时改变了 OCP 状态/控制结构与 omega-rate 处理方式；只能作为结构诊断或另起一张 backend 消融表。

SPMPC vs TEB/DWA 外部 baseline：
  如果声称 common-limit，对外部 planner 的 max_vel_x/max_vel_theta/acc_lim_x/acc_lim_theta/goal_tolerance 必须与 SPMPC 当前实际限制对齐。
  若 SPMPC RouteB 使用 alpha_max=3.5，而 TEB/DWA 仍用 acc_lim_theta=1.2，则不能称为 common-limit，只能称为各方法调参后对比，并必须报告实际 cmd_omega_rate。
```

配置检查：`teb_local_planner_standalone_sim.yaml` 与 `dwa_local_planner_standalone_sim.yaml` 当前已按 0.8 / 1.2 / 0.6 / 0.20 对齐 alpha-state common-limit；但 `dwa_local_planner_sim.yaml` 仍是 `acc_lim_x=0.8`、`acc_lim_theta=2.0`，若用于正式 common-limit 对比必须先改或不要使用。

---

## 2. 重复次数、顺序与无效 run 规则

### 2.1 重复次数

最低可用设计：每组 `N=3` 次。时间允许时建议 `N=5`。

### 2.2 平衡顺序

不要按 `B0 B0 B0 -> B_ours B_ours B_ours` 连续跑完一组。推荐交错顺序降低电量、光照、地面和操作者因素影响：

```text
Round 1: B0 -> B_smooth -> B_slosh -> B_ours
Round 2: B_ours -> B_slosh -> B_smooth -> B0
Round 3: B_smooth -> B0 -> B_ours -> B_slosh
```

如做 `N=5`，继续交错即可；每轮内每组各一次。

### 2.3 无效 run 与重跑规则

以下情况作废并重跑，不纳入统计：

```text
起步前液体未静稳
起点明显偏离地面标记
实际 /spmpc/solver_backend 不是本 run 预设值（continuous_mpcc_acados 或 continuous_mpcc_direct_omega_legacy）
/spmpc/status 长时间 ACADOS_NOT_IMPLEMENTED / ACADOS_NOT_CREATED / ACADOS_SOLVE_FAILED_* / ACADOS_DIRECT_OMEGA_SOLVE_FAILED_*
bag 缺少 /camera/color/image_raw 或 /camera/color/camera_info
RGB ROI/HSV/calibration 明显错误，离线推断失败或全 0
机器人明显未跟上固定路径或发生外部干扰/急停/碰撞
录包损坏或关键 topic 缺失
```

---

## 3. 实物机一次性准备

### 3.1 装 acados（只做一次）

按 `docs/实物实验注意事项/代码移植/20260602_实物端代码拉取与子模块注意事项.md` 的 acados 章节完整执行。装完确认：

```bash
echo "$ACADOS_SOURCE_DIR"          # /home/geist/acados
ls ~/acados/lib/libacados.so || ls ~/acados/lib64/libacados.so
```

### 3.2 生成求解器 + 构建（代码更新后重做）

```bash
source /opt/ros/noetic/setup.bash && source ~/.bashrc
cd /home/geist/scout_ws

source ~/acados_venv/bin/activate
cd src/scout_apps/control/spmpc_local_planner/scripts/acados
python generate_spmpc_acados.py --model b0
python generate_spmpc_acados.py --model slosh
# RouteB / direct-omega 后端需要以下 generated solver；若不跑 RouteB 可跳过。
python generate_spmpc_acados.py --model b0_direct_omega_legacy
python generate_spmpc_acados.py --model slosh_direct_omega
deactivate

cd /home/geist/scout_ws
catkin_make --pkg spmpc_local_planner --force-cmake
source devel/setup.bash
```

构建日志应打印 `building continuous_mpcc_acados backend`。如果运行时 `/spmpc/status=ACADOS_NOT_IMPLEMENTED`，说明节点仍是 stub 构建，需要检查 `ACADOS_SOURCE_DIR`、generated solver 和 `--force-cmake`。

> 2026-06-08 起 alpha-state 模型含 omega-rate 约束：重生成后确认维度宏 `SPMPC_B0_NX=6`、`SPMPC_SLOSH_NX=10`（`grep -h _NX generated/acados/*/acados_solver_*.h`）。RouteB direct-omega 诊断模型则应为 `SPMPC_B0_DIRECT_OMEGA_LEGACY_NX=5`、`SPMPC_SLOSH_DIRECT_OMEGA_NX=9`。若维度不符说明链接到旧 generated solver；C++ `static_assert` 也会在编译期直接因 `NP` 不一致报错。

---

## 4. 现场启动总顺序

### 4.1 终端 A：传感器/定位栈

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws
src/scout_apps/control/scout_local_planner/scripts/launch_real_sensors_stack.sh
```

确认：`/odom`、`/camera/color/image_raw`、`/camera/color/camera_info`、`/imu/data` 有数据。

### 4.2 终端 B：冻结 RGB 参数（当天全组同一份）

```bash
OUT_DIR=/home/geist/slosh_bags/real/<DATE>_visual_tuning/realsense_rgb_fixed_params \
src/scout_apps/control/scout_local_planner/scripts/set_realsense_rgb_manual_params.sh
```

组间不要重开自动曝光，不要改白平衡/增益。

### 4.3 终端 C：跑一组 continuous

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
source ~/.bashrc        # ACADOS_SOURCE_DIR / LD_LIBRARY_PATH

DATE=<DATE> \
GOAL_X=<x> GOAL_Y=<y> GOAL_YAW=<yaw> \
VARIANT=B0 \
SOLVER_BACKEND=continuous_mpcc_acados \
bash src/scout_apps/control/spmpc_local_planner/scripts/run_continuous_real.sh
```

每组前把车摆回同一地面标记；当天所有组使用同一 goal 和同一模板参数。

> 注意：当前 `run_continuous_real.sh` 已支持 `SOLVER_BACKEND` 环境变量，但尚未把 `alpha_max` 运行时覆盖透传到 `spmpc_fixed_path.launch`。若 RouteB 实物要使用仿真诊断得到的 `alpha_max=3.5`，正式跑前需先补脚本参数透传（或手动 `roslaunch spmpc_fixed_path.launch solver_backend:=continuous_mpcc_direct_omega_legacy alpha_max:=3.5 ...`），否则会回到 `scout_mini.yaml` 默认 `alpha_max=1.2`。

### 4.4 终端 D：在线 RGB 调试监控（可选但推荐）

在线 RGB 的定位：**调试和现场观察**。它用于确认相机、ROI、HSV、液面识别是否正常，帮助现场及时发现失败 run；最终论文/报告指标仍以离线 RGB 从 bag 推断为准。

在线识别与 UI 均在独立包 `realsense_liquid_measurement` 内；`spmpc_local_planner` 不依赖该包，关掉在线 RGB/UI 不应影响控制器。

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash

roslaunch realsense_liquid_measurement online_liquid_height.launch \
  calibration:=<当天 calibration.yaml> \
  image_topic:=/camera/color/image_raw \
  publish_debug:=true
```

发布：`/liquid/height`、`/liquid/height_lcr`、`/liquid/height_median`、`/liquid/debug_image`。这些话题名可通过 launch arg 覆盖；默认只作为现场 proxy。

一屏调试前端：

```bash
roslaunch realsense_liquid_measurement liquid_monitor.launch \
  refresh_hz:=5.0 \
  export_dir:=/home/geist/slosh_bags/real/<DATE>_liquid_monitor_exports
```

当前 UI 是英文深色 dashboard，避免实物机 matplotlib 中文字体缺失：

```text
RGB Liquid Online Monitor
  左上：RGB Overlay / Online Liquid Detection，显示 /liquid/debug_image 与 LIVE 状态
  右上：Height Trend (last Ns)，显示 max / L / C / R / model 曲线
  左下：Key Metrics，显示 RGB MAX、LEFT、CENTER、RIGHT、MODEL、SOLVER、CMD V、ODOM V、H PEAK、AX EST、MODE
  右下：Topic Health，显示 overlay、liquid、spmpc、odom/cmd 的 OK/WAIT/LOST 与消息 age
  底部：START / STOP / ZERO / EXPORT，只做本地会话记录、归零和 CSV+SVG 导出
```

按钮语义：

```text
START   清空 UI 会话并开始记录该段
STOP    冻结 UI 会话，不影响 planner
ZERO    用当前液面作为显示基线
EXPORT  导出该 UI 会话的 CSV + SVG 到 export_dir
q/Esc   只关闭 UI 前端
```

注意：在线值与离线脚本共享检测思路，但在线结果不做最终统计，因为离线流程能统一帧采样、平滑、质量检查和异常剔除。`run_continuous_real.sh` 默认不录 `/liquid/*`；如确实需要把在线 proxy 同步进 bag，显式设置 `RECORD_ONLINE_LIQUID=true`。

---

## 5. 对比组与 w_slosh 实物扫

### 5.1 四组主消融

alpha-state 主线：

```bash
DATE=<D> GOAL_X=<x> GOAL_Y=<y> GOAL_YAW=<yaw> SOLVER_BACKEND=continuous_mpcc_acados VARIANT=B0       bash src/scout_apps/control/spmpc_local_planner/scripts/run_continuous_real.sh
DATE=<D> GOAL_X=<x> GOAL_Y=<y> GOAL_YAW=<yaw> SOLVER_BACKEND=continuous_mpcc_acados VARIANT=B_smooth bash src/scout_apps/control/spmpc_local_planner/scripts/run_continuous_real.sh
DATE=<D> GOAL_X=<x> GOAL_Y=<y> GOAL_YAW=<yaw> SOLVER_BACKEND=continuous_mpcc_acados VARIANT=B_slosh  bash src/scout_apps/control/spmpc_local_planner/scripts/run_continuous_real.sh
DATE=<D> GOAL_X=<x> GOAL_Y=<y> GOAL_YAW=<yaw> SOLVER_BACKEND=continuous_mpcc_acados VARIANT=B_ours   bash src/scout_apps/control/spmpc_local_planner/scripts/run_continuous_real.sh
```

RouteB 若升为候选主线，也必须四组全用同一 backend 和同一 `alpha_max`；不要只给 B0 或 B_ours 单独放宽。当前脚本需先补 `alpha_max` 透传后再写正式命令。

### 5.2 w_slosh 实物扫

先用 `B_slosh` 扫定工作点，再跑四组主实验：

```bash
DATE=<D> GOAL_X=<x> GOAL_Y=<y> GOAL_YAW=<yaw> SOLVER_BACKEND=continuous_mpcc_acados VARIANT=B_slosh W_SLOSH=1 BAG_NAME=B_slosh_w1 bash src/scout_apps/control/spmpc_local_planner/scripts/run_continuous_real.sh
DATE=<D> GOAL_X=<x> GOAL_Y=<y> GOAL_YAW=<yaw> SOLVER_BACKEND=continuous_mpcc_acados VARIANT=B_slosh W_SLOSH=2 BAG_NAME=B_slosh_w2 bash src/scout_apps/control/spmpc_local_planner/scripts/run_continuous_real.sh
DATE=<D> GOAL_X=<x> GOAL_Y=<y> GOAL_YAW=<yaw> SOLVER_BACKEND=continuous_mpcc_acados VARIANT=B_slosh W_SLOSH=3 BAG_NAME=B_slosh_w3 bash src/scout_apps/control/spmpc_local_planner/scripts/run_continuous_real.sh
DATE=<D> GOAL_X=<x> GOAL_Y=<y> GOAL_YAW=<yaw> SOLVER_BACKEND=continuous_mpcc_acados VARIANT=B_slosh W_SLOSH=5 BAG_NAME=B_slosh_w5 bash src/scout_apps/control/spmpc_local_planner/scripts/run_continuous_real.sh
```

辅助判据：

```text
slosh cost 占比 < 10%     压晃项偏弱，可能需要增大 w_slosh
slosh cost 占比 20%~40%   通常较健康，仍需看 RGB peak/p95
slosh cost 占比 > 50%     可能牺牲跟踪或速度，需检查 contour/progress/cmd_omega
```

主依据仍是离线 RGB peak/p95/RMS/AUC，不是 cost 占比。

---

## 6. RGB 真值离线推断

每个 bag 跑一次：

```bash
python3 src/scout_apps/sensors/realsense_liquid_measurement/scripts/red_liquid_infer_from_bag.py \
  --bag <BAG_DIR>/<VARIANT>.bag \
  --topic /camera/color/image_raw \
  --calibration <当天 calibration.json> \
  --out-dir <BAG_DIR>/<VARIANT>_rgb
```

当天所有组必须使用同一 calibration、ROI、HSV。主真值定义：

```text
h_rgb(t) = max(h_left(t), h_center(t), h_right(t))
```

---

## 7. 分析口径与报表模板

### 7.1 单 run 输出

| 字段 | 单位 | 来源 | 说明 |
|---|---:|---|---|
| `peak_mm` | mm | 离线 RGB | 主指标，最大液面高度 |
| `p95_mm` | mm | 离线 RGB | 抗单帧噪声 |
| `rms_mm` | mm | 离线 RGB | 整体晃动强度 |
| `auc_mm_s` | mm*s | 离线 RGB | 晃动暴露总量 |
| `mean_cmd_v` | m/s | `/cmd_vel` | 效率/保守性辅助 |
| `max_abs_cmd_omega` | rad/s | `/cmd_vel` | 是否控制过抖 |
| `cmd_omega_rate_p95/max` | rad/s² | `/cmd_vel` 差分 | RouteB/TEB/DWA 公平性辅助，检查实际转向角加速度 |
| `mean_solver_ms` | ms | `/spmpc/solver_time_ms` | 实时性 |
| `tracking_error` | m | `/spmpc/local_trajectory`/odom | 排除明显跑偏 |

### 7.2 每组汇总

建议报告：

```text
mean ± std
median [IQR]
N valid / N attempted
```

如果有效样本少于 3，不建议做强结论，只作为 pilot。

### 7.3 模型辅助量

```text
/spmpc/slosh_height                 当前模型高度 proxy，单位 mm
/spmpc/slosh_horizon_summary         预测 h_peak/h_p95，主要对 slosh 变体有意义（alpha-state 10D / direct 9D）
/spmpc/debug/slosh_state             模态 η，不是高度
/spmpc/cost_breakdown                解释权重作用，不是真值
/spmpc/solver_time_ms                实时性
```

论文结论必须写 RGB 离线真值，不写成“模型预测高度降低”。

---

## 8. 常见错误

```text
/spmpc/status = ACADOS_NOT_IMPLEMENTED
  节点是无 acados 的 stub 构建。检查 ACADOS_SOURCE_DIR、generated solver 和 --force-cmake。

/spmpc/status = ACADOS_NOT_CREATED
  generated solver 已链接但运行时 capsule 创建失败。检查 .so、LD_LIBRARY_PATH、codegen 版本。

节点启动即崩或找不到 libacados/hpipm/blasfeo
  当前终端没有 source ~/.bashrc，或 LD_LIBRARY_PATH 没指向 ~/acados/lib 或 ~/acados/lib64。

/spmpc/status = ACADOS_SOLVE_FAILED_* 或 ACADOS_DIRECT_OMEGA_SOLVE_FAILED_*
  初始 infeasible、参考拟合异常或约束过紧。检查车是否在路径起点附近、goal 是否过近；RouteB 还要检查 alpha_max 是否过紧或脚本是否未透传 alpha_max 覆盖。

RGB 推断无液面 / 全 0
  ROI/HSV/calibration 或曝光参数错误。该 run 不进入统计，修正后重跑。

四组结果不可比
  goal/calibration/ROI/起摆位/RGB 参数不一致。废弃重做。
```
