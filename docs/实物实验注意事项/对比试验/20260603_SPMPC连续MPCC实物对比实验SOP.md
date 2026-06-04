# 20260603 SPMPC 连续 MPCC 实物对比实验 SOP

> 状态：2026-06-04 整理版。对象：`spmpc_local_planner` 的 `continuous_mpcc_acados` 后端，即规控一体连续 MPCC 实物主线。
> 主评价真值：**离线从 bag 推断的 RGB max(left, center, right) 液面高度**。`/spmpc/*`、observer、在线 RGB 只作调试/工程辅助。
> 仿真只用于集成联调，**抑晃效果只在实物上以离线 RGB 真值评定**。
> 运行脚本：`src/scout_apps/control/spmpc_local_planner/scripts/run_continuous_real.sh`
> 工控机迁移与 acados 安装：`docs/实物实验注意事项/代码移植/20260602_实物端代码拉取与子模块注意事项.md`

---

## 0. 实验设计总览

### 0.1 主实验问题

本实验验证：在同一 Scout 实物平台、同一固定路径模板、同一 RGB 标定条件下，连续 MPCC 中加入 slosh-aware 模型/代价是否能降低真实液面晃动。

主线只比较 `continuous_mpcc_acados` 后端下的变体；`primitive` 后端和外部 planner 只作为附录或工程 baseline，不能混入主表，否则会同时改变“求解器形式”和“是否 slosh-aware”。

### 0.2 主实验组

| variant | solver backend | generated model | 状态维度 | slosh 状态/代价 | smooth | 用途 |
|---|---|---|---:|---|---|---|
| `B0` | `continuous_mpcc_acados` | `spmpc_b0` | 5D | 否 | 否 | 基础连续 MPCC baseline |
| `B_smooth` | `continuous_mpcc_acados` | `spmpc_b0` | 5D | 否 | 是 | 只看控制平滑是否降晃 |
| `B_slosh` | `continuous_mpcc_acados` | `spmpc_slosh` | 9D | 是 | 否 | 只看 slosh-aware 是否有效 |
| `B_ours` | `continuous_mpcc_acados` | `spmpc_slosh` | 9D | 是 | 是 | 我们最终方法 |

核心对照关系：

```text
B_slosh vs B0        slosh 模型/代价是否有效
B_smooth vs B0       仅靠平滑控制是否有效
B_ours  vs B_smooth  slosh-aware 是否优于 smooth-only
B_ours  vs B0        最终方法总体收益
```

可选附录组：`B_slosh_linear`、`B_slosh_anti`、`B_ours_anti`、`primitive` 后端、`scout_local_planner`/`mpc_planner` 外部 baseline。

### 0.3 真值和辅助量边界

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
同一 RealSense RGB 手动曝光/增益/白平衡
同一天同一 calibration / ROI / HSV
同一容器、液位、安装姿态
同一录包 topic 口径
组间回到同一起点并等待液体静稳
```

实物端不强求 bit-level replay 同一条 JSON 路径。每次回位和定位存在厘米级误差；正式公平性口径是“同一起点标记 + 同目标点 + 同模板生成规则 + 同参数”，同时在分析中报告跟踪误差与路径进度，用于排除明显异常 run。

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
实际 /spmpc/solver_backend 不是预期 continuous_mpcc_acados
/spmpc/status 长时间 ACADOS_NOT_IMPLEMENTED / ACADOS_NOT_CREATED / ACADOS_SOLVE_FAILED_*
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
deactivate

cd /home/geist/scout_ws
catkin_make --pkg spmpc_local_planner --force-cmake
source devel/setup.bash
```

构建日志应打印 `building continuous_mpcc_acados backend`。如果运行时 `/spmpc/status=ACADOS_NOT_IMPLEMENTED`，说明节点仍是 stub 构建，需要检查 `ACADOS_SOURCE_DIR`、generated solver 和 `--force-cmake`。

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
bash src/scout_apps/control/spmpc_local_planner/scripts/run_continuous_real.sh
```

每组前把车摆回同一地面标记；当天所有组使用同一 goal 和同一模板参数。

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

```bash
DATE=<D> GOAL_X=<x> GOAL_Y=<y> GOAL_YAW=<yaw> VARIANT=B0       bash src/scout_apps/control/spmpc_local_planner/scripts/run_continuous_real.sh
DATE=<D> GOAL_X=<x> GOAL_Y=<y> GOAL_YAW=<yaw> VARIANT=B_smooth bash src/scout_apps/control/spmpc_local_planner/scripts/run_continuous_real.sh
DATE=<D> GOAL_X=<x> GOAL_Y=<y> GOAL_YAW=<yaw> VARIANT=B_slosh  bash src/scout_apps/control/spmpc_local_planner/scripts/run_continuous_real.sh
DATE=<D> GOAL_X=<x> GOAL_Y=<y> GOAL_YAW=<yaw> VARIANT=B_ours   bash src/scout_apps/control/spmpc_local_planner/scripts/run_continuous_real.sh
```

### 5.2 w_slosh 实物扫

先用 `B_slosh` 扫定工作点，再跑四组主实验：

```bash
DATE=<D> GOAL_X=<x> GOAL_Y=<y> GOAL_YAW=<yaw> VARIANT=B_slosh W_SLOSH=1 BAG_NAME=B_slosh_w1 bash src/scout_apps/control/spmpc_local_planner/scripts/run_continuous_real.sh
DATE=<D> GOAL_X=<x> GOAL_Y=<y> GOAL_YAW=<yaw> VARIANT=B_slosh W_SLOSH=2 BAG_NAME=B_slosh_w2 bash src/scout_apps/control/spmpc_local_planner/scripts/run_continuous_real.sh
DATE=<D> GOAL_X=<x> GOAL_Y=<y> GOAL_YAW=<yaw> VARIANT=B_slosh W_SLOSH=3 BAG_NAME=B_slosh_w3 bash src/scout_apps/control/spmpc_local_planner/scripts/run_continuous_real.sh
DATE=<D> GOAL_X=<x> GOAL_Y=<y> GOAL_YAW=<yaw> VARIANT=B_slosh W_SLOSH=5 BAG_NAME=B_slosh_w5 bash src/scout_apps/control/spmpc_local_planner/scripts/run_continuous_real.sh
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
/spmpc/slosh_horizon_summary         预测 h_peak/h_p95，主要对 9D slosh 变体有意义
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

/spmpc/status = ACADOS_SOLVE_FAILED_*
  初始 infeasible、参考拟合异常或约束过紧。检查车是否在路径起点附近、goal 是否过近。

RGB 推断无液面 / 全 0
  ROI/HSV/calibration 或曝光参数错误。该 run 不进入统计，修正后重跑。

四组结果不可比
  goal/calibration/ROI/起摆位/RGB 参数不一致。废弃重做。
```
