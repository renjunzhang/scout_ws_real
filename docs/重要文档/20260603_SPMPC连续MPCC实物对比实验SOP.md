# 20260603 SPMPC 连续 MPCC 实物对比实验 SOP

> 状态：2026-06-03 首版。对象：`spmpc_local_planner` 的 `continuous_mpcc_acados` 后端(规控一体连续 MPCC)。
> 主评价真值：RGB max(left, center, right) 液面高度(离线从 bag 推)；`/spmpc/*` 与 observer 只作模型辅助。
> 仿真只用于集成联调(已通)，**抑晃效果只在实物上以 RGB 真值评定**。
> 运行脚本：`src/scout_apps/control/spmpc_local_planner/scripts/run_continuous_real.sh`
> acados 安装：`docs/实物实验注意事项/代码移植/20260602_实物端代码拉取与子模块注意事项.md` §3.1
> 路径公平性：实物不强求 bit-level replay 同一条 JSON；每组从同一地面起点标记出发，发送同一目标点，使用同一 `P2_s_curve` 模板参数，最大程度生成同构路径。

---

## 0. 实验结构

### 0.1 后端与变体

```text
solver_backend = continuous_mpcc_acados   (本 SOP 主线; primitive 为回退/附录)

变体(planner_variant) -> 模型:
  B0      / B_smooth   -> spmpc_b0   (5 维, 无 slosh)
  B_slosh / B_ours     -> spmpc_slosh (9 维, slosh 模态)

权重(w_contour/w_lag/w_progress/w_control/w_smooth/w_slosh) 由 variants.yaml 注入;
w_slosh 可经 launch 运行时覆盖扫值, 无需重 codegen。
```

### 0.2 论文表格结构

```text
正文主表(内部消融, RGB 真值):
  B0 / B_slosh / B_smooth / B_ours   (均 continuous_mpcc_acados)
证明关系:
  B_slosh vs B0      slosh 项是否有效
  B_smooth vs B0     普通平滑是否有效
  B_ours  vs B_smooth  slosh-aware 是否优于 smooth-only
方法学对照(可选): primitive vs continuous 同物理核(见连续 MPCC 升级方案 §8.3)
```

---

## 1. 实物机一次性准备

### 1.1 装 acados(只做一次)

按 `20260602_实物端...md §3.1` 完整步骤(focal 的 GLIBC/rustup 两坑已记)。装完确认：

```bash
echo "$ACADOS_SOURCE_DIR"          # /home/geist/acados
ls ~/acados/lib/libacados.so
```

### 1.2 生成求解器 + 构建(代码更新后才需重做)

```bash
source /opt/ros/noetic/setup.bash && source ~/.bashrc
cd /home/geist/scout_ws
# codegen(在 acados venv 里):
source ~/acados_venv/bin/activate
python scripts/.../acados/generate_spmpc_acados.py --model b0
python scripts/.../acados/generate_spmpc_acados.py --model slosh
deactivate
# 带 acados 构建:
catkin_make --pkg spmpc_local_planner --force-cmake     # 应打印 "building continuous_mpcc_acados backend (b0 + slosh)"
source devel/setup.bash
```

构建日志没出现 "acados found" 就是没探测到(检查 `ACADOS_SOURCE_DIR` 与生成的 `.so`)；
此时节点会是 stub，运行时 `/spmpc/status` 报 `ACADOS_NOT_IMPLEMENTED`。

---

## 2. 现场启动总顺序

每组实验按此开终端；终端 A/B 与 Route A `20260527` 完全一致(传感器栈 + RGB 冻结)。

### 2.1 终端 A：传感器/定位栈

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws
src/scout_apps/control/scout_local_planner/scripts/launch_real_sensors_stack.sh
```

确认 `/odom`、`/camera/color/image_raw`、`/camera/color/camera_info`、`/imu/data` 有数据。

### 2.2 终端 B：冻结 RGB 参数(当天全组同一份)

```bash
OUT_DIR=/home/geist/slosh_bags/real/<DATE>_visual_tuning/realsense_rgb_fixed_params \
src/scout_apps/control/scout_local_planner/scripts/set_realsense_rgb_manual_params.sh
```

细节(交互调参、频率检查)见 `20260527` §1.2。**组间不要重开自动曝光/改白平衡。**

### 2.3 终端 C：跑一组 continuous(脚本负责路径+goal+启动+录包)

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
source ~/.bashrc        # 带 ACADOS_SOURCE_DIR / LD_LIBRARY_PATH(continuous 必需)

DATE=<DATE> GOAL_X=<x> GOAL_Y=<y> GOAL_YAW=<yaw> VARIANT=B0 \
  bash src/scout_apps/control/spmpc_local_planner/scripts/run_continuous_real.sh
```

每组前把车摆回同一地面标记；当天所有组用**同一 goal**、同一模板参数和同一起点标记。
脚本会做 acados/传感器 preflight、从当前车位姿生成模板路径、发 goal、启 continuous 后端、录包(含相机)。

说明：实物端严格复用同一条路线不可操作，原因是每次摆车、定位和底盘初始姿态都会有小偏差。正式口径不是“完全同一 JSON 路径”，而是“同目标点 + 同模板生成规则 + 同起点标记 + 同参数”，并在分析中报告跟踪误差与路径进度，用于排除明显异常 run。

### 2.4 终端 D(可选, 推荐)：在线液面高度监控

复用离线三标尺检测核, 实时发布 max-LCR 液面高度, 边跑边看(不替代离线 RGB 真值, 但便于现场判断)。
前提: 当天已按 §3(离线流程文档)标定 ROI+三标尺+HSV, 并把 HSV 写入 calibration 的 `hsv:` 段(或用参数传)。

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
roslaunch realsense_liquid_measurement online_liquid_height.launch \
  calibration:=<当天 calibration.yaml> publish_debug:=true
```

发布: `/liquid/height`(max-LCR mm)、`/liquid/height_lcr`([L,C,R])、`/liquid/height_median`、`/liquid/debug_image`。

**推荐: 一屏监控前端**(纯订阅, 不开相机/不碰控制, 关窗只退本前端)：

```bash
roslaunch realsense_liquid_measurement liquid_monitor.launch
```

一屏 2x2：RGB overlay / 液面曲线(max-LCR+L/C/R+模型 h) / 状态灯(OVERLAY·LIQUID·SPMPC·SOLVER) / 控制·模型数值。
轻量看单条曲线也可 `rqt_plot /liquid/height /spmpc/slosh_height`(都 mm)或 `rqt_image_view /liquid/debug_image`。

注意: 在线值与离线 `red_liquid_infer_from_bag.py` 同检测核, 但**论文真值仍以离线为准**(离线可控帧采样/平滑/质量检查)。

---

## 3. 对比组与 w_slosh 实物扫

### 3.1 四组主消融(各一次, 组间重摆车)

```bash
DATE=<D> GOAL_X=<x> GOAL_Y=<y> GOAL_YAW=<yaw> VARIANT=B0       bash .../run_continuous_real.sh
DATE=<D> GOAL_X=<x> GOAL_Y=<y> GOAL_YAW=<yaw> VARIANT=B_smooth bash .../run_continuous_real.sh
DATE=<D> GOAL_X=<x> GOAL_Y=<y> GOAL_YAW=<yaw> VARIANT=B_slosh  bash .../run_continuous_real.sh
DATE=<D> GOAL_X=<x> GOAL_Y=<y> GOAL_YAW=<yaw> VARIANT=B_ours   bash .../run_continuous_real.sh
```

### 3.2 w_slosh 实物扫(确定 B_slosh/B_ours 工作点)

```bash
# 同 VARIANT=B_slosh, 改 W_SLOSH + BAG_NAME, 每值一次:
DATE=<D> GOAL_X=.. GOAL_Y=.. GOAL_YAW=.. VARIANT=B_slosh W_SLOSH=1 BAG_NAME=B_slosh_w1 bash .../run_continuous_real.sh
#  W_SLOSH=2 BAG_NAME=B_slosh_w2 ...  3 ...  5 ...
```

实物扫以 RGB peak 为准(不是 sim observer)。先扫定 w_slosh，再跑 3.1 四组。

---

## 4. RGB 真值离线推断(每个 bag)

主真值 = RGB max(left,center,right) 液面高度，离线从录包算：

```bash
python3 src/scout_apps/sensors/realsense_liquid_measurement/scripts/red_liquid_infer_from_bag.py \
  --bag <BAG_DIR>/<VARIANT>.bag \
  --topic /camera/color/image_raw \
  --calibration <当天 calibration.json> \
  --out-dir <BAG_DIR>/<VARIANT>_rgb
```

calibration / ROI / 红色 HSV 阈值口径见 `20260527` 与 `红色液体视觉验证固定流程.md`。
当天所有组用**同一 calibration 与同一 ROI**。

---

## 5. 分析口径

```text
主指标(RGB 真值):  max-LCR 的 peak / p95 / rms / AUC (mm)
模型辅助(可选):    /spmpc/slosh_height(当前模型高度, mm); /spmpc/slosh_horizon_summary(预测 h_peak/h_p95, mm, 仅 9 维变体有意义);
                   observer /spmpc/debug/slosh_state(模态 η, 非高度; 跨变体同尺度但不可与 RGB 混为真值)
单位说明:          模型高度已统一发布为 mm, 与 RGB /liquid/height 同单位可并排; 但模型高度是 proxy, 论文真值仍以 RGB 离线为准。
代价/行为:         /spmpc/cost_breakdown(已含 slosh 项), cmd_v/odom_v, /spmpc/solver_time_ms
统计窗口:          tracking 主窗口; 终点收敛段单列, 不混入主效果统计
```

### 5.1 cost 占比作为 w_slosh 辅助判据

`/spmpc/cost_breakdown` 字段 12-21 是各项占比(分母已修为各项绝对值之和, 有界);
`analyze_b0_bslosh_compare.py` 的 `cost占比` 行给离线均值占比。w_slosh 实物扫时同时盯它:

```text
健康区间(B_slosh/B_ours):  slosh 占比约 20%~40%, contour/progress 仍有份额
偏大:  slosh > ~50% 且 contour≈0  -> 跟踪被牺牲, 控制"画龙"(cmd_omega 上升), 调小 w_slosh
偏小:  slosh < ~10%               -> 压晃不足, 调大 w_slosh
```

判定主依据仍是 RGB peak; cost 占比用于解释"为什么"和缩小 w_slosh 搜索范围。

`/spmpc/*` 与 observer 只能作"模型预测/工程 proxy"，**论文结论以 RGB 真值为准**。

---

## 6. 常见错误

```text
/spmpc/status = ACADOS_NOT_IMPLEMENTED
  节点是无 acados 的 stub 构建。带 ACADOS_SOURCE_DIR 重编(§1.2), 确认打印 "acados found"。

/spmpc/status = ACADOS_NOT_CREATED
  生成的 .so 缺失或链接失败。重跑 generate_spmpc_acados.py(b0+slosh)并 --force-cmake。

节点启动即崩 / 找不到 libacados/hpipm/blasfeo
  运行 continuous 的终端没 source ~/.bashrc(缺 LD_LIBRARY_PATH=~/acados/lib)。

/spmpc/status = ACADOS_SOLVE_FAILED_*
  初始 infeasible 或参考拟合异常; 检查车是否在路径起点附近、goal 是否过近。

RGB 推断无液面 / 全 0
  ROI/HSV/calibration 不对, 或当天换了曝光; 用 20260527 §1.2 重新冻结并核对 calibration。

四组结果不可比
  组间 goal/calibration/ROI/起摆位 必须一致; 否则废弃重做。
```
