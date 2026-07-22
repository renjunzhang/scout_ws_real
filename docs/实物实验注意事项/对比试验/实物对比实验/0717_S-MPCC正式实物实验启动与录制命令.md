# S-MPCC 正式实物实验启动与录制命令

> 日期：2026-07-18
>
> 适用范围：[0717_S-MPCC正式实物实验矩阵_先40后88.md](./0717_S-MPCC正式实物实验矩阵_先40后88.md) 的参数冻结 PF pilot，以及 S1、S2A、S2B 正式 trial。
>
> 本文件只规定现场命令。实验次数、随机顺序、统计口径、失败规则和 recorder 设置以矩阵文件为准。

---

## 1. 当前脚本入口、执行顺序与边界

除一次不计入矩阵的 H0 path-freeze smoke 外，参数 pilot 和正式 trial 都必须 replay 已冻结的 H0/H1/L1 JSON，禁止根据每次实际起点重新生成路径。

当前入口状态：

```text
run_spmpc_real_fixed_path_trial.sh：已支持 generate/replay，C1 参数 PF pilot 的主入口
run_continuous_real.sh：仍按当前起点生成路径，只用于 smoke
```

`run_spmpc_real_fixed_path_trial.sh` 是“单次 trial 一键脚本”，不会重复启动底盘、定位、传感器或 standalone slosh monitor；运行它之前先按第 3 节启动基础栈一键脚本。trial 脚本内部实际调用：

```text
spmpc_fixed_path.launch
record_spmpc_full_rgb_bag.sh
template_fixed_path_generator.py 或 fixed_global_path_runner.py
```

当前 `spmpc_fixed_path.launch` 固定使用 `scout_mini + tube_default + fixed_path`，一键脚本没有 `container_config` 参数。因此：

- `C1/tube_default` 的 PF 权重筛选使用一键脚本；
- `C2`、正式 40/88、需要人工 Enter 门或需要单独核查 recorder/planner 时，使用本文第 5 节之后的多终端流程；
- 不得在 C2 run 中仅修改标签却继续调用当前一键脚本，否则实际容器模型仍是 `tube_default`。

一键 replay 的真实执行顺序为：

```text
检查冻结 JSON
→ 启动 fixed_global_path_runner.py
→ 自动等待起点门控（无 --manual-start、无需按 Enter，最长默认 120 s）
→ 路径开始持续发布
→ 启动 recorder
→ 等待 recorder 默认 8 s
→ 启动 spmpc_fixed_path.launch
→ recorder 超时、planner 退出或 Ctrl-C
→ 自动停 planner、发零速度、停 recorder 和 path source
```

这意味着一键 replay 必须在执行命令前就完成“遥控回起点、航向对齐、液体静稳和安全确认”。如果尚未进入门控范围，脚本只等待路径，不会提前启动 recorder 浪费录包时间。

正式 40/88 次仍使用多终端流程，保留路径 Enter 门、recorder、planner 和开始时刻的独立人工检查：

```text
实物基础栈
  + standalone slosh monitor
  + fixed_global_path_runner.py --mode replay
  + spmpc_experiment.launch
  + record_spmpc_full_rgb_bag.sh
```

在 H1 JSON、最终 `w_slosh`、Smooth-match `v_ref` 或 C2 配置仍为占位值时，对应正式实验为 NO-GO。旧 0705/0706 的 `w_slosh=5` 只作为历史 pilot 先验，不能直接替代下面的参数冻结步骤。

---

## 2. 每个终端的公共环境

每个新终端都先执行：

```bash
export SCOUT_WS="${SCOUT_WS:-${HOME}/scout_ws}"
source /opt/ros/noetic/setup.bash
source "${SCOUT_WS}/devel/setup.bash"
cd "${SCOUT_WS}"
```

确认当前代码和构建：

```bash
git status --short
git rev-parse HEAD
rospack find spmpc_local_planner
rospack find scout_local_planner
```

正式 revision 必须与冻结记录一致。工作区存在未归档修改时，不开始正式采集。

---

## 3. 实物基础系统

现场优先使用已有的一键基础栈脚本，不逐个手动启动节点：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

SCOUT_WS=/home/geist/scout_ws \
REALSENSE_COLOR_WIDTH=1920 \
REALSENSE_COLOR_HEIGHT=1080 \
REALSENSE_COLOR_FPS=30 \
REALSENSE_ENABLE_DEPTH=false \
REALSENSE_ENABLE_INFRA=false \
WAIT_FOR_ODOM=true \
WAIT_FOR_LOCALIZATION_MAP=false \
bash src/scout_apps/control/scout_local_planner/scripts/launch_real_sensors_stack.sh
```

该脚本会统一完成：

```text
CAN can0 500000
→ Scout Mini 底盘和 /odom
→ nanoscan3 前雷达和 /scan_front
→ Cartographer localization
→ Scout IMU 和 /imu/data
→ RealSense 1920x1080@30 Hz（depth/infra 关闭）
→ camera_info 与 camera/IMU hz 检查
```

日志默认写入 `/tmp/launch_real_sensors_stack_<时间戳>/`。该终端必须保持运行；按 `Ctrl-C` 会停止它启动的全部基础节点。

无论 PF pilot 是否录 RGB，基础栈都必须在最开始统一启动 RealSense，并在本批实验期间保持运行，不得在 pilot 与正式 trial 之间停止后重新启动。这样可以避免后续再次启动带来的节点状态、相机时间基准、曝光稳定过程和现场操作差异。

PF pilot 的“无 RGB”只表示 `run_spmpc_real_fixed_path_trial.sh` 强制不订阅、不写入 raw RGB、compressed RGB、depth 或 online liquid bag 数据，不表示关闭相机节点，因此不会产生 RGB rosbag 存储占用。正式 trial 开始前只需再次核对冻结的分辨率、帧率、曝光、增益和白平衡，不重新启动 RealSense。

基础栈打印 `Stack is running` 后，另开终端做实验前补充检查：

```bash
timeout 10s rostopic hz /odom
timeout 10s rostopic hz /scan_front
timeout 10s rostopic hz /imu/data
timeout 5s rosrun tf tf_echo map base_link
rostopic info /cmd_vel
df -h "${HOME}/slosh_bags"
```

正式 trial 再核对相机冻结状态：

```bash
timeout 10s rostopic hz /camera/color/image_raw
rostopic echo -n 1 /camera/color/camera_info
```

开始 planner 前，`/cmd_vel` 不得存在旧 planner publisher。若基础栈脚本退出，不得继续启动 path/recorder/planner。

只有基础栈一键脚本本身失败时，才按其 `/tmp/launch_real_sensors_stack_*` 日志定位具体的 base、LiDAR、localization、IMU 或 RealSense 节点，不把逐节点启动作为日常 SOP。

---

## 4. 正式前参数冻结 pilot

### 4.1 去现场前冻结候选和门槛

默认参数候选：

```text
W1: w_slosh=1
W2: w_slosh=2
W5: w_slosh=5
```

是否加入 `W10` 必须在查看本轮内部模型筛选结果前写入 pilot 协议。优先用仿真/离线 horizon 检查决定；若只根据 W5 的安全和 tracking 门槛条件性加入，也必须事先写明触发规则，不能看到权重排序后临时追加。

除 `w_slosh` 外统一保持：

```text
v_ref=0.20
v_max=0.8
omega_max=1.2
a_max=0.6
alpha_max=1.2
horizon=60
rho_eta_dot=0.3
delay=fixed_closed_loop 0.15/0.22
terminal/gate/limiter/fallback 相同
C1、液深、安装和冻结路径相同
RECORD_RGB=false，不启动在线视觉液面节点
```

在开始候选运行前，必须写明并冻结：tracking p95、completion time、solve-time/deadline、fallback、执行层干预和数据完整性门槛，以及由 B0/B_smooth 重复 run 的内部 `H_modal` 波动计算 `delta_model` 的规则。

### 4.2 一键启动单次参数 pilot

#### 4.2.1 当前脚本的 pilot 默认值

| 字段 | `PILOT_MODE=true` 的实际行为 |
| --- | --- |
| `PATH_SOURCE_MODE` | 默认 `replay`；首次冻结 H0 时显式改为 `generate` |
| `PATH_FILE` | `${HOME}/fixed_paths/real/${DATE}_spmpc_parameter_pilot/H0_weight_pilot.json` |
| `RUN_OUT_DIR` | 未指定时为 `${HOME}/slosh_bags/real/${DATE}_spmpc_parameter_pilot/${PILOT_METHOD}` |
| generate 几何 | goal `(-5.424,-4.736,0)`；`s_curve`、spacing `0.05`、amplitude ratio `0.18`、min/max `0.25/1.20`、left、smooth `3` |
| 起点门控 | `START_POS_TOL=0.08`、`START_YAW_TOL=0.15`、`START_HOLD_SEC=0.5`、最长等待 `120 s` |
| 路径发布 | `2 Hz`，持续发布至清理；replay 不发送 `/scout/goal` |
| topics/frames | reference `/scout/global_path_fixed`、goal `/scout/goal`、cmd `/cmd_vel`、costmap `/map`、target frame `map`、base frame `base_link` |
| planner | `spmpc_fixed_path.launch`，默认容器 `tube_default`，仅用于 C1 |
| solver/control | `continuous_mpcc_acados`、`V_REF=0.20`、`fixed_closed_loop 0.15/0.22` |
| 公共限制 | `alpha_max=1.2`、线加速度 `0.6`、角速度/角加速度 `1.2` |
| RGB/视觉 | `PILOT_RECORD_RGB=false` 时强制 raw RGB、compressed RGB、depth、online liquid 全部关闭 |
| 其他录制 | scan 和 `/spmpc/*` 默认记录；`RECORD_STANDALONE_SLOSH=true` 只表示订阅已有 `/slosh/*`，不会启动 monitor |
| timing | path source 启动等待 `2 s`、recorder 启动等待 `8 s`、planner 启动等待 `2 s` |
| recorder | `RECORD_SEC=60`、`MAX_RECORD_SEC=60`、`RECORD_TOPIC_INFO=false` |
| 退出 | 当 `CMD_TOPIC=/cmd_vel` 时自动发布一次零速度；随后停止 recorder 和 path source |

脚本会验证数值、ROS master、冻结路径是否存在以及 recorder 是否可读。pilot generate 遇到已有 `PATH_FILE` 时默认失败，不会静默覆盖。

一键脚本可直接覆盖的是 `V_REF`、`W_SLOSH`、delay、`ALPHA_MAX` 和 shared execution limits；`v_max`、horizon、`rho_eta_dot`、terminal/gate/fallback 仍来自当前加载的 YAML，不是该脚本的环境变量。若修改这些 YAML，必须重新归档配置并重新冻结，不能只改文档中的命令标签。

#### 4.2.2 首次生成并冻结 H0

历史方案确认现场机使用：

```text
工作空间：/home/geist/scout_ws
历史 bag：/home/geist/slosh_bags/real/<DATE>_fixed_path_compare/
历史路径：/home/geist/fixed_paths/real/<DATE>_fixed_path_compare/fixed_s_curve_compare.json
```

历史路径文件由旧一键流程逐 run 重新生成并覆盖，不能直接当作冻结 H0。当前开发机上的 0706 同名镜像文件终点为 `(7, 2)`，也不符合本实验的 `(-5.424, -4.736)` 终点，因此下一次现场先做一次不计入 15 次矩阵的 path-freeze smoke：

`generate` 模式没有起点门控。执行下面命令前必须把机器人精确放在地面起点标记并对齐航向，随后保持静止；生成器会以发送 goal 时的当前 map 位姿和 heading 生成路径。

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

DATE=20260718 \
PILOT_MODE=true \
PILOT_METHOD=B0 \
PATH_SOURCE_MODE=generate \
PATH_FILE=/home/geist/fixed_paths/real/20260718_spmpc_parameter_pilot/H0_weight_pilot.json \
RUN_LABEL=PF_PATH_FREEZE_H0_B0_smoke01 \
RUN_OUT_DIR=/home/geist/slosh_bags/real/20260718_spmpc_parameter_pilot/PF/PATH/H0_C1/B0 \
CMD_TOPIC=/cmd_vel \
PILOT_RECORD_RGB=false \
RECORD_TOPIC_INFO=false \
RECORDER_STARTUP_SEC=8 \
RECORD_SEC=60 \
MAX_RECORD_SEC=60 \
V_REF=0.20 \
ALPHA_MAX=1.2 \
SHARED_LINEAR_ACCEL_LIMIT_ENABLE=true \
SHARED_LINEAR_ACCEL_MAX=0.6 \
SHARED_ANGULAR_LIMIT_ENABLE=true \
SHARED_ANGULAR_RATE_MAX=1.2 \
SHARED_ANGULAR_ACCEL_MAX=1.2 \
DELAY_PHASE_MODE=fixed_closed_loop \
DELAY_PHASE_LINEAR_DELAY_SEC=0.15 \
DELAY_PHASE_ANGULAR_DELAY_SEC=0.22 \
OPERATOR_NOTE="C1 H0 path-freeze smoke; no RGB" \
bash src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_real_fixed_path_trial.sh
```

在该 run 成功到点、路径安全且 tracking 合格后，冻结文件为：

```text
/home/geist/fixed_paths/real/20260718_spmpc_parameter_pilot/H0_weight_pilot.json
```

保存冻结证据：

```bash
sha256sum /home/geist/fixed_paths/real/20260718_spmpc_parameter_pilot/H0_weight_pilot.json \
  | tee /home/geist/slosh_bags/real/20260718_spmpc_parameter_pilot/PF/PATH/H0_C1/B0/PF_PATH_FREEZE_H0_B0_smoke01_path_sha256.txt
```

记录其 SHA-256，并且后续不得再用 `generate` 覆盖。脚本在 pilot generate 时默认拒绝覆盖已有文件；`ALLOW_PILOT_PATH_OVERWRITE=true` 只允许在书面作废旧 H0、准备完整重新冻结时使用。

#### 4.2.3 replay 单次权重候选

执行命令前先把机器人遥控到 H0 起点附近、对齐航向并等待液体静稳。命令启动后没有 Enter 门；一旦位置和航向连续满足门控 `0.5 s`，脚本自动进入 recorder/planner 启动流程。

按本次 block 设置命名，只改 `PILOT_METHOD` 即可切换候选：

```bash
export SCOUT_WS=/home/geist/scout_ws
export EXP_DATE=20260718
export PATH_JSON=/home/geist/fixed_paths/real/20260718_spmpc_parameter_pilot/H0_weight_pilot.json

PILOT_MODE=true \
PILOT_METHOD=W2 \
DATE="${EXP_DATE}" \
PATH_SOURCE_MODE=replay \
PATH_FILE="${PATH_JSON}" \
RUN_LABEL=PF_WS_H0_C1_W2_b01_r01 \
RUN_OUT_DIR="${HOME}/slosh_bags/real/${EXP_DATE}_spmpc_parameter_pilot/PF/WS/H0_C1/W2" \
START_POS_TOL=0.08 \
START_YAW_TOL=0.15 \
START_HOLD_SEC=0.5 \
START_GATE_TIMEOUT_SEC=120 \
PATH_PUBLISH_RATE=2.0 \
CMD_TOPIC=/cmd_vel \
PILOT_RECORD_RGB=false \
RECORD_SCAN=true \
RECORD_STANDALONE_SLOSH=true \
RECORD_ALL_EXISTING_TOPICS=false \
RECORD_TOPIC_INFO=false \
RECORDER_STARTUP_SEC=8 \
RECORD_SEC=60 \
MAX_RECORD_SEC=60 \
V_REF=0.20 \
ALPHA_MAX=1.2 \
SHARED_LINEAR_ACCEL_LIMIT_ENABLE=true \
SHARED_LINEAR_ACCEL_MAX=0.6 \
SHARED_ANGULAR_LIMIT_ENABLE=true \
SHARED_ANGULAR_RATE_MAX=1.2 \
SHARED_ANGULAR_ACCEL_MAX=1.2 \
DELAY_PHASE_MODE=fixed_closed_loop \
DELAY_PHASE_LINEAR_DELAY_SEC=0.15 \
DELAY_PHASE_ANGULAR_DELAY_SEC=0.22 \
OPERATOR_NOTE="C1 internal-model weight screening W2; no RGB" \
bash src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_real_fixed_path_trial.sh
```

`PILOT_METHOD` 可直接写 `B0`、`Bsmooth`、`W1`、`W2`、`W5`、`W10`，也支持 `W3.5` 这类临时候选。脚本会自动映射 `VARIANT/W_SLOSH`，pilot 默认不录 RGB，并把路径模式、起点容差和实际权重写入 sidecar。机器人用遥控回到起点附近即可；脚本默认允许 `0.08 m / 0.15 rad` 的误差，但仍需以地面起点标记为准，不能继续无上限放宽。

#### 4.2.4 运行中观察、停止与输出

启动横幅至少核对：

```text
pilot=true
variant / PILOT_METHOD 正确
record_rgb=false
path_source=replay
path_file 指向冻结 H0
start_gate=0.08 m / 0.15 rad
v_ref/w_slosh 与候选一致
```

正常日志顺序应为：

```text
[path] starting replay source
[path] waiting ... start gate
[record] starting black-box recorder
[goal] replay mode ... skipped
[launch] starting planner
[run] recording ...
```

到点后继续保留至少 `5 s` 残余晃动。可以等待 recorder 到 `60 s` 自动退出；如果要提前结束，先等满到达后 `5 s`，再在一键终端按 `Ctrl-C`，由脚本统一清理。不要另开终端抢先杀 recorder 或 path runner。

每个 run 的主要输出为：

```text
${RUN_OUT_DIR}/${RUN_LABEL}.bag
${RUN_OUT_DIR}/${RUN_LABEL}_one_click_meta.env
${RUN_OUT_DIR}/${RUN_LABEL}_info.txt
${RUN_OUT_DIR}/${RUN_LABEL}_recorded_topics.txt
${RUN_OUT_DIR}/${RUN_LABEL}_{path_generator,recorder,planner}.log
```

### 4.3 15 次交错顺序

| Pilot block | 顺序 |
| ---: | --- |
| 01 | B0 → W1 → B_smooth → W2 → W5 |
| 02 | W2 → B0 → W5 → W1 → B_smooth |
| 03 | B_smooth → W5 → W2 → B0 → W1 |

每次优先使用第 4.2 节的一键命令；如果需要逐节点排障，也可使用本文后面的多终端 replay/recorder/planner 流程。对应运行环境设为：

```bash
export EXP_DATE=20260718       # 改成实际日期
export STAGE=PF
export GROUP=WS
export PATH_ID=H0             # 权重筛选默认使用独立冻结 H0
export CONTAINER=C1
export METHOD=W1              # B0 / Bsmooth / W1 / W2 / W5
export BLOCK=01
export REPEAT=01

export PILOT_METHOD="${METHOD}"
export RUN_LABEL="${STAGE}_${GROUP}_${PATH_ID}_${CONTAINER}_${METHOD}_b${BLOCK}_r${REPEAT}"
export RUN_OUT_DIR="/home/geist/slosh_bags/real/${EXP_DATE}_spmpc_parameter_pilot/${STAGE}/${GROUP}/${PATH_ID}_${CONTAINER}/${METHOD}"
```

对应命名示例：

```text
PF_WS_H0_C1_B0_b01_r01
PF_WS_H0_C1_Bsmooth_b01_r01
PF_WS_H0_C1_W1_b01_r01
PF_WS_H0_C1_W2_b01_r01
PF_WS_H0_C1_W5_b01_r01
```

执行第 4.2.3 节命令时，将示例里的 `PILOT_METHOD=W2`、`RUN_LABEL=...W2...` 和 `RUN_OUT_DIR=.../W2` 分别替换为上述三个已导出的变量，避免方法名、实际权重、标签和目录不一致。

所有 `PF` bag 只用于模型侧参数决策，不进入正式 40/64/88。每次运行后只做数据完整性和安全检查；不得看完一个候选的内部模型排序后改变剩余顺序。

### 4.4 权重选择与 Smooth-match 后置

完成全部预注册候选后统一分析：

1. 先剔除未通过任务、tracking、实时性、fallback、执行层和数据完整性门槛的候选；
2. 检查 Z2/Z3 的完整 horizon、optimized first action 和局部速度/激励分配，排除全程统一降速；
3. 比较 block-paired 内部 `H_modal`/horizon peak、RMS、post-arrival RMS、完成时间和 tracking；
4. 相邻权重的额外模型侧收益低于冻结 `delta_model` 时选择较小权重；
5. 形成书面权重决策，设置 `FINAL_W_SLOSH=<最终值>`，并明确标记为 `model-side frozen weight`；
6. 另开 `PF/SM` 独立 pilot，只根据 completion time 调 `SmoothMatch` 的 `SMOOTH_MATCH_V_REF`，不查看该 pilot 的 RGB 排名；不同速度候选用 `GROUP=SMV<速度标签>` 区分，例如 `SMOOTH_MATCH_V_REF=0.18` 使用 `GROUP=SMV018`；
7. 归档四方法配置、effective config、路径/config hash、pilot 数据和 Git revision，提交正式 freeze commit。

只有第 7 步完成后，才允许把 `STAGE` 改为 `S1` 开始正式 40 次。

不录 RGB 的 PF pilot 不能证明真实液面改善。若时间允许，建议在正式 40 次前对最终候选和相邻候选另做少量、独立的 RGB freeze confirmation；若跳过该确认，也可以直接进入预注册的正式 40 次，但之后不得根据正式 RGB 结果重新调权重，否则原正式数据必须整体降级为 pilot。

---

## 5. 正式 40/88 与排障用多终端环境

本节及第 6–11 节不是一键脚本的内部步骤，而是正式采集/排障时的独立多终端入口。它使用 `spmpc_experiment.launch`，因此可以显式设置 C1/C2 container config、使用人工 Enter 门，并让 recorder 录制 90 s。

变量名注意：

```text
一键脚本输出变量：RUN_OUT_DIR
直接 recorder 输出变量：OUT_DIR
```

二者不要混用。PF 日常权重筛选优先按第 4 节一键执行；只有节点排障时才把 PF 放到本节的多终端流程。

先按随机表填写以下字段。示例是 S1/E2、H1、C1、B0、block 01：

```bash
export EXP_DATE=20260718
export STAGE=S1
export GROUP=E2
export PATH_ID=H1
export CONTAINER=C1
export METHOD=B0
export BLOCK=01
export REPEAT=01
```

正式运行 `METHOD=Bslosh` 前还必须从只读 freeze 配置填写：

```bash
read -rp "FINAL_W_SLOSH（从只读 freeze 配置填写）: " FINAL_W_SLOSH
export FINAL_W_SLOSH
```

正式或 pilot 运行 `METHOD=SmoothMatch` 前填写对应的独立调速候选/最终值：

```bash
read -rp "SMOOTH_MATCH_V_REF（候选或只读 freeze 值）: " SMOOTH_MATCH_V_REF
export SMOOTH_MATCH_V_REF
```

方法名与代码映射：

| `METHOD` | `VARIANT` | `V_REF` | `W_SLOSH` |
| --- | --- | ---: | ---: |
| `B0` | `B0` | `0.20` | `0` |
| `Bsmooth` | `B_smooth` | `0.20` | `0` |
| `W1` | `B_slosh` | `0.20` | `1` |
| `W2` | `B_slosh` | `0.20` | `2` |
| `W5` | `B_slosh` | `0.20` | `5` |
| `W10` | `B_slosh` | `0.20` | `10`，仅预注册启用 |
| `Bslosh` | `B_slosh` | `0.20` | `FINAL_W_SLOSH` |
| `SmoothMatch` | `B_smooth` | 独立 pilot 冻结值 | `0` |

执行映射：

```bash
case "${METHOD}" in
  B0)      export VARIANT=B0;       export V_REF=0.20; export W_SLOSH=0.0 ;;
  Bsmooth) export VARIANT=B_smooth; export V_REF=0.20; export W_SLOSH=0.0 ;;
  W1)      export VARIANT=B_slosh;  export V_REF=0.20; export W_SLOSH=1.0 ;;
  W2)      export VARIANT=B_slosh;  export V_REF=0.20; export W_SLOSH=2.0 ;;
  W5)      export VARIANT=B_slosh;  export V_REF=0.20; export W_SLOSH=5.0 ;;
  W10)     export VARIANT=B_slosh;  export V_REF=0.20; export W_SLOSH=10.0 ;;
  Bslosh)
    : "${FINAL_W_SLOSH:?先完成参数 pilot 并填写冻结的 FINAL_W_SLOSH}"
    export VARIANT=B_slosh
    export V_REF=0.20
    export W_SLOSH="${FINAL_W_SLOSH}"
    ;;
  SmoothMatch)
    : "${SMOOTH_MATCH_V_REF:?先填写独立 pilot 冻结的 SMOOTH_MATCH_V_REF}"
    export VARIANT=B_smooth
    export V_REF="${SMOOTH_MATCH_V_REF}"
    export W_SLOSH=0.0
    ;;
  *) echo "未知 METHOD=${METHOD}" >&2; exit 2 ;;
esac
```

路径和容器映射：

```bash
FREEZE_ROOT="${SCOUT_WS}/docs/实物实验注意事项/对比试验/实物对比实验/freeze"

case "${PATH_ID}" in
  H1) export PATH_JSON="${FREEZE_ROOT}/paths/H1_P2_s_curve.json" ;;
  L1) export PATH_JSON="${FREEZE_ROOT}/paths/L1_gentle.json" ;;
  H0) export PATH_JSON="${FREEZE_ROOT}/paths/H0_weight_pilot.json" ;;
  *) echo "未知 PATH_ID=${PATH_ID}" >&2; exit 2 ;;
esac

case "${CONTAINER}" in
  C1)
    export CONTAINER_CONFIG=tube_default
    export CONTAINER_RADIUS=0.0185
    export LIQUID_HEIGHT=0.058
    export DAMPING_RATIO=0.05
    ;;
  C2)
    : "${C2_CONTAINER_CONFIG:?填写冻结的 C2 配置名，不带 .yaml}"
    : "${C2_CONTAINER_RADIUS:?填写冻结的 C2 内半径}"
    : "${C2_LIQUID_HEIGHT:?填写冻结的 C2 液深}"
    export CONTAINER_CONFIG="${C2_CONTAINER_CONFIG}"
    export CONTAINER_RADIUS="${C2_CONTAINER_RADIUS}"
    export LIQUID_HEIGHT="${C2_LIQUID_HEIGHT}"
    export DAMPING_RATIO=0.05
    ;;
  *) echo "未知 CONTAINER=${CONTAINER}" >&2; exit 2 ;;
esac
```

生成 run 名称和目录：

```bash
case "${STAGE}" in
  PF)
    export RUN_CLASS=pilot
    export PILOT_MODE_META=true
    export PILOT_METHOD_META="${METHOD}"
    export RECORD_RGB=false
    export DATASET_ROOT="${HOME}/slosh_bags/real/${EXP_DATE}_spmpc_parameter_pilot"
    ;;
  S1|S2A|S2B)
    export RUN_CLASS=formal
    export PILOT_MODE_META=false
    export PILOT_METHOD_META=""
    export RECORD_RGB=true
    export DATASET_ROOT="${HOME}/slosh_bags/real/${EXP_DATE}_spmpc_formal"
    ;;
  *) echo "未知 STAGE=${STAGE}" >&2; exit 2 ;;
esac

export RUN_LABEL="${STAGE}_${GROUP}_${PATH_ID}_${CONTAINER}_${METHOD}_b${BLOCK}_r${REPEAT}"
export OUT_DIR="${DATASET_ROOT}/${STAGE}/${GROUP}/${PATH_ID}_${CONTAINER}/${METHOD}"
export NAME="${RUN_LABEL}"
export CONTAINER_YAML="${SCOUT_WS}/src/scout_apps/control/spmpc_local_planner/config/containers/${CONTAINER_CONFIG}.yaml"

test -s "${PATH_JSON}"
test -s "${CONTAINER_YAML}"
mkdir -p "${OUT_DIR}"
test ! -e "${OUT_DIR}/${NAME}.bag"

echo "RUN_LABEL=${RUN_LABEL}"
echo "PATH_JSON=${PATH_JSON}"
echo "CONTAINER_CONFIG=${CONTAINER_CONFIG}"
echo "RUN_CLASS=${RUN_CLASS}"
echo "VARIANT=${VARIANT} V_REF=${V_REF} W_SLOSH=${W_SLOSH}"
echo "OUT_DIR=${OUT_DIR}"
```

把本次公共变量保存到临时文件，随后每个 trial 终端都 source 它：

```bash
export RUN_ENV="/tmp/${RUN_LABEL}.env"
declare -px SCOUT_WS EXP_DATE STAGE GROUP PATH_ID CONTAINER METHOD BLOCK REPEAT \
  RUN_CLASS PILOT_MODE_META PILOT_METHOD_META RECORD_RGB DATASET_ROOT \
  VARIANT V_REF W_SLOSH PATH_JSON CONTAINER_CONFIG \
  CONTAINER_RADIUS LIQUID_HEIGHT DAMPING_RATIO RUN_LABEL OUT_DIR NAME \
  CONTAINER_YAML RUN_ENV > "${RUN_ENV}"
```

同时保存路径、配置和 revision 证据：

```bash
sha256sum "${PATH_JSON}" > "${OUT_DIR}/${NAME}_path_sha256.txt"
sha256sum \
  "${SCOUT_WS}/src/scout_apps/control/spmpc_local_planner/config/planner/common.yaml" \
  "${SCOUT_WS}/src/scout_apps/control/spmpc_local_planner/config/planner/variants.yaml" \
  "${SCOUT_WS}/src/scout_apps/control/spmpc_local_planner/config/platforms/scout_mini.yaml" \
  "${SCOUT_WS}/src/scout_apps/control/spmpc_local_planner/config/experiments/fixed_path.yaml" \
  "${CONTAINER_YAML}" \
  > "${OUT_DIR}/${NAME}_config_sha256.txt"

git rev-parse HEAD > "${OUT_DIR}/${NAME}_git_revision.txt"
git status --short > "${OUT_DIR}/${NAME}_git_status.txt"
cp "${RUN_ENV}" "${OUT_DIR}/${NAME}_run_env.env"
```

---

## 6. 终端 A：standalone slosh monitor

每次切换容器时重启 monitor。同一容器连续 block 可以保持进程运行，但每个 trial 运动前必须 reset。

边界说明：一键脚本中的 `RECORD_STANDALONE_SLOSH=true` 只会把已经存在的 `/slosh/*` 加入 recorder whitelist，不会执行下面的 `roslaunch`。PF 只比较 planner 内部 `/spmpc/slosh_height` 和 horizon 时，本终端可省略；如果需要统一的 monitor-only 模型作为附加离线指标，则必须在启动一键命令之前另行运行并 reset。

```bash
source /opt/ros/noetic/setup.bash
source "${HOME}/scout_ws/devel/setup.bash"
export RUN_ENV=/tmp/S1_E2_H1_C1_B0_b01_r01.env  # 改成本次 RUN_LABEL
source "${RUN_ENV}"

roslaunch slosh_models slosh_monitor.launch \
  odom_topic:=/odom \
  cmd_vel_topic:=/cmd_vel \
  output_namespace:=/slosh \
  container_radius:="${CONTAINER_RADIUS}" \
  liquid_height:="${LIQUID_HEIGHT}" \
  damping_ratio:="${DAMPING_RATIO}" \
  use_parabola_term:=false \
  model_dt:=0.02 \
  accel_filter_alpha:=0.3 \
  min_dt:=0.001 \
  max_dt:=0.1
```

另一个终端确认并 reset：

```bash
source /opt/ros/noetic/setup.bash
source "${HOME}/scout_ws/devel/setup.bash"
timeout 5s rostopic echo -n 1 /slosh/height
rosservice call /slosh/reset
```

`/slosh/*` 只用于统一的模型监测和离线评价，不作为 B0/B_smooth 的控制输入。

---

## 7. 正式多终端 B：装载冻结路径并停在人工开始门

下面的 `--manual-start + 0.05/0.10` 是正式多终端口径，与一键 pilot 的“无 Enter + 0.08/0.15”不同，不要复制到第 4 节后误以为一键脚本也会等待 Enter。

```bash
source /opt/ros/noetic/setup.bash
source "${HOME}/scout_ws/devel/setup.bash"
export RUN_ENV=/tmp/S1_E2_H1_C1_B0_b01_r01.env  # 改成本次 RUN_LABEL
source "${RUN_ENV}"

rosrun scout_local_planner fixed_global_path_runner.py \
  --mode replay \
  --path-file "${PATH_JSON}" \
  --output-topic /scout/global_path_fixed \
  --base-frame base_link \
  --manual-start \
  --start-pos-tol 0.05 \
  --start-yaw-tol 0.10 \
  --start-hold-sec 0.5 \
  --publish-rate 2.0 \
  --publish-count 0
```

看到下面提示后先不要按 Enter：

```text
Press Enter to continue fixed-path replay...
```

机器人必须先回到冻结路径起点标记，液体达到静稳门槛，planner 和 recorder 也必须已经启动。

---

## 8. 正式多终端 C：启动 recorder

这里直接调用 recorder，所以可以设置 `RECORD_SEC=90`；一键脚本仍受 `MAX_RECORD_SEC` 约束，默认只录 60 s。

```bash
source /opt/ros/noetic/setup.bash
source "${HOME}/scout_ws/devel/setup.bash"
export RUN_ENV=/tmp/S1_E2_H1_C1_B0_b01_r01.env  # 改成本次 RUN_LABEL
source "${RUN_ENV}"
cd "${SCOUT_WS}"

DATE="${EXP_DATE}" \
VARIANT="${VARIANT}" \
RUN_CLASS="${RUN_CLASS}" \
PILOT_MODE="${PILOT_MODE_META}" \
PILOT_METHOD="${PILOT_METHOD_META}" \
RUN_LABEL="${RUN_LABEL}" \
NAME="${NAME}" \
OUT_DIR="${OUT_DIR}" \
RECORD_SEC=90 \
RECORD_RGB="${RECORD_RGB}" \
RECORD_CAMERA="${RECORD_RGB}" \
RECORD_CAMERA_COMPRESSED=false \
RECORD_DEPTH=false \
RECORD_SCAN=true \
RECORD_STANDALONE_SLOSH=true \
RECORD_ONLINE_LIQUID=false \
RECORD_ALL_EXISTING_TOPICS=false \
RECORD_TOPIC_INFO=true \
ROSBAG_BUFFER_SIZE_MB=4096 \
SOLVER_BACKEND=continuous_mpcc_acados \
V_REF="${V_REF}" \
W_SLOSH="${W_SLOSH}" \
SLOSH_HEIGHT_MAX=-1.0 \
DELAY_PHASE_MODE=fixed_closed_loop \
DELAY_PHASE_LINEAR_DELAY_SEC=0.15 \
DELAY_PHASE_ANGULAR_DELAY_SEC=0.22 \
CONTROL_FREQUENCY=30 \
PATH_SOURCE_MODE=replay \
PATH_FILE="${PATH_JSON}" \
START_POS_TOL=0.05 \
START_YAW_TOL=0.10 \
START_HOLD_SEC=0.5 \
OPERATOR_NOTE="${RUN_CLASS} ${STAGE}/${GROUP} ${PATH_ID}/${CONTAINER} block ${BLOCK} ${METHOD} w_slosh=${W_SLOSH}" \
bash src/scout_apps/control/spmpc_local_planner/scripts/record_spmpc_full_rgb_bag.sh
```

等待 recorder 打印输出路径且没有立即退出，再启动 planner。90 s 是防遗忘的最长录制保护；trial 提前结束时仍需保证到达/停止后继续录制 5 s，再用 Ctrl+C 正常关闭 recorder。

---

## 9. 正式多终端 D：启动本次 planner

这里使用 `spmpc_experiment.launch` 并显式传入 `container_config`。一键脚本使用的是 `spmpc_fixed_path.launch`，当前只能采用其默认 `tube_default`。

```bash
source /opt/ros/noetic/setup.bash
source "${HOME}/scout_ws/devel/setup.bash"
export RUN_ENV=/tmp/S1_E2_H1_C1_B0_b01_r01.env  # 改成本次 RUN_LABEL
source "${RUN_ENV}"

roslaunch spmpc_local_planner spmpc_experiment.launch \
  planner_variant:="${VARIANT}" \
  experiment_mode:=fixed_path \
  experiment_config:=fixed_path \
  platform_config:=scout_mini \
  container_config:="${CONTAINER_CONFIG}" \
  reference_path_topic:=/scout/global_path_fixed \
  reference_target_frame:=map \
  costmap_topic:=/map \
  cmd_vel_topic:=/cmd_vel \
  publish_cmd_vel:=true \
  solver_backend:=continuous_mpcc_acados \
  v_ref:="${V_REF}" \
  w_slosh:="${W_SLOSH}" \
  slosh_height_max:=-1.0 \
  alpha_max:=1.2 \
  shared_linear_accel_limit_enable:=true \
  shared_linear_accel_max:=0.6 \
  shared_angular_limit_enable:=true \
  shared_angular_rate_max:=1.2 \
  shared_angular_accel_max:=1.2 \
  delay_phase_mode:=fixed_closed_loop \
  delay_phase_linear_delay_sec:=0.15 \
  delay_phase_angular_delay_sec:=0.22
```

路径尚未发布时 planner 应保持等待，不得自行运动。在第五个观察终端检查并补存 planner 参数：

```bash
source /opt/ros/noetic/setup.bash
source "${HOME}/scout_ws/devel/setup.bash"
export RUN_ENV=/tmp/S1_E2_H1_C1_B0_b01_r01.env  # 改成本次 RUN_LABEL
source "${RUN_ENV}"

rostopic echo -n 1 /spmpc/solver_backend
rostopic echo -n 1 /spmpc/controller_variant
rostopic echo -n 1 /spmpc/debug/effective_config \
  > "${OUT_DIR}/${NAME}_effective_config_before_start.txt"
rosparam get "/spmpc_local_planner/variants/${VARIANT}/w_slosh" \
  | tee "${OUT_DIR}/${NAME}_actual_w_slosh.txt"
rosparam get /spmpc_local_planner \
  > "${OUT_DIR}/${NAME}_planner_rosparam_before_start.yaml"
rostopic info /cmd_vel
```

`actual_w_slosh` 必须与本次 `W_SLOSH` 完全一致；`/cmd_vel` 必须只有本次 S-MPCC planner 一个有效控制 publisher。

---

## 10. 正式多终端开始运动

planner 和 recorder 均正常后：

1. 再次调用 `rosservice call /slosh/reset`；
2. 确认液面连续至少 2 s 低于冻结静止阈值；
3. 确认安全员、急停和走廊；
4. 回到终端 B，按 Enter；
5. 等待起点门控输出 `Start pose aligned`，随后路径开始发布，车辆开始运动。

运动期间不修改参数、不切换节点、不手动重发路径。发生 safety/solver/tracking failure 时安全停止，但保留本次 bag 作为方法失败。

---

## 11. 正式多终端到达与停止顺序

到达、方法失败或安全停止后：

1. 保持 recorder 继续运行 5 s；
2. Ctrl+C 停止终端 C recorder，使 bag 正常闭合；
3. Ctrl+C 停止终端 D planner；
4. Ctrl+C 停止终端 B path replay；
5. 需要时发布一次零速度；
6. 同一容器继续下一 trial 时保留 monitor，但下一次必须再次 `/slosh/reset`。

零速度命令：

```bash
rostopic pub -1 /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

---

## 12. 每次 run 的即时验收

### 12.1 一键 PF pilot

一键终端结束时会打印 `bag/meta dir`、run meta 和三个 log 路径。按本次命令填写：

```bash
export CHECK_DIR=/home/geist/slosh_bags/real/20260718_spmpc_parameter_pilot/PF/WS/H0_C1/W2
export NAME=PF_WS_H0_C1_W2_b01_r01
export BAG="${CHECK_DIR}/${NAME}.bag"

test -s "${BAG}"
test -s "${CHECK_DIR}/${NAME}_one_click_meta.env"
test -s "${CHECK_DIR}/${NAME}_info.txt"
test -s "${CHECK_DIR}/${NAME}_recorded_topics.txt"

rg '^(variant|pilot_mode|pilot_method|path_source_mode|path_file|start_pos_tol|start_yaw_tol|v_ref|w_slosh|record_rgb)=' \
  "${CHECK_DIR}/${NAME}_one_click_meta.env"

rg '^(run_class|pilot_mode|pilot_method|path_source_mode|path_file|start_pos_tol|start_yaw_tol|v_ref|w_slosh|record_rgb|record_camera)=' \
  "${CHECK_DIR}/${NAME}_info.txt"

rosbag info "${BAG}" | tee "${CHECK_DIR}/${NAME}_manual_bag_info.txt"
```

两份 metadata 必须一致满足：`pilot_mode=true`、候选方法/权重正确、`path_source_mode=replay`、路径为冻结 H0、`record_rgb=false`、`record_camera=false`。

若 run 失败，先查看：

```bash
tail -n 80 "${CHECK_DIR}/${NAME}_path_generator.log"
tail -n 80 "${CHECK_DIR}/${NAME}_recorder.log"
tail -n 80 "${CHECK_DIR}/${NAME}_planner.log"
```

### 12.2 正式多终端 run

```bash
export RUN_ENV=/tmp/S1_E2_H1_C1_B0_b01_r01.env  # 改成本次 RUN_LABEL
source "${RUN_ENV}"
export BAG="${OUT_DIR}/${NAME}.bag"
export CHECK_DIR="${OUT_DIR}"
export REQUIRE_RGB=true

test -s "${BAG}"
rosbag info "${BAG}" | tee "${CHECK_DIR}/${NAME}_manual_bag_info.txt"
```

### 12.3 核对核心话题

PF 和正式 run 的公共必录话题：

```bash
for topic in \
  /cmd_vel \
  /odom \
  /tf \
  /scout/global_path_fixed \
  /spmpc/status \
  /spmpc/solver_backend \
  /spmpc/controller_variant \
  /spmpc/slosh_height \
  /spmpc/slosh_horizon_summary \
  /spmpc/debug/effective_config \
  /spmpc/debug/predicted_horizon \
  /spmpc/debug/pre_solve_snapshot; do
  grep -Fxq "${topic}" "${CHECK_DIR}/${NAME}_recorded_topics.txt" \
    || echo "MISSING_REQUIRED_TOPIC ${topic}"
done
```

正式 run 额外核对 RGB；PF 必须确认这两个话题没有被录入：

```bash
if [[ "${REQUIRE_RGB:-false}" == "true" ]]; then
  for topic in /camera/color/image_raw /camera/color/camera_info; do
    grep -Fxq "${topic}" "${CHECK_DIR}/${NAME}_recorded_topics.txt" \
      || echo "MISSING_REQUIRED_RGB_TOPIC ${topic}"
  done
else
  if grep -Eq '^/camera/color/(image_raw|image_raw/compressed)$' \
      "${CHECK_DIR}/${NAME}_recorded_topics.txt"; then
    echo "UNEXPECTED_RGB_TOPIC_IN_PILOT"
  fi
fi
```

### 12.4 检查 replay 消息

```bash
rostopic echo -b "${BAG}" -n 1 /spmpc/debug/predicted_horizon \
  | rg 'valid:|backend:|variant:|horizon_steps:'

rostopic echo -b "${BAG}" -n 1 /spmpc/debug/pre_solve_snapshot \
  | rg 'valid:|backend:|variant:|primal_guess_only:|horizon_steps:|state_width:|control_width:|parameter_width:'
```

预期：

```text
valid=true
backend=continuous_mpcc_acados
horizon_steps=60
预测状态/控制=61/60
state_width=10
control_width=3
B0/B_smooth parameter_width=23
B_slosh parameter_width=32
primal_guess_only=true
```

正式 run 若缺少 RGB、odom、TF、冻结路径或两个 replay 话题，本次按矩阵文件第 12 节判断为采集故障并保留原始文件；PF 内部模型筛选不要求 RGB，但必须包含 odom、TF、冻结路径、内部 slosh 状态/高度、完整 horizon 和两个 replay 话题。不得删除后静默重跑。若 solver/tracking/safety 本身失败，则作为方法失败进入 success-rate 分母。

---

## 13. 下一次 trial

一键 PF pilot：

1. 等脚本正常结束，确认 planner、recorder、path source 已清理且 `/cmd_vel` 为零；
2. 按矩阵顺序修改 `METHOD/PILOT_METHOD`、`BLOCK`、`RUN_LABEL`、`RUN_OUT_DIR` 和 `OPERATOR_NOTE`；
3. `PATH_SOURCE_MODE` 始终保持 `replay`，`PATH_FILE` 始终指向同一冻结 H0；
4. 遥控回起点并对齐航向；若启动 standalone monitor，执行 `/slosh/reset`；
5. 液体静稳后再运行下一条一键命令；脚本自动等待起点门控，不需要按 Enter；
6. 每个 run 生成新的独立 bag、metadata 和 log；
7. 不根据已看到的内部模型排序改变后续顺序或临时追加权重。

正式多终端 trial：

1. 按矩阵随机顺序修改 `METHOD` 和 `BLOCK`；
2. 重新执行第 5 节，生成新的 `RUN_LABEL` 和 `RUN_ENV`；
3. 每个 trial 重新启动 planner；
4. 路径仍从同一冻结 JSON replay；
5. monitor reset，液体重新静稳；
6. recorder 产生新的独立 bag；
7. 不根据已看到的 RGB 排名修改后续顺序、权重或 `v_ref`。
