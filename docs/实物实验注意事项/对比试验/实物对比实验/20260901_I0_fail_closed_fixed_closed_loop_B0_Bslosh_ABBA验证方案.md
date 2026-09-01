# processed-IMU I0 + fail_closed + fixed_closed_loop 的 B0/Bslosh-short100 ABBA 实物验证方案

日期：2026-09-01

协议 ID：`SMPCC_I0_FAILCLOSED_FIXED_SHORT100_ABBA_DEV_V2`

状态：`CODE-AUDITED / HARDWARE-UNVERIFIED / WORKTREE-IMPLEMENTED / UNCOMMITTED`

## Material Passport

- Origin Skill: experiment-agent
- Origin Mode: plan
- Origin Date: 2026-09-01
- Verification Status: CODE-AUDITED / HARDWARE-UNVERIFIED
- Source Status: WORKTREE-IMPLEMENTED / UNCOMMITTED
- Version Label: i0_failclosed_fixed_short100_abba_dev_v2

## 0. 一页结论

本实验只回答一个问题：

> 在同日、同路径、同起点和同一速度配置下，启用现有 legacy `fixed_closed_loop` 后，仅在 solver horizon 的 state stages `0..3`（`dt≈33.3 ms`，即 `0..100 ms`）计入液体代价的 `B_slosh_short100`，其 RGB 液面晃动是否优于 `B0`？

冻结顺序：

```text
Block 1: Row01 B0              -> Row02 Bslosh-short100
Block 2: Row03 Bslosh-short100 -> Row04 B0
```

核心配置：

```text
observer source          = processed_imu
observer fallback        = fail_closed
require_common_epoch     = true
delay_phase_mode         = fixed_closed_loop
configured delay         = linear 0.15 s / angular 0.22 s
v_ref                    = 0.20 m/s
v_safe_max               = 0.25 m/s
B0                       = slosh_enable=false, w_slosh=0
Bslosh-short100          = slosh_enable=true, w_slosh=5
robot OCP horizon        = N=60, dt≈33.3 ms, total≈2.0 s（不缩短）
liquid cost state stages = 0..3（四个 state nodes，t=0..100 ms）
liquid cost stages 4..60 = w_slosh_eta=w_slosh_eta_dot=0
```

这里的 `0..100 ms` 从 **legacy rollout 后的 L22 solver 初始状态** 起算；不是从原始 IMU 样本起算，也不是命令发布后的前 `100 ms`。液体动力学仍传播到 stage 60，只有液体代价在 stage 3 后关闭；机器人状态、路径、终点、tracking、control 和 smooth 代价仍使用完整约 `2 s` 时域。

其中 stage 0 是固定的 solver 初始状态，其液体代价对同一次 solve 的候选排序是常数；真正随控制量变化的是预测 state stages `1..3`（约 `33.3/66.7/100 ms`）。因此这里的含义是“只信任并惩罚到未来约 `100 ms`”，不是把整套 MPC 缩成 `100 ms`。

这是一项 **development 系统级快速筛查**，不是正式论文 efficacy 结论，也不是新 FOPDT 执行器模型实验。当前底层实现实际统一回放约 `max(0.15, 0.22)=0.22 s` 的历史 `/cmd_vel`；线/角 `delay/tau/gain` 候选没有进入控制。

当前代码准备状态：

```text
静态/纯逻辑回归 = 以本次工作区实现的最终验证记录为准；阶段提交前新增回归均须 PASS
实车运行         = 尚未执行
源码状态         = 工作区已实现；阶段提交、实车机同步和重编译尚未完成
```

正式运动前必须把包含本方案、runner 和证据工具的阶段提交同步到实车机、重新编译，并保证 `git status --short` 无输出。专用 runner 会拒绝带未提交 runtime/evidence 修改的实车运动；v2 还会在发路径前核对 latched `/spmpc/controller_variant`，旧二进制将新名字回退为 `B0` 时不会放车。

## 1. 研究问题、假设和结论边界

### 1.1 唯一研究问题

```text
RQ: 在冻结的 processed-I0 + fail_closed + legacy fixed_closed_loop 系统中，
    Bslosh-short100 的实物 RGB H_vis 是否低于 literal B0？
```

### 1.2 预先冻结的 development-positive 假设

定义：

```text
DeltaP95 = P95(H_vis, B0) - P95(H_vis, Bslosh-short100)
DeltaRMS = RMS(H_vis, B0) - RMS(H_vis, Bslosh-short100)
```

正值表示 Bslosh-short100 更好。

Block 1 快速筛查要求：

```text
DeltaP95 >= +0.05 mm
DeltaRMS >= 0
```

完整 ABBA 的 `DEVELOPMENT_POSITIVE` 还要求：

1. 两个 block 的 `DeltaP95` 都大于 `0`；
2. 两个 block 的平均 `DeltaP95 >= +0.05 mm`；
3. 两个 block 的平均 `DeltaRMS >= 0`；
4. Block 1 已按预注册规则晋级；
5. `median(T_goal,Bslosh-short100 / T_goal,B0) <= 1.05`，不能主要靠减速换取低 RGB；
6. 四包的 observer、fixed、求解、跟踪、安全和 RGB 质量门全部通过。

`0.05 mm` 仅保留为与 v1 一致的 development 快速筛查门，避免在改变液体代价窗口的同时又改变早停规则。由于旧数据主要量化梯级约为 `0.076923 mm`，`0.05 <= DeltaP95 < 0.10 mm` 只能称为方向性信号；只有 `>=0.10 mm` 且跨 block 一致时，才可称为达到当前测量分辨率的改善。自动输出的 `DEVELOPMENT_POSITIVE` 仍不等于正式 efficacy 结论。

### 1.3 能回答与不能回答的内容

能回答：

> 这套完整 development Bslosh-short100 系统是否在实物 RGB 指标上超过 B0。

不能单独回答：

- 延迟是否是仿真正向、实物不正向的唯一根因；
- processed-IMU、fail-closed、slosh cost 和 legacy rollout 中哪一项单独贡献效果；
- `0.15/0.22 s` 是否是真实的线/角执行器参数；
- 新 FOPDT 执行器模型是否有效；
- literal B0/Bslosh-short100 是否构成严格只差 slosh cost 的 matched 因果比较；B0 仍是 6D solver，treatment 是 10D solver。
- `100 ms` 是否是最优或已由实物验证的可信边界；它只是 evidence-informed development cutoff。

## 2. 与前序提交和方案的关系

### 2.1 v1 历史身份与本次 v2

上一版完整协议冻结在提交：

```text
41f0937831aab2edfb82d579c6ed853b7919556b
SMPCC_I0_FAILCLOSED_FIXED_ABBA_DEV_V1
treatment = 历史 B_slosh，全约 2 s horizon 都计入液体代价
hardware  = 未执行
```

v1 没有实物结果，不撤回也不覆盖；它保留为设计演进证据。当前 v2 以独立 protocol、variant、runner、输出目录、stem、prereg/order、postflight suffix、analysis report type 和 session marker 取代 v1 的待执行入口。v1/v2 的任何 bag、report 或 PASS marker 都不得互相满足前序门，也不得池化分析。

### 2.2 `03bcfee` 的边界

`03bcfee846042412cccc03d8c4996660a9f41dba` 搭建的是：

```text
processed-I0 + fail_closed + common_epoch
matched0/matched5
delay_phase=shadow
I0 applied；I1/L22 shadow
```

其中候选执行器模型只写 metadata：

```text
legacy_delay_applied=false
actuator_candidate_applied=false
```

它没有实现新执行器模型，也没有改变既有控制器默认行为。本次实验不撤回 `03bcfee`，但使用的是当前工作区新增、尚未阶段提交的独立 B0/Bslosh-short100 ABBA profile。

### 2.3 与未来正式方案的关系

《共同状态时刻与执行器延迟模型实施思路》主张未来使用 MPC 内显式执行器响应模型，并让旧 `delay_phase` 保持 shadow/off。本实验是在投入该实现前，对现有 legacy 能力做一次有停止规则的快速筛查：

```text
当前 legacy 组合正向 -> 完成 ABBA，记录系统级 development 结果
当前 legacy 组合有效负向 -> 停止微调，转入新执行器模型实施
```

本次不修改 `0.15/0.22 s`，因为现在只想检验“将液体代价限制在已有证据支持的短前缀”这一项变化。如果同时改延迟，就无法判断正/负结果来自 cost window 还是延迟参数。前面的延迟实验并未白做：它决定了本轮继续保留 fixed compensation，并为后续共同状态时刻与执行器模型提供了辨识范围；只是本轮不把渐进响应时间常数误当成纯延迟叠加进去。

## 3. A/B 配置与真实数据链

### 3.1 冻结配置

| 项目 | B0 | Bslosh-short100 |
|---|---:|---:|
| variant | `B0` | `B_slosh_short100` |
| solver state | 6D robot | 10D robot + liquid |
| `slosh_enable` | false | true |
| `w_slosh` | 0 | 5 |
| smooth priority | false | false |
| hard slosh constraint | false | false |
| observer source | processed-IMU（诊断） | processed-IMU（求解输入来源） |
| observer policy | fail_closed（后验健康门） | fail_closed（失效时停车） |
| delay mode | fixed_closed_loop | fixed_closed_loop |
| `v_ref / v_safe_max` | `0.20 / 0.25 m/s` | `0.20 / 0.25 m/s` |
| robot horizon | `N=60, ≈2 s` | `N=60, ≈2 s` |
| liquid cost window | N/A | state stages `0..3` |
| stage `4..60` liquid weight | N/A | `0` |

`B_slosh_short100` 逐字段复制历史 literal `B_slosh`，唯一新增的内生配置是 `slosh_cost_horizon_steps=3`、`slosh_cost_tail_discount=0`。历史 `B_slosh` 不修改，因此 v1 和旧 bag 的含义不变。

不用 `B_slosh_matched0/matched5`：它们的 smooth/control 权重不同，且当前 C++ 明确禁止 matched development variant 与 `fixed_closed_loop` 组合，启动时会 fail-fast。本实验不拆除这个保护。

### 3.2 Bslosh-short100 的实际状态链

```text
/imu/data
  -> processed-IMU 管线
  -> I0 liquid observer state
  -> fail_closed source selector
  -> robot state 对齐到 I0 state stamp
  -> legacy fixed_closed_loop 历史命令 rollout
  -> final L22 predicted robot + liquid
  -> 以 L22 为 solver stage 0
  -> B_slosh_short100 10D solver
       stages 0..3: w_slosh_eta=5, w_slosh_eta_dot=1.5
       stages 4..60: 两项液体代价权重均为 0
       liquid dynamics 仍传播到 stage 60
```

因此 I0 是 observer 起点，最终进入 solver 的是 legacy rollout 后的 L22，而不是未变换的 I0。short100 的 `0..100 ms` 相对这个 L22 solver epoch 定义，不能与 `0.15/0.22 s` 补偿窗相加，也不能写成“从 IMU 原始采样开始预测 100 ms”。

### 3.3 B0 的实际状态链

```text
latest robot state
  -> legacy fixed_closed_loop robot rollout
  -> B0 6D solver
```

B0 不消费液体状态，short100 对它为 N/A，也不因 observer fail_closed 自动停车。processed-IMU 对 B0 主要是诊断和后验有效性门；若 B0 包中 IMU stale/reset，该包仍必须由 postflight 判无效，不能进入配对比较。

## 4. 冻结资产与输出

### 4.1 路径、地图和 RGB 标定

| 资产 | 冻结路径 | SHA-256 |
|---|---|---|
| C02 路径 | `/home/geist/fixed_paths/real/20260829_spmpc_mocap_execution_chain/candidates/mocap_compact_s_C02.json` | `1464ef37857bcb899d8b0e4867ff63ea06f017e1b871bed80e077f450be14164` |
| 动捕场地地图 | `/home/geist/scout_maps/real/20260829_mocap_exec/map_carto_20260829_mocap_exec_v1.pbstream` | `34e45fd8205a766dbc6e3dcea667c5a0a618e26b331d48351c25645e31a19595` |
| RGB calibration | `/home/geist/slosh_bags/real/20260731_spmpc_g2s_source_selection/calibration/red_3ruler_g2s_20260731_relabel_frozen_v2.yaml` | `7186b4bda05a1b73c19fd97b3a34b08a82bfab0df52272eaf2829115de049d01` |

定位环境文件：

```text
/home/geist/scout_maps/real/20260829_mocap_exec/map_carto_20260829_mocap_exec_v1_freeze.env
```

### 4.2 运行入口和默认输出

```text
runner = src/scout_apps/control/spmpc_local_planner/scripts/
         run_spmpc_i0_failclosed_fixed_short100_abba_trial.sh

output root = /home/geist/slosh_bags/real/
              ${DATE}_spmpc_i0_failclosed_fixed_short100_abba_v2/H0
```

该入口只是选择 `short100_v2` 的薄 wrapper；运动门、采集、postflight 和 ABBA 状态机由共享 engine 执行。历史 v1 wrapper 仍选择 `legacy_v1`，二者的冻结身份集中在同一个 profile 模块中，避免复制约 500 行 runner。

v2 必须使用 direct-variant 选择：B0 行直接传 `VARIANT=B0 / ALG=B0`，treatment 行直接传 `VARIANT=B_slosh_short100 / ALG=B_slosh_short100`，且 `PILOT_METHOD` 为空。**禁止使用历史 `run_spmpc_i0_failclosed_fixed_abba_trial.sh`，也禁止以 `PILOT_METHOD=W5` 执行 v2**；旧 W5 映射的是全 horizon 历史 `B_slosh`，不是 `B_slosh_short100`。v2 profile 同时显式钉死 solver backend、命令 topic、weight override 开关、`alpha_max` 和关键 observer/nowcast/state-timing 容差，避免旧 shell `export` 改变本次唯一变量。

默认文件身份：

| Row | 条件 | 默认 stem |
|---|---|---|
| 01 | B0 | `DEV_I0FC_FIXED_S100_V2_01_B0_b01_p01_a01` |
| 02 | Bslosh-short100 | `DEV_I0FC_FIXED_S100_V2_02_BsloshS100_b01_p02_a01` |
| 03 | Bslosh-short100 | `DEV_I0FC_FIXED_S100_V2_03_BsloshS100_b02_p01_a01` |
| 04 | B0 | `DEV_I0FC_FIXED_S100_V2_04_B0_b02_p02_a01` |

公共分析输出：

```text
I0_FAILCLOSED_FIXED_SHORT100_ABBA_RGB_ANALYSIS.json
SMPCC_I0_FAILCLOSED_FIXED_SHORT100_ABBA_DEV_V2_prereg.env
SMPCC_I0_FAILCLOSED_FIXED_SHORT100_ABBA_DEV_V2_order.csv
SMPCC_I0_FAILCLOSED_FIXED_SHORT100_ABBA_DEV_V2_session.env
```

每行的 v2 证据后缀冻结为：

| 证据 | v2 专属 suffix |
|---|---|
| exact postflight | `_i0_fixed_short100_v2_postflight.json` |
| observer postflight | `_short100_v2_observer_postflight.json` |
| NOKOV chain postflight | `_short100_v2_mocap_chain_postflight.json` |
| RGB postflight | `_i0_fixed_short100_v2_rgb_postflight.json` |
| unit PASS marker | `_short100_v2_unit_pass.env` |

每行还会生成 summary。Row01 只允许初始化全新 v2 session；Row02～04 必须逐字匹配 prereg、order 和 session marker。发现 v1 身份文件时 fail-closed，不能混用。

### 4.3 图像数据政策

RealSense 原始图像仅供在线 detector 使用。bag 只保存 `/liquid/measurement` 标量和 camera info，禁止 raw/compressed/depth/debug image stream。

因此这些包不能在事后更换 HSV、ROI 或 calibration 重跑图像推理，也不能逐帧复核遮挡、反光或误分割；必须依靠现场冻结几何和在线质量标志。

## 5. 实验前代码冻结与编译

本方案相关源码已在工作区实现，但尚未形成最终实验提交。必须先完成测试、阶段提交，再在实车机同步该提交、重新编译并确认相关路径 clean；此前不允许运动。同步后执行：

```bash
cd /home/geist/scout_ws
source /opt/ros/noetic/setup.bash

catkin_make --force-cmake \
  -DCATKIN_WHITELIST_PACKAGES="slosh_models;spmpc_local_planner;realsense_liquid_measurement" \
  -DCMAKE_BUILD_TYPE=Release

source /home/geist/scout_ws/devel/setup.bash
git rev-parse HEAD
git status --short
```

最后一条必须无输出。记录最终 `HEAD`：

```text
experiment_commit = 实车运行前填写 git rev-parse HEAD；必须包含 short100 v2 实现与本方案
```

runner 只能证明相关源代码 clean，不能自动证明 `devel` 对应当前提交，因此本节的重新编译不能省略。

## 6. 终端 A：底盘、传感器与冻结地图定位

### 6.1 防止重复启动

先检查：

```bash
pgrep -af launch_real_sensors_stack.sh
rosnode list | sort
```

如果整套脚本已经运行，不要重复启动，否则可能重复占用 CAN、LiDAR、Cartographer、IMU 或 RealSense 节点。

### 6.2 启动命令

在终端 A 执行用户冻结的命令：

```bash
cd /home/geist/scout_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
source /home/geist/scout_maps/real/20260829_mocap_exec/map_carto_20260829_mocap_exec_v1_freeze.env

: "${LOCALIZATION_MAP_FILE:?freeze env must set LOCALIZATION_MAP_FILE}"
: "${LOCALIZATION_MAP_EXPECTED_SHA256:?freeze env must set map SHA-256}"
test -s "${LOCALIZATION_MAP_FILE}"

LOG_DIR=/tmp/mocap_localization_$(date +%Y%m%d_%H%M%S) \
START_LOCALIZATION=true START_REALSENSE=true \
  bash src/scout_apps/control/scout_local_planner/scripts/launch_real_sensors_stack.sh
```

该终端会启动并持有：

```text
Scout base + /odom
front LiDAR + /scan_front
Cartographer frozen-map localization + /map + map->base_link
IMU + /imu/data
RealSense color + /camera/color/image_raw + camera_info
```

显式地图模式会校验 pbstream SHA，并把 `/map` 和 `map -> base_link` 等待升级为致命门。保持终端 A 运行到四行实验全部结束；关闭或 `Ctrl+C` 会停止它负责的整套进程。

不要单独提前启动 `/liquid/measurement` 在线液面节点；ABBA runner 每行会检查 publisher 唯一性并自行启动、停止该节点。

## 7. 终端 B：NOKOV/VRPN 监测

### 7.1 网络只读检查

```bash
ip route get 192.168.203.85
ping -c 3 192.168.203.85
```

若现场禁止 ICMP，以后续原始 pose 和 `/mocap/status` 为最终硬门。

### 7.2 启动命令

另开终端 B：

```bash
cd /home/geist/scout_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch nokov_mocap_monitor nokov_monitor.launch \
  server:=192.168.203.85 \
  vrpn_port:=3883 \
  tracker:=Tracker0 \
  publish_tf:=false \
  use_rviz:=false
```

该 launch 同时启动 VRPN client 和 `/mocap/*` 监测桥。NOKOV 只作为记录和离线真值，不替换 `/odom`，不接管控制 TF，也不参与 `/cmd_vel`。

保持终端 B 运行到实验结束。

## 8. 终端 C：联合就绪检查

另开终端 C：

```bash
cd /home/geist/scout_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

### 8.1 Topic、地图和 NOKOV

```bash
rostopic echo --noarr -n 1 /map
rostopic echo --noarr -n 1 /scan_front
rostopic echo --noarr -n 1 /odom
rostopic echo --noarr -n 1 /imu/data
rostopic echo --noarr -n 1 /camera/color/image_raw
rostopic echo -n 1 /camera/color/camera_info

rostopic echo -n 1 /vrpn_client_node/Tracker0/pose
rostopic echo -n 1 /mocap/status

rosparam get /cartographer_node/frozen_map_file
rosparam get /cartographer_node/frozen_map_expected_sha256
```

预期：

```text
/mocap/status 包含 OK 和 tracker=Tracker0
runtime map 指向冻结 pbstream
runtime map SHA = 34e45fd8205a766dbc6e3dcea667c5a0a618e26b331d48351c25645e31a19595
camera_info = 1920 x 1080
```

### 8.2 唯一命令发布者

实验开始前不得有 teleop、旧 planner、path publisher 或其他 `/cmd_vel` publisher：

```bash
rostopic info /cmd_vel
```

预期 `Publishers: None`。专用 runner 在真正启动 planner 前还会再做一次机器检查。

### 8.3 现场物理检查

- 急停可用，操作者全程在急停旁；
- C02 路径及停车延伸区无人、无临时障碍和线缆；
- 容器、液量、标尺、相机支架和 Marker 固定；
- RealSense 视野、曝光、白平衡、ROI、HSV/calibration 没有改变；
- 小车放在冻结起点，位置和航向满足 start gate；
- 没有建图节点与定位节点同时运行；
- 没有手工启动在线液面 detector；
- 接受本次新的 `v_ref=0.20 / v_safe_max=0.25 m/s` development profile。

## 9. Validate-only：先检查，不动车

在终端 C 固定本次日期、输出目录和 runner。若跨午夜，四行仍保持同一个 `I0FC_DATE`：

```bash
export I0FC_DATE=20260901
export I0FC_OUT=/home/geist/slosh_bags/real/${I0FC_DATE}_spmpc_i0_failclosed_fixed_short100_abba_v2/H0
export I0FC_RUNNER=src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_i0_failclosed_fixed_short100_abba_trial.sh
```

先验证两个 variant：

```bash
PAIR_ROW=01 DATE="${I0FC_DATE}" RUN_OUT_DIR="${I0FC_OUT}" \
VALIDATE_ONLY=true bash "${I0FC_RUNNER}"

PAIR_ROW=02 DATE="${I0FC_DATE}" RUN_OUT_DIR="${I0FC_OUT}" \
VALIDATE_ONLY=true bash "${I0FC_RUNNER}"
```

`validate-only PASS` 只证明：

- 冻结路径/map/RGB 文件和 SHA 可读；
- launch 参数展开符合协议；
- treatment 解析为 `B_slosh_short100`，且 launch namespace 为 `steps=3/tail=0`；
- 静态合同测试和相机准备检查通过。

它在 dirty gate、上一行 PASS marker 和实车 runtime topic/map/唯一 `/cmd_vel` publisher 检查之前退出，因此不能替代第 5～8 节。

## 10. 正式 ABBA 执行

不要使用循环，也不要用 `&&` 连续执行多行。每行退出并完成 postflight 后，人工回位、静置、复核，再单独输入下一条命令。

### 10.1 Row01：B0 基线和新速度 profile 安全门

```bash
PAIR_ROW=01 DATE="${I0FC_DATE}" RUN_OUT_DIR="${I0FC_OUT}" \
VALIDATE_ONLY=false \
ARM_MOTION=YES CONFIRM_RGB_GEOMETRY=YES CONFIRM_NEW_SPEED_PROFILE=YES \
bash "${I0FC_RUNNER}"
```

Row01 同时承担 `.20/.25` profile 的集成运动门。以下任一情况发生都停止，不启动 Row02：

- 未到点、明显偏离路径或航向异常；
- safety/tracking gate、速度锁存或非终端限幅介入；
- observer、fixed application、warm-start、RGB 或 NOKOV postflight 失败；
- 操作者认为制动距离、容器稳定性或路径净空不安全。

### 10.2 Row02：Bslosh-short100 快速效果屏

Row01 `_short100_v2_unit_pass.env` 存在、车已回位且液体重新静置后：

```bash
PAIR_ROW=02 DATE="${I0FC_DATE}" RUN_OUT_DIR="${I0FC_OUT}" \
VALIDATE_ONLY=false \
ARM_MOTION=YES CONFIRM_RGB_GEOMETRY=YES CONFIRM_NEW_SPEED_PROFILE=YES \
bash "${I0FC_RUNNER}"
row02_rc=$?
printf 'Row02 rc=%s\n' "${row02_rc}"
```

Row02 自动分析 Row01/02：

| 返回码/决策 | 含义 | 下一步 |
|---|---|---|
| `0 / PROMOTE_BLOCK2` | Block 1 达到预注册 RGB 门 | 才允许 Row03 |
| `10 / STOP_BLOCK1_FUTILITY` | 包有效，但当前组合没有达到快速效果门 | 停止，不选择性重录 |
| `2` 或其他失败 | 证据无效、依赖或采集链失败 | 保留证据，排查；不能解释为方法负向 |

### 10.3 Row03：反序 Bslosh-short100

只有 Row02 返回 `0` 且分析 JSON 精确为 `PASS / PROMOTE_BLOCK2`：

```bash
PAIR_ROW=03 DATE="${I0FC_DATE}" RUN_OUT_DIR="${I0FC_OUT}" \
VALIDATE_ONLY=false \
ARM_MOTION=YES CONFIRM_RGB_GEOMETRY=YES CONFIRM_NEW_SPEED_PROFILE=YES \
bash "${I0FC_RUNNER}"
```

runner 会机器拒绝绕过 Block 1 晋级门。

### 10.4 Row04：反序 B0 和最终裁决

```bash
PAIR_ROW=04 DATE="${I0FC_DATE}" RUN_OUT_DIR="${I0FC_OUT}" \
VALIDATE_ONLY=false \
ARM_MOTION=YES CONFIRM_RGB_GEOMETRY=YES CONFIRM_NEW_SPEED_PROFILE=YES \
bash "${I0FC_RUNNER}"
row04_rc=$?
printf 'Row04 rc=%s\n' "${row04_rc}"
```

Row04 自动输出以下三种之一：

| 决策 | 返回码 | 含义 |
|---|---:|---|
| `DEVELOPMENT_POSITIVE` | 0 | RGB 门和 goal-time slowdown 门全部通过 |
| `RGB_POSITIVE_SLOWDOWN_CONFOUNDED` | 10 | RGB 正向，但 Bslosh-short100 相对 B0 慢超过 5% |
| `NO_DEVELOPMENT_POSITIVE` | 10 | 完整 ABBA 未达到冻结效果门 |

返回码 `10` 在这里是有效停止裁决，不等同于脚本或采集失败。

## 11. 每行之间的复位合同

每行结束后严格执行：

```text
确认 runner 已退出且小车零速
-> 检查本行 _short100_v2_unit_pass.env 和 postflight
-> 保留 bag/report/log，不删除失败证据
-> 遥控或人工回到同一冻结起点和航向
-> 从最后一次非零命令起静置至少 60 s
-> 检查液面、液量、容器、相机、标尺和 Marker 未变化
-> 确认 /cmd_vel 再次无 publisher
-> 执行下一行
```

每行 runner 都会重新启动 online RGB、用 `30` 个 valid frame 锁零，并在 start gate、IMU READY、运行时 variant 身份和唯一命令发布者通过后才释放路径。

## 12. 单包硬有效性门

### 12.1 processed-IMU 与 observer

运动窗口内：

```text
processed-IMU READY/valid/fresh = 100%
nominal/effective source       = processed_imu
fallback policy                = fail_closed
fallback active/latched        = 0
IMU reset_epoch                = 不变
zero_due_to_waiting_observer   = 0
```

B0 也必须通过这些后验门。

### 12.2 fixed closed-loop

```text
delay mode                            = fixed_closed_loop
configured delay                      = 0.15 / 0.22 s
history_complete                      = 100%
shadow_valid                          = 100%
fixed_closed_loop_configured          = 100%
fixed_closed_loop_applied             = 100%
robot_delay_compensation_applied      = 100%
liquid_delay_compensation_applied     = 100%
```

任一运动周期为 false 或字段不可读都判无效，不能放过少量 fixed/raw 混用。B0 的 liquid flag 只是 composer 诊断，6D solver 仍不消费液体。

### 12.3 short100 solver artifact 合同

v2 四行都必须验证运动窗口内全部已记录的 `effective_config`，并逐个 `cycle_id` 核对每个 active solve 的 `PreSolveSnapshot` 和 `PredictedHorizon`，不能只看最后一条 rosparam/config。`effective_config` 消息没有 `cycle_id`，因此不宣称它与 audit cycle 一一关联；逐周期一一关联只用于带 `cycle_id` 的 solver artifacts。

共同要求：

```text
robot horizon_steps = 60
dt                  ≈ 1/30 s
robot horizon       ≈ 2.0 s
motion audit cycle  = 严格递增、无重复、相邻 cycle_id 差 1
snapshot/horizon    = 每个 solve cycle 恰好一份 valid artifact
variant             = 与本行冻结值一致
resolved weights    = w_smooth/w_alpha/w_du_a/w_du_vs=0.1
resolved limits     = slosh_height_max=0.001 m, alpha_max=1.2
```

v2 runner 会把上述 resolved 值作为显式 launch 参数传入；不再用 `-1` sentinel 依赖 YAML/C++ fallback。preflight 展开、实际 runner 环境和 exact postflight 都来自同一份 `short100_v2` profile，避免预注册值与真实求解器值出现两套解释。

Bslosh-short100 额外要求：

```text
slosh_cost_horizon_steps = 3
slosh_cost_horizon_sec   = 0.1 s
slosh_cost_tail_discount = 0
stage 0..3:
  w_slosh_eta             = 5.0
  w_slosh_eta_dot         = 1.5（5.0 * eta_dot_ratio 0.3）
stage 4..60:
  w_slosh_eta             = 0
  w_slosh_eta_dot         = 0
```

B0 仍检查 variant、完整 robot horizon 和历史 `steps=-1/tail=1` 元数据，但因 6D solver 没有液体参数列，不要求 `w_slosh_eta/*dot`。

### 12.4 solver、warm-start、命令和安全

```text
GOAL_REACHED                              = true
solver failure                            = 0
warm-start fallback                       = 0
command-contract violation                = 0
nonterminal safety/tracking intervention  = 0
speed-safety violation/latch              = 0
|published v|                             <= 0.2501 m/s
normal-motion post-solver limiter change  = 0
```

### 12.5 RGB 初始状态和主窗口

```text
30-frame zero window P95-P05                 <= 0.25 mm
运动前最后连续 5 s clean coverage             >= 98%
该 5 s corrected H_vis P95                    <= 0.25 mm
该 5 s |signed corrected height| P95           <= 0.25 mm
该 5 s signed 前后半窗 median 漂移            <= 0.05 mm
motion+tail RGB clean coverage                >= 98%
```

这些数值是运动后的 postflight qualification：能阻止坏包进入 A/B 结论，但实时发车门只检查 zero-lock 和 clean 消息，因此仍可能在事后判定该行无效。

## 13. RGB 指标和自动分析

### 13.1 冻结处理

**RGB `H_vis` 是本实验唯一的液面效果裁决量**：`P95(H_vis)` 为主指标，`RMS(H_vis)` 为次指标。solver 内部的 slosh state/height、stage cost、snapshot 和 predicted horizon 只用于确认 short100 确实生效、解释机制和排查失败，不得用它们代替 RGB 宣称“减晃有效”。`T_goal`、tracking 和 NOKOV 只是慢速/执行链混杂门，不是第二套液面效果裁决。

```text
H_vis  = max(0, height_max_lcr_mm)
stamp  = image source header.stamp
filter = 5-point causal rolling median
window = first effective motion -> first GOAL_REACHED + 5 s tail
```

主指标为 `P95(H_vis)`，次指标为 `RMS(H_vis)`；peak 仅作描述性审计，不参与自动效果裁决。不能看完结果后逐包调 lag、scale、HSV、zero 或 calibration。

### 13.2 slowdown 门

当前自动门使用：

```text
T_goal = motion start -> first GOAL_REACHED
```

完整 ABBA 若：

```text
median(T_goal,Bslosh-short100 / T_goal,B0) > 1.05
```

即使 RGB 效果门通过，也只能输出 `RGB_POSITIVE_SLOWDOWN_CONFOUNDED`。progress `T10-90`、实际 `v/omega` 分布和 NOKOV tracking 作为补充审计，不冒充当前自动门。

## 14. 失败、无效包和停止规则

### 14.1 有效负结果

- Row02 `STOP_BLOCK1_FUTILITY`：立即停止当前组合；
- Row04 `NO_DEVELOPMENT_POSITIVE`：记录完整 ABBA 有效负结果；
- `RGB_POSITIVE_SLOWDOWN_CONFOUNDED`：记录 RGB 方向正向但速度混杂，不写成无条件有效。

不得选择性重录直到得到正向，也不得在看过结果后修改阈值。

### 14.2 无效包

observer reset/fallback、fixed/raw 混用、solver/safety 停车、NOKOV 缺失、RGB 零点或 coverage 失败属于证据无效，不属于方法负向。

如果已经生成任何 bag、report、marker 或日志：

1. 原样保留，不删除、不覆盖；
2. 修复根因；
3. 使用新的完整 `RUN_OUT_DIR` 会话，从 Row01 重新开始 ABBA；
4. 不用同一目录中的选择性单行替换构造结果。

当前协议只允许 `ATTEMPT=01` 身份；不要覆盖 `RUN_LABEL`、`NAME` 或 `ATTEMPT` 绕过证据合同。

### 14.3 紧急停止

任何路径侵入、失控趋势、容器松动、相机/Marker 移位或异常噪声出现时，立即使用物理急停；安全后再终止当前 runner。该行保留为失败证据，不继续下一行。

### 14.4 结果与后续路线的预先映射

| 本协议输出 | 是否推进《共同状态时刻与执行器延迟模型实施思路》 | 固定动作 |
|---|---|---|
| `PROMOTE_BLOCK2` | 暂不裁决 | 只说明 Block 1 晋级；按冻结顺序完成 Row03/04 |
| `DEVELOPMENT_POSITIVE` | 不立即触发，可降低优先级 | 冻结当前系统级正向结果；若后续追求正式公平比较和机制可解释性，仍可进入新模型路线 |
| `STOP_BLOCK1_FUTILITY` | 推进 | Row01/02 证据有效但未过快速门；停止选择性重录和 legacy 微调，转入 Stage A/B |
| `NO_DEVELOPMENT_POSITIVE` | 推进 | 冻结完整 ABBA 有效负结果，转入 Stage A/B |
| `RGB_POSITIVE_SLOWDOWN_CONFOUNDED` | 推进，但先做 speed-matched 审计 | 不宣称无条件正向，也不宣称 FOPDT 必然解决问题 |
| `INVALID_EVIDENCE`、postflight 失败或运行错误 | 不触发，无权裁决 | 修复证据链，换新目录并从 Row01 重做完整 ABBA |

“推进”也不是现场直接改 OCP，而是按路线文档的 `Stage A 契约冻结 -> Stage B 执行器辨识 -> 离线 shadow/replay -> 同步修改 B0/Bslosh OCP -> 无运动回归 -> 小步实物放行` 执行。

## 15. 结果记录表

实验结束后填写：

| Row | 条件 | bag | unit PASS | P95 mm | RMS mm | T_goal s | tracking/NOKOV | 备注 |
|---|---|---|---|---:|---:|---:|---|---|
| 01 | B0 |  |  |  |  |  |  |  |
| 02 | Bslosh-short100 |  |  |  |  |  |  |  |
| 03 | Bslosh-short100 |  |  |  |  |  |  |  |
| 04 | B0 |  |  |  |  |  |  |  |

Block 汇总：

| Block | 顺序 | DeltaP95 mm | DeltaRMS mm | Bslosh-short100/B0 T_goal | 判定 |
|---|---|---:|---:|---:|---|
| 1 | B0 -> Bslosh-short100 |  |  |  |  |
| 2 | Bslosh-short100 -> B0 |  |  |  |  |

最终：

```text
analysis report =
decision        =
exit code       =
experiment HEAD =
operator        =
```

## 16. 解释限制

1. B0 是 6D solver，Bslosh-short100 是 10D solver，且 fail_closed/common-epoch 对两者的控制作用不完全对称；这是系统级比较，不是严格 matched 因果试验。
2. legacy rollout 从已有状态出发，却使用墙钟 `now-0.22 ... now` 的命令窗并将结果标为 `now`，存在 epoch 不严谨、重复前推或错相位风险。
3. fixed predictor 自身失败时会回退 raw，而不是 fail-closed；本协议靠 100% postflight 排除这种包。
4. image-free bag 无法事后逐帧核对视觉误检。
5. 正向只能写成“当前 processed-I0 + fail_closed + legacy L22 + short100 Bslosh development 系统在 RGB 上优于 B0”，不能写成“100 ms 已证明最优”“远期液体代价是旧失败唯一根因”“延迟是唯一根因”或“新执行器模型已验证”。
6. 负向只否定当前冻结的 I0 + legacy L22 组合，不否定所有延迟补偿或未来 MPC 内执行器模型。
7. G3R3 的 short100 提交只提供软件合同和无运动 smoke，不是本次实物效果证据。
8. 本 ABBA 只有 `B0` 和 `B_slosh_short100` 实车臂，没有同批次、同配置的 full-horizon 历史 `B_slosh` 实车臂；因此无论结果正负，都不能写成“short100 优于/劣于历史 Bslosh”。

## 17. 关联记录

- [slosh 状态输入与短时延迟补偿验证方案](20260831_slosh状态输入与短时延迟补偿验证方案.md)
- [slosh 状态输入与短时延迟补偿三包分析](../实物对比试验分析/20260831_slosh状态输入与短时延迟补偿三包分析.md)
- [实物问题解决思路：延迟规划与数据模型](../解决问题的思路/20260825_实物问题解决思路_延迟规划与数据模型.md)
- [延迟问题](../解决问题的思路/20260830_延迟问题.md)
- [共同状态时刻与执行器延迟模型实施思路](../解决问题的思路/20260831_共同状态时刻与执行器延迟模型实施思路.md)
- [0705 B0/Bslosh/Bours 实物验证指南](0705B0_B_slosh_B_ours_SPMPC内部晃动高度实物验证指南.md)
- [0705 fixed-path N3 RGB 模型综合分析](../实物对比试验分析/20260705_fixed_path_N3_RGB_模型综合分析.md)
- [0705/0706 最新 RGB 口径复分析](../实物对比试验分析/20260714_0705_0706按最新RGB口径复分析.md)
- [动捕场地 S-MPCC 执行链辨识实验方案](20260826_动捕场地SMPCC执行链辨识实验方案.md)
- [动捕场地建图定位与双场地切换 SOP](20260828_动捕场地建图定位与双场地切换SOP.md)
- [配合动捕测动作延迟](20260829_配合动捕测动作延迟.md)
- [G3 processed-IMU W5 vs Bsmooth 在线 RGB 实验方案](正式论文实验/20260801_G3_processed-IMU_W5_vs_Bsmooth在线RGB实验方案.md)
- [G3R2 逐预测步 RGB 与 IMU observer 匹配度分析（short100 依据）](../实物对比试验分析/20260802_G3R2逐预测步_RGB与IMU_observer匹配度初筛及完整曲线方案.md)
- [G3R3 短可信时域公平配对实现与实物放行记录](../实物对比试验分析/20260819_G3R3短可信时域公平配对实现与实物放行记录.md)
- [G3R3 短可信时域公平配对实物验证方案](20260819_G3R3短可信时域公平配对实物验证方案.md)
