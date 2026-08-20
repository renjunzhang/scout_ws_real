# spmpc_local_planner 运行与实验脚本

本目录只放 SPMPC 自研 planner 的实物辅助、pilot/smoke、录包和实验编排入口。离线分析已迁到 `../tools/analysis/`，acados codegen 已迁到 `../tools/codegen/acados/`，Python 回归测试位于 `../test/python/`。这些目录均不进入运行时安装空间。除 `run_external_baseline_real_fixed_path_trial.sh` 外，其他脚本都服务于 `spmpc_local_planner` 本身。

实物运动前的 RealSense 时间戳放行门现为 C++ executable
`spmpc_realsense_timestamp_health_gate`。G3R2 runner 直接执行构建产物并把二进制
SHA-256 绑定到 prereg/runtime bundle。历史 Python 实现在
`../tools/legacy/validate_realsense_timestamp_health.py`，只用于旧报告复核，不能放行新运行。

当前候选协议已经升级为 `SMPCC-REAL-40-64-88-v2.0`，但尚未冻结；所有 Stage I/II formal trial 均为 `NO-GO`。本目录现有 validator、template 及若干 runner 仍带 `SMPCC-REAL-40-88-v1.0`/E2/E3 语义，旧 validator 的 PASS 不具有 v2.0 放行权。下表中的“正式链”仅表示未来完成 v2.0 升级后拟承担的角色；在只读 `freeze_manifest.yaml`、唯一 `FREEZE_ID`、新随机表、完整 G0--G6 报告链和 upgraded validator 同时存在前，任何脚本都不能直接产生 v2.0 formal 数据。其余脚本只用于开发、仿真、pilot、历史复现或诊断。

| 脚本 | 用途 | 协议角色 |
| --- | --- | --- |
| `record_spmpc_full_rgb_bag.sh` | 候选黑匣子 recorder，录 ROS topic 和 sidecar | future v2.0 正式链；当前仅 development |
| `run_spmpc_real_fixed_path_trial.sh` | SPMPC 实物单次一键 runner，支持生成或重放路径 | future v2.0 正式链；当前仅 development，未来 formal 只允许 replay |
| `run_spmpc_g2s_h0s_source_selection_trial.sh` | 固定 H0_G2、Bsmooth、IMU READY gate 和在线 RGB stamped scalar 的单条 G2S paired unit；bag 禁止图像流 | development G2S；不进入 40/64/88 |
| `analyze_spmpc_g2s_source_selection.sh` | 四条 G2S PASS 后的一键只读 source analyzer，自动加载 ROS/workspace 并使用冻结目录 | development source decision |
| `../tools/analysis/validate_g2s_paired_trial.py` | 单条 G2S bag 的 motion/在线视觉质量/零图像话题/双 observer/READY/selection postflight | development fail-closed QC |
| `../tools/analysis/analyze_g2s_source_selection.py` | 4 条同-trial paired unit 对 RGB 的 odom/IMU 决策 | development source decision；不自动签 formal release |
| `summarize_spmpc_real_trial.py` | 单包/目录离线完整性、配置和 fallback 摘要 | future v2.0 即时 QC；schema 尚未升级完整 |
| `validate_spmpc_formal_freeze.py` | 旧 v1.0 只读 freeze 校验 | **不得放行 v2.0**；必须升级后才可成为强制门控 |
| `run_external_baseline_real_fixed_path_trial.sh` | LT-DWA、TEB、MPC 外部 baseline 的 shadow/actuated 实物运行 | 独立外部 baseline，不属于当前内部 88 单元 |
| `record_spmpc_mainline_ground_smoke.sh` | 轻量地面 smoke recorder | smoke |
| `record_spmpc_experiment.sh` | planner 已手动启动时的备用 recorder | 手动调试 |
| `prepare_phase_rejoin_development_artifact.py` | 从严格配对的 rolling horizon/audit 离线导出 phase-rejoin 接口 artifact | **仅 development interface smoke；不是 OfflineSloshOCP 或实物正式 artifact** |
| `run_continuous_real.sh` | 历史 continuous MPCC 实物一键运行 | 历史/开发，不是正式 runner |
| `compare_b0_bslosh_smoke.sh` | B0/B_slosh 等内部 variant 仿真 smoke | 仿真 smoke |
| `verify_continuous_smoke.sh` | continuous acados 后端闭环仿真检查 | 仿真 smoke |
| `phase3_smoke.sh` | corridor/obstacle/guidance Phase 3 开发检查 | 仿真 smoke |
| `phase4_fixed_path_run.sh` | 历史开发 Phase 4 fixed-path 实物运行 | 历史脚本；不是正式 Smooth-match P4 |
| `sweep_w_slosh.sh` | 仿真中单值运行开发性 `w_slosh` 扫描 | 开发扫描 |
| `sweep_w_slosh_summary.py` | 汇总开发性权重扫描 bag | 离线开发分析 |
| `analyze_b0_bslosh_compare.py` | 对比多个 variant smoke bag | 离线开发分析 |
| `analyze_spmpc_delay_phase.py` | 从 bag 诊断 command/odom 延迟和 phase 状态 | 离线诊断 |
| `check_omega_smoke.py` | 快查停滞、角速度变化率和模型晃动峰值 | 离线 smoke 速查 |

配套目录用途：

- `../tools/codegen/acados/generate_spmpc_acados.py` 负责模型检查和求解器代码生成；同目录的模型、代价和约束文件是其装配模块，不单独运行；
- `../tools/analysis/estimate_cmd_odom_delay.py` 是早期 cmd/odom 互相关与绘图工具，当前优先使用本目录的 `analyze_spmpc_delay_phase.py`；
- `../test/python/` 保存 analysis、summary、artifact 和 freeze validator 的回归测试。

## prepare_phase_rejoin_development_artifact.py

该工具只为 Phase-Rejoining 的文件接口和闭环机制 smoke 准备临时输入。它从 bag 中按 `cycle_id` 一对一连接：

- `/spmpc/debug/predicted_horizon` 的第 0 个预测状态和第 0 个控制；
- `/spmpc/debug/control_cycle_audit` 中同周期最终实际发布的 `published_cmd_v/omega`。

所以得到的是 rolling local planner first-stage proxy，**不是** OfflineSloshOCP 输出，也不是 independently validated empirical artifact。工具把下面标记写死，不能通过命令行升级：

```text
evidence_level=development_only
source=development_proxy_replay
artifact_role=interface_smoke_only
offline_slosh_ocp=false
hardware_formal_release=false
paper_main_result_eligible=false
gate_evidence=none_development_input_only
recovery_policy_evidence=none_development_input_only
```

现有 bag 不包含经过独立验证的 recovery policy 或 gate 半径。工具因此没有默认值，也不会把零命令、预测控制或样本方差悄悄包装成这些对象。调用者必须显式提供逐 `cycle_id` 的 development 参数 CSV，且所有字段都要填写：

```csv
cycle_id,kappa_v,kappa_omega,r_x,r_y,r_yaw,r_v,r_omega,r_eta_x,r_eta_x_dot,r_eta_y,r_eta_y_dot
```

这些数值仍只作为接口输入；SHA-256、来源和“无证据”标记会写入输出。缺行、重复 cycle、空值、非正半径一律失败。

典型导出命令：

```bash
python3 src/scout_apps/control/spmpc_local_planner/scripts/prepare_phase_rejoin_development_artifact.py export \
  --bag /path/to/development_proxy.bag \
  --development-parameters /path/to/operator_supplied_development_parameters.csv \
  --output /tmp/phase_rejoin_development_only.csv \
  --contract-id proxy_route_config_v1 \
  --frame-id map \
  --dt 0.03333333333333333 \
  --path-length 8.0 \
  --start-cycle-id 120 \
  --end-cycle-id 359
```

默认话题为 `/spmpc/debug/predicted_horizon` 和 `/spmpc/debug/control_cycle_audit`，可显式覆盖。导出器要求选中区间内 cycle 连续、两种消息严格一对一、共享 timing 字段一致，且 solver 成功、状态已对齐、命令确实发布、无 safety/terminal/limiter/command-contract 干预。任一条件不满足即拒绝导出；应重新选择完整的连续 development 区间，不能手工补行。

输出会在原子替换前按 `NominalSequenceArtifact` 的固定表头、元数据、有限值、连续索引、采样周期、进度和正半径口径重新校验。也可单独运行：

```bash
python3 src/scout_apps/control/spmpc_local_planner/scripts/prepare_phase_rejoin_development_artifact.py validate \
  --artifact /tmp/phase_rejoin_development_only.csv

python3 -m unittest \
  src/scout_apps/control/spmpc_local_planner/test/python/test_phase_rejoin_development_artifact.py
```

正式 OfflineSloshOCP 必须由独立求解/验证链生成同 schema 文件，并使用与本工具不同、如实反映证据等级的来源合同；不得重命名或修改本工具输出的 metadata 来冒充正式 artifact。

## record_spmpc_full_rgb_bag.sh

实物 SPMPC 黑匣子录包脚本。它只负责录制 topic 和保存 metadata，**不发送 `/cmd_vel`、不发送目标点、不启动/停止 planner**。

文件名中的 `full_rgb` 是历史兼容名称；当前默认和 G2S/formal 政策均为 image-free，不代表会录视频。

主要用途：正式实物 run 前先启动 recorder，把事后分析可能需要的证据一次录全，包括：

- `/cmd_vel`、`/odom`、`/tf`、`/map`、固定路径/goal；
- `/spmpc/status`、`/spmpc/solver_backend`、`/spmpc/controller_variant`；
- `/spmpc/debug/effective_config`、`/spmpc/cost_breakdown`、`/spmpc/slosh_horizon_summary`；
- `/spmpc/debug/raw_state`、`/spmpc/debug/predicted_state`、`/spmpc/debug/solver_input_state`、`/spmpc/debug/command_intervention`；
- `/spmpc/debug/slosh_observer_odom`、`/spmpc/debug/slosh_observer_imu`、`/spmpc/debug/slosh_observer_selection`，用于区分 nominal/effective source、fallback、freshness 和 epoch；
- `/spmpc/debug/predicted_horizon`、`/spmpc/debug/pre_solve_snapshot`，用于完整预测时域和 actual/zero replay；
- `/spmpc/debug/warm_start`、`/spmpc/debug/warm_start_status`，用于核对 warm-start 来源和 `used_fallback`；
- `/liquid/measurement`、`/liquid/height*`、camera_info、scan、standalone `/slosh/*` 等；当前默认不录 RGB/depth/debug 图像流。

当前 recorder 默认 `RECORD_RGB=false`、`RECORD_ONLINE_LIQUID=true`、`RECORD_ONLINE_LIQUID_DEBUG_IMAGES=false`。G2S/G3/formal 应再设置 `FORBID_IMAGE_STREAMS=true`；这样启动时拒绝 `-a`/图像开关，闭包后还会检查 bag 内所有 message type，任何 `sensor_msgs/Image` 或 `CompressedImage` 都使 postflight 失败。相机仍实时向在线节点供图，但 bag 只保存带源图时间戳、有效性、零点原始样本/时间窗、置信度和裁剪状态的小体积派生消息。

典型用法：

```bash
RUN_LABEL=Bslosh_delay_off_run01 \
VARIANT=B_slosh \
RECORD_SEC=60 \
RECORD_RGB=false \
OUT_DIR=/home/geist/slosh_bags/real/20260704_fixed_path_compare/B_slosh \
bash src/scout_apps/control/spmpc_local_planner/scripts/record_spmpc_full_rgb_bag.sh
```

输出包括 `.bag`、`*_info.txt`、`*_rosparam.yaml`、`*_recorded_topics.txt`、`*_selected_topics_not_recorded.txt`、`*_topic_info/` 等 sidecar。正式 run 还应通过环境变量写入：

- `BLOCK_SEGMENT_ID`、`SPLIT_BLOCK`、`ORDER_POSITION`；
- `ACQUISITION_RETRY`、`RETRY_REASON_FILE`；
- `PATH_EXPECTED_SHA256`、`PATH_ACTUAL_SHA256`、`REQUIRE_PATH_HASH`；
- pilot 使用 `PILOT_CONDITION` 区分 `B0/Bsmooth/W1/W2/W5` 或 `S/Mminus/M0/Mplus`。

recorder 只保存调用者传入的这些字段，不负责判断 block 顺序是否合法，也不签署 freeze。正式合法性由上游 protocol shell 和 `validate_spmpc_formal_freeze.py` 检查。

### replay 话题 smoke 检查

启动 `continuous_mpcc_acados` 并进入正常求解后，正式录制前至少检查一次：

```bash
rostopic echo -n 1 /spmpc/debug/predicted_horizon
rostopic echo -n 1 /spmpc/debug/pre_solve_snapshot
rostopic echo -n 1 /spmpc/debug/warm_start
rostopic echo -n 1 /spmpc/debug/warm_start_status
```

必须满足：

- 两条消息的 `valid` 均为 `true`，`backend` 为 `continuous_mpcc_acados`；
- `horizon_steps: 60`；预测时域有 61 个状态样本和 60 个控制样本；
- pre-solve 快照的 `state_width: 10`、`control_width: 3`；
- B0 为 `parameter_width: 23`、`stage_parameters` 共 1403 个数；含 slosh 的 variant 为 `parameter_width: 32`、共 1952 个数；
- `initial_guess_states` 对应 `61 x 10`，`initial_guess_controls` 对应 `60 x 3`；
- 第二个及后续有效求解周期通常应有 `have_previous_solution: true`；
- `primal_guess_only: true` 是当前预期值，表示未录对偶变量和内部 SQP memory。

短录一包后再确认两个话题实际进入 bag：

```bash
rosbag info /path/to/run.bag | rg '/spmpc/debug/(predicted_horizon|pre_solve_snapshot|warm_start|warm_start_status)'
```

若现场机没有 `rg`，可把最后一段替换为 `grep -E`。上述检查只证明录制接口和显式输入完整；actual replay 还必须在冻结容差内复现在线 solver status、第一控制量和 raw command，才能用于正式反事实分析。

## run_spmpc_real_fixed_path_trial.sh

实物 SPMPC fixed-path 单次试验一键脚本。前提是实物传感器/定位/底盘栈已经启动。它支持两种路径来源：

```text
PATH_SOURCE_MODE=generate：按当前位姿生成路径，兼容旧用法
PATH_SOURCE_MODE=replay：回放冻结 JSON，通过起点位置/航向门控后启动
```

`generate` 只用于历史兼容、开发 smoke，以及正式采集前生成一次 H0/H1/L1 候选。正式 40/64/88 run 必须使用：

```bash
: "${H1_EXPECTED_SHA256:?从冻结 manifest 填写 H1 SHA-256}"
PATH_SOURCE_MODE=replay \
PATH_FILE=/path/to/frozen/H1_P2_s_curve.json \
REQUIRE_PATH_HASH=true \
PATH_EXPECTED_SHA256="${H1_EXPECTED_SHA256}" \
bash src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_real_fixed_path_trial.sh
```

replay 时脚本计算实际 SHA-256 并写入 `*_path_sha256.txt`；摘要不匹配立即退出。若 `${RUN_OUT_DIR}/${NAME}.bag` 或 `.bag.active` 已存在，脚本也会拒绝覆盖，补采必须改用协议允许的 `r02` 标签并保留原因文件。

非 pilot 默认仍为 `generate`，目标点固定为 2026-07-02 实物 bag 中恢复出的终点：`GOAL_X=-5.424`、`GOAL_Y=-4.736`、`GOAL_YAW=0.0`。`PILOT_MODE=true` 时默认切换到 `replay`，默认读取 `${HOME}/fixed_paths/real/${DATE}_spmpc_parameter_pilot/H0_weight_pilot.json`；起点门控默认放宽为 `0.08 m / 0.15 rad`，也可用环境变量覆盖。

历史实物机工作空间为 `/home/geist/scout_ws`，旧路径文件位于 `/home/geist/fixed_paths/real/<DATE>_fixed_path_compare/fixed_s_curve_compare.json`。旧脚本会逐 run 覆盖这个文件，因此不能把某个历史日期目录下的最后一份 JSON 自动视为冻结路径。下一次现场先在地面起点标记处运行一次 `PATH_SOURCE_MODE=generate` 的 H0 path-freeze smoke，再让全部权重 pilot replay 这份新 JSON。

首次生成 H0：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

DATE=20260718 \
PILOT_MODE=true \
PILOT_METHOD=B0 \
PATH_SOURCE_MODE=generate \
RUN_LABEL=PF_PATH_FREEZE_H0_B0_smoke01 \
RUN_OUT_DIR=/home/geist/slosh_bags/real/20260718_spmpc_parameter_pilot/PF/PATH/H0_C1/B0 \
CMD_TOPIC=/cmd_vel \
PILOT_RECORD_RGB=false \
RECORD_TOPIC_INFO=false \
RECORDER_STARTUP_SEC=8 \
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
bash src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_real_fixed_path_trial.sh
```

成功后路径应保存在：

```text
/home/geist/fixed_paths/real/20260718_spmpc_parameter_pilot/H0_weight_pilot.json
```

pilot generate 默认拒绝覆盖已经存在的 H0。后续直接使用 replay；只有明确作废旧路径并准备重新冻结时，才可设置 `ALLOW_PILOT_PATH_OVERWRITE=true`。

pilot 默认强制 `RECORD_RGB=false`，不向 bag 写入原始彩色图像。模型侧 pilot 默认也不启用视觉；需要物理液面证据时使用 `PILOT_RECORD_ONLINE_LIQUID=true` 并由冻结在线节点发布 `/liquid/measurement`。`PILOT_RECORD_RGB=true` 只保留给协议外的短时相机调试，不得用于 G2S、G3 或 formal bag。

`PILOT_METHOD` 提供现场快速参数映射。正式协议 v1.0 只允许下列 5 个 P3 条件：

| `PILOT_METHOD` | `VARIANT` | `W_SLOSH` |
| --- | --- | ---: |
| `B0` | `B0` | `0` |
| `Bsmooth` | `B_smooth` | `0` |
| `W1` / `W2` / `W5` | `B_slosh` | `1` / `2` / `5` |

脚本为非正式开发调试保留其他 `W<number>` 的解析能力，但 `W10`、`W3.5` 或任何临时候选都不属于 `SMPCC-REAL-40-88-v1.0`，不得进入正式 P3、40、64 或 88 单元，也不能在看到 pilot 结果后追加。

参数 pilot 推荐命令：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

PILOT_MODE=true \
PILOT_METHOD=W2 \
DATE=20260718 \
PATH_FILE=/home/geist/fixed_paths/real/20260718_spmpc_parameter_pilot/H0_weight_pilot.json \
RUN_LABEL=PF_WS_H0_C1_W2_b01_r01 \
START_POS_TOL=0.08 \
START_YAW_TOL=0.15 \
CMD_TOPIC=/cmd_vel \
PILOT_RECORD_RGB=false \
RECORD_TOPIC_INFO=false \
RECORDER_STARTUP_SEC=8 \
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
bash src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_real_fixed_path_trial.sh
```

下一次按冻结的 15 单元顺序修改 `PILOT_METHOD`、`PILOT_CONDITION`、`RUN_LABEL`、block/位置字段。脚本会把 pilot、路径模式、起点容差、`V_REF`、`W_SLOSH`、路径 hash、block/retry metadata 和实际 planner 命令同时写入 one-click 与 recorder sidecar。

`RECORD_SEC` 默认 60，且大于 `MAX_RECORD_SEC` 时会被截断；若希望录 90 秒，应同时设置 `RECORD_SEC=90 MAX_RECORD_SEC=90`。非 pilot 为保持旧用法仍默认 `DELAY_PHASE_MODE=off`；pilot 根据 0705/0706 实物稳定口径默认使用 `fixed_closed_loop 0.15/0.22`、`RECORDER_STARTUP_SEC=8` 和 `RECORD_TOPIC_INFO=false`，命令中仍建议显式写出以便现场复核。

旧的按当前位姿生成路径用法保持不变：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

ALG=B_slosh \
RUN_LABEL=Bslosh_delay_off_run01 \
CMD_TOPIC=/cmd_vel \
DELAY_PHASE_MODE=off \
DELAY_PHASE_LINEAR_DELAY_SEC=-1.0 \
DELAY_PHASE_ANGULAR_DELAY_SEC=-1.0 \
RECORD_RGB=false \
bash src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_real_fixed_path_trial.sh
```

该脚本只覆盖 SPMPC 内部消融，不用于外部 baseline。

### liquid observer source 边界

默认 `CURRENT_OBSERVER_SOURCE=odom`，保持当前发布语义。只有显式设置 `processed_imu` 时，IMU observer 才能作为当前液体状态进入 solver、risk governor 和 delay predictor；未来 horizon 仍由机器人/液体模型传播，不复制当前 IMU 值。

```text
CURRENT_OBSERVER_SOURCE=odom | processed_imu
OBSERVER_FALLBACK_POLICY=odom | fail_closed
OBSERVER_LATCH_FALLBACK=true
OBSERVER_MAX_IMU_STATE_AGE_SEC=0.10
OBSERVER_MAX_ODOM_STATE_AGE_SEC=0.50
```

nominal IMU 在本次节点生命周期第一次达到 `READY + valid + bias_ready + filter_ready` 前禁止 fallback，solver 消费液体状态的 variant 保持零命令；这避免一次 IMU trial 在启动阶段静默变成 odom trial。IMU 曾 READY 后若失效，默认转 odom 并锁存到本次节点退出。bag 中必须以 `/spmpc/debug/slosh_observer_selection` 的 `nominal_source/effective_source/fallback_active/reason/selection_epoch` 判定实际输入，不能只看启动参数。Bsmooth/B0 的 `solver_consumes_selected_state=false`，即便 parallel observers 正常发布，也不能写成 solver 使用了某路液体状态。

## G2S：H0_G2 上的 odom/processed-IMU paired source selection

该流程每次只运行一条，4 条均复用：

```text
/home/geist/fixed_paths/real/20260727_spmpc_development/H0/H0_G2.json
SHA-256 = 578a4dd7663c2f49b4270c37755a08b2b0dc70735fb6b818da35b60a60f3990e
```

先只检查配置，不连车运动：

```bash
cd /home/geist/scout_ws
VALIDATE_ONLY=true G2S_ROW=01 \
bash src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_g2s_h0s_source_selection_trial.sh
```

确认相机/容器/液深与指定 RGB 标定一致、场地清空、机器人已在旧 G2 起点并对齐后，逐条运行；`G2S_ROW` 依次改为 `01`、`02`、`03`、`04`，每条之间必须回位并等液体静稳：

```bash
ARM_MOTION=YES \
CONFIRM_RGB_GEOMETRY=YES \
G2S_ROW=01 \
bash src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_g2s_h0s_source_selection_trial.sh
```

RealSense 仍提供 `1920x1080@30` 在线输入，G2S 使用 2026-07-31 当前场景冻结标定 `/home/geist/slosh_bags/real/20260731_spmpc_g2s_source_selection/calibration/red_3ruler_g2s_20260731_frozen.yaml`。默认 bag 不保存 raw/compressed/depth/debug 图像；诊断采集可显式设置 `G2S_RECORD_RAW_RGB=true`，此时只允许 `/camera/color/image_raw`，仍禁止 compressed/depth/debug。该标定只用于 G2S development，不自动升格为正式 Stage 标定。运行前还必须用 `set_realsense_rgb_manual_params.sh` 关闭自动曝光和自动白平衡；row 01 捕获 exposure/gain/white-balance，后续三条必须完全一致。

脚本会自动启动 `online_liquid_height.launch`（强制 `publish_debug=false`），冻结并记录 calibration、detector、node、message、launch 和 config hash；等待 `/liquid/measurement` 达到 `zero_locked + valid + status=OK` 后才进入底层 runner。recorder 只收 stamped scalar/quality、camera_info 和控制/observer 证据，完整记录 IMU bias/filter transient，IMU READY 后才发布路径。90 s 录制结束后自动 postflight（默认同时计算 bag SHA-256），要求 image-stream count=0、在线视觉覆盖完整 motion/tail、实际速率至少 10 Hz、有效率至少 90%，并检查双 observer coverage、IMU READY、selection epoch 与 `GOAL_REACHED`。任一门失败时 attempt 保留但不进入 eligible 4 条。

4 条都 PASS 后执行唯一 paired analyzer：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash

python3 src/scout_apps/control/spmpc_local_planner/tools/analysis/analyze_g2s_source_selection.py \
  --bag-dir /home/geist/slosh_bags/real/20260731_spmpc_g2s_source_selection/H0s_Bsmooth \
  --calibration /home/geist/slosh_bags/real/20260731_spmpc_g2s_source_selection/calibration/red_3ruler_g2s_20260731_frozen.yaml \
  --out-dir /home/geist/slosh_bags/real/20260731_spmpc_g2s_source_selection/analysis
```

analyzer 不再调用离线图像推理，也不需要视频；它直接读取消息内的源图 `header.stamp` 和 `height_max_lcr_mm`，对干净有效标量使用冻结的 5 帧 centered rolling median。禁止逐 trial 调 lag/scale/filter，固定用 observer `total_height_m` 对在线 RGB 标量的 motion-window MAE。只有 aggregate MAE 改善至少 10%、至少 3/4 trial 同方向、去掉最佳单条后总改善仍为正且 IMU coverage 不劣化时，输出 `processed_imu`；相当、冲突、不确定或任一 gate 失败均输出 `odom`。该输出仍是 development source decision；若选择 IMU，还要用新 revision 做 `publish_cmd_vel=false` replay 和 2--3 对 W2/W5 闭环确认，不能把 shadow-only 旧 revision 冒充为同一 release。

2026-07-31 实际批次由操作员在 `u01..u03` 后停止，不补 `u04`，因此上面的正式四包 analyzer 不适用。随后三条 raw-RGB diagnostic bag 使用 v2 重标尺离线复算，processed-IMU 的平均 MAE 改善为 `34.56%`、方向 `3/3` 一致、两路 coverage 均为 `100%`。可重复生成 development-only 机器报告：

```bash
bash src/scout_apps/control/spmpc_local_planner/scripts/analyze_spmpc_g2s_raw_rgb_three_trial.sh
```

该报告只能放行 `run_spmpc_g2c_processed_imu_w2w5_trial.sh` 的 G2C development，不能写成正式四包 G2S PASS，也不能放行 G3 或 40/64/88。

## summarize_spmpc_real_trial.py

实物 SPMPC bag 离线 summary 脚本。支持传入单个 `.bag` 或 run 目录，读取 rosbag 与 recorder / one-click sidecar，输出：

```text
${bag_stem}_summary.json
${bag_stem}_summary.md
```

它按 `Float32MultiArray.layout.dim[0].label` 动态解析字段，避免依赖硬编码索引。旧 bag 缺少新 debug topic 时不会失败，而是在 summary 中列出 red flags。

当前 summary 还会：

- 只对 `B_slosh` 强制要求在线 slosh state/cost/horizon；B0 和 B_smooth 不会因论文定义中关闭液体状态而被误报；
- 读取 `/spmpc/debug/warm_start` 的 `used_fallback`，字段不存在、没有 label 或实际使用 fallback 都会产生 red flag；
- 汇总 `PILOT_CONDITION`、`BLOCK_SEGMENT_ID`、`SPLIT_BLOCK`、`ORDER_POSITION` 和 acquisition retry 信息；
- 比较 sidecar 中的 intent 与 `/spmpc/debug/effective_config`，用于发现 `v_ref`、`w_slosh` 等实际生效值不一致。

典型用法：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash

python3 src/scout_apps/control/spmpc_local_planner/scripts/summarize_spmpc_real_trial.py \
  /home/geist/slosh_bags/real/20260704_fixed_path_compare/B_slosh/Bslosh_delay_off_run01.bag
```

## validate_spmpc_formal_freeze.py

正式采集前的只读、fail-closed 校验器。它不生成、不修改也不签署 manifest；只验证已经由受控流程填写并设为只读的 `freeze_manifest.yaml`。当前模板仍是 `NO-GO`，不能把模板改一个 `status` 后直接使用。

校验范围包括协议版本、10 个 gate、40 组 artifact/hash、Git clean/HEAD、方法与 stage/group/path/container 合法组合、路径和容器配置、实际 `v_ref/w_slosh`、Smooth-match 安全区间、C2 参数、`T_SETTLE` 以及本轮禁用的后续 release 功能。

正式 H1/C1/B0 单元的调用形态如下；数值必须来自本次只读 manifest 和运行环境，不能照抄占位符：

```bash
: "${CONTAINER_RADIUS:?从 manifest 载入 C1 半径}"
: "${LIQUID_HEIGHT:?从 manifest 载入 C1 液深}"
: "${DAMPING_RATIO:?从 manifest 载入 C1 阻尼比}"

python3 src/scout_apps/control/spmpc_local_planner/scripts/validate_spmpc_formal_freeze.py \
  --manifest docs/实物实验注意事项/对比试验/实物对比实验/freeze/freeze_manifest.yaml \
  --repo-root /home/geist/scout_ws \
  --stage S1 \
  --group E2 \
  --method B0 \
  --variant B0 \
  --v-ref 0.20 \
  --w-slosh 0 \
  --path-id H1 \
  --path-file /path/to/frozen/H1_P2_s_curve.json \
  --container-id C1 \
  --container-config tube_default \
  --container-yaml /path/to/frozen/tube_default.yaml \
  --container-radius "${CONTAINER_RADIUS}" \
  --liquid-height "${LIQUID_HEIGHT}" \
  --damping-ratio "${DAMPING_RATIO}"
```

成功时退出码为 0，并输出 `FORMAL_FREEZE_VALIDATION=PASS`、`FREEZE_ID` 和 `T_SETTLE`；任意一项不一致时退出码为 2，输出全部错误并阻止正式 run。完整的现场变量映射和 validation report 保存方式以 `0717_S-MPCC正式实物实验启动与录制命令.md` 为准。

## compare_b0_bslosh_smoke.sh

Phase 2/3 内部消融 smoke 脚本。用于在仿真中跑一个 SPMPC variant，
并录制 `/spmpc/*`、`/cmd_vel`、`/odom` 等诊断话题。

典型用法：

```bash
source /home/a/scout_ws/devel/setup.bash
cd /home/a/scout_ws

VARIANT=B0 OUT_DIR=/data/a/spmpc_compare \
  bash src/scout_apps/control/spmpc_local_planner/scripts/compare_b0_bslosh_smoke.sh

VARIANT=B_slosh OUT_DIR=/data/a/spmpc_compare \
  bash src/scout_apps/control/spmpc_local_planner/scripts/compare_b0_bslosh_smoke.sh
```

## analyze_b0_bslosh_compare.py

离线分析 smoke bag，比较不同 variant 的控制输出、预测晃动、cost breakdown、
primitive 选择和 progress 单调性。

```bash
python3 src/scout_apps/control/spmpc_local_planner/scripts/analyze_b0_bslosh_compare.py \
  /data/a/spmpc_compare B0 B_slosh B_smooth B_ours
```

默认统计 active solver window，排除 `GOAL_REACHED` 后的全零段。

## phase3_smoke.sh

Phase 3 corridor / obstacle / guidance smoke 脚本。用于验证 Phase 3
点亮后 planner 是否能在 fixed-path 或 point-to-point 场景中闭环运行。

```bash
PHASE3_MODE=p2p VARIANT=B0 OUT_DIR=/data/a/spmpc_phase3_p2p_smoke \
  bash src/scout_apps/control/spmpc_local_planner/scripts/phase3_smoke.sh

PHASE3_MODE=fixed_path VARIANT=B0 OUT_DIR=/data/a/spmpc_phase3_fixed_smoke \
  bash src/scout_apps/control/spmpc_local_planner/scripts/phase3_smoke.sh
```

## phase4_fixed_path_run.sh

Phase 4 fixed-path 实物对比实验脚本。它只负责固定路径生成、目标发送、
启动 `spmpc_fixed_path.launch` 和录包；不替代实物传感器/定位/底盘启动脚本。

这里的“Phase 4”是历史开发阶段名称，不是当前现场协议中的 P4 Smooth-match pilot。该脚本会生成路径，未接入 freeze manifest 和正式路径 hash 门控，因此不能用于当前 40/64/88 正式矩阵。

实物前置启动仍使用现有传感器栈，例如：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws
src/scout_apps/control/scout_local_planner/scripts/launch_real_sensors_stack.sh
```

另开终端运行 Phase 4：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

VARIANT=B_ours_anti \
OUT_DIR=/home/geist/slosh_bags/real/20260602_spmpc_phase4 \
GOAL_X=7.164488315582275 \
GOAL_Y=9.307367324829102 \
GOAL_YAW=1.0808 \
bash src/scout_apps/control/spmpc_local_planner/scripts/phase4_fixed_path_run.sh
```

常用环境变量：

```text
VARIANT      B0 / B_slosh / B_smooth / B_ours / B_slosh_anti / B_ours_anti
OUT_DIR      输出 bag、log、meta.yaml 的目录
RECORD_SEC   录包时长，默认 45
GOAL_X/Y/YAW 固定目标点
PATH_FILE    固定路径 JSON 输出位置
RECORD_CAMERA true/false
RUN_ID       可手动指定 run 名称
```

## record_spmpc_experiment.sh

手动录包备用入口。适合 planner 已经手动启动，只需要记录 SPMPC 诊断、
相机、odom、cmd_vel 和 slosh 话题的场景。

```bash
OUT_DIR=/home/geist/slosh_bags/real/20260602_spmpc_manual \
RUN_ID=B_ours_anti_run01 \
bash src/scout_apps/control/spmpc_local_planner/scripts/record_spmpc_experiment.sh
```

它没有正式 block、路径 hash、freeze validation 和 warm-start 完整性门控，只适合作为调试备用入口。正式实验统一使用 `record_spmpc_full_rgb_bag.sh`，通常由正式 runner 或现场协议命令调用。

## run_external_baseline_real_fixed_path_trial.sh

外部 LT-DWA、TEB、MPC local planner 的单次实物 wrapper。它会启动 standalone slosh monitor、模板路径生成器、黑匣子 recorder 和指定 baseline，支持：

```text
METHOD=lt_dwa_official | teb | mpc_local_planner
STAGE=shadow | actuated
```

先用 shadow 验证配置和输出话题；actuated 会发布 `/cmd_vel`，脚本会拒绝在已有 `/cmd_vel` publisher 时启动：

```bash
METHOD=teb \
STAGE=shadow \
DATE=20260726 \
RUN_LABEL=TEB_shadow_smoke01 \
RECORD_SEC=60 \
bash src/scout_apps/control/spmpc_local_planner/scripts/run_external_baseline_real_fixed_path_trial.sh
```

该脚本仍按当前起点生成模板路径，没有接入本轮内部消融的 formal manifest/replay 门控。因此它服务于独立外部 baseline 方案，不属于 `SMPCC-REAL-40-88-v1.0` 的 B0/B_smooth/B_slosh 88 单元。

## record_spmpc_mainline_ground_smoke.sh

轻量地面 smoke recorder。planner、路径和底盘均需由操作者另行启动；脚本只记录主要 `/spmpc/*`、`/cmd_vel`、odom、TF 和可选相机/scan/standalone slosh：

```bash
VARIANT=B_slosh \
RECORD_SEC=20 \
RECORD_CAMERA=false \
OUT_DIR=/home/geist/slosh_bags/real/20260726_spmpc_mainline_ground \
bash src/scout_apps/control/spmpc_local_planner/scripts/record_spmpc_mainline_ground_smoke.sh
```

`RECORD_SEC=0` 表示录到 Ctrl+C。它的话题和 sidecar 少于正式 recorder，只用于短 smoke。

## run_continuous_real.sh

历史 continuous acados 实物一键运行脚本：从当前位姿生成模板路径、发送目标、启动 planner 并录包。要求基础传感器栈和 acados 环境已经启动，且必须显式提供当天统一终点：

```bash
DATE=20260726 \
VARIANT=B_slosh \
GOAL_X=-5.424 \
GOAL_Y=-4.736 \
GOAL_YAW=0.0 \
REQUIRE_RGB_TRUTH=true \
RECORD_RGB_CAMERA=true \
bash src/scout_apps/control/spmpc_local_planner/scripts/run_continuous_real.sh
```

该脚本会逐 run 生成路径，没有正式 replay/hash/freeze 门控，保留用于历史复现和开发，不用于当前正式矩阵。

## verify_continuous_smoke.sh

continuous acados 后端的仿真闭环 smoke。先从固定 spawn 启动仿真，随后每个 variant 单独重启仿真再运行：

```bash
VARIANT=B0 \
OUT_DIR=/data/${USER}/spmpc_continuous \
bash src/scout_apps/control/spmpc_local_planner/scripts/verify_continuous_smoke.sh

# 重启仿真回同一起点后
VARIANT=B_slosh \
OUT_DIR=/data/${USER}/spmpc_continuous \
bash src/scout_apps/control/spmpc_local_planner/scripts/verify_continuous_smoke.sh
```

脚本检查 acados 库和生成物、backend/status、首末 `/cmd_vel` 与 solver time，并生成 smoke bag。它会生成临时路径，只用于仿真。

## sweep_w_slosh.sh 与 sweep_w_slosh_summary.py

开发性仿真权重扫描。每个权重必须从相同 spawn 重新启动仿真，单值运行后统一汇总：

```bash
W_SLOSH=1 OUT_DIR=/data/${USER}/spmpc_wsweep \
  bash src/scout_apps/control/spmpc_local_planner/scripts/sweep_w_slosh.sh

# 重启仿真后分别运行 W_SLOSH=2、5

python3 src/scout_apps/control/spmpc_local_planner/scripts/sweep_w_slosh_summary.py \
  /data/${USER}/spmpc_wsweep B_slosh
```

summary 输出 observer peak/mean、命令、solver time 和 progress 单调性，用于开发诊断。正式 P3 必须按冻结的 15 单元、H0 replay、W1/W2/W5、standalone reset 和 `delta_model` 规则执行，不能用这个单次仿真 sweep 替代。

## analyze_spmpc_delay_phase.py

只读离线分析一个或多个 bag 的 command→odom 正延迟、消息间隔、solver 和 delay-phase 状态；不会发布命令或修改 bag：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash

python3 src/scout_apps/control/spmpc_local_planner/scripts/analyze_spmpc_delay_phase.py \
  /home/geist/slosh_bags/real/20260726_spmpc_formal \
  --cmd-source spmpc_limited \
  --out-csv /tmp/spmpc_delay_summary.csv
```

`--cmd-source cmd_vel` 使用最终 `/cmd_vel`；`spmpc_limited` 使用 `/spmpc/debug/cmd_vel_output` 的 limited command。正 lag 表示 odom 响应晚于 command。该工具给出估计和 proxy，不把 bag 消息间隔命名为严格 control-cycle deadline miss。

## check_omega_smoke.py

递归扫描目录内 bag，快速报告是否停滞、`omega` 变化率 p95/max 和去除起步 2 s 后的模型晃动峰值：

```bash
python3 src/scout_apps/control/spmpc_local_planner/scripts/check_omega_smoke.py \
  /data/${USER}/spmpc_continuous
```

它是阈值速查，不代替正式 summary、RGB 物理液面或统计分析。

## acados/ 代码生成

检查 CasADi 模型装配而不调用 acados：

```bash
python3 src/scout_apps/control/spmpc_local_planner/tools/codegen/acados/generate_spmpc_acados.py \
  --model b0 --check

python3 src/scout_apps/control/spmpc_local_planner/tools/codegen/acados/generate_spmpc_acados.py \
  --model slosh --check
```

安装并配置 `acados_template` 后，去掉 `--check` 生成 C 代码和求解器：

```bash
python3 src/scout_apps/control/spmpc_local_planner/tools/codegen/acados/generate_spmpc_acados.py \
  --model slosh
```

正式实验使用的 codegen 输出和 build log 必须先归档 hash 并进入 freeze manifest；正式采集期间禁止重新生成后继续沿用同一 `FREEZE_ID`。

## tools/analysis/estimate_cmd_odom_delay.py

早期互相关延迟估计工具，内置配置参考值 `150/220 ms`，可输出相关曲线：

```bash
python3 src/scout_apps/control/spmpc_local_planner/tools/analysis/estimate_cmd_odom_delay.py \
  /path/to/run01.bag /path/to/run02.bag \
  --plot --out_dir /tmp/delay_analysis
```

新实验优先使用参数更完整、能输出 CSV 并识别更多 SPMPC 状态的 `analyze_spmpc_delay_phase.py`；本脚本保留用于复核历史分析。

## test/python/

运行本目录的全部回归测试：

```bash
cd /home/a/scout_ws
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover \
  -s src/scout_apps/control/spmpc_local_planner/test/python \
  -p 'test_*.py' -v
```

当前测试覆盖 formal freeze fail-closed 行为、正式 `w_slosh` 候选限制、路径/配置/hash、Git clean、Smooth-match 匹配，以及 summary 的 variant-aware 和 warm-start/fallback 解析。
