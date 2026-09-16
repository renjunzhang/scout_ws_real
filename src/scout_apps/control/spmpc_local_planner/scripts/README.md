# 实物与离线脚本索引

更新：2026-09-16，当前 `feat/spmpc-liquid-control`（五项复审修复实现至 `d17a8ea`）。优先按9月实物记录和Git历史整理，便于复用最近的C03、Full/Smooth、动捕辨识及诊断链。**此前实物使用的是 `diag/lt-dwa-collision-tracking`**；旧冻结runner仍有旧模型/场景合同，不能直接作为当前两层主线的实物验收入口。

本包为 **ROS1/catkin**。实包读取通常需要ROS1 `rosbag`和对应消息工作区；纯计划/数值工具使用Python、NumPy、CasADi/IPOPT，真实OCP重算还需acados。不要将`roslaunch/rosbag`换成ROS2命令直接运行。

本轮连续投影、区域停车与回放合同的参数和回归结果见[五项修复记录](../../../../../docs/实物实验注意事项/后续改进/20260916_局部规划器五项修复与回归.md)。无输出回放使用下方现有入口；本轮没有增加新的实物runner。

## 先按目的选入口

| 本次要做什么 | 优先入口 | 行为与输出 |
| --- | --- | --- |
| 最近C03 Full/Smooth无RGB配对、三组RGB、初态来源或jerk对照 | [run_spmpc_ablation_smoke.sh](run_spmpc_ablation_smoke.sh) | 默认校验；`--run`会启动采集/运动；bag、元数据、验收及图 |
| 对已有Full/Smooth包算内部液体指标 | [analyze_internal_slosh_pair.py](analysis/analyze_internal_slosh_pair.py)、[freeze_internal_slosh_evaluation.py](analysis/freeze_internal_slosh_evaluation.py) | 只读bag/报告，输出指标、配对报告或评价锁 |
| 看完整时域的命令、预测和液体链 | [plot_spmpc_full_da_diagnostics.py](analysis/plot_spmpc_full_da_diagnostics.py) | 只读bag，固定六张图 |
| 核对车体初态与未来实际运动 | [analyze_robot_state_prediction.py](analysis/analyze_robot_state_prediction.py) | bag + 场地reference JSON；summary、逐拍CSV和三张图 |
| 复用历史Full-DA smoke | [run_spmpc_full_da_smoke.sh](run_spmpc_full_da_smoke.sh) | 默认校验；`--run`会运动；复用runtime引擎，先画图再汇总验收 |
| 静止核查RGB录制卡顿 | [diagnose_spmpc_rgb_recording.py](diagnose_spmpc_rgb_recording.py) | ROS静态采集，对比no_record/rgb/full；不发速度 |
| 已手动启动planner，只需录全链 | [record_spmpc_full_rgb_bag.sh](record_spmpc_full_rgb_bag.sh) | 只录制，默认无图像；bag、rosparam、topic等sidecar |
| 通用固定路线单次运行 | [run_spmpc_real_fixed_path_trial.sh](run_spmpc_real_fixed_path_trial.sh) | 启动planner、路径和录制，会运动；近期专项优先通过上层wrapper |
| 测动捕、速度阶跃与连续性 | 见下方“场地和执行器辨识” | 部分入口直接发布速度，不能按文件名判断是否只录包 |
| 新两层四组实验 | [配置README](../config/experiments/trajectory_mpcc/README.md)、[record_trajectory_mpcc_comparison.sh](record_trajectory_mpcc_comparison.sh) | 新软件候选，尚未实物验收；recorder只录已有节点 |

## 最近使用的C03链：先核对分支

共同调用链是 `run_spmpc_ablation_smoke.sh` → [runtime引擎](run_spmpc_i0_failclosed_explicit_actuator_runtime_smoke.sh)（source `lib/spmpc_ablation_profile.sh`、`lib/spmpc_ablation_scene.sh`）→ 通用fixed-path runner → recorder → 六图及各项postflight。不要跳过wrapper后仍沿用原实验标签。

| 模式 | 当前分支支持的关键参数 | 适用范围 |
| --- | --- | --- |
| C03内部消融 | `--scene 20260907_c03 --condition full|smooth|nostate|b0|no_jerk --jerk-max ...` | 默认场景仍是C02，C03必须显式指定；b0关闭硬jerk，不能冒充Smooth |
| 无RGB Full/Smooth V2 | `--experiment internal-slosh --condition full|smooth --trial-id ... --phase screening|validation` | 固定IMU、v_ref=0.2、jerk=0.6、N=60；Full默认w_slosh=1，可设0<w<=20；Smooth为0 |
| V2复验评价锁 | `--evaluation-lock FILE --evaluation-row 01|02|03|04` | validation行序Full→Smooth→Smooth→Full；先筛选再冻结监视器，不能逐包挑有利指标 |
| C03三组RGB | `--experiment ablation-rgb --condition smooth|nostate|full --trial-id ...` | 强制RGB/NOKOV/双监视器；jerk=0.6、v_ref=0.2；液体组权重1或0.5 |
| Full初态来源对照 | `--condition full --observer-source imu|odom`，可加`--record-rgb` | 保留Tracker0和双观察器，只改变OCP液体初态；车体来源不因此切换 |

所有模式默认只校验，`--run`才进入运动采集。历史wrapper会核对冻结地图、路径、环境和模型合同；本机缺现场文件、或当前新ABI与旧冻结合同不符时应失败，不能删除门禁来声称复现成功。

**只在collision实物分支里的近期扩展：** `aa08be3`及后续提交新增V3的 `--w-v/--w-contour/--w-lag`、速度/jerk扩展；`4fb5820/de93315`新增并校验`--skip-start-wait`，另有`--slosh-height-max-mm`。当前feat入口没有这些选项。`analysis/analyze_exact_ocp_cost.py`、`analysis/analyze_ocp_imu_forecast.py`也只存在于diag分支；它们并非当前新模型的可直接运行工具。

不用切换分支即可查阅：

```bash
git show diag/lt-dwa-collision-tracking:src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_ablation_smoke.sh
git diff HEAD diag/lt-dwa-collision-tracking -- src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_ablation_smoke.sh
```

当前V2只校验示例（仓库根目录）：

```bash
SPMPC_SCRIPTS="$PWD/src/scout_apps/control/spmpc_local_planner/scripts"
bash "$SPMPC_SCRIPTS/run_spmpc_ablation_smoke.sh" \
  --scene 20260907_c03 --experiment internal-slosh \
  --condition full --trial-id s01_full --phase screening \
  --w-slosh 1 --validate-only
```

近期依据：[9月7日C03](../../../../../docs/实物实验注意事项/对比试验/实物对比试验分析/20260907_C03新地图四组消融内部液面高度分析.md)、[9月10日三组与权重思路](../../../../../docs/实物实验注意事项/对比试验/解决问题的思路/20260910_C03固定条件三组对比与液体权重小范围调参思路.md)。后续9月14日用法按上述diag版本查阅。

## 每次实物如何复用

1. 确认代码分支、bag所属版本、场地地图/路径SHA、容器、观察器和实验条件；先读对应wrapper的参数段。
2. 按原流程启动传感器、定位、底盘及独立测量。地图/S路径、NOKOV、RGB准备入口见下面的表。
3. 使用本次协议的单包wrapper；手动启动planner时才单独用recorder。给新run标签/输出目录，保留bag与全部sidecar。
4. 先看runner/postflight返回码与失败原因，再看六图、状态预测和配对指标。失败包同样保留；有效采集不等于有效降晃。

### 通用运行与录包参数

| 入口 | 参数与容易用错之处 |
| --- | --- |
| [run_spmpc_real_fixed_path_trial.sh](run_spmpc_real_fixed_path_trial.sh) | 环境变量接口，**没有通用`--run/--help`命令行门**，直接执行可能运动。`PATH_SOURCE_MODE=generate|replay`；非pilot默认generate且有历史目标坐标，复用固定路线应明确replay、PATH_FILE、PATH_EXPECTED_SHA256、REQUIRE_PATH_HASH=true |
| 同上 | `ALG`、`RUN_LABEL`、`V_REF`、`W_SLOSH`、`CURRENT_OBSERVER_SOURCE`、`OBSERVER_FALLBACK_POLICY`、`DELAY_PHASE_MODE`需和本次实验匹配；保留起点位置/朝向门。不要从diag复制当前未透传的参数 |
| 同上 | `RECORD_SEC`默认60，超过`MAX_RECORD_SEC`会被截断；延长到90需两者都设90。输出含bag、one-click metadata、实际planner命令及recorder sidecar |
| [record_spmpc_full_rgb_bag.sh](record_spmpc_full_rgb_bag.sh) | 只录包，不发速度/目标，不启停planner。`OUT_DIR`、`RUN_LABEL`、`VARIANT`、`RECORD_SEC`；名字full_rgb是历史名称，当前`RECORD_RGB=false`、`RECORD_ONLINE_LIQUID=true`、debug images=false |
| 同上 | 无图像协议可设`FORBID_IMAGE_STREAMS=true`；三组RGB/来源RGB协议会另行开启图像，不能把所有C03笼统写成image-free。输出bag、`*_info.txt`、`*_rosparam.yaml`、`*_recorded_topics.txt`、`*_topic_info/`等 |
| [record_spmpc_mainline_ground_smoke.sh](record_spmpc_mainline_ground_smoke.sh)、[record_spmpc_experiment.sh](record_spmpc_experiment.sh) | 历史轻量/手动录制入口；不能替代新四组的配置冻结和完整预测记录 |

I0/odom区分以`/spmpc/debug/slosh_observer_selection`的nominal/effective source、fallback和epoch为准，不只看启动参数。B0可以旁路记录观察器，记录存在不等于参与液体决策。

### 录包后的常用命令

以下从仓库根目录执行；`SPMPC_SCRIPTS`沿用上面的定义。只读bag的命令仍会创建分析输出目录。

```bash
python3 "$SPMPC_SCRIPTS/summarize_spmpc_real_trial.py" /path/run.bag
python3 "$SPMPC_SCRIPTS/analysis/plot_spmpc_full_da_diagnostics.py" \
  /path/run.bag --output-dir /path/new_diagnostic_plots
python3 "$SPMPC_SCRIPTS/analysis/analyze_robot_state_prediction.py" \
  /path/run.bag --reference-config /path/robot_state_reference.json \
  --output-dir /path/new_state_prediction
python3 "$SPMPC_SCRIPTS/analysis/analyze_internal_slosh_pair.py" \
  --bag /path/full.bag --report /path/full_internal.json
python3 "$SPMPC_SCRIPTS/analysis/analyze_internal_slosh_pair.py" \
  --compare /path/full_internal.json /path/smooth_internal.json \
  --output /path/pair.json
```

- summary输出`*_summary.json/md`，缺诊断字段会列red flags，不是新四组的独立验收器。
- 六图依次为时间、命令链、命令与实际、未来预测、液体、代价/频谱。代价图有近似诊断口径，不能替代精确OCP重构。
- 车体reference模板在[robot_state_reference.template.json](../config/experiments/robot_state_reference.template.json)，必须填写本场地/Tracker信息；该历史工具的模型与bag版本要核对。
- internal指标是IMU/odom驱动的模型响应，不是RGB真值。冻结评价用`freeze_internal_slosh_evaluation.py create --full-report ... --smooth-report ... --primary-monitor imu|odom --reason ... --output ...`；`check`供复验行核对。
- [audit_ocp_cost_and_anticreep.py](analysis/audit_ocp_cost_and_anticreep.py)输入为`{snapshots:[...]}` **JSON**，含x0和完整stage_parameters，输出JSON；调用为`python ... input.json output.json`，依赖匹配的真实acados生成物。这是同快照gain=8/0重优化，不是通用bag精确代价分析，也不能直接替代diag的exact-cost脚本。
- [analyze_horizon_future_liquid_alignment.py](analysis/analyze_horizon_future_liquid_alignment.py)是历史G3R2 W5/两包Smooth投影训练链，需要相应postflight；`--bag`和`--training-bag`各可重复指定，`--output`必须是新目录。后接[summarize_horizon_future_liquid_alignment.py](analysis/summarize_horizon_future_liquid_alignment.py)的`--analysis-dir`。不要用它冒充任意新schema8 bag的验收。

## 场地和执行器辨识

| 阶段 | 脚本 | 输入 / 输出及边界 |
| --- | --- | --- |
| 场地地图冻结 | [freeze_spmpc_mocap_field_map.sh](freeze_spmpc_mocap_field_map.sh)、[validate_mocap_field_map.py](analysis/validate_mocap_field_map.py) | 使用[mocap_field_mapping.env.example](mocap_field_mapping.env.example)；真实冻结需ARM_MAP_FREEZE=YES，写地图三件套/SHA；validator只读 |
| 定位静止检查 | [validate_mocap_field_localization.py](analysis/validate_mocap_field_localization.py) | 核对磁盘地图、Cartographer参数、TF/话题和静止稳定性，不发运动 |
| S路径准备/选择 | [prepare_spmpc_mocap_s_path.sh](prepare_spmpc_mocap_s_path.sh)、[validate_mocap_s_path.py](analysis/validate_mocap_s_path.py)、[run_spmpc_mocap_path_selection_trial.sh](run_spmpc_mocap_path_selection_trial.sh) | 准备/冻结路径；selection是会运动的PATH_SELECTION试验，不占R01–R05 |
| 静止/执行链 | [record_spmpc_mocap_static_smoke.sh](record_spmpc_mocap_static_smoke.sh)、[run_spmpc_mocap_execution_chain_trial.sh](run_spmpc_mocap_execution_chain_trial.sh) | 前者只录静止；后者R01–R05会运动，需ARM_MOTION=YES；后接[bag验收](analysis/validate_mocap_execution_chain_bag.py)、[执行链分析](analysis/analyze_mocap_execution_chain.py) |
| 单阶跃/矩阵 | [run_spmpc_mocap_velocity_step_trial.sh](run_spmpc_mocap_velocity_step_trial.sh)、[矩阵入口](run_spmpc_mocap_velocity_step_matrix_trial.sh) | TEST_AXIS、STEP_MAGNITUDE、STEP_DIRECTION、阶段时长/输出；解锁后直接发速度；[单包分析](analysis/analyze_mocap_velocity_step.py)和[矩阵汇总](analysis/summarize_mocap_velocity_step_matrix.py) |
| 连续性/频谱 | [run_spmpc_mocap_velocity_continuity_trial.sh](run_spmpc_mocap_velocity_continuity_trial.sh)、[连续性分析](analysis/analyze_mocap_velocity_continuity.py)、[频谱分析](analysis/analyze_velocity_continuity_spectrum.py) | runner会发平滑测试命令，分析只读；对应9月2/8日速度连续性诊断 |
| 硬件加速度脉冲 | [run_spmpc_mocap_hardware_accel_limit_trial.sh](run_spmpc_mocap_hardware_accel_limit_trial.sh) | HARDWARE_TEST_ID=H01/H02/H03，默认VALIDATE_ONLY=true；实际解锁需ARM_MOTION/CONFIRM_TEST_SETUP，属于专门硬件辨识，不是普通低速路径测试 |
| NOKOV旋转中心/相对延迟 | [NOKOV README](../../../sensors/nokov_mocap_monitor/README.md) | 当前分支已有9月11日`run_mocap_spin_center.sh`及离线拟合；运动入口与纯recorder分开，具体参数见所属README |
| RGB相机/在线液面 | [prepare_spmpc_g3_realsense.sh](prepare_spmpc_g3_realsense.sh)、[timestamp health](validate_realsense_timestamp_health.py)、[静态录制诊断](diagnose_spmpc_rgb_recording.py) | 前者配置相机；后两者检查时间/吞吐。静态诊断`--modes no_record,rgb,full --duration-sec 30 --out-dir ...`需要已运行ROS和相机，不发速度 |
| RGB标定/离线真值 | [RealSense README](../../../sensors/realsense_liquid_measurement/README.md) | 复用原RGB/max-LCR或对应v2标定链，保留本次ROI/尺度/零点与版本，不把不同测量口径直接混合 |

地图顺序：静止smoke → 建图并冻结 → 带地图SHA重启定位 → 定位校验 → 冻结S路径 → static smoke → 单条运动。详细命令见[动捕场地SOP](../../../../../docs/实物实验注意事项/对比试验/实物对比实验/20260828_动捕场地建图定位与双场地切换SOP.md)。Cartographer负责控制TF；NOKOV只作外部测量，不替换odom。

## 新两层主线的离线与录制入口

| 入口 | 输入 / 输出 | 行为 |
| --- | --- | --- |
| [generate_trajectory_plan.py](generate_trajectory_plan.py) | 完整任务JSON → plan.json；`--warm-plan`可提供种子，`--validate-plan`把输入当已有plan并输出重新传播报告 | 无ROS，求解或验证全程计划 |
| [extract_trajectory_checkpoint.py](extract_trajectory_checkpoint.py) | 一条schema8 PredictedHorizon JSON/YAML → checkpoint.json；`--stage`默认末端 | 无ROS；要求28维完整slosh状态，不能从B0虚构液体 |
| [diagnose_trajectory_suffix.py](diagnose_trajectory_suffix.py) | plan.json + checkpoint.json → suffix.json；`--nominal-only`只重放名义后缀 | 无ROS，默认还尝试重优化；结果不改在线命令 |
| [record_trajectory_mpcc_comparison.sh](record_trajectory_mpcc_comparison.sh) | profile、region、任务覆盖层、planned组计划 → bag/manifest/artifacts | 只录已有节点，VALIDATE_ONLY只校验输入；具体环境变量见[配置README](../config/experiments/trajectory_mpcc/README.md) |
| [仓库根run_native_tests.sh](../../../../../scripts/run_native_tests.sh) | 默认stub；`--with-acados`使用真实solver | 独立CMake，不启动ROS；stub不验证真实OCP |
| [仓库根run_geometry_trials.py](../../../../../scripts/run_geometry_trials.py) | 三任务×四模式×两执行器条件 → CSV、报告、horizon/检查点 | 模型闭环，非实物；`--output-dir`必须新建，失败组保留 |

上层任务样例与ROS启动overlay是不同文件。复现从仓库根目录执行，使用已安装CasADi的Python：

```bash
SPMPC_PYTHON=/home/zrj/.cache/scout_spmpc_dev/venv/bin/python
"$SPMPC_PYTHON" "$SPMPC_SCRIPTS/generate_trajectory_plan.py" \
  test/native/scenarios/corner.json /path/new_plan.json
"$SPMPC_PYTHON" "$SPMPC_SCRIPTS/generate_trajectory_plan.py" \
  /path/new_plan.json /path/validation.json --validate-plan
python3 "$SPMPC_SCRIPTS/extract_trajectory_checkpoint.py" \
  /path/horizon.yaml /path/checkpoint.json
"$SPMPC_PYTHON" "$SPMPC_SCRIPTS/diagnose_trajectory_suffix.py" \
  /path/new_plan.json /path/checkpoint.json /path/suffix.json --nominal-only
```

依赖、真实solver生成和24组复现集中见[native README](../../../../../test/native/README.md)。计划/后缀输出可能覆盖同名文件，应使用新路径。

当前有效预测的结构应为backend=`continuous_mpcc_acados_explicit_actuator`、schema8、60阶段/61状态；B0/slosh为NX24/28、NU3、NP92/104。读取`model_states`和各宽度/版本字段，不能再用旧`61×10`或NP23/32校验。参数、初态、模型版本和真实发布审计要一起保留。

## 其余可复用脚本：按专题回溯

这些是历史协议或专门诊断，不等同于新四组实验。脚本中的冻结路径、阈值、模型版本和报告角色以其源码及对应历史记录为准；普通shell未必实现`--help`，先阅读参数段。

| 专题 | 运行入口 | 离线/验收入口 |
| --- | --- | --- |
| B0延迟/同包辨识 | [B0 delay trial](run_spmpc_mocap_b0_delay_mode_trial.sh) | [summary验收](analysis/validate_mocap_b0_delay_mode_summary.py)、[同包执行器延迟](analysis/analyze_same_bag_actuator_delay.py)、[delay phase](analyze_spmpc_delay_phase.py)、[早期互相关](analysis/estimate_cmd_odom_delay.py) |
| I0/O0/I1/L22状态与nowcast | [shadow](run_spmpc_mocap_slosh_nowcast_shadow_trial.sh)、[delay diagnostic](run_spmpc_mocap_slosh_delay_diagnostic_trial.sh)、[replay](run_spmpc_slosh_nowcast_replay.sh) | [同包分析](analysis/analyze_slosh_nowcast_same_bag.py)、[shadow验收](analysis/validate_slosh_nowcast_shadow_bag.py)、[replay验收](analysis/validate_slosh_nowcast_replay.py) |
| I0 ABBA与显式执行器 | [fixed](run_spmpc_i0_failclosed_fixed_abba_trial.sh)、[short100](run_spmpc_i0_failclosed_fixed_short100_abba_trial.sh)、[explicit](run_spmpc_i0_failclosed_explicit_actuator_abba_trial.sh)、[WS1/WA03](run_spmpc_i0_failclosed_explicit_actuator_ws1_wa03_abba_trial.sh)、[RGB ABBA](run_spmpc_ws1_wa03_rgb_abba.sh) | [ABBA bag验收](analysis/validate_i0_failclosed_fixed_abba_bag.py)、[RGB分析](analysis/analyze_i0_failclosed_fixed_abba_rgb.py)、[runtime验收](analysis/validate_explicit_actuator_runtime_smoke.py) |
| O0/L22与短时域 | [B0/Bslosh](run_spmpc_o0_l22_b0_bslosh_trial.sh)、[Smooth/Ours](run_spmpc_o0_l22_bsmooth_bours_trial.sh)、[short horizon](run_spmpc_short_horizon_matched_trial.sh)、[weight smoke](run_spmpc_weight_smoke.sh) | [短时域验收](analysis/validate_short_horizon_matched_bag.py) |
| G2S/G2C source选择 | [G2S](run_spmpc_g2s_h0s_source_selection_trial.sh)、[G2C](run_spmpc_g2c_processed_imu_w2w5_trial.sh)、[分析wrapper](analyze_spmpc_g2s_source_selection.sh)、[三包RGB分析wrapper](analyze_spmpc_g2s_raw_rgb_three_trial.sh) | [source分析](analysis/analyze_g2s_source_selection.py)、[三包RGB](analysis/analyze_g2s_raw_rgb_three_trial.py)、[G2S验收](analysis/validate_g2s_paired_trial.py)、[G2C验收](analysis/validate_g2c_processed_imu_trial.py) |
| G3/G3R/G3R2 | [G3](run_spmpc_g3_processed_imu_w5_vs_bsmooth_trial.sh)、[G3R](run_spmpc_g3r_weight_screen_trial.sh)、[G3R2筛选](run_spmpc_g3r2_weight_screen_trial.sh)、[配对复验](run_spmpc_g3r2_paired_confirmation_trial.sh)、[robot smoke](run_spmpc_g3r2_robot_only_smoke_trial.sh)、[失败行续接](continue_spmpc_g3_after_failed_row.sh) | [G3](analysis/analyze_g3_w5_vs_bsmooth.py)、[G3R](analysis/analyze_g3r_weight_screen.py)、[G3R2筛选](analysis/analyze_g3r2_weight_screen.py)、[配对](analysis/analyze_g3r2_paired_confirmation.py)、[状态对齐](analysis/analyze_g3_delay_state_alignment.py)、[RGB验收](analysis/validate_g3_online_rgb_trial.py) |
| G4/G5/G6历史发布链 | [G4](run_spmpc_g4_from_g3.sh)、[G5准备](prepare_spmpc_g5_comparators.sh)、[G5](run_spmpc_g5_minimal_trial.sh)、[G6冻结准备](prepare_spmpc_g6_freeze.py) | [G4 replay](analysis/g4_replay_from_g3.py)、[G5准备实现](analysis/prepare_g5_comparators.py)、[G5分析](analysis/analyze_g5_minimal.py)、[G5验收](analysis/validate_g5_minimal_trial.py) |
| 外部/固定profile基线 | [external baseline](run_external_baseline_real_fixed_path_trial.sh)、[fixed profile](run_fixed_profile_real_trial.sh) | LT-DWA/TEB/MPC或profile运行，可能发速度；不是当前raw MPCC内部基线 |
| 早期smoke/权重扫描 | [phase3](phase3_smoke.sh)、[phase4](phase4_fixed_path_run.sh)、[continuous real](run_continuous_real.sh)、[continuous smoke](verify_continuous_smoke.sh)、[B0/Bslosh smoke](compare_b0_bslosh_smoke.sh)、[sweep](sweep_w_slosh.sh) | [variant分析](analyze_b0_bslosh_compare.py)、[sweep汇总](sweep_w_slosh_summary.py)、[omega快查](check_omega_smoke.py)；仿真脚本也会发布命令，只连接对应仿真环境 |
| 旧正式冻结 | [validate_spmpc_formal_freeze.py](validate_spmpc_formal_freeze.py) | 旧SMPCC-REAL-40-88-v1.0合同；不放行v2.0，也不放行新trajectory四组 |
| 采集独立验收 | [validate_spmpc_comparison_recording.py](analysis/validate_spmpc_comparison_recording.py)、[validate_spmpc_ablation_smoke.py](analysis/validate_spmpc_ablation_smoke.py) | 最近C03 wrapper调用的postflight，不等同于新trajectory manifest校验 |

仿真批量矩阵/指标脚本在[spmpc_experiments/scripts README](../../spmpc_experiments/scripts/README.md)，不承担实物主入口。固定路径publisher、模板/profile生成器在[scout_local_planner/scripts README](../../scout_local_planner/scripts/README.md)。

## 无运动回放

新主线已提供外部最终命令历史入口。使用离线独立 ROS1 master，设置 `/use_sim_time=true`，按原 bag 对应的 profile、region、task/plan 启动 `trajectory_mpcc.launch`，额外传入：

```bash
publish_cmd_vel:=false command_history_source:=external_audit \
external_audit_topic:=/spmpc/replay/control_cycle_audit
```

若同时用 `record_trajectory_mpcc_comparison.sh`录制回放重算结果，当前还须在同一planner overlay中声明：

```yaml
command_history:
  source: external_audit
  external_audit_topic: /spmpc/replay/control_cycle_audit
```

launch加载该文件（`planner_overlay_file:=/path/replay_overlay.yaml`）并保留上面的显式source/topic参数；recorder传 `PLANNER_OVERLAY_FILE=/path/replay_overlay.yaml PUBLISH_CMD_VEL=false`。已有overlay应合入这两个字段，其他profile、region、task/plan参数仍须匹配。**目前需要两边同时传同值**：launch最后会覆盖YAML的history字段，recorder尚无对应环境变量；只改launch会被live核对拒绝，只改YAML会被launch默认published覆盖。`LAUNCH_ARGS`只记备注，不能代替参数合并。纯文件核对已验证不匹配时拒绝、上述匹配配置通过；ROS1实际回放待联调，见[主线复盘](../../../../../docs/实物实验注意事项/后续改进/20260916_当前主线复盘与剩余缺口.md)。

播放时只选择状态、路径、TF 和原始审计消息；本节点审计输出与输入分开：

```bash
rosbag play --clock --pause recorded.bag \
  --topics /odom /imu/data /tf /tf_static /scout/global_path_fixed /spmpc/debug/control_cycle_audit \
  /spmpc/debug/control_cycle_audit:=/spmpc/replay/control_cycle_audit
```

topic 名按原录制参数替换。放开暂停后，先等真实历史覆盖线/角速度延迟和加速度记忆，再观察 solve/audit；空历史、断流、过去的 watchdog 大间隙继续失败关闭。外部源只允许一个 publisher；时钟回跳、cycle 倒序或来源冲突要求重启节点及任务，不支持同一节点循环播 bag。`execution_model/require_complete_history` 必须为 true。只有 `/cmd_vel`、没有 schema 2 audit 的旧 bag 不能使用此入口。

这用于在录制状态上重新求解，重算命令不改变录制的后续运动，不能作为闭环改善证据。当前只完成消息接线静态审查与 ROS 无关的时间/历史接纳测试；ROS1 整节点编译、TF 和 bag 联调仍待对应环境验证。

## 实现模块与开发工具

- `planning/`是上层任务、优化、验证和后缀实现；`experiments/recording_contract.py`由新recorder调用，不再新增平行录制框架。
- `lib/`是被wrapper source的公共引擎/配置，不应当独立运行。
- `analysis/*_core.py`、`horizon_liquid_replay.py`、`rotating_liquid_replay.py`、`ocp_snapshot_contract.py`、`liquid_cost_window_contract.py`和`i0_failclosed_fixed_abba_profile.py`是分析/合同模块。`analysis/publish_mocap_velocity_step.py`及`publish_mocap_velocity_continuity.py`会发速度，应由对应runner管理。
- [acados/generate_spmpc_acados.py](acados/generate_spmpc_acados.py)提供`--model b0|slosh`和`--check`；同目录model/cost/constraints/planning_terms等负责装配。`generate_slosh_kernel.py`、`generate_cost_kernel.py`同步生成共源C代码，生成物变化需重建。B0/slosh应串行生成，详见native README。
- `tests/`与`experiments/test_recording_contract.py`供开发回归，部分依赖ROS1/rosbag，不能把无ROS的定向测试写成整包验收。
