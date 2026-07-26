# S-MPCC 正式实物实验启动与录制命令：v2.0 五条件协议

> 候选协议 ID（仅在最终冻结 `n=8` 时成立）：`SMPCC-REAL-40-64-88-v2.0`
>
> 版本日期：2026-07-26
>
> 当前状态：**development/smoke 可按各自 gate 执行；所有 formal Stage I/II trial NO-GO。**
>
> 适用矩阵：[0717_S-MPCC正式实物实验矩阵_先40后88.md](./0717_S-MPCC正式实物实验矩阵_先40后88.md)
>
> 本文档 supersede 旧 v1.0 的 S1/E2、S1/E3 和重复 Bslosh 命令。旧命令只能从 Git 历史查阅，不得用于 v2.0 formal 数据。

本文件只规定命令合同和现场顺序。矩阵、随机化、统计和 failure 规则以配套矩阵文档为准。

---

## 1. 当前能力与硬性 NO-GO

### 1.1 当前事实

| 功能 | 当前实现 | v2.0 formal 状态 |
| --- | --- | --- |
| 基础传感器/定位栈 | `launch_real_sensors_stack.sh` | 可 smoke |
| frozen JSON replay | `fixed_global_path_runner.py` | 可 smoke，路径尚未正式冻结 |
| B0/Bsmooth/Bslosh online backend | `spmpc_experiment.launch` | 候选可运行，release 未冻结 |
| SmoothMatch | `B_smooth` + `v_ref` override | 候选可运行，值未冻结 |
| Hamaguchi profile generator | `scout_profile_baselines` | development/sim 候选 |
| FixedProfile current suite | `run_fixed_path_profile_baseline_suite.sh` + `slosh_experiment_sim.launch` | **不是实物 formal runner** |
| recorder | `record_spmpc_full_rgb_bag.sh` | online topics 基本可用；FixedProfile condition contract 未验收 |
| freeze validator | `validate_spmpc_formal_freeze.py` | 仍硬编码旧 v1.0/E2/E3，拒绝 v2.0 |
| manifest | 只有旧 template | 无 `freeze_manifest.yaml/FREEZE_ID` |
| randomization | v2.0 文件不存在 | NO-GO |
| `s_proj/t(σ)` trajectory extractor | 未冻结 | NO-GO |
| online-input/zero-state tool | 未验收 | NO-GO |
| longitudinal/lateral four-phase tool | 未冻结 | NO-GO |
| K6 | 旧 32-unit 协议 | 与新 8/16/24 不兼容 |

因此即使 online 条件单独能运动，也不能把任何 run 标为 v2.0 formal。

### 1.2 明确禁止

- 禁止继续使用旧 `S1/E2`、`S1/E3`、`S2A/E1`、`S2B/E2` 标签；
- 禁止在 Stage I 采两次 Bslosh；
- 禁止把 `FixedProfile` 伪装成 `VARIANT=B_fixed` 或某个 S-MPCC variant；
- 禁止在实物 formal 中调用 `slosh_experiment_sim.launch`；
- 禁止用 current-sim suite 产物冒充 formal FixedProfile；
- 禁止操作者手工输入本应来自只读 manifest 的最终权重、`v_ref`、C2 参数或 profile hash；
- 禁止在 fixed profile formal trial 现场生成、修改或 time-warp profile；
- 禁止旧 validator 通过后把 run 标成 v2.0 formal。

---

## 2. 每个终端的公共环境

开发机或现场机统一使用显式 workspace，不修改 `HOME`：

```bash
export SCOUT_WS="${SCOUT_WS:-${HOME}/scout_ws}"
source /opt/ros/noetic/setup.bash
source "${SCOUT_WS}/devel/setup.bash"
cd "${SCOUT_WS}"

git status --short
git rev-parse HEAD
rospack find spmpc_local_planner
rospack find scout_profile_baselines
```

formal revision 必须等于 manifest 中的 revision，且工作树没有未归档修改。当前仅做 development 时，也要把 revision 和状态写入 sidecar。

---

## 3. 实物基础系统

现场优先使用统一基础栈：

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

另开终端检查：

```bash
timeout 10s rostopic hz /odom
timeout 10s rostopic hz /scan_front
timeout 10s rostopic hz /imu/data
timeout 5s rosrun tf tf_echo map base_link
timeout 10s rostopic hz /camera/color/image_raw
rostopic echo -n 1 /camera/color/camera_info
rostopic info /cmd_vel
df -h "${HOME}/slosh_bags"
```

开始 trial 前 `/cmd_vel` 不得存在旧 planner/tracker publisher。基础栈退出后不得继续。

RealSense 在同一 development/formal batch 中保持运行。曝光、增益、白平衡和安装姿态不得在 block 中途修改。

---

## 4. Formal 前 development 命令边界

### 4.1 G1：method release

current 与 rotation-consistent candidate 必须先通过冻结的 rotation-relevance replay，比较 modal propagation、first action 和 `t(σ)/v(σ)`。若差异超过预冻结可辨识门槛，必须先完成 rotation-only 小规模 RGB pilot，再选择唯一 release。当前没有可直接宣告通过的现场命令。

输出至少包括：

- candidate revision/config hash；
- high-curvature 与 curvature-reversal cases；
- relevance thresholds；
- 选择的唯一 release；
- rejected release 的归档记录。

未生成唯一 release 时，后续只能做 development。

### 4.2 G2：candidate screening

旧 one-click runner 可用于 H0 development screening：

```bash
export SCOUT_WS=/home/geist/scout_ws
export DEV_DATE="${DEV_DATE:-20260726}"
: "${DEV_RELEASE_ID:?先导出预注册的 development release ID}"
: "${G2_PATH_JSON:?先导出冻结的 H0 路径绝对路径}"
: "${G2_PATH_SHA256:?先导出 development registry 中的 H0 SHA-256}"
: "${PILOT_METHOD:?从 G2 development 随机表读取 B0/Bsmooth/W<number>}"
: "${G2_BLOCK:?从 G2 development 随机表读取 block ID}"
: "${G2_ORDER_POSITION:?从 G2 development 随机表读取 position}"
: "${G2_BLOCK_SEGMENT_ID:?导出 G2 block segment ID}"
test -s "${G2_PATH_JSON}"

export RUN_LABEL="DEV_G2_H0_C1_${PILOT_METHOD}_b${G2_BLOCK}_r01"
export RUN_OUT_DIR="${HOME}/slosh_bags/real/${DEV_DATE}_spmpc_development/G2/${PILOT_METHOD}"

unset MATRIX_PRESET

PILOT_MODE=true \
PILOT_METHOD="${PILOT_METHOD}" \
PILOT_CONDITION="${PILOT_METHOD}" \
PATH_SOURCE_MODE=replay \
PATH_FILE="${G2_PATH_JSON}" \
PATH_EXPECTED_SHA256="${G2_PATH_SHA256}" \
REQUIRE_PATH_HASH=true \
BLOCK_SEGMENT_ID="${G2_BLOCK_SEGMENT_ID}" \
ORDER_POSITION="${G2_ORDER_POSITION}" \
SPLIT_BLOCK=false \
ACQUISITION_RETRY=false \
RUN_LABEL="${RUN_LABEL}" \
RUN_OUT_DIR="${RUN_OUT_DIR}" \
DATE="${DEV_DATE}" \
PILOT_RECORD_RGB=false \
RECORD_TOPIC_INFO=true \
RECORD_STANDALONE_SLOSH=true \
RECORD_SCAN=true \
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
OPERATOR_NOTE="development G2 release=${DEV_RELEASE_ID}" \
bash src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_real_fixed_path_trial.sh
```

这类 no-RGB screen 只能筛安全、trajectory mechanism、tracking 和 runtime。它不能替代 G3，也不能凭 `H_modal` 宣告物理有效。当前 one-click runner 不负责启动/reset standalone monitor、生成 `T_SETTLE` sidecar 或验证 development release/config hash；在这些能力进入冻结的 development wrapper 前，上述命令只能作执行器 smoke，不能单独签署 G2 gate。

是否继续使用旧 W1/W2/W5 × 3 的 15-unit 设计，必须由新的 development registry 明确决定；旧 v1.0 顺序不自动继承为 v2.0 冻结事实。

### 4.3 G3：独立 RGB efficacy pilot

第一条 G3 run 前必须已有：

- 一个最终 Bslosh candidate；
- held-out formal H1 之外的 H0/H0b；
- 精确 `n_dev` 和两条件随机表；
- $\delta_{H,dev}$、success、tracking、runtime 和 no-early-stop 规则；
- 两条件公平的调试/调参预算与 single-block-dominance 规则；
- RGB/同步/visual-start QC 版本；
- 新 development release ID。

当前默认建议 `n_dev=4`，但只有 preregistration 文件可以决定实际值。

one-click runner 只可作为 development 执行器；示例条件映射：

| `DEV_CONDITION` | `PILOT_METHOD` | RGB |
| --- | --- | ---: |
| `Bsmooth` | `Bsmooth` | true |
| `Bslosh` | 最终 `W<number>` | true |

```bash
: "${DEV_DATE:?先导出 development 日期}"
: "${DEV_RELEASE_ID:?先导出预注册的 development release ID}"
: "${G3_PATH_JSON:?先导出冻结且未进入 formal 的 H0b 路径绝对路径}"
: "${G3_PATH_SHA256:?先导出 development registry 中的 H0b SHA-256}"
: "${DEV_CONDITION:?从 G3 两条件随机表读取 Bsmooth 或 Bslosh}"
: "${G3_BLOCK:?从 G3 development 随机表读取 block ID}"
: "${G3_ORDER_POSITION:?从 G3 development 随机表读取 position}"
: "${G3_BLOCK_SEGMENT_ID:?导出 G3 block segment ID}"
test -s "${G3_PATH_JSON}"

case "${DEV_CONDITION}" in
  Bsmooth)
    export PILOT_METHOD=Bsmooth
    ;;
  Bslosh)
    : "${FINAL_PILOT_METHOD:?Bslosh 条件必须指定已冻结的 W<number> candidate}"
    case "${FINAL_PILOT_METHOD}" in
      W*) ;;
      *)
        echo "FINAL_PILOT_METHOD must be W<number> for Bslosh" >&2
        exit 2
        ;;
    esac
    export PILOT_METHOD="${FINAL_PILOT_METHOD}"
    ;;
  *)
    echo "DEV_CONDITION must be Bsmooth or Bslosh" >&2
    exit 2
    ;;
esac

export RUN_LABEL="DEV_G3_RGB_H0b_C1_${DEV_CONDITION}_b${G3_BLOCK}_r01"
export RUN_OUT_DIR="${HOME}/slosh_bags/real/${DEV_DATE}_spmpc_development/G3/${DEV_CONDITION}"

unset MATRIX_PRESET

PILOT_MODE=true \
PILOT_METHOD="${PILOT_METHOD}" \
PILOT_CONDITION="${DEV_CONDITION}" \
PATH_SOURCE_MODE=replay \
PATH_FILE="${G3_PATH_JSON}" \
PATH_EXPECTED_SHA256="${G3_PATH_SHA256}" \
REQUIRE_PATH_HASH=true \
BLOCK_SEGMENT_ID="${G3_BLOCK_SEGMENT_ID}" \
ORDER_POSITION="${G3_ORDER_POSITION}" \
SPLIT_BLOCK=false \
ACQUISITION_RETRY=false \
RUN_LABEL="${RUN_LABEL}" \
RUN_OUT_DIR="${RUN_OUT_DIR}" \
DATE="${DEV_DATE}" \
PILOT_RECORD_RGB=true \
RECORD_TOPIC_INFO=true \
RECORD_STANDALONE_SLOSH=true \
RECORD_SCAN=true \
RECORD_SEC=90 \
MAX_RECORD_SEC=90 \
V_REF=0.20 \
OPERATOR_NOTE="development G3 RGB efficacy release=${DEV_RELEASE_ID}; condition=${DEV_CONDITION}" \
bash src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_real_fixed_path_trial.sh
```

执行前必须已经遥控回起点、完成相同的 `T_SETTLE` 和安全检查。完整 block 全部完成前不得提前停止，也不得看一个 block 后更换 candidate。与 G2 相同，当前 one-click runner 尚不生成 monitor-reset/`T_SETTLE`/release-hash 证据；在冻结 wrapper 补齐前，该示例只能验证在线执行器与 RGB 录制，不能单独签署 G3 efficacy gate。

### 4.4 SmoothMatch development

SmoothMatch 只允许改变 `B_smooth` 的一个 `v_ref`，并只用冻结的 odometry-derived completion-time 规则选值。候选运行仍映射：

```text
condition_id=SmoothMatchCandidate
planner_variant=B_smooth
w_slosh=0
v_ref=SMOOTH_MATCH_V_REF_CANDIDATE
```

最终 `SMOOTH_MATCH_V_REF` 必须由 future v2.0 manifest 导出，不能在 formal 终端 `read -p` 临时填写。

### 4.5 FixedProfile development

当前只允许做 generator/simulation audit：

```bash
rosrun scout_profile_baselines generate_hamaguchi_profile.py --help
PHASE0_PREFLIGHT_MODE=sim \
bash src/scout_apps/control/spmpc_experiments/scripts/bench_run_phase0_preflight.sh \
  hamaguchi_profile
```

第二条命令只是只读的现有 benchmark Phase-0 审计；当前配置仍把该方法标为非直接 main-table eligible。future formal preflight 必须启用 strict-main-table 语义，而当前会按预期阻断。普通 PASS 不解除本文的 FixedProfile 实物 `NO-GO`。

`run_fixed_path_profile_baseline_suite.sh` 目前是 current-sim suite，不能在实物 formal 调用。正式前必须新增并验收：

1. hard `v_max` 与唯一 unshaped base-profile timing 参数分离；
2. ZV impulse amplitudes/$\Delta T$ 固定；
3. 不对 shaped profile 做 time-warp；
4. `(C1,H1)`、`(C1,L1)` profile 全部预生成并 hash；若保留 Stage II-B，再冻结 `(C2,H1)`；
5. real frozen tracker formal wrapper；共同 tracker 只在技术上适用且已预声明时采用；
6. tracker raw → shared gate/limiter → published `/cmd_vel`；
7. 纵向激励、$a_y\simeq v^2\kappa=v\omega$ 与零/近零名义初态审计；
8. FixedProfile-specific recorder/QC；
9. profile/tracker failure 进入方法失败分母。

在这些接口存在前，本文不提供“可运行的 FixedProfile formal 命令”，以防把 sim/smoke 错标为正式数据。

### 4.6 G4：trajectory、四相位与 replay

当前没有可签署 G4 的唯一命令。future toolchain 必须冻结 longitudinal/lateral checkpoint、相位幅值、数值容差与完整 horizon/first-action 输出；四相位差异必须超过冻结容差。online-input/zero-state replay 只适用于 `Bslosh`，并须从同一不可变 pre-solve snapshot 克隆，online-input branch 复现在线 status、first action 和 raw command。当前 extractor 与 replay 工具均未达到该合同，因此 G4 为 `NO-GO`。

---

## 5. v2.0 formal trial 身份与 fail-closed 检查

### 5.1 Stage/condition 合法组合

每条 formal trial 必须从新随机表读取：

```bash
export PROTOCOL_ID=SMPCC-REAL-40-64-88-v2.0
export STAGE=S1
export GROUP=CORE
export PATH_ID=H1
export CONTAINER=C1
export CONDITION_ID=B0
export BLOCK=01
export ORDER_POSITION=01
export REPEAT=01
export BLOCK_SEGMENT_ID=S1_b01_seg01
export SPLIT_BLOCK=false
export ACQUISITION_RETRY=false
export RETRY_REASON_FILE=""
```

合法组合：

| `STAGE/GROUP` | 路径/容器 | 合法 `CONDITION_ID` | position |
| --- | --- | --- | --- |
| `S1/CORE` | H1/C1 | B0、Bsmooth、SmoothMatch、FixedProfile、Bslosh | 01–05 |
| `S2A/SELECTIVITY` | L1/C1 | Bsmooth、FixedProfile、Bslosh | 01–03 |
| `S2B/TRANSFER` | H1/C2 | Bsmooth、FixedProfile、Bslosh | 01–03 |

```bash
case "${STAGE}/${GROUP}" in
  S1/CORE)
    [[ "${PATH_ID}/${CONTAINER}" == H1/C1 ]] || exit 2
    case "${CONDITION_ID}" in B0|Bsmooth|SmoothMatch|FixedProfile|Bslosh) ;; *) exit 2 ;; esac
    [[ "${ORDER_POSITION}" =~ ^0[1-5]$ ]] || exit 2
    ;;
  S2A/SELECTIVITY)
    [[ "${PATH_ID}/${CONTAINER}" == L1/C1 ]] || exit 2
    case "${CONDITION_ID}" in Bsmooth|FixedProfile|Bslosh) ;; *) exit 2 ;; esac
    [[ "${ORDER_POSITION}" =~ ^0[1-3]$ ]] || exit 2
    ;;
  S2B/TRANSFER)
    [[ "${PATH_ID}/${CONTAINER}" == H1/C2 ]] || exit 2
    case "${CONDITION_ID}" in Bsmooth|FixedProfile|Bslosh) ;; *) exit 2 ;; esac
    [[ "${ORDER_POSITION}" =~ ^0[1-3]$ ]] || exit 2
    ;;
  *) exit 2 ;;
esac
```

Stage II-B 还必须由 manifest 明确导出 `stage2b_enabled=true` 和 trigger pass。

### 5.2 condition 与 backend 分层

```bash
case "${CONDITION_ID}" in
  B0)
    export METHOD_BACKEND=online_mpcc
    export VARIANT=B0
    ;;
  Bsmooth|SmoothMatch)
    export METHOD_BACKEND=online_mpcc
    export VARIANT=B_smooth
    ;;
  Bslosh)
    export METHOD_BACKEND=online_mpcc
    export VARIANT=B_slosh
    ;;
  FixedProfile)
    export METHOD_BACKEND=fixed_profile_tracker
    export VARIANT=not_applicable
    ;;
  *) exit 2 ;;
esac
```

`V_REF`、`W_SLOSH`、container、profile 和 tracker 字段必须由升级后的 validator 从只读 manifest 验证并输出。禁止通过上面的 case 写死最终数值。

FixedProfile 至少需要：

```text
PROFILE_ID
PROFILE_CSV
PROFILE_SHA256
PROFILE_GENERATOR_SHA256
PROFILE_CONFIG_SHA256
PROFILE_ZV_DELTA_T
PROFILE_ZV_AMPLITUDES
PROFILE_BASE_PARAMETER_NAME
PROFILE_BASE_PARAMETER_VALUE
TRACKER_CONFIG
TRACKER_CONFIG_SHA256
```

### 5.3 当前 capability smoke（不是 formal validation）

现有 validator 仍硬编码旧 ID。以下仅检查 future validator 源码是否至少出现 v2.0 能力标记：

```bash
export FREEZE_ROOT="${SCOUT_WS}/docs/实物实验注意事项/对比试验/实物对比实验/freeze"
export FREEZE_MANIFEST="${FREEZE_ROOT}/freeze_manifest.yaml"
export VALIDATOR="${SCOUT_WS}/src/scout_apps/control/spmpc_local_planner/scripts/validate_spmpc_formal_freeze.py"

test -s "${FREEZE_MANIFEST}"
test -x "${VALIDATOR}"
rg -q 'SMPCC-REAL-40-64-88-v2\.0' "${VALIDATOR}"
rg -q 'FixedProfile' "${VALIDATOR}"
rg -q 'S1.*CORE' "${VALIDATOR}"
```

这些检查当前会失败，这是预期的安全行为。不得删除检查后继续。即使将来三条 `rg` 通过，也只证明字符串存在，**不等于 formal validation**。

升级后的 validator 必须被实际调用并以非零退出码 fail closed；成功时保存完整 validation report、manifest hash 和明确的 `FORMAL_FREEZE_VALIDATION=PASS`，再从该报告生成并 hash `RUN_ENV`。它还必须验证 G0–G6、sample size、random table row/hash、release selection、trajectory/RGB/replay smoke、K6=8/16/24 和 condition-specific topic contract。当前尚无这个 v2.0 invocation/export contract，所以本文不虚构命令。

### 5.4 run label

```bash
export RUN_LABEL="${STAGE}_${GROUP}_${PATH_ID}_${CONTAINER}_${CONDITION_ID}_b${BLOCK}_r${REPEAT}"
export RUN_ENV="/tmp/${RUN_LABEL}.env"
```

这里只定义目标文件名，**不是生成步骤**。future upgraded validator/exporter 必须从只读 manifest 和随机表原子生成 `RUN_ENV`，至少包含并验证：

- `PROTOCOL_ID/FREEZE_ID`、stage/group/condition/backend、randomization row/hash；
- path/profile/planner/tracker config hashes、`V_REF/W_SLOSH` 和容器参数；
- `T_SETTLE/T_ADMISSION_MAX/T_RGB_PRE`，其中 `T_RGB_PRE` 不小于 2 s；
- `RAW_CMD_TOPIC/POST_GATE_TOPIC/PUBLISHED_CMD_TOPIC` 及各自 publisher contract；
- `SHARED_EXECUTION_CONFIG`、`SHARED_EXECUTION_CONFIG_SHA256`、`FALLBACK_POLICY`、`FALLBACK_POLICY_SHA256` 和共同 motion/delay limits；
- RGB coverage/QC tool hash、输出目录和 acquisition-retry 字段。

当前仓库没有该 exporter，因此下面多终端 formal 流程仍不可执行。

示例：

```text
S1_CORE_H1_C1_FixedProfile_b01_r01
S2A_SELECTIVITY_L1_C1_Bslosh_b01_r01
S2B_TRANSFER_H1_C2_Bsmooth_b01_r01
```

`r02` 只用于矩阵文档允许的 acquisition failure。原 bag 和原因必须保留。

---

## 6. 多终端正式流程的公共部分

以下命令只说明 future v2.0 runner 的公共顺序。第 5.3 节未通过、或 validator 尚未生成只读 `RUN_ENV` 时，不得执行为 formal。每个终端在启动 ROS 节点前都必须先确认 `RUN_ENV` 可读，且其中的 protocol、freeze 与随机表身份已经由 upgraded validator 验证，不能手写一个同名文件绕过。

### 6.1 终端 A：standalone monitor

每个 trial 前 reset；monitor 只作方法无关支持，不进入任何控制 backend。

```bash
source /opt/ros/noetic/setup.bash
source "${SCOUT_WS}/devel/setup.bash"
source "${RUN_ENV}"

roslaunch slosh_models slosh_monitor.launch \
  odom_topic:=/odom \
  cmd_vel_topic:=/cmd_vel \
  output_namespace:=/slosh \
  container_radius:="${CONTAINER_RADIUS}" \
  liquid_height:="${LIQUID_HEIGHT}" \
  damping_ratio:="${DAMPING_RATIO}" \
  use_parabola_term:=false
```

### 6.2 终端 B：冻结路径 replay，停在人工门

```bash
source /opt/ros/noetic/setup.bash
source "${SCOUT_WS}/devel/setup.bash"
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

看到 `Press Enter` 后不要立即继续。先启动 recorder 和本次 backend，完成 `T_SETTLE` 与配置检查。

### 6.3 终端 C：recorder

online 与 FixedProfile 的公共 recorder 必须先由 v2.0 smoke 证明 condition identity、raw/post/published command 和方法专属 topics 都能落包。

```bash
source /opt/ros/noetic/setup.bash
source "${SCOUT_WS}/devel/setup.bash"
source "${RUN_ENV}"
cd "${SCOUT_WS}"

DATE="${EXP_DATE}" \
CONDITION_ID="${CONDITION_ID}" \
METHOD_BACKEND="${METHOD_BACKEND}" \
VARIANT="${VARIANT}" \
RAW_CMD_TOPIC="${RAW_CMD_TOPIC}" \
POST_GATE_TOPIC="${POST_GATE_TOPIC}" \
PUBLISHED_CMD_TOPIC="${PUBLISHED_CMD_TOPIC}" \
SHARED_EXECUTION_CONFIG="${SHARED_EXECUTION_CONFIG}" \
SHARED_EXECUTION_CONFIG_SHA256="${SHARED_EXECUTION_CONFIG_SHA256}" \
FALLBACK_POLICY="${FALLBACK_POLICY}" \
FALLBACK_POLICY_SHA256="${FALLBACK_POLICY_SHA256}" \
RUN_LABEL="${RUN_LABEL}" \
NAME="${RUN_LABEL}" \
OUT_DIR="${OUT_DIR}" \
RECORD_SEC=90 \
RECORD_RGB=true \
RECORD_CAMERA=true \
RECORD_CAMERA_COMPRESSED=false \
RECORD_DEPTH=false \
RECORD_SCAN=true \
RECORD_STANDALONE_SLOSH=true \
RECORD_ONLINE_LIQUID=false \
RECORD_ALL_EXISTING_TOPICS=false \
RECORD_TOPIC_INFO=true \
ROSBAG_BUFFER_SIZE_MB=4096 \
PATH_SOURCE_MODE=replay \
PATH_FILE="${PATH_JSON}" \
OPERATOR_NOTE="protocol=${PROTOCOL_ID} freeze=${FREEZE_ID} condition=${CONDITION_ID} backend=${METHOD_BACKEND} block=${BLOCK} position=${ORDER_POSITION}" \
bash src/scout_apps/control/spmpc_local_planner/scripts/record_spmpc_full_rgb_bag.sh
```

注意：当前 recorder 可见若干 `/reference/*`、`/profile_cap/*` 和 `/mpc/*` topics，但还没有通过 FixedProfile formal contract smoke，也不会按 `RAW_CMD_TOPIC/POST_GATE_TOPIC/PUBLISHED_CMD_TOPIC` 动态构造录制白名单。`CONDITION_ID/METHOD_BACKEND` 目前只能进入环境快照，旧 summarizer 不会按 v2.0 结构化解析。未经升级的 recorder/sidecar/summarizer 不得签署 FixedProfile trial。

---

## 7. 终端 D：启动 condition backend

### 7.1 online MPCC 条件

只适用于 `B0/Bsmooth/SmoothMatch/Bslosh`：

```bash
[[ "${METHOD_BACKEND}" == online_mpcc ]] || exit 2

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
  solver_backend:="${SOLVER_BACKEND}" \
  v_ref:="${V_REF}" \
  w_slosh:="${W_SLOSH}" \
  slosh_height_max:="${SLOSH_HEIGHT_MAX}" \
  alpha_max:="${ALPHA_MAX}" \
  shared_linear_accel_limit_enable:="${SHARED_LINEAR_ACCEL_LIMIT_ENABLE}" \
  shared_linear_accel_max:="${SHARED_LINEAR_ACCEL_MAX}" \
  shared_angular_limit_enable:="${SHARED_ANGULAR_LIMIT_ENABLE}" \
  shared_angular_rate_max:="${SHARED_ANGULAR_RATE_MAX}" \
  shared_angular_accel_max:="${SHARED_ANGULAR_ACCEL_MAX}" \
  delay_phase_mode:="${DELAY_PHASE_MODE}" \
  delay_phase_linear_delay_sec:="${DELAY_PHASE_LINEAR_DELAY_SEC}" \
  delay_phase_angular_delay_sec:="${DELAY_PHASE_ANGULAR_DELAY_SEC}"
```

开始前保存：

```bash
rostopic echo -n 1 /spmpc/debug/effective_config \
  > "${OUT_DIR}/${RUN_LABEL}_effective_config_before_start.txt"
rosparam get /spmpc_local_planner \
  > "${OUT_DIR}/${RUN_LABEL}_planner_rosparam_before_start.yaml"
rostopic info /cmd_vel
```

upgraded validator 必须核对实际 `v_ref`、`w_slosh`、container 和 release hash。当前旧 `effective_scalar_check` 只适合 online branch。

### 7.2 FixedProfile 条件

当前 formal 命令故意 fail closed：

```bash
if [[ "${CONDITION_ID}" == FixedProfile ]]; then
  echo "NO-GO: formal real FixedProfile runner/tracker/execution-chain validator is not implemented" >&2
  exit 2
fi
```

future branch 必须：

1. 只加载 manifest 指定的 `PROFILE_CSV`；
2. 验证 profile、generator、config 和 tracker SHA-256；
3. 禁止 runtime regeneration；
4. 通过冻结的 real tracker 输出 method-native raw command；共同 tracker 仅在适用且预声明时采用；
5. raw command 再通过共享 gate/limiter/fallback；
6. `/cmd_vel` 只有一个 publisher；
7. 保存 profile index/reference、tracker error/latency/status；
8. 到达、timeout、tracker/profile failure 使用与 online 相同的 failure taxonomy。

不能直接复制 simulation suite 的 `slosh_experiment_sim.launch` 命令。

---

## 8. 开始运动、到达与停止

### 8.1 方法无关静稳门

recorder 与 backend 正常后，先确认适用 backend 的 method-state reset 已由 future v2.0 runner 执行并留有成功 sidecar。当前仓库没有统一 reset contract，这也是 formal `NO-GO`。从 admission 开始到按 Enter 的总等待还必须小于 manifest 的 `T_ADMISSION_MAX`；超时或无法静稳时不得运动，按方法无关 acquisition failure 归档。

```bash
source "${RUN_ENV}"

export RESET_SIDECAR="${OUT_DIR}/${RUN_LABEL}_slosh_reset.txt"
{
  printf 'reset_start_utc=%s\n' "$(date --utc --iso-8601=ns)"
  rosservice call /slosh/reset
  echo 'pass=true'
} > "${RESET_SIDECAR}" 2>&1
grep -Fxq 'pass=true' "${RESET_SIDECAR}"

export SETTLE_SIDECAR="${OUT_DIR}/${RUN_LABEL}_t_settle.txt"
python3 - "${T_SETTLE}" "${SETTLE_SIDECAR}" <<'PY'
import pathlib
import sys
import time

target = float(sys.argv[1])
if target <= 0:
    raise SystemExit("invalid T_SETTLE")
start = time.monotonic()
time.sleep(target)
elapsed = time.monotonic() - start
if elapsed < target:
    raise SystemExit("T_SETTLE incomplete")
pathlib.Path(sys.argv[2]).write_text(
    f"pass=true\ntarget_sec={target}\nelapsed_sec={elapsed}\n",
    encoding="utf-8",
)
PY
grep -Fxq 'pass=true' "${SETTLE_SIDECAR}"
```

按 Enter 前，future runner 还必须留下可审计的运动前 RGB 证据：recorder-ready/首个有效 RGB timestamp、满足 `T_RGB_PRE≥2 s` 的 elapsed gate，以及 bag 闭合后“首个有效 RGB → first effective motion”不少于 `T_RGB_PRE` 的复核 sidecar。当前 recorder/runner 没有这套自动合同，因此人工看到画面或等待约 2 s 不能解除 formal `NO-GO`。

随后确认：

- recorder 已连续保存运动前 raw RGB；
- 起点/航向门合格；
- condition/backend/config/profile hash 与随机表行一致；
- `RAW_CMD_TOPIC/POST_GATE_TOPIC/PUBLISHED_CMD_TOPIC` 与当前 backend 一致、非空且 publisher 合法；
- shared execution/fallback config hash 与 manifest 一致；
- `/cmd_vel` publisher 唯一；
- 安全员、急停和路径走廊就位。

回到终端 B 按 Enter。运动前 RGB 覆盖只证明数据存在；冻结且对 condition label 盲化的 visual-start QC 在采集后判定最终视觉资格。离线 QC 失败时保留原 bag，登记方法无关 acquisition failure，并按 `r02`/split-block 规则处理，不追溯改写成方法失败。

### 8.2 到达或失败后的顺序

1. 到达、方法失败或安全停止；
2. recorder 继续 5 s；
3. Ctrl+C 正常关闭 recorder；
4. 停 planner 或 tracker；
5. 停 path replay；
6. 必要时发布一次零速度；
7. 原始 failure bag 永久保留。

```bash
rostopic pub -1 /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

每个 trial 都重启 online planner 或 FixedProfile tracker，不能在同一进程中切换条件。

---

## 9. 每次 run 的即时验收合同

### 9.1 所有条件公共

必须存在：

- bag、recorded-topics、topic-info 和 run-env sidecars；
- protocol/`FREEZE_ID`、condition/backend、stage/block/position；
- path/config/profile hashes；
- `/cmd_vel`、`/odom`、`/tf`、`/scout/global_path_fixed`；
- raw RGB 与 camera_info；
- method-native raw、post-gate、published command；
- arrival/success/failure/fallback/intervention；
- monitor reset 与 `T_SETTLE` 证据。

当前 recorder/summarizer 尚未满足全部 v2.0 公共 contract，因此必须先补 smoke。

### 9.2 online MPCC 专属

`B0/Bsmooth/SmoothMatch/Bslosh` 还必须有：

```text
/spmpc/status
/spmpc/solver_backend
/spmpc/solver_time_ms
/spmpc/controller_variant
/spmpc/debug/effective_config
/spmpc/debug/command_intervention
/spmpc/debug/warm_start
/spmpc/debug/warm_start_status
/spmpc/debug/predicted_horizon
/spmpc/debug/pre_solve_snapshot
```

`Bslosh` 额外要求：

```text
/spmpc/slosh_height
/spmpc/slosh_horizon_summary
/spmpc/debug/slosh_state
```

online-input/zero-state modal replay 只对 `Bslosh` 运行。其他 online conditions 可做普通 solver reproduction/QC，但不能称为 model-state online-input/zero-state replay。`actual` 已改称 online-input branch，表示 solver 实际收到的内部模型状态，不是真实液体状态。

### 9.3 FixedProfile 专属

future formal FixedProfile 必须有：

```text
profile_id/profile_sha256
profile_generator/config/tracker sha256
reference s_fp,ref(t), v_fp,ref(s), omega_fp,ref(s)
current profile index/progress
tracker status/error/latency
method-native raw tracker command
post-gate command
published /cmd_vel
offline generation report and constraint report
```

不得要求 FixedProfile 提供 `/spmpc/debug/pre_solve_snapshot` 或 OCP horizon，也不得把它的 tracker command 称 solver command。

### 9.4 trajectory 与 RGB

当前 `summarize_spmpc_real_trial.py` 不能从 odom+path 完整重建 `s_proj`、`t(σ)` 和 Z1–Z5。它只能保留为 online runtime/QC helper。

正式验收还需要冻结的新 trajectory pipeline，按条件检查：

- 所有条件：odometry-derived `s_proj`、actual `t(σ)`、Z1–Z5、`v/ω`、braking/re-acceleration 和 constant-speed-scaling residual；
- 所有条件：冻结同步/滤波/微分下的 $a_x\simeq\dot v_{odom}$ 与 $a_y\simeq v_{odom}\omega_{odom}$；
- online MPCC：`s_ocp`、pre-solve input、通用完整 horizon 与 solver 结果；
- `Bslosh`：modal state/horizon、`H_modal` 和 online-input/zero-state replay；
- FixedProfile：profile reference/index/progress 与 tracker state/error/latency；
- 跨层比较：FixedProfile reference、S-MPCC predicted timing、odometry-derived execution；
- RGB full-motion p95、10%–90% sensitivity、post-arrival 5 s；
- missing frame、clipping、visual-start 和同步质量。

### 9.5 failure 与 retry

- solver、tracker、profile execution、tracking、timeout、安全终止：方法失败，不得成功补跑替换；
- fallback 是方法相关事件，必须保留；是否使整条 trial 失败或按冻结事件率/容限判定，以 manifest 规则为准，不能现场决定；
- camera/rosbag 未启动、运动前 RGB 覆盖合同失败、采集后离线 visual-start QC 不合格、错误路径/配置在开始前被发现、方法无关现场侵入：可登记 acquisition failure；
- `r02` 必须保留原 bag/reason，并使用相同 condition/backend/profile/config hash；
- 跨 `block_segment_id` 的 retry 不进入主配对；
- 现场只检查完整性和安全，不查看正式方法排名后改变剩余随机顺序。

---

## 10. 下一次 trial

1. 从相应 v2.0 randomization CSV 读取下一行；
2. 更新 `CONDITION_ID/BLOCK/ORDER_POSITION`，不手工重排；
3. 生成新的 `RUN_LABEL/RUN_ENV`；
4. 验证同一 `FREEZE_ID`、path/config/profile hash；
5. 每个 trial 重启 planner/tracker；
6. monitor reset，等待 `T_SETTLE`；
7. 新 recorder 生成独立 bag；
8. 发生方法失败后仍按随机表继续其余条件；
9. 不根据 RGB、tracking 或 internal-model ranking 改方法、权重、`v_ref` 或 profile。

当前下一步不是运行 Stage I，而是依次完成：

```text
确认唯一 FixedProfile 算法
→ 审计/修改 generator 的唯一 base-profile 参数
→ 实现/冻结 FixedProfile real formal wrapper + shared execution chain
→ 升级 recorder/QC/trajectory extractor
→ 完成并归档 G0–G6 全部 gates
→ 升级 manifest、validator、tests 和 K6
→ 冻结 sample size 与新随机表
→ 生成只读 FREEZE_ID
→ 才允许第一条 S1_CORE formal trial
```
