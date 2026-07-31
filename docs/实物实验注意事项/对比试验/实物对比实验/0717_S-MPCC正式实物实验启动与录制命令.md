# S-MPCC 正式实物实验启动与录制命令：v2.0 五条件协议

> 候选协议 ID（仅在最终冻结 `n=8` 时成立）：`SMPCC-REAL-40-64-88-v2.0`
>
> 版本日期：2026-07-31
>
> 当前状态：**development/smoke 可按各自 gate 执行；所有 formal Stage I/II trial NO-GO。**
>
> 适用矩阵：[0717_S-MPCC正式实物实验矩阵_先40后88.md](./0717_S-MPCC正式实物实验矩阵_先40后88.md)
>
> 本文档 supersede 旧 v1.0 的 S1/E2、S1/E3 和重复 Bslosh 命令。旧命令只能从 Git 历史查阅，不得用于 v2.0 formal 数据。
>
> 2026-07-30 增补：吸收 20260727 G2 诊断和 20260729 IMU 标定证据，新增 ROS1 构建/回放门、`G2S` 输入源选择和 processed-IMU shadow 的自动 `READY` 启动顺序。该增补不增加第六条件，不改变 40 → 64 → 条件性 88。

本文件只规定命令合同和现场顺序。矩阵、随机化、统计和 failure 规则以配套矩阵文档为准。

> 现场优先使用精简版：[20260731_S-MPCC实物实验矩阵现场命令速查.md](./20260731_S-MPCC实物实验矩阵现场命令速查.md)。本文档保留完整协议合同与排错细节。

## 现场实际只用下面 5 条命令

长参数已经封装进脚本。不要拆开手动 `roslaunch` planner、path、recorder 或在线 RGB，否则会绕过 READY、录包和自动停车顺序。

### 1. 启动基础传感器栈（终端 A，保持运行）

```bash
bash /home/geist/scout_ws/src/scout_apps/control/scout_local_planner/scripts/launch_real_sensors_stack.sh
```

### 2. 每个 batch 冻结一次相机参数（终端 B）

```bash
OUT_DIR=/home/geist/slosh_bags/real/20260731_spmpc_g2s_source_selection/camera_params bash /home/geist/scout_ws/src/scout_apps/control/scout_local_planner/scripts/set_realsense_rgb_manual_params.sh
```

### 3. 检查 G2S 配置，不动车

```bash
VALIDATE_ONLY=true G2S_ROW=01 bash /home/geist/scout_ws/src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_g2s_h0s_source_selection_trial.sh
```

### 4. 执行一条 G2S，会自动发速度并自动停止

```bash
ARM_MOTION=YES CONFIRM_RGB_GEOMETRY=YES G2S_ROW=01 bash /home/geist/scout_ws/src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_g2s_h0s_source_selection_trial.sh
```

依次把 `G2S_ROW` 改成 `01`、`02`、`03`、`04`。每次只跑一条；回位并等液体静稳后再执行下一条。

### 5. 四条都 PASS 后分析 odom/IMU

```bash
bash /home/geist/scout_ws/src/scout_apps/control/spmpc_local_planner/scripts/analyze_spmpc_g2s_source_selection.sh
```

当前到这里就停止，不执行 W2/W5、G3 或正式 40 条。

<details>
<summary>展开：完整检查、协议合同和历史命令（现场通常不需要）</summary>

## 0. 2026-07-31 现场命令速查（当前唯一执行入口）

> 本节优先级高于后文旧 development-registry 和 formal 多终端模板。当前只允许完成 **4 条 G2S development trial**；G2C、G3 和 Stage I/II 均未放行。若后文命令与本节冲突，以本节为准，不得自行拼接。

### 0.1 当前命令状态

| 命令 | 当前状态 | 是否会驱动车辆 |
| --- | --- | --- |
| 基础传感器/定位栈 | 可执行 | 启动底盘驱动但不主动发送速度 |
| RealSense 手动参数冻结 | 可执行 | 否 |
| G2S `VALIDATE_ONLY` | 可执行 | 否 |
| G2S `ARM_MOTION=YES` | 当前唯一允许的实车 development 运动 | **是，脚本发布 `/cmd_vel`** |
| 4 条后的 source analyzer | 可执行 | 否 |
| G2C/W2-W5、G3、Stage I/II | **NO-GO** | 不得执行 |

### 0.2 终端 A：启动基础传感器栈

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash

export SCOUT_WS=/home/geist/scout_ws
export SENSOR_LOG_DIR="/home/geist/slosh_bags/real/20260731_spmpc_g2s_source_selection/sensor_stack_$(date +%H%M%S)"

SCOUT_WS="${SCOUT_WS}" \
LOG_DIR="${SENSOR_LOG_DIR}" \
REALSENSE_COLOR_WIDTH=1920 \
REALSENSE_COLOR_HEIGHT=1080 \
REALSENSE_COLOR_FPS=30 \
REALSENSE_ENABLE_DEPTH=false \
REALSENSE_ENABLE_INFRA=false \
WAIT_FOR_ODOM=true \
WAIT_FOR_LOCALIZATION_MAP=true \
bash /home/geist/scout_ws/src/scout_apps/control/scout_local_planner/scripts/launch_real_sensors_stack.sh
```

该脚本会请求一次 `sudo` 配置 `can0`，随后启动底盘、NanoScan3、Cartographer localization、`/imu/data` 和 RealSense。保持终端 A 运行；全部 G2S 完成后才按 `Ctrl+C`，脚本会停止它启动的进程。它不会启动在线液面节点；G2S wrapper 会为每条 trial 单独启动并关闭冻结的在线节点。

### 0.3 终端 B：冻结相机参数并检查基础数据

每个 G2S batch 只执行一次 `freeze_current`，之后四条之间不得改曝光、增益、白平衡、相机姿态、容器姿态或液深：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash

MODE=freeze_current \
OUT_DIR=/home/geist/slosh_bags/real/20260731_spmpc_g2s_source_selection/camera_params \
bash /home/geist/scout_ws/src/scout_apps/control/scout_local_planner/scripts/set_realsense_rgb_manual_params.sh
```

然后检查：

```bash
rostopic echo --noarr -n 1 /odom
rostopic echo --noarr -n 1 /imu/data
rostopic echo --noarr -n 1 /camera/color/camera_info

timeout 5s rostopic hz /imu/data || true
timeout 5s rostopic hz /camera/color/image_raw || true
timeout 5s rosrun tf tf_echo map base_link || true

rosrun dynamic_reconfigure dynparam get /camera/rgb_camera | \
  rg 'enable_auto_exposure|exposure:|gain:|enable_auto_white_balance|white_balance:'
rostopic info /cmd_vel || true
```

必须看到 `1920×1080`、IMU frame 为 `imu_link`、自动曝光和自动白平衡均为 `false`，并且开始 trial 前没有旧 planner 在发布 `/cmd_vel`。若 `map -> base_link` 不存在或跳变，不能执行路径 replay。

### 0.4 终端 B：先做只读检查

该命令只检查路径、标定文件和最终 runner 环境，不启动 ROS 节点、不录 bag、不发布速度：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash

VALIDATE_ONLY=true \
G2S_ROW=01 \
bash /home/geist/scout_ws/src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_g2s_h0s_source_selection_trial.sh
```

输出必须明确包含：

```text
PILOT_METHOD=Bsmooth
PILOT_RECORD_RGB=false
PILOT_RECORD_ONLINE_LIQUID=true
FORBID_IMAGE_STREAMS=true
PATH_FILE=/home/geist/fixed_paths/real/20260727_spmpc_development/H0/H0_G2.json
PATH_EXPECTED_SHA256=578a4dd7663c2f49b4270c37755a08b2b0dc70735fb6b818da35b60a60f3990e
```

### 0.5 终端 B：执行一条 G2S

执行前必须同时满足：车辆已回到旧 G2 起点并对齐、走廊清空、急停可用、液体静稳、0629 calibration 的 ROI/三标尺仍与当前相机和容器几何一致。确认后一次只执行一条：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash

DATE=20260731 \
ARM_MOTION=YES \
CONFIRM_RGB_GEOMETRY=YES \
G2S_ROW=01 \
G2S_ATTEMPT=01 \
bash /home/geist/scout_ws/src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_g2s_h0s_source_selection_trial.sh
```

四条依次把 `G2S_ROW` 改为 `01`、`02`、`03`、`04`，每条的首次 attempt 都是 `G2S_ATTEMPT=01`。脚本行为固定为：

```text
检查相机手动参数与路径/calibration hash
→ 自动启动在线 RGB（publish_debug=false）
→ 等待 zero_locked + valid + status=OK
→ 启动 recorder
→ 启动 Bsmooth planner 和两路 observer
→ 等待 processed-IMU READY
→ replay H0_G2 并发布 /cmd_vel
→ 90 s 录包边界到达后停止 planner/path、发送零速度
→ 自动运行 image-free G2S postflight
```

它不会自动连续执行下一条。任何 `Ctrl+C`、planner/recorder 异常或超时都会进入清理并尝试发送零速度，但现场仍须保持急停就绪。

### 0.6 每条结束后的验收

以 row 01 为例：

```bash
export G2S_DIR=/home/geist/slosh_bags/real/20260731_spmpc_g2s_source_selection/H0s_Bsmooth
export G2S_STEM=DEV_G2S_H0s_C1_Bsmooth_u01_a01

test -s "${G2S_DIR}/${G2S_STEM}.bag"
test ! -e "${G2S_DIR}/${G2S_STEM}.bag.active"
test -s "${G2S_DIR}/${G2S_STEM}_g2s_postflight.json"

rg -n '"status"|"failures"|"recorded_image_topics"|"valid_motion_fraction"|"ready_fraction"' \
  "${G2S_DIR}/${G2S_STEM}_g2s_postflight.json"
```

只有 postflight 顶层 `status` 为 `PASS`、`recorded_image_topics=[]`，且 failures 为空，才能回位、重新等液体静稳并执行下一 row。失败 bag 不删除、不覆盖，也不能直接把同一 row 重跑成 `a01`；先停止并分析 failure/retry 身份。

### 0.7 四条 PASS 后进行唯一 source 分析

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash

python3 /home/geist/scout_ws/src/scout_apps/control/spmpc_local_planner/scripts/analysis/analyze_g2s_source_selection.py \
  --bag-dir /home/geist/slosh_bags/real/20260731_spmpc_g2s_source_selection/H0s_Bsmooth \
  --calibration /home/geist/slosh_bags/real/20260629_calib/red_3ruler.yaml \
  --out-dir /home/geist/slosh_bags/real/20260731_spmpc_g2s_source_selection/analysis
```

读取：

```bash
python3 -m json.tool \
  /home/geist/slosh_bags/real/20260731_spmpc_g2s_source_selection/analysis/G2S_SOURCE_SELECTION_REPORT.json | less
```

decision 只可能是 `odom` 或 `processed_imu`。这仍是 development source decision；生成报告后先停，不得直接接着运行 W2/W5 或 Stage I。

### 0.8 当前禁止复制的命令

- 不单独运行 `rosbag record /camera/color/image_raw`；
- 不设置 `PILOT_RECORD_RGB=true`、`RECORD_CAMERA=true` 或 `RECORD_ALL_EXISTING_TOPICS=true`；
- 不手动启动 `online_liquid_height.launch` 与 G2S wrapper 叠加；
- 不执行本文件第 4.3 节的旧长模板；
- 不执行第 4.4、4.5、6、7、8 节的 G2C/G3/formal 多终端命令；
- 不把任何当前 bag 命名为 `S1_CORE`、`S2A_SELECTIVITY` 或 `S2B_TRANSFER`。

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
| recorder | `record_spmpc_full_rgb_bag.sh`（历史文件名） | 默认在线标量、G2S 强制零图像流；FixedProfile/formal contract 未验收 |
| freeze validator | `validate_spmpc_formal_freeze.py` | 仍硬编码旧 v1.0/E2/E3，拒绝 v2.0 |
| manifest | 只有旧 template | 无 `freeze_manifest.yaml/FREEZE_ID` |
| randomization | v2.0 文件不存在 | NO-GO |
| `s_proj/t(σ)` trajectory extractor | 未冻结 | NO-GO |
| online-input/zero-state tool | 未验收 | NO-GO |
| longitudinal/lateral four-phase tool | 未冻结 | NO-GO |
| K6 | 旧 32-unit 协议 | 与新 8/16/24 不兼容 |
| processed-IMU pipeline | 已实现去重力、静止 bias、因果滤波、gyro 修正、杠臂补偿、显式 source selector、freshness 和 IMU→odom 锁存 fallback | development implementation PASS；输入源尚未由 4 条 G2S 冻结，formal NO-GO |
| odom/IMU 双 observer debug | `/spmpc/debug/slosh_observer_odom`、`/spmpc/debug/slosh_observer_imu` 与 selection diagnostic | 可 development replay/录包，尚未通过在线 RGB source-selection gate |
| 在线 RGB image-free 链 | `/liquid/measurement` + recorder/postflight/analyzer | message/launch/mock/validate-only PASS；真实相机吞吐与 4 条 G2S 未完成 |
| ROS1 整包门禁 | 2026-07-31 已在本机 Ubuntu 20.04.6 + ROS1 Noetic 完成 codegen、构建、168 tests、ABI/link、launch 解析和隔离 bag replay | **development 技术门通过**；工作树非 clean、generated provenance 与 frozen report 尚未签署，formal 仍 NO-GO |

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
- 禁止把 IMU shadow 当成第六个 condition，或把同一 trial 重复计为 odom/IMU 两个样本；
- 禁止 raw `/imu/data` 直接进入液体模型，禁止把当前 IMU 值复制到未来 MPC horizon；
- 禁止在同一 formal release 或 Stage I 中途把液体当前状态输入从 odom 切成 IMU；
- 禁止把 `IMU_SHADOW_ENABLE=true` 解释为 solver 已使用 IMU；working tree 虽有 selector，nominal source 仍须由 G2S 决策和 source-specific release 冻结。
- 禁止 G2S/G3/formal bag 录入 raw/compressed RGB、depth 或 debug image；当前只允许冻结的在线 stamped scalar/quality。

---

## 2. 每个终端的公共环境

### 2.1 基本环境

开发机或现场机统一使用显式 workspace，不修改 `HOME`。本机当前路径如下；若迁移主机，只改 `SCOUT_WS/SLOSH_BAG_ROOT`，不要混用两个 workspace 的 build/devel：

```bash
set -euo pipefail
export SCOUT_WS="${SCOUT_WS:-/home/geist/scout_ws}"
export SLOSH_BAG_ROOT="${SLOSH_BAG_ROOT:-/home/geist/slosh_bags/real}"
test -r /opt/ros/noetic/setup.bash
source /opt/ros/noetic/setup.bash
test -r "${SCOUT_WS}/devel/setup.bash"
source "${SCOUT_WS}/devel/setup.bash"
[[ "${ROS_VERSION:-}" == 1 && "${ROS_DISTRO:-}" == noetic ]]
cd "${SCOUT_WS}"

git status --short
git rev-parse HEAD
rospack find spmpc_local_planner
rospack find scout_profile_baselines
```

formal revision 必须等于 manifest 中的 revision，且工作树没有未归档修改。当前仅做 development 时，也要把 revision 和状态写入 sidecar。

### 2.2 ROS1 Noetic 构建门

本机已经核实为 Ubuntu 20.04.6 + ROS1 Noetic，并在 revision `a2fe0c25cec12bb007612878ff69af17ee92b2a4` 上完成 development 技术门；证据位于 `/home/geist/slosh_bags/real/20260731_spmpc_preformal_gate/`。这不自动签署 formal：正式门仍要求 clean worktree、generated artifact provenance、冻结报告及 manifest 绑定。复跑时 `SCOUT_WS`、`ACADOS_SOURCE_DIR` 和新证据目录必须显式给出，不能依赖操作者用户名：

```bash
set -euo pipefail
: "${SCOUT_WS:?导出 ROS1 workspace 的绝对路径}"
: "${ACADOS_SOURCE_DIR:?导出目标机 acados 安装目录}"
: "${ROS1_GATE_OUT:?导出本次只写证据目录}"
export SCOUT_WS="$(readlink -f "${SCOUT_WS}")"
export ACADOS_SOURCE_DIR="$(readlink -f "${ACADOS_SOURCE_DIR}")"
[[ "${SCOUT_WS}" == /* && "${ACADOS_SOURCE_DIR}" == /* && "${ROS1_GATE_OUT}" == /* ]]

test -r /opt/ros/noetic/setup.bash
source /opt/ros/noetic/setup.bash
[[ "${ROS_VERSION:-}" == 1 && "${ROS_DISTRO:-}" == noetic ]]

export SPMPC_PKG_DIR="${SCOUT_WS}/src/scout_apps/control/spmpc_local_planner"
test -d "${SPMPC_PKG_DIR}"
if [[ -e "${ROS1_GATE_OUT}" ]]; then
  echo "NO-GO: choose a new ROS1_GATE_OUT; existing evidence must not be overwritten" >&2
  exit 2
fi
mkdir -p "${ROS1_GATE_OUT}"
git -C "${SCOUT_WS}" rev-parse HEAD > "${ROS1_GATE_OUT}/git_revision_before.txt"
git -C "${SCOUT_WS}" status --short > "${ROS1_GATE_OUT}/git_status_before.txt"
[[ ! -s "${ROS1_GATE_OUT}/git_status_before.txt" ]]

if [[ -s "${ACADOS_SOURCE_DIR}/lib/libacados.so" ]]; then
  export ACADOS_LIB_DIR="${ACADOS_SOURCE_DIR}/lib"
elif [[ -s "${ACADOS_SOURCE_DIR}/lib64/libacados.so" ]]; then
  export ACADOS_LIB_DIR="${ACADOS_SOURCE_DIR}/lib64"
else
  echo "NO-GO: libacados.so missing under lib/ and lib64/" >&2
  exit 2
fi
export LD_LIBRARY_PATH="${ACADOS_LIB_DIR}:${LD_LIBRARY_PATH:-}"

export B0_SOLVER_SO="${SPMPC_PKG_DIR}/generated/acados/spmpc_b0/libacados_ocp_solver_spmpc_b0.so"
export SLOSH_SOLVER_SO="${SPMPC_PKG_DIR}/generated/acados/spmpc_slosh/libacados_ocp_solver_spmpc_slosh.so"
test -s "${B0_SOLVER_SO}"
test -s "${SLOSH_SOLVER_SO}"

cd "${SPMPC_PKG_DIR}/scripts/acados"
python3 generate_spmpc_acados.py --check --model b0 \
  2>&1 | tee "${ROS1_GATE_OUT}/acados_check_b0.log"
python3 generate_spmpc_acados.py --check --model slosh \
  2>&1 | tee "${ROS1_GATE_OUT}/acados_check_slosh.log"

cd "${SCOUT_WS}"
export BUILD_LOG="${ROS1_GATE_OUT}/catkin_build.log"
catkin_make --force-cmake \
  -DACADOS_SOURCE_DIR:PATH="${ACADOS_SOURCE_DIR}" \
  --pkg slosh_models spmpc_local_planner \
  2>&1 | tee "${BUILD_LOG}"

grep -Fq 'building continuous_mpcc_acados backend (b0 + slosh)' "${BUILD_LOG}"
grep -Fq "acados found at ${ACADOS_SOURCE_DIR}" "${BUILD_LOG}"
export CMAKE_CACHE="${SCOUT_WS}/build/CMakeCache.txt"
test -s "${CMAKE_CACHE}"
grep -Fxq "ACADOS_SOURCE_DIR:PATH=${ACADOS_SOURCE_DIR}" "${CMAKE_CACHE}"
if grep -Eiq 'continuous backend = stub|continuous_mpcc_acados backend \(b0 only|slosh solver missing' "${BUILD_LOG}"; then
  echo "NO-GO: planner silently degraded to stub or B0-only acados" >&2
  exit 2
fi

catkin_make \
  run_tests_spmpc_local_planner_gtest_test_processed_imu_pipeline \
  run_tests_spmpc_local_planner_gtest_test_slosh_observer_bank \
  run_tests_spmpc_local_planner_gtest_test_replay_diagnostics \
  2>&1 | tee "${ROS1_GATE_OUT}/required_test_targets.log"
catkin_make run_tests_spmpc_local_planner \
  2>&1 | tee "${ROS1_GATE_OUT}/package_tests.log"
catkin_test_results --all "${SCOUT_WS}/build/test_results" \
  2>&1 | tee "${ROS1_GATE_OUT}/catkin_test_results.log"

source "${SCOUT_WS}/devel/setup.bash"
[[ "${ROS_VERSION:-}" == 1 && "${ROS_DISTRO:-}" == noetic ]]
[[ "$(readlink -f "$(rospack find spmpc_local_planner)")" == "$(readlink -f "${SPMPC_PKG_DIR}")" ]]

test -s "${SCOUT_WS}/devel/include/spmpc_local_planner/SloshObserverDebug.h"
python3 - <<'PY'
from spmpc_local_planner.msg import SloshObserverDebug
assert SloshObserverDebug is not None
PY

export SPMPC_NODE="${SCOUT_WS}/devel/lib/spmpc_local_planner/spmpc_local_planner_node"
export SPMPC_LIB="${SCOUT_WS}/devel/lib/libspmpc_local_planner.so"
test -x "${SPMPC_NODE}"
test -s "${SPMPC_LIB}"
ldd "${SPMPC_NODE}" | tee "${ROS1_GATE_OUT}/ldd_node.log"
ldd "${SPMPC_LIB}" | tee "${ROS1_GATE_OUT}/ldd_library.log"
! grep -Fq 'not found' "${ROS1_GATE_OUT}/ldd_node.log"
! grep -Fq 'not found' "${ROS1_GATE_OUT}/ldd_library.log"

roslaunch --files spmpc_local_planner spmpc_fixed_path.launch \
  > "${ROS1_GATE_OUT}/launch_files_fixed_path.txt"
roslaunch --files spmpc_local_planner spmpc_experiment.launch \
  > "${ROS1_GATE_OUT}/launch_files_experiment.txt"

sha256sum \
  "${ACADOS_LIB_DIR}/libacados.so" \
  "${B0_SOLVER_SO}" \
  "${SLOSH_SOLVER_SO}" \
  "${SPMPC_NODE}" \
  "${SPMPC_LIB}" \
  > "${ROS1_GATE_OUT}/binary_sha256.txt"
git status --short > "${ROS1_GATE_OUT}/git_status_after.txt"
git rev-parse HEAD > "${ROS1_GATE_OUT}/git_revision_after.txt"
[[ ! -s "${ROS1_GATE_OUT}/git_status_after.txt" ]]
cmp -s \
  "${ROS1_GATE_OUT}/git_revision_before.txt" \
  "${ROS1_GATE_OUT}/git_revision_after.txt"
```

`rosmsg show` 只能显示 ROS 接口，不能替代上面的 generated C++ header 与 Python import 检查。必须归档完整 configure/build/test 日志、三个显式 gtest run target、全包 test result、launch 解析、CMake cache、动态链接、实际 acados/generated `.so` SHA-256，以及构建前后均为空且 revision 未变化的 Git status/revision sidecars。任何缺项均记 `CANNOT_VERIFY / FORMAL NO-GO`；只有非 ROS 核心单测通过不能替代本门。`generate_spmpc_acados.py --check` 只验证 CasADi 装配，不证明现有 generated `.so` 与当前 source/config 同源；生成命令、JSON/C/.so provenance 与 hash 仍必须由 G2A/freeze 单独签署。

### 2.3 `publish_cmd_vel=false` bag replay 门

构建通过后分两类回放；不要启动实车传感器栈或底盘驱动。所有终端都必须使用独立的 `11321` master，不能把默认 `11311` 称为“隔离”。每个回放终端先执行：

```bash
set -euo pipefail
: "${SCOUT_WS:?导出 ROS1 workspace 的绝对路径}"
: "${REPLAY_GATE_OUT:?导出 replay 证据目录}"
export ROS_MASTER_URI=http://127.0.0.1:11321
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME ROS_NAMESPACE
source /opt/ros/noetic/setup.bash
source "${SCOUT_WS}/devel/setup.bash"
[[ "${ROS_VERSION:-}" == 1 && "${ROS_DISTRO:-}" == noetic ]]
mkdir -p "${REPLAY_GATE_OUT}"
```

终端 A 只启动隔离 master：

```bash
export ROS_MASTER_URI=http://127.0.0.1:11321
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME ROS_NAMESPACE
roscore -p 11321
```

所有 freshness-sensitive IMU 回放固定使用 `--rate=1 --hz=1000`。2026-07-31 的实测表明，`rosbag play --clock` 默认 100 Hz 时，IMU header 会周期性领先当前 `/clock` 约 5--12 ms，超过当前 `max_future_skew_sec=0.005`，继而产生伪 `STALE_SAMPLE` 和 `SAMPLE_GAP`；4 倍速回放也不能签本门。

`/use_sim_time=true` 且尚无 `/clock` 时，`rosbag record` 会等待而不创建 `.bag.active`。因此每一轮必须先用输入 bag 的起始时间启动临时 bootstrap clock；确认 recorder 的 `.bag.active` 已出现后停止 bootstrap publisher，再启动唯一的 `rosbag play --clock`。bootstrap 与 player 禁止同时存活：

```bash
: "${REPLAY_INPUT_BAG:?导出本轮输入 bag 的绝对路径}"
read -r BOOTSTRAP_CLOCK_SECS BOOTSTRAP_CLOCK_NSECS < <(
  python3 - "$REPLAY_INPUT_BAG" <<'PY'
import math
import sys
import rosbag

with rosbag.Bag(sys.argv[1], "r") as bag:
    stamp = bag.get_start_time()
secs = math.floor(stamp)
nsecs = round((stamp - secs) * 1e9)
if nsecs >= 1_000_000_000:
    secs += 1
    nsecs -= 1_000_000_000
print(secs, nsecs)
PY
)
rostopic pub -r 20 /clock rosgraph_msgs/Clock \
  "clock: {secs: $BOOTSTRAP_CLOCK_SECS, nsecs: $BOOTSTRAP_CLOCK_NSECS}"
```

该终端保持运行；另一终端确认目标 `.bag.active` 后对它按 `Ctrl+C`，确认 bootstrap publisher 已退出，才允许执行下文 playback。READY 文本文件非空也不等于 READY：`rostopic echo` 在模拟时钟尚未开始时会写 warning；postflight 必须从完整 debug bag 验证实际存在 `input_status=READY && valid && bias_ready && filter_ready`。

#### 2.3.1 0729 `planar_r03`：READY、bias 与时间异常门

显式设置 `REPLAY_CAL_BAG` 为 `imu_mocap_planar_r03_153448.bag`。终端 B：

```bash
: "${REPLAY_CAL_BAG:?导出 0729 planar_r03 bag 绝对路径}"
: "${REPLAY_CAL_BAG_SHA256:?导出冻结的 planar_r03 bag SHA-256}"
test -s "${REPLAY_CAL_BAG}"
CAL_BAG_ACTUAL_SHA256="$(sha256sum "${REPLAY_CAL_BAG}" | awk '{print $1}')"
[[ "${CAL_BAG_ACTUAL_SHA256}" == "${REPLAY_CAL_BAG_SHA256}" ]]
export CAL_INPUT_INFO="${REPLAY_GATE_OUT}/planar_r03_input_bag_info.txt"
test ! -e "${CAL_INPUT_INFO}"
rosbag info "${REPLAY_CAL_BAG}" > "${CAL_INPUT_INFO}"
python3 - "${REPLAY_CAL_BAG}" /imu/data /odom <<'PY'
import sys
import rosbag

bag_path, *required = sys.argv[1:]
with rosbag.Bag(bag_path, "r") as bag:
    info = bag.get_type_and_topic_info()
    topics = info.topics if hasattr(info, "topics") else info[1]
missing = [
    topic for topic in required
    if topic not in topics or getattr(topics[topic], "message_count", 0) <= 0
]
if missing:
    raise SystemExit("missing/empty exact input topics: " + ",".join(missing))
PY
rosparam set /use_sim_time true

roslaunch spmpc_local_planner spmpc_fixed_path.launch \
  planner_variant:=B_smooth \
  reference_path_topic:=/scout/global_path_fixed \
  cmd_vel_topic:=/spmpc/replay/cmd_vel_unused \
  publish_cmd_vel:=false \
  imu_topic:=/imu/data \
  imu_shadow_enable:=true \
  delay_phase_mode:=off \
  2>&1 | tee "${REPLAY_GATE_OUT}/planar_r03_planner.log"
```

终端 C 在播放前记录 raw input 与完整 observer 状态流：

```bash
export CAL_OUTPUT_BAG="${REPLAY_GATE_OUT}/planar_r03_observer.bag"
test ! -e "${CAL_OUTPUT_BAG}"
test ! -e "${CAL_OUTPUT_BAG}.active"
rosbag record -O "${CAL_OUTPUT_BAG}" \
  /clock \
  /odom \
  /imu/data \
  /spmpc/status \
  /spmpc/debug/slosh_observer_odom \
  /spmpc/debug/slosh_observer_imu \
  /spmpc/replay/cmd_vel_unused
```

终端 E 在播放前启动 READY 监听并保存结果：

```bash
timeout 120s rostopic echo -n 1 \
  --filter "m.input_status == 'READY' and m.valid and m.bias_ready and m.filter_ready" \
  /spmpc/debug/slosh_observer_imu \
  | tee "${REPLAY_GATE_OUT}/planar_r03_ready.txt"
```

终端 D：

```bash
rosbag play -q --clock --rate=1 --hz=1000 "${REPLAY_CAL_BAG}" \
  --topics /odom /imu/data /tf /tf_static \
  2>&1 | tee "${REPLAY_GATE_OUT}/planar_r03_playback.log"
```

播放结束后正常关闭终端 C，再执行 `test -s "${CAL_OUTPUT_BAG}"`、`test ! -e "${CAL_OUTPUT_BAG}.active"` 和 `rosbag info "${CAL_OUTPUT_BAG}"`。只保存单条 READY 输出不能签 transient/stamp/gap/reset；这些结论必须来自完整 debug 流与 frozen analyzer。

本门只签署 processed-IMU 能按冻结 frame 进入 `READY`，并复核该 bag 实际包含的 bias/filter transient、stamp、gap 与 reset epoch。重复时间戳、倒退时间戳、clock reset 和人为越过 `35 ms` 阈值若未在 bag 中发生，必须由 `test_processed_imu_pipeline`/synthetic replay 签署，不能从“未观察到异常”反推状态码正确。特别检查 `/imu/data.header.frame_id=imu_link`；旧 `scout_imu.launch` 的默认 `base_link` 会触发 `FRAME_MISMATCH`，不能绕过。

#### 2.3.2 0705 同 bag、shadow off/on 成对不变量门

显式设置同一个 `REPLAY_0705_BAG`，分别以 `SHADOW_MODE=false` 和 `true` 完整回放一次；两次之间停止 planner/recorder、重启隔离 master，并清空 ROS 参数。不得复用一个持续运行的 planner，也不得启动现场一键 runner。每次的终端 B 使用：

```bash
: "${REPLAY_0705_BAG:?导出同一条 0705 Bslosh bag 的绝对路径}"
: "${REPLAY_0705_BAG_SHA256:?导出冻结的 0705 bag SHA-256}"
: "${SHADOW_MODE:?本轮只能为 false 或 true}"
case "${SHADOW_MODE}" in false|true) ;; *) exit 2 ;; esac
test -s "${REPLAY_0705_BAG}"
REPLAY_0705_ACTUAL_SHA256="$(sha256sum "${REPLAY_0705_BAG}" | awk '{print $1}')"
[[ "${REPLAY_0705_ACTUAL_SHA256}" == "${REPLAY_0705_BAG_SHA256}" ]]
export REPLAY_0705_INPUT_INFO="${REPLAY_GATE_OUT}/0705_shadow_${SHADOW_MODE}_input_bag_info.txt"
test ! -e "${REPLAY_0705_INPUT_INFO}"
rosbag info "${REPLAY_0705_BAG}" > "${REPLAY_0705_INPUT_INFO}"
python3 - "${REPLAY_0705_BAG}" \
  /odom /imu/data /map /scout/global_path_fixed <<'PY'
import sys
import rosbag

bag_path, *required = sys.argv[1:]
with rosbag.Bag(bag_path, "r") as bag:
    info = bag.get_type_and_topic_info()
    topics = info.topics if hasattr(info, "topics") else info[1]
missing = [
    topic for topic in required
    if topic not in topics or getattr(topics[topic], "message_count", 0) <= 0
]
if missing:
    raise SystemExit("missing/empty exact input topics: " + ",".join(missing))
PY
export PAIR_INPUT_SIDECAR="${REPLAY_GATE_OUT}/0705_shadow_${SHADOW_MODE}_input.env"
test ! -e "${PAIR_INPUT_SIDECAR}"
printf 'input_bag=%s\ninput_sha256=%s\nshadow_mode=%s\n' \
  "${REPLAY_0705_BAG}" "${REPLAY_0705_ACTUAL_SHA256}" "${SHADOW_MODE}" \
  > "${PAIR_INPUT_SIDECAR}"
rosparam set /use_sim_time true

roslaunch spmpc_local_planner spmpc_fixed_path.launch \
  planner_variant:=B_slosh \
  reference_path_topic:=/scout/global_path_fixed \
  cmd_vel_topic:=/spmpc/replay/cmd_vel_unused \
  publish_cmd_vel:=false \
  imu_topic:=/imu/data \
  imu_shadow_enable:="${SHADOW_MODE}" \
  delay_phase_mode:=fixed_closed_loop \
  2>&1 | tee "${REPLAY_GATE_OUT}/0705_shadow_${SHADOW_MODE}_planner.log"
```

终端 C 必须在 playback 前记录输出，正常关闭后确认没有 `.bag.active`：

```bash
export PAIR_OUTPUT_BAG="${REPLAY_GATE_OUT}/0705_shadow_${SHADOW_MODE}.bag"
test ! -e "${PAIR_OUTPUT_BAG}"
test ! -e "${PAIR_OUTPUT_BAG}.active"

rosbag record -O "${PAIR_OUTPUT_BAG}" \
  /clock \
  /odom \
  /imu/data \
  /spmpc/status \
  /spmpc/debug/effective_config \
  /spmpc/debug/slosh_observer_odom \
  /spmpc/debug/slosh_observer_imu \
  /spmpc/debug/raw_state \
  /spmpc/debug/predicted_state \
  /spmpc/debug/solver_input_state \
  /spmpc/debug/pre_solve_snapshot \
  /spmpc/debug/predicted_horizon \
  /spmpc/debug/slosh_governor \
  /spmpc/debug/delay_phase \
  /spmpc/debug/cmd_vel_output \
  /spmpc/replay/cmd_vel_unused
```

终端 D 对两轮使用完全相同的输入 topics：

```bash
rosbag play -q --clock --rate=1 --hz=1000 "${REPLAY_0705_BAG}" --topics \
  /odom /imu/data /tf /tf_static /map /scout/global_path_fixed \
  2>&1 | tee "${REPLAY_GATE_OUT}/0705_shadow_${SHADOW_MODE}_playback.log"
```

每轮正常关闭 recorder 后必须确认 output bag 非空、没有 `.bag.active`，保存 `rosbag info` 与 output SHA-256；两个 `*_input.env` 的 `input_sha256` 必须完全相同。成对 analyzer 必须按输入 stamp/solver step 对齐两轮，在预冻结数值容差内比较 odom-derived solver input、governor、delay predictor、pre-solve snapshot、完整 horizon、first action/status，并确认 `/spmpc/replay/cmd_vel_unused` 无消息。当前 `shadow=false` 不发布两路 observer debug，故该轮必须从 raw `/odom` 离线复算 odom observer，不能把缺 topic 当数据相等。允许变化的只有 IMU shadow 诊断、其 callback/CPU 开销及预声明的 timing 指标。

回放总门至少检查：

- IMU debug 能从 bias/filter transient 进入 `READY`，运动开场 bag 应稳定 fail closed；
- odom 与 IMU observer 的 stamp、`sample_dt_sec`、epoch、有效率和高度均可复算；
- `/spmpc/replay/cmd_vel_unused` 没有消息，真实 `/cmd_vel` 完全不被触碰；
- 成对报告证明 IMU traffic 在冻结容差内不改变 odom-derived `SolverInput.slosh`、governor、delay predictor、horizon 或 first action；
- frame mismatch、时间倒退、重复时间戳和 `>35 ms` gap 使用预期状态码，不能静默沿用旧加速度。

当前仓库尚无可签署的 paired analyzer/report，因此即使手工看起来一致，也只能记 `CANNOT_VERIFY / FORMAL NO-GO`。本门未来通过只表示 shadow 可进入下一步 development；不表示 IMU 比 odom 更接近 RGB。

---

## 3. 实物基础系统

现场优先使用统一基础栈：

```bash
set -euo pipefail
test -r /opt/ros/noetic/setup.bash
source /opt/ros/noetic/setup.bash
test -r "${SCOUT_WS}/devel/setup.bash"
source "${SCOUT_WS}/devel/setup.bash"
[[ "${ROS_VERSION:-}" == 1 && "${ROS_DISTRO:-}" == noetic ]]
cd "${SCOUT_WS}"

SCOUT_WS="${SCOUT_WS}" \
REALSENSE_COLOR_WIDTH=1920 \
REALSENSE_COLOR_HEIGHT=1080 \
REALSENSE_COLOR_FPS=30 \
REALSENSE_ENABLE_DEPTH=false \
REALSENSE_ENABLE_INFRA=false \
WAIT_FOR_ODOM=true \
WAIT_FOR_LOCALIZATION_MAP=true \
bash /home/geist/scout_ws/src/scout_apps/control/scout_local_planner/scripts/launch_real_sensors_stack.sh
```

另开终端检查：

```bash
set -euo pipefail
: "${SLOSH_BAG_ROOT:?导出 bag 根目录}"
: "${MIN_BAG_FREE_GIB:?导出本 batch 冻结的最小剩余 GiB（正整数）}"

check_hz() {
  local topic="$1" seconds="${2:-10}" output rc
  set +e
  output="$(timeout "${seconds}s" rostopic hz "${topic}" 2>&1)"
  rc=$?
  set -e
  printf '%s\n' "${output}"
  [[ ${rc} -eq 0 || ${rc} -eq 124 ]]
  grep -Fq 'average rate:' <<< "${output}"
}

check_hz /odom
check_hz /scan_front
check_hz /imu/data
check_hz /camera/color/image_raw

set +e
TF_OUTPUT="$(timeout 5s rosrun tf tf_echo map base_link 2>&1)"
TF_RC=$?
set -e
printf '%s\n' "${TF_OUTPUT}"
[[ ${TF_RC} -eq 0 || ${TF_RC} -eq 124 ]]
grep -Fq 'Translation:' <<< "${TF_OUTPUT}"
grep -Fq 'Rotation:' <<< "${TF_OUTPUT}"

timeout 10s rostopic echo -n 1 /camera/color/camera_info
timeout 10s rostopic echo -n 1 /map
IMU_FRAME_OUTPUT="$(timeout 10s rostopic echo -n 1 /imu/data/header/frame_id)"
grep -Fq 'imu_link' <<< "${IMU_FRAME_OUTPUT}"

if rostopic list | grep -Fxq /cmd_vel; then
  CMD_INFO="$(rostopic info /cmd_vel)"
  printf '%s\n' "${CMD_INFO}"
  CMD_PUBLISHERS="$(awk '
    /^Publishers:/ {inside=1; next}
    /^Subscribers:/ {inside=0}
    inside && /^[[:space:]]*\*/ {print}
  ' <<< "${CMD_INFO}")"
  [[ -z "${CMD_PUBLISHERS}" ]]
fi

test -d "${SLOSH_BAG_ROOT}"
test -w "${SLOSH_BAG_ROOT}"
BAG_WRITE_PROBE="$(mktemp "${SLOSH_BAG_ROOT}/.spmpc_write_probe.XXXXXX")"
printf 'spmpc preflight\n' > "${BAG_WRITE_PROBE}"
test -s "${BAG_WRITE_PROBE}"
rm -- "${BAG_WRITE_PROBE}"
case "${MIN_BAG_FREE_GIB}" in ''|*[!0-9]*) exit 2 ;; esac
(( MIN_BAG_FREE_GIB > 0 ))
AVAILABLE_BYTES="$(df --output=avail -B1 "${SLOSH_BAG_ROOT}" | awk 'NR==2 {print $1}')"
[[ "${AVAILABLE_BYTES}" =~ ^[0-9]+$ ]]
REQUIRED_BYTES=$(( MIN_BAG_FREE_GIB * 1024 * 1024 * 1024 ))
(( AVAILABLE_BYTES >= REQUIRED_BYTES ))
df -h "${SLOSH_BAG_ROOT}"
```

`rostopic hz` 正常采到数据后通常仍因 `timeout` 返回 124，因此 PASS 依据是“退出码为 0/124 且输出含 `average rate:`”，不能把 124 直接当失败或把空输出当通过。上述命令只证明有数据；各 topic 的最低频率仍必须在 manifest 中冻结并由 sidecar 检查。开始 trial 前 `/cmd_vel` 不得存在任何旧 planner/tracker publisher。基础栈退出、`/map` 或相机信息超时、IMU frame 非 `imu_link`、目录实际写入失败或空间低于冻结阈值时均不得继续。

RealSense 在同一 development/formal batch 中保持运行。曝光、增益、白平衡和安装姿态不得在 block 中途修改。

---

## 4. Formal 前 development 命令边界

G2S/G2C/G3 共用同一 attempt 合同。冻结顺序表定义的是 **planned row**，不为 retry 增加新行；每次实际启动使用唯一 `ATTEMPT_ID=RUN_LABEL` 和两位 `DEV_REPEAT`。首次固定为 `01`。`02+` 只有在前一 attempt 的 append-only `RETRY_FAILURE_EVIDENCE_MANIFEST` 已记录原始 artifact/启动日志、由冻结 development-gate verifier 实际打开并判为 `METHOD_INDEPENDENT_ACQUISITION`，再由该 verifier 原子签出带 hash 的 `RETRY_REASON_FILE` authorization，且仍在同一 `block_segment_id` 时才可执行；任意 64 位字符串不能代替 failure evidence。authorization/evidence 还必须含冻结 allowlist 中的 `failure_reason_code`、`condition_independent=true`、`condition_specific=false`、`motion_induced=false` 和独立证据 hash；condition-specific runner/profile/config、backend/topic/CPU 压力或运动诱发视频失效必须被 verifier 拒绝。solver/tracker/observer fallback、tracking、timeout、安全终止或其他方法失败不得授权 retry。所有 attempts 都进入 acquisition/readiness/postflight 可靠性账本，method success/failure 则固定以 planned rows 为分母；每个 planned row 最多贡献一个 eligible outcome。最大 `DEV_MAX_REPEAT` 在第一条相应 gate trial 前冻结，runner 不得自动 retry。当前三个 development wrappers 固定 `SPLIT_BLOCK=false`，因此不支持跨 segment retry；跨段数据只能另标 `RECOVERY_OBSERVATION` 进入 reliability ledger，不得进入 gate dataset 的 row/retry chain 或 gate PASS。

授权必须由 future `verify_spmpc_development_gate.py classify-and-authorize-retry` 子命令从 previous-attempt ledger、实际 failure-evidence manifest 与冻结 classifier rule **原子生成**；后面的 trial 模板只消费和复核该产物。当前仓库没有这个子命令，因此 `r02+` 与 G2S/G2C/G3 PASS 一样 fail closed，禁止手写 authorization 文件。failure-evidence manifest 即使记录 `raw_artifact_index_sha256=none`，也必须绑定并实际打开一个非空 startup/runner failure log；两类证据不能同时缺失。

### 4.1 G0/G1：claim/comparator 与 base rotation release

G0 先冻结 prescribed-path 主张边界、五条件身份、primary/secondary contrasts 和 fairness variables，并生成唯一 `G0_CLAIM` PASS report/hash。该报告至少以唯一 key 记录 `report_type/status/release_id/git_revision`、`claim_boundary_sha256`、`five_condition_registry_sha256`、`contrast_registry_sha256`、`contrast_hierarchy_sha256`、`fairness_variables_sha256`、协议草案 hash 与 reviewer/audit rule hash；formal 静态层的 `CONTRAST_REGISTRY_SHA256` 必须等于这里的外部锚定值。G1 report 必须反向绑定 `g0_claim_report_sha256`。当前没有 G0 自动生成命令；只能由冻结审计器生成，手写最小 PASS 文件无效。没有 G0，不得通过后验删改 comparator 来解释 G1/G2 结果。

current 与 rotation-consistent candidate 必须先通过冻结的 rotation-relevance replay，比较 modal propagation、first action 和 `t(σ)/v(σ)`。若差异超过预冻结可辨识门槛，必须先完成 rotation-only 小规模 RGB pilot，再选择唯一 base rotation release。它不是 G2S 后的 final source-specific method release；若 source 变化可能改变 rotation relevance，implementation bridge 必须证明可继承或重做 source-sensitive replay。当前没有可直接宣告通过的现场命令。

输出至少包括：

- candidate revision/config hash；
- high-curvature 与 curvature-reversal cases；
- relevance thresholds；
- 选择的唯一 release；
- rejected release 的归档记录。

未生成唯一 release 时，后续只能做 development。

### 4.2 G2A：既有 candidate screening 与离线审计

20260727 已完成 15 条 `B0/Bsmooth/W1/W2/W5` H0 no-RGB bag。该批数据证明第一空间模态的二阶 ODE 在 raw observer、0.22 s predictor 和 acados horizon 中持续传播，支持淘汰 W1，但不能冻结 W5：W5 的明显低值主要在 delay-predicted solver x0/2 s terminal，odom-driven raw P95 与 W2 未可靠拉开，且 W5 约慢 5.9%。因此**不得原样重跑这 15 条**。

先使用既有 bag/snapshot 完成：

1. `predicted(t)` 对 `raw(t+tau)` 的 future-alignment，并分离 0.15/0.22 s 线/角 delay；
2. online/raw/zero-modal 三分支 replay；
3. ERK `num_steps=1/2/4` 或 exact-discrete sensitivity；
4. running/terminal objective 与 generated acados 的一致诊断；
5. generated JSON/C/.so、生成命令和 SHA-256 归档。

若上述审计导致 predictor、积分器或 objective 改变，先建立新 development release。完成 G2A 后先进入 G2S 冻结输入源；此时不要提前补跑 W2/W5。输入源冻结后再执行 G2C，避免用 odom 选出的权重直接外推到相位/幅值不同的 IMU 输入。

### 4.3 G2S：odom/processed-IMU/在线 RGB 标量同 trial 输入源选择

> **2026-07-31 执行覆盖：** 本节后面的长命令模板仍保留旧 raw-RGB/development-registry 设计，只作历史合同参考，**当前不得直接执行**。G2S 的唯一现场入口已经改为下列 image-free wrapper；它自动启动冻结在线检测、强制 `publish_debug=false`，bag 只录 `/liquid/measurement`/质量/控制证据，并在闭包后验证所有 image message type 为 0：
>
> ```bash
> VALIDATE_ONLY=true G2S_ROW=01 \
> bash /home/geist/scout_ws/src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_g2s_h0s_source_selection_trial.sh
>
> ARM_MOTION=YES CONFIRM_RGB_GEOMETRY=YES G2S_ROW=01 \
> bash /home/geist/scout_ws/src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_g2s_h0s_source_selection_trial.sh
> ```
>
> `G2S_ROW=01..04` 每次只跑一条。旧模板中的 `PILOT_RECORD_RGB=true`、`/camera/color/image_raw` required topic 和 80 GiB raw-RGB 空间门均已被覆盖，不得复制到新命令。正式 development registry 以后要吸收新 wrapper 的 stamped-quality、相机手动参数和 algorithm/config hash，而不是重新启用视频录制。

#### 4.3.1 当前语义

G2S 使用 `Bsmooth` 产生与液体 observer 无关的同一段真实运动；同一 bag 同时记录 odom observer、processed-IMU observer 和冻结在线 RGB 标量。`Bsmooth` 不消费两路液体状态，因此这一步比较的是 observer 与同一物理参考的一致性，不是两套控制器。working tree 已有 selector/fallback implementation，但 nominal source 尚未由 4 条 G2S 决策冻结。

#### 4.3.2 旧 development-registry 合同草案（不可执行）

以下内容直到第 4.4 节仅保留尚未实现的完整 registry/retry/hash 合同，不能覆盖第 0 节的当前 wrapper，也不能从中复制 raw-RGB 或旧 runner 命令。

G2S 是 G2C/G3 前的强制子门，不是论文第六条件。当前代码始终由 odom 向 solver 提供液体状态；`IMU_SHADOW_ENABLE=true` 只启动并行诊断。

执行顺序：

```text
ROS1 build/test
  -> G0 claim/comparator PASS
  -> G1 唯一 base rotation release PASS
  -> G2A existing-bag/internal audit PASS
  -> r02 tuning 回放
  -> remote_r03 + planar_r03 calibration-validation 回放（不得再调参数；只签软件链）
  -> 冻结 source-selection preregistration
  -> 新 H0s + Bsmooth + RGB 同-trial配对
  -> 冻结 odom，或触发 IMU 新 method release
```

第 2.3.1 节只给出了 `planar_r03` 的详细 replay 模板；当前尚缺覆盖 `r02` tuning 与 `remote_r03` calibration-validation 的统一冻结 wrapper/analyzer，仓库中也尚无模板要求的 `verify_spmpc_development_gate.py`。因此在两者能够产出并验证一个同时绑定三条 input-bag SHA、processed-IMU config、analyzer、G0/G1/G2A reports、release/revision 且 `status=PASS` 的 `IMU_CAL_VALIDATION_REPORT` 前，下面 H0s 命令会在 `test -x` 处 fail closed；不得手写一个最小 PASS 文件代替缺失回放。

第一条 H0s 前必须生成并 hash：

```text
development/G2S/
├── source_selection_prereg.yaml
├── source_selection_order.csv        # block,order_position,condition,block_segment_id
├── processed_imu_config.yaml
├── rgb_sync_and_metric.yaml          # 内含 RGB calibration 路径/hash 与适用性检查
└── sha256.txt
```

其中预注册 `n_src`、冻结 lag/filter/外参版本、RGB 同步、motion window、非负 envelope 定义、trial-level 主误差、`H_vis,p95` peak bias、最小实质改善 `delta_src`、有效率和 leave-one-trial-out 规则。`DEV_RELEASE_ID` 是贯穿 G2S→G2C→G3 的同一 development release-lineage ID，不是每个 gate 临时换一个名字；任一 source/code/config/candidate 变更按矩阵规则建立新 ID 并重做受影响门禁。`processed_imu_config.yaml` 必须是该 Git release 中 launch 实际加载的 `config/planner/common.yaml` 的逐字节冻结副本，不能另写一份“说明配置”只用于贴 hash；READY 后还必须用参数快照验证 live namespace。`sha256.txt` 必须至少逐一且唯一覆盖上述四个预注册 artifact，并由外部冻结的 `G2S_PREREG_INDEX_SHA256` 和每条 row 反向绑定，不能靠同时重写 artifact 与自校验清单通过。不得在 H0s RGB 上逐 trial 重调 lag、scale、滤波器或 RGB 参数。建议资源起点是 `n_src=4` 条完整配对 trial，但它只是 development 提案，必须在首条 trial 前冻结。

使用 `Bsmooth` 是为了让本次运动生成不依赖 odom/IMU 液体状态。每条 trial 同时产生两套 observer，高度比较仍只有一个物理统计样本：

```bash
set -euo pipefail
export SCOUT_WS="${SCOUT_WS:-/home/zrj/scout_ws}"
export SLOSH_BAG_ROOT="${SLOSH_BAG_ROOT:-/home/zrj/slosh_bags}"
test -r /opt/ros/noetic/setup.bash
source /opt/ros/noetic/setup.bash
test -r "${SCOUT_WS}/devel/setup.bash"
source "${SCOUT_WS}/devel/setup.bash"
[[ "${ROS_VERSION:-}" == 1 && "${ROS_DISTRO:-}" == noetic ]]
: "${DEV_DATE:?导出 development 日期}"
: "${DEV_RELEASE_ID:?导出 G2S development release ID}"
: "${DEV_RELEASE_GIT_REVISION:?导出 G2S release Git revision}"
: "${G0_CLAIM_REPORT:?导出 G0 claim/comparator PASS report}"
: "${G0_CLAIM_REPORT_SHA256:?导出 G0 report SHA-256}"
: "${G1_ROTATION_RELEASE_REPORT:?导出 G1 唯一 base rotation release PASS report}"
: "${G1_ROTATION_RELEASE_REPORT_SHA256:?导出 G1 report SHA-256}"
: "${G2A_AUDIT_REPORT:?导出 G2A audit PASS report}"
: "${G2A_AUDIT_REPORT_SHA256:?导出 G2A report SHA-256}"
: "${IMU_CAL_VALIDATION_REPORT:?导出 r02/remote_r03/planar_r03 calibration-validation PASS report}"
: "${IMU_CAL_VALIDATION_REPORT_SHA256:?导出 calibration-validation report SHA-256}"
: "${DEVELOPMENT_GATE_VERIFIER_SHA256:?导出冻结 development-gate verifier SHA-256}"
: "${G2S_PREREG_ROOT:?导出 development/G2S 预注册目录}"
: "${G2S_PREREG_INDEX_SHA256:?导出预注册 sha256.txt 自身的外部冻结 SHA-256}"
: "${G2S_N_SRC:?导出预冻结 paired-trial 数}"
: "${G2S_ROW_REPORT:?导出本 trial 的已验证顺序行报告}"
: "${G2S_ROW_REPORT_SHA256:?导出顺序行报告 SHA-256}"
: "${PROCESSED_IMU_CONFIG_SHA256:?导出 processed-IMU config SHA-256}"
: "${G2S_PATH_JSON:?导出冻结 H0s 路径绝对路径}"
: "${G2S_PATH_SHA256:?导出 H0s SHA-256}"
: "${G2S_BLOCK:?从 G2S 顺序表读取 block ID}"
: "${G2S_ORDER_POSITION:?从 G2S 顺序表读取 position}"
: "${G2S_BLOCK_SEGMENT_ID:?导出 block segment ID}"
: "${DEV_REPEAT:?导出两位 attempt 序号；首次固定 01}"
: "${DEV_MAX_REPEAT:?导出该 gate 预冻结的最大 repeat}"
: "${RGB_CALIB_FILE:?导出冻结 RGB 标定文件；0705 retrospective 使用 20260629_calib/red_3ruler.yaml}"
: "${RGB_CALIB_SHA256:?导出 RGB 标定文件 SHA-256}"
export RETRY_OF_ATTEMPT_ID="${RETRY_OF_ATTEMPT_ID:-}"
export RETRY_REASON_FILE="${RETRY_REASON_FILE:-}"
export RETRY_REASON_FILE_SHA256="${RETRY_REASON_FILE_SHA256:-}"
export RETRY_FAILURE_EVIDENCE_MANIFEST="${RETRY_FAILURE_EVIDENCE_MANIFEST:-}"
export RETRY_FAILURE_EVIDENCE_MANIFEST_SHA256="${RETRY_FAILURE_EVIDENCE_MANIFEST_SHA256:-}"
export EXPECTED_SPMPC_PKG="${SCOUT_WS}/src/scout_apps/control/spmpc_local_planner"
export RESOLVED_SPMPC_PKG="$(readlink -f "$(rospack find spmpc_local_planner)")"
[[ "${RESOLVED_SPMPC_PKG}" == "$(readlink -f "${EXPECTED_SPMPC_PKG}")" ]]
export LIVE_COMMON_YAML="${RESOLVED_SPMPC_PKG}/config/planner/common.yaml"
export PROCESSED_IMU_CONFIG_FILE="${LIVE_COMMON_YAML}"
export PREREG_PROCESSED_IMU_CONFIG="${G2S_PREREG_ROOT}/processed_imu_config.yaml"
export SPMPC_EXPERIMENT_LAUNCH="${RESOLVED_SPMPC_PKG}/launch/spmpc_experiment.launch"
export DEVELOPMENT_GATE_VERIFIER="${RESOLVED_SPMPC_PKG}/scripts/verify_spmpc_development_gate.py"

assert_unique_kv_keys() {
  local file_path="$1" duplicate_keys
  duplicate_keys="$(
    sed -n 's/^\([[:alnum:]_][[:alnum:]_]*\)=.*/\1/p' "${file_path}" \
      | sort | uniq -d
  )"
  [[ -z "${duplicate_keys}" ]]
}

verify_release_report() {
  local report_path="$1" expected_sha256="$2" report_type="$3" actual_sha256
  test -s "${report_path}"
  [[ "${expected_sha256}" =~ ^[0-9a-f]{64}$ ]]
  assert_unique_kv_keys "${report_path}"
  actual_sha256="$(sha256sum "${report_path}" | awk '{print $1}')"
  [[ "${actual_sha256}" == "${expected_sha256}" ]]
  grep -Fxq "report_type=${report_type}" "${report_path}"
  grep -Fxq "release_id=${DEV_RELEASE_ID}" "${report_path}"
  grep -Fxq "git_revision=${DEV_RELEASE_GIT_REVISION}" "${report_path}"
  grep -Fxq 'status=PASS' "${report_path}"
}

prereg_artifacts=(
  source_selection_prereg.yaml \
  source_selection_order.csv \
  processed_imu_config.yaml \
  rgb_sync_and_metric.yaml
)
for prereg_file in "${prereg_artifacts[@]}"; do
  test -s "${G2S_PREREG_ROOT}/${prereg_file}"
done
test -s "${G2S_PREREG_ROOT}/sha256.txt"
(cd "${G2S_PREREG_ROOT}" && sha256sum --check sha256.txt)
G2S_PREREG_INDEX_ACTUAL_SHA256="$(sha256sum "${G2S_PREREG_ROOT}/sha256.txt" | awk '{print $1}')"
[[ "${G2S_PREREG_INDEX_ACTUAL_SHA256}" == "${G2S_PREREG_INDEX_SHA256}" ]]
for prereg_file in "${prereg_artifacts[@]}"; do
  entry_count="$(
    awk -v target="${prereg_file}" '
      { name=$2; sub(/^\*/, "", name); if (name == target) count += 1 }
      END { print count + 0 }
    ' "${G2S_PREREG_ROOT}/sha256.txt"
  )"
  [[ "${entry_count}" == 1 ]]
done
G2S_ORDER_CSV="${G2S_PREREG_ROOT}/source_selection_order.csv"
G2S_ORDER_SHA256="$(sha256sum "${G2S_ORDER_CSV}" | awk '{print $1}')"
[[ "${G2S_N_SRC}" =~ ^[1-9][0-9]*$ ]]
python3 - "${G2S_ORDER_CSV}" "${G2S_N_SRC}" \
  "${G2S_BLOCK}" "${G2S_ORDER_POSITION}" "${G2S_BLOCK_SEGMENT_ID}" <<'PY'
import csv
import sys

path, n_expected, block, position, segment = sys.argv[1:]
with open(path, newline="", encoding="utf-8") as stream:
    reader = csv.DictReader(stream)
    required = {"block", "order_position", "condition", "block_segment_id"}
    if not reader.fieldnames or not required.issubset(reader.fieldnames):
        raise SystemExit("invalid G2S order-table header")
    rows = list(reader)
if len(rows) != int(n_expected):
    raise SystemExit("G2S row count does not match G2S_N_SRC")
if len({row["block"] for row in rows}) != len(rows):
    raise SystemExit("G2S block IDs are not unique")
if any(row["condition"] != "Bsmooth" for row in rows):
    raise SystemExit("G2S order table contains a condition other than Bsmooth")
matches = [
    row for row in rows
    if row["block"] == block
    and row["order_position"] == position
    and row["condition"] == "Bsmooth"
    and row["block_segment_id"] == segment
]
if len(matches) != 1:
    raise SystemExit("G2S row report is not a unique member of the frozen order table")
PY
test -s "${G2S_ROW_REPORT}"
test -s "${PROCESSED_IMU_CONFIG_FILE}"
test -s "${SPMPC_EXPERIMENT_LAUNCH}"
test -s "${G2S_PATH_JSON}"
test -s "${RGB_CALIB_FILE}"
grep -Fq '<rosparam file="$(find spmpc_local_planner)/config/planner/common.yaml" command="load"/>' \
  "${SPMPC_EXPERIMENT_LAUNCH}"
cmp -s "${PREREG_PROCESSED_IMU_CONFIG}" "${PROCESSED_IMU_CONFIG_FILE}"
assert_unique_kv_keys "${G2S_ROW_REPORT}"
[[ "$(git -C "${SCOUT_WS}" rev-parse HEAD)" == "${DEV_RELEASE_GIT_REVISION}" ]]
[[ -z "$(git -C "${SCOUT_WS}" status --porcelain)" ]]
verify_release_report "${G0_CLAIM_REPORT}" "${G0_CLAIM_REPORT_SHA256}" G0_CLAIM
verify_release_report "${G1_ROTATION_RELEASE_REPORT}" "${G1_ROTATION_RELEASE_REPORT_SHA256}" G1_ROTATION_RELEASE
grep -Fxq "g0_claim_report_sha256=${G0_CLAIM_REPORT_SHA256}" "${G1_ROTATION_RELEASE_REPORT}"
verify_release_report "${G2A_AUDIT_REPORT}" "${G2A_AUDIT_REPORT_SHA256}" G2A_AUDIT
grep -Fxq "g1_rotation_release_report_sha256=${G1_ROTATION_RELEASE_REPORT_SHA256}" "${G2A_AUDIT_REPORT}"
verify_release_report \
  "${IMU_CAL_VALIDATION_REPORT}" "${IMU_CAL_VALIDATION_REPORT_SHA256}" \
  G2S_CALIBRATION_VALIDATION
grep -Fxq "g2a_audit_report_sha256=${G2A_AUDIT_REPORT_SHA256}" "${IMU_CAL_VALIDATION_REPORT}"
grep -Fxq "processed_imu_config_sha256=${PROCESSED_IMU_CONFIG_SHA256}" "${IMU_CAL_VALIDATION_REPORT}"
for hash_field in \
  r02_bag_sha256 remote_r03_bag_sha256 planar_r03_bag_sha256 analyzer_sha256; do
  grep -Eq "^${hash_field}=[0-9a-f]{64}$" "${IMU_CAL_VALIDATION_REPORT}"
done
G2S_ROW_ACTUAL_SHA256="$(sha256sum "${G2S_ROW_REPORT}" | awk '{print $1}')"
[[ "${G2S_ROW_ACTUAL_SHA256}" == "${G2S_ROW_REPORT_SHA256}" ]]
PROCESSED_IMU_CONFIG_ACTUAL_SHA256="$(sha256sum "${PROCESSED_IMU_CONFIG_FILE}" | awk '{print $1}')"
[[ "${PROCESSED_IMU_CONFIG_ACTUAL_SHA256}" == "${PROCESSED_IMU_CONFIG_SHA256}" ]]
RGB_CALIB_ACTUAL_SHA256="$(sha256sum "${RGB_CALIB_FILE}" | awk '{print $1}')"
[[ "${RGB_CALIB_ACTUAL_SHA256}" == "${RGB_CALIB_SHA256}" ]]
for expected_line in \
  "gate=G2S" \
  "release_id=${DEV_RELEASE_ID}" \
  "git_revision=${DEV_RELEASE_GIT_REVISION}" \
  "solver_source=odom" \
  "condition=Bsmooth" \
  "pilot_method=Bsmooth" \
  "g2a_audit_report_sha256=${G2A_AUDIT_REPORT_SHA256}" \
  "imu_cal_validation_report_sha256=${IMU_CAL_VALIDATION_REPORT_SHA256}" \
  "development_gate_verifier_sha256=${DEVELOPMENT_GATE_VERIFIER_SHA256}" \
  "g2s_prereg_index_sha256=${G2S_PREREG_INDEX_SHA256}" \
  "order_table_sha256=${G2S_ORDER_SHA256}" \
  "n_src=${G2S_N_SRC}" \
  "block=${G2S_BLOCK}" \
  "order_position=${G2S_ORDER_POSITION}" \
  "block_segment_id=${G2S_BLOCK_SEGMENT_ID}" \
  "path_sha256=${G2S_PATH_SHA256}" \
  "processed_imu_config_sha256=${PROCESSED_IMU_CONFIG_SHA256}" \
  "rgb_calib_sha256=${RGB_CALIB_SHA256}"; do
  grep -Fxq "${expected_line}" "${G2S_ROW_REPORT}"
done

[[ "${DEV_REPEAT}" =~ ^(0[1-9]|[1-9][0-9])$ ]]
[[ "${DEV_MAX_REPEAT}" =~ ^(0[1-9]|[1-9][0-9])$ ]]
(( 10#${DEV_REPEAT} <= 10#${DEV_MAX_REPEAT} ))
attempt_prefix="DEV_G2S_H0s_C1_Bsmooth_b${G2S_BLOCK}"
export RUN_LABEL="${attempt_prefix}_r${DEV_REPEAT}"
export ATTEMPT_ID="${RUN_LABEL}"
if [[ "${DEV_REPEAT}" == 01 ]]; then
  export ACQUISITION_RETRY=false
  [[ -z "${RETRY_OF_ATTEMPT_ID}" ]]
  [[ -z "${RETRY_REASON_FILE}" ]]
  [[ -z "${RETRY_REASON_FILE_SHA256}" ]]
  [[ -z "${RETRY_FAILURE_EVIDENCE_MANIFEST}" ]]
  [[ -z "${RETRY_FAILURE_EVIDENCE_MANIFEST_SHA256}" ]]
else
  export ACQUISITION_RETRY=true
  [[ -n "${RETRY_OF_ATTEMPT_ID}" ]]
  [[ "${RETRY_OF_ATTEMPT_ID%_r*}" == "${attempt_prefix}" ]]
  previous_repeat="${RETRY_OF_ATTEMPT_ID##*_r}"
  [[ "${previous_repeat}" =~ ^(0[1-9]|[1-9][0-9])$ ]]
  (( 10#${DEV_REPEAT} == 10#${previous_repeat} + 1 ))
  test -s "${RETRY_REASON_FILE}"
  [[ "${RETRY_REASON_FILE_SHA256}" =~ ^[0-9a-f]{64}$ ]]
  [[ "$(sha256sum "${RETRY_REASON_FILE}" | awk '{print $1}')" == "${RETRY_REASON_FILE_SHA256}" ]]
  test -s "${RETRY_FAILURE_EVIDENCE_MANIFEST}"
  [[ "${RETRY_FAILURE_EVIDENCE_MANIFEST_SHA256}" =~ ^[0-9a-f]{64}$ ]]
  [[ "$(sha256sum "${RETRY_FAILURE_EVIDENCE_MANIFEST}" | awk '{print $1}')" == "${RETRY_FAILURE_EVIDENCE_MANIFEST_SHA256}" ]]
  assert_unique_kv_keys "${RETRY_REASON_FILE}"
  assert_unique_kv_keys "${RETRY_FAILURE_EVIDENCE_MANIFEST}"
  for expected_line in \
    "report_type=DEVELOPMENT_RETRY_AUTHORIZATION" \
    "status=PASS" \
    "gate=G2S" \
    "release_id=${DEV_RELEASE_ID}" \
    "git_revision=${DEV_RELEASE_GIT_REVISION}" \
    "planned_row_report_sha256=${G2S_ROW_REPORT_SHA256}" \
    "failed_attempt_id=${RETRY_OF_ATTEMPT_ID}" \
    "authorized_attempt_id=${ATTEMPT_ID}" \
    "block_segment_id=${G2S_BLOCK_SEGMENT_ID}" \
    "failure_class=METHOD_INDEPENDENT_ACQUISITION" \
    "method_failure=false" \
    "retry_authorized=true" \
    "failure_evidence_manifest_path=${RETRY_FAILURE_EVIDENCE_MANIFEST}" \
    "failure_evidence_manifest_sha256=${RETRY_FAILURE_EVIDENCE_MANIFEST_SHA256}" \
    "development_gate_verifier_sha256=${DEVELOPMENT_GATE_VERIFIER_SHA256}"; do
    grep -Fxq "${expected_line}" "${RETRY_REASON_FILE}"
  done
  for expected_line in \
    "report_type=DEVELOPMENT_FAILED_ATTEMPT_EVIDENCE" \
    "status=CLASSIFIED" \
    "gate=G2S" \
    "release_id=${DEV_RELEASE_ID}" \
    "git_revision=${DEV_RELEASE_GIT_REVISION}" \
    "planned_row_report_sha256=${G2S_ROW_REPORT_SHA256}" \
    "attempt_id=${RETRY_OF_ATTEMPT_ID}" \
    "block_segment_id=${G2S_BLOCK_SEGMENT_ID}" \
    "failure_class=METHOD_INDEPENDENT_ACQUISITION" \
    "method_failure=false" \
    "eligible_outcome=false"; do
    grep -Fxq "${expected_line}" "${RETRY_FAILURE_EVIDENCE_MANIFEST}"
  done
  grep -Eq '^raw_artifact_index_sha256=([0-9a-f]{64}|none)$' "${RETRY_FAILURE_EVIDENCE_MANIFEST}"
  startup_log_path="$(sed -n 's/^startup_log_path=//p' "${RETRY_FAILURE_EVIDENCE_MANIFEST}")"
  test -s "${startup_log_path}"
  startup_log_sha256="$(sha256sum "${startup_log_path}" | awk '{print $1}')"
  grep -Fxq "startup_log_sha256=${startup_log_sha256}" "${RETRY_FAILURE_EVIDENCE_MANIFEST}"
fi

test -x "${DEVELOPMENT_GATE_VERIFIER}"
DEVELOPMENT_GATE_VERIFIER_ACTUAL_SHA256="$(sha256sum "${DEVELOPMENT_GATE_VERIFIER}" | awk '{print $1}')"
[[ "${DEVELOPMENT_GATE_VERIFIER_ACTUAL_SHA256}" == "${DEVELOPMENT_GATE_VERIFIER_SHA256}" ]]
export G2S_PREREQ_VERIFICATION_REPORT="${G2S_ROW_REPORT}.${ATTEMPT_ID}.prereq_verification.txt"
test ! -e "${G2S_PREREQ_VERIFICATION_REPORT}"
"${DEVELOPMENT_GATE_VERIFIER}" verify-prerequisites \
  --gate G2S \
  --release-id "${DEV_RELEASE_ID}" \
  --git-revision "${DEV_RELEASE_GIT_REVISION}" \
  --g0-report "${G0_CLAIM_REPORT}" --g0-sha256 "${G0_CLAIM_REPORT_SHA256}" \
  --g1-report "${G1_ROTATION_RELEASE_REPORT}" --g1-sha256 "${G1_ROTATION_RELEASE_REPORT_SHA256}" \
  --g2a-report "${G2A_AUDIT_REPORT}" --g2a-sha256 "${G2A_AUDIT_REPORT_SHA256}" \
  --cal-validation-report "${IMU_CAL_VALIDATION_REPORT}" \
  --cal-validation-sha256 "${IMU_CAL_VALIDATION_REPORT_SHA256}" \
  --prereg-root "${G2S_PREREG_ROOT}" \
  --prereg-index-sha256 "${G2S_PREREG_INDEX_SHA256}" \
  --row-report "${G2S_ROW_REPORT}" --row-sha256 "${G2S_ROW_REPORT_SHA256}" \
  --attempt-id "${ATTEMPT_ID}" --dev-repeat "${DEV_REPEAT}" \
  --max-repeat "${DEV_MAX_REPEAT}" --acquisition-retry "${ACQUISITION_RETRY}" \
  --retry-of-attempt-id "${RETRY_OF_ATTEMPT_ID:-none}" \
  --retry-reason-file "${RETRY_REASON_FILE:-none}" \
  --retry-reason-file-sha256 "${RETRY_REASON_FILE_SHA256:-none}" \
  --failure-evidence-manifest "${RETRY_FAILURE_EVIDENCE_MANIFEST:-none}" \
  --failure-evidence-manifest-sha256 "${RETRY_FAILURE_EVIDENCE_MANIFEST_SHA256:-none}" \
  --live-common-yaml "${PROCESSED_IMU_CONFIG_FILE}" \
  --processed-config-sha256 "${PROCESSED_IMU_CONFIG_SHA256}" \
  --rgb-calibration "${RGB_CALIB_FILE}" --rgb-calibration-sha256 "${RGB_CALIB_SHA256}" \
  --output "${G2S_PREREQ_VERIFICATION_REPORT}"
grep -Fxq 'status=PASS' "${G2S_PREREQ_VERIFICATION_REPORT}"
G2S_PREREQ_VERIFICATION_SHA256="$(sha256sum "${G2S_PREREQ_VERIFICATION_REPORT}" | awk '{print $1}')"

export RUN_OUT_DIR="${SLOSH_BAG_ROOT}/real/${DEV_DATE}_spmpc_development/G2S/Bsmooth"
export CURRENT_OBSERVER_SOURCE=odom
[[ "${CURRENT_OBSERVER_SOURCE}" == odom ]]

unset MATRIX_PRESET RECORDER_SCRIPT VARIANT ALG W_SLOSH

PILOT_MODE=true \
PILOT_METHOD=Bsmooth \
PILOT_CONDITION=G2S_SourceSelection \
SOLVER_BACKEND=continuous_mpcc_acados \
REF_TOPIC=/scout/global_path_fixed \
CMD_TOPIC=/cmd_vel \
COSTMAP_TOPIC=/map \
REFERENCE_TARGET_FRAME=map \
BASE_FRAME=base_link \
PATH_SOURCE_MODE=replay \
PATH_FILE="${G2S_PATH_JSON}" \
PATH_EXPECTED_SHA256="${G2S_PATH_SHA256}" \
REQUIRE_PATH_HASH=true \
BLOCK_SEGMENT_ID="${G2S_BLOCK_SEGMENT_ID}" \
ORDER_POSITION="${G2S_ORDER_POSITION}" \
SPLIT_BLOCK=false \
ACQUISITION_RETRY="${ACQUISITION_RETRY}" \
RETRY_REASON_FILE="${RETRY_REASON_FILE}" \
RUN_LABEL="${RUN_LABEL}" \
NAME="${RUN_LABEL}" \
RUN_OUT_DIR="${RUN_OUT_DIR}" \
DATE="${DEV_DATE}" \
PILOT_RECORD_RGB=true \
RECORD_TOPIC_INFO=true \
RECORD_STANDALONE_SLOSH=false \
RECORD_SCAN=true \
RECORD_MOCAP=false \
RECORD_ROSOUT=true \
RECORD_ALL_EXISTING_TOPICS=false \
LIQUID_EXPORT_AFTER_RECORD=false \
ROSBAG_BUFFER_SIZE_MB=4096 \
RECORD_SEC=90 \
MAX_RECORD_SEC=90 \
START_POS_TOL=0.08 \
START_YAW_TOL=0.15 \
START_HOLD_SEC=0.5 \
START_GATE_TIMEOUT_SEC=15 \
PATH_PUBLISH_RATE=2.0 \
PATH_GENERATOR_STARTUP_SEC=2 \
RECORDER_STARTUP_SEC=8 \
PLANNER_STARTUP_SEC=2 \
SEND_ZERO_ON_EXIT=true \
V_REF=0.20 \
SLOSH_HEIGHT_MAX=-1.0 \
ALPHA_MAX=1.2 \
SHARED_LINEAR_ACCEL_LIMIT_ENABLE=true \
SHARED_LINEAR_ACCEL_MAX=0.6 \
SHARED_ANGULAR_LIMIT_ENABLE=true \
SHARED_ANGULAR_RATE_MAX=1.2 \
SHARED_ANGULAR_ACCEL_MAX=1.2 \
DELAY_PHASE_MODE=off \
DELAY_PHASE_LINEAR_DELAY_SEC=-1.0 \
DELAY_PHASE_ANGULAR_DELAY_SEC=-1.0 \
IMU_SHADOW_ENABLE=true \
IMU_TOPIC=/imu/data \
IMU_SHADOW_READY_TOPIC=/spmpc/debug/slosh_observer_imu \
IMU_SHADOW_READY_TIMEOUT_SEC=20 \
RECORDER_ACTIVE_TIMEOUT_SEC=15 \
OPERATOR_NOTE="development G2S source-selection release=${DEV_RELEASE_ID}; revision=${DEV_RELEASE_GIT_REVISION}; attempt_id=${ATTEMPT_ID}; repeat=${DEV_REPEAT}; retry_of=${RETRY_OF_ATTEMPT_ID:-none}; retry_reason_sha256=${RETRY_REASON_FILE_SHA256:-none}; failure_evidence_sha256=${RETRY_FAILURE_EVIDENCE_MANIFEST_SHA256:-none}; solver_source=odom; imu=shadow-only; g2a_sha256=${G2A_AUDIT_REPORT_SHA256}; cal_validation_sha256=${IMU_CAL_VALIDATION_REPORT_SHA256}; prereg_index_sha256=${G2S_PREREG_INDEX_SHA256}; row_sha256=${G2S_ROW_REPORT_SHA256}; prereq_verification_sha256=${G2S_PREREQ_VERIFICATION_SHA256}; imu_config_sha256=${PROCESSED_IMU_CONFIG_SHA256}; rgb_calib_sha256=${RGB_CALIB_SHA256}" \
bash src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_real_fixed_path_trial.sh
```

运行命令前车辆必须已经位于 H0s 起点门内，且 `/imu/data.header.frame_id=imu_link`。该命令在 replay 模式下的真实安全顺序是：检查 reference/cmd/IMU-debug 残留 publisher → recorder `.bag.active` → planner 在无 reference 时零速等待 → IMU `READY + valid + bias_ready + filter_ready` → 启动 replay path/start gate；replay 不发布 goal，goal idle 检查只适用于 generate 模式。车辆必须从 planner 启动前一直静止到 `READY`；脚本超时或 recorder/planner 提前退出会自动停止，不得手工绕过。预先启动 planner、path 或其他 shadow 多终端流程会触发 idle gate，不能与此 one-click 命令混用。

`RECORD_SEC=90` 从 recorder 启动开始计时，包含 recorder/planner startup、约 10.2 s 的 bias/filter warmup 和 start gate，不是 90 s 纯运动窗口。验收时必须确认它仍覆盖完整有效运动和冻结的运动后 tail；否则首条 trial 前统一增大 `RECORD_SEC=MAX_RECORD_SEC`，不能事后只给失败条件延长。当前 one-click launch 未暴露 `platform_config/container_config`，实际采用 `scout_mini/tube_default` 默认值；只有 preregistration 明确将其定义为本次 C1 时才可用于 G2S，不能据此签署 C2。

每条 bag 至少验收：

```text
/odom
/imu/data
/spmpc/debug/slosh_observer_odom
/spmpc/debug/slosh_observer_imu
/camera/color/image_raw
/camera/color/camera_info
/tf
/tf_static
```

当前 recorder 的 topic-info 与 rosparam dump 发生在 planner 启动前，因此其中缺少 planner/debug topic 或最终 `/spmpc_local_planner` 参数是预期现象，但不能当作最终配置证据。G2S 接受前还需由 wrapper 增加 planner `READY` 后参数快照，或由独立只读 watcher 留下等价 sidecar；topic 存在性以闭包后的 `bag_info` 和实际消息计数为准，而不是启动前 topic-info。

G2S runner 退出 0 后仍必须通过第 4.5.1 节的独立 bag closure/integrity/topic-count gate；失败 attempt 不进入 paired continuous-accuracy 的 `continuous_eligible_count`，但仍进入完整 dataset index。attempt-level acquisition/readiness/postflight 只进入 `N_attempt` 可靠性字段，source-ready 与 method success/failure 则进入固定 `N_plan=n_src` 字段，不能混成一个 failure denominator，也不能从 source reliability 证据中删除。

选择规则：只有 processed-IMU 在冻结指标上超过 `delta_src`、方向不由单条 trial 独占，且 READY/coverage/静止残差不劣化时，才触发新的 IMU method release。相当、冲突或不确定时保留 odom。全部计划行结束后先生成 G2S dataset index，逐行列出 planned row、每个 `ATTEMPT_ID`/retry chain、prerequisite-verification、postflight、bag、失败/排除及其 hash；缺计划行、attempt 未分类或提前停止不得生成 PASS。冻结的 source-selection report 至少以唯一 key 记录 `report_type/status/release_id/git_revision`、G2A/calibration-validation/prereg/verifier hashes、`selected_source`、处理配置/RGB/指标/完整 G2S dataset-index hash、decision rule，以及 `n_src/planned_row_coverage_count/minimum_continuous_eligible_count/continuous_eligible_count/attempt_count/postflight_complete_attempt_count/acquisition_failure_attempt_count/readiness_failure_attempt_count/method_success_planned_row_count/method_failure_planned_row_count/unresolved_acquisition_planned_row_count`。attempt 分类由 dataset-index verifier 逐 attempt 重算，planned-row method partition 也必须从 index 独立重算且总和等于 `n_src`；不得用 `attempts-eligible` 反推。若触发 IMU，当前代码仍不能直接 formal 使用：必须实现 source selector，并生成绑定本 G2S report、新 method release/revision、ROS1 build/replay、effective-source 与 fallback/solver invariant 的 `IMU_IMPLEMENTATION_VALIDATION` report/hash，再重做 G2C 的最小 W2/W5 与完整 G3，并更新论文/manifest。禁止把旧 odom trial 与新 IMU trial 合并。

### 4.4 G2C：选定输入源后的 W2/W5 最小确认

G2C 只比较 G2A 后存活的 W2/W5；不恢复 W1，也不重复五条件 × 3。确认数量、顺序、release 和 acceptance rule 必须先写入 development registry。下面模板**只适用于 G2S 保留 odom 的 release**。若 G2S 触发 IMU，禁止使用或仅改 metadata：必须先完成 source selector/fallback、新 ROS1 门禁和 effective-source 检查，再另写 IMU-release 模板。

第一条 G2C 前冻结并对外登记 index hash：

```text
development/G2C/
├── candidate_confirmation_prereg.yaml
├── candidate_confirmation_order.csv   # block,order_position,pilot_method,block_segment_id
├── candidate_confirmation_analysis.yaml
└── sha256.txt
```

完整顺序表必须使每个 block 恰含 W2/W5 各一次；逐条 row report 必须绑定整个 prereg index 与 order-table hash，不能按当天想跑的条件临时生成。

```bash
set -euo pipefail
export SCOUT_WS="${SCOUT_WS:-/home/zrj/scout_ws}"
export SLOSH_BAG_ROOT="${SLOSH_BAG_ROOT:-/home/zrj/slosh_bags}"
test -r /opt/ros/noetic/setup.bash
source /opt/ros/noetic/setup.bash
test -r "${SCOUT_WS}/devel/setup.bash"
source "${SCOUT_WS}/devel/setup.bash"
[[ "${ROS_VERSION:-}" == 1 && "${ROS_DISTRO:-}" == noetic ]]
: "${DEV_DATE:?导出 development 日期}"
: "${DEV_RELEASE_ID:?导出与 G2S 决策报告一致的 release-lineage ID}"
: "${DEV_RELEASE_GIT_REVISION:?导出 G2C release Git revision}"
: "${SOURCE_SELECTION_REPORT:?导出冻结 G2S 决策报告}"
: "${SOURCE_SELECTION_REPORT_SHA256:?导出 G2S 决策报告 SHA-256}"
: "${G2S_DATASET_INDEX:?导出含全部 G2S row/prereq/postflight/bag hash 的数据集 index}"
: "${G2S_DATASET_INDEX_SHA256:?导出 G2S dataset index SHA-256}"
: "${G2S_N_SRC:?导出 G2S 预冻结 paired-trial 数}"
: "${G2S_PREREG_INDEX_SHA256:?导出 G2S prereg index SHA-256}"
: "${G2A_AUDIT_REPORT_SHA256:?导出 G2A audit report SHA-256}"
: "${IMU_CAL_VALIDATION_REPORT_SHA256:?导出 IMU calibration-validation report SHA-256}"
: "${PROCESSED_IMU_CONFIG_SHA256:?导出冻结 processed-IMU config SHA-256}"
: "${RGB_CALIB_SHA256:?导出冻结 RGB calibration SHA-256}"
: "${G2C_PREREG_ROOT:?导出 development/G2C 预注册目录}"
: "${G2C_PREREG_INDEX_SHA256:?导出 G2C sha256.txt 的外部冻结 SHA-256}"
: "${G2C_N_CAND:?导出预冻结 G2C paired-block 数}"
: "${DEVELOPMENT_GATE_VERIFIER_SHA256:?导出冻结 development-gate verifier SHA-256}"
: "${G2C_ROW_REPORT:?导出本 trial 的已验证 G2C 顺序行报告}"
: "${G2C_ROW_REPORT_SHA256:?导出 G2C 顺序行报告 SHA-256}"
: "${G2C_PATH_JSON:?导出冻结 H0 路径绝对路径}"
: "${G2C_PATH_SHA256:?导出 H0 SHA-256}"
: "${PILOT_METHOD:?从 G2C 顺序表读取 W2 或 W5}"
: "${G2C_BLOCK:?从 G2C 顺序表读取 block ID}"
: "${G2C_ORDER_POSITION:?从 G2C 顺序表读取 position}"
: "${G2C_BLOCK_SEGMENT_ID:?导出 block segment ID}"
: "${DEV_REPEAT:?导出两位 attempt 序号；首次固定 01}"
: "${DEV_MAX_REPEAT:?导出该 gate 预冻结的最大 repeat}"
export RETRY_OF_ATTEMPT_ID="${RETRY_OF_ATTEMPT_ID:-}"
export RETRY_REASON_FILE="${RETRY_REASON_FILE:-}"
export RETRY_REASON_FILE_SHA256="${RETRY_REASON_FILE_SHA256:-}"
export RETRY_FAILURE_EVIDENCE_MANIFEST="${RETRY_FAILURE_EVIDENCE_MANIFEST:-}"
export RETRY_FAILURE_EVIDENCE_MANIFEST_SHA256="${RETRY_FAILURE_EVIDENCE_MANIFEST_SHA256:-}"
export EXPECTED_SPMPC_PKG="${SCOUT_WS}/src/scout_apps/control/spmpc_local_planner"
export RESOLVED_SPMPC_PKG="$(readlink -f "$(rospack find spmpc_local_planner)")"
[[ "${RESOLVED_SPMPC_PKG}" == "$(readlink -f "${EXPECTED_SPMPC_PKG}")" ]]
export DEVELOPMENT_GATE_VERIFIER="${RESOLVED_SPMPC_PKG}/scripts/verify_spmpc_development_gate.py"
assert_unique_kv_keys() {
  local file_path="$1" duplicate_keys
  duplicate_keys="$(
    sed -n 's/^\([[:alnum:]_][[:alnum:]_]*\)=.*/\1/p' "${file_path}" \
      | sort | uniq -d
  )"
  [[ -z "${duplicate_keys}" ]]
}
report_field() {
  local report_path="$1" field_name="$2" field_count
  field_count="$(grep -c "^${field_name}=" "${report_path}" || true)"
  [[ "${field_count}" == 1 ]]
  sed -n "s/^${field_name}=//p" "${report_path}"
}
require_report_field() {
  [[ "$(report_field "$1" "$2")" == "$3" ]]
}
require_report_sha256_field() {
  local field_value
  field_value="$(report_field "$1" "$2")"
  [[ "${field_value}" =~ ^[0-9a-f]{64}$ ]]
}
require_report_uint() {
  local field_value
  field_value="$(report_field "$1" "$2")"
  [[ "${field_value}" =~ ^[0-9]+$ ]]
  printf '%s\n' "${field_value}"
}

g2c_prereg_artifacts=(
  candidate_confirmation_prereg.yaml
  candidate_confirmation_order.csv
  candidate_confirmation_analysis.yaml
)
for prereg_file in "${g2c_prereg_artifacts[@]}"; do
  test -s "${G2C_PREREG_ROOT}/${prereg_file}"
done
test -s "${G2C_PREREG_ROOT}/sha256.txt"
(cd "${G2C_PREREG_ROOT}" && sha256sum --check sha256.txt)
G2C_PREREG_INDEX_ACTUAL_SHA256="$(sha256sum "${G2C_PREREG_ROOT}/sha256.txt" | awk '{print $1}')"
[[ "${G2C_PREREG_INDEX_ACTUAL_SHA256}" == "${G2C_PREREG_INDEX_SHA256}" ]]
for prereg_file in "${g2c_prereg_artifacts[@]}"; do
  entry_count="$(
    awk -v target="${prereg_file}" '
      { name=$2; sub(/^\*/, "", name); if (name == target) count += 1 }
      END { print count + 0 }
    ' "${G2C_PREREG_ROOT}/sha256.txt"
  )"
  [[ "${entry_count}" == 1 ]]
done
G2C_ORDER_CSV="${G2C_PREREG_ROOT}/candidate_confirmation_order.csv"
G2C_ORDER_SHA256="$(sha256sum "${G2C_ORDER_CSV}" | awk '{print $1}')"
[[ "${G2C_N_CAND}" =~ ^[1-9][0-9]*$ ]]
python3 - "${G2C_ORDER_CSV}" "${G2C_N_CAND}" \
  "${G2C_BLOCK}" "${G2C_ORDER_POSITION}" "${PILOT_METHOD}" \
  "${G2C_BLOCK_SEGMENT_ID}" <<'PY'
import csv
import sys

path, n_expected, block, position, method, segment = sys.argv[1:]
with open(path, newline="", encoding="utf-8") as stream:
    reader = csv.DictReader(stream)
    required = {"block", "order_position", "pilot_method", "block_segment_id"}
    if not reader.fieldnames or not required.issubset(reader.fieldnames):
        raise SystemExit("invalid G2C order-table header")
    rows = list(reader)
blocks = {}
for row in rows:
    key = row["block"]
    blocks.setdefault(key, []).append(row)
if len(blocks) != int(n_expected):
    raise SystemExit("G2C block count does not match G2C_N_CAND")
for key, group in blocks.items():
    if len(group) != 2:
        raise SystemExit(f"G2C block {key} is not a complete pair")
    if {row["pilot_method"] for row in group} != {"W2", "W5"}:
        raise SystemExit(f"G2C block {key} does not contain W2/W5 exactly once")
    if {row["order_position"] for row in group} != {"01", "02"}:
        raise SystemExit(f"G2C block {key} has invalid positions")
    if len({row["block_segment_id"] for row in group}) != 1:
        raise SystemExit(f"G2C block {key} crosses block segments")
matches = [
    row for row in rows
    if row["block"] == block
    and row["order_position"] == position
    and row["pilot_method"] == method
    and row["block_segment_id"] == segment
]
if len(matches) != 1:
    raise SystemExit("G2C row report is not a unique member of the frozen order table")
PY
case "${PILOT_METHOD}" in W2|W5) ;; *) exit 2 ;; esac
test -s "${SOURCE_SELECTION_REPORT}"
test -s "${G2S_DATASET_INDEX}"
test -s "${G2C_ROW_REPORT}"
test -s "${G2C_PATH_JSON}"
assert_unique_kv_keys "${SOURCE_SELECTION_REPORT}"
assert_unique_kv_keys "${G2C_ROW_REPORT}"
[[ "$(git -C "${SCOUT_WS}" rev-parse HEAD)" == "${DEV_RELEASE_GIT_REVISION}" ]]
[[ -z "$(git -C "${SCOUT_WS}" status --porcelain)" ]]
SOURCE_SELECTION_ACTUAL_SHA256="$(sha256sum "${SOURCE_SELECTION_REPORT}" | awk '{print $1}')"
[[ "${SOURCE_SELECTION_ACTUAL_SHA256}" == "${SOURCE_SELECTION_REPORT_SHA256}" ]]
G2S_DATASET_INDEX_ACTUAL_SHA256="$(sha256sum "${G2S_DATASET_INDEX}" | awk '{print $1}')"
[[ "${G2S_DATASET_INDEX_ACTUAL_SHA256}" == "${G2S_DATASET_INDEX_SHA256}" ]]
require_report_field "${SOURCE_SELECTION_REPORT}" report_type G2S_SOURCE_SELECTION
require_report_field "${SOURCE_SELECTION_REPORT}" release_id "${DEV_RELEASE_ID}"
require_report_field "${SOURCE_SELECTION_REPORT}" git_revision "${DEV_RELEASE_GIT_REVISION}"
require_report_field "${SOURCE_SELECTION_REPORT}" status PASS
require_report_field "${SOURCE_SELECTION_REPORT}" selected_source odom
require_report_field "${SOURCE_SELECTION_REPORT}" \
  g2a_audit_report_sha256 "${G2A_AUDIT_REPORT_SHA256}"
require_report_field "${SOURCE_SELECTION_REPORT}" \
  imu_cal_validation_report_sha256 "${IMU_CAL_VALIDATION_REPORT_SHA256}"
require_report_field "${SOURCE_SELECTION_REPORT}" \
  g2s_prereg_index_sha256 "${G2S_PREREG_INDEX_SHA256}"
require_report_field "${SOURCE_SELECTION_REPORT}" \
  processed_imu_config_sha256 "${PROCESSED_IMU_CONFIG_SHA256}"
require_report_field "${SOURCE_SELECTION_REPORT}" rgb_calib_sha256 "${RGB_CALIB_SHA256}"
require_report_field "${SOURCE_SELECTION_REPORT}" \
  dataset_index_sha256 "${G2S_DATASET_INDEX_SHA256}"
require_report_field "${SOURCE_SELECTION_REPORT}" \
  development_gate_verifier_sha256 "${DEVELOPMENT_GATE_VERIFIER_SHA256}"
for hash_field in metric_config_sha256 analyzer_sha256 decision_rule_sha256; do
  require_report_sha256_field "${SOURCE_SELECTION_REPORT}" "${hash_field}"
done
source_n_src="$(require_report_uint "${SOURCE_SELECTION_REPORT}" n_src)"
source_row_coverage="$(require_report_uint "${SOURCE_SELECTION_REPORT}" planned_row_coverage_count)"
source_min_eligible="$(require_report_uint "${SOURCE_SELECTION_REPORT}" minimum_continuous_eligible_count)"
source_eligible="$(require_report_uint "${SOURCE_SELECTION_REPORT}" continuous_eligible_count)"
source_attempts="$(require_report_uint "${SOURCE_SELECTION_REPORT}" attempt_count)"
source_postflight_complete_attempts="$(require_report_uint "${SOURCE_SELECTION_REPORT}" postflight_complete_attempt_count)"
source_acquisition_failure_attempts="$(require_report_uint "${SOURCE_SELECTION_REPORT}" acquisition_failure_attempt_count)"
source_readiness_failure_attempts="$(require_report_uint "${SOURCE_SELECTION_REPORT}" readiness_failure_attempt_count)"
source_method_success_rows="$(require_report_uint "${SOURCE_SELECTION_REPORT}" method_success_planned_row_count)"
source_method_failure_rows="$(require_report_uint "${SOURCE_SELECTION_REPORT}" method_failure_planned_row_count)"
source_unresolved_acquisition_rows="$(require_report_uint "${SOURCE_SELECTION_REPORT}" unresolved_acquisition_planned_row_count)"
[[ "${source_n_src}" == "${G2S_N_SRC}" ]]
(( source_n_src > 0 ))
(( source_row_coverage == source_n_src ))
(( source_min_eligible > 0 && source_min_eligible <= source_n_src ))
(( source_eligible >= source_min_eligible && source_eligible <= source_n_src ))
(( source_attempts >= source_n_src ))
(( source_postflight_complete_attempts <= source_attempts ))
(( source_acquisition_failure_attempts <= source_attempts ))
(( source_readiness_failure_attempts <= source_attempts ))
(( source_method_success_rows <= source_n_src ))
(( source_method_failure_rows <= source_n_src ))
(( source_unresolved_acquisition_rows <= source_n_src ))
(( source_method_success_rows + source_method_failure_rows + source_unresolved_acquisition_rows == source_n_src ))
(( source_eligible <= source_method_success_rows ))
G2C_ROW_ACTUAL_SHA256="$(sha256sum "${G2C_ROW_REPORT}" | awk '{print $1}')"
[[ "${G2C_ROW_ACTUAL_SHA256}" == "${G2C_ROW_REPORT_SHA256}" ]]
for expected_line in \
  "gate=G2C" \
  "release_id=${DEV_RELEASE_ID}" \
  "git_revision=${DEV_RELEASE_GIT_REVISION}" \
  "current_observer_source=odom" \
  "g2c_prereg_index_sha256=${G2C_PREREG_INDEX_SHA256}" \
  "order_table_sha256=${G2C_ORDER_SHA256}" \
  "n_cand=${G2C_N_CAND}" \
  "development_gate_verifier_sha256=${DEVELOPMENT_GATE_VERIFIER_SHA256}" \
  "source_selection_report_sha256=${SOURCE_SELECTION_REPORT_SHA256}" \
  "pilot_method=${PILOT_METHOD}" \
  "block=${G2C_BLOCK}" \
  "order_position=${G2C_ORDER_POSITION}" \
  "block_segment_id=${G2C_BLOCK_SEGMENT_ID}" \
  "path_sha256=${G2C_PATH_SHA256}"; do
  grep -Fxq "${expected_line}" "${G2C_ROW_REPORT}"
done

[[ "${DEV_REPEAT}" =~ ^(0[1-9]|[1-9][0-9])$ ]]
[[ "${DEV_MAX_REPEAT}" =~ ^(0[1-9]|[1-9][0-9])$ ]]
(( 10#${DEV_REPEAT} <= 10#${DEV_MAX_REPEAT} ))
attempt_prefix="DEV_G2C_H0_C1_${PILOT_METHOD}_b${G2C_BLOCK}"
export RUN_LABEL="${attempt_prefix}_r${DEV_REPEAT}"
export ATTEMPT_ID="${RUN_LABEL}"
if [[ "${DEV_REPEAT}" == 01 ]]; then
  export ACQUISITION_RETRY=false
  [[ -z "${RETRY_OF_ATTEMPT_ID}" ]]
  [[ -z "${RETRY_REASON_FILE}" ]]
  [[ -z "${RETRY_REASON_FILE_SHA256}" ]]
  [[ -z "${RETRY_FAILURE_EVIDENCE_MANIFEST}" ]]
  [[ -z "${RETRY_FAILURE_EVIDENCE_MANIFEST_SHA256}" ]]
else
  export ACQUISITION_RETRY=true
  [[ -n "${RETRY_OF_ATTEMPT_ID}" ]]
  [[ "${RETRY_OF_ATTEMPT_ID%_r*}" == "${attempt_prefix}" ]]
  previous_repeat="${RETRY_OF_ATTEMPT_ID##*_r}"
  [[ "${previous_repeat}" =~ ^(0[1-9]|[1-9][0-9])$ ]]
  (( 10#${DEV_REPEAT} == 10#${previous_repeat} + 1 ))
  test -s "${RETRY_REASON_FILE}"
  [[ "${RETRY_REASON_FILE_SHA256}" =~ ^[0-9a-f]{64}$ ]]
  [[ "$(sha256sum "${RETRY_REASON_FILE}" | awk '{print $1}')" == "${RETRY_REASON_FILE_SHA256}" ]]
  test -s "${RETRY_FAILURE_EVIDENCE_MANIFEST}"
  [[ "${RETRY_FAILURE_EVIDENCE_MANIFEST_SHA256}" =~ ^[0-9a-f]{64}$ ]]
  [[ "$(sha256sum "${RETRY_FAILURE_EVIDENCE_MANIFEST}" | awk '{print $1}')" == "${RETRY_FAILURE_EVIDENCE_MANIFEST_SHA256}" ]]
  assert_unique_kv_keys "${RETRY_REASON_FILE}"
  assert_unique_kv_keys "${RETRY_FAILURE_EVIDENCE_MANIFEST}"
  for expected_line in \
    "report_type=DEVELOPMENT_RETRY_AUTHORIZATION" \
    "status=PASS" \
    "gate=G2C" \
    "release_id=${DEV_RELEASE_ID}" \
    "git_revision=${DEV_RELEASE_GIT_REVISION}" \
    "planned_row_report_sha256=${G2C_ROW_REPORT_SHA256}" \
    "failed_attempt_id=${RETRY_OF_ATTEMPT_ID}" \
    "authorized_attempt_id=${ATTEMPT_ID}" \
    "block_segment_id=${G2C_BLOCK_SEGMENT_ID}" \
    "failure_class=METHOD_INDEPENDENT_ACQUISITION" \
    "method_failure=false" \
    "retry_authorized=true" \
    "failure_evidence_manifest_path=${RETRY_FAILURE_EVIDENCE_MANIFEST}" \
    "failure_evidence_manifest_sha256=${RETRY_FAILURE_EVIDENCE_MANIFEST_SHA256}" \
    "development_gate_verifier_sha256=${DEVELOPMENT_GATE_VERIFIER_SHA256}"; do
    grep -Fxq "${expected_line}" "${RETRY_REASON_FILE}"
  done
  for expected_line in \
    "report_type=DEVELOPMENT_FAILED_ATTEMPT_EVIDENCE" \
    "status=CLASSIFIED" \
    "gate=G2C" \
    "release_id=${DEV_RELEASE_ID}" \
    "git_revision=${DEV_RELEASE_GIT_REVISION}" \
    "planned_row_report_sha256=${G2C_ROW_REPORT_SHA256}" \
    "attempt_id=${RETRY_OF_ATTEMPT_ID}" \
    "block_segment_id=${G2C_BLOCK_SEGMENT_ID}" \
    "failure_class=METHOD_INDEPENDENT_ACQUISITION" \
    "method_failure=false" \
    "eligible_outcome=false"; do
    grep -Fxq "${expected_line}" "${RETRY_FAILURE_EVIDENCE_MANIFEST}"
  done
  grep -Eq '^raw_artifact_index_sha256=([0-9a-f]{64}|none)$' "${RETRY_FAILURE_EVIDENCE_MANIFEST}"
  startup_log_path="$(sed -n 's/^startup_log_path=//p' "${RETRY_FAILURE_EVIDENCE_MANIFEST}")"
  test -s "${startup_log_path}"
  startup_log_sha256="$(sha256sum "${startup_log_path}" | awk '{print $1}')"
  grep -Fxq "startup_log_sha256=${startup_log_sha256}" "${RETRY_FAILURE_EVIDENCE_MANIFEST}"
fi

test -x "${DEVELOPMENT_GATE_VERIFIER}"
DEVELOPMENT_GATE_VERIFIER_ACTUAL_SHA256="$(sha256sum "${DEVELOPMENT_GATE_VERIFIER}" | awk '{print $1}')"
[[ "${DEVELOPMENT_GATE_VERIFIER_ACTUAL_SHA256}" == "${DEVELOPMENT_GATE_VERIFIER_SHA256}" ]]
export G2C_PREREQ_VERIFICATION_REPORT="${G2C_ROW_REPORT}.${ATTEMPT_ID}.prereq_verification.txt"
test ! -e "${G2C_PREREQ_VERIFICATION_REPORT}"
"${DEVELOPMENT_GATE_VERIFIER}" verify-prerequisites \
  --gate G2C \
  --release-id "${DEV_RELEASE_ID}" \
  --git-revision "${DEV_RELEASE_GIT_REVISION}" \
  --source-selection-report "${SOURCE_SELECTION_REPORT}" \
  --source-selection-sha256 "${SOURCE_SELECTION_REPORT_SHA256}" \
  --upstream-dataset-index "${G2S_DATASET_INDEX}" \
  --upstream-dataset-index-sha256 "${G2S_DATASET_INDEX_SHA256}" \
  --prereg-root "${G2C_PREREG_ROOT}" \
  --prereg-index-sha256 "${G2C_PREREG_INDEX_SHA256}" \
  --row-report "${G2C_ROW_REPORT}" --row-sha256 "${G2C_ROW_REPORT_SHA256}" \
  --attempt-id "${ATTEMPT_ID}" --dev-repeat "${DEV_REPEAT}" \
  --max-repeat "${DEV_MAX_REPEAT}" --acquisition-retry "${ACQUISITION_RETRY}" \
  --retry-of-attempt-id "${RETRY_OF_ATTEMPT_ID:-none}" \
  --retry-reason-file "${RETRY_REASON_FILE:-none}" \
  --retry-reason-file-sha256 "${RETRY_REASON_FILE_SHA256:-none}" \
  --failure-evidence-manifest "${RETRY_FAILURE_EVIDENCE_MANIFEST:-none}" \
  --failure-evidence-manifest-sha256 "${RETRY_FAILURE_EVIDENCE_MANIFEST_SHA256:-none}" \
  --path-json "${G2C_PATH_JSON}" --path-sha256 "${G2C_PATH_SHA256}" \
  --output "${G2C_PREREQ_VERIFICATION_REPORT}"
grep -Fxq 'status=PASS' "${G2C_PREREQ_VERIFICATION_REPORT}"
G2C_PREREQ_VERIFICATION_SHA256="$(sha256sum "${G2C_PREREQ_VERIFICATION_REPORT}" | awk '{print $1}')"

export RUN_OUT_DIR="${SLOSH_BAG_ROOT}/real/${DEV_DATE}_spmpc_development/G2C/${PILOT_METHOD}"
export CURRENT_OBSERVER_SOURCE=odom
[[ "${CURRENT_OBSERVER_SOURCE}" == odom ]]

unset MATRIX_PRESET RECORDER_SCRIPT VARIANT ALG W_SLOSH

PILOT_MODE=true \
PILOT_METHOD="${PILOT_METHOD}" \
PILOT_CONDITION="G2C_${PILOT_METHOD}" \
SOLVER_BACKEND=continuous_mpcc_acados \
REF_TOPIC=/scout/global_path_fixed \
CMD_TOPIC=/cmd_vel \
COSTMAP_TOPIC=/map \
REFERENCE_TARGET_FRAME=map \
BASE_FRAME=base_link \
PATH_SOURCE_MODE=replay \
PATH_FILE="${G2C_PATH_JSON}" \
PATH_EXPECTED_SHA256="${G2C_PATH_SHA256}" \
REQUIRE_PATH_HASH=true \
BLOCK_SEGMENT_ID="${G2C_BLOCK_SEGMENT_ID}" \
ORDER_POSITION="${G2C_ORDER_POSITION}" \
SPLIT_BLOCK=false \
ACQUISITION_RETRY="${ACQUISITION_RETRY}" \
RETRY_REASON_FILE="${RETRY_REASON_FILE}" \
RUN_LABEL="${RUN_LABEL}" \
NAME="${RUN_LABEL}" \
RUN_OUT_DIR="${RUN_OUT_DIR}" \
DATE="${DEV_DATE}" \
PILOT_RECORD_RGB=false \
RECORD_TOPIC_INFO=true \
RECORD_STANDALONE_SLOSH=true \
RECORD_SCAN=true \
RECORD_MOCAP=false \
RECORD_ROSOUT=true \
RECORD_ALL_EXISTING_TOPICS=false \
LIQUID_EXPORT_AFTER_RECORD=false \
ROSBAG_BUFFER_SIZE_MB=4096 \
RECORD_SEC=60 \
MAX_RECORD_SEC=60 \
START_POS_TOL=0.08 \
START_YAW_TOL=0.15 \
START_HOLD_SEC=0.5 \
START_GATE_TIMEOUT_SEC=15 \
PATH_PUBLISH_RATE=2.0 \
PATH_GENERATOR_STARTUP_SEC=2 \
RECORDER_STARTUP_SEC=8 \
PLANNER_STARTUP_SEC=2 \
SEND_ZERO_ON_EXIT=true \
V_REF=0.20 \
SLOSH_HEIGHT_MAX=-1.0 \
ALPHA_MAX=1.2 \
SHARED_LINEAR_ACCEL_LIMIT_ENABLE=true \
SHARED_LINEAR_ACCEL_MAX=0.6 \
SHARED_ANGULAR_LIMIT_ENABLE=true \
SHARED_ANGULAR_RATE_MAX=1.2 \
SHARED_ANGULAR_ACCEL_MAX=1.2 \
DELAY_PHASE_MODE=fixed_closed_loop \
DELAY_PHASE_LINEAR_DELAY_SEC=0.15 \
DELAY_PHASE_ANGULAR_DELAY_SEC=0.22 \
IMU_SHADOW_ENABLE=false \
OPERATOR_NOTE="development G2C release=${DEV_RELEASE_ID}; revision=${DEV_RELEASE_GIT_REVISION}; attempt_id=${ATTEMPT_ID}; repeat=${DEV_REPEAT}; retry_of=${RETRY_OF_ATTEMPT_ID:-none}; retry_reason_sha256=${RETRY_REASON_FILE_SHA256:-none}; failure_evidence_sha256=${RETRY_FAILURE_EVIDENCE_MANIFEST_SHA256:-none}; source=odom; source_report_sha256=${SOURCE_SELECTION_REPORT_SHA256}; prereg_index_sha256=${G2C_PREREG_INDEX_SHA256}; row_sha256=${G2C_ROW_REPORT_SHA256}; prereq_verification_sha256=${G2C_PREREQ_VERIFICATION_SHA256}; method=${PILOT_METHOD}" \
bash src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_real_fixed_path_trial.sh
```

该命令只能筛安全、trajectory mechanism、tracking、runtime 和内部候选差异，不能凭 `H_modal` 宣告物理有效。全部 W2/W5 planned rows 结束后先生成 G2C dataset index，逐行绑定 prereg、planned row、每个 `ATTEMPT_ID`/retry chain、prerequisite-verification、postflight、bag 与失败/排除 hash。G2C 完成后的 final-candidate report 至少以唯一 key 记录 `report_type/status/release_id/git_revision`、`current_observer_source`、source-selection/G2C-prereg/verifier/完整 G2C dataset-index hashes、`final_pilot_method`，以及 `n_cand/planned_row_coverage_count/minimum_eligible_pair_count/eligible_pair_count/attempt_count/postflight_complete_attempt_count/acquisition_failure_attempt_count/method_success_planned_row_count/method_failure_planned_row_count/unresolved_acquisition_planned_row_count/valid_unpaired_planned_row_count`、acceptance/failure/analyzer hashes；attempt 分类与 planned-row partition 均由 dataset index 独立重算，不得用 `attempts-2*pairs` 代替。随后整体计算 SHA-256，供每条 G3 row 反向绑定。当前 one-click runner 尚不生成 monitor-reset/`T_SETTLE`/release-hash 全部证据，且 runner 退出 0 不证明 bag 可读；未补齐 wrapper 并通过第 4.5.1 节 postflight 前，只能作执行器 smoke，不能单独签署 G2C。

### 4.5 G3：独立 RGB efficacy pilot

第一条 G3 run 前必须已有：

- 一个最终 Bslosh candidate；
- held-out formal H1 之外的 H0/H0b；
- 精确 `n_dev` 和两条件随机表；
- $\delta_{H,dev}$、success、tracking、runtime 和 no-early-stop 规则；
- 两条件公平的调试/调参预算与 single-block-dominance 规则；
- RGB/同步/visual-start QC 版本，以及统一的 `T_HVIS_TAIL`/outcome-window rule；
- 与 G2S/G2C artifact 一致的 development release-lineage ID。

下列 G3 one-click 模板同样只适用于 odom release，因为当前 runner/launch 没有 source selector。若 G2S 选择 IMU，必须在新实现中由只读 release 明确导出 `CURRENT_OBSERVER_SOURCE=processed_imu`、在线验证 effective source 与 fallback policy，并建立新的 G3 模板；不得沿用下面的 `IMU_SHADOW_ENABLE=false` 命令冒充 IMU 输入。

当前默认建议 `n_dev=4`，但只有 preregistration 文件可以决定实际值。

第一条 G3 前冻结并对外登记 index hash：

```text
development/G3/
├── efficacy_prereg.yaml              # 直接绑定 t_hvis_tail_sec 与 g3_outcome_window_rule_sha256
├── efficacy_order.csv               # block,order_position,condition,pilot_method,block_segment_id
├── rgb_sync_and_metric.yaml          # 含唯一顶层 t_hvis_tail_sec；整文件 hash 即 outcome-window rule hash
└── sha256.txt
```

完整顺序表必须使每个 block 恰含 `Bsmooth/Bslosh` 各一次，并固定 Bslosh 的 final `W2` 或 `W5`；逐条 row 必须绑定整个 prereg index 与 order-table hash，才能执行 no-early-stop 和完整 paired denominator。`rgb_sync_and_metric.yaml` 必须只有一个顶层 `t_hvis_tail_sec: <positive number>`，并使用首条 G3 前冻结的 canonical 十进制文本；其整文件 SHA-256 定义为 `G3_OUTCOME_WINDOW_RULE_SHA256`，文件内部不得反向写入自身 hash。`efficacy_prereg.yaml` 必须以唯一顶层字段直接重复绑定相同的 `t_hvis_tail_sec` 与 `g3_outcome_window_rule_sha256`，不能只依赖 bundle index 的间接关系。下面的代码只是 canonical-text 初筛，会拒绝正常拼写下的重复字段和显式自 hash，但**不能证明 YAML 唯一语义**；quoted/escaped/inline/nested/duplicate/alias/merge 变体必须由最终冻结 verifier 使用 reject-duplicate-keys 的 safe YAML loader 全部拒绝。该 verifier 尚未实现时 G3 仍为 `NO-GO`。首条 G3 前二者一经登记就不得修改；G3 prereg、每条 row、最终 `G3_EFFICACY` report、G6 和 formal `RUN_ENV` 必须原样继承。任何修改都建立新 development release，并从 G3 第一条 planned row 重做。

one-click runner 只可作为 development 执行器；示例条件映射：

| `DEV_CONDITION` | `PILOT_METHOD` | RGB |
| --- | --- | ---: |
| `Bsmooth` | `Bsmooth` | true |
| `Bslosh` | 最终 `W<number>` | true |

```bash
set -euo pipefail
export SCOUT_WS="${SCOUT_WS:-/home/zrj/scout_ws}"
export SLOSH_BAG_ROOT="${SLOSH_BAG_ROOT:-/home/zrj/slosh_bags}"
test -r /opt/ros/noetic/setup.bash
source /opt/ros/noetic/setup.bash
test -r "${SCOUT_WS}/devel/setup.bash"
source "${SCOUT_WS}/devel/setup.bash"
[[ "${ROS_VERSION:-}" == 1 && "${ROS_DISTRO:-}" == noetic ]]
: "${DEV_DATE:?先导出 development 日期}"
: "${DEV_RELEASE_ID:?导出贯穿 G2S/G2C/G3 的 release-lineage ID}"
: "${DEV_RELEASE_GIT_REVISION:?导出 G3 release Git revision}"
: "${SOURCE_SELECTION_REPORT:?导出冻结 G2S 决策报告}"
: "${SOURCE_SELECTION_REPORT_SHA256:?导出 G2S 决策报告 SHA-256}"
: "${FINAL_CANDIDATE_REPORT:?导出 G2C 最终候选报告}"
: "${FINAL_CANDIDATE_REPORT_SHA256:?导出 G2C 最终候选报告 SHA-256}"
: "${G2C_DATASET_INDEX:?导出全部 G2C row/prereq/postflight/bag hash 的数据集 index}"
: "${G2C_DATASET_INDEX_SHA256:?导出 G2C dataset index SHA-256}"
: "${G2C_PREREG_INDEX_SHA256:?导出 G2C prereg index SHA-256}"
: "${G2C_N_CAND:?导出 G2C 预冻结 paired-block 数}"
: "${FINAL_PILOT_METHOD:?导出 G2C 冻结的唯一 W<number> candidate}"
: "${G3_PREREG_ROOT:?导出 development/G3 预注册目录}"
: "${G3_PREREG_INDEX_SHA256:?导出 G3 sha256.txt 的外部冻结 SHA-256}"
: "${T_HVIS_TAIL:?导出 G3 前冻结的统一 post-arrival tail 秒数}"
: "${G3_OUTCOME_WINDOW_RULE_SHA256:?导出 rgb_sync_and_metric.yaml 的冻结 SHA-256}"
: "${G3_N_DEV:?导出预冻结 G3 paired-block 数}"
: "${DEVELOPMENT_GATE_VERIFIER_SHA256:?导出冻结 development-gate verifier SHA-256}"
: "${G3_ROW_REPORT:?导出本 trial 的已验证 G3 顺序行报告}"
: "${G3_ROW_REPORT_SHA256:?导出 G3 顺序行报告 SHA-256}"
: "${G3_PATH_JSON:?先导出冻结且未进入 formal 的 H0b 路径绝对路径}"
: "${G3_PATH_SHA256:?先导出 development registry 中的 H0b SHA-256}"
: "${DEV_CONDITION:?从 G3 两条件随机表读取 Bsmooth 或 Bslosh}"
: "${G3_BLOCK:?从 G3 development 随机表读取 block ID}"
: "${G3_ORDER_POSITION:?从 G3 development 随机表读取 position}"
: "${G3_BLOCK_SEGMENT_ID:?导出 G3 block segment ID}"
: "${DEV_REPEAT:?导出两位 attempt 序号；首次固定 01}"
: "${DEV_MAX_REPEAT:?导出该 gate 预冻结的最大 repeat}"
export RETRY_OF_ATTEMPT_ID="${RETRY_OF_ATTEMPT_ID:-}"
export RETRY_REASON_FILE="${RETRY_REASON_FILE:-}"
export RETRY_REASON_FILE_SHA256="${RETRY_REASON_FILE_SHA256:-}"
export RETRY_FAILURE_EVIDENCE_MANIFEST="${RETRY_FAILURE_EVIDENCE_MANIFEST:-}"
export RETRY_FAILURE_EVIDENCE_MANIFEST_SHA256="${RETRY_FAILURE_EVIDENCE_MANIFEST_SHA256:-}"
export EXPECTED_SPMPC_PKG="${SCOUT_WS}/src/scout_apps/control/spmpc_local_planner"
export RESOLVED_SPMPC_PKG="$(readlink -f "$(rospack find spmpc_local_planner)")"
[[ "${RESOLVED_SPMPC_PKG}" == "$(readlink -f "${EXPECTED_SPMPC_PKG}")" ]]
export DEVELOPMENT_GATE_VERIFIER="${RESOLVED_SPMPC_PKG}/scripts/verify_spmpc_development_gate.py"
assert_unique_kv_keys() {
  local file_path="$1" duplicate_keys
  duplicate_keys="$(
    sed -n 's/^\([[:alnum:]_][[:alnum:]_]*\)=.*/\1/p' "${file_path}" \
      | sort | uniq -d
  )"
  [[ -z "${duplicate_keys}" ]]
}
report_field() {
  local report_path="$1" field_name="$2" field_count
  field_count="$(grep -c "^${field_name}=" "${report_path}" || true)"
  [[ "${field_count}" == 1 ]]
  sed -n "s/^${field_name}=//p" "${report_path}"
}
require_report_field() {
  [[ "$(report_field "$1" "$2")" == "$3" ]]
}
require_report_sha256_field() {
  local field_value
  field_value="$(report_field "$1" "$2")"
  [[ "${field_value}" =~ ^[0-9a-f]{64}$ ]]
}
require_report_uint() {
  local field_value
  field_value="$(report_field "$1" "$2")"
  [[ "${field_value}" =~ ^[0-9]+$ ]]
  printf '%s\n' "${field_value}"
}

g3_prereg_artifacts=(
  efficacy_prereg.yaml
  efficacy_order.csv
  rgb_sync_and_metric.yaml
)
for prereg_file in "${g3_prereg_artifacts[@]}"; do
  test -s "${G3_PREREG_ROOT}/${prereg_file}"
done
test -s "${G3_PREREG_ROOT}/sha256.txt"
(cd "${G3_PREREG_ROOT}" && sha256sum --check sha256.txt)
G3_PREREG_INDEX_ACTUAL_SHA256="$(sha256sum "${G3_PREREG_ROOT}/sha256.txt" | awk '{print $1}')"
[[ "${G3_PREREG_INDEX_ACTUAL_SHA256}" == "${G3_PREREG_INDEX_SHA256}" ]]
for prereg_file in "${g3_prereg_artifacts[@]}"; do
  entry_count="$(
    awk -v target="${prereg_file}" '
      { name=$2; sub(/^\*/, "", name); if (name == target) count += 1 }
      END { print count + 0 }
    ' "${G3_PREREG_ROOT}/sha256.txt"
  )"
  [[ "${entry_count}" == 1 ]]
done
G3_ORDER_CSV="${G3_PREREG_ROOT}/efficacy_order.csv"
G3_ORDER_SHA256="$(sha256sum "${G3_ORDER_CSV}" | awk '{print $1}')"
G3_EFFICACY_PREREG="${G3_PREREG_ROOT}/efficacy_prereg.yaml"
G3_RGB_SYNC_AND_METRIC="${G3_PREREG_ROOT}/rgb_sync_and_metric.yaml"
G3_OUTCOME_WINDOW_RULE_ACTUAL_SHA256="$(sha256sum "${G3_RGB_SYNC_AND_METRIC}" | awk '{print $1}')"
[[ "${G3_OUTCOME_WINDOW_RULE_SHA256}" =~ ^[0-9a-f]{64}$ ]]
[[ "${G3_OUTCOME_WINDOW_RULE_ACTUAL_SHA256}" == "${G3_OUTCOME_WINDOW_RULE_SHA256}" ]]
[[ "${G3_N_DEV}" =~ ^[1-9][0-9]*$ ]]
python3 - "${G3_ORDER_CSV}" "${G3_N_DEV}" "${FINAL_PILOT_METHOD}" \
  "${G3_BLOCK}" "${G3_ORDER_POSITION}" "${DEV_CONDITION}" \
  "${G3_BLOCK_SEGMENT_ID}" "${G3_RGB_SYNC_AND_METRIC}" "${T_HVIS_TAIL}" \
  "${G3_EFFICACY_PREREG}" "${G3_OUTCOME_WINDOW_RULE_SHA256}" <<'PY'
import csv
from decimal import Decimal, InvalidOperation
import re
import sys

(
    path,
    n_expected,
    final_method,
    block,
    position,
    condition,
    segment,
    metric_path,
    expected_tail,
    prereg_path,
    expected_rule_hash,
) = sys.argv[1:]
with open(path, newline="", encoding="utf-8") as stream:
    reader = csv.DictReader(stream)
    required = {"block", "order_position", "condition", "pilot_method", "block_segment_id"}
    if not reader.fieldnames or not required.issubset(reader.fieldnames):
        raise SystemExit("invalid G3 order-table header")
    rows = list(reader)
blocks = {}
for row in rows:
    blocks.setdefault(row["block"], []).append(row)
if len(blocks) != int(n_expected):
    raise SystemExit("G3 block count does not match G3_N_DEV")
for key, group in blocks.items():
    if len(group) != 2:
        raise SystemExit(f"G3 block {key} is not a complete pair")
    if {row["condition"] for row in group} != {"Bsmooth", "Bslosh"}:
        raise SystemExit(f"G3 block {key} lacks one Bsmooth/Bslosh trial")
    if {row["order_position"] for row in group} != {"01", "02"}:
        raise SystemExit(f"G3 block {key} has invalid positions")
    if len({row["block_segment_id"] for row in group}) != 1:
        raise SystemExit(f"G3 block {key} crosses block segments")
    method_by_condition = {row["condition"]: row["pilot_method"] for row in group}
    if method_by_condition != {"Bsmooth": "Bsmooth", "Bslosh": final_method}:
        raise SystemExit(f"G3 block {key} does not use the frozen final candidate")
expected_method = "Bsmooth" if condition == "Bsmooth" else final_method
matches = [
    row for row in rows
    if row["block"] == block
    and row["order_position"] == position
    and row["condition"] == condition
    and row["pilot_method"] == expected_method
    and row["block_segment_id"] == segment
]
if len(matches) != 1:
    raise SystemExit("G3 row report is not a unique member of the frozen order table")

tail_pattern = re.compile(
    r"^t_hvis_tail_sec:\s*([+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)\s*(?:#.*)?$"
)
with open(metric_path, encoding="utf-8") as stream:
    metric_text = stream.read()
if metric_text.count("t_hvis_tail_sec") != 1:
    raise SystemExit("rgb_sync_and_metric.yaml contains an ambiguous t_hvis_tail_sec spelling")
if "g3_outcome_window_rule_sha256" in metric_text:
    raise SystemExit("rgb_sync_and_metric.yaml must not contain its own SHA-256")
with open(metric_path, encoding="utf-8") as stream:
    tail_values = [
        match.group(1)
        for raw_line in stream
        if (match := tail_pattern.fullmatch(raw_line.rstrip("\r\n")))
    ]
if len(tail_values) != 1:
    raise SystemExit("rgb_sync_and_metric.yaml must contain exactly one top-level t_hvis_tail_sec")
try:
    frozen_tail = Decimal(tail_values[0])
    exported_tail = Decimal(expected_tail)
except InvalidOperation as exc:
    raise SystemExit("invalid T_HVIS_TAIL") from exc
if (
    not frozen_tail.is_finite()
    or frozen_tail <= 0
    or exported_tail != frozen_tail
    or expected_tail != tail_values[0]
):
    raise SystemExit("T_HVIS_TAIL does not match the positive frozen t_hvis_tail_sec")

prereg_tail_pattern = re.compile(
    r"^t_hvis_tail_sec:\s*([+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)\s*(?:#.*)?$"
)
prereg_rule_pattern = re.compile(
    r"^g3_outcome_window_rule_sha256:\s*([0-9a-f]{64})\s*(?:#.*)?$"
)
with open(prereg_path, encoding="utf-8") as stream:
    prereg_text = stream.read()
if prereg_text.count("t_hvis_tail_sec") != 1:
    raise SystemExit("efficacy_prereg.yaml contains an ambiguous t_hvis_tail_sec spelling")
if prereg_text.count("g3_outcome_window_rule_sha256") != 1:
    raise SystemExit("efficacy_prereg.yaml contains an ambiguous rule-hash spelling")
prereg_lines = prereg_text.splitlines()
prereg_tails = [
    match.group(1)
    for line in prereg_lines
    if (match := prereg_tail_pattern.fullmatch(line))
]
prereg_rule_hashes = [
    match.group(1)
    for line in prereg_lines
    if (match := prereg_rule_pattern.fullmatch(line))
]
if prereg_tails != [expected_tail]:
    raise SystemExit("efficacy_prereg.yaml must directly bind the canonical t_hvis_tail_sec")
if prereg_rule_hashes != [expected_rule_hash]:
    raise SystemExit("efficacy_prereg.yaml must directly bind g3_outcome_window_rule_sha256")
PY
test -s "${G3_PATH_JSON}"
test -s "${SOURCE_SELECTION_REPORT}"
test -s "${FINAL_CANDIDATE_REPORT}"
test -s "${G2C_DATASET_INDEX}"
test -s "${G3_ROW_REPORT}"
assert_unique_kv_keys "${SOURCE_SELECTION_REPORT}"
assert_unique_kv_keys "${FINAL_CANDIDATE_REPORT}"
assert_unique_kv_keys "${G3_ROW_REPORT}"
case "${FINAL_PILOT_METHOD}" in W2|W5) ;; *) exit 2 ;; esac
[[ "$(git -C "${SCOUT_WS}" rev-parse HEAD)" == "${DEV_RELEASE_GIT_REVISION}" ]]
[[ -z "$(git -C "${SCOUT_WS}" status --porcelain)" ]]
SOURCE_SELECTION_ACTUAL_SHA256="$(sha256sum "${SOURCE_SELECTION_REPORT}" | awk '{print $1}')"
[[ "${SOURCE_SELECTION_ACTUAL_SHA256}" == "${SOURCE_SELECTION_REPORT_SHA256}" ]]
require_report_field "${SOURCE_SELECTION_REPORT}" report_type G2S_SOURCE_SELECTION
require_report_field "${SOURCE_SELECTION_REPORT}" release_id "${DEV_RELEASE_ID}"
require_report_field "${SOURCE_SELECTION_REPORT}" git_revision "${DEV_RELEASE_GIT_REVISION}"
require_report_field "${SOURCE_SELECTION_REPORT}" status PASS
require_report_field "${SOURCE_SELECTION_REPORT}" selected_source odom
FINAL_CANDIDATE_ACTUAL_SHA256="$(sha256sum "${FINAL_CANDIDATE_REPORT}" | awk '{print $1}')"
[[ "${FINAL_CANDIDATE_ACTUAL_SHA256}" == "${FINAL_CANDIDATE_REPORT_SHA256}" ]]
G2C_DATASET_INDEX_ACTUAL_SHA256="$(sha256sum "${G2C_DATASET_INDEX}" | awk '{print $1}')"
[[ "${G2C_DATASET_INDEX_ACTUAL_SHA256}" == "${G2C_DATASET_INDEX_SHA256}" ]]
require_report_field "${FINAL_CANDIDATE_REPORT}" report_type G2C_FINAL_CANDIDATE
require_report_field "${FINAL_CANDIDATE_REPORT}" release_id "${DEV_RELEASE_ID}"
require_report_field "${FINAL_CANDIDATE_REPORT}" git_revision "${DEV_RELEASE_GIT_REVISION}"
require_report_field "${FINAL_CANDIDATE_REPORT}" status PASS
require_report_field "${FINAL_CANDIDATE_REPORT}" current_observer_source odom
require_report_field "${FINAL_CANDIDATE_REPORT}" \
  source_selection_report_sha256 "${SOURCE_SELECTION_REPORT_SHA256}"
require_report_field "${FINAL_CANDIDATE_REPORT}" \
  g2c_prereg_index_sha256 "${G2C_PREREG_INDEX_SHA256}"
require_report_field "${FINAL_CANDIDATE_REPORT}" \
  dataset_index_sha256 "${G2C_DATASET_INDEX_SHA256}"
require_report_field "${FINAL_CANDIDATE_REPORT}" \
  development_gate_verifier_sha256 "${DEVELOPMENT_GATE_VERIFIER_SHA256}"
require_report_field "${FINAL_CANDIDATE_REPORT}" final_pilot_method "${FINAL_PILOT_METHOD}"
for hash_field in acceptance_rule_sha256 failure_rule_sha256 analyzer_sha256; do
  require_report_sha256_field "${FINAL_CANDIDATE_REPORT}" "${hash_field}"
done
candidate_n="$(require_report_uint "${FINAL_CANDIDATE_REPORT}" n_cand)"
candidate_row_coverage="$(require_report_uint "${FINAL_CANDIDATE_REPORT}" planned_row_coverage_count)"
candidate_min_eligible_pairs="$(require_report_uint "${FINAL_CANDIDATE_REPORT}" minimum_eligible_pair_count)"
candidate_eligible_pairs="$(require_report_uint "${FINAL_CANDIDATE_REPORT}" eligible_pair_count)"
candidate_attempts="$(require_report_uint "${FINAL_CANDIDATE_REPORT}" attempt_count)"
candidate_postflight_complete_attempts="$(require_report_uint "${FINAL_CANDIDATE_REPORT}" postflight_complete_attempt_count)"
candidate_acquisition_failure_attempts="$(require_report_uint "${FINAL_CANDIDATE_REPORT}" acquisition_failure_attempt_count)"
candidate_method_success_rows="$(require_report_uint "${FINAL_CANDIDATE_REPORT}" method_success_planned_row_count)"
candidate_method_failure_rows="$(require_report_uint "${FINAL_CANDIDATE_REPORT}" method_failure_planned_row_count)"
candidate_unresolved_acquisition_rows="$(require_report_uint "${FINAL_CANDIDATE_REPORT}" unresolved_acquisition_planned_row_count)"
candidate_valid_unpaired_rows="$(require_report_uint "${FINAL_CANDIDATE_REPORT}" valid_unpaired_planned_row_count)"
[[ "${candidate_n}" == "${G2C_N_CAND}" ]]
(( candidate_n > 0 ))
(( candidate_row_coverage == 2 * candidate_n ))
(( candidate_min_eligible_pairs > 0 && candidate_min_eligible_pairs <= candidate_n ))
(( candidate_eligible_pairs >= candidate_min_eligible_pairs && candidate_eligible_pairs <= candidate_n ))
(( candidate_attempts >= 2 * candidate_n ))
(( candidate_postflight_complete_attempts <= candidate_attempts ))
(( candidate_acquisition_failure_attempts <= candidate_attempts ))
(( candidate_method_success_rows <= 2 * candidate_n ))
(( candidate_method_failure_rows <= 2 * candidate_n ))
(( candidate_unresolved_acquisition_rows <= 2 * candidate_n ))
(( candidate_method_success_rows + candidate_method_failure_rows + candidate_unresolved_acquisition_rows == 2 * candidate_n ))
(( candidate_valid_unpaired_rows <= 2 * candidate_n ))

case "${DEV_CONDITION}" in
  Bsmooth)
    export PILOT_METHOD=Bsmooth
    ;;
  Bslosh)
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

G3_ROW_ACTUAL_SHA256="$(sha256sum "${G3_ROW_REPORT}" | awk '{print $1}')"
[[ "${G3_ROW_ACTUAL_SHA256}" == "${G3_ROW_REPORT_SHA256}" ]]
for expected_line in \
  "gate=G3" \
  "release_id=${DEV_RELEASE_ID}" \
  "git_revision=${DEV_RELEASE_GIT_REVISION}" \
  "current_observer_source=odom" \
  "g3_prereg_index_sha256=${G3_PREREG_INDEX_SHA256}" \
  "order_table_sha256=${G3_ORDER_SHA256}" \
  "t_hvis_tail_sec=${T_HVIS_TAIL}" \
  "g3_outcome_window_rule_sha256=${G3_OUTCOME_WINDOW_RULE_SHA256}" \
  "n_dev=${G3_N_DEV}" \
  "development_gate_verifier_sha256=${DEVELOPMENT_GATE_VERIFIER_SHA256}" \
  "source_selection_report_sha256=${SOURCE_SELECTION_REPORT_SHA256}" \
  "final_candidate_report_sha256=${FINAL_CANDIDATE_REPORT_SHA256}" \
  "condition=${DEV_CONDITION}" \
  "pilot_method=${PILOT_METHOD}" \
  "block=${G3_BLOCK}" \
  "order_position=${G3_ORDER_POSITION}" \
  "block_segment_id=${G3_BLOCK_SEGMENT_ID}" \
  "path_sha256=${G3_PATH_SHA256}"; do
  grep -Fxq "${expected_line}" "${G3_ROW_REPORT}"
done

[[ "${DEV_REPEAT}" =~ ^(0[1-9]|[1-9][0-9])$ ]]
[[ "${DEV_MAX_REPEAT}" =~ ^(0[1-9]|[1-9][0-9])$ ]]
(( 10#${DEV_REPEAT} <= 10#${DEV_MAX_REPEAT} ))
attempt_prefix="DEV_G3_RGB_H0b_C1_${DEV_CONDITION}_b${G3_BLOCK}"
export RUN_LABEL="${attempt_prefix}_r${DEV_REPEAT}"
export ATTEMPT_ID="${RUN_LABEL}"
if [[ "${DEV_REPEAT}" == 01 ]]; then
  export ACQUISITION_RETRY=false
  [[ -z "${RETRY_OF_ATTEMPT_ID}" ]]
  [[ -z "${RETRY_REASON_FILE}" ]]
  [[ -z "${RETRY_REASON_FILE_SHA256}" ]]
  [[ -z "${RETRY_FAILURE_EVIDENCE_MANIFEST}" ]]
  [[ -z "${RETRY_FAILURE_EVIDENCE_MANIFEST_SHA256}" ]]
else
  export ACQUISITION_RETRY=true
  [[ -n "${RETRY_OF_ATTEMPT_ID}" ]]
  [[ "${RETRY_OF_ATTEMPT_ID%_r*}" == "${attempt_prefix}" ]]
  previous_repeat="${RETRY_OF_ATTEMPT_ID##*_r}"
  [[ "${previous_repeat}" =~ ^(0[1-9]|[1-9][0-9])$ ]]
  (( 10#${DEV_REPEAT} == 10#${previous_repeat} + 1 ))
  test -s "${RETRY_REASON_FILE}"
  [[ "${RETRY_REASON_FILE_SHA256}" =~ ^[0-9a-f]{64}$ ]]
  [[ "$(sha256sum "${RETRY_REASON_FILE}" | awk '{print $1}')" == "${RETRY_REASON_FILE_SHA256}" ]]
  test -s "${RETRY_FAILURE_EVIDENCE_MANIFEST}"
  [[ "${RETRY_FAILURE_EVIDENCE_MANIFEST_SHA256}" =~ ^[0-9a-f]{64}$ ]]
  [[ "$(sha256sum "${RETRY_FAILURE_EVIDENCE_MANIFEST}" | awk '{print $1}')" == "${RETRY_FAILURE_EVIDENCE_MANIFEST_SHA256}" ]]
  assert_unique_kv_keys "${RETRY_REASON_FILE}"
  assert_unique_kv_keys "${RETRY_FAILURE_EVIDENCE_MANIFEST}"
  for expected_line in \
    "report_type=DEVELOPMENT_RETRY_AUTHORIZATION" \
    "status=PASS" \
    "gate=G3" \
    "release_id=${DEV_RELEASE_ID}" \
    "git_revision=${DEV_RELEASE_GIT_REVISION}" \
    "planned_row_report_sha256=${G3_ROW_REPORT_SHA256}" \
    "failed_attempt_id=${RETRY_OF_ATTEMPT_ID}" \
    "authorized_attempt_id=${ATTEMPT_ID}" \
    "block_segment_id=${G3_BLOCK_SEGMENT_ID}" \
    "failure_class=METHOD_INDEPENDENT_ACQUISITION" \
    "method_failure=false" \
    "retry_authorized=true" \
    "failure_evidence_manifest_path=${RETRY_FAILURE_EVIDENCE_MANIFEST}" \
    "failure_evidence_manifest_sha256=${RETRY_FAILURE_EVIDENCE_MANIFEST_SHA256}" \
    "development_gate_verifier_sha256=${DEVELOPMENT_GATE_VERIFIER_SHA256}"; do
    grep -Fxq "${expected_line}" "${RETRY_REASON_FILE}"
  done
  for expected_line in \
    "report_type=DEVELOPMENT_FAILED_ATTEMPT_EVIDENCE" \
    "status=CLASSIFIED" \
    "gate=G3" \
    "release_id=${DEV_RELEASE_ID}" \
    "git_revision=${DEV_RELEASE_GIT_REVISION}" \
    "planned_row_report_sha256=${G3_ROW_REPORT_SHA256}" \
    "attempt_id=${RETRY_OF_ATTEMPT_ID}" \
    "block_segment_id=${G3_BLOCK_SEGMENT_ID}" \
    "failure_class=METHOD_INDEPENDENT_ACQUISITION" \
    "method_failure=false" \
    "eligible_outcome=false"; do
    grep -Fxq "${expected_line}" "${RETRY_FAILURE_EVIDENCE_MANIFEST}"
  done
  grep -Eq '^raw_artifact_index_sha256=([0-9a-f]{64}|none)$' "${RETRY_FAILURE_EVIDENCE_MANIFEST}"
  startup_log_path="$(sed -n 's/^startup_log_path=//p' "${RETRY_FAILURE_EVIDENCE_MANIFEST}")"
  test -s "${startup_log_path}"
  startup_log_sha256="$(sha256sum "${startup_log_path}" | awk '{print $1}')"
  grep -Fxq "startup_log_sha256=${startup_log_sha256}" "${RETRY_FAILURE_EVIDENCE_MANIFEST}"
fi

test -x "${DEVELOPMENT_GATE_VERIFIER}"
DEVELOPMENT_GATE_VERIFIER_ACTUAL_SHA256="$(sha256sum "${DEVELOPMENT_GATE_VERIFIER}" | awk '{print $1}')"
[[ "${DEVELOPMENT_GATE_VERIFIER_ACTUAL_SHA256}" == "${DEVELOPMENT_GATE_VERIFIER_SHA256}" ]]
export G3_PREREQ_VERIFICATION_REPORT="${G3_ROW_REPORT}.${ATTEMPT_ID}.prereq_verification.txt"
test ! -e "${G3_PREREQ_VERIFICATION_REPORT}"
"${DEVELOPMENT_GATE_VERIFIER}" verify-prerequisites \
  --gate G3 \
  --release-id "${DEV_RELEASE_ID}" \
  --git-revision "${DEV_RELEASE_GIT_REVISION}" \
  --source-selection-report "${SOURCE_SELECTION_REPORT}" \
  --source-selection-sha256 "${SOURCE_SELECTION_REPORT_SHA256}" \
  --final-candidate-report "${FINAL_CANDIDATE_REPORT}" \
  --final-candidate-sha256 "${FINAL_CANDIDATE_REPORT_SHA256}" \
  --upstream-dataset-index "${G2C_DATASET_INDEX}" \
  --upstream-dataset-index-sha256 "${G2C_DATASET_INDEX_SHA256}" \
  --prereg-root "${G3_PREREG_ROOT}" \
  --prereg-index-sha256 "${G3_PREREG_INDEX_SHA256}" \
  --t-hvis-tail "${T_HVIS_TAIL}" \
  --g3-outcome-window-rule-sha256 "${G3_OUTCOME_WINDOW_RULE_SHA256}" \
  --row-report "${G3_ROW_REPORT}" --row-sha256 "${G3_ROW_REPORT_SHA256}" \
  --attempt-id "${ATTEMPT_ID}" --dev-repeat "${DEV_REPEAT}" \
  --max-repeat "${DEV_MAX_REPEAT}" --acquisition-retry "${ACQUISITION_RETRY}" \
  --retry-of-attempt-id "${RETRY_OF_ATTEMPT_ID:-none}" \
  --retry-reason-file "${RETRY_REASON_FILE:-none}" \
  --retry-reason-file-sha256 "${RETRY_REASON_FILE_SHA256:-none}" \
  --failure-evidence-manifest "${RETRY_FAILURE_EVIDENCE_MANIFEST:-none}" \
  --failure-evidence-manifest-sha256 "${RETRY_FAILURE_EVIDENCE_MANIFEST_SHA256:-none}" \
  --path-json "${G3_PATH_JSON}" --path-sha256 "${G3_PATH_SHA256}" \
  --output "${G3_PREREQ_VERIFICATION_REPORT}"
grep -Fxq 'status=PASS' "${G3_PREREQ_VERIFICATION_REPORT}"
assert_unique_kv_keys "${G3_PREREQ_VERIFICATION_REPORT}"
require_report_field "${G3_PREREQ_VERIFICATION_REPORT}" t_hvis_tail_sec "${T_HVIS_TAIL}"
require_report_field "${G3_PREREQ_VERIFICATION_REPORT}" \
  g3_outcome_window_rule_sha256 "${G3_OUTCOME_WINDOW_RULE_SHA256}"
G3_PREREQ_VERIFICATION_SHA256="$(sha256sum "${G3_PREREQ_VERIFICATION_REPORT}" | awk '{print $1}')"

export RUN_OUT_DIR="${SLOSH_BAG_ROOT}/real/${DEV_DATE}_spmpc_development/G3/${DEV_CONDITION}"
export CURRENT_OBSERVER_SOURCE=odom
[[ "${CURRENT_OBSERVER_SOURCE}" == odom ]]

unset MATRIX_PRESET RECORDER_SCRIPT VARIANT ALG W_SLOSH

PILOT_MODE=true \
PILOT_METHOD="${PILOT_METHOD}" \
PILOT_CONDITION="${DEV_CONDITION}" \
SOLVER_BACKEND=continuous_mpcc_acados \
REF_TOPIC=/scout/global_path_fixed \
CMD_TOPIC=/cmd_vel \
COSTMAP_TOPIC=/map \
REFERENCE_TARGET_FRAME=map \
BASE_FRAME=base_link \
PATH_SOURCE_MODE=replay \
PATH_FILE="${G3_PATH_JSON}" \
PATH_EXPECTED_SHA256="${G3_PATH_SHA256}" \
REQUIRE_PATH_HASH=true \
BLOCK_SEGMENT_ID="${G3_BLOCK_SEGMENT_ID}" \
ORDER_POSITION="${G3_ORDER_POSITION}" \
SPLIT_BLOCK=false \
ACQUISITION_RETRY="${ACQUISITION_RETRY}" \
RETRY_REASON_FILE="${RETRY_REASON_FILE}" \
RUN_LABEL="${RUN_LABEL}" \
NAME="${RUN_LABEL}" \
RUN_OUT_DIR="${RUN_OUT_DIR}" \
DATE="${DEV_DATE}" \
PILOT_RECORD_RGB=true \
RECORD_TOPIC_INFO=true \
RECORD_STANDALONE_SLOSH=true \
RECORD_SCAN=true \
RECORD_MOCAP=false \
RECORD_ROSOUT=true \
RECORD_ALL_EXISTING_TOPICS=false \
LIQUID_EXPORT_AFTER_RECORD=false \
ROSBAG_BUFFER_SIZE_MB=4096 \
RECORD_SEC=90 \
MAX_RECORD_SEC=90 \
START_POS_TOL=0.08 \
START_YAW_TOL=0.15 \
START_HOLD_SEC=0.5 \
START_GATE_TIMEOUT_SEC=15 \
PATH_PUBLISH_RATE=2.0 \
PATH_GENERATOR_STARTUP_SEC=2 \
RECORDER_STARTUP_SEC=8 \
PLANNER_STARTUP_SEC=2 \
SEND_ZERO_ON_EXIT=true \
V_REF=0.20 \
SLOSH_HEIGHT_MAX=-1.0 \
ALPHA_MAX=1.2 \
SHARED_LINEAR_ACCEL_LIMIT_ENABLE=true \
SHARED_LINEAR_ACCEL_MAX=0.6 \
SHARED_ANGULAR_LIMIT_ENABLE=true \
SHARED_ANGULAR_RATE_MAX=1.2 \
SHARED_ANGULAR_ACCEL_MAX=1.2 \
DELAY_PHASE_MODE=fixed_closed_loop \
DELAY_PHASE_LINEAR_DELAY_SEC=0.15 \
DELAY_PHASE_ANGULAR_DELAY_SEC=0.22 \
IMU_SHADOW_ENABLE=false \
OPERATOR_NOTE="development G3 RGB efficacy release=${DEV_RELEASE_ID}; revision=${DEV_RELEASE_GIT_REVISION}; attempt_id=${ATTEMPT_ID}; repeat=${DEV_REPEAT}; retry_of=${RETRY_OF_ATTEMPT_ID:-none}; retry_reason_sha256=${RETRY_REASON_FILE_SHA256:-none}; failure_evidence_sha256=${RETRY_FAILURE_EVIDENCE_MANIFEST_SHA256:-none}; source=odom; source_report_sha256=${SOURCE_SELECTION_REPORT_SHA256}; candidate_report_sha256=${FINAL_CANDIDATE_REPORT_SHA256}; prereg_index_sha256=${G3_PREREG_INDEX_SHA256}; t_hvis_tail_sec=${T_HVIS_TAIL}; g3_outcome_window_rule_sha256=${G3_OUTCOME_WINDOW_RULE_SHA256}; row_sha256=${G3_ROW_REPORT_SHA256}; prereq_verification_sha256=${G3_PREREQ_VERIFICATION_SHA256}; condition=${DEV_CONDITION}" \
bash src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_real_fixed_path_trial.sh
```

#### 4.5.1 G2S/G2C/G3 独立 postflight（runner 退出 0 之后仍必做）

当前 recorder 的 `rosbag info`/Python 摘要路径含容错分支，因此 one-click 返回 0 不能替代 bag 验收。对刚完成且已正常闭包的 trial 执行下列门禁；`POSTFLIGHT_CLASS` 必须从不可混淆的 run label 与 one-click/recorder metadata 共同推导，禁止由操作者手填：

```bash
set -euo pipefail
: "${RUN_OUT_DIR:?}" "${RUN_LABEL:?}" \
  "${DEV_RELEASE_ID:?}" "${DEV_RELEASE_GIT_REVISION:?}" \
  "${ATTEMPT_ID:?}" "${DEV_REPEAT:?}" "${DEV_MAX_REPEAT:?}" \
  "${ACQUISITION_RETRY:?}"
export RETRY_OF_ATTEMPT_ID="${RETRY_OF_ATTEMPT_ID:-}"
export RETRY_REASON_FILE="${RETRY_REASON_FILE:-}"
export RETRY_REASON_FILE_SHA256="${RETRY_REASON_FILE_SHA256:-}"
export RETRY_FAILURE_EVIDENCE_MANIFEST="${RETRY_FAILURE_EVIDENCE_MANIFEST:-}"
export RETRY_FAILURE_EVIDENCE_MANIFEST_SHA256="${RETRY_FAILURE_EVIDENCE_MANIFEST_SHA256:-}"

[[ "${RUN_LABEL}" == "${ATTEMPT_ID}" ]]
[[ "${DEV_REPEAT}" =~ ^(0[1-9]|[1-9][0-9])$ ]]
[[ "${DEV_MAX_REPEAT}" =~ ^(0[1-9]|[1-9][0-9])$ ]]
(( 10#${DEV_REPEAT} <= 10#${DEV_MAX_REPEAT} ))
[[ "${RUN_LABEL}" == *_r"${DEV_REPEAT}" ]]

export POSTFLIGHT_BAG="${RUN_OUT_DIR}/${RUN_LABEL}.bag"
export POSTFLIGHT_META="${RUN_OUT_DIR}/${RUN_LABEL}_one_click_meta.env"
export POSTFLIGHT_RUN_INFO="${RUN_OUT_DIR}/${RUN_LABEL}_info.txt"
export POSTFLIGHT_INFO="${RUN_OUT_DIR}/${RUN_LABEL}_postflight_bag_info.txt"
export POSTFLIGHT_COUNTS="${RUN_OUT_DIR}/${RUN_LABEL}_postflight_topic_counts.txt"
export POSTFLIGHT_SHA="${RUN_OUT_DIR}/${RUN_LABEL}_postflight_sha256.txt"

test -s "${POSTFLIGHT_BAG}"
test ! -e "${POSTFLIGHT_BAG}.active"
test -s "${POSTFLIGHT_META}"
test -s "${POSTFLIGHT_RUN_INFO}"
test ! -e "${POSTFLIGHT_INFO}"
test ! -e "${POSTFLIGHT_COUNTS}"
test ! -e "${POSTFLIGHT_SHA}"

read_single_field() {
  local file_path="$1" field_name="$2" field_count
  field_count="$(grep -c "^${field_name}=" "${file_path}" || true)"
  [[ "${field_count}" == 1 ]]
  sed -n "s/^${field_name}=//p" "${file_path}"
}

verify_file_sha256() {
  local file_path="$1" expected_sha256="$2" actual_sha256
  test -s "${file_path}"
  [[ "${expected_sha256}" =~ ^[0-9a-f]{64}$ ]]
  actual_sha256="$(sha256sum "${file_path}" | awk '{print $1}')"
  [[ "${actual_sha256}" == "${expected_sha256}" ]]
}

required_note_fragments=(
  "release=${DEV_RELEASE_ID}"
  "revision=${DEV_RELEASE_GIT_REVISION}"
  "attempt_id=${ATTEMPT_ID}"
  "repeat=${DEV_REPEAT}"
  "retry_of=${RETRY_OF_ATTEMPT_ID:-none}"
  "retry_reason_sha256=${RETRY_REASON_FILE_SHA256:-none}"
  "failure_evidence_sha256=${RETRY_FAILURE_EVIDENCE_MANIFEST_SHA256:-none}"
)
if [[ "${RUN_LABEL}" =~ ^DEV_G2S_H0s_C1_Bsmooth_b[0-9]+_r(0[1-9]|[1-9][0-9])$ ]]; then
  : "${G2S_ROW_REPORT:?}" "${G2S_ROW_REPORT_SHA256:?}" \
    "${G2S_PREREQ_VERIFICATION_REPORT:?}" "${G2S_PREREQ_VERIFICATION_SHA256:?}"
  export POSTFLIGHT_CLASS=G2S
  postflight_row_report="${G2S_ROW_REPORT}"
  postflight_row_sha256="${G2S_ROW_REPORT_SHA256}"
  postflight_prereq_report="${G2S_PREREQ_VERIFICATION_REPORT}"
  postflight_prereq_sha256="${G2S_PREREQ_VERIFICATION_SHA256}"
  expected_run_label="DEV_G2S_H0s_C1_Bsmooth_b${G2S_BLOCK}_r${DEV_REPEAT}"
  expected_row_gate=G2S
  expected_block="${G2S_BLOCK}"
  expected_order_position="${G2S_ORDER_POSITION}"
  expected_block_segment="${G2S_BLOCK_SEGMENT_ID}"
  expected_path_sha256="${G2S_PATH_SHA256}"
  expected_pilot_method=Bsmooth
  expected_pilot_condition=G2S_SourceSelection
  expected_variant=B_smooth
  required_note_fragments+=(
    "row_sha256=${G2S_ROW_REPORT_SHA256}"
    "prereq_verification_sha256=${G2S_PREREQ_VERIFICATION_SHA256}"
  )
elif [[ "${RUN_LABEL}" =~ ^DEV_G2C_H0_C1_(W2|W5)_b[0-9]+_r(0[1-9]|[1-9][0-9])$ ]]; then
  : "${G2C_ROW_REPORT:?}" "${G2C_ROW_REPORT_SHA256:?}" \
    "${G2C_PREREQ_VERIFICATION_REPORT:?}" "${G2C_PREREQ_VERIFICATION_SHA256:?}" \
    "${SOURCE_SELECTION_REPORT:?}" "${SOURCE_SELECTION_REPORT_SHA256:?}"
  export POSTFLIGHT_CLASS=G2C
  postflight_row_report="${G2C_ROW_REPORT}"
  postflight_row_sha256="${G2C_ROW_REPORT_SHA256}"
  postflight_prereq_report="${G2C_PREREQ_VERIFICATION_REPORT}"
  postflight_prereq_sha256="${G2C_PREREQ_VERIFICATION_SHA256}"
  expected_run_label="DEV_G2C_H0_C1_${PILOT_METHOD}_b${G2C_BLOCK}_r${DEV_REPEAT}"
  expected_row_gate=G2C
  expected_block="${G2C_BLOCK}"
  expected_order_position="${G2C_ORDER_POSITION}"
  expected_block_segment="${G2C_BLOCK_SEGMENT_ID}"
  expected_path_sha256="${G2C_PATH_SHA256}"
  case "${PILOT_METHOD}" in W2|W5) ;; *) exit 2 ;; esac
  expected_pilot_method="${PILOT_METHOD}"
  expected_pilot_condition="G2C_${expected_pilot_method}"
  expected_variant=B_slosh
  verify_file_sha256 "${SOURCE_SELECTION_REPORT}" "${SOURCE_SELECTION_REPORT_SHA256}"
  required_note_fragments+=(
    "source_report_sha256=${SOURCE_SELECTION_REPORT_SHA256}"
    "row_sha256=${G2C_ROW_REPORT_SHA256}"
    "prereq_verification_sha256=${G2C_PREREQ_VERIFICATION_SHA256}"
  )
elif [[ "${RUN_LABEL}" =~ ^DEV_G3_RGB_H0b_C1_Bsmooth_b[0-9]+_r(0[1-9]|[1-9][0-9])$ ]]; then
  : "${G3_ROW_REPORT:?}" "${G3_ROW_REPORT_SHA256:?}" \
    "${G3_PREREQ_VERIFICATION_REPORT:?}" "${G3_PREREQ_VERIFICATION_SHA256:?}" \
    "${SOURCE_SELECTION_REPORT:?}" "${SOURCE_SELECTION_REPORT_SHA256:?}" \
    "${FINAL_CANDIDATE_REPORT:?}" "${FINAL_CANDIDATE_REPORT_SHA256:?}" \
    "${G3_PREREG_ROOT:?}" "${T_HVIS_TAIL:?}" "${G3_OUTCOME_WINDOW_RULE_SHA256:?}"
  export POSTFLIGHT_CLASS=G3_Bsmooth
  postflight_row_report="${G3_ROW_REPORT}"
  postflight_row_sha256="${G3_ROW_REPORT_SHA256}"
  postflight_prereq_report="${G3_PREREQ_VERIFICATION_REPORT}"
  postflight_prereq_sha256="${G3_PREREQ_VERIFICATION_SHA256}"
  expected_run_label="DEV_G3_RGB_H0b_C1_Bsmooth_b${G3_BLOCK}_r${DEV_REPEAT}"
  expected_row_gate=G3
  expected_block="${G3_BLOCK}"
  expected_order_position="${G3_ORDER_POSITION}"
  expected_block_segment="${G3_BLOCK_SEGMENT_ID}"
  expected_path_sha256="${G3_PATH_SHA256}"
  expected_pilot_method=Bsmooth
  expected_pilot_condition=Bsmooth
  expected_variant=B_smooth
  verify_file_sha256 "${SOURCE_SELECTION_REPORT}" "${SOURCE_SELECTION_REPORT_SHA256}"
  verify_file_sha256 "${FINAL_CANDIDATE_REPORT}" "${FINAL_CANDIDATE_REPORT_SHA256}"
  g3_outcome_window_rule="${G3_PREREG_ROOT}/rgb_sync_and_metric.yaml"
  verify_file_sha256 "${g3_outcome_window_rule}" "${G3_OUTCOME_WINDOW_RULE_SHA256}"
  frozen_final_method="$(read_single_field "${FINAL_CANDIDATE_REPORT}" final_pilot_method)"
  case "${frozen_final_method}" in W2|W5) ;; *) exit 2 ;; esac
  required_note_fragments+=(
    "source_report_sha256=${SOURCE_SELECTION_REPORT_SHA256}"
    "candidate_report_sha256=${FINAL_CANDIDATE_REPORT_SHA256}"
    "t_hvis_tail_sec=${T_HVIS_TAIL}"
    "g3_outcome_window_rule_sha256=${G3_OUTCOME_WINDOW_RULE_SHA256}"
    "row_sha256=${G3_ROW_REPORT_SHA256}"
    "prereq_verification_sha256=${G3_PREREQ_VERIFICATION_SHA256}"
  )
elif [[ "${RUN_LABEL}" =~ ^DEV_G3_RGB_H0b_C1_Bslosh_b[0-9]+_r(0[1-9]|[1-9][0-9])$ ]]; then
  : "${G3_ROW_REPORT:?}" "${G3_ROW_REPORT_SHA256:?}" \
    "${G3_PREREQ_VERIFICATION_REPORT:?}" "${G3_PREREQ_VERIFICATION_SHA256:?}" \
    "${SOURCE_SELECTION_REPORT:?}" "${SOURCE_SELECTION_REPORT_SHA256:?}" \
    "${FINAL_CANDIDATE_REPORT:?}" "${FINAL_CANDIDATE_REPORT_SHA256:?}" \
    "${G3_PREREG_ROOT:?}" "${T_HVIS_TAIL:?}" "${G3_OUTCOME_WINDOW_RULE_SHA256:?}"
  export POSTFLIGHT_CLASS=G3_Bslosh
  postflight_row_report="${G3_ROW_REPORT}"
  postflight_row_sha256="${G3_ROW_REPORT_SHA256}"
  postflight_prereq_report="${G3_PREREQ_VERIFICATION_REPORT}"
  postflight_prereq_sha256="${G3_PREREQ_VERIFICATION_SHA256}"
  expected_run_label="DEV_G3_RGB_H0b_C1_Bslosh_b${G3_BLOCK}_r${DEV_REPEAT}"
  expected_row_gate=G3
  expected_block="${G3_BLOCK}"
  expected_order_position="${G3_ORDER_POSITION}"
  expected_block_segment="${G3_BLOCK_SEGMENT_ID}"
  expected_path_sha256="${G3_PATH_SHA256}"
  verify_file_sha256 "${SOURCE_SELECTION_REPORT}" "${SOURCE_SELECTION_REPORT_SHA256}"
  verify_file_sha256 "${FINAL_CANDIDATE_REPORT}" "${FINAL_CANDIDATE_REPORT_SHA256}"
  g3_outcome_window_rule="${G3_PREREG_ROOT}/rgb_sync_and_metric.yaml"
  verify_file_sha256 "${g3_outcome_window_rule}" "${G3_OUTCOME_WINDOW_RULE_SHA256}"
  expected_pilot_method="$(read_single_field "${FINAL_CANDIDATE_REPORT}" final_pilot_method)"
  case "${expected_pilot_method}" in W2|W5) ;; *) exit 2 ;; esac
  expected_pilot_condition=Bslosh
  expected_variant=B_slosh
  required_note_fragments+=(
    "source_report_sha256=${SOURCE_SELECTION_REPORT_SHA256}"
    "candidate_report_sha256=${FINAL_CANDIDATE_REPORT_SHA256}"
    "t_hvis_tail_sec=${T_HVIS_TAIL}"
    "g3_outcome_window_rule_sha256=${G3_OUTCOME_WINDOW_RULE_SHA256}"
    "row_sha256=${G3_ROW_REPORT_SHA256}"
    "prereq_verification_sha256=${G3_PREREQ_VERIFICATION_SHA256}"
  )
else
  echo "NO-GO: run label cannot determine a unique postflight class" >&2
  exit 2
fi

[[ "${RUN_LABEL}" == "${expected_run_label}" ]]
verify_file_sha256 "${postflight_row_report}" "${postflight_row_sha256}"
verify_file_sha256 "${postflight_prereq_report}" "${postflight_prereq_sha256}"
[[ "$(read_single_field "${postflight_prereq_report}" status)" == PASS ]]
[[ "$(read_single_field "${postflight_prereq_report}" attempt_id)" == "${ATTEMPT_ID}" ]]
[[ "$(read_single_field "${postflight_prereq_report}" dev_repeat)" == "${DEV_REPEAT}" ]]
[[ "$(read_single_field "${postflight_prereq_report}" max_repeat)" == "${DEV_MAX_REPEAT}" ]]
[[ "$(read_single_field "${postflight_prereq_report}" acquisition_retry)" == "${ACQUISITION_RETRY}" ]]
[[ "$(read_single_field "${postflight_prereq_report}" retry_of_attempt_id)" == "${RETRY_OF_ATTEMPT_ID:-none}" ]]
[[ "$(read_single_field "${postflight_prereq_report}" retry_reason_file_sha256)" == "${RETRY_REASON_FILE_SHA256:-none}" ]]
[[ "$(read_single_field "${postflight_prereq_report}" failure_evidence_manifest_sha256)" == "${RETRY_FAILURE_EVIDENCE_MANIFEST_SHA256:-none}" ]]
[[ "$(read_single_field "${postflight_row_report}" gate)" == "${expected_row_gate}" ]]
[[ "$(read_single_field "${postflight_row_report}" release_id)" == "${DEV_RELEASE_ID}" ]]
[[ "$(read_single_field "${postflight_row_report}" git_revision)" == "${DEV_RELEASE_GIT_REVISION}" ]]
[[ "$(read_single_field "${postflight_row_report}" block)" == "${expected_block}" ]]
[[ "$(read_single_field "${postflight_row_report}" order_position)" == "${expected_order_position}" ]]
[[ "$(read_single_field "${postflight_row_report}" block_segment_id)" == "${expected_block_segment}" ]]
[[ "$(read_single_field "${postflight_row_report}" path_sha256)" == "${expected_path_sha256}" ]]
[[ "$(read_single_field "${postflight_row_report}" pilot_method)" == "${expected_pilot_method}" ]]

if [[ "${DEV_REPEAT}" == 01 ]]; then
  [[ "${ACQUISITION_RETRY}" == false ]]
  [[ -z "${RETRY_OF_ATTEMPT_ID}" ]]
  [[ -z "${RETRY_REASON_FILE}" ]]
  [[ -z "${RETRY_REASON_FILE_SHA256}" ]]
  [[ -z "${RETRY_FAILURE_EVIDENCE_MANIFEST}" ]]
  [[ -z "${RETRY_FAILURE_EVIDENCE_MANIFEST_SHA256}" ]]
else
  [[ "${ACQUISITION_RETRY}" == true ]]
  [[ "${RETRY_OF_ATTEMPT_ID%_r*}" == "${RUN_LABEL%_r*}" ]]
  previous_repeat="${RETRY_OF_ATTEMPT_ID##*_r}"
  [[ "${previous_repeat}" =~ ^(0[1-9]|[1-9][0-9])$ ]]
  (( 10#${DEV_REPEAT} == 10#${previous_repeat} + 1 ))
  verify_file_sha256 "${RETRY_REASON_FILE}" "${RETRY_REASON_FILE_SHA256}"
  verify_file_sha256 "${RETRY_FAILURE_EVIDENCE_MANIFEST}" "${RETRY_FAILURE_EVIDENCE_MANIFEST_SHA256}"
  [[ "$(read_single_field "${RETRY_REASON_FILE}" report_type)" == DEVELOPMENT_RETRY_AUTHORIZATION ]]
  [[ "$(read_single_field "${RETRY_REASON_FILE}" status)" == PASS ]]
  [[ "$(read_single_field "${RETRY_REASON_FILE}" planned_row_report_sha256)" == "${postflight_row_sha256}" ]]
  [[ "$(read_single_field "${RETRY_REASON_FILE}" failed_attempt_id)" == "${RETRY_OF_ATTEMPT_ID}" ]]
  [[ "$(read_single_field "${RETRY_REASON_FILE}" authorized_attempt_id)" == "${ATTEMPT_ID}" ]]
  [[ "$(read_single_field "${RETRY_REASON_FILE}" block_segment_id)" == "${expected_block_segment}" ]]
  [[ "$(read_single_field "${RETRY_REASON_FILE}" failure_class)" == METHOD_INDEPENDENT_ACQUISITION ]]
  [[ "$(read_single_field "${RETRY_REASON_FILE}" method_failure)" == false ]]
  [[ "$(read_single_field "${RETRY_REASON_FILE}" retry_authorized)" == true ]]
  [[ "$(read_single_field "${RETRY_REASON_FILE}" failure_evidence_manifest_path)" == "${RETRY_FAILURE_EVIDENCE_MANIFEST}" ]]
  [[ "$(read_single_field "${RETRY_REASON_FILE}" failure_evidence_manifest_sha256)" == "${RETRY_FAILURE_EVIDENCE_MANIFEST_SHA256}" ]]
  [[ "$(read_single_field "${RETRY_FAILURE_EVIDENCE_MANIFEST}" report_type)" == DEVELOPMENT_FAILED_ATTEMPT_EVIDENCE ]]
  [[ "$(read_single_field "${RETRY_FAILURE_EVIDENCE_MANIFEST}" status)" == CLASSIFIED ]]
  [[ "$(read_single_field "${RETRY_FAILURE_EVIDENCE_MANIFEST}" attempt_id)" == "${RETRY_OF_ATTEMPT_ID}" ]]
  [[ "$(read_single_field "${RETRY_FAILURE_EVIDENCE_MANIFEST}" block_segment_id)" == "${expected_block_segment}" ]]
  [[ "$(read_single_field "${RETRY_FAILURE_EVIDENCE_MANIFEST}" failure_class)" == METHOD_INDEPENDENT_ACQUISITION ]]
  [[ "$(read_single_field "${RETRY_FAILURE_EVIDENCE_MANIFEST}" method_failure)" == false ]]
  [[ "$(read_single_field "${RETRY_FAILURE_EVIDENCE_MANIFEST}" eligible_outcome)" == false ]]
  retry_startup_log_path="$(read_single_field "${RETRY_FAILURE_EVIDENCE_MANIFEST}" startup_log_path)"
  retry_startup_log_sha256="$(read_single_field "${RETRY_FAILURE_EVIDENCE_MANIFEST}" startup_log_sha256)"
  verify_file_sha256 "${retry_startup_log_path}" "${retry_startup_log_sha256}"
fi

case "${POSTFLIGHT_CLASS}" in
  G2S)
    [[ "$(read_single_field "${postflight_row_report}" condition)" == Bsmooth ]]
    [[ "$(read_single_field "${postflight_row_report}" g2s_prereg_index_sha256)" == "${G2S_PREREG_INDEX_SHA256}" ]]
    [[ "$(read_single_field "${postflight_row_report}" g2a_audit_report_sha256)" == "${G2A_AUDIT_REPORT_SHA256}" ]]
    [[ "$(read_single_field "${postflight_row_report}" imu_cal_validation_report_sha256)" == "${IMU_CAL_VALIDATION_REPORT_SHA256}" ]]
    ;;
  G2C)
    [[ "$(read_single_field "${postflight_row_report}" g2c_prereg_index_sha256)" == "${G2C_PREREG_INDEX_SHA256}" ]]
    [[ "$(read_single_field "${postflight_row_report}" source_selection_report_sha256)" == "${SOURCE_SELECTION_REPORT_SHA256}" ]]
    ;;
  G3_Bsmooth|G3_Bslosh)
    [[ "$(read_single_field "${postflight_row_report}" condition)" == "${expected_pilot_condition}" ]]
    [[ "$(read_single_field "${postflight_row_report}" g3_prereg_index_sha256)" == "${G3_PREREG_INDEX_SHA256}" ]]
    [[ "$(read_single_field "${postflight_row_report}" t_hvis_tail_sec)" == "${T_HVIS_TAIL}" ]]
    [[ "$(read_single_field "${postflight_row_report}" g3_outcome_window_rule_sha256)" == "${G3_OUTCOME_WINDOW_RULE_SHA256}" ]]
    [[ "$(read_single_field "${postflight_prereq_report}" t_hvis_tail_sec)" == "${T_HVIS_TAIL}" ]]
    [[ "$(read_single_field "${postflight_prereq_report}" g3_outcome_window_rule_sha256)" == "${G3_OUTCOME_WINDOW_RULE_SHA256}" ]]
    [[ "$(read_single_field "${postflight_row_report}" source_selection_report_sha256)" == "${SOURCE_SELECTION_REPORT_SHA256}" ]]
    [[ "$(read_single_field "${postflight_row_report}" final_candidate_report_sha256)" == "${FINAL_CANDIDATE_REPORT_SHA256}" ]]
    ;;
esac

for identity_file in "${POSTFLIGHT_META}" "${POSTFLIGHT_RUN_INFO}"; do
  [[ "$(read_single_field "${identity_file}" run_label)" == "${RUN_LABEL}" ]]
  [[ "$(read_single_field "${identity_file}" run_class)" == pilot ]]
  [[ "$(read_single_field "${identity_file}" pilot_mode)" == true ]]
  [[ "$(read_single_field "${identity_file}" pilot_method)" == "${expected_pilot_method}" ]]
  [[ "$(read_single_field "${identity_file}" pilot_condition)" == "${expected_pilot_condition}" ]]
  [[ "$(read_single_field "${identity_file}" variant)" == "${expected_variant}" ]]
  [[ "$(read_single_field "${identity_file}" block_segment_id)" == "${expected_block_segment}" ]]
  [[ "$(read_single_field "${identity_file}" order_position)" == "${expected_order_position}" ]]
  [[ "$(read_single_field "${identity_file}" split_block)" == false ]]
  [[ "$(read_single_field "${identity_file}" acquisition_retry)" == "${ACQUISITION_RETRY}" ]]
  [[ "$(read_single_field "${identity_file}" path_expected_sha256)" == "${expected_path_sha256}" ]]
  [[ "$(read_single_field "${identity_file}" path_actual_sha256)" == "${expected_path_sha256}" ]]
done
[[ "$(read_single_field "${POSTFLIGHT_META}" name)" == "${RUN_LABEL}" ]]
[[ "$(read_single_field "${POSTFLIGHT_RUN_INFO}" git_commit)" == "${DEV_RELEASE_GIT_REVISION}" ]]
[[ "$(read_single_field "${POSTFLIGHT_RUN_INFO}" acquisition_retry)" == "${ACQUISITION_RETRY}" ]]
[[ "$(read_single_field "${POSTFLIGHT_RUN_INFO}" retry_reason_file)" == "${RETRY_REASON_FILE}" ]]
[[ "$(read_single_field "${POSTFLIGHT_RUN_INFO}" record_mocap)" == false ]]
[[ "$(read_single_field "${POSTFLIGHT_RUN_INFO}" record_rosout)" == true ]]
[[ "$(read_single_field "${POSTFLIGHT_RUN_INFO}" liquid_export_after_record)" == false ]]
operator_note="$(read_single_field "${POSTFLIGHT_RUN_INFO}" operator_note)"
for required_fragment in "${required_note_fragments[@]}"; do
  [[ "${operator_note}" == *"${required_fragment}"* ]]
done

postflight_evidence_files=("${postflight_row_report}" "${postflight_prereq_report}")
if [[ "${ACQUISITION_RETRY}" == true ]]; then
  postflight_evidence_files+=(
    "${RETRY_REASON_FILE}"
    "${RETRY_FAILURE_EVIDENCE_MANIFEST}"
    "${retry_startup_log_path}"
  )
fi
case "${POSTFLIGHT_CLASS}" in
  G2C)
    postflight_evidence_files+=("${SOURCE_SELECTION_REPORT}")
    ;;
  G3_Bsmooth|G3_Bslosh)
    postflight_evidence_files+=(
      "${SOURCE_SELECTION_REPORT}"
      "${FINAL_CANDIDATE_REPORT}"
      "${g3_outcome_window_rule}"
    )
    ;;
esac

required_topics=(
  /cmd_vel
  /odom
  /imu/data
  /tf
  /tf_static
  /scout/global_path_fixed
  /spmpc/status
  /spmpc/debug/effective_config
  /spmpc/debug/solver_input_state
)
case "${POSTFLIGHT_CLASS}" in
  G2S)
    required_topics+=(
      /spmpc/debug/slosh_observer_odom
      /spmpc/debug/slosh_observer_imu
      /camera/color/image_raw
      /camera/color/camera_info
    )
    ;;
  G2C)
    required_topics+=(/spmpc/slosh_height /spmpc/debug/slosh_state)
    ;;
  G3_Bsmooth)
    required_topics+=(/camera/color/image_raw /camera/color/camera_info)
    ;;
  G3_Bslosh)
    required_topics+=(
      /camera/color/image_raw
      /camera/color/camera_info
      /spmpc/slosh_height
      /spmpc/debug/slosh_state
    )
    ;;
  *) exit 2 ;;
esac

rosbag info "${POSTFLIGHT_BAG}" > "${POSTFLIGHT_INFO}"
python3 - "${POSTFLIGHT_BAG}" "${required_topics[@]}" \
  > "${POSTFLIGHT_COUNTS}" <<'PY'
import sys
import rosbag

bag_path, *required = sys.argv[1:]
counts = {topic: 0 for topic in required}
with rosbag.Bag(bag_path, "r") as bag:
    for topic, _, _ in bag.read_messages(topics=required):
        counts[topic] += 1
for topic in required:
    print(f"{topic}={counts[topic]}")
missing = [topic for topic, count in counts.items() if count <= 0]
if missing:
    raise SystemExit("missing/empty required topics: " + ",".join(missing))
PY
sha256sum \
  "${POSTFLIGHT_BAG}" \
  "${POSTFLIGHT_META}" \
  "${POSTFLIGHT_RUN_INFO}" \
  "${postflight_evidence_files[@]}" \
  > "${POSTFLIGHT_SHA}"
```

此外必须从 bag 验证有效运动段、完整路径、预运动 RGB（适用时）和冻结 post-motion tail；仅有 topic 各一条消息仍不能通过。G3 postflight 必须由冻结 RGB/QC 工具生成不可变 window-QC report/hash，至少记录 `first_effective_motion/first_arrival_or_timeout/window_end/observed_tail_sec/tail_complete/continuous_hvis_eligible/t_hvis_tail_sec/g3_outcome_window_rule_sha256`；该 report/hash 必须进入 G3 dataset index，缺失或 mixed tail/rule 时不得生成 G3 PASS。固定写 `RECORD_SEC=90` 不能证明窗口覆盖充分，首条 G3 前的 wrapper 还必须验证 recorder/backend startup、admission、冻结 motion 上限与 tail 的总预算。任何 postflight 失败都保留原 bag 和原因，不得修改 bag 后重算为 PASS。

执行前必须已经遥控回起点、完成相同的 `T_SETTLE` 和安全检查。完整 block 全部完成前不得提前停止，也不得看一个 block 后更换 candidate。显式 `IMU_SHADOW_ENABLE=false` 用于阻断 stale shell export；旧 `MATRIX_PRESET` 也不得代替本命令。与 G2C 相同，当前 one-click runner 尚不生成 monitor-reset/`T_SETTLE`/release-hash 证据；在冻结 wrapper、通过第 4.5.1 节 postflight 前，该示例只能验证在线执行器与 RGB 录制，不能单独签署 G3 efficacy gate。

G3 只能在 G2S 已冻结唯一输入语义、且 G2C 已冻结最终 Bslosh candidate 后开始；G2S 与 G3 的 RGB 不得复用为同一验证目的。

全部 G3 planned rows 和额外 attempts 结束后，必须先生成不可变 `G3_DATASET_INDEX`，逐行绑定 G3 prereg/row、每个 `ATTEMPT_ID` 与 retry chain、prerequisite verification、postflight、bag、RGB/window QC、failure/exclusion 及其 hash；planned row 缺失、attempt 未分类、mixed tail/rule 或提前停止时不得签 PASS。随后由冻结 analyzer/verifier 生成唯一 `G3_EFFICACY` report/hash，至少含 `report_type/status/release_id/git_revision`、source-selection/final-candidate/G3-prereg/development-verifier/dataset-index hashes、`t_hvis_tail_sec/g3_outcome_window_rule_sha256`、`n_dev/planned_row_coverage_count/minimum_eligible_pair_count/eligible_pair_count/attempt_count/postflight_complete_attempt_count/postflight_failure_attempt_count/acquisition_failure_attempt_count/readiness_failure_attempt_count/method_success_planned_row_count/method_failure_planned_row_count/unresolved_acquisition_planned_row_count/valid_unpaired_planned_row_count`、primary=`H_vis,p95(motion+tail)`、effect/interval 与 `delta_H_dev`、success/tracking/runtime、single-block-dominance、no-early-stop 和 analysis/gate-rule hashes。attempt 分类与 planned-row partition 均由 dataset index 独立重算，不得用 `attempts-2*pairs` 反推。G3 PASS 只是进入 G4/G5 并最终接受 G6 的必要非充分条件；G6 对 G3 的唯一有效输入是该 immutable report/hash，并且还必须绑定 G4/G5 PASS reports。G6 verifier 必须实际打开 G3 report，证明其 `t_hvis_tail_sec/g3_outcome_window_rule_sha256` 与 G6 measurement/analysis freeze、formal manifest 和每个 `RUN_ENV` 完全相同；不允许在 G6 重选窗口。

### 4.6 G4：trajectory、四相位与 replay

当前没有可签署 G4 的唯一命令。future toolchain 必须冻结 longitudinal/lateral checkpoint、相位幅值、数值容差与完整 horizon/first-action 输出；四相位差异必须超过冻结容差。online-input/zero-state replay 只适用于 `Bslosh`，并须从同一不可变 pre-solve snapshot 克隆，online-input branch 复现在线 status、first action 和 raw command。当前 extractor 与 replay 工具均未达到该合同，因此 G4 为 `NO-GO`。

### 4.7 SmoothMatch development

SmoothMatch 只允许改变 `B_smooth` 的一个 `v_ref`，并只用冻结的 odometry-derived completion-time 规则选值。候选运行仍映射：

```text
condition_id=SmoothMatchCandidate
planner_variant=B_smooth
w_slosh=0
v_ref=SMOOTH_MATCH_V_REF_CANDIDATE
```

最终 `SMOOTH_MATCH_V_REF` 必须由 future v2.0 manifest 导出，不能在 formal 终端 `read -p` 临时填写。

### 4.8 FixedProfile development

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
export PLANNED_BLOCK_SEGMENT_ID=S1_b01_seg01
```

上面只展示静态 randomized row/run-slot 身份。`ACTUAL_BLOCK_SEGMENT_ID/SPLIT_BLOCK/ACQUISITION_RETRY/RETRY_*` 与 stage-entry fields 不得手工预设；它们必须由第 5.4 节已验证的动态 evidence env 完整赋值。首次 `r01` 也必须显式得到 `SPLIT_BLOCK=false`、`ACQUISITION_RETRY=false` 和空 retry fields，不能依赖 shell 里“碰巧没有旧变量”。

合法组合：

| `STAGE/GROUP` | 路径/容器 | 合法 `CONDITION_ID` | position |
| --- | --- | --- | --- |
| `S1/CORE` | H1/C1 | B0、Bsmooth、SmoothMatch、FixedProfile、Bslosh | 01–05 |
| `S2A/SELECTIVITY` | L1/C1 | Bsmooth、FixedProfile、Bslosh | 01–03 |
| `S2B/TRANSFER` | H1/C2 | Bsmooth、FixedProfile、Bslosh | 01–03 |

```bash
set -euo pipefail
: "${PROTOCOL_ID:?}" "${FREEZE_ID:?}" \
  "${FAILURE_INCLUSIVE_ESTIMAND:?}" "${FAILURE_INCLUSIVE_ESTIMAND_SHA256:?}" \
  "${DENOMINATOR_SCHEMA:?}" "${DENOMINATOR_SCHEMA_SHA256:?}" \
  "${CONTRAST_REGISTRY:?}" "${CONTRAST_REGISTRY_SHA256:?}" \
  "${CONTRAST_DENOMINATOR_VERIFIER:?}" \
  "${CONTRAST_DENOMINATOR_VERIFIER_SHA256:?}" \
  "${G3_OUTCOME_WINDOW_RULE_SHA256:?}"

report_field() {
  local report_path="$1" field_name="$2" field_count
  field_count="$(grep -c "^${field_name}=" "${report_path}" || true)"
  [[ "${field_count}" == 1 ]]
  sed -n "s/^${field_name}=//p" "${report_path}"
}

require_report_field() {
  local report_path="$1" field_name="$2" expected_value="$3"
  [[ "$(report_field "${report_path}" "${field_name}")" == "${expected_value}" ]]
}

require_planned_outcome_partition() {
  local report_path="$1" expected_plan="$2" method_success method_failure unresolved_acquisition
  method_success="$(report_field "${report_path}" method_success_planned_row_count)"
  method_failure="$(report_field "${report_path}" method_failure_planned_row_count)"
  unresolved_acquisition="$(report_field "${report_path}" unresolved_acquisition_planned_row_count)"
  [[ "${method_success}" =~ ^[0-9]+$ ]]
  [[ "${method_failure}" =~ ^[0-9]+$ ]]
  [[ "${unresolved_acquisition}" =~ ^[0-9]+$ ]]
  (( method_success + method_failure + unresolved_acquisition == expected_plan ))
}

require_attempt_ledger_counts() {
  local report_path="$1" minimum_attempts="$2" attempt_count field_name field_value
  attempt_count="$(report_field "${report_path}" attempt_count)"
  [[ "${attempt_count}" =~ ^[0-9]+$ ]]
  (( attempt_count >= minimum_attempts ))
  for field_name in \
    acquisition_failure_attempt_count readiness_failure_attempt_count \
    postflight_complete_attempt_count postflight_failure_attempt_count; do
    field_value="$(report_field "${report_path}" "${field_name}")"
    [[ "${field_value}" =~ ^[0-9]+$ ]]
    (( field_value <= attempt_count ))
  done
}

verify_frozen_report_hash() {
  local report_path="$1" expected_sha256="$2" actual_sha256
  test -s "${report_path}"
  [[ "${expected_sha256}" =~ ^[0-9a-f]{64}$ ]]
  actual_sha256="$(sha256sum "${report_path}" | awk '{print $1}')"
  [[ "${actual_sha256}" == "${expected_sha256}" ]]
}

verify_common_stage_report() {
  local report_path="$1" expected_sha256="$2" report_type="$3" report_stage="$4"
  verify_frozen_report_hash "${report_path}" "${expected_sha256}"
  require_report_field "${report_path}" protocol_id "${PROTOCOL_ID}"
  require_report_field "${report_path}" freeze_id "${FREEZE_ID}"
  require_report_field "${report_path}" report_type "${report_type}"
  require_report_field "${report_path}" stage "${report_stage}"
  require_report_field "${report_path}" status PASS
}

verify_contrast_denominator_index() {
  local index_path="$1" expected_sha256="$2" expected_stage="$3" expected_path="$4"
  local expected_n_block_plan="$5" require_minimum="$6" registry_path="$7"
  verify_frozen_report_hash "${index_path}" "${expected_sha256}"
  python3 - "${index_path}" "${expected_stage}" "${expected_path}" \
    "${expected_n_block_plan}" "${require_minimum}" "${registry_path}" <<'PY'
import csv
import sys

(
    index_path,
    expected_stage,
    expected_path,
    expected_n_block_plan_text,
    require_minimum_text,
    registry_path,
) = sys.argv[1:]
if not expected_n_block_plan_text.isdigit() or int(expected_n_block_plan_text) <= 0:
    raise SystemExit("invalid expected_n_block_plan")
expected_n_block_plan = int(expected_n_block_plan_text)
if require_minimum_text not in {"true", "false"}:
    raise SystemExit("invalid require_minimum flag")
require_minimum = require_minimum_text == "true"
required = {
    "stage",
    "contrast_id",
    "path_id",
    "n_block_plan",
    "minimum_n_pair",
    "n_pair",
    "valid_unpaired_planned_row_count",
    "method_failure_planned_row_count",
    "unresolved_acquisition_planned_row_count",
}
integer_fields = required - {"stage", "contrast_id", "path_id"}
registry_required = {
    "stage",
    "contrast_id",
    "path_id",
    "n_block_plan",
    "minimum_n_pair",
}
with open(registry_path, newline="", encoding="utf-8") as stream:
    registry_reader = csv.DictReader(stream)
    if not registry_reader.fieldnames or not registry_required.issubset(registry_reader.fieldnames):
        raise SystemExit("invalid frozen contrast-registry header")
    if len(registry_reader.fieldnames) != len(set(registry_reader.fieldnames)):
        raise SystemExit("duplicate frozen contrast-registry header")
    registry_rows = list(registry_reader)

expected = {}
for line_number, row in enumerate(registry_rows, start=2):
    if None in row:
        raise SystemExit(f"extra contrast-registry cells at line {line_number}")
    if any(row.get(field, "").strip() == "" for field in registry_required):
        raise SystemExit(f"missing contrast-registry field at line {line_number}")
    if row["stage"] != expected_stage or row["path_id"] != expected_path:
        continue
    if row["contrast_id"] != row["contrast_id"].strip():
        raise SystemExit(f"non-canonical registry contrast_id at line {line_number}")
    key = (row["stage"], row["contrast_id"], row["path_id"])
    if key in expected:
        raise SystemExit(f"duplicate frozen contrast-registry row: {key}")
    if not row["n_block_plan"].isdigit() or not row["minimum_n_pair"].isdigit():
        raise SystemExit(f"non-uint frozen denominator at line {line_number}")
    n_block_plan = int(row["n_block_plan"])
    minimum_n_pair = int(row["minimum_n_pair"])
    if n_block_plan <= 0 or not 0 < minimum_n_pair <= n_block_plan:
        raise SystemExit(f"invalid frozen denominator at line {line_number}")
    if n_block_plan != expected_n_block_plan:
        raise SystemExit(f"registry n_block_plan disagrees with protocol at line {line_number}")
    expected[key] = (n_block_plan, minimum_n_pair)
if not expected:
    raise SystemExit("frozen contrast registry has no rows for the expected stage/path")

with open(index_path, newline="", encoding="utf-8") as stream:
    reader = csv.DictReader(stream)
    if not reader.fieldnames or not required.issubset(reader.fieldnames):
        raise SystemExit("invalid contrast-denominator index header")
    if len(reader.fieldnames) != len(set(reader.fieldnames)):
        raise SystemExit("duplicate contrast-denominator index header")
    rows = list(reader)
if not rows:
    raise SystemExit("empty contrast-denominator index")

seen = set()
for line_number, row in enumerate(rows, start=2):
    if None in row:
        raise SystemExit(f"extra CSV cells at line {line_number}")
    if any(row.get(field, "").strip() == "" for field in required):
        raise SystemExit(f"missing denominator field at CSV line {line_number}")
    if row["contrast_id"] != row["contrast_id"].strip():
        raise SystemExit(f"non-canonical contrast_id at CSV line {line_number}")
    if row["stage"] != expected_stage or row["path_id"] != expected_path:
        raise SystemExit(f"wrong stage/path at CSV line {line_number}")
    key = (row["stage"], row["contrast_id"], row["path_id"])
    if key in seen:
        raise SystemExit(f"duplicate contrast denominator row: {key}")
    seen.add(key)
    if key not in expected:
        raise SystemExit(f"contrast is absent from frozen registry: {key}")
    if any(not row[field].isdigit() for field in integer_fields):
        raise SystemExit(f"non-uint denominator at CSV line {line_number}")
    values = {field: int(row[field]) for field in integer_fields}
    n_block_plan = values["n_block_plan"]
    if (n_block_plan, values["minimum_n_pair"]) != expected[key]:
        raise SystemExit(f"denominator plan differs from frozen registry at CSV line {line_number}")
    if values["n_pair"] > n_block_plan:
        raise SystemExit(f"n_pair exceeds n_block_plan at CSV line {line_number}")
    if require_minimum and values["n_pair"] < values["minimum_n_pair"]:
        raise SystemExit(f"PASS gate does not meet minimum_n_pair at CSV line {line_number}")
    for field in (
        "valid_unpaired_planned_row_count",
        "method_failure_planned_row_count",
        "unresolved_acquisition_planned_row_count",
    ):
        if values[field] > 2 * n_block_plan:
            raise SystemExit(f"{field} exceeds planned rows at CSV line {line_number}")
    method_success = (
        2 * n_block_plan
        - values["method_failure_planned_row_count"]
        - values["unresolved_acquisition_planned_row_count"]
    )
    if method_success < 0:
        raise SystemExit(f"planned-row partition is negative at CSV line {line_number}")
    if 2 * values["n_pair"] + values["valid_unpaired_planned_row_count"] > method_success:
        raise SystemExit(f"continuous eligible rows exceed method-success rows at CSV line {line_number}")
if seen != set(expected):
    missing = sorted(set(expected) - seen)
    raise SystemExit(f"contrast-denominator index omits frozen registry rows: {missing}")
PY
}

verify_frozen_report_hash \
  "${FAILURE_INCLUSIVE_ESTIMAND}" "${FAILURE_INCLUSIVE_ESTIMAND_SHA256}"
verify_frozen_report_hash "${DENOMINATOR_SCHEMA}" "${DENOMINATOR_SCHEMA_SHA256}"
verify_frozen_report_hash "${CONTRAST_REGISTRY}" "${CONTRAST_REGISTRY_SHA256}"
test -x "${CONTRAST_DENOMINATOR_VERIFIER}"
verify_frozen_report_hash \
  "${CONTRAST_DENOMINATOR_VERIFIER}" "${CONTRAST_DENOMINATOR_VERIFIER_SHA256}"
require_report_field "${FAILURE_INCLUSIVE_ESTIMAND}" \
  g3_outcome_window_rule_sha256 "${G3_OUTCOME_WINDOW_RULE_SHA256}"
require_report_field "${FAILURE_INCLUSIVE_ESTIMAND}" \
  denominator_schema_sha256 "${DENOMINATOR_SCHEMA_SHA256}"
require_report_field "${FAILURE_INCLUSIVE_ESTIMAND}" \
  contrast_registry_sha256 "${CONTRAST_REGISTRY_SHA256}"

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
    : "${STAGE1_EXTENSION_GATE_REPORT:?}"
    : "${STAGE1_EXTENSION_GATE_SHA256:?}"
    : "${STAGE1_EXTENSION_GATE_RULE_SHA256:?}"
    : "${STAGE1_DATASET_INDEX:?}"
    : "${STAGE1_DATASET_INDEX_SHA256:?}"
    : "${STAGE1_CONTRAST_DENOMINATOR_INDEX:?}"
    : "${STAGE1_CONTRAST_DENOMINATOR_INDEX_SHA256:?}"
    : "${S2A_SELECTIVITY_ANALYSIS_RULE_SHA256:?}"
    verify_common_stage_report \
      "${STAGE1_EXTENSION_GATE_REPORT}" "${STAGE1_EXTENSION_GATE_SHA256}" \
      stage1_extension_gate S1
    verify_frozen_report_hash "${STAGE1_DATASET_INDEX}" "${STAGE1_DATASET_INDEX_SHA256}"
    verify_contrast_denominator_index \
      "${STAGE1_CONTRAST_DENOMINATOR_INDEX}" \
      "${STAGE1_CONTRAST_DENOMINATOR_INDEX_SHA256}" S1 H1 \
      8 true "${CONTRAST_REGISTRY}"
    require_report_field "${STAGE1_EXTENSION_GATE_REPORT}" planned_units 40
    require_report_field "${STAGE1_EXTENSION_GATE_REPORT}" planned_row_coverage_count 40
    require_report_field "${STAGE1_EXTENSION_GATE_REPORT}" missing_planned_row_count 0
    require_report_field "${STAGE1_EXTENSION_GATE_REPORT}" unclassified_attempt_count 0
    require_report_field "${STAGE1_EXTENSION_GATE_REPORT}" invalid_retry_chain_count 0
    require_report_field "${STAGE1_EXTENSION_GATE_REPORT}" duplicate_eligible_outcome_count 0
    require_attempt_ledger_counts "${STAGE1_EXTENSION_GATE_REPORT}" 40
    require_planned_outcome_partition "${STAGE1_EXTENSION_GATE_REPORT}" 40
    require_report_field "${STAGE1_EXTENSION_GATE_REPORT}" \
      dataset_index_sha256 "${STAGE1_DATASET_INDEX_SHA256}"
    require_report_field "${STAGE1_EXTENSION_GATE_REPORT}" \
      gate_rule_sha256 "${STAGE1_EXTENSION_GATE_RULE_SHA256}"
    require_report_field "${STAGE1_EXTENSION_GATE_REPORT}" \
      failure_inclusive_estimand_sha256 "${FAILURE_INCLUSIVE_ESTIMAND_SHA256}"
    require_report_field "${STAGE1_EXTENSION_GATE_REPORT}" \
      denominator_schema_sha256 "${DENOMINATOR_SCHEMA_SHA256}"
    require_report_field "${STAGE1_EXTENSION_GATE_REPORT}" \
      contrast_registry_sha256 "${CONTRAST_REGISTRY_SHA256}"
    require_report_field "${STAGE1_EXTENSION_GATE_REPORT}" \
      contrast_denominator_index_sha256 "${STAGE1_CONTRAST_DENOMINATOR_INDEX_SHA256}"
    require_report_field "${STAGE1_EXTENSION_GATE_REPORT}" \
      contrast_denominator_recomputed_from_dataset_index true
    require_report_field "${STAGE1_EXTENSION_GATE_REPORT}" \
      contrast_denominator_verifier_sha256 \
      "${CONTRAST_DENOMINATOR_VERIFIER_SHA256}"
    ;;
  S2B/TRANSFER)
    [[ "${PATH_ID}/${CONTAINER}" == H1/C2 ]] || exit 2
    case "${CONDITION_ID}" in Bsmooth|FixedProfile|Bslosh) ;; *) exit 2 ;; esac
    [[ "${ORDER_POSITION}" =~ ^0[1-3]$ ]] || exit 2
    : "${STAGE1_EXTENSION_GATE_REPORT:?}"
    : "${STAGE1_EXTENSION_GATE_SHA256:?}"
    : "${STAGE1_EXTENSION_GATE_RULE_SHA256:?}"
    : "${STAGE1_DATASET_INDEX:?}"
    : "${STAGE1_DATASET_INDEX_SHA256:?}"
    : "${STAGE1_CONTRAST_DENOMINATOR_INDEX:?}"
    : "${STAGE1_CONTRAST_DENOMINATOR_INDEX_SHA256:?}"
    : "${STAGE2A_COMPLETION_REPORT:?}"
    : "${STAGE2A_COMPLETION_SHA256:?}"
    : "${STAGE2A_COMPLETION_RULE_SHA256:?}"
    : "${STAGE2A_DATASET_INDEX:?}"
    : "${STAGE2A_DATASET_INDEX_SHA256:?}"
    : "${STAGE2A_CONTRAST_DENOMINATOR_INDEX:?}"
    : "${STAGE2A_CONTRAST_DENOMINATOR_INDEX_SHA256:?}"
    : "${S2A_SELECTIVITY_ANALYSIS_RULE_SHA256:?}"
    : "${S2A_SELECTIVITY_ANALYSIS_REPORT:?}"
    : "${S2A_SELECTIVITY_ANALYSIS_REPORT_SHA256:?}"
    : "${STAGE2B_TRIGGER_REPORT:?}"
    : "${STAGE2B_TRIGGER_SHA256:?}"
    : "${STAGE2B_TRIGGER_RULE_SHA256:?}"
    : "${C2_CONFIG_SHA256:?}"
    verify_common_stage_report \
      "${STAGE1_EXTENSION_GATE_REPORT}" "${STAGE1_EXTENSION_GATE_SHA256}" \
      stage1_extension_gate S1
    verify_frozen_report_hash "${STAGE1_DATASET_INDEX}" "${STAGE1_DATASET_INDEX_SHA256}"
    verify_contrast_denominator_index \
      "${STAGE1_CONTRAST_DENOMINATOR_INDEX}" \
      "${STAGE1_CONTRAST_DENOMINATOR_INDEX_SHA256}" S1 H1 \
      8 true "${CONTRAST_REGISTRY}"
    require_report_field "${STAGE1_EXTENSION_GATE_REPORT}" planned_units 40
    require_report_field "${STAGE1_EXTENSION_GATE_REPORT}" planned_row_coverage_count 40
    require_report_field "${STAGE1_EXTENSION_GATE_REPORT}" missing_planned_row_count 0
    require_report_field "${STAGE1_EXTENSION_GATE_REPORT}" unclassified_attempt_count 0
    require_report_field "${STAGE1_EXTENSION_GATE_REPORT}" invalid_retry_chain_count 0
    require_report_field "${STAGE1_EXTENSION_GATE_REPORT}" duplicate_eligible_outcome_count 0
    require_attempt_ledger_counts "${STAGE1_EXTENSION_GATE_REPORT}" 40
    require_planned_outcome_partition "${STAGE1_EXTENSION_GATE_REPORT}" 40
    require_report_field "${STAGE1_EXTENSION_GATE_REPORT}" \
      dataset_index_sha256 "${STAGE1_DATASET_INDEX_SHA256}"
    require_report_field "${STAGE1_EXTENSION_GATE_REPORT}" \
      gate_rule_sha256 "${STAGE1_EXTENSION_GATE_RULE_SHA256}"
    require_report_field "${STAGE1_EXTENSION_GATE_REPORT}" \
      failure_inclusive_estimand_sha256 "${FAILURE_INCLUSIVE_ESTIMAND_SHA256}"
    require_report_field "${STAGE1_EXTENSION_GATE_REPORT}" \
      denominator_schema_sha256 "${DENOMINATOR_SCHEMA_SHA256}"
    require_report_field "${STAGE1_EXTENSION_GATE_REPORT}" \
      contrast_registry_sha256 "${CONTRAST_REGISTRY_SHA256}"
    require_report_field "${STAGE1_EXTENSION_GATE_REPORT}" \
      contrast_denominator_index_sha256 "${STAGE1_CONTRAST_DENOMINATOR_INDEX_SHA256}"
    require_report_field "${STAGE1_EXTENSION_GATE_REPORT}" \
      contrast_denominator_recomputed_from_dataset_index true
    require_report_field "${STAGE1_EXTENSION_GATE_REPORT}" \
      contrast_denominator_verifier_sha256 \
      "${CONTRAST_DENOMINATOR_VERIFIER_SHA256}"
    verify_common_stage_report \
      "${STAGE2A_COMPLETION_REPORT}" "${STAGE2A_COMPLETION_SHA256}" \
      stage2a_completion S2A
    verify_frozen_report_hash "${STAGE2A_DATASET_INDEX}" "${STAGE2A_DATASET_INDEX_SHA256}"
    verify_contrast_denominator_index \
      "${STAGE2A_CONTRAST_DENOMINATOR_INDEX}" \
      "${STAGE2A_CONTRAST_DENOMINATOR_INDEX_SHA256}" S2A L1 \
      8 true "${CONTRAST_REGISTRY}"
    require_report_field "${STAGE2A_COMPLETION_REPORT}" planned_units 24
    require_report_field "${STAGE2A_COMPLETION_REPORT}" planned_row_coverage_count 24
    require_report_field "${STAGE2A_COMPLETION_REPORT}" missing_planned_row_count 0
    require_report_field "${STAGE2A_COMPLETION_REPORT}" unclassified_attempt_count 0
    require_report_field "${STAGE2A_COMPLETION_REPORT}" invalid_retry_chain_count 0
    require_report_field "${STAGE2A_COMPLETION_REPORT}" duplicate_eligible_outcome_count 0
    require_attempt_ledger_counts "${STAGE2A_COMPLETION_REPORT}" 24
    require_planned_outcome_partition "${STAGE2A_COMPLETION_REPORT}" 24
    require_report_field "${STAGE2A_COMPLETION_REPORT}" \
      dataset_index_sha256 "${STAGE2A_DATASET_INDEX_SHA256}"
    require_report_field "${STAGE2A_COMPLETION_REPORT}" \
      stage1_extension_report_sha256 "${STAGE1_EXTENSION_GATE_SHA256}"
    require_report_field "${STAGE2A_COMPLETION_REPORT}" \
      completion_rule_sha256 "${STAGE2A_COMPLETION_RULE_SHA256}"
    require_report_field "${STAGE2A_COMPLETION_REPORT}" \
      selectivity_analysis_rule_sha256 "${S2A_SELECTIVITY_ANALYSIS_RULE_SHA256}"
    require_report_field "${STAGE2A_COMPLETION_REPORT}" \
      failure_inclusive_estimand_sha256 "${FAILURE_INCLUSIVE_ESTIMAND_SHA256}"
    require_report_field "${STAGE2A_COMPLETION_REPORT}" \
      denominator_schema_sha256 "${DENOMINATOR_SCHEMA_SHA256}"
    require_report_field "${STAGE2A_COMPLETION_REPORT}" \
      contrast_registry_sha256 "${CONTRAST_REGISTRY_SHA256}"
    require_report_field "${STAGE2A_COMPLETION_REPORT}" \
      contrast_denominator_index_sha256 "${STAGE2A_CONTRAST_DENOMINATOR_INDEX_SHA256}"
    require_report_field "${STAGE2A_COMPLETION_REPORT}" \
      contrast_denominator_recomputed_from_dataset_index true
    require_report_field "${STAGE2A_COMPLETION_REPORT}" \
      contrast_denominator_verifier_sha256 \
      "${CONTRAST_DENOMINATOR_VERIFIER_SHA256}"
    verify_frozen_report_hash \
      "${S2A_SELECTIVITY_ANALYSIS_REPORT}" "${S2A_SELECTIVITY_ANALYSIS_REPORT_SHA256}"
    require_report_field "${S2A_SELECTIVITY_ANALYSIS_REPORT}" protocol_id "${PROTOCOL_ID}"
    require_report_field "${S2A_SELECTIVITY_ANALYSIS_REPORT}" freeze_id "${FREEZE_ID}"
    require_report_field "${S2A_SELECTIVITY_ANALYSIS_REPORT}" \
      report_type s2a_selectivity_analysis
    require_report_field "${S2A_SELECTIVITY_ANALYSIS_REPORT}" stage S2A
    require_report_field "${S2A_SELECTIVITY_ANALYSIS_REPORT}" status ANALYZED
    require_report_field "${S2A_SELECTIVITY_ANALYSIS_REPORT}" \
      stage1_extension_report_sha256 "${STAGE1_EXTENSION_GATE_SHA256}"
    require_report_field "${S2A_SELECTIVITY_ANALYSIS_REPORT}" \
      stage2a_completion_report_sha256 "${STAGE2A_COMPLETION_SHA256}"
    require_report_field "${S2A_SELECTIVITY_ANALYSIS_REPORT}" \
      stage1_dataset_index_sha256 "${STAGE1_DATASET_INDEX_SHA256}"
    require_report_field "${S2A_SELECTIVITY_ANALYSIS_REPORT}" \
      stage2a_dataset_index_sha256 "${STAGE2A_DATASET_INDEX_SHA256}"
    require_report_field "${S2A_SELECTIVITY_ANALYSIS_REPORT}" \
      stage1_contrast_denominator_index_sha256 \
      "${STAGE1_CONTRAST_DENOMINATOR_INDEX_SHA256}"
    require_report_field "${S2A_SELECTIVITY_ANALYSIS_REPORT}" \
      stage2a_contrast_denominator_index_sha256 \
      "${STAGE2A_CONTRAST_DENOMINATOR_INDEX_SHA256}"
    require_report_field "${S2A_SELECTIVITY_ANALYSIS_REPORT}" \
      analysis_rule_sha256 "${S2A_SELECTIVITY_ANALYSIS_RULE_SHA256}"
    require_report_field "${S2A_SELECTIVITY_ANALYSIS_REPORT}" \
      failure_inclusive_estimand_sha256 "${FAILURE_INCLUSIVE_ESTIMAND_SHA256}"
    require_report_field "${S2A_SELECTIVITY_ANALYSIS_REPORT}" \
      denominator_schema_sha256 "${DENOMINATOR_SCHEMA_SHA256}"
    require_report_field "${S2A_SELECTIVITY_ANALYSIS_REPORT}" \
      contrast_registry_sha256 "${CONTRAST_REGISTRY_SHA256}"
    require_report_field "${S2A_SELECTIVITY_ANALYSIS_REPORT}" \
      contrast_denominator_verifier_sha256 \
      "${CONTRAST_DENOMINATOR_VERIFIER_SHA256}"
    s2a_selectivity_status="$(report_field "${S2A_SELECTIVITY_ANALYSIS_REPORT}" selectivity_status)"
    case "${s2a_selectivity_status}" in
      SUPPORTED|NOT_SUPPORTED|INCONCLUSIVE) ;;
      *) exit 2 ;;
    esac
    verify_common_stage_report \
      "${STAGE2B_TRIGGER_REPORT}" "${STAGE2B_TRIGGER_SHA256}" \
      stage2b_trigger S2B
    require_report_field "${STAGE2B_TRIGGER_REPORT}" stage2b_enabled true
    require_report_field "${STAGE2B_TRIGGER_REPORT}" planned_units 24
    require_report_field "${STAGE2B_TRIGGER_REPORT}" \
      stage2a_completion_report_sha256 "${STAGE2A_COMPLETION_SHA256}"
    require_report_field "${STAGE2B_TRIGGER_REPORT}" \
      s2a_selectivity_analysis_report_sha256 "${S2A_SELECTIVITY_ANALYSIS_REPORT_SHA256}"
    require_report_field "${STAGE2B_TRIGGER_REPORT}" \
      s2a_selectivity_status "${s2a_selectivity_status}"
    require_report_field "${STAGE2B_TRIGGER_REPORT}" \
      trigger_rule_sha256 "${STAGE2B_TRIGGER_RULE_SHA256}"
    require_report_field "${STAGE2B_TRIGGER_REPORT}" \
      c2_config_sha256 "${C2_CONFIG_SHA256}"
    ;;
  *) exit 2 ;;
esac
```

这段内容检查只能在第 5.4 节的外部 verifier 已同时验证静态 `RUN_ENV` 与 append-only execution-evidence index 后执行；stage report/hash、完整 dataset-index hash、逐 contrast denominator-index hash 和 rule/config hash 必须被同一 `FREEZE_ID` 下的 stage-entry evidence chain 绑定，但不得反向改写 `FREEZE_ID`。S2A 必须有 Stage I 40 planned-unit extension-gate 的 `status=PASS` report，且 40 个唯一 planned rows 全覆盖、所有 attempts 已分类、retry chain 无断裂、每 row 最多一个 eligible outcome；S2B 还必须有同样闭包的 S2A 24-unit completion PASS、独立 `S2A_SELECTIVITY_ANALYSIS` ANALYZED report 与预注册 trigger PASS reports。上面的 Python 只作 CSV/registry structural smoke；冻结的 `CONTRAST_DENOMINATOR_VERIFIER` 必须实际打开 denominator CSV、完整 dataset index、retry/segment/eligibility ledger、estimand/schema 与 contrast registry，从 planned rows 重新推导全部逐 contrast counts，拒绝自报数字，并把 `contrast_denominator_recomputed_from_dataset_index=true` 和自身 hash 写入 stage report。它还必须验证 S1/S2A 各为 8 个 planned blocks，进入下一 stage 前每个适用 contrast 均满足 `n_pair>=minimum_n_pair`。Stage I extension 和 S2A completion 分别绑定 H1、L1 index，S2A analysis 同时绑定二者，但禁止按相同 block 编号跨 batch 强配对。`S2A_SELECTIVITY_ANALYSIS_RULE_SHA256` 必须在 Stage I 前冻结，明确 H1/L1 不跨 batch 强配对、effect-difference/等价或非劣界、选择后限制和“batch-restricted supporting”措辞；completion PASS 只表示数据闭包，科学状态只由 `selectivity_status=SUPPORTED/NOT_SUPPORTED/INCONCLUSIVE` 表示；若 L1 未达 minimum，completion 仍可记录闭包，但不得进入 S2B。独立 shell 布尔值不再具有放行权，报告缺少唯一 protocol/freeze/type/stage/status/count/dataset/upstream/rule 字段时一律失败。

### 5.2 condition 与 backend 分层

```bash
set -euo pipefail
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

液体状态来源还必须由 manifest 明确导出：

```text
CURRENT_OBSERVER_SOURCE=odom
FUTURE_EXCITATION_SOURCE=predicted_robot_state_and_control
OBSERVER_FALLBACK_SOURCE=none
OBSERVER_FALLBACK_POLICY_SHA256=<sha256>
FORMAL_IMU_SHADOW_ENABLE=false
IMU_PROCESSING_CONFIG_SHA256=<sha256>
SOURCE_SELECTION_REPORT_SHA256=<sha256>
```

`CURRENT_OBSERVER_SOURCE` 表示 release 的 nominal source。当前代码唯一合法值是 `odom`，其 `OBSERVER_FALLBACK_SOURCE=none`；`FORMAL_IMU_SHADOW_ENABLE` 只控制诊断，不改变 solver source。若 G2S 触发 IMU 新 release，必须先修改代码、协议和 validator，生成 `IMU_IMPLEMENTATION_VALIDATION` report/hash，并固定导出 `CURRENT_OBSERVER_SOURCE=processed_imu`、`OBSERVER_FALLBACK_SOURCE=odom`、effective-source/status/epoch 与 fallback-policy hash，不能手工改一个字段冒充实现。该 release 必须具有自动 fallback；任何 `processed_imu -> odom` fallback trial 固定计 method failure，不能重标为 odom trial。

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
set -euo pipefail
: "${SCOUT_WS:?}"
export FREEZE_ROOT="${SCOUT_WS}/docs/实物实验注意事项/对比试验/实物对比实验/freeze"
export FREEZE_MANIFEST="${FREEZE_ROOT}/freeze_manifest.yaml"
export VALIDATOR="${SCOUT_WS}/src/scout_apps/control/spmpc_local_planner/scripts/validate_spmpc_formal_freeze.py"

test -s "${FREEZE_MANIFEST}"
test -x "${VALIDATOR}"
rg -q 'SMPCC-REAL-40-64-88-v2\.0' "${VALIDATOR}"
rg -q 'FixedProfile' "${VALIDATOR}"
rg -q 'S1.*CORE' "${VALIDATOR}"
rg -q 'G2A' "${VALIDATOR}"
rg -q 'G2S' "${VALIDATOR}"
rg -q 'G2C' "${VALIDATOR}"
rg -q 'G3' "${VALIDATOR}"
rg -q 'CURRENT_OBSERVER_SOURCE' "${VALIDATOR}"
```

这些检查当前会失败，这是预期的安全行为。不得删除检查后继续。即使将来所有 `rg` 通过，也只证明字符串存在，**不等于 formal validation**。

升级后的 validator 必须被实际调用并以非零退出码 fail closed；成功时保存完整 validation report、manifest hash 和明确的 `FORMAL_FREEZE_VALIDATION=PASS`，再从该报告生成并 hash 静态 `RUN_ENV` slots。它还必须验证 G0→G1→G2A→G2S→G2C→G3→G4→G5→G6 的完整顺序及各报告 hash、sample size、random table row/hash、release/source selection、trajectory/RGB/replay smoke、K6=8/16/24、stage-entry/retry evidence 的**规则与 schema**、condition-specific topic contract。采集后才产生的 stage-entry reports、retry authorizations 和 dataset indices 不可能进入首 trial 前的 freeze validation，只能由第 5.4 节的动态证据链验证。当前尚无这个 v2.0 invocation/export contract，所以本文不虚构命令。

### 5.4 run label

```bash
: "${SCOUT_WS:?}" "${FREEZE_ROOT:?导出只读 v2.0 freeze 目录}" \
  "${EXPECTED_FREEZE_ID:?从冻结登记导出预期 FREEZE_ID}" \
  "${EXECUTION_EVIDENCE_ROOT:?从静态 manifest 导出 append-only 证据根目录}" \
  "${EXECUTION_EVIDENCE_INDEX_SHA256:?导出本 attempt 动态证据 index SHA-256}" \
  "${FORMAL_ENV_VERIFIER_SHA256:?从只读 freeze receipt 导出 verifier SHA-256}"
export RUN_LABEL="${STAGE}_${GROUP}_${PATH_ID}_${CONTAINER}_${CONDITION_ID}_b${BLOCK}_r${REPEAT}"
export ATTEMPT_ID="${RUN_LABEL}"
export RUN_ENV="${FREEZE_ROOT}/run_env/${RUN_LABEL}.env"
export EXECUTION_EVIDENCE_ENV="${EXECUTION_EVIDENCE_ROOT}/${RUN_LABEL}.env"
export EXECUTION_EVIDENCE_INDEX="${EXECUTION_EVIDENCE_ROOT}/${RUN_LABEL}.index"
export FORMAL_ENV_VERIFIER="${SCOUT_WS}/src/scout_apps/control/spmpc_local_planner/scripts/verify_spmpc_formal_run_env.py"
```

这里只定义目标文件名，**不是生成步骤**。证据分两层：

1. 静态 freeze layer：future upgraded validator 先对 canonical manifest payload、随机表、预分配 `r01..rMAX` slots、dynamic schema 与 verifier hash 计算可重算 `FREEZE_ID`；随后 exporter 原子生成只含 allowlisted assignments、并引用该 ID 的 base `RUN_ENV`、静态 index 和 validation receipt。由于这些派生产物内部含 `FREEZE_ID`，其自身 hash 只能进入 anchored derived-artifact registry，不能反向作为 `FREEZE_ID` 输入。
2. 动态 execution layer：每个 attempt 前由冻结 verifier 生成 assignment-only `EXECUTION_EVIDENCE_ENV` 与 append-only index。它绑定同一 `FREEZE_ID`、静态 run slot、前一 chain head、适用的 stage-entry report，或 retry failure manifest/authorization。它在采集后产生，**不重算或改变 `FREEZE_ID`**。

两层文件都不得放在 `/tmp`，也不得在内部自报一个无法从外部复核的 PASS。future `verify_spmpc_formal_run_env.py` 必须在每个终端 `source` 前同时验证：静态 assignment-only 语法、freeze manifest/validation report、base run-env index/随机表行/slot、动态 assignment allowlist、execution-evidence index/hash/前向链、stage/retry reports、`RUN_LABEL` 和预期 `FREEZE_ID`；任一不符返回非零。验证后先 source 静态 `RUN_ENV`，再 source 仅含动态 allowlist 的 `EXECUTION_EVIDENCE_ENV`。当前该 verifier 与动态 exporter 都不存在，因此下面模板会按设计 fail closed。

静态 `RUN_ENV` 至少包含并验证：

- `PROTOCOL_ID/FREEZE_ID`、stage/group/condition/backend、randomization row/hash；
- freeze-validation report/path/hash 与 run-env SHA-256 index/hash；
- `PATH_JSON/PATH_JSON_SHA256`、profile/planner/tracker config hashes、`V_REF/W_SLOSH` 和容器参数；
- `CURRENT_OBSERVER_SOURCE/FUTURE_EXCITATION_SOURCE`、observer fallback source/policy hash、`FORMAL_IMU_SHADOW_ENABLE`、IMU processing/source-selection report hashes；
- `G3_EFFICACY_REPORT/G3_EFFICACY_REPORT_SHA256`、`G6_MEASUREMENT_ANALYSIS_REPORT/G6_MEASUREMENT_ANALYSIS_REPORT_SHA256` 与 `G3_OUTCOME_WINDOW_RULE_SHA256`；validator 必须打开两份报告，证明 G6 直接绑定唯一 G3 report/hash，且二者的 canonical `t_hvis_tail_sec/g3_outcome_window_rule_sha256` 与静态 `T_HVIS_TAIL/G3_OUTCOME_WINDOW_RULE_SHA256` 完全相同；
- `S2A_SELECTIVITY_ANALYSIS_RULE_SHA256`，绑定 batch-restricted effect-difference、等价/非劣界和禁止跨 batch 强配对的规则；
- `FAILURE_INCLUSIVE_ESTIMAND/FAILURE_INCLUSIVE_ESTIMAND_SHA256`、`DENOMINATOR_SCHEMA/DENOMINATOR_SCHEMA_SHA256`、`CONTRAST_REGISTRY/CONTRAST_REGISTRY_SHA256` 与 `CONTRAST_DENOMINATOR_VERIFIER/CONTRAST_DENOMINATOR_VERIFIER_SHA256`，绑定唯一 `Y_plan` hierarchical win–loss、missing/tie rule、`N_plan/N_attempt/N_method/N_pair`、逐 contrast schema/set 和 exact-inference 边界；validator 必须打开 estimand、schema、registry 与 verifier，核对前者对 G3 rule/schema/registry 的交叉绑定以及 registry 逐 contrast 的冻结 `n_block_plan/minimum_n_pair`，不能只接受并列 hash；
- stage-entry/retry schema、classifier/verifier hashes（包括 `FORMAL_ENV_VERIFIER_SHA256`）、最大 repeat 与每个预分配 repeat slot；不包含尚未发生的失败或尚未生成的 Stage I/II report hash；
- `T_SETTLE/T_ADMISSION_MAX/T_RGB_PRE/T_MOTION_MAX/T_HVIS_TAIL/T_POST_RECORD/FORMAL_RECORD_SEC`，其中 `T_RGB_PRE` 不小于 2 s、`T_POST_RECORD >= T_HVIS_TAIL`，record budget 覆盖全部 startup/admission/motion/tail；
- `START_POS_TOL/START_YAW_TOL/START_HOLD_SEC` 与冻结 path-replay gate；
- `RAW_CMD_TOPIC/POST_GATE_TOPIC/PUBLISHED_CMD_TOPIC` 及各自 publisher contract；
- `SHARED_EXECUTION_CONFIG`、`SHARED_EXECUTION_CONFIG_SHA256`、`FALLBACK_POLICY`、`FALLBACK_POLICY_SHA256` 和共同 motion/delay limits；
- RGB coverage/QC tool hash、输出目录，以及静态 `RUN_LABEL/ATTEMPT_ID/REPEAT/MAX_REPEAT/PLANNED_BLOCK_SEGMENT_ID` 和该 slot 是否允许消费动态 retry authorization。

动态 `EXECUTION_EVIDENCE_ENV` 的键集必须**恰好**为下列 allowlist，每个 key 恰好出现一次；不适用的路径/hash 使用空值，布尔量必须显式为 `false`，禁止省略、重复或增加额外键：

```text
ACTUAL_BLOCK_SEGMENT_ID
SPLIT_BLOCK
ACQUISITION_RETRY
RETRY_OF_ATTEMPT_ID
RETRY_REASON_FILE
RETRY_REASON_FILE_SHA256
RETRY_FAILURE_EVIDENCE_MANIFEST
RETRY_FAILURE_EVIDENCE_MANIFEST_SHA256
RETRY_AUTHORIZATION_REPORT
RETRY_AUTHORIZATION_REPORT_SHA256
STAGE1_EXTENSION_GATE_REPORT
STAGE1_EXTENSION_GATE_SHA256
STAGE1_DATASET_INDEX
STAGE1_DATASET_INDEX_SHA256
STAGE1_CONTRAST_DENOMINATOR_INDEX
STAGE1_CONTRAST_DENOMINATOR_INDEX_SHA256
STAGE2A_COMPLETION_REPORT
STAGE2A_COMPLETION_SHA256
STAGE2A_DATASET_INDEX
STAGE2A_DATASET_INDEX_SHA256
STAGE2A_CONTRAST_DENOMINATOR_INDEX
STAGE2A_CONTRAST_DENOMINATOR_INDEX_SHA256
S2A_SELECTIVITY_ANALYSIS_REPORT
S2A_SELECTIVITY_ANALYSIS_REPORT_SHA256
STAGE2B_TRIGGER_REPORT
STAGE2B_TRIGGER_SHA256
```

`EXECUTION_EVIDENCE_INDEX` 必须对这些值、静态 slot、前一 chain head 和适用报告做不可变登记。verifier 必须同时打开 retry/stage report 与逐 contrast denominator index 的**路径**并核对内容/hash，不能只验证一个 64 位字符串；它还必须强制 `SPLIT_BLOCK == (ACTUAL_BLOCK_SEGMENT_ID != PLANNED_BLOCK_SEGMENT_ID)`。动态层不能覆盖 `RUN_LABEL/ATTEMPT_ID/REPEAT`、condition/backend/path/config/profile/source 或任何分析规则。

`EXECUTION_EVIDENCE_INDEX_SHA256` 故意不在上述 allowlist：它必须在 source 前由只读 chain receipt/previous chain head 作为外部 trust anchor 提供，verifier 用它核对实际 index；各终端先保存该值，source 后重新计算 index hash，而不是允许 evidence env 覆盖它。

当前仓库没有该 exporter，因此下面多终端 formal 流程仍不可执行。

示例：

```text
S1_CORE_H1_C1_FixedProfile_b01_r01
S2A_SELECTIVITY_L1_C1_Bslosh_b01_r01
S2B_TRANSFER_H1_C2_Bsmooth_b01_r01
```

`r02+` 只用于矩阵文档允许且由冻结 verifier 签署的 method-independent acquisition failure；每个新 repeat 必须指向前一失败 attempt，并保留已有 artifact 或显式的 no-bag/no-postflight failure record。只有 `ACTUAL_BLOCK_SEGMENT_ID == PLANNED_BLOCK_SEGMENT_ID` 且与失败 attempt 同 segment 时才可恢复主配对；跨 segment recovery 必须 `SPLIT_BLOCK=true`，只能进入 reliability ledger。方法失败不得生成 retry authorization。

---

## 6. 多终端正式流程的公共部分

以下命令只说明 future v2.0 runner 的公共顺序。第 5.3 节未通过、或 validator 尚未生成只读 `RUN_ENV` 时，不得执行为 formal。每个终端在启动 ROS 节点前都必须先确认 `RUN_ENV` 可读，且其中的 protocol、freeze 与随机表身份已经由 upgraded validator 验证，不能手写一个同名文件绕过。

### 6.1 终端 A：standalone monitor

每个 trial 前 reset；monitor 只作方法无关支持，不进入任何控制 backend。

```bash
set -euo pipefail
: "${SCOUT_WS:?}" "${RUN_ENV:?}" "${RUN_LABEL:?}" \
  "${FREEZE_ROOT:?}" "${EXPECTED_FREEZE_ID:?}" \
  "${EXECUTION_EVIDENCE_ENV:?}" "${EXECUTION_EVIDENCE_INDEX:?}" \
  "${EXECUTION_EVIDENCE_INDEX_SHA256:?}"
test -r /opt/ros/noetic/setup.bash
source /opt/ros/noetic/setup.bash
test -r "${SCOUT_WS}/devel/setup.bash"
source "${SCOUT_WS}/devel/setup.bash"
test -r "${RUN_ENV}"
test ! -w "${RUN_ENV}"
test -r "${EXECUTION_EVIDENCE_ENV}"
test ! -w "${EXECUTION_EVIDENCE_ENV}"
test -r "${EXECUTION_EVIDENCE_INDEX}"
test ! -w "${EXECUTION_EVIDENCE_INDEX}"
FORMAL_ENV_VERIFIER="${SCOUT_WS}/src/scout_apps/control/spmpc_local_planner/scripts/verify_spmpc_formal_run_env.py"
test -x "${FORMAL_ENV_VERIFIER}"
: "${FORMAL_ENV_VERIFIER_SHA256:?从静态 freeze 登记导出 verifier SHA-256}"
[[ "${FORMAL_ENV_VERIFIER_SHA256}" =~ ^[0-9a-f]{64}$ ]]
[[ "$(sha256sum "${FORMAL_ENV_VERIFIER}" | awk '{print $1}')" == "${FORMAL_ENV_VERIFIER_SHA256}" ]]
TRUST_RUN_LABEL="${RUN_LABEL}"
TRUST_EXPECTED_FREEZE_ID="${EXPECTED_FREEZE_ID}"
TRUST_RUN_ENV="$(readlink -f "${RUN_ENV}")"
TRUST_EXECUTION_EVIDENCE_ENV="$(readlink -f "${EXECUTION_EVIDENCE_ENV}")"
TRUST_EXECUTION_EVIDENCE_INDEX="$(readlink -f "${EXECUTION_EVIDENCE_INDEX}")"
TRUST_EXECUTION_EVIDENCE_INDEX_SHA256="${EXECUTION_EVIDENCE_INDEX_SHA256}"
"${FORMAL_ENV_VERIFIER}" \
  --freeze-root "${FREEZE_ROOT}" \
  --run-env "${RUN_ENV}" \
  --run-label "${RUN_LABEL}" \
  --expected-freeze-id "${EXPECTED_FREEZE_ID}" \
  --execution-evidence-env "${EXECUTION_EVIDENCE_ENV}" \
  --execution-evidence-index "${EXECUTION_EVIDENCE_INDEX}" \
  --execution-evidence-index-sha256 "${EXECUTION_EVIDENCE_INDEX_SHA256}"
unset FREEZE_ID RUN_LABEL ATTEMPT_ID REPEAT PLANNED_BLOCK_SEGMENT_ID BLOCK_SEGMENT_ID
source "${TRUST_RUN_ENV}"
dynamic_evidence_keys=(
  ACTUAL_BLOCK_SEGMENT_ID SPLIT_BLOCK ACQUISITION_RETRY
  RETRY_OF_ATTEMPT_ID RETRY_REASON_FILE RETRY_REASON_FILE_SHA256
  RETRY_FAILURE_EVIDENCE_MANIFEST RETRY_FAILURE_EVIDENCE_MANIFEST_SHA256
  RETRY_AUTHORIZATION_REPORT RETRY_AUTHORIZATION_REPORT_SHA256
  STAGE1_EXTENSION_GATE_REPORT STAGE1_EXTENSION_GATE_SHA256
  STAGE1_DATASET_INDEX STAGE1_DATASET_INDEX_SHA256
  STAGE1_CONTRAST_DENOMINATOR_INDEX STAGE1_CONTRAST_DENOMINATOR_INDEX_SHA256
  STAGE2A_COMPLETION_REPORT STAGE2A_COMPLETION_SHA256
  STAGE2A_DATASET_INDEX STAGE2A_DATASET_INDEX_SHA256
  STAGE2A_CONTRAST_DENOMINATOR_INDEX STAGE2A_CONTRAST_DENOMINATOR_INDEX_SHA256
  S2A_SELECTIVITY_ANALYSIS_REPORT S2A_SELECTIVITY_ANALYSIS_REPORT_SHA256
  STAGE2B_TRIGGER_REPORT STAGE2B_TRIGGER_SHA256
)
unset "${dynamic_evidence_keys[@]}"
source "${TRUST_EXECUTION_EVIDENCE_ENV}"
for evidence_key in "${dynamic_evidence_keys[@]}"; do
  [[ -v "${evidence_key}" ]]
done
[[ "$(sha256sum "${TRUST_EXECUTION_EVIDENCE_INDEX}" | awk '{print $1}')" == "${TRUST_EXECUTION_EVIDENCE_INDEX_SHA256}" ]]
[[ "${FREEZE_ID}" == "${TRUST_EXPECTED_FREEZE_ID}" ]]
[[ "${RUN_LABEL}" == "${TRUST_RUN_LABEL}" ]]
[[ "${ATTEMPT_ID}" == "${TRUST_RUN_LABEL}" ]]
[[ "${REPEAT}" == "${TRUST_RUN_LABEL##*_r}" ]]
[[ "${REPEAT}" =~ ^(0[1-9]|[1-9][0-9])$ ]]
[[ "${MAX_REPEAT}" =~ ^(0[1-9]|[1-9][0-9])$ ]]
(( 10#${REPEAT} <= 10#${MAX_REPEAT} ))
[[ -n "${PLANNED_BLOCK_SEGMENT_ID}" && -n "${ACTUAL_BLOCK_SEGMENT_ID}" ]]
case "${SPLIT_BLOCK}" in true|false) ;; *) exit 2 ;; esac
if [[ "${ACTUAL_BLOCK_SEGMENT_ID}" == "${PLANNED_BLOCK_SEGMENT_ID}" ]]; then
  [[ "${SPLIT_BLOCK}" == false ]]
else
  [[ "${SPLIT_BLOCK}" == true ]]
fi
export BLOCK_SEGMENT_ID="${ACTUAL_BLOCK_SEGMENT_ID}"

retry_evidence_keys=(
  RETRY_OF_ATTEMPT_ID RETRY_REASON_FILE RETRY_REASON_FILE_SHA256
  RETRY_FAILURE_EVIDENCE_MANIFEST RETRY_FAILURE_EVIDENCE_MANIFEST_SHA256
  RETRY_AUTHORIZATION_REPORT RETRY_AUTHORIZATION_REPORT_SHA256
)
if [[ "${REPEAT}" == 01 ]]; then
  [[ "${ACQUISITION_RETRY}" == false && "${SPLIT_BLOCK}" == false ]]
  for evidence_key in "${retry_evidence_keys[@]}"; do
    [[ -z "${!evidence_key}" ]]
  done
else
  [[ "${ACQUISITION_RETRY}" == true ]]
  for evidence_key in "${retry_evidence_keys[@]}"; do
    [[ -n "${!evidence_key}" ]]
  done
  [[ "${RETRY_OF_ATTEMPT_ID%_r*}" == "${ATTEMPT_ID%_r*}" ]]
fi

stage1_evidence_keys=(
  STAGE1_EXTENSION_GATE_REPORT STAGE1_EXTENSION_GATE_SHA256
  STAGE1_DATASET_INDEX STAGE1_DATASET_INDEX_SHA256
  STAGE1_CONTRAST_DENOMINATOR_INDEX STAGE1_CONTRAST_DENOMINATOR_INDEX_SHA256
)
stage2_evidence_keys=(
  STAGE2A_COMPLETION_REPORT STAGE2A_COMPLETION_SHA256
  STAGE2A_DATASET_INDEX STAGE2A_DATASET_INDEX_SHA256
  STAGE2A_CONTRAST_DENOMINATOR_INDEX STAGE2A_CONTRAST_DENOMINATOR_INDEX_SHA256
  S2A_SELECTIVITY_ANALYSIS_REPORT S2A_SELECTIVITY_ANALYSIS_REPORT_SHA256
  STAGE2B_TRIGGER_REPORT STAGE2B_TRIGGER_SHA256
)
case "${STAGE}/${GROUP}" in
  S1/CORE)
    for evidence_key in "${stage1_evidence_keys[@]}" "${stage2_evidence_keys[@]}"; do
      [[ -z "${!evidence_key}" ]]
    done
    ;;
  S2A/SELECTIVITY)
    for evidence_key in "${stage1_evidence_keys[@]}"; do [[ -n "${!evidence_key}" ]]; done
    for evidence_key in "${stage2_evidence_keys[@]}"; do [[ -z "${!evidence_key}" ]]; done
    ;;
  S2B/TRANSFER)
    for evidence_key in "${stage1_evidence_keys[@]}" "${stage2_evidence_keys[@]}"; do
      [[ -n "${!evidence_key}" ]]
    done
    ;;
  *) exit 2 ;;
esac
[[ "${ROS_VERSION:-}" == 1 && "${ROS_DISTRO:-}" == noetic ]]
[[ "${FORMAL_FREEZE_VALIDATION:-}" == PASS ]]
: "${FREEZE_ID:?}"
[[ "${FREEZE_ID}" == "${EXPECTED_FREEZE_ID}" ]]

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
set -euo pipefail
: "${SCOUT_WS:?}" "${RUN_ENV:?}" "${RUN_LABEL:?}" \
  "${FREEZE_ROOT:?}" "${EXPECTED_FREEZE_ID:?}" \
  "${EXECUTION_EVIDENCE_ENV:?}" "${EXECUTION_EVIDENCE_INDEX:?}" \
  "${EXECUTION_EVIDENCE_INDEX_SHA256:?}"
test -r /opt/ros/noetic/setup.bash
source /opt/ros/noetic/setup.bash
test -r "${SCOUT_WS}/devel/setup.bash"
source "${SCOUT_WS}/devel/setup.bash"
test -r "${RUN_ENV}"
test ! -w "${RUN_ENV}"
test -r "${EXECUTION_EVIDENCE_ENV}"
test ! -w "${EXECUTION_EVIDENCE_ENV}"
test -r "${EXECUTION_EVIDENCE_INDEX}"
test ! -w "${EXECUTION_EVIDENCE_INDEX}"
FORMAL_ENV_VERIFIER="${SCOUT_WS}/src/scout_apps/control/spmpc_local_planner/scripts/verify_spmpc_formal_run_env.py"
test -x "${FORMAL_ENV_VERIFIER}"
: "${FORMAL_ENV_VERIFIER_SHA256:?从静态 freeze 登记导出 verifier SHA-256}"
[[ "${FORMAL_ENV_VERIFIER_SHA256}" =~ ^[0-9a-f]{64}$ ]]
[[ "$(sha256sum "${FORMAL_ENV_VERIFIER}" | awk '{print $1}')" == "${FORMAL_ENV_VERIFIER_SHA256}" ]]
TRUST_RUN_LABEL="${RUN_LABEL}"
TRUST_EXPECTED_FREEZE_ID="${EXPECTED_FREEZE_ID}"
TRUST_RUN_ENV="$(readlink -f "${RUN_ENV}")"
TRUST_EXECUTION_EVIDENCE_ENV="$(readlink -f "${EXECUTION_EVIDENCE_ENV}")"
TRUST_EXECUTION_EVIDENCE_INDEX="$(readlink -f "${EXECUTION_EVIDENCE_INDEX}")"
TRUST_EXECUTION_EVIDENCE_INDEX_SHA256="${EXECUTION_EVIDENCE_INDEX_SHA256}"
"${FORMAL_ENV_VERIFIER}" \
  --freeze-root "${FREEZE_ROOT}" \
  --run-env "${RUN_ENV}" \
  --run-label "${RUN_LABEL}" \
  --expected-freeze-id "${EXPECTED_FREEZE_ID}" \
  --execution-evidence-env "${EXECUTION_EVIDENCE_ENV}" \
  --execution-evidence-index "${EXECUTION_EVIDENCE_INDEX}" \
  --execution-evidence-index-sha256 "${EXECUTION_EVIDENCE_INDEX_SHA256}"
unset FREEZE_ID RUN_LABEL ATTEMPT_ID REPEAT PLANNED_BLOCK_SEGMENT_ID BLOCK_SEGMENT_ID
source "${TRUST_RUN_ENV}"
dynamic_evidence_keys=(
  ACTUAL_BLOCK_SEGMENT_ID SPLIT_BLOCK ACQUISITION_RETRY
  RETRY_OF_ATTEMPT_ID RETRY_REASON_FILE RETRY_REASON_FILE_SHA256
  RETRY_FAILURE_EVIDENCE_MANIFEST RETRY_FAILURE_EVIDENCE_MANIFEST_SHA256
  RETRY_AUTHORIZATION_REPORT RETRY_AUTHORIZATION_REPORT_SHA256
  STAGE1_EXTENSION_GATE_REPORT STAGE1_EXTENSION_GATE_SHA256
  STAGE1_DATASET_INDEX STAGE1_DATASET_INDEX_SHA256
  STAGE1_CONTRAST_DENOMINATOR_INDEX STAGE1_CONTRAST_DENOMINATOR_INDEX_SHA256
  STAGE2A_COMPLETION_REPORT STAGE2A_COMPLETION_SHA256
  STAGE2A_DATASET_INDEX STAGE2A_DATASET_INDEX_SHA256
  STAGE2A_CONTRAST_DENOMINATOR_INDEX STAGE2A_CONTRAST_DENOMINATOR_INDEX_SHA256
  S2A_SELECTIVITY_ANALYSIS_REPORT S2A_SELECTIVITY_ANALYSIS_REPORT_SHA256
  STAGE2B_TRIGGER_REPORT STAGE2B_TRIGGER_SHA256
)
unset "${dynamic_evidence_keys[@]}"
source "${TRUST_EXECUTION_EVIDENCE_ENV}"
for evidence_key in "${dynamic_evidence_keys[@]}"; do
  [[ -v "${evidence_key}" ]]
done
[[ "$(sha256sum "${TRUST_EXECUTION_EVIDENCE_INDEX}" | awk '{print $1}')" == "${TRUST_EXECUTION_EVIDENCE_INDEX_SHA256}" ]]
[[ "${FREEZE_ID}" == "${TRUST_EXPECTED_FREEZE_ID}" ]]
[[ "${RUN_LABEL}" == "${TRUST_RUN_LABEL}" ]]
[[ "${ATTEMPT_ID}" == "${TRUST_RUN_LABEL}" ]]
[[ "${REPEAT}" == "${TRUST_RUN_LABEL##*_r}" ]]
[[ "${REPEAT}" =~ ^(0[1-9]|[1-9][0-9])$ ]]
[[ "${MAX_REPEAT}" =~ ^(0[1-9]|[1-9][0-9])$ ]]
(( 10#${REPEAT} <= 10#${MAX_REPEAT} ))
[[ -n "${PLANNED_BLOCK_SEGMENT_ID}" && -n "${ACTUAL_BLOCK_SEGMENT_ID}" ]]
case "${SPLIT_BLOCK}" in true|false) ;; *) exit 2 ;; esac
if [[ "${ACTUAL_BLOCK_SEGMENT_ID}" == "${PLANNED_BLOCK_SEGMENT_ID}" ]]; then
  [[ "${SPLIT_BLOCK}" == false ]]
else
  [[ "${SPLIT_BLOCK}" == true ]]
fi
export BLOCK_SEGMENT_ID="${ACTUAL_BLOCK_SEGMENT_ID}"

retry_evidence_keys=(
  RETRY_OF_ATTEMPT_ID RETRY_REASON_FILE RETRY_REASON_FILE_SHA256
  RETRY_FAILURE_EVIDENCE_MANIFEST RETRY_FAILURE_EVIDENCE_MANIFEST_SHA256
  RETRY_AUTHORIZATION_REPORT RETRY_AUTHORIZATION_REPORT_SHA256
)
if [[ "${REPEAT}" == 01 ]]; then
  [[ "${ACQUISITION_RETRY}" == false && "${SPLIT_BLOCK}" == false ]]
  for evidence_key in "${retry_evidence_keys[@]}"; do
    [[ -z "${!evidence_key}" ]]
  done
else
  [[ "${ACQUISITION_RETRY}" == true ]]
  for evidence_key in "${retry_evidence_keys[@]}"; do
    [[ -n "${!evidence_key}" ]]
  done
  [[ "${RETRY_OF_ATTEMPT_ID%_r*}" == "${ATTEMPT_ID%_r*}" ]]
fi

stage1_evidence_keys=(
  STAGE1_EXTENSION_GATE_REPORT STAGE1_EXTENSION_GATE_SHA256
  STAGE1_DATASET_INDEX STAGE1_DATASET_INDEX_SHA256
  STAGE1_CONTRAST_DENOMINATOR_INDEX STAGE1_CONTRAST_DENOMINATOR_INDEX_SHA256
)
stage2_evidence_keys=(
  STAGE2A_COMPLETION_REPORT STAGE2A_COMPLETION_SHA256
  STAGE2A_DATASET_INDEX STAGE2A_DATASET_INDEX_SHA256
  STAGE2A_CONTRAST_DENOMINATOR_INDEX STAGE2A_CONTRAST_DENOMINATOR_INDEX_SHA256
  S2A_SELECTIVITY_ANALYSIS_REPORT S2A_SELECTIVITY_ANALYSIS_REPORT_SHA256
  STAGE2B_TRIGGER_REPORT STAGE2B_TRIGGER_SHA256
)
case "${STAGE}/${GROUP}" in
  S1/CORE)
    for evidence_key in "${stage1_evidence_keys[@]}" "${stage2_evidence_keys[@]}"; do
      [[ -z "${!evidence_key}" ]]
    done
    ;;
  S2A/SELECTIVITY)
    for evidence_key in "${stage1_evidence_keys[@]}"; do [[ -n "${!evidence_key}" ]]; done
    for evidence_key in "${stage2_evidence_keys[@]}"; do [[ -z "${!evidence_key}" ]]; done
    ;;
  S2B/TRANSFER)
    for evidence_key in "${stage1_evidence_keys[@]}" "${stage2_evidence_keys[@]}"; do
      [[ -n "${!evidence_key}" ]]
    done
    ;;
  *) exit 2 ;;
esac
[[ "${ROS_VERSION:-}" == 1 && "${ROS_DISTRO:-}" == noetic ]]
[[ "${FORMAL_FREEZE_VALIDATION:-}" == PASS ]]
: "${FREEZE_ID:?}" "${PATH_JSON:?}" "${PATH_JSON_SHA256:?}"
[[ "${FREEZE_ID}" == "${EXPECTED_FREEZE_ID}" ]]
test -s "${PATH_JSON}"
test ! -w "${PATH_JSON}"
PATH_JSON_ACTUAL_SHA256="$(sha256sum "${PATH_JSON}" | awk '{print $1}')"
[[ "${PATH_JSON_ACTUAL_SHA256}" == "${PATH_JSON_SHA256}" ]]

rosrun scout_local_planner fixed_global_path_runner.py \
  --mode replay \
  --path-file "${PATH_JSON}" \
  --output-topic /scout/global_path_fixed \
  --base-frame base_link \
  --manual-start \
  --start-pos-tol "${START_POS_TOL}" \
  --start-yaw-tol "${START_YAW_TOL}" \
  --start-hold-sec "${START_HOLD_SEC}" \
  --publish-rate 2.0 \
  --publish-count 0
```

看到 `Press Enter` 后不要立即继续。先启动 recorder 和本次 backend，完成 `T_SETTLE` 与配置检查。

### 6.3 终端 C：recorder

online 与 FixedProfile 的公共 recorder 必须先由 v2.0 smoke 证明 condition identity、raw/post/published command 和方法专属 topics 都能落包。

```bash
set -euo pipefail
: "${SCOUT_WS:?}" "${RUN_ENV:?}" "${RUN_LABEL:?}" \
  "${FREEZE_ROOT:?}" "${EXPECTED_FREEZE_ID:?}" \
  "${EXECUTION_EVIDENCE_ENV:?}" "${EXECUTION_EVIDENCE_INDEX:?}" \
  "${EXECUTION_EVIDENCE_INDEX_SHA256:?}"
test -r /opt/ros/noetic/setup.bash
source /opt/ros/noetic/setup.bash
test -r "${SCOUT_WS}/devel/setup.bash"
source "${SCOUT_WS}/devel/setup.bash"
test -r "${RUN_ENV}"
test ! -w "${RUN_ENV}"
test -r "${EXECUTION_EVIDENCE_ENV}"
test ! -w "${EXECUTION_EVIDENCE_ENV}"
test -r "${EXECUTION_EVIDENCE_INDEX}"
test ! -w "${EXECUTION_EVIDENCE_INDEX}"
FORMAL_ENV_VERIFIER="${SCOUT_WS}/src/scout_apps/control/spmpc_local_planner/scripts/verify_spmpc_formal_run_env.py"
test -x "${FORMAL_ENV_VERIFIER}"
: "${FORMAL_ENV_VERIFIER_SHA256:?从静态 freeze 登记导出 verifier SHA-256}"
[[ "${FORMAL_ENV_VERIFIER_SHA256}" =~ ^[0-9a-f]{64}$ ]]
[[ "$(sha256sum "${FORMAL_ENV_VERIFIER}" | awk '{print $1}')" == "${FORMAL_ENV_VERIFIER_SHA256}" ]]
TRUST_RUN_LABEL="${RUN_LABEL}"
TRUST_EXPECTED_FREEZE_ID="${EXPECTED_FREEZE_ID}"
TRUST_RUN_ENV="$(readlink -f "${RUN_ENV}")"
TRUST_EXECUTION_EVIDENCE_ENV="$(readlink -f "${EXECUTION_EVIDENCE_ENV}")"
TRUST_EXECUTION_EVIDENCE_INDEX="$(readlink -f "${EXECUTION_EVIDENCE_INDEX}")"
TRUST_EXECUTION_EVIDENCE_INDEX_SHA256="${EXECUTION_EVIDENCE_INDEX_SHA256}"
"${FORMAL_ENV_VERIFIER}" \
  --freeze-root "${FREEZE_ROOT}" \
  --run-env "${RUN_ENV}" \
  --run-label "${RUN_LABEL}" \
  --expected-freeze-id "${EXPECTED_FREEZE_ID}" \
  --execution-evidence-env "${EXECUTION_EVIDENCE_ENV}" \
  --execution-evidence-index "${EXECUTION_EVIDENCE_INDEX}" \
  --execution-evidence-index-sha256 "${EXECUTION_EVIDENCE_INDEX_SHA256}"
unset FREEZE_ID RUN_LABEL ATTEMPT_ID REPEAT PLANNED_BLOCK_SEGMENT_ID BLOCK_SEGMENT_ID
source "${TRUST_RUN_ENV}"
dynamic_evidence_keys=(
  ACTUAL_BLOCK_SEGMENT_ID SPLIT_BLOCK ACQUISITION_RETRY
  RETRY_OF_ATTEMPT_ID RETRY_REASON_FILE RETRY_REASON_FILE_SHA256
  RETRY_FAILURE_EVIDENCE_MANIFEST RETRY_FAILURE_EVIDENCE_MANIFEST_SHA256
  RETRY_AUTHORIZATION_REPORT RETRY_AUTHORIZATION_REPORT_SHA256
  STAGE1_EXTENSION_GATE_REPORT STAGE1_EXTENSION_GATE_SHA256
  STAGE1_DATASET_INDEX STAGE1_DATASET_INDEX_SHA256
  STAGE1_CONTRAST_DENOMINATOR_INDEX STAGE1_CONTRAST_DENOMINATOR_INDEX_SHA256
  STAGE2A_COMPLETION_REPORT STAGE2A_COMPLETION_SHA256
  STAGE2A_DATASET_INDEX STAGE2A_DATASET_INDEX_SHA256
  STAGE2A_CONTRAST_DENOMINATOR_INDEX STAGE2A_CONTRAST_DENOMINATOR_INDEX_SHA256
  S2A_SELECTIVITY_ANALYSIS_REPORT S2A_SELECTIVITY_ANALYSIS_REPORT_SHA256
  STAGE2B_TRIGGER_REPORT STAGE2B_TRIGGER_SHA256
)
unset "${dynamic_evidence_keys[@]}"
source "${TRUST_EXECUTION_EVIDENCE_ENV}"
for evidence_key in "${dynamic_evidence_keys[@]}"; do
  [[ -v "${evidence_key}" ]]
done
[[ "$(sha256sum "${TRUST_EXECUTION_EVIDENCE_INDEX}" | awk '{print $1}')" == "${TRUST_EXECUTION_EVIDENCE_INDEX_SHA256}" ]]
[[ "${FREEZE_ID}" == "${TRUST_EXPECTED_FREEZE_ID}" ]]
[[ "${RUN_LABEL}" == "${TRUST_RUN_LABEL}" ]]
[[ "${ATTEMPT_ID}" == "${TRUST_RUN_LABEL}" ]]
[[ "${REPEAT}" == "${TRUST_RUN_LABEL##*_r}" ]]
[[ "${REPEAT}" =~ ^(0[1-9]|[1-9][0-9])$ ]]
[[ "${MAX_REPEAT}" =~ ^(0[1-9]|[1-9][0-9])$ ]]
(( 10#${REPEAT} <= 10#${MAX_REPEAT} ))
[[ -n "${PLANNED_BLOCK_SEGMENT_ID}" && -n "${ACTUAL_BLOCK_SEGMENT_ID}" ]]
case "${SPLIT_BLOCK}" in true|false) ;; *) exit 2 ;; esac
if [[ "${ACTUAL_BLOCK_SEGMENT_ID}" == "${PLANNED_BLOCK_SEGMENT_ID}" ]]; then
  [[ "${SPLIT_BLOCK}" == false ]]
else
  [[ "${SPLIT_BLOCK}" == true ]]
fi
export BLOCK_SEGMENT_ID="${ACTUAL_BLOCK_SEGMENT_ID}"

retry_evidence_keys=(
  RETRY_OF_ATTEMPT_ID RETRY_REASON_FILE RETRY_REASON_FILE_SHA256
  RETRY_FAILURE_EVIDENCE_MANIFEST RETRY_FAILURE_EVIDENCE_MANIFEST_SHA256
  RETRY_AUTHORIZATION_REPORT RETRY_AUTHORIZATION_REPORT_SHA256
)
if [[ "${REPEAT}" == 01 ]]; then
  [[ "${ACQUISITION_RETRY}" == false && "${SPLIT_BLOCK}" == false ]]
  for evidence_key in "${retry_evidence_keys[@]}"; do
    [[ -z "${!evidence_key}" ]]
  done
else
  [[ "${ACQUISITION_RETRY}" == true ]]
  for evidence_key in "${retry_evidence_keys[@]}"; do
    [[ -n "${!evidence_key}" ]]
  done
  [[ "${RETRY_OF_ATTEMPT_ID%_r*}" == "${ATTEMPT_ID%_r*}" ]]
fi

stage1_evidence_keys=(
  STAGE1_EXTENSION_GATE_REPORT STAGE1_EXTENSION_GATE_SHA256
  STAGE1_DATASET_INDEX STAGE1_DATASET_INDEX_SHA256
  STAGE1_CONTRAST_DENOMINATOR_INDEX STAGE1_CONTRAST_DENOMINATOR_INDEX_SHA256
)
stage2_evidence_keys=(
  STAGE2A_COMPLETION_REPORT STAGE2A_COMPLETION_SHA256
  STAGE2A_DATASET_INDEX STAGE2A_DATASET_INDEX_SHA256
  STAGE2A_CONTRAST_DENOMINATOR_INDEX STAGE2A_CONTRAST_DENOMINATOR_INDEX_SHA256
  S2A_SELECTIVITY_ANALYSIS_REPORT S2A_SELECTIVITY_ANALYSIS_REPORT_SHA256
  STAGE2B_TRIGGER_REPORT STAGE2B_TRIGGER_SHA256
)
case "${STAGE}/${GROUP}" in
  S1/CORE)
    for evidence_key in "${stage1_evidence_keys[@]}" "${stage2_evidence_keys[@]}"; do
      [[ -z "${!evidence_key}" ]]
    done
    ;;
  S2A/SELECTIVITY)
    for evidence_key in "${stage1_evidence_keys[@]}"; do [[ -n "${!evidence_key}" ]]; done
    for evidence_key in "${stage2_evidence_keys[@]}"; do [[ -z "${!evidence_key}" ]]; done
    ;;
  S2B/TRANSFER)
    for evidence_key in "${stage1_evidence_keys[@]}" "${stage2_evidence_keys[@]}"; do
      [[ -n "${!evidence_key}" ]]
    done
    ;;
  *) exit 2 ;;
esac
[[ "${ROS_VERSION:-}" == 1 && "${ROS_DISTRO:-}" == noetic ]]
[[ "${FORMAL_FREEZE_VALIDATION:-}" == PASS ]]
: "${FREEZE_ID:?}"
[[ "${FREEZE_ID}" == "${EXPECTED_FREEZE_ID}" ]]
: "${FORMAL_RECORD_SEC:?}" "${T_HVIS_TAIL:?}" \
  "${G3_OUTCOME_WINDOW_RULE_SHA256:?}" \
  "${G3_EFFICACY_REPORT:?}" "${G3_EFFICACY_REPORT_SHA256:?}" \
  "${G6_MEASUREMENT_ANALYSIS_REPORT:?}" "${G6_MEASUREMENT_ANALYSIS_REPORT_SHA256:?}"
test -s "${G3_EFFICACY_REPORT}"
test -s "${G6_MEASUREMENT_ANALYSIS_REPORT}"
[[ "${G3_EFFICACY_REPORT_SHA256}" =~ ^[0-9a-f]{64}$ ]]
[[ "${G6_MEASUREMENT_ANALYSIS_REPORT_SHA256}" =~ ^[0-9a-f]{64}$ ]]
[[ "$(sha256sum "${G3_EFFICACY_REPORT}" | awk '{print $1}')" == "${G3_EFFICACY_REPORT_SHA256}" ]]
[[ "$(sha256sum "${G6_MEASUREMENT_ANALYSIS_REPORT}" | awk '{print $1}')" == "${G6_MEASUREMENT_ANALYSIS_REPORT_SHA256}" ]]
grep -Fxq "t_hvis_tail_sec=${T_HVIS_TAIL}" "${G3_EFFICACY_REPORT}"
grep -Fxq "g3_outcome_window_rule_sha256=${G3_OUTCOME_WINDOW_RULE_SHA256}" "${G3_EFFICACY_REPORT}"
grep -Fxq "g3_efficacy_report_sha256=${G3_EFFICACY_REPORT_SHA256}" "${G6_MEASUREMENT_ANALYSIS_REPORT}"
grep -Fxq "t_hvis_tail_sec=${T_HVIS_TAIL}" "${G6_MEASUREMENT_ANALYSIS_REPORT}"
grep -Fxq "g3_outcome_window_rule_sha256=${G3_OUTCOME_WINDOW_RULE_SHA256}" "${G6_MEASUREMENT_ANALYSIS_REPORT}"
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
ATTEMPT_ID="${ATTEMPT_ID}" \
REPEAT="${REPEAT}" \
MAX_REPEAT="${MAX_REPEAT}" \
PLANNED_BLOCK_SEGMENT_ID="${PLANNED_BLOCK_SEGMENT_ID}" \
BLOCK_SEGMENT_ID="${BLOCK_SEGMENT_ID}" \
SPLIT_BLOCK="${SPLIT_BLOCK}" \
NAME="${RUN_LABEL}" \
OUT_DIR="${OUT_DIR}" \
RECORD_SEC="${FORMAL_RECORD_SEC}" \
T_HVIS_TAIL="${T_HVIS_TAIL}" \
G3_OUTCOME_WINDOW_RULE_SHA256="${G3_OUTCOME_WINDOW_RULE_SHA256}" \
G3_EFFICACY_REPORT="${G3_EFFICACY_REPORT}" \
G3_EFFICACY_REPORT_SHA256="${G3_EFFICACY_REPORT_SHA256}" \
G6_MEASUREMENT_ANALYSIS_REPORT="${G6_MEASUREMENT_ANALYSIS_REPORT}" \
G6_MEASUREMENT_ANALYSIS_REPORT_SHA256="${G6_MEASUREMENT_ANALYSIS_REPORT_SHA256}" \
RECORD_RGB=true \
RECORD_CAMERA=true \
RECORD_CAMERA_COMPRESSED=false \
RECORD_DEPTH=false \
RECORD_SCAN=true \
RECORD_MOCAP=false \
RECORD_ROSOUT=true \
RECORD_STANDALONE_SLOSH=true \
RECORD_ONLINE_LIQUID=false \
LIQUID_EXPORT_AFTER_RECORD=false \
IMU_TOPIC=/imu/data \
IMU_SHADOW_ENABLE="${FORMAL_IMU_SHADOW_ENABLE}" \
RECORD_ALL_EXISTING_TOPICS=false \
RECORD_TOPIC_INFO=true \
ROSBAG_BUFFER_SIZE_MB=4096 \
PATH_SOURCE_MODE=replay \
PATH_FILE="${PATH_JSON}" \
PATH_EXPECTED_SHA256="${PATH_JSON_SHA256}" \
REQUIRE_PATH_HASH=true \
ACQUISITION_RETRY="${ACQUISITION_RETRY}" \
RETRY_OF_ATTEMPT_ID="${RETRY_OF_ATTEMPT_ID}" \
RETRY_REASON_FILE="${RETRY_REASON_FILE}" \
RETRY_REASON_FILE_SHA256="${RETRY_REASON_FILE_SHA256}" \
RETRY_FAILURE_EVIDENCE_MANIFEST="${RETRY_FAILURE_EVIDENCE_MANIFEST}" \
RETRY_FAILURE_EVIDENCE_MANIFEST_SHA256="${RETRY_FAILURE_EVIDENCE_MANIFEST_SHA256}" \
RETRY_AUTHORIZATION_REPORT="${RETRY_AUTHORIZATION_REPORT}" \
RETRY_AUTHORIZATION_REPORT_SHA256="${RETRY_AUTHORIZATION_REPORT_SHA256}" \
OPERATOR_NOTE="protocol=${PROTOCOL_ID} freeze=${FREEZE_ID} attempt=${ATTEMPT_ID} repeat=${REPEAT} condition=${CONDITION_ID} backend=${METHOD_BACKEND} block=${BLOCK} position=${ORDER_POSITION} planned_segment=${PLANNED_BLOCK_SEGMENT_ID} actual_segment=${ACTUAL_BLOCK_SEGMENT_ID} split_block=${SPLIT_BLOCK} retry_of=${RETRY_OF_ATTEMPT_ID:-none} t_hvis_tail_sec=${T_HVIS_TAIL} g3_outcome_window_rule_sha256=${G3_OUTCOME_WINDOW_RULE_SHA256} g3_efficacy_report_sha256=${G3_EFFICACY_REPORT_SHA256} g6_measurement_analysis_report_sha256=${G6_MEASUREMENT_ANALYSIS_REPORT_SHA256}" \
bash src/scout_apps/control/spmpc_local_planner/scripts/record_spmpc_full_rgb_bag.sh
```

`FORMAL_RECORD_SEC` 不能固定凭经验写 90 s。validator 必须证明 `T_HVIS_TAIL>0`、`T_POST_RECORD >= T_HVIS_TAIL`，并证明总记录预算不少于 recorder/backend startup、`T_SETTLE`、`T_ADMISSION_MAX`、冻结的 `T_MOTION_MAX` 与 `T_POST_RECORD` 之和，另留 buffer；由于 recorder 在 backend 和 admission 前启动，任一部分都必须计入。第 8.1 节释放路径前还必须实时证明 `.bag.active` 仍存在且持续增长。

注意：当前 recorder 可见若干 `/reference/*`、`/profile_cap/*` 和 `/mpc/*` topics，但还没有通过 FixedProfile formal contract smoke，也不会按 `RAW_CMD_TOPIC/POST_GATE_TOPIC/PUBLISHED_CMD_TOPIC` 动态构造录制白名单。`CONDITION_ID/METHOD_BACKEND` 目前只能进入环境快照，旧 summarizer 不会按 v2.0 结构化解析。未经升级的 recorder/sidecar/summarizer 不得签署 FixedProfile trial。

当前 recorder 白名单已包含 raw `/imu/data` 和两路 observer debug；缺失的 debug topic 不会使 rosbag 启动失败。当前实现中 `FORMAL_IMU_SHADOW_ENABLE=false` 时不会发布两路 observer-bank debug，因此 formal 只能要求 raw `/imu/data`、raw `/odom` 和 solver 实际状态完整，并用冻结脚本离线复算 observer；不能要求 bag 中存在在线 odom/IMU shadow 消息，也不能把空 topic 当成零值。

---

## 7. 终端 D：启动 condition backend

### 7.1 online MPCC 条件

只适用于 `B0/Bsmooth/SmoothMatch/Bslosh`：

```bash
set -euo pipefail
: "${SCOUT_WS:?}" "${RUN_ENV:?}" "${RUN_LABEL:?}" \
  "${FREEZE_ROOT:?}" "${EXPECTED_FREEZE_ID:?}" \
  "${EXECUTION_EVIDENCE_ENV:?}" "${EXECUTION_EVIDENCE_INDEX:?}" \
  "${EXECUTION_EVIDENCE_INDEX_SHA256:?}"
test -r /opt/ros/noetic/setup.bash
source /opt/ros/noetic/setup.bash
test -r "${SCOUT_WS}/devel/setup.bash"
source "${SCOUT_WS}/devel/setup.bash"
test -r "${RUN_ENV}"
test ! -w "${RUN_ENV}"
test -r "${EXECUTION_EVIDENCE_ENV}"
test ! -w "${EXECUTION_EVIDENCE_ENV}"
test -r "${EXECUTION_EVIDENCE_INDEX}"
test ! -w "${EXECUTION_EVIDENCE_INDEX}"
FORMAL_ENV_VERIFIER="${SCOUT_WS}/src/scout_apps/control/spmpc_local_planner/scripts/verify_spmpc_formal_run_env.py"
test -x "${FORMAL_ENV_VERIFIER}"
: "${FORMAL_ENV_VERIFIER_SHA256:?从静态 freeze 登记导出 verifier SHA-256}"
[[ "${FORMAL_ENV_VERIFIER_SHA256}" =~ ^[0-9a-f]{64}$ ]]
[[ "$(sha256sum "${FORMAL_ENV_VERIFIER}" | awk '{print $1}')" == "${FORMAL_ENV_VERIFIER_SHA256}" ]]
TRUST_RUN_LABEL="${RUN_LABEL}"
TRUST_EXPECTED_FREEZE_ID="${EXPECTED_FREEZE_ID}"
TRUST_RUN_ENV="$(readlink -f "${RUN_ENV}")"
TRUST_EXECUTION_EVIDENCE_ENV="$(readlink -f "${EXECUTION_EVIDENCE_ENV}")"
TRUST_EXECUTION_EVIDENCE_INDEX="$(readlink -f "${EXECUTION_EVIDENCE_INDEX}")"
TRUST_EXECUTION_EVIDENCE_INDEX_SHA256="${EXECUTION_EVIDENCE_INDEX_SHA256}"
"${FORMAL_ENV_VERIFIER}" \
  --freeze-root "${FREEZE_ROOT}" \
  --run-env "${RUN_ENV}" \
  --run-label "${RUN_LABEL}" \
  --expected-freeze-id "${EXPECTED_FREEZE_ID}" \
  --execution-evidence-env "${EXECUTION_EVIDENCE_ENV}" \
  --execution-evidence-index "${EXECUTION_EVIDENCE_INDEX}" \
  --execution-evidence-index-sha256 "${EXECUTION_EVIDENCE_INDEX_SHA256}"
unset FREEZE_ID RUN_LABEL ATTEMPT_ID REPEAT PLANNED_BLOCK_SEGMENT_ID BLOCK_SEGMENT_ID
source "${TRUST_RUN_ENV}"
dynamic_evidence_keys=(
  ACTUAL_BLOCK_SEGMENT_ID SPLIT_BLOCK ACQUISITION_RETRY
  RETRY_OF_ATTEMPT_ID RETRY_REASON_FILE RETRY_REASON_FILE_SHA256
  RETRY_FAILURE_EVIDENCE_MANIFEST RETRY_FAILURE_EVIDENCE_MANIFEST_SHA256
  RETRY_AUTHORIZATION_REPORT RETRY_AUTHORIZATION_REPORT_SHA256
  STAGE1_EXTENSION_GATE_REPORT STAGE1_EXTENSION_GATE_SHA256
  STAGE1_DATASET_INDEX STAGE1_DATASET_INDEX_SHA256
  STAGE1_CONTRAST_DENOMINATOR_INDEX STAGE1_CONTRAST_DENOMINATOR_INDEX_SHA256
  STAGE2A_COMPLETION_REPORT STAGE2A_COMPLETION_SHA256
  STAGE2A_DATASET_INDEX STAGE2A_DATASET_INDEX_SHA256
  STAGE2A_CONTRAST_DENOMINATOR_INDEX STAGE2A_CONTRAST_DENOMINATOR_INDEX_SHA256
  S2A_SELECTIVITY_ANALYSIS_REPORT S2A_SELECTIVITY_ANALYSIS_REPORT_SHA256
  STAGE2B_TRIGGER_REPORT STAGE2B_TRIGGER_SHA256
)
unset "${dynamic_evidence_keys[@]}"
source "${TRUST_EXECUTION_EVIDENCE_ENV}"
for evidence_key in "${dynamic_evidence_keys[@]}"; do
  [[ -v "${evidence_key}" ]]
done
[[ "$(sha256sum "${TRUST_EXECUTION_EVIDENCE_INDEX}" | awk '{print $1}')" == "${TRUST_EXECUTION_EVIDENCE_INDEX_SHA256}" ]]
[[ "${FREEZE_ID}" == "${TRUST_EXPECTED_FREEZE_ID}" ]]
[[ "${RUN_LABEL}" == "${TRUST_RUN_LABEL}" ]]
[[ "${ATTEMPT_ID}" == "${TRUST_RUN_LABEL}" ]]
[[ "${REPEAT}" == "${TRUST_RUN_LABEL##*_r}" ]]
[[ "${REPEAT}" =~ ^(0[1-9]|[1-9][0-9])$ ]]
[[ "${MAX_REPEAT}" =~ ^(0[1-9]|[1-9][0-9])$ ]]
(( 10#${REPEAT} <= 10#${MAX_REPEAT} ))
[[ -n "${PLANNED_BLOCK_SEGMENT_ID}" && -n "${ACTUAL_BLOCK_SEGMENT_ID}" ]]
case "${SPLIT_BLOCK}" in true|false) ;; *) exit 2 ;; esac
if [[ "${ACTUAL_BLOCK_SEGMENT_ID}" == "${PLANNED_BLOCK_SEGMENT_ID}" ]]; then
  [[ "${SPLIT_BLOCK}" == false ]]
else
  [[ "${SPLIT_BLOCK}" == true ]]
fi
export BLOCK_SEGMENT_ID="${ACTUAL_BLOCK_SEGMENT_ID}"

retry_evidence_keys=(
  RETRY_OF_ATTEMPT_ID RETRY_REASON_FILE RETRY_REASON_FILE_SHA256
  RETRY_FAILURE_EVIDENCE_MANIFEST RETRY_FAILURE_EVIDENCE_MANIFEST_SHA256
  RETRY_AUTHORIZATION_REPORT RETRY_AUTHORIZATION_REPORT_SHA256
)
if [[ "${REPEAT}" == 01 ]]; then
  [[ "${ACQUISITION_RETRY}" == false && "${SPLIT_BLOCK}" == false ]]
  for evidence_key in "${retry_evidence_keys[@]}"; do
    [[ -z "${!evidence_key}" ]]
  done
else
  [[ "${ACQUISITION_RETRY}" == true ]]
  for evidence_key in "${retry_evidence_keys[@]}"; do
    [[ -n "${!evidence_key}" ]]
  done
  [[ "${RETRY_OF_ATTEMPT_ID%_r*}" == "${ATTEMPT_ID%_r*}" ]]
fi

stage1_evidence_keys=(
  STAGE1_EXTENSION_GATE_REPORT STAGE1_EXTENSION_GATE_SHA256
  STAGE1_DATASET_INDEX STAGE1_DATASET_INDEX_SHA256
  STAGE1_CONTRAST_DENOMINATOR_INDEX STAGE1_CONTRAST_DENOMINATOR_INDEX_SHA256
)
stage2_evidence_keys=(
  STAGE2A_COMPLETION_REPORT STAGE2A_COMPLETION_SHA256
  STAGE2A_DATASET_INDEX STAGE2A_DATASET_INDEX_SHA256
  STAGE2A_CONTRAST_DENOMINATOR_INDEX STAGE2A_CONTRAST_DENOMINATOR_INDEX_SHA256
  S2A_SELECTIVITY_ANALYSIS_REPORT S2A_SELECTIVITY_ANALYSIS_REPORT_SHA256
  STAGE2B_TRIGGER_REPORT STAGE2B_TRIGGER_SHA256
)
case "${STAGE}/${GROUP}" in
  S1/CORE)
    for evidence_key in "${stage1_evidence_keys[@]}" "${stage2_evidence_keys[@]}"; do
      [[ -z "${!evidence_key}" ]]
    done
    ;;
  S2A/SELECTIVITY)
    for evidence_key in "${stage1_evidence_keys[@]}"; do [[ -n "${!evidence_key}" ]]; done
    for evidence_key in "${stage2_evidence_keys[@]}"; do [[ -z "${!evidence_key}" ]]; done
    ;;
  S2B/TRANSFER)
    for evidence_key in "${stage1_evidence_keys[@]}" "${stage2_evidence_keys[@]}"; do
      [[ -n "${!evidence_key}" ]]
    done
    ;;
  *) exit 2 ;;
esac
[[ "${ROS_VERSION:-}" == 1 && "${ROS_DISTRO:-}" == noetic ]]
[[ "${FORMAL_FREEZE_VALIDATION:-}" == PASS ]]
: "${FREEZE_ID:?}"
[[ "${FREEZE_ID}" == "${EXPECTED_FREEZE_ID}" ]]

[[ "${METHOD_BACKEND}" == online_mpcc ]] || exit 2
[[ "${CURRENT_OBSERVER_SOURCE}" == odom ]] || {
  echo "NO-GO: current release only supports odom as SolverInput.slosh" >&2
  exit 2
}

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
  imu_topic:=/imu/data \
  imu_shadow_enable:="${FORMAL_IMU_SHADOW_ENABLE}" \
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

`roslaunch` 在终端 D 前台持续运行，所以下列检查必须在另一个只读终端 E、且在终端 B 按 Enter 之前执行；不能把它们写在 `roslaunch` 后面等待节点退出：

```bash
set -euo pipefail
: "${SCOUT_WS:?}" "${RUN_ENV:?}" "${RUN_LABEL:?}" \
  "${FREEZE_ROOT:?}" "${EXPECTED_FREEZE_ID:?}" \
  "${EXECUTION_EVIDENCE_ENV:?}" "${EXECUTION_EVIDENCE_INDEX:?}" \
  "${EXECUTION_EVIDENCE_INDEX_SHA256:?}"
test -r /opt/ros/noetic/setup.bash
source /opt/ros/noetic/setup.bash
test -r "${SCOUT_WS}/devel/setup.bash"
source "${SCOUT_WS}/devel/setup.bash"
test -r "${RUN_ENV}"
test ! -w "${RUN_ENV}"
test -r "${EXECUTION_EVIDENCE_ENV}"
test ! -w "${EXECUTION_EVIDENCE_ENV}"
test -r "${EXECUTION_EVIDENCE_INDEX}"
test ! -w "${EXECUTION_EVIDENCE_INDEX}"
FORMAL_ENV_VERIFIER="${SCOUT_WS}/src/scout_apps/control/spmpc_local_planner/scripts/verify_spmpc_formal_run_env.py"
test -x "${FORMAL_ENV_VERIFIER}"
: "${FORMAL_ENV_VERIFIER_SHA256:?从静态 freeze 登记导出 verifier SHA-256}"
[[ "${FORMAL_ENV_VERIFIER_SHA256}" =~ ^[0-9a-f]{64}$ ]]
[[ "$(sha256sum "${FORMAL_ENV_VERIFIER}" | awk '{print $1}')" == "${FORMAL_ENV_VERIFIER_SHA256}" ]]
TRUST_RUN_LABEL="${RUN_LABEL}"
TRUST_EXPECTED_FREEZE_ID="${EXPECTED_FREEZE_ID}"
TRUST_RUN_ENV="$(readlink -f "${RUN_ENV}")"
TRUST_EXECUTION_EVIDENCE_ENV="$(readlink -f "${EXECUTION_EVIDENCE_ENV}")"
TRUST_EXECUTION_EVIDENCE_INDEX="$(readlink -f "${EXECUTION_EVIDENCE_INDEX}")"
TRUST_EXECUTION_EVIDENCE_INDEX_SHA256="${EXECUTION_EVIDENCE_INDEX_SHA256}"
"${FORMAL_ENV_VERIFIER}" \
  --freeze-root "${FREEZE_ROOT}" \
  --run-env "${RUN_ENV}" \
  --run-label "${RUN_LABEL}" \
  --expected-freeze-id "${EXPECTED_FREEZE_ID}" \
  --execution-evidence-env "${EXECUTION_EVIDENCE_ENV}" \
  --execution-evidence-index "${EXECUTION_EVIDENCE_INDEX}" \
  --execution-evidence-index-sha256 "${EXECUTION_EVIDENCE_INDEX_SHA256}"
unset FREEZE_ID RUN_LABEL ATTEMPT_ID REPEAT PLANNED_BLOCK_SEGMENT_ID BLOCK_SEGMENT_ID
source "${TRUST_RUN_ENV}"
dynamic_evidence_keys=(
  ACTUAL_BLOCK_SEGMENT_ID SPLIT_BLOCK ACQUISITION_RETRY
  RETRY_OF_ATTEMPT_ID RETRY_REASON_FILE RETRY_REASON_FILE_SHA256
  RETRY_FAILURE_EVIDENCE_MANIFEST RETRY_FAILURE_EVIDENCE_MANIFEST_SHA256
  RETRY_AUTHORIZATION_REPORT RETRY_AUTHORIZATION_REPORT_SHA256
  STAGE1_EXTENSION_GATE_REPORT STAGE1_EXTENSION_GATE_SHA256
  STAGE1_DATASET_INDEX STAGE1_DATASET_INDEX_SHA256
  STAGE1_CONTRAST_DENOMINATOR_INDEX STAGE1_CONTRAST_DENOMINATOR_INDEX_SHA256
  STAGE2A_COMPLETION_REPORT STAGE2A_COMPLETION_SHA256
  STAGE2A_DATASET_INDEX STAGE2A_DATASET_INDEX_SHA256
  STAGE2A_CONTRAST_DENOMINATOR_INDEX STAGE2A_CONTRAST_DENOMINATOR_INDEX_SHA256
  S2A_SELECTIVITY_ANALYSIS_REPORT S2A_SELECTIVITY_ANALYSIS_REPORT_SHA256
  STAGE2B_TRIGGER_REPORT STAGE2B_TRIGGER_SHA256
)
unset "${dynamic_evidence_keys[@]}"
source "${TRUST_EXECUTION_EVIDENCE_ENV}"
for evidence_key in "${dynamic_evidence_keys[@]}"; do
  [[ -v "${evidence_key}" ]]
done
[[ "$(sha256sum "${TRUST_EXECUTION_EVIDENCE_INDEX}" | awk '{print $1}')" == "${TRUST_EXECUTION_EVIDENCE_INDEX_SHA256}" ]]
[[ "${FREEZE_ID}" == "${TRUST_EXPECTED_FREEZE_ID}" ]]
[[ "${RUN_LABEL}" == "${TRUST_RUN_LABEL}" ]]
[[ "${ATTEMPT_ID}" == "${TRUST_RUN_LABEL}" ]]
[[ "${REPEAT}" == "${TRUST_RUN_LABEL##*_r}" ]]
[[ "${REPEAT}" =~ ^(0[1-9]|[1-9][0-9])$ ]]
[[ "${MAX_REPEAT}" =~ ^(0[1-9]|[1-9][0-9])$ ]]
(( 10#${REPEAT} <= 10#${MAX_REPEAT} ))
[[ -n "${PLANNED_BLOCK_SEGMENT_ID}" && -n "${ACTUAL_BLOCK_SEGMENT_ID}" ]]
case "${SPLIT_BLOCK}" in true|false) ;; *) exit 2 ;; esac
if [[ "${ACTUAL_BLOCK_SEGMENT_ID}" == "${PLANNED_BLOCK_SEGMENT_ID}" ]]; then
  [[ "${SPLIT_BLOCK}" == false ]]
else
  [[ "${SPLIT_BLOCK}" == true ]]
fi
export BLOCK_SEGMENT_ID="${ACTUAL_BLOCK_SEGMENT_ID}"

retry_evidence_keys=(
  RETRY_OF_ATTEMPT_ID RETRY_REASON_FILE RETRY_REASON_FILE_SHA256
  RETRY_FAILURE_EVIDENCE_MANIFEST RETRY_FAILURE_EVIDENCE_MANIFEST_SHA256
  RETRY_AUTHORIZATION_REPORT RETRY_AUTHORIZATION_REPORT_SHA256
)
if [[ "${REPEAT}" == 01 ]]; then
  [[ "${ACQUISITION_RETRY}" == false && "${SPLIT_BLOCK}" == false ]]
  for evidence_key in "${retry_evidence_keys[@]}"; do
    [[ -z "${!evidence_key}" ]]
  done
else
  [[ "${ACQUISITION_RETRY}" == true ]]
  for evidence_key in "${retry_evidence_keys[@]}"; do
    [[ -n "${!evidence_key}" ]]
  done
  [[ "${RETRY_OF_ATTEMPT_ID%_r*}" == "${ATTEMPT_ID%_r*}" ]]
fi

stage1_evidence_keys=(
  STAGE1_EXTENSION_GATE_REPORT STAGE1_EXTENSION_GATE_SHA256
  STAGE1_DATASET_INDEX STAGE1_DATASET_INDEX_SHA256
  STAGE1_CONTRAST_DENOMINATOR_INDEX STAGE1_CONTRAST_DENOMINATOR_INDEX_SHA256
)
stage2_evidence_keys=(
  STAGE2A_COMPLETION_REPORT STAGE2A_COMPLETION_SHA256
  STAGE2A_DATASET_INDEX STAGE2A_DATASET_INDEX_SHA256
  STAGE2A_CONTRAST_DENOMINATOR_INDEX STAGE2A_CONTRAST_DENOMINATOR_INDEX_SHA256
  S2A_SELECTIVITY_ANALYSIS_REPORT S2A_SELECTIVITY_ANALYSIS_REPORT_SHA256
  STAGE2B_TRIGGER_REPORT STAGE2B_TRIGGER_SHA256
)
case "${STAGE}/${GROUP}" in
  S1/CORE)
    for evidence_key in "${stage1_evidence_keys[@]}" "${stage2_evidence_keys[@]}"; do
      [[ -z "${!evidence_key}" ]]
    done
    ;;
  S2A/SELECTIVITY)
    for evidence_key in "${stage1_evidence_keys[@]}"; do [[ -n "${!evidence_key}" ]]; done
    for evidence_key in "${stage2_evidence_keys[@]}"; do [[ -z "${!evidence_key}" ]]; done
    ;;
  S2B/TRANSFER)
    for evidence_key in "${stage1_evidence_keys[@]}" "${stage2_evidence_keys[@]}"; do
      [[ -n "${!evidence_key}" ]]
    done
    ;;
  *) exit 2 ;;
esac
[[ "${ROS_VERSION:-}" == 1 && "${ROS_DISTRO:-}" == noetic ]]
[[ "${FORMAL_FREEZE_VALIDATION:-}" == PASS ]]
: "${FREEZE_ID:?}"
[[ "${FREEZE_ID}" == "${EXPECTED_FREEZE_ID}" ]]

timeout 10s rostopic echo -n 1 /spmpc/debug/effective_config \
  > "${OUT_DIR}/${RUN_LABEL}_effective_config_before_start.txt"
rosparam get /spmpc_local_planner \
  > "${OUT_DIR}/${RUN_LABEL}_planner_rosparam_before_start.yaml"
rostopic info /cmd_vel
```

upgraded validator 必须核对实际 `v_ref`、`w_slosh`、container、`CURRENT_OBSERVER_SOURCE=odom`、shadow policy 和 release hash。当前旧 `effective_scalar_check` 只适合 online branch。

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

当前 formal 候选默认 `FORMAL_IMU_SHADOW_ENABLE=false`。若以后决定在 formal 在线保留 shadow，下面人工多终端流程不再足够：必须使用已验收的一键 runner 自动执行“无残留 publisher → recorder `.bag.active` → planner 零速 → IMU READY → path/start gate（generate 模式才含 goal）”，并把 bias failure、frame mismatch、gap/epoch reset 和 READY timeout 写入固定 failure taxonomy。不得人工按 Enter 绕过 READY。

```bash
set -euo pipefail
: "${SCOUT_WS:?}" "${RUN_ENV:?}" "${RUN_LABEL:?}" \
  "${FREEZE_ROOT:?}" "${EXPECTED_FREEZE_ID:?}" \
  "${EXECUTION_EVIDENCE_ENV:?}" "${EXECUTION_EVIDENCE_INDEX:?}" \
  "${EXECUTION_EVIDENCE_INDEX_SHA256:?}"
test -r /opt/ros/noetic/setup.bash
source /opt/ros/noetic/setup.bash
test -r "${SCOUT_WS}/devel/setup.bash"
source "${SCOUT_WS}/devel/setup.bash"
test -r "${RUN_ENV}"
test ! -w "${RUN_ENV}"
test -r "${EXECUTION_EVIDENCE_ENV}"
test ! -w "${EXECUTION_EVIDENCE_ENV}"
test -r "${EXECUTION_EVIDENCE_INDEX}"
test ! -w "${EXECUTION_EVIDENCE_INDEX}"
FORMAL_ENV_VERIFIER="${SCOUT_WS}/src/scout_apps/control/spmpc_local_planner/scripts/verify_spmpc_formal_run_env.py"
test -x "${FORMAL_ENV_VERIFIER}"
: "${FORMAL_ENV_VERIFIER_SHA256:?从静态 freeze 登记导出 verifier SHA-256}"
[[ "${FORMAL_ENV_VERIFIER_SHA256}" =~ ^[0-9a-f]{64}$ ]]
[[ "$(sha256sum "${FORMAL_ENV_VERIFIER}" | awk '{print $1}')" == "${FORMAL_ENV_VERIFIER_SHA256}" ]]
TRUST_RUN_LABEL="${RUN_LABEL}"
TRUST_EXPECTED_FREEZE_ID="${EXPECTED_FREEZE_ID}"
TRUST_RUN_ENV="$(readlink -f "${RUN_ENV}")"
TRUST_EXECUTION_EVIDENCE_ENV="$(readlink -f "${EXECUTION_EVIDENCE_ENV}")"
TRUST_EXECUTION_EVIDENCE_INDEX="$(readlink -f "${EXECUTION_EVIDENCE_INDEX}")"
TRUST_EXECUTION_EVIDENCE_INDEX_SHA256="${EXECUTION_EVIDENCE_INDEX_SHA256}"
"${FORMAL_ENV_VERIFIER}" \
  --freeze-root "${FREEZE_ROOT}" \
  --run-env "${RUN_ENV}" \
  --run-label "${RUN_LABEL}" \
  --expected-freeze-id "${EXPECTED_FREEZE_ID}" \
  --execution-evidence-env "${EXECUTION_EVIDENCE_ENV}" \
  --execution-evidence-index "${EXECUTION_EVIDENCE_INDEX}" \
  --execution-evidence-index-sha256 "${EXECUTION_EVIDENCE_INDEX_SHA256}"
unset FREEZE_ID RUN_LABEL ATTEMPT_ID REPEAT PLANNED_BLOCK_SEGMENT_ID BLOCK_SEGMENT_ID
source "${TRUST_RUN_ENV}"
dynamic_evidence_keys=(
  ACTUAL_BLOCK_SEGMENT_ID SPLIT_BLOCK ACQUISITION_RETRY
  RETRY_OF_ATTEMPT_ID RETRY_REASON_FILE RETRY_REASON_FILE_SHA256
  RETRY_FAILURE_EVIDENCE_MANIFEST RETRY_FAILURE_EVIDENCE_MANIFEST_SHA256
  RETRY_AUTHORIZATION_REPORT RETRY_AUTHORIZATION_REPORT_SHA256
  STAGE1_EXTENSION_GATE_REPORT STAGE1_EXTENSION_GATE_SHA256
  STAGE1_DATASET_INDEX STAGE1_DATASET_INDEX_SHA256
  STAGE1_CONTRAST_DENOMINATOR_INDEX STAGE1_CONTRAST_DENOMINATOR_INDEX_SHA256
  STAGE2A_COMPLETION_REPORT STAGE2A_COMPLETION_SHA256
  STAGE2A_DATASET_INDEX STAGE2A_DATASET_INDEX_SHA256
  STAGE2A_CONTRAST_DENOMINATOR_INDEX STAGE2A_CONTRAST_DENOMINATOR_INDEX_SHA256
  S2A_SELECTIVITY_ANALYSIS_REPORT S2A_SELECTIVITY_ANALYSIS_REPORT_SHA256
  STAGE2B_TRIGGER_REPORT STAGE2B_TRIGGER_SHA256
)
unset "${dynamic_evidence_keys[@]}"
source "${TRUST_EXECUTION_EVIDENCE_ENV}"
for evidence_key in "${dynamic_evidence_keys[@]}"; do
  [[ -v "${evidence_key}" ]]
done
[[ "$(sha256sum "${TRUST_EXECUTION_EVIDENCE_INDEX}" | awk '{print $1}')" == "${TRUST_EXECUTION_EVIDENCE_INDEX_SHA256}" ]]
[[ "${FREEZE_ID}" == "${TRUST_EXPECTED_FREEZE_ID}" ]]
[[ "${RUN_LABEL}" == "${TRUST_RUN_LABEL}" ]]
[[ "${ATTEMPT_ID}" == "${TRUST_RUN_LABEL}" ]]
[[ "${REPEAT}" == "${TRUST_RUN_LABEL##*_r}" ]]
[[ "${REPEAT}" =~ ^(0[1-9]|[1-9][0-9])$ ]]
[[ "${MAX_REPEAT}" =~ ^(0[1-9]|[1-9][0-9])$ ]]
(( 10#${REPEAT} <= 10#${MAX_REPEAT} ))
[[ -n "${PLANNED_BLOCK_SEGMENT_ID}" && -n "${ACTUAL_BLOCK_SEGMENT_ID}" ]]
case "${SPLIT_BLOCK}" in true|false) ;; *) exit 2 ;; esac
if [[ "${ACTUAL_BLOCK_SEGMENT_ID}" == "${PLANNED_BLOCK_SEGMENT_ID}" ]]; then
  [[ "${SPLIT_BLOCK}" == false ]]
else
  [[ "${SPLIT_BLOCK}" == true ]]
fi
export BLOCK_SEGMENT_ID="${ACTUAL_BLOCK_SEGMENT_ID}"

retry_evidence_keys=(
  RETRY_OF_ATTEMPT_ID RETRY_REASON_FILE RETRY_REASON_FILE_SHA256
  RETRY_FAILURE_EVIDENCE_MANIFEST RETRY_FAILURE_EVIDENCE_MANIFEST_SHA256
  RETRY_AUTHORIZATION_REPORT RETRY_AUTHORIZATION_REPORT_SHA256
)
if [[ "${REPEAT}" == 01 ]]; then
  [[ "${ACQUISITION_RETRY}" == false && "${SPLIT_BLOCK}" == false ]]
  for evidence_key in "${retry_evidence_keys[@]}"; do
    [[ -z "${!evidence_key}" ]]
  done
else
  [[ "${ACQUISITION_RETRY}" == true ]]
  for evidence_key in "${retry_evidence_keys[@]}"; do
    [[ -n "${!evidence_key}" ]]
  done
  [[ "${RETRY_OF_ATTEMPT_ID%_r*}" == "${ATTEMPT_ID%_r*}" ]]
fi

stage1_evidence_keys=(
  STAGE1_EXTENSION_GATE_REPORT STAGE1_EXTENSION_GATE_SHA256
  STAGE1_DATASET_INDEX STAGE1_DATASET_INDEX_SHA256
  STAGE1_CONTRAST_DENOMINATOR_INDEX STAGE1_CONTRAST_DENOMINATOR_INDEX_SHA256
)
stage2_evidence_keys=(
  STAGE2A_COMPLETION_REPORT STAGE2A_COMPLETION_SHA256
  STAGE2A_DATASET_INDEX STAGE2A_DATASET_INDEX_SHA256
  STAGE2A_CONTRAST_DENOMINATOR_INDEX STAGE2A_CONTRAST_DENOMINATOR_INDEX_SHA256
  S2A_SELECTIVITY_ANALYSIS_REPORT S2A_SELECTIVITY_ANALYSIS_REPORT_SHA256
  STAGE2B_TRIGGER_REPORT STAGE2B_TRIGGER_SHA256
)
case "${STAGE}/${GROUP}" in
  S1/CORE)
    for evidence_key in "${stage1_evidence_keys[@]}" "${stage2_evidence_keys[@]}"; do
      [[ -z "${!evidence_key}" ]]
    done
    ;;
  S2A/SELECTIVITY)
    for evidence_key in "${stage1_evidence_keys[@]}"; do [[ -n "${!evidence_key}" ]]; done
    for evidence_key in "${stage2_evidence_keys[@]}"; do [[ -z "${!evidence_key}" ]]; done
    ;;
  S2B/TRANSFER)
    for evidence_key in "${stage1_evidence_keys[@]}" "${stage2_evidence_keys[@]}"; do
      [[ -n "${!evidence_key}" ]]
    done
    ;;
  *) exit 2 ;;
esac
[[ "${ROS_VERSION:-}" == 1 && "${ROS_DISTRO:-}" == noetic ]]
[[ "${FORMAL_FREEZE_VALIDATION:-}" == PASS ]]
: "${FREEZE_ID:?}" "${OUT_DIR:?}" "${RUN_LABEL:?}" "${T_SETTLE:?}"
[[ "${FREEZE_ID}" == "${EXPECTED_FREEZE_ID}" ]]
test -d "${OUT_DIR}"

export RESET_SIDECAR="${OUT_DIR}/${RUN_LABEL}_slosh_reset.txt"
export RESET_SIDECAR_TMP="${RESET_SIDECAR}.tmp.$$"
test ! -e "${RESET_SIDECAR}"
test ! -e "${RESET_SIDECAR_TMP}"
set +e
(
  printf 'reset_start_utc=%s\n' "$(date --utc --iso-8601=ns)"
  if rosservice call /slosh/reset; then
    echo 'pass=true'
  else
    service_rc=$?
    echo 'pass=false'
    printf 'service_rc=%s\n' "${service_rc}"
    exit "${service_rc}"
  fi
) > "${RESET_SIDECAR_TMP}" 2>&1
RESET_RC=$?
set -e
mv -- "${RESET_SIDECAR_TMP}" "${RESET_SIDECAR}"
if (( RESET_RC != 0 )); then
  exit "${RESET_RC}"
fi
grep -Fxq 'pass=true' "${RESET_SIDECAR}"

export SETTLE_SIDECAR="${OUT_DIR}/${RUN_LABEL}_t_settle.txt"
test ! -e "${SETTLE_SIDECAR}"
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
with pathlib.Path(sys.argv[2]).open("x", encoding="utf-8") as stream:
    stream.write(f"pass=true\ntarget_sec={target}\nelapsed_sec={elapsed}\n")
PY
grep -Fxq 'pass=true' "${SETTLE_SIDECAR}"

export RECORDER_ACTIVE_BAG="${OUT_DIR}/${RUN_LABEL}.bag.active"
export RECORDER_ACTIVE_SIDECAR="${OUT_DIR}/${RUN_LABEL}_recorder_active_gate.txt"
test ! -e "${RECORDER_ACTIVE_SIDECAR}"
test -s "${RECORDER_ACTIVE_BAG}"
ACTIVE_SIZE_BEFORE="$(stat -c %s "${RECORDER_ACTIVE_BAG}")"
sleep 2
test -s "${RECORDER_ACTIVE_BAG}"
ACTIVE_SIZE_AFTER="$(stat -c %s "${RECORDER_ACTIVE_BAG}")"
(( ACTIVE_SIZE_AFTER > ACTIVE_SIZE_BEFORE ))
printf 'pass=true\nactive_bag=%s\nsize_before=%s\nsize_after=%s\n' \
  "${RECORDER_ACTIVE_BAG}" "${ACTIVE_SIZE_BEFORE}" "${ACTIVE_SIZE_AFTER}" \
  > "${RECORDER_ACTIVE_SIDECAR}"
```

这里的 `/slosh/reset` 只重置 terminal A 的 standalone monitor，不等价于 planner 内部 observer、delay predictor、governor、warm start 或 FixedProfile tracker 的 method-state reset。future unified runner 必须分别留下适用 backend 的 reset 成功证据；任一 reset 失败都不能继续等待或启动运动。

按 Enter 前，future runner 还必须留下可审计的运动前在线 RGB 证据：recorder-ready/首个 `zero_locked+valid` `/liquid/measurement.header.stamp`、满足 `T_RGB_PRE≥2 s` 的 elapsed gate，以及 bag 闭合后“首个有效源图 stamp → first effective motion”不少于 `T_RGB_PRE` 的复核 sidecar。人工看到画面或等待约 2 s 不能解除 formal `NO-GO`。

随后确认：

- recorder 已连续保存运动前 stamped online RGB scalar/quality，且 topic whitelist 不含任何图像流；
- 起点/航向门合格；
- condition/backend/config/profile hash 与随机表行一致；
- `RAW_CMD_TOPIC/POST_GATE_TOPIC/PUBLISHED_CMD_TOPIC` 与当前 backend 一致、非空且 publisher 合法；
- shared execution/fallback config hash 与 manifest 一致；
- `/cmd_vel` publisher 唯一；
- 安全员、急停和路径走廊就位。

回到终端 B 按 Enter。运动前在线 RGB 覆盖只证明数据存在；冻结 visual-start/motion/tail QC 必须由 stamped-quality 字段重算。只有 visual-start QC 使用的全部 source stamps 都早于 first effective motion、且失败原因与 assignment 无关时，才可登记 acquisition failure 并按 verifier 授权的 `r02+`/split-block 规则处理。运动后的遮挡、clipping、振动或相机失效不得追溯改写成 visual-start failure；协议不保存视频，因此也不得事后换检测算法恢复。

### 8.2 到达或失败后的顺序

1. 到达、方法失败或安全停止，记录 first effective motion/first arrival/stop/timeout timestamp；
2. 成功 trial 在 first arrival 后继续记录 G3→G6→formal 原样继承的 `T_HVIS_TAIL/G3_OUTCOME_WINDOW_RULE_SHA256`；failure/timeout trial 保持零命令并记录至 first effective motion 后 `T_MOTION_MAX + T_HVIS_TAIL`；
3. 满足冻结 tail 后 Ctrl+C 正常关闭 recorder；若安全要求立即断电，保留 bag 并标记 tail incomplete/method failure，不补跑替换；
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
- `/cmd_vel`、`/odom`、raw `/imu/data`、`/tf`、`/tf_static`、`/scout/global_path_fixed`；
- `/liquid/measurement` stamped scalar/quality、`/liquid/height*`、camera_info 和视觉配置/hash sidecar；bag 内 image message type 必须为 0；
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

若 manifest 冻结 `FORMAL_IMU_SHADOW_ENABLE=true`，所有适用 online 条件还必须记录：

```text
/spmpc/debug/slosh_observer_odom
/spmpc/debug/slosh_observer_imu
```

并验证其 CPU/回调开销没有破坏 runtime fairness。`Bslosh` 必须记录 nominal/effective source、selected-source state/status/epoch 和 `H_modal`；只有 non-selected observer 才是 supporting diagnostic。当前 odom release 的 selected source 才是 odom；若未来 IMU release 通过实现门，processed-IMU debug 就成为 selected-source 正式证据，odom observer 降为 supporting/fallback 证据。若 shadow 冻结为 false，则不要求 non-selected online debug 存在，但 raw `/odom` 与 `/imu/data` 仍按冻结合同保留。

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
- RGB primary `H_vis,p95(motion+T_HVIS_TAIL)`、`G3_OUTCOME_WINDOW_RULE_SHA256`、10%–90% progress sensitivity、统一 tail RMS，以及 failure/timeout 的冻结截窗与 tail-completeness；
- missing frame、clipping、visual-start 和同步质量。

### 9.5 failure 与 retry

- solver、tracker、profile execution、tracking、timeout、安全终止：方法失败，不得成功补跑替换；
- observer-source fallback 固定使该 trial 成为 method failure，保留在 nominal IMU-release 分母且不得重标；其他 execution fallback 也是方法相关事件，必须保留，其 trial/event-level 规则以 manifest 为准，不能现场决定；
- 运动前 camera/rosbag 未启动、完全基于 first-motion 前帧的 blinded visual-start QC 失败、可由独立日志证明的外部存储/相机硬件中断、方法无关现场侵入：经冻结 classifier 证明与 assignment 无关后才可登记 acquisition failure；
- condition-specific runner/profile/config 拒绝或不匹配、冻结 assignment/environment 配置错误、backend/topic/CPU 压力导致的数据损失，以及运动诱发的遮挡、clipping、振动或视频失效：不得签发 acquisition retry；按冻结 protocol/readiness 或 method/measurement failure 规则归档；
- `r02+` 必须由冻结 verifier 逐次授权，保留已有 bag/sidecar/log/reason 或显式 no-bag/no-postflight 记录，并使用相同 condition/backend/profile/config hash；
- 只有 `ACTUAL_BLOCK_SEGMENT_ID == PLANNED_BLOCK_SEGMENT_ID` 且与失败 attempt 同 segment 的授权 retry 才能恢复 paired eligibility；跨 segment recovery 必须 `SPLIT_BLOCK=true`，仅进入 reliability ledger；
- 全部 attempts 分别记录 acquisition/readiness/postflight 状态；method success/failure 固定按 planned rows 计数，并另报 `N_pair/N_block_plan`，不能用一个 generic `failure_count`、attempt 分母或 `attempts-eligible` 公式代替；Stage I/S2A 还必须生成由相应 stage report 绑定的逐 contrast denominator CSV/index hash，S2A analysis 同时绑定 H1/L1 两份 index；
- 现场只检查完整性和安全，不查看正式方法排名后改变剩余随机顺序。

---

## 10. 下一次 trial

1. 从相应 v2.0 randomization CSV 读取下一行；
2. 更新 `CONDITION_ID/BLOCK/ORDER_POSITION`，不手工重排；
3. 选择预冻结的 `RUN_LABEL/RUN_ENV` slot，并由冻结 verifier 生成本 attempt 的 append-only execution-evidence env/index；
4. 同时验证同一 `FREEZE_ID`、静态 slot、动态 chain head、path/config/profile hash；
5. 每个 trial 重启 planner/tracker；
6. monitor reset，等待 `T_SETTLE`；
7. 新 recorder 生成独立 bag；
8. 发生方法失败后仍按随机表继续其余条件；
9. 不根据 RGB、tracking 或 internal-model ranking 改方法、权重、`v_ref` 或 profile。

当前下一步不是运行 Stage I，而是依次完成：

```text
ROS1 Noetic build/test + publish_cmd_vel=false replay
→ G0：冻结 claim/comparator 与 fairness variables
→ G1：冻结唯一 base rotation release
→ G2A：用既有 15 条关闭 delay/integration/objective/generated-artifact 审计
→ G2S 回放 r02/r03，再录冻结 H0s + Bsmooth + RGB 同-trial配对
→ 冻结 odom；若 IMU 达门槛则建立新 release 并重做 source-sensitive gates
→ G2C：在选定 source 下最小 W2/W5 确认并冻结唯一 Bslosh candidate
→ G3 独立 RGB efficacy
→ 实现/冻结 recorder/QC/trajectory extractor 与 replay 工具
→ G4：关闭 trajectory/四相位/online-input replay gate
→ 审计/修改 generator 的唯一 base-profile 参数
→ 实现/冻结 FixedProfile real formal wrapper + shared execution chain
→ 冻结 SmoothMatch 和唯一 FixedProfile comparator
→ G5：关闭 comparator-fairness gate
→ 升级 manifest、validator、tests 和 K6
→ G6：冻结 measurement/analysis、sample size、failure、S2A selectivity rule 与新随机表，并归档完整 gate hash chain
→ 生成只读 FREEZE_ID
→ 才允许第一条 S1_CORE formal trial
```

</details>
